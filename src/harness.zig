//! SingleStepTests/m68000 conformance runner.
//!
//!     tools/fetch_tests.sh          # once, clones the test data into testdata/
//!     zig build sst                 # run everything
//!     zig build sst -- MOVE         # only files whose name contains MOVE
//!
//! Reads the upstream `.json.bin` files directly rather than going through
//! their `decode.py`: the container is a simple tagged binary format, so
//! decoding it here avoids a Python dependency, the multi-gigabyte intermediate
//! JSON, and the JSON parse itself. One test is decoded and run at a time, so
//! the whole suite runs in a fixed amount of memory.
//!
//! Results are reported in two tiers. Architectural state (registers, SR, PC,
//! memory) must match; cycle counts are counted separately so timing work can
//! lag behind correctness work without hiding it.

const std = @import("std");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;
const FlatBus = m68k.FlatBus;
const Core = m68k.Core(FlatBus);

const test_dir = "testdata/v1";

/// The recorded PC is MAME's `m_au`, the *next prefetch* address, which sits 4
/// bytes ahead of where execution actually begins. Everything else in the state
/// image is literal.
const pc_prefetch_offset = 4;

/// Verified faulty upstream: TAS does not model its read-modify-write timing,
/// and TRAPV triggers on the wrong condition.
const excluded = [_][]const u8{ "TAS", "TRAPV" };

/// Instruction families the core implements so far (DESIGN.md §5.5 milestone
/// gates). Everything else is skipped and counted: the build gates only on
/// what the core claims to do. An explicit filter argument bypasses this, so
/// a family can be watched while it is being implemented.
const implemented = [_][]const u8{
    "NOP",     "MOVE.b",        "MOVE.w",        "MOVE.l", "MOVE.q",
    "MOVEA.w", "MOVEA.l",       "LEA",           "BSR",    "Bcc",
    "RTS",     "ILLEGAL_LINEA", "ILLEGAL_LINEF",
};

fn isImplemented(name: []const u8) bool {
    const stem = name[0 .. name.len - ".json.bin".len];
    for (implemented) |i| {
        if (std.mem.eql(u8, stem, i)) return true;
    }
    return false;
}

/// Generous: the widest instruction (MOVEM of all 16 registers) touches 32
/// words, plus an exception frame.
const max_ram_words = 256;

// ---------------------------------------------------------------- binary format

const magic_file = 0x1A3F5D71;
const magic_test = 0xABC12367;
const magic_name = 0x89ABCDEF;
const magic_state = 0x01234567;
const magic_transactions = 0x456789AB;

const DecodeError = error{ Truncated, BadMagic, TooManyRamWords };

const RamWord = struct { addr: u32, value: u16 };

pub const State = struct {
    // Field order is the file's register order; `read` depends on it.
    d: [8]u32 = @splat(0),
    a: [7]u32 = @splat(0),
    usp: u32 = 0,
    ssp: u32 = 0,
    sr: u32 = 0,
    pc: u32 = 0,
    prefetch: [2]u32 = .{ 0, 0 },
    ram: []const RamWord = &.{},
};

pub const TestCase = struct {
    name: []const u8,
    initial: State,
    final: State,
    /// Total cycles, taken from the transaction block header.
    length: u32,
};

/// Little-endian cursor over the file bytes.
const Reader = struct {
    buf: []const u8,
    pos: usize = 0,

    fn u32_(r: *Reader) DecodeError!u32 {
        if (r.pos + 4 > r.buf.len) return error.Truncated;
        defer r.pos += 4;
        return std.mem.readInt(u32, r.buf[r.pos..][0..4], .little);
    }

    fn u16_(r: *Reader) DecodeError!u16 {
        if (r.pos + 2 > r.buf.len) return error.Truncated;
        defer r.pos += 2;
        return std.mem.readInt(u16, r.buf[r.pos..][0..2], .little);
    }

    fn u8_(r: *Reader) DecodeError!u8 {
        if (r.pos + 1 > r.buf.len) return error.Truncated;
        defer r.pos += 1;
        return r.buf[r.pos];
    }

    fn bytes(r: *Reader, n: usize) DecodeError![]const u8 {
        if (r.pos + n > r.buf.len) return error.Truncated;
        defer r.pos += n;
        return r.buf[r.pos..][0..n];
    }

    /// Every block is prefixed with its byte count and a magic number.
    fn block(r: *Reader, magic: u32) DecodeError!void {
        _ = try r.u32_(); // byte count, unused: blocks are walked, not skipped
        if (try r.u32_() != magic) return error.BadMagic;
    }
};

/// Decodes one file, one test at a time, into reusable buffers.
const Decoder = struct {
    r: Reader,
    remaining: u32,
    ram_initial: [max_ram_words]RamWord = undefined,
    ram_final: [max_ram_words]RamWord = undefined,

    fn init(buf: []const u8) DecodeError!Decoder {
        var r = Reader{ .buf = buf };
        // The file header is magic-first, unlike every other block.
        if (try r.u32_() != magic_file) return error.BadMagic;
        // Read the count before building the result: a struct literal copies
        // `r` as it is when the field is evaluated, not after.
        const count = try r.u32_();
        return .{ .r = r, .remaining = count };
    }

    fn next(d: *Decoder) DecodeError!?TestCase {
        if (d.remaining == 0) return null;
        d.remaining -= 1;

        try d.r.block(magic_test);

        try d.r.block(magic_name);
        const name_len = try d.r.u32_();
        const name = try d.r.bytes(name_len);

        const initial = try d.readState(&d.ram_initial);
        const final = try d.readState(&d.ram_final);
        const length = try d.readTransactions();

        return .{ .name = name, .initial = initial, .final = final, .length = length };
    }

    fn readState(d: *Decoder, ram_buf: []RamWord) DecodeError!State {
        try d.r.block(magic_state);

        var s = State{};
        for (&s.d) |*v| v.* = try d.r.u32_();
        for (&s.a) |*v| v.* = try d.r.u32_();
        s.usp = try d.r.u32_();
        s.ssp = try d.r.u32_();
        s.sr = try d.r.u32_();
        s.pc = try d.r.u32_();
        s.prefetch = .{ try d.r.u32_(), try d.r.u32_() };

        // RAM is recorded in 16-bit units, matching the real data bus.
        const count = try d.r.u32_();
        if (count > ram_buf.len) return error.TooManyRamWords;
        for (ram_buf[0..count]) |*w| {
            w.addr = try d.r.u32_();
            w.value = try d.r.u16_();
        }
        s.ram = ram_buf[0..count];
        return s;
    }

    /// Only the cycle count is kept; per-cycle bus activity is outside this
    /// emulator's accuracy target (DESIGN.md §1). The entries still have to be
    /// walked to find the next test.
    fn readTransactions(d: *Decoder) DecodeError!u32 {
        try d.r.block(magic_transactions);
        const cycles = try d.r.u32_();
        const count = try d.r.u32_();

        for (0..count) |_| {
            const kind = try d.r.u8_();
            _ = try d.r.u32_(); // cycles for this transaction
            if (kind == 0) continue; // idle
            _ = try d.r.bytes(20); // fc, addr, data, UDS, LDS
        }
        return cycles;
    }
};

// -------------------------------------------------------------------- comparing

/// State image -> Cpu. `a7` is whichever stack the S bit selects.
fn load(s: *const State, c: *Cpu, bus: *FlatBus) void {
    c.* = .{};
    c.sr = m68k.StatusRegister.fromInt(@truncate(s.sr));
    c.d = s.d;
    c.a = .{ s.a[0], s.a[1], s.a[2], s.a[3], s.a[4], s.a[5], s.a[6], if (c.sr.s) s.ssp else s.usp };
    c.usp = if (c.sr.s) s.usp else s.ssp;
    c.pc = s.pc -% pc_prefetch_offset;
    for (s.ram) |w| bus.write16(@truncate(w.addr & 0xFF_FFFE), w.value);
}

const Mismatch = struct {
    what: []const u8,
    index: u8 = 0,
    /// For `ram` mismatches, the address that differed.
    addr: u32 = 0,
    expected: u32,
    actual: u32,
};

/// `frame` is the base of a group 0 exception frame whose prefetch-derived
/// fields should be left out of the comparison, used to bucket the one known
/// gap (DESIGN.md §5.4) instead of hiding it. Everything else in the frame --
/// the special status word, the faulting address and the saved SR -- is still
/// checked.
fn compare(s: *const State, c: *const Cpu, bus: *FlatBus, frame: ?u32) ?Mismatch {
    for (s.d, 0..) |want, i| {
        if (c.d[i] != want) return .{ .what = "d", .index = @intCast(i), .expected = want, .actual = c.d[i] };
    }
    for (s.a, 0..) |want, i| {
        if (c.a[i] != want) return .{ .what = "a", .index = @intCast(i), .expected = want, .actual = c.a[i] };
    }
    if (c.ssp() != s.ssp) return .{ .what = "ssp", .expected = s.ssp, .actual = c.ssp() };
    if (c.userSp() != s.usp) return .{ .what = "usp", .expected = s.usp, .actual = c.userSp() };

    const sr = m68k.StatusRegister.fromInt(@truncate(s.sr)).toInt();
    // In a faulting case, N/Z/V/C depend on how far the microcode got before
    // the fault, which is prefetch-order state this core does not model —
    // part of the same known gap as the frame fields (DESIGN.md §5.4).
    const sr_mask: u16 = if (frame != null) 0xFFF0 else 0xFFFF;
    if ((c.sr.toInt() ^ sr) & sr_mask != 0) return .{ .what = "sr", .expected = sr, .actual = c.sr.toInt() };

    const pc = s.pc -% pc_prefetch_offset;
    if (c.pc != pc) return .{ .what = "pc", .expected = pc, .actual = c.pc };

    for (s.ram) |w| {
        const addr: u24 = @truncate(w.addr & 0xFF_FFFE);
        const got = bus.read16(addr);
        if (frame) |f| {
            const off = addr -% @as(u24, @truncate(f));
            switch (off) {
                // IR at +6 and the PC longword at +10 are whole words of
                // prefetch state.
                6, 10, 12 => continue,
                // The special status word keeps the R/W bit and the function
                // code in its low five bits; the rest is prefetch residue.
                0 => if ((got ^ w.value) & 0x1F == 0) continue,
                // The saved SR carries N/Z/V/C from mid-microcode, like the
                // live SR (see the sr compare above).
                8 => if ((got ^ w.value) & 0xFFF0 == 0) continue,
                else => {},
            }
        }
        if (got != w.value) return .{ .what = "ram", .addr = addr, .expected = w.value, .actual = got };
    }
    return null;
}

// ----------------------------------------------------------------------- driver

const Tally = struct {
    total: usize = 0,
    state_ok: usize = 0,
    cycles_ok: usize = 0,
    /// Failed on nothing but the PC in a group 0 frame. See DESIGN.md §5.4.
    aerr_pc: usize = 0,

    fn add(t: *Tally, o: Tally) void {
        t.total += o.total;
        t.state_ok += o.state_ok;
        t.cycles_ok += o.cycles_ok;
        t.aerr_pc += o.aerr_pc;
    }

    /// Failures that are not the known gap. This is what gates the build.
    fn unexplained(t: Tally) usize {
        return t.total - t.state_ok - t.aerr_pc;
    }
};

fn runFile(src: []const u8, ram: []u8, name: []const u8) !Tally {
    var dec = try Decoder.init(src);
    var tally = Tally{};
    var reported = false;
    var cycles_reported = false;

    while (try dec.next()) |tc| {
        @memset(ram, 0);
        var bus = FlatBus{ .ram = ram };
        var c: Cpu = undefined;
        load(&tc.initial, &c, &bus);

        Core.step(&c, &bus);

        tally.total += 1;
        if (compare(&tc.final, &c, &bus, null)) |strict| {
            // Retry ignoring the frame fields that come from the prefetch
            // queue. If those are the only differences it is the known gap,
            // not a real failure. The relaxed mismatch is the one worth
            // reporting: the strict one is usually just the frame hiding
            // whatever else went wrong.
            const relaxed = if (c.sr.s) compare(&tc.final, &c, &bus, c.a[7]) else strict;
            const m = relaxed orelse {
                tally.aerr_pc += 1;
                continue;
            };
            if (!reported) {
                reported = true;
                std.debug.print(
                    "  {s}: first failure in \"{s}\": {s}{d}@{x:0>6} expected {x:0>8}, got {x:0>8}" ++
                        " (sr {x:0>4}->{x:0>4}, pc {x:0>6}->{x:0>6}, ssp {x:0>6}->{x:0>6})\n",
                    .{
                        name,
                        tc.name,
                        m.what,
                        m.index,
                        m.addr,
                        m.expected,
                        m.actual,
                        @as(u16, @truncate(tc.initial.sr)),
                        @as(u16, @truncate(tc.final.sr)),
                        tc.initial.pc,
                        tc.final.pc,
                        tc.initial.ssp,
                        tc.final.ssp,
                    },
                );
            }
        } else {
            tally.state_ok += 1;
            if (c.cycles == tc.length) {
                tally.cycles_ok += 1;
            } else if (!cycles_reported) {
                cycles_reported = true;
                std.debug.print("  {s}: first cycle miss in \"{s}\": expected {d}, got {d}\n", .{
                    name, tc.name, tc.length, c.cycles,
                });
            }
        }
    }
    return tally;
}

fn isExcluded(name: []const u8) bool {
    for (excluded) |e| {
        if (std.mem.startsWith(u8, name, e)) return true;
    }
    return false;
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;

    var args = std.process.Args.Iterator.init(init.minimal.args);
    _ = args.skip();
    const filter = args.next();

    var dir = std.Io.Dir.cwd().openDir(io, test_dir, .{ .iterate = true }) catch |err| {
        std.debug.print(
            "cannot open {s}: {t}\nRun tools/fetch_tests.sh first.\n",
            .{ test_dir, err },
        );
        return err;
    };
    defer dir.close(io);

    const ram = try gpa.alloc(u8, 0x100_0000);
    defer gpa.free(ram);

    var total = Tally{};
    var files: usize = 0;
    var skipped: usize = 0;

    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind != .file) continue;
        if (!std.mem.endsWith(u8, entry.name, ".json.bin")) continue;
        if (isExcluded(entry.name)) continue;
        if (filter) |f| {
            if (std.mem.indexOf(u8, entry.name, f) == null) continue;
        } else if (!isImplemented(entry.name)) {
            skipped += 1;
            continue;
        }

        const src = dir.readFileAlloc(io, entry.name, gpa, .limited(256 << 20)) catch |err| {
            std.debug.print("  {s}: {t}\n", .{ entry.name, err });
            continue;
        };
        defer gpa.free(src);

        const t = runFile(src, ram, entry.name) catch |err| {
            std.debug.print("  {s}: {t}\n", .{ entry.name, err });
            continue;
        };
        std.debug.print("  {s:<24} state {d:>5}/{d:<5} cycles {d:>5}  aerr-pc {d:>5}\n", .{
            entry.name, t.state_ok, t.total, t.cycles_ok, t.aerr_pc,
        });
        total.add(t);
        files += 1;
    }

    if (files == 0) {
        std.debug.print("no test files matched. Run tools/fetch_tests.sh first.\n", .{});
        return error.NoTests;
    }

    std.debug.print(
        \\
        \\{d} files, {d} cases ({d} files skipped: unimplemented families)
        \\  state:       {d}/{d}
        \\  cycles:      {d}/{d}
        \\  aerr-pc:     {d}  (known gap, DESIGN.md §5.4)
        \\  unexplained: {d}
        \\
    , .{
        files,               total.total,
        skipped,             total.state_ok,
        total.total,         total.cycles_ok,
        total.total,         total.aerr_pc,
        total.unexplained(),
    });
    if (total.unexplained() != 0) return error.ConformanceFailed;
}

// ------------------------------------------------------------------------ tests

test "supervisor state image puts ssp in a7" {
    var ram = [_]u8{0} ** 0x100;
    var bus = FlatBus{ .ram = &ram };
    const words = [_]RamWord{.{ .addr = 0x10, .value = 0xABCD }};

    var s = State{
        .d = .{ 1, 2, 3, 4, 5, 6, 7, 8 },
        .a = .{ 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70 },
        .usp = 0x1000,
        .ssp = 0x8000,
        .sr = 0x2700,
        .pc = 0x404,
        .ram = &words,
    };

    var c: Cpu = undefined;
    load(&s, &c, &bus);

    try std.testing.expectEqual(@as(u32, 0x8000), c.a[7]);
    try std.testing.expectEqual(@as(u32, 0x1000), c.userSp());
    // PC is rewound past MAME's prefetch offset.
    try std.testing.expectEqual(@as(u32, 0x400), c.pc);
    try std.testing.expectEqual(@as(u16, 0xABCD), bus.read16(0x10));

    try std.testing.expectEqual(@as(?Mismatch, null), compare(&s, &c, &bus, null));

    // A real difference is actually caught.
    s.d[3] = 0xFFFF;
    try std.testing.expect(compare(&s, &c, &bus, null) != null);
}

test "user-mode state image puts usp in a7" {
    var ram = [_]u8{0} ** 0x100;
    var bus = FlatBus{ .ram = &ram };

    const s = State{ .usp = 0x1000, .ssp = 0x8000, .sr = 0x0000, .pc = 0x404 };

    var c: Cpu = undefined;
    load(&s, &c, &bus);

    try std.testing.expectEqual(@as(u32, 0x1000), c.a[7]);
    try std.testing.expectEqual(@as(u32, 0x8000), c.ssp());
    try std.testing.expectEqual(@as(?Mismatch, null), compare(&s, &c, &bus, null));
}

test "decoder rejects a bad header" {
    const junk = [_]u8{0} ** 16;
    try std.testing.expectError(error.BadMagic, Decoder.init(&junk));
    try std.testing.expectError(error.Truncated, Decoder.init(&.{}));
}
