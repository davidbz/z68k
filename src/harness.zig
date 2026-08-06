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
//! Results are reported in two tiers, architectural state (registers, SR, PC,
//! memory) and total cycles, and every file must pass both: the counts are
//! kept apart only so a regression says which kind it is.

const std = @import("std");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;
const Core = m68k.Core(TrackedBus);

const test_dir = "testdata/v1";
const ram_bytes = 0x100_0000;

/// The recorded PC is MAME's `m_au`, the *next prefetch* address, which sits 4
/// bytes ahead of where execution actually begins. Everything else in the state
/// image is literal.
const pc_prefetch_offset = 4;
const even_bus_addr_mask = 0xFF_FFFE;

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
    d: [8]u32 = @splat(0),
    /// a0..a6 only; a7 is `usp` or `ssp` depending on the S bit in `sr`.
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

    fn int(r: *Reader, comptime T: type) DecodeError!T {
        const n = @divExact(@typeInfo(T).int.bits, 8);
        if (r.pos + n > r.buf.len) return error.Truncated;
        defer r.pos += n;
        return std.mem.readInt(T, r.buf[r.pos..][0..n], .little);
    }

    fn bytes(r: *Reader, n: usize) DecodeError![]const u8 {
        if (r.pos + n > r.buf.len) return error.Truncated;
        defer r.pos += n;
        return r.buf[r.pos..][0..n];
    }

    /// Opens a block: `{ u32 size, u32 magic, payload... }`, where `size`
    /// spans the whole block, header included. Returns the offset one past the
    /// block, to be handed back to `close`.
    fn open(r: *Reader, magic: u32) DecodeError!usize {
        const start = r.pos;
        const size = try r.int(u32);
        if (try r.int(u32) != magic) return error.BadMagic;
        return start + size;
    }

    /// Seeks to the end of a block opened by `open`. Payload we do not care
    /// about therefore costs nothing to walk past, and a payload we do read is
    /// resynchronised here even if the format grows a trailing field.
    fn close(r: *Reader, block_end: usize) DecodeError!void {
        if (block_end < r.pos or block_end > r.buf.len) return error.Truncated;
        r.pos = block_end;
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
        if (try r.int(u32) != magic_file) return error.BadMagic;
        // Read the count before building the result: a struct literal copies
        // `r` as it is when the field is evaluated, not after.
        const count = try r.int(u32);
        return .{ .r = r, .remaining = count };
    }

    fn next(d: *Decoder) DecodeError!?TestCase {
        if (d.remaining == 0) return null;
        d.remaining -= 1;

        const test_end = try d.r.open(magic_test);
        const name = try d.readName();
        const initial = try d.readState(&d.ram_initial);
        const final = try d.readState(&d.ram_final);
        const length = try d.readCycles();
        try d.r.close(test_end);

        return .{ .name = name, .initial = initial, .final = final, .length = length };
    }

    fn readName(d: *Decoder) DecodeError![]const u8 {
        const end = try d.r.open(magic_name);
        const len = try d.r.int(u32);
        const name = try d.r.bytes(len);
        try d.r.close(end);
        return name;
    }

    fn readState(d: *Decoder, ram_buf: []RamWord) DecodeError!State {
        const end = try d.r.open(magic_state);

        var s = State{};
        for (&s.d) |*v| v.* = try d.r.int(u32);
        for (&s.a) |*v| v.* = try d.r.int(u32);
        s.usp = try d.r.int(u32);
        s.ssp = try d.r.int(u32);
        s.sr = try d.r.int(u32);
        s.pc = try d.r.int(u32);
        s.prefetch = .{ try d.r.int(u32), try d.r.int(u32) };

        // RAM is recorded in 16-bit units, matching the real data bus.
        const count = try d.r.int(u32);
        if (count > ram_buf.len) return error.TooManyRamWords;
        for (ram_buf[0..count]) |*w| {
            w.addr = try d.r.int(u32);
            w.value = try d.r.int(u16);
        }
        s.ram = ram_buf[0..count];

        try d.r.close(end);
        return s;
    }

    /// Only the block's total is kept; per-cycle bus activity is outside this
    /// emulator's accuracy target (DESIGN.md §1), so the transaction list is
    /// skipped rather than decoded.
    fn readCycles(d: *Decoder) DecodeError!u32 {
        const end = try d.r.open(magic_transactions);
        const cycles = try d.r.int(u32);
        try d.r.close(end);
        return cycles;
    }
};

// -------------------------------------------------------------------- comparing

/// Test RAM is addressed in 16-bit units on a 24-bit bus.
fn busAddr(addr: u32) u24 {
    return @truncate(addr & even_bus_addr_mask);
}

/// Same flat 16 MiB layout as `m68k.FlatBus`, but also remembers which
/// addresses were written so the driver can undo just those between test
/// cases instead of re-zeroing the whole buffer (each case only ever
/// touches a couple hundred bytes of it).
const TrackedBus = struct {
    ram: []u8,
    dirty: [4096]u24 = undefined,
    dirty_len: usize = 0,

    fn mark(b: *TrackedBus, addr: u24) void {
        if (b.dirty_len < b.dirty.len) {
            b.dirty[b.dirty_len] = addr;
            b.dirty_len += 1;
        }
    }

    pub fn read8(b: *TrackedBus, addr: u24) u8 {
        return b.ram[addr];
    }
    pub fn read16(b: *TrackedBus, addr: u24) u16 {
        return std.mem.readInt(u16, b.ram[addr..][0..2], .big);
    }
    pub fn write8(b: *TrackedBus, addr: u24, v: u8) void {
        b.ram[addr] = v;
        b.mark(addr);
    }
    pub fn write16(b: *TrackedBus, addr: u24, v: u16) void {
        std.mem.writeInt(u16, b.ram[addr..][0..2], v, .big);
        b.mark(addr);
        b.mark(addr +% 1);
    }

    /// Undoes every write since the last reset. If a single test case
    /// somehow dirtied more than `dirty` can track (far beyond the widest
    /// real instruction), falls back to a full clear rather than leaving
    /// stale bytes behind.
    fn reset(b: *TrackedBus) void {
        if (b.dirty_len == b.dirty.len) {
            @memset(b.ram, 0);
        } else {
            for (b.dirty[0..b.dirty_len]) |a| b.ram[a] = 0;
        }
        b.dirty_len = 0;
    }
};

/// State image -> Cpu. `a7` is whichever stack the S bit selects.
fn load(s: *const State, c: *Cpu, bus: *TrackedBus) void {
    c.* = .{};
    c.sr = m68k.StatusRegister.fromInt(@truncate(s.sr));
    c.d = s.d;
    c.a[0..7].* = s.a;
    c.a[7] = if (c.sr.s) s.ssp else s.usp;
    c.usp = if (c.sr.s) s.usp else s.ssp;
    c.pc = s.pc -% pc_prefetch_offset;
    for (s.ram) |w| bus.write16(busAddr(w.addr), w.value);
}

const Mismatch = struct {
    at: At,
    expected: u32,
    actual: u32,

    /// Where the difference is. `d` and `a` carry a register number and `ram`
    /// an address; the rest are one of a kind.
    const At = union(enum) {
        d: u8,
        a: u8,
        ram: u24,
        ssp,
        usp,
        sr,
        pc,
    };

    pub fn format(m: Mismatch, w: *std.Io.Writer) std.Io.Writer.Error!void {
        switch (m.at) {
            .d => |i| try w.print("d{d}", .{i}),
            .a => |i| try w.print("a{d}", .{i}),
            .ram => |addr| try w.print("ram@{x:0>6}", .{addr}),
            inline else => |_, tag| try w.writeAll(@tagName(tag)),
        }
        try w.print(" expected {x:0>8}, got {x:0>8}", .{ m.expected, m.actual });
    }
};

/// Everything is compared exactly, including every word of a group 0
/// exception frame: nothing here is masked or excused.
fn compare(s: *const State, c: *const Cpu, bus: *TrackedBus) ?Mismatch {
    for (s.d, c.d, 0..) |want, got, i| {
        if (got != want) return .{ .at = .{ .d = @intCast(i) }, .expected = want, .actual = got };
    }
    // a7 is covered by the ssp/usp checks below.
    for (s.a, c.a[0..7], 0..) |want, got, i| {
        if (got != want) return .{ .at = .{ .a = @intCast(i) }, .expected = want, .actual = got };
    }
    if (c.ssp() != s.ssp) return .{ .at = .ssp, .expected = s.ssp, .actual = c.ssp() };
    if (c.userSp() != s.usp) return .{ .at = .usp, .expected = s.usp, .actual = c.userSp() };

    const sr = m68k.StatusRegister.fromInt(@truncate(s.sr)).toInt();
    if (c.sr.toInt() != sr) return .{ .at = .sr, .expected = sr, .actual = c.sr.toInt() };

    const pc = s.pc -% pc_prefetch_offset;
    if (c.pc != pc) return .{ .at = .pc, .expected = pc, .actual = c.pc };

    for (s.ram) |w| {
        const addr = busAddr(w.addr);
        const got = bus.read16(addr);
        if (got != w.value) return .{ .at = .{ .ram = addr }, .expected = w.value, .actual = got };
    }
    return null;
}

// ----------------------------------------------------------------------- driver

const Tally = struct {
    total: usize = 0,
    state_ok: usize = 0,
    cycles_ok: usize = 0,

    fn add(t: *Tally, other: Tally) void {
        t.total += other.total;
        t.state_ok += other.state_ok;
        t.cycles_ok += other.cycles_ok;
    }

    fn clean(t: Tally) bool {
        return t.state_ok == t.total and t.cycles_ok == t.total;
    }
};

/// Prints the first state failure and the first cycle miss in a file and then
/// goes quiet: once a file is broken the other 2499 cases almost always repeat
/// the same story, and the tally already says how widespread it is.
const Reporter = struct {
    file: []const u8,
    state_shown: bool = false,
    cycles_shown: bool = false,

    fn stateFailure(rep: *Reporter, tc: TestCase, m: Mismatch) void {
        if (rep.state_shown) return;
        rep.state_shown = true;
        std.debug.print(
            \\  {s}: first failure in "{s}": {f}
            \\      sr {x:0>4}->{x:0>4}  pc {x:0>6}->{x:0>6}  ssp {x:0>6}->{x:0>6}
            \\
        , .{
            rep.file,
            tc.name,
            m,
            @as(u16, @truncate(tc.initial.sr)),
            @as(u16, @truncate(tc.final.sr)),
            tc.initial.pc,
            tc.final.pc,
            tc.initial.ssp,
            tc.final.ssp,
        });
    }

    fn cycleFailure(rep: *Reporter, tc: TestCase, actual: u64) void {
        if (rep.cycles_shown) return;
        rep.cycles_shown = true;
        std.debug.print("  {s}: first cycle miss in \"{s}\": expected {d}, got {d}\n", .{
            rep.file, tc.name, tc.length, actual,
        });
    }
};

fn runFile(src: []const u8, ram: []u8, name: []const u8) !Tally {
    var dec = try Decoder.init(src);
    var tally = Tally{};
    var rep = Reporter{ .file = name };

    // One full clear per file; `bus.reset()` below undoes just what each
    // case actually touched, rather than re-zeroing all 16 MiB every time.
    @memset(ram, 0);
    var bus = TrackedBus{ .ram = ram };

    while (try dec.next()) |tc| {
        bus.reset();
        var c: Cpu = undefined;
        load(&tc.initial, &c, &bus);

        Core.step(&c, &bus);

        tally.total += 1;
        if (compare(&tc.final, &c, &bus)) |m| {
            rep.stateFailure(tc, m);
        } else {
            tally.state_ok += 1;
            if (c.cycles == tc.length) {
                tally.cycles_ok += 1;
            } else {
                rep.cycleFailure(tc, c.cycles);
            }
        }
    }
    return tally;
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

    const ram = try gpa.alloc(u8, ram_bytes);
    defer gpa.free(ram);

    var total = Tally{};
    var files: usize = 0;

    var it = dir.iterate();
    while (try it.next(io)) |entry| {
        if (entry.kind != .file) continue;
        if (!std.mem.endsWith(u8, entry.name, ".json.bin")) continue;
        if (filter) |f| {
            if (std.mem.indexOf(u8, entry.name, f) == null) continue;
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
        std.debug.print("  {s:<24} state {d:>5}/{d:<5} cycles {d:>5}\n", .{
            entry.name, t.state_ok, t.total, t.cycles_ok,
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
        \\{d} files, {d} cases
        \\  state:  {d}/{d}
        \\  cycles: {d}/{d}
        \\
    , .{
        files,           total.total,
        total.state_ok,  total.total,
        total.cycles_ok, total.total,
    });
    if (!total.clean()) return error.ConformanceFailed;
}

// ------------------------------------------------------------------------ tests

test "supervisor state image puts ssp in a7" {
    var ram = [_]u8{0} ** 0x100;
    var bus = TrackedBus{ .ram = &ram };
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

    try std.testing.expectEqual(@as(?Mismatch, null), compare(&s, &c, &bus));

    // A real difference is actually caught, and it names the right register.
    s.d[3] = 0xFFFF;
    try std.testing.expectEqual(Mismatch.At{ .d = 3 }, compare(&s, &c, &bus).?.at);
}

test "user-mode state image puts usp in a7" {
    var ram = [_]u8{0} ** 0x100;
    var bus = TrackedBus{ .ram = &ram };

    const s = State{ .usp = 0x1000, .ssp = 0x8000, .sr = 0x0000, .pc = 0x404 };

    var c: Cpu = undefined;
    load(&s, &c, &bus);

    try std.testing.expectEqual(@as(u32, 0x1000), c.a[7]);
    try std.testing.expectEqual(@as(u32, 0x8000), c.ssp());
    try std.testing.expectEqual(@as(?Mismatch, null), compare(&s, &c, &bus));
}

test "mismatch names the location" {
    var buf: [64]u8 = undefined;
    const m = Mismatch{ .at = .{ .ram = 0x1234 }, .expected = 0xAA, .actual = 0xBB };
    try std.testing.expectEqualStrings(
        "ram@001234 expected 000000aa, got 000000bb",
        try std.fmt.bufPrint(&buf, "{f}", .{m}),
    );
    const sr = Mismatch{ .at = .sr, .expected = 0, .actual = 1 };
    try std.testing.expectEqualStrings(
        "sr expected 00000000, got 00000001",
        try std.fmt.bufPrint(&buf, "{f}", .{sr}),
    );
}

test "decoder rejects a bad header" {
    const junk = [_]u8{0} ** 16;
    try std.testing.expectError(error.BadMagic, Decoder.init(&junk));
    try std.testing.expectError(error.Truncated, Decoder.init(&.{}));
}

test "decoder rejects a block that runs past the file" {
    // File header, then a test block claiming a size the file cannot back.
    var buf: [16]u8 = undefined;
    std.mem.writeInt(u32, buf[0..4], magic_file, .little);
    std.mem.writeInt(u32, buf[4..8], 1, .little); // one test
    std.mem.writeInt(u32, buf[8..12], 1 << 20, .little); // absurd block size
    std.mem.writeInt(u32, buf[12..16], magic_test, .little);

    var dec = try Decoder.init(&buf);
    try std.testing.expectError(error.Truncated, dec.next());
}
