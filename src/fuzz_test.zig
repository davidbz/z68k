//! Fuzzing `step()` against a bus that checks the host-facing contract.
//!
//!     zig build fuzz --fuzz     # coverage-guided search; ^C to stop
//!     zig build test            # the PRNG soak below, ~2k runs
//!
//! SingleStepTests covers what the core computes; this covers what it must
//! never do, from states the suite never enters. The interesting failures are
//! the ones no expected-value table can express: a panic, an odd address on a
//! word cycle, a step that costs nothing, a CPU that un-halts itself.
//!
//! `--fuzz` does not build under Zig 0.16.0 — its own `test_runner.zig:566`
//! passes a `builtin.StackTrace` where `debug.writeStackTrace` wants a
//! `debug.StackTrace`. That is why the soak test exists: same body, same
//! invariants, PRNG instead of coverage feedback, and it runs in CI today.

const std = @import("std");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;

/// 64 KiB mirrored across the 24-bit address space. A host is free to decode
/// as little as it likes, and a small space keeps every iteration's memset
/// cheap and every input reproducible.
const ram_bytes = 0x1_0000;

/// Asserts the four-function contract DESIGN.md §3.7 promises hosts: word
/// accesses are word-aligned, so a host can use `readInt` without a bounds or
/// parity check of its own. A core that breaks this corrupts memory in a way
/// no state comparison would attribute to the right instruction.
const CheckedBus = struct {
    ram: []u8,

    fn at(addr: u24) usize {
        return addr & (ram_bytes - 1);
    }

    pub fn read8(b: *CheckedBus, addr: u24) u8 {
        return b.ram[at(addr)];
    }

    pub fn read16(b: *CheckedBus, addr: u24) u16 {
        std.debug.assert(addr & 1 == 0);
        return std.mem.readInt(u16, b.ram[at(addr)..][0..2], .big);
    }

    pub fn write8(b: *CheckedBus, addr: u24, v: u8) void {
        b.ram[at(addr)] = v;
    }

    pub fn write16(b: *CheckedBus, addr: u24, v: u16) void {
        std.debug.assert(addr & 1 == 0);
        std.mem.writeInt(u16, b.ram[at(addr)..][0..2], v, .big);
    }
};

const Core = m68k.Core(CheckedBus);

const code_at = 0x400;
const code_bytes = 128;
const steps_per_run = 200;
const soak_runs = 2000;

test "step never breaks the bus contract, stalls, or resurrects a halted CPU" {
    const ram = try std.testing.allocator.alloc(u8, ram_bytes);
    defer std.testing.allocator.free(ram);
    try std.testing.fuzz(ram, oneRun, .{});
}

// The same invariants driven by a PRNG instead of the fuzzer. `Smith` takes
// its bytes from anywhere, so this is the identical body over pseudo-random
// input: it runs in plain `zig build test` (and therefore in CI) and does not
// need the coverage-guided machinery. Fixed seed, so a failure reproduces.
test "soak: random states through the same invariants" {
    const ram = try std.testing.allocator.alloc(u8, ram_bytes);
    defer std.testing.allocator.free(ram);

    var prng = std.Random.DefaultPrng.init(0x68000);
    var buf: [512]u8 = undefined;
    for (0..soak_runs) |_| {
        prng.random().bytes(&buf);
        var smith = std.testing.Smith{ .in = &buf };
        try oneRun(ram, &smith);
    }
}

fn oneRun(ram: []u8, smith: *std.testing.Smith) anyerror!void {
    var bus = CheckedBus{ .ram = ram };
    @memset(ram, 0);

    // Every vector points back into the fuzzed code, so an exception keeps the
    // run going instead of parking it on a page of zeroes. Vector 0/1 are the
    // reset SP and PC, which `step` never reads.
    for (0..256) |v| std.mem.writeInt(u32, ram[v * 4 ..][0..4], code_at, .big);
    smith.bytes(ram[code_at..][0..code_bytes]);

    var c = Cpu{ .pc = code_at };
    for (&c.d) |*r| r.* = smith.value(u32);
    for (&c.a) |*r| r.* = smith.value(u32);
    c.usp = smith.value(u32);
    // Through `fromInt` rather than field by field: the undefined SR bits are
    // not ours to invent, and this is the shape a host restoring a save state
    // would produce.
    c.sr = m68k.StatusRegister.fromInt(smith.value(u16));

    var halted_seen = false;
    for (0..steps_per_run) |_| {
        const before = c.cycles;
        const was_halted = c.halted;

        Core.step(&c, &bus);

        // Time always moves, so a host's `run(budget)` loop always terminates.
        // The one exception is the step that takes the double bus fault: it
        // abandons the half-written frame and charges nothing, and how many
        // cycles the real chip spent before asserting HALT is not something
        // any recording here can say. Halted is sticky and a halted step
        // costs the idle 4, so at most one free step can ever happen.
        const just_halted = c.halted and !was_halted;
        if (c.cycles <= before and !just_halted) return error.StepCostNothing;
        if (was_halted and !c.halted) return error.HaltedCpuResumed;
        halted_seen = halted_seen or c.halted;

        // The SR is a packed register with fixed undefined bits; nothing may
        // leave a value in it that does not round-trip.
        const sr = c.sr;
        if (!std.meta.eql(m68k.StatusRegister.fromInt(sr.toInt()), sr)) return error.SrNotRoundTrippable;
    }

    // Reset is the only way out of halted, and it must work from any state.
    if (halted_seen) {
        Core.reset(&c, &bus);
        if (c.halted or c.stopped) return error.ResetDidNotRecover;
    }
}
