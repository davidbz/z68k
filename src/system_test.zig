//! Sequence-level tests: the parts of the core SingleStepTests cannot reach.
//!
//! The conformance suite runs exactly one instruction, from a clean state,
//! with nothing pending, and compares the end result (DESIGN.md §5.2). That
//! leaves `run`, `reset`, the interrupt path, STOP, trace chains, the halted
//! state and the USP/SSP discipline verified only by whatever is written here.

const std = @import("std");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;
const Core = m68k.Core(m68k.FlatBus);

const ram_bytes = 0x1_0000;
const nop = 0x4E71;
const rte = 0x4E73;
const stop = 0x4E72;
const illegal = 0x4AFC;
const trap0 = 0x4E40;

/// A CPU with its memory, wired the way a host would wire it. `pc`/`a[7]`
/// start where the little programs below live rather than at the reset
/// vector, so a test only sets up what it is actually about.
const Sys = struct {
    bus: m68k.FlatBus,
    cpu: Cpu = .{ .pc = code },

    const code = 0x400; // where test programs are assembled
    const sp = 0x1800; // supervisor stack, well clear of the vectors

    fn init(alloc: std.mem.Allocator) !Sys {
        const ram = try alloc.alloc(u8, ram_bytes);
        @memset(ram, 0);
        var s = Sys{ .bus = .{ .ram = ram } };
        s.cpu.a[7] = sp;
        return s;
    }

    fn deinit(s: *Sys, alloc: std.mem.Allocator) void {
        alloc.free(s.bus.ram);
    }

    /// Assembles words at `code`, in order.
    fn program(s: *Sys, words: []const u16) void {
        for (words, 0..) |w, i| s.bus.write16(@intCast(code + i * 2), w);
    }

    fn vector(s: *Sys, e: m68k.Exception, target: u32) void {
        const at: u24 = @intCast(e.vectorAddr());
        s.bus.write16(at, @truncate(target >> 16));
        s.bus.write16(at + 2, @truncate(target));
    }

    fn step(s: *Sys) void {
        Core.step(&s.cpu, &s.bus);
    }
};

test "an interrupt is taken exactly when its level beats the mask" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    s.program(&.{nop});
    // A distinct handler per level, so a wrong vector is not a passing test.
    for (1..8) |level| s.vector(m68k.Exception.autovector(@intCast(level)), 0x1000 + @as(u32, @intCast(level)) * 0x100);

    for (0..8) |pending| {
        for (0..8) |mask| {
            s.cpu = .{ .pc = Sys.code, .sr = .{ .s = true, .ipl = @intCast(mask) } };
            s.cpu.a[7] = Sys.sp;
            Core.setIpl(&s.cpu, @intCast(pending));
            s.step();

            // Level 7 is edge-triggered on real hardware and cannot be
            // masked; every other level has to be strictly above the mask.
            const taken = pending != 0 and (pending == 7 or pending > mask);
            if (taken) {
                try std.testing.expectEqual(0x1000 + @as(u32, @intCast(pending)) * 0x100, s.cpu.pc);
                try std.testing.expectEqual(@as(u3, @intCast(pending)), s.cpu.sr.ipl);
                try std.testing.expectEqual(Sys.sp - 6, s.cpu.a[7]);
            } else {
                // The NOP ran instead, and the mask is untouched.
                try std.testing.expectEqual(Sys.code + 2, s.cpu.pc);
                try std.testing.expectEqual(@as(u3, @intCast(mask)), s.cpu.sr.ipl);
                try std.testing.expectEqual(Sys.sp, s.cpu.a[7]);
            }
        }
    }
}

test "an interrupt from user mode switches stacks and RTE switches back" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    const usp = 0x2000;
    const handler = 0x1200;
    s.vector(m68k.Exception.autovector(4), handler);
    s.bus.write16(handler, rte);

    s.cpu = .{ .pc = Sys.code, .sr = .{ .s = false, .ipl = 0 } };
    s.cpu.a[7] = usp;
    s.cpu.usp = Sys.sp; // inactive half holds the supervisor stack in user mode

    Core.setIpl(&s.cpu, 4);
    s.step();

    try std.testing.expect(s.cpu.sr.s);
    try std.testing.expectEqual(Sys.sp - 6, s.cpu.a[7]); // frame is on the supervisor stack
    try std.testing.expectEqual(usp, s.cpu.userSp()); // user stack untouched
    try std.testing.expectEqual(handler, s.cpu.pc);

    Core.setIpl(&s.cpu, 0);
    s.step(); // RTE

    try std.testing.expect(!s.cpu.sr.s);
    try std.testing.expectEqual(usp, s.cpu.a[7]);
    try std.testing.expectEqual(Sys.sp, s.cpu.ssp());
    try std.testing.expectEqual(Sys.code, s.cpu.pc);
    try std.testing.expectEqual(@as(u3, 0), s.cpu.sr.ipl); // restored from the frame
}

test "STOP burns cycles until an unmasked interrupt arrives" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    // stop #$2300 — supervisor, mask 3.
    s.program(&.{ stop, 0x2300, nop });
    s.vector(m68k.Exception.autovector(2), 0x1400);
    s.vector(m68k.Exception.autovector(5), 0x1500);
    s.cpu.sr.s = true;

    s.step();
    try std.testing.expect(s.cpu.stopped);
    try std.testing.expectEqual(@as(u3, 3), s.cpu.sr.ipl);

    // A stopped CPU still consumes the budget, so `run` returns rather than
    // spinning - the host stays in control of the timeslice.
    const before = s.cpu.cycles;
    const ran = Core.run(&s.cpu, &s.bus, 100);
    try std.testing.expect(ran >= 100);
    try std.testing.expect(s.cpu.cycles > before);
    try std.testing.expect(s.cpu.stopped);

    // Masked: no wake.
    Core.setIpl(&s.cpu, 2);
    s.step();
    try std.testing.expect(s.cpu.stopped);

    Core.setIpl(&s.cpu, 5);
    s.step();
    try std.testing.expect(!s.cpu.stopped);
    try std.testing.expectEqual(@as(u32, 0x1500), s.cpu.pc);
}

test "trace chains: every instruction traps, and clearing T inside the handler stops it" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    const handler = 0x1300;
    s.vector(.trace, handler);
    s.bus.write16(handler, rte);
    s.program(&.{ nop, nop });
    s.cpu.sr = .{ .s = true, .t = true };

    // nop, trace trap, rte -> back to the second nop with T restored.
    s.step();
    try std.testing.expect(s.cpu.trace_pending);
    s.step();
    try std.testing.expectEqual(handler, s.cpu.pc);
    try std.testing.expect(!s.cpu.sr.t); // T is cleared on entry, so the handler is not traced
    s.step(); // rte
    try std.testing.expectEqual(Sys.code + 2, s.cpu.pc);
    try std.testing.expect(s.cpu.sr.t);

    // Second instruction traces the same way: the chain does not run dry.
    s.step();
    s.step();
    try std.testing.expectEqual(handler, s.cpu.pc);

    // Clear T in the saved SR on the stack; RTE then returns to an untraced
    // instruction and the chain ends.
    const saved_sr = s.bus.read16(@truncate(s.cpu.a[7]));
    s.bus.write16(@truncate(s.cpu.a[7]), saved_sr & ~@as(u16, 0x8000));
    s.step(); // rte
    try std.testing.expect(!s.cpu.sr.t);
    s.step();
    try std.testing.expect(!s.cpu.trace_pending);
}

test "a fault while stacking a fault frame halts the CPU until reset" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    s.program(&.{illegal});
    s.vector(.illegal_instruction, 0x1600);
    s.cpu.sr.s = true;
    s.cpu.a[7] = Sys.sp + 1; // odd: the frame write cannot succeed

    s.step();
    try std.testing.expect(s.cpu.halted);

    // Halted is sticky: no instruction runs, no interrupt is taken, only time
    // passes - which is what keeps `run` from spinning.
    const pc = s.cpu.pc;
    Core.setIpl(&s.cpu, 7);
    const ran = Core.run(&s.cpu, &s.bus, 64);
    try std.testing.expect(ran >= 64);
    try std.testing.expect(s.cpu.halted);
    try std.testing.expectEqual(pc, s.cpu.pc);

    // Only RESET clears it.
    s.bus.write16(2, 0x0800);
    s.bus.write16(6, 0x0400);
    Core.reset(&s.cpu, &s.bus);
    try std.testing.expect(!s.cpu.halted);
    try std.testing.expectEqual(@as(u32, 0x0800), s.cpu.a[7]);
    try std.testing.expectEqual(@as(u32, 0x0400), s.cpu.pc);
    try std.testing.expect(s.cpu.sr.s and s.cpu.sr.ipl == 7 and !s.cpu.sr.t);
    try std.testing.expectEqual(@as(u3, 0), s.cpu.pending_ipl);
}

test "reset from a stopped CPU also clears STOP" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    s.cpu.stopped = true;
    s.bus.write16(2, 0x0800);
    s.bus.write16(6, 0x0400);

    const before = s.cpu.cycles;
    Core.reset(&s.cpu, &s.bus);
    try std.testing.expect(!s.cpu.stopped);
    try std.testing.expect(s.cpu.cycles > before); // reset is not free
}

test "run overshoots but never undershoots, exceptions included" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    // A trap handler that returns immediately, so the program keeps running
    // across the exception rather than parking in the handler.
    const handler = 0x1700;
    s.vector(.trap_0, handler);
    s.bus.write16(handler, rte);
    s.program(&.{ nop, trap0, nop, nop, nop, nop, nop, nop });
    s.cpu.sr.s = true;

    const ran = Core.run(&s.cpu, &s.bus, 50);
    try std.testing.expect(ran >= 50);
    try std.testing.expectEqual(ran, s.cpu.cycles);
    // The trap was taken and returned from: execution is back in the main
    // program, past the TRAP, not parked in the handler.
    try std.testing.expect(s.cpu.pc >= Sys.code + 4);
    try std.testing.expect(!s.cpu.halted);

    // A zero budget still runs nothing at all.
    const c = s.cpu;
    try std.testing.expectEqual(@as(u64, 0), Core.run(&s.cpu, &s.bus, 0));
    try std.testing.expectEqual(c.pc, s.cpu.pc);
}

test "exactly one of a[7] and usp is the active stack pointer across a privilege change" {
    var s = try Sys.init(std.testing.allocator);
    defer s.deinit(std.testing.allocator);

    const usp = 0x2400;
    const handler = 0x1800;
    s.vector(.trap_0, handler);
    s.bus.write16(handler, rte);
    s.program(&.{trap0});

    s.cpu = .{ .pc = Sys.code, .sr = .{ .s = false } };
    s.cpu.a[7] = usp;
    s.cpu.usp = Sys.sp;

    // User mode: a[7] is the user stack, ssp() reads the inactive half.
    try std.testing.expectEqual(usp, s.cpu.userSp());
    try std.testing.expectEqual(Sys.sp, s.cpu.ssp());

    s.step(); // trap -> supervisor
    try std.testing.expectEqual(usp, s.cpu.userSp());
    try std.testing.expectEqual(s.cpu.a[7], s.cpu.ssp());

    s.step(); // rte -> user
    try std.testing.expectEqual(s.cpu.a[7], s.cpu.userSp());
    try std.testing.expectEqual(Sys.sp, s.cpu.ssp());
    try std.testing.expectEqual(usp, s.cpu.a[7]);
}
