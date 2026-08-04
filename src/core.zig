//! Execution. `Core` is generic over the bus type so that memory access is
//! inlined rather than dispatched through a vtable: the flat-RAM test bus and
//! the future Genesis mapper each get their own specialisation for free.
//!
//! A bus type must provide:
//!     read8(*B, u24) u8      read16(*B, u24) u16
//!     write8(*B, u24, u8)    write16(*B, u24, u16)
//!
//! Alignment is the core's problem, not the bus's: a word access to an odd
//! address raises an address error before the bus ever sees it. 32-bit accesses
//! are composed from two 16-bit ones, matching the real 16-bit data bus.

const std = @import("std");
const cpu_mod = @import("cpu.zig");
const decode = @import("decode.zig");
const flags = @import("flags.zig");

const Cpu = cpu_mod.Cpu;
const Size = cpu_mod.Size;
const Exception = cpu_mod.Exception;
const Group0Info = cpu_mod.Group0Info;
const EaMode = decode.EaMode;

/// Aborts the instruction in progress. `step` turns these into exceptions.
/// Using an error union here rather than a status code keeps every handler's
/// happy path free of unwinding boilerplate.
// TODO(M2): Privilege, ZeroDivide and Trap faults, with the instructions that
// raise them.
pub const Fault = error{
    AddressError,
    IllegalInstruction,
    LineA,
    LineF,
};

pub fn Core(comptime BusT: type) type {
    return struct {
        /// Transient per-instruction state. Deliberately *not* part of `Cpu`,
        /// which stays a clean snapshot of architectural state.
        pub const Ctx = struct {
            cpu: *Cpu,
            bus: *BusT,
            /// Set alongside `Fault.AddressError` to build the group 0 frame.
            fault: ?Group0Info = null,
            /// Opcode currently executing. Only used to fill the group 0
            /// frame's instruction register word.
            ir: u16 = 0,
        };

        // ---------------------------------------------------------------- bus

        fn read8(ctx: *Ctx, addr: u32) u8 {
            return ctx.bus.read8(@truncate(addr));
        }

        fn write8(ctx: *Ctx, addr: u32, v: u8) void {
            ctx.bus.write8(@truncate(addr), v);
        }

        fn read16(ctx: *Ctx, addr: u32, program: bool) Fault!u16 {
            if (addr & 1 != 0) return addressError(ctx, addr, true, program);
            return ctx.bus.read16(@truncate(addr));
        }

        fn write16(ctx: *Ctx, addr: u32, v: u16) Fault!void {
            if (addr & 1 != 0) return addressError(ctx, addr, false, false);
            ctx.bus.write16(@truncate(addr), v);
        }

        fn read32(ctx: *Ctx, addr: u32, program: bool) Fault!u32 {
            const hi = try read16(ctx, addr, program);
            const lo = try read16(ctx, addr +% 2, program);
            return (@as(u32, hi) << 16) | lo;
        }

        fn write32(ctx: *Ctx, addr: u32, v: u32) Fault!void {
            // TODO(M2): -(An) destinations write the low word first on real
            // hardware. Only observable through a bus-error frame, which we do
            // not model yet.
            try write16(ctx, addr, @truncate(v >> 16));
            try write16(ctx, addr +% 2, @truncate(v));
        }

        fn addressError(ctx: *Ctx, addr: u32, read: bool, program: bool) Fault {
            ctx.fault = .{ .access_addr = addr, .ir = ctx.ir, .read = read, .program = program };
            return Fault.AddressError;
        }

        fn fetch16(ctx: *Ctx) Fault!u16 {
            const w = try read16(ctx, ctx.cpu.pc, true);
            ctx.cpu.pc +%= 2;
            return w;
        }

        fn fetch32(ctx: *Ctx) Fault!u32 {
            const v = try read32(ctx, ctx.cpu.pc, true);
            ctx.cpu.pc +%= 4;
            return v;
        }

        fn push16(ctx: *Ctx, v: u16) Fault!void {
            ctx.cpu.a[7] -%= 2;
            try write16(ctx, ctx.cpu.a[7], v);
        }

        fn push32(ctx: *Ctx, v: u32) Fault!void {
            ctx.cpu.a[7] -%= 4;
            try write32(ctx, ctx.cpu.a[7], v);
        }

        fn pop32(ctx: *Ctx) Fault!u32 {
            const v = try read32(ctx, ctx.cpu.a[7], false);
            ctx.cpu.a[7] +%= 4;
            return v;
        }

        // --------------------------------------------------- effective address

        /// Byte access through A7 still moves the stack by 2 so it stays word
        /// aligned. The single most commonly missed 68000 quirk.
        fn stackAdjust(reg: u3, size: Size) u32 {
            if (reg == 7 and size == .byte) return 2;
            return size.bytes();
        }

        /// Computes the address for a memory mode, applying `(An)+` / `-(An)`
        /// side effects. Must be called exactly once per operand: a
        /// read-modify-write instruction computes the address, then reuses it.
        fn calcEa(ctx: *Ctx, mode: EaMode, reg: u3, size: Size, write: bool) Fault!u32 {
            const c = ctx.cpu;
            return switch (mode) {
                .addr_ind => c.a[reg],
                .addr_postinc => blk: {
                    const addr = c.a[reg];
                    // Reads increment before the bus cycle, writes after, so a
                    // misaligned write faults with the register still holding
                    // its original value.
                    if (write and size != .byte and addr & 1 != 0) {
                        break :blk addressError(ctx, addr, false, false);
                    }
                    c.a[reg] +%= stackAdjust(reg, size);
                    break :blk addr;
                },
                .addr_predec => blk: {
                    c.a[reg] -%= stackAdjust(reg, size);
                    break :blk c.a[reg];
                },
                .addr_disp => blk: {
                    const d: i16 = @bitCast(try fetch16(ctx));
                    break :blk c.a[reg] +% @as(u32, @bitCast(@as(i32, d)));
                },
                .addr_index => try indexed(ctx, c.a[reg]),
                .abs_word => blk: {
                    const w: i16 = @bitCast(try fetch16(ctx));
                    break :blk @bitCast(@as(i32, w)); // sign extended
                },
                .abs_long => try fetch32(ctx),
                .pc_disp => blk: {
                    const base = c.pc;
                    const d: i16 = @bitCast(try fetch16(ctx));
                    break :blk base +% @as(u32, @bitCast(@as(i32, d)));
                },
                .pc_index => blk: {
                    const base = c.pc;
                    break :blk try indexed(ctx, base);
                },
                // Register direct and immediate have no address.
                .data_reg, .addr_reg, .immediate => unreachable,
            };
        }

        /// Brief extension word: d8 + index register, word-sized indexes sign
        /// extended to 32 bits.
        fn indexed(ctx: *Ctx, base: u32) Fault!u32 {
            const ext = try fetch16(ctx);
            const reg: u3 = @truncate(ext >> 12);
            const is_addr = ext & 0x8000 != 0;
            const raw = if (is_addr) ctx.cpu.a[reg] else ctx.cpu.d[reg];
            const index: u32 = if (ext & 0x0800 != 0)
                raw
            else
                @bitCast(@as(i32, @as(i16, @bitCast(@as(u16, @truncate(raw))))));
            const disp: i8 = @bitCast(@as(u8, @truncate(ext)));
            return base +% index +% @as(u32, @bitCast(@as(i32, disp)));
        }

        fn readEa(ctx: *Ctx, mode: EaMode, reg: u3, size: Size) Fault!u32 {
            const c = ctx.cpu;
            return switch (mode) {
                .data_reg => c.d[reg] & size.mask(),
                .addr_reg => c.a[reg] & size.mask(),
                .immediate => switch (size) {
                    .byte => @as(u8, @truncate(try fetch16(ctx))),
                    .word => try fetch16(ctx),
                    .long => try fetch32(ctx),
                },
                else => try readAt(ctx, try calcEa(ctx, mode, reg, size, false), size, mode.isProgram()),
            };
        }

        fn readAt(ctx: *Ctx, addr: u32, size: Size, program: bool) Fault!u32 {
            return switch (size) {
                .byte => read8(ctx, addr),
                .word => try read16(ctx, addr, program),
                .long => try read32(ctx, addr, program),
            };
        }

        fn writeAt(ctx: *Ctx, addr: u32, size: Size, v: u32) Fault!void {
            switch (size) {
                .byte => write8(ctx, addr, @truncate(v)),
                .word => try write16(ctx, addr, @truncate(v)),
                .long => try write32(ctx, addr, v),
            }
        }

        fn writeEa(ctx: *Ctx, mode: EaMode, reg: u3, size: Size, v: u32) Fault!void {
            const c = ctx.cpu;
            switch (mode) {
                .data_reg => setReg(&c.d[reg], size, v),
                // Address registers are never partially written: word values
                // are sign extended across all 32 bits.
                .addr_reg => c.a[reg] = signExtend(v, size),
                .immediate => unreachable,
                else => try writeAt(ctx, try calcEa(ctx, mode, reg, size, true), size, v),
            }
        }

        fn setReg(dst: *u32, size: Size, v: u32) void {
            const m = size.mask();
            dst.* = (dst.* & ~m) | (v & m);
        }

        fn signExtend(v: u32, size: Size) u32 {
            return switch (size) {
                .byte => @bitCast(@as(i32, @as(i8, @bitCast(@as(u8, @truncate(v)))))),
                .word => @bitCast(@as(i32, @as(i16, @bitCast(@as(u16, @truncate(v)))))),
                .long => v,
            };
        }

        // ----------------------------------------------------------- exceptions

        /// Cycle cost of taking an exception (M68000UM exception timing).
        /// TODO(M2): zero divide (38) and CHK (40) get their own costs.
        fn exceptionCycles(e: Exception) u8 {
            return if (e.isGroup0()) 50 else 34;
        }

        pub fn enterException(c: *Cpu, bus: *BusT, e: Exception, g0: ?Group0Info) void {
            var ctx = Ctx{ .cpu = c, .bus = bus };

            const old_sr = c.sr;
            c.setSupervisor(true);
            c.sr.t = false;
            c.stopped = false;

            // A fault while building a fault frame is the double bus fault: the
            // real chip asserts HALT and stops until reset.
            frame(&ctx, old_sr, e, g0) catch {
                c.halted = true;
                return;
            };

            c.pc = read32(&ctx, e.vectorAddr(), true) catch {
                c.halted = true;
                return;
            };
            c.cycles += exceptionCycles(e);
        }

        fn frame(ctx: *Ctx, old_sr: cpu_mod.StatusRegister, e: Exception, g0: ?Group0Info) Fault!void {
            if (e.isGroup0()) {
                const info = g0 orelse Group0Info{
                    .access_addr = 0,
                    .ir = 0,
                    .read = true,
                    .program = true,
                };
                try push32(ctx, ctx.cpu.pc);
                try push16(ctx, old_sr.toInt());
                try push16(ctx, info.ir);
                try push32(ctx, info.access_addr);
                // Special status word: R/W, I/N and the three FC pins. Bits
                // 5..15 are undefined on hardware; MAME leaves the instruction
                // register there and the tests check it, so we match.
                const fc: u16 = (info.ir & 0xFFE0) |
                    (if (info.read) @as(u16, 0x10) else 0) |
                    (if (info.program) @as(u16, 2) else 1) |
                    (if (old_sr.s) @as(u16, 4) else 0);
                try push16(ctx, fc);
            } else {
                try push32(ctx, ctx.cpu.pc);
                try push16(ctx, old_sr.toInt());
            }
        }

        // ------------------------------------------------------------ interrupts

        /// Host-facing: assert an interrupt level on IPL0-2. Pure data write.
        pub fn setIpl(c: *Cpu, level: u3) void {
            c.pending_ipl = level;
        }

        fn interruptPending(c: *const Cpu) bool {
            return c.pending_ipl != 0 and
                (c.pending_ipl == 7 or c.pending_ipl > c.sr.ipl);
        }

        fn takeInterrupt(c: *Cpu, bus: *BusT) void {
            const level = c.pending_ipl;
            // Autovector only: the Genesis has no vectored peripherals. A
            // vectored ack would slot in right here.
            enterException(c, bus, Exception.autovector(level), null);
            c.sr.ipl = level;
            c.cycles += 10; // interrupt ack overhead on top of the frame
        }

        // ------------------------------------------------------------------ step

        pub fn reset(c: *Cpu, bus: *BusT) void {
            var ctx = Ctx{ .cpu = c, .bus = bus };
            c.sr = .{ .s = true, .ipl = 7 };
            c.stopped = false;
            c.halted = false;
            c.pending_ipl = 0;
            c.a[7] = read32(&ctx, 0, true) catch 0;
            c.pc = read32(&ctx, 4, true) catch 0;
            c.cycles += 40;
        }

        /// Execute one instruction (or take one pending exception).
        pub fn step(c: *Cpu, bus: *BusT) void {
            if (c.halted) {
                c.cycles += 4;
                return;
            }
            // Trace outranks interrupts, and the traced instruction has already
            // completed in a previous step.
            if (c.trace_pending) {
                c.trace_pending = false;
                enterException(c, bus, .trace, null);
                return;
            }
            if (interruptPending(c)) {
                takeInterrupt(c, bus);
                return;
            }
            if (c.stopped) {
                c.cycles += 4;
                return;
            }

            // Sampled before execution: an instruction that sets T does not
            // trace itself.
            const tracing = c.sr.t;

            var ctx = Ctx{ .cpu = c, .bus = bus };
            const start_pc = c.pc;
            execute(&ctx) catch |f| {
                if (ctx.fault) |*info| info.ir = bus.read16(@truncate(start_pc & ~@as(u32, 1)));
                // Illegal instruction and privilege violation stack the address
                // of the offending instruction, not the one after it. The
                // trap-like faults stack the following instruction.
                switch (f) {
                    Fault.IllegalInstruction, Fault.LineA, Fault.LineF => {
                        c.pc = start_pc;
                    },
                    else => {},
                }
                enterException(c, bus, faultVector(f), ctx.fault);
                return;
            };

            // An instruction that faulted has already entered an exception;
            // it does not additionally trace.
            c.trace_pending = tracing;
        }

        fn faultVector(f: Fault) Exception {
            return switch (f) {
                Fault.AddressError => .address_error,
                Fault.IllegalInstruction => .illegal_instruction,
                Fault.LineA => .line_a,
                Fault.LineF => .line_f,
            };
        }

        /// Run until the cycle budget is spent. Instructions are not split, so
        /// this may overshoot; the caller carries the debt into the next slice.
        pub fn run(c: *Cpu, bus: *BusT, budget: u64) u64 {
            const start = c.cycles;
            while (c.cycles -% start < budget) step(c, bus);
            return c.cycles -% start;
        }

        // -------------------------------------------------------------- dispatch

        fn execute(ctx: *Ctx) Fault!void {
            const op = try fetch16(ctx);
            ctx.ir = op;
            const instr = decode.table[op];
            ctx.cpu.cycles += instr.base_cycles;

            switch (instr.mnemonic) {
                .move, .movea => try opMove(ctx, op, instr),
                .moveq => opMoveq(ctx, op),
                .lea => try opLea(ctx, op),
                .nop => {},
                .rts => ctx.cpu.pc = try pop32(ctx),
                .bra, .bsr, .bcc => try opBranch(ctx, op, instr),

                .line_a => return Fault.LineA,
                .line_f => return Fault.LineF,
                // TODO(M1-M4): the remaining families. Until a handler exists,
                // the opcode behaves as an illegal instruction, which is a
                // loud, correct-shaped failure rather than a silent no-op.
                else => return Fault.IllegalInstruction,
            }
        }

        // -------------------------------------------------------------- handlers

        fn opMove(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const src_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const src_reg: u3 = @truncate(op);
            const dst_mode = EaMode.decode(@truncate(op >> 6), @truncate(op >> 9)).?;
            const dst_reg: u3 = @truncate(op >> 9);

            // Each operand's cost is charged before its bus cycle: an address
            // error partway through still pays for the work already done.
            ctx.cpu.cycles += src_mode.cycles(size);
            const v = try readEa(ctx, src_mode, src_reg, size);

            if (instr.mnemonic == .movea) {
                // MOVEA sign extends and touches no flags.
                ctx.cpu.a[dst_reg] = signExtend(v, size);
                return;
            }
            ctx.cpu.sr.setNzvc(flags.logic(v, size));
            ctx.cpu.cycles += dst_mode.destCycles(size);
            try writeEa(ctx, dst_mode, dst_reg, size, v);
        }

        fn opMoveq(ctx: *Ctx, op: u16) void {
            const v: u32 = @bitCast(@as(i32, @as(i8, @bitCast(@as(u8, @truncate(op))))));
            ctx.cpu.d[@as(u3, @truncate(op >> 9))] = v;
            ctx.cpu.sr.setNzvc(flags.logic(v, .long));
        }

        fn opLea(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const addr = try calcEa(ctx, mode, @truncate(op), .long, false);
            ctx.cpu.a[@as(u3, @truncate(op >> 9))] = addr;
        }

        fn opBranch(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const c = ctx.cpu;
            const base = c.pc; // address of the extension word
            const disp8: i8 = @bitCast(@as(u8, @truncate(op)));

            const disp: i32 = if (disp8 == 0)
                @as(i16, @bitCast(try fetch16(ctx)))
            else
                disp8;
            const target = base +% @as(u32, @bitCast(disp));

            switch (instr.mnemonic) {
                .bra => try jump(ctx, target),
                .bsr => {
                    // The return address is written before the new PC is
                    // prefetched, so an odd target still leaves it on the stack.
                    try push32(ctx, c.pc);
                    try jump(ctx, target);
                },
                .bcc => {
                    if (flags.testCondition(c.sr, @truncate(op >> 8))) {
                        try jump(ctx, target);
                    } else {
                        // Not taken costs less; the 16-bit form costs more.
                        c.cycles += if (disp8 == 0) @as(u8, 4) else 0;
                    }
                },
                else => unreachable,
            }
        }

        /// Loading an odd PC faults immediately. Without a prefetch queue this
        /// is where the fault has to be raised: the real chip discovers it when
        /// it prefetches from the new address, which is still inside this
        /// instruction.
        fn jump(ctx: *Ctx, target: u32) Fault!void {
            // The PC is loaded first and the prefetch from it is what faults,
            // so the address error frame stacks the target, not the return
            // address.
            ctx.cpu.pc = target;
            if (target & 1 != 0) return addressError(ctx, target, true, true);
        }
    };
}

// ------------------------------------------------------------------------ tests

/// Flat 16 MiB memory. The reference bus implementation, and what the
/// conformance harness runs against.
pub const FlatBus = struct {
    ram: []u8,

    pub fn read8(b: *FlatBus, addr: u24) u8 {
        return b.ram[addr];
    }
    pub fn read16(b: *FlatBus, addr: u24) u16 {
        return std.mem.readInt(u16, b.ram[addr..][0..2], .big);
    }
    pub fn write8(b: *FlatBus, addr: u24, v: u8) void {
        b.ram[addr] = v;
    }
    pub fn write16(b: *FlatBus, addr: u24, v: u16) void {
        std.mem.writeInt(u16, b.ram[addr..][0..2], v, .big);
    }
};

const TestCore = Core(FlatBus);

fn testBus(alloc: std.mem.Allocator) !FlatBus {
    const ram = try alloc.alloc(u8, 0x1_0000);
    @memset(ram, 0);
    return .{ .ram = ram };
}

test "reset loads SSP and PC from the vector table" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0, 0x0000);
    bus.write16(2, 0x8000); // SSP = 0x00008000
    bus.write16(4, 0x0000);
    bus.write16(6, 0x0400); // PC  = 0x00000400

    var c = Cpu{};
    TestCore.reset(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x8000), c.a[7]);
    try std.testing.expectEqual(@as(u32, 0x0400), c.pc);
    try std.testing.expect(c.sr.s and c.sr.ipl == 7);
}

test "moveq sign extends and sets flags" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x70FF); // moveq #-1,d0
    var c = Cpu{ .pc = 0x400 };
    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0xFFFF_FFFF), c.d[0]);
    try std.testing.expect(c.sr.n and !c.sr.z and !c.sr.v and !c.sr.c);
    try std.testing.expectEqual(@as(u32, 0x402), c.pc);
    try std.testing.expectEqual(@as(u64, 4), c.cycles);
}

test "move.w #imm,(a0) writes memory and sets flags" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x30BC); // move.w #$1234,(a0)
    bus.write16(0x402, 0x1234);
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1000;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u16, 0x1234), bus.read16(0x1000));
    try std.testing.expectEqual(@as(u32, 0x404), c.pc);
    try std.testing.expect(!c.sr.z and !c.sr.n);
}

test "byte access through -(a7) keeps the stack word aligned" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x1F00); // move.b d0,-(a7)
    var c = Cpu{ .pc = 0x400 };
    c.a[7] = 0x1000;
    c.d[0] = 0x42;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x0FFE), c.a[7]);
    try std.testing.expectEqual(@as(u8, 0x42), bus.read8(0x0FFE));
}

test "odd word access raises an address error" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000); // address error vector -> 0x2000

    bus.write16(0x400, 0x3010); // move.w (a0),d0
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1001; // odd
    c.a[7] = 0x1800;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x2000), c.pc);
    try std.testing.expect(c.sr.s);
    // 7-word group 0 frame.
    try std.testing.expectEqual(@as(u32, 0x1800 - 14), c.a[7]);
}

test "a faulting postincrement write leaves the register alone" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000);

    // A read increments before the bus cycle, a write after, so only the
    // write leaves the register untouched when the address is odd.
    bus.write16(0x400, 0x30C0); // move.w d0,(a0)+
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1001;
    c.a[7] = 0x1800;
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x1001), c.a[0]);

    bus.write16(0x400, 0x3018); // move.w (a0)+,d0
    c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1001;
    c.a[7] = 0x1800;
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x1003), c.a[0]);
}

test "PC relative operands report program space in the group 0 frame" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000);

    bus.write16(0x400, 0x303A); // move.w (d16,pc),d0
    bus.write16(0x402, 0x0001); // odd displacement -> odd address
    var c = Cpu{ .pc = 0x400, .sr = .{ .s = false } };
    c.a[7] = 0x1800;
    c.usp = 0x1800;

    TestCore.step(&c, &bus);

    // Special status word: read from program space, user mode.
    const ssw = bus.read16(@truncate(c.a[7]));
    try std.testing.expectEqual(@as(u16, 0x12), ssw & 0x1F);
}

test "unimplemented opcodes take the illegal instruction vector" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x12, 0x3000); // illegal instruction vector
    bus.write16(0x400, 0x4AFC); // ILLEGAL
    var c = Cpu{ .pc = 0x400, .a = .{ 0, 0, 0, 0, 0, 0, 0, 0x1800 } };

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x3000), c.pc);
    try std.testing.expectEqual(@as(u32, 0x1800 - 6), c.a[7]); // 3-word frame
}

test "bra and bsr" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x6010); // bra.s +16
    var c = Cpu{ .pc = 0x400 };
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x412), c.pc);

    bus.write16(0x500, 0x6100); // bsr.w
    bus.write16(0x502, 0x0020);
    c = Cpu{ .pc = 0x500 };
    c.a[7] = 0x1800;
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x522), c.pc);
    try std.testing.expectEqual(@as(u32, 0x504), bus.read16(0x17FE)); // return addr low
}

test "interrupts respect the mask and wake STOP" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x6E, 0x2000); // autovector 3 -> vector 27 -> 0x6C..0x6F

    var c = Cpu{ .pc = 0x400, .sr = .{ .s = true, .ipl = 3 }, .stopped = true };
    c.a[7] = 0x1800;

    // Level 3 does not beat a mask of 3.
    TestCore.setIpl(&c, 3);
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x400), c.pc);
    try std.testing.expect(c.stopped);

    // Level 4 does.
    TestCore.setIpl(&c, 4);
    TestCore.step(&c, &bus);
    try std.testing.expect(!c.stopped);
    try std.testing.expectEqual(@as(u3, 4), c.sr.ipl);
}

test "run returns at least the requested budget" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    for (0..64) |i| bus.write16(@intCast(0x400 + i * 2), 0x4E71); // nop
    var c = Cpu{ .pc = 0x400 };

    const ran = TestCore.run(&c, &bus, 40);
    try std.testing.expect(ran >= 40);
    try std.testing.expectEqual(@as(u32, 0x400 + 40 / 4 * 2), c.pc);
}
