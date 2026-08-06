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
pub const Fault = error{
    AddressError,
    IllegalInstruction,
    LineA,
    LineF,
    PrivilegeViolation,
    ZeroDivide,
    Chk,
    Trap,
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
            /// TRAP's vector depends on which of the 16 it is; every other
            /// fault has a fixed one, so `faultVector` only reads this for
            /// `Fault.Trap`.
            trap_vector: Exception = .trap_0,
            /// CHK's trap cost depends on which bound it tripped (`Dn > bound`
            /// exits 2 cycles faster than `Dn < 0`); read back in `step` once
            /// `enterException` has added the flat exception overhead, since
            /// that overhead hasn't landed yet at the point CHK itself knows
            /// which case applies.
            chk_delta: u8 = 0,
            /// Set once a handler has passed a point where the group 0 frame's
            /// PC diverges from the pre-fault fallback. IR keeps the fallback
            /// value in every case here -- see the comment on `markPrefetch`.
            prefetch: ?struct { ir: ?u16, pc: u32 } = null,
        };

        /// Write-side mark: real hardware has already prefetched one word past
        /// a memory destination's own EA/write cycle by fault time, so PC
        /// ends up one word ahead of where it'd otherwise be -- regardless of
        /// the destination's own extension-word count, even for absolute
        /// destinations (unlike the read side, where the source's extension
        /// words directly determine the prefetch advance -- see
        /// `markReadFault`). The one exception: when the *source* operand
        /// used no bus cycle at all (register direct), an absolute
        /// destination's prefetch gets a second bonus word too, landing past
        /// both its extension words instead of just one -- that spare source
        /// bus slot is what buys it. IR sometimes follows along and
        /// sometimes doesn't, in a way that (unlike PC) depends on the source
        /// addressing mode's own internal bus timing (e.g. predecrement's
        /// extra address-computation cycle) rather than just word counts --
        /// not modeled here; see DESIGN.md §5.4.
        fn markPrefetch(ctx: *Ctx, pc_before: u32, mode: EaMode, src_free: bool) void {
            // abs_long has two extension words; "fully consumed" is +4. Every
            // other mode's bonus is +2 either way, so only abs_long can tell
            // the difference.
            const bonus: u32 = if (src_free and mode == .abs_long) 4 else 2;
            ctx.prefetch = .{ .ir = null, .pc = pc_before +% bonus };
        }

        /// Read-side mark: unlike a destination write, an operand read's own
        /// extension-word fetches never get a chance to prefetch ahead
        /// before the read itself faults, so PC stays exactly where it was
        /// before this operand's addressing mode touched the bus at all --
        /// regardless of how many extension words that addressing needed.
        /// Predecrement is the one exception: its address-computation cycle
        /// has no bus work of its own, giving the prefetch a free slot to
        /// complete in before the read, same as a destination write. IR
        /// keeps reflecting the instruction still executing, i.e. the
        /// pre-fault fallback. See DESIGN.md §5.4.
        fn markReadFault(ctx: *Ctx, pc_before: u32, mode: EaMode, size: Size) void {
            // Modes with no ALU work between the last extension-word fetch
            // and the access (predecrement's subtract, or absolute's fetched
            // value used as-is) leave the bus idle for one extra prefetch;
            // modes that must add the extension word to a register
            // (displacement/indexed, base or PC-relative) don't, and the
            // frame shows PC as if none of this operand's extension words
            // had been fetched yet at all. Predecrement's bonus needs a
            // word/byte-sized subtract to fit in that idle slot -- a long
            // subtract takes long enough that the slot's gone, same as if
            // there were no idle time at all.
            const pc = switch (mode) {
                .addr_predec => if (size == .long) pc_before else pc_before +% 2,
                .abs_word, .abs_long, .addr_ind, .addr_postinc => ctx.cpu.pc,
                else => pc_before,
            };
            ctx.prefetch = .{ .ir = null, .pc = pc };
        }

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
            // -(An) writes low word first on real hardware; unobservable here
            // since calcEa's predecrement check already faults (at addr+2,
            // matching that ordering) before either word is written.
            try write16(ctx, addr, @truncate(v >> 16));
            try write16(ctx, addr +% 2, @truncate(v));
        }

        /// One MOVEM register transfer's worth of predecrement/postincrement
        /// address stepping. Checks alignment before committing the register
        /// update in every case (read or write, word or long) - MOVEM has no
        /// analog of the single-operand "word reads increment before the bus
        /// cycle" quirk `calcEa` models for other instructions.
        fn movemStep(ctx: *Ctx, reg: u3, size: Size, predec: bool, read: bool) Fault!u32 {
            const cur = ctx.cpu.a[reg];
            const addr = if (predec) cur -% size.bytes() else cur;
            if (addr & 1 != 0) {
                const fault_addr = if (predec and size == .long) addr +% 2 else addr;
                return addressError(ctx, fault_addr, read, false);
            }
            ctx.cpu.a[reg] = if (predec) addr else addr +% size.bytes();
            return addr;
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

        fn pop16(ctx: *Ctx) Fault!u16 {
            const v = try read16(ctx, ctx.cpu.a[7], false);
            ctx.cpu.a[7] +%= 2;
            return v;
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
            // Marked up front, before any extension-word fetch or the
            // predecrement/postincrement misalignment checks below: those
            // checks raise `Fault.AddressError` straight out of this switch,
            // never reaching a caller's own mark call, so every caller (not
            // just MOVE) needs this covered here. `src_free` is a MOVE-only
            // refinement (a register-direct source frees up a bus slot for
            // an absolute destination); every other caller passes `false`,
            // and MOVE's `writeEa` re-marks afterward when it applies.
            const pc_before = c.pc;
            if (write) markPrefetch(ctx, pc_before, mode, false) else markReadFault(ctx, pc_before, mode, size);
            return switch (mode) {
                .addr_ind => c.a[reg],
                .addr_postinc => blk: {
                    const addr = c.a[reg];
                    // Word reads increment before the bus cycle; writes and
                    // long reads write the register back only after the first
                    // bus cycle succeeds, so their misaligned faults leave it
                    // holding its original value. (Empirical, from the
                    // conformance data; word reads really do differ.)
                    if (addr & 1 != 0 and size != .byte and (write or size == .long)) {
                        break :blk addressError(ctx, addr, !write, false);
                    }
                    c.a[reg] +%= stackAdjust(reg, size);
                    break :blk addr;
                },
                .addr_predec => blk: {
                    const addr = c.a[reg] -% stackAdjust(reg, size);
                    // A long write goes low word first, so the first bus cycle
                    // is at addr+2 and a misaligned fault happens before the
                    // register writeback. Reads go high word first and fault
                    // with the register already decremented. Word writes
                    // commit the register before the (later) fault too -
                    // unlike long, there's no split access to catch it early.
                    if (write and size == .long and addr & 1 != 0) {
                        break :blk addressError(ctx, addr +% 2, false, false);
                    }
                    c.a[reg] = addr;
                    break :blk addr;
                },
                .addr_disp => blk: {
                    const d: i16 = @bitCast(try fetch16(ctx));
                    break :blk c.a[reg] +% @as(u32, @bitCast(@as(i32, d)));
                },
                .addr_index => try indexed(ctx, c.a[reg]),
                .abs_word => blk: {
                    const w: i16 = @bitCast(try fetch16(ctx));
                    // Read-side only: the extra word actually advances the
                    // prefetch here, unlike the write side's flat bonus.
                    if (!write) markReadFault(ctx, pc_before, mode, size);
                    break :blk @bitCast(@as(i32, w)); // sign extended
                },
                .abs_long => blk: {
                    const v = try fetch32(ctx);
                    if (!write) markReadFault(ctx, pc_before, mode, size);
                    break :blk v;
                },
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
                else => blk: {
                    // `calcEa` marks the prefetch state itself, including
                    // for faults it raises internally.
                    const addr = try calcEa(ctx, mode, reg, size, false);
                    break :blk try readAt(ctx, addr, size, mode.isProgram());
                },
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

        fn writeEa(ctx: *Ctx, mode: EaMode, reg: u3, size: Size, v: u32, src_free: bool) Fault!void {
            const c = ctx.cpu;
            switch (mode) {
                .data_reg => setReg(&c.d[reg], size, v),
                // Address registers are never partially written: word values
                // are sign extended across all 32 bits.
                .addr_reg => c.a[reg] = signExtend(v, size),
                .immediate => unreachable,
                else => {
                    // `calcEa` marks the prefetch state itself (with
                    // `src_free` always false); re-mark afterward when the
                    // MOVE-specific refinement applies.
                    const pc_before = ctx.cpu.pc;
                    const addr = try calcEa(ctx, mode, reg, size, true);
                    if (src_free and mode == .abs_long) markPrefetch(ctx, pc_before, mode, true);
                    try writeAt(ctx, addr, size, v);
                },
            }
        }

        fn setReg(dst: *u32, size: Size, v: u32) void {
            const m = size.mask();
            dst.* = (dst.* & ~m) | (v & m);
        }

        /// A read-modify-write operand, holding wherever it was read from
        /// (register or memory address) alongside the value read. This is a
        /// read-modify-write, but the first bus cycle is a read: `read`
        /// passes write=false to `calcEa` so postinc/predec's fault-timing
        /// and register-writeback rules match a plain read (real hardware
        /// increments/decrements the register on the read side, then reuses
        /// the same address for the write with no further side effect).
        const RmwOperand = struct {
            loc: union(enum) { reg: u3, mem: u32 },
            old: u32,

            fn read(ctx: *Ctx, mode: EaMode, reg: u3, size: Size) Fault!RmwOperand {
                if (mode == .data_reg) {
                    return .{ .loc = .{ .reg = reg }, .old = ctx.cpu.d[reg] & size.mask() };
                }
                const addr = try calcEa(ctx, mode, reg, size, false);
                return .{ .loc = .{ .mem = addr }, .old = try readAt(ctx, addr, size, false) };
            }

            fn writeBack(o: RmwOperand, ctx: *Ctx, size: Size, v: u32) Fault!void {
                switch (o.loc) {
                    .reg => |r| setReg(&ctx.cpu.d[r], size, v),
                    .mem => |addr| try writeAt(ctx, addr, size, v),
                }
            }
        };

        /// A `Dn`/`An`/immediate source never risks a fault: the read is
        /// always a register access or the instruction stream itself.
        fn noFaultSource(m: EaMode) bool {
            return m == .data_reg or m == .addr_reg or m == .immediate;
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
        fn exceptionCycles(e: Exception) u8 {
            if (e.isGroup0()) return 50;
            if (e == .zero_divide) return 38;
            if (e == .chk) return 40;
            return 34;
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
                if (ctx.fault) |*info| {
                    // Past the mark, real hardware has already prefetched the
                    // next opcode into IR and advanced PC past it; before the
                    // mark, both still reflect the faulting instruction itself.
                    if (ctx.prefetch) |p| {
                        info.ir = p.ir orelse bus.read16(@truncate(start_pc & ~@as(u32, 1)));
                        c.pc = p.pc;
                    } else {
                        info.ir = bus.read16(@truncate(start_pc & ~@as(u32, 1)));
                    }
                }
                // Illegal instruction and privilege violation stack the address
                // of the offending instruction, not the one after it. The
                // trap-like faults stack the following instruction.
                switch (f) {
                    Fault.IllegalInstruction, Fault.LineA, Fault.LineF, Fault.PrivilegeViolation => {
                        c.pc = start_pc;
                    },
                    else => {},
                }
                enterException(c, bus, faultVector(f, &ctx), ctx.fault);
                if (f == Fault.Chk) c.cycles -= ctx.chk_delta;
                return;
            };

            // An instruction that faulted has already entered an exception;
            // it does not additionally trace.
            c.trace_pending = tracing;
        }

        fn faultVector(f: Fault, ctx: *const Ctx) Exception {
            return switch (f) {
                Fault.AddressError => .address_error,
                Fault.IllegalInstruction => .illegal_instruction,
                Fault.LineA => .line_a,
                Fault.LineF => .line_f,
                Fault.PrivilegeViolation => .privilege_violation,
                Fault.ZeroDivide => .zero_divide,
                Fault.Chk => .chk,
                Fault.Trap => ctx.trap_vector,
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
                .rts => {
                    // Like Bcc/DBcc, the fault frame shows RTS's own
                    // pre-jump PC, not the popped (bad) target.
                    const ret = try pop32(ctx);
                    ctx.prefetch = .{ .ir = null, .pc = ctx.cpu.pc };
                    try jump(ctx, ret);
                },
                .bra, .bsr, .bcc => try opBranch(ctx, op, instr),

                .negx, .clr, .neg, .not, .tst, .nbcd => try opAluSingle(ctx, op, instr),
                .addq, .subq => try opQuick(ctx, op, instr),
                .scc => try opScc(ctx, op),
                .dbcc => try opDbcc(ctx, op),
                .add, .sub, .and_, .or_, .eor, .cmp => try opAlu2(ctx, op, instr),
                .adda, .suba, .cmpa => try opAluA(ctx, op, instr),
                .ori, .andi, .subi, .addi, .eori, .cmpi => try opImmediateAlu(ctx, op, instr),
                .ori_ccr, .ori_sr, .andi_ccr, .andi_sr, .eori_ccr, .eori_sr => try opImmediateSr(ctx, instr),
                .addx, .subx => try opAddSubX(ctx, op, instr),
                .cmpm => try opCmpm(ctx, op, instr),
                .asl, .asr, .lsl, .lsr, .rol, .ror, .roxl, .roxr => try opShift(ctx, op, instr),
                .btst, .bchg, .bclr, .bset => try opBitOp(ctx, op, instr),
                .movep => try opMovep(ctx, op, instr),
                .abcd, .sbcd => try opBcd(ctx, op, instr),
                .exg => opExg(ctx, op),
                .mulu, .muls => try opMul(ctx, op, instr),
                .divu, .divs => try opDiv(ctx, op, instr),

                .swap => opSwap(ctx, op),
                .ext => opExt(ctx, op, instr),
                .pea => try opPea(ctx, op),
                .link => try opLink(ctx, op),
                .unlk => try opUnlk(ctx, op),
                .jsr => try opJsr(ctx, op),
                .jmp => try opJmp(ctx, op),
                .move_from_sr => try opMoveFromSr(ctx, op),
                .move_to_ccr => try opMoveToCcr(ctx, op),
                .move_to_sr => try opMoveToSr(ctx, op, instr),
                .move_usp => try opMoveUsp(ctx, op, instr),
                .trap => try opTrap(ctx, op),
                .trapv => try opTrapv(ctx),
                .chk => try opChk(ctx, op),
                .stop => try opStop(ctx, instr),
                .reset_insn => try opReset(ctx, instr),
                .rte => try opRte(ctx, instr),
                .rtr => try opRtr(ctx),
                .movem => try opMovem(ctx, op, instr),
                .tas => try opTas(ctx, op),

                .line_a => return Fault.LineA,
                .line_f => return Fault.LineF,
                // ILLEGAL (0x4AFC) and any opcode decode couldn't identify
                // both fall here, which is exactly right: the "illegal
                // instruction" trap is itself an illegal instruction fault.
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
            // error partway through still pays for the work already done. A
            // faulting long operand only got as far as its first word access,
            // so it pays the word cost, not the long one.
            ctx.cpu.cycles += src_mode.cycles(size);
            const v = blk: {
                errdefer if (size == .long) {
                    ctx.cpu.cycles -= 4;
                };
                break :blk try readEa(ctx, src_mode, src_reg, size);
            };

            if (instr.mnemonic == .movea) {
                // MOVEA sign extends and touches no flags.
                ctx.cpu.a[dst_reg] = signExtend(v, size);
                return;
            }

            // Flag latch order matters when the destination write faults:
            // word moves and the low-word-first predecrement have already
            // updated CCR, everything else has not. (MAME's microcode order,
            // checked by the conformance data.)
            const cc = flags.logic(v, size);
            const flags_first = size != .long or dst_mode == .addr_predec;
            if (flags_first) ctx.cpu.sr.setNzvc(cc);

            ctx.cpu.cycles += dst_mode.destCycles(size);
            {
                const src_free = src_mode == .data_reg or src_mode == .addr_reg;
                // Predecrement pays its full cost even when it faults; its
                // first access is the one the destCycles discount covers.
                // A long destination gives back a flat 4 regardless of EA
                // kind, same shape as the source fault above. A word/byte
                // destination only needs that same -4 for `abs_long`: its
                // two extension words overlap the bus differently once the
                // source read (anything but a free register) has already
                // used it.
                errdefer if (dst_mode != .addr_predec) {
                    if (size == .long or (dst_mode == .abs_long and !src_free)) {
                        ctx.cpu.cycles -= 4;
                    }
                };
                try writeEa(ctx, dst_mode, dst_reg, size, v, src_free);
            }
            if (!flags_first) ctx.cpu.sr.setNzvc(cc);
        }

        /// NEGX/CLR/NEG/NOT/TST: one data operand, read (and for all but TST,
        /// written back) at the same address computed once. CLR/NOT leave X
        /// alone (`logic` shape); NEG/NEGX affect it (`sub`/`subx` shape).
        fn opAluSingle(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const mn = instr.mnemonic;

            if (mn == .tst) {
                ctx.cpu.cycles += mode.cycles(size);
                // A faulting long read only ever got as far as its first
                // word; give back the second word's cost it never paid for.
                errdefer if (size == .long) {
                    ctx.cpu.cycles -= 4;
                };
                const v = try readEa(ctx, mode, reg, size);
                ctx.cpu.sr.setNzvc(flags.logic(v, size));
                return;
            }

            // Charged as if this were the word-sized, register-destination
            // form: a faulting read never reaches the long size's extra ALU
            // cycles or a memory destination's write-back extra, both added
            // below only once the read has actually succeeded.
            ctx.cpu.cycles += mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const operand = try RmwOperand.read(ctx, mode, reg, size);
            const old = operand.old;

            const x_in = ctx.cpu.sr.x;
            const old_z = ctx.cpu.sr.z;
            const mask = size.mask();
            const Result = struct { value: u32, cc: cpu_mod.Ccr };
            const r: Result = switch (mn) {
                .clr => .{ .value = 0, .cc = flags.logic(0, size) },
                .not => blk: {
                    const v = ~old & mask;
                    break :blk .{ .value = v, .cc = flags.logic(v, size) };
                },
                .neg => .{ .value = (0 -% old) & mask, .cc = flags.sub(0, old, size) },
                .negx => .{
                    .value = (0 -% old -% @as(u32, @intFromBool(x_in))) & mask,
                    .cc = flags.subx(0, old, x_in, size, old_z),
                },
                .nbcd => blk: {
                    const r = flags.bcdSub(0, @truncate(old), x_in, old_z);
                    break :blk .{ .value = r.value, .cc = r.cc };
                },
                else => unreachable,
            };

            // NBCD's decode-time base already bakes in its own (smaller,
            // register-vs-memory) write-back cost; NEGX/CLR/NEG/NOT instead
            // pay a flat 2 more for long's extra ALU pass plus a memory
            // destination's write-back extra, once the read is known good.
            if (mn != .nbcd) {
                const alu_extra: u8 = if (size == .long) 2 else 0;
                const rmw_extra: u8 = if (mode == .data_reg) 0 else if (size == .long) 6 else 4;
                ctx.cpu.cycles += alu_extra + rmw_extra;
            }

            try operand.writeBack(ctx, size, r.value);

            switch (mn) {
                .clr, .not => ctx.cpu.sr.setNzvc(r.cc),
                else => ctx.cpu.sr.setNzvcx(r.cc),
            }
        }

        /// ADDQ/SUBQ: immediate 1-8 (0 in the field means 8). An destination
        /// is always treated as a long add/sub on the whole register with no
        /// flags touched, regardless of the encoded size — a real hardware
        /// special case, not a composable one.
        fn opQuick(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const field: u3 = @truncate(op >> 9);
            const data: u32 = if (field == 0) 8 else field;
            const is_sub = instr.mnemonic == .subq;

            if (mode == .addr_reg) {
                ctx.cpu.a[reg] = if (is_sub) ctx.cpu.a[reg] -% data else ctx.cpu.a[reg] +% data;
                return;
            }

            // Charged as the word-sized form; a faulting read never reaches
            // the long/write-back bonus added below, once the read has
            // actually succeeded (same shape as `opAlu2`'s RMW branch).
            ctx.cpu.cycles += mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const operand = try RmwOperand.read(ctx, mode, reg, size);
            const old = operand.old;

            const cc = if (is_sub) flags.sub(old, data, size) else flags.add(old, data, size);
            const result = (if (is_sub) old -% data else old +% data) & size.mask();

            if (mode == .data_reg) {
                if (size == .long) ctx.cpu.cycles += 4;
            } else {
                ctx.cpu.cycles += if (size == .long) @as(u8, 8) else 4;
            }
            try operand.writeBack(ctx, size, result);
            ctx.cpu.sr.setNzvcx(cc);
        }

        /// Scc: writes all-ones on true, all-zero on false, to a byte
        /// destination. Read-modify-write in shape only for memory operands.
        fn opScc(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const cond: u4 = @truncate(op >> 8);
            const true_cond = flags.testCondition(ctx.cpu.sr, cond);
            const v: u32 = if (true_cond) 0xFF else 0x00;

            if (mode == .data_reg) {
                // A true condition costs 2 more cycles than false, for any
                // condition (not just ST) — a real hardware quirk the
                // decode-time base_cycles can't express.
                if (true_cond) ctx.cpu.cycles += 2;
                setReg(&ctx.cpu.d[reg], .byte, v);
            } else {
                // Full EA cost, not the predecrement-discounted `destCycles`:
                // Scc's memory form is a plain write, not a read that
                // computes its address ahead of the bus cycle.
                ctx.cpu.cycles += mode.cycles(.byte);
                const addr = try calcEa(ctx, mode, reg, .byte, false);
                try writeAt(ctx, addr, .byte, v);
            }
        }

        /// DBcc: if the condition is false, decrement the low word of Dn and
        /// branch while it hasn't wrapped from 0 to -1. The condition true and
        /// the counter-expired cases both fall through without branching.
        fn opDbcc(ctx: *Ctx, op: u16) Fault!void {
            const c = ctx.cpu;
            const reg: u3 = @truncate(op);
            const cond: u4 = @truncate(op >> 8);
            const base = c.pc; // address of the displacement word
            const disp: i16 = @bitCast(try fetch16(ctx));

            if (flags.testCondition(c.sr, cond)) return;

            const count: u16 = @truncate(c.d[reg]);
            const new_count = count -% 1;

            if (new_count != 0xFFFF) {
                c.cycles -= 2; // branch taken: 10 total, base covers the untaken 12
                // The branch target is validated before Dn commits: a
                // faulting branch (odd target) leaves Dn at its old value.
                // Unlike Bcc's 8-bit form, DBcc always fetches its
                // displacement word, and the fault frame reflects that
                // fetch having completed -- the natural post-fetch PC,
                // not the pre-fetch `base`.
                ctx.prefetch = .{ .ir = null, .pc = ctx.cpu.pc };
                try jump(ctx, base +% @as(u32, @bitCast(@as(i32, disp))));
                setReg(&c.d[reg], .word, new_count);
            } else {
                c.cycles += 2; // counter expired, no branch: 14 total
                setReg(&c.d[reg], .word, new_count);
            }
        }

        /// Shared by `opAlu2`'s ea-is-destination direction and
        /// `opImmediateAlu`: both are "read old, combine with a source, maybe
        /// write back" over the same six operations, just with the source
        /// coming from Dn vs an immediate.
        const AluOp = enum { add, sub, cmp, and_, or_, eor };
        const AluResult = struct { value: u32, cc: cpu_mod.Ccr, affects_x: bool, writes: bool };

        fn aluCompute(kind: AluOp, old: u32, src: u32, size: Size) AluResult {
            return switch (kind) {
                .add => .{ .value = (old +% src) & size.mask(), .cc = flags.add(old, src, size), .affects_x = true, .writes = true },
                .sub => .{ .value = (old -% src) & size.mask(), .cc = flags.sub(old, src, size), .affects_x = true, .writes = true },
                .cmp => .{ .value = 0, .cc = flags.sub(old, src, size), .affects_x = false, .writes = false },
                .and_ => blk: {
                    const v = old & src;
                    break :blk .{ .value = v, .cc = flags.logic(v, size), .affects_x = false, .writes = true };
                },
                .or_ => blk: {
                    const v = old | src;
                    break :blk .{ .value = v, .cc = flags.logic(v, size), .affects_x = false, .writes = true };
                },
                .eor => blk: {
                    const v = old ^ src;
                    break :blk .{ .value = v, .cc = flags.logic(v, size), .affects_x = false, .writes = true };
                },
            };
        }

        /// ADD/SUB/AND/OR/EOR/CMP two-operand forms. One operand is always
        /// Dn; `is_re` picks which side is the destination. EOR only ever
        /// writes to `ea` (which may itself be Dn); CMP only ever reads,
        /// discarding the result.
        fn opAlu2(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const mn = instr.mnemonic;
            const dn: u3 = @truncate(op >> 9);
            const ea_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const ea_reg: u3 = @truncate(op);
            const is_re = switch (mn) {
                .cmp => false,
                .eor => true,
                else => op & 0x0100 != 0,
            };
            if (is_re) {
                try opAlu2ToEa(ctx, mn, size, dn, ea_mode, ea_reg);
            } else {
                try opAlu2ToReg(ctx, mn, size, dn, ea_mode, ea_reg);
            }
        }

        // Dn = Dn op ea (or, for CMP, just Dn - ea into the flags).
        fn opAlu2ToReg(ctx: *Ctx, mn: decode.Mnemonic, size: Size, dn: u3, ea_mode: EaMode, ea_reg: u3) Fault!void {
            // Charged as the word-sized form; a faulting read never
            // reaches long's extra ALU pass, added below only once the
            // read has actually succeeded.
            ctx.cpu.cycles += ea_mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const src = try readEa(ctx, ea_mode, ea_reg, size);
            if (size == .long) {
                // A register or immediate source never risks a fault, so
                // it takes the full register-destination long bonus (4,
                // or 2 for CMP, which never writes back); a real memory
                // read instead takes a flat 2, the rest of that bonus
                // already spent on the read itself.
                const no_fault = noFaultSource(ea_mode);
                ctx.cpu.cycles += if (no_fault) (if (mn == .cmp) @as(u8, 2) else 4) else 2;
            }
            const dst = ctx.cpu.d[dn] & size.mask();

            switch (mn) {
                .cmp => ctx.cpu.sr.setNzvc(flags.sub(dst, src, size)),
                .add => {
                    const cc = flags.add(dst, src, size);
                    setReg(&ctx.cpu.d[dn], size, (dst +% src) & size.mask());
                    ctx.cpu.sr.setNzvcx(cc);
                },
                .sub => {
                    const cc = flags.sub(dst, src, size);
                    setReg(&ctx.cpu.d[dn], size, (dst -% src) & size.mask());
                    ctx.cpu.sr.setNzvcx(cc);
                },
                .and_, .or_ => {
                    const v = if (mn == .and_) dst & src else dst | src;
                    setReg(&ctx.cpu.d[dn], size, v);
                    ctx.cpu.sr.setNzvc(flags.logic(v, size));
                },
                else => unreachable,
            }
        }

        // ea = ea op Dn. Read-modify-write in shape only for a memory ea;
        // EOR's ea may also be Dn, a plain register-register op.
        fn opAlu2ToEa(ctx: *Ctx, mn: decode.Mnemonic, size: Size, dn: u3, ea_mode: EaMode, ea_reg: u3) Fault!void {
            // Charged as the word-sized, register-destination form; a
            // faulting read never reaches long's extra ALU pass or a memory
            // destination's write-back extra, both added below only once
            // the read has actually succeeded.
            ctx.cpu.cycles += ea_mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const src = ctx.cpu.d[dn] & size.mask();
            const operand = try RmwOperand.read(ctx, ea_mode, ea_reg, size);
            const old = operand.old;

            const kind: AluOp = switch (mn) {
                .add => .add,
                .sub => .sub,
                .and_ => .and_,
                .or_ => .or_,
                .eor => .eor,
                else => unreachable,
            };
            const r = aluCompute(kind, old, src, size);

            if (ea_mode == .data_reg) {
                // EOR.l to Dn is a real hardware outlier: 4 extra, not the
                // usual 2, since the register is read and written in the
                // same cycle slot.
                if (size == .long) ctx.cpu.cycles += if (mn == .eor) @as(u8, 4) else 2;
            } else {
                ctx.cpu.cycles += if (size == .long) @as(u8, 8) else 4;
            }
            try operand.writeBack(ctx, size, r.value);
            if (r.affects_x) ctx.cpu.sr.setNzvcx(r.cc) else ctx.cpu.sr.setNzvc(r.cc);
        }

        /// ADDA/SUBA/CMPA: word source is sign extended before the 32-bit
        /// address-register op. Unlike ADD/SUB/CMP, ADDA/SUBA never touch the
        /// flags; CMPA sets them the same way CMP does.
        fn opAluA(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const an: u3 = @truncate(op >> 9);
            const ea_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const ea_reg: u3 = @truncate(op);

            // Charged as the word-sized form; a faulting read never reaches
            // the bonus added below once the read has actually succeeded.
            ctx.cpu.cycles += ea_mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const src = signExtend(try readEa(ctx, ea_mode, ea_reg, size), size);

            switch (instr.mnemonic) {
                .adda, .suba => {
                    // Word ADDA/SUBA takes a flat +4 regardless of EA kind;
                    // long instead takes +2 for a real memory read (folded
                    // into its extra ALU pass) but +4 for a register or
                    // immediate source (which never risked a fault at all).
                    const no_fault = noFaultSource(ea_mode);
                    ctx.cpu.cycles += if (size == .word or no_fault) @as(u8, 4) else 2;
                    if (instr.mnemonic == .adda) ctx.cpu.a[an] +%= src else ctx.cpu.a[an] -%= src;
                },
                .cmpa => {
                    // CMPA takes a flat +2, the same for every EA kind and
                    // either size: it never writes back, so there's no
                    // bigger register-destination bonus to fold in.
                    ctx.cpu.cycles += 2;
                    ctx.cpu.sr.setNzvc(flags.sub(ctx.cpu.a[an], src, .long));
                },
                else => unreachable,
            }
        }

        /// MULU/MULS: 16x16 -> 32 into Dn, X unaffected. Dynamic cost is
        /// `base_cycles` (38) plus 2 per set bit of the source word for
        /// MULU; MULS instead counts 01/10 bit-pair transitions in the
        /// source (cheaper for runs of same-value bits, an MC68000 quirk).
        fn opMul(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const dn: u3 = @truncate(op >> 9);
            const ea_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const ea_reg: u3 = @truncate(op);
            ctx.cpu.cycles += ea_mode.cycles(.word);
            // A faulting read gives back a flat 34, the same for every EA
            // kind, once the automatic exception overhead lands.
            const src: u16 = @truncate(readEa(ctx, ea_mode, ea_reg, .word) catch |e| {
                ctx.cpu.cycles -= 34;
                return e;
            });

            const value: u32 = if (instr.mnemonic == .muls) blk: {
                const s: i32 = @as(i16, @bitCast(src));
                const d: i32 = @as(i16, @bitCast(@as(u16, @truncate(ctx.cpu.d[dn]))));
                break :blk @bitCast(s *% d);
            } else @as(u32, src) *% (ctx.cpu.d[dn] & 0xFFFF);

            const transition_bits: u16 = if (instr.mnemonic == .muls) src ^ (src << 1) else src;
            ctx.cpu.cycles += @as(u64, @popCount(transition_bits)) * 2;

            ctx.cpu.d[dn] = value;
            ctx.cpu.sr.setNzvc(.{ .n = value & 0x8000_0000 != 0, .z = value == 0 });
        }

        /// The real divider is a bit-serial non-restoring binary divider: 15
        /// trial subtract/shift steps whose add-vs-subtract choice (based on
        /// the running remainder's sign) each cost a data-dependent 0 or 2
        /// extra cycles. `decode.Instr.base_cycles` can't express that, so
        /// `opDiv` throws it away and computes the true total from scratch;
        /// this reproduces Jorge Cwik's reverse-engineered algorithm.
        fn divuCycles(dividend: u32, divisor: u32) u32 {
            if (dividend >> 16 >= divisor) return 10;
            var mcycles: u32 = 38;
            const hdivisor = divisor << 16;
            var dvd = dividend;
            for (0..15) |_| {
                const temp = dvd;
                dvd <<= 1;
                if (temp & 0x8000_0000 != 0) {
                    dvd -%= hdivisor;
                } else {
                    mcycles += 2;
                    if (dvd >= hdivisor) {
                        dvd -%= hdivisor;
                        mcycles -= 1;
                    }
                }
            }
            return mcycles * 2;
        }

        fn divsCycles(dividend: i32, divisor: i32) u32 {
            var mcycles: u32 = 6;
            if (dividend < 0) mcycles += 1;
            const abs_dividend: u32 = @abs(dividend);
            const abs_divisor: u32 = @abs(divisor);
            if (abs_dividend >> 16 >= abs_divisor) return (mcycles + 2) * 2;

            var aquot: u32 = abs_dividend / abs_divisor;
            mcycles += 55;
            if (divisor >= 0) {
                if (dividend >= 0) mcycles -= 1 else mcycles += 1;
            }
            for (0..15) |_| {
                const s16: i16 = @bitCast(@as(u16, @truncate(aquot)));
                if (s16 >= 0) mcycles += 1;
                aquot <<= 1;
            }
            return mcycles * 2;
        }

        /// DIVU/DIVS: 32-bit Dn / 16-bit ea -> 16-bit quotient (low word),
        /// 16-bit remainder (high word), X unaffected. Overflow (quotient
        /// doesn't fit 16 bits) sets V, clears C, and leaves Dn untouched —
        /// the value is discarded, not just the flags being "off". Division
        /// by zero traps rather than computing.
        fn opDiv(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const dn: u3 = @truncate(op >> 9);
            const ea_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const ea_reg: u3 = @truncate(op);
            // decode's base_cycles is just a placeholder (the real cost is
            // fully data-dependent, computed below); throw it away.
            ctx.cpu.cycles -= instr.base_cycles;
            ctx.cpu.cycles += ea_mode.cycles(.word);
            // A faulting read gives back a flat -4 (i.e. needs 4 added back),
            // the same for every EA kind, once the automatic exception
            // overhead lands.
            const src: u16 = @truncate(readEa(ctx, ea_mode, ea_reg, .word) catch |e| {
                ctx.cpu.cycles += 4;
                return e;
            });
            if (src == 0) return Fault.ZeroDivide;

            if (instr.mnemonic == .divu) {
                const dividend = ctx.cpu.d[dn];
                const divisor: u32 = src;
                const quotient = dividend / divisor;
                ctx.cpu.cycles += divuCycles(dividend, divisor);
                if (quotient > 0xFFFF) {
                    ctx.cpu.sr.setNzvc(.{ .v = true, .n = true });
                    return;
                }
                const remainder = dividend % divisor;
                ctx.cpu.d[dn] = (remainder << 16) | quotient;
                ctx.cpu.sr.setNzvc(.{ .n = quotient & 0x8000 != 0, .z = quotient == 0 });
            } else {
                const dividend: i32 = @bitCast(ctx.cpu.d[dn]);
                const divisor: i32 = @as(i16, @bitCast(src));
                ctx.cpu.cycles += divsCycles(dividend, divisor);
                const quotient = @divTrunc(dividend, divisor);
                if (quotient > 32767 or quotient < -32768) {
                    ctx.cpu.sr.setNzvc(.{ .v = true, .n = true });
                    return;
                }
                const remainder = @rem(dividend, divisor);
                const q16: u16 = @bitCast(@as(i16, @truncate(quotient)));
                const r16: u16 = @bitCast(@as(i16, @truncate(remainder)));
                ctx.cpu.d[dn] = (@as(u32, r16) << 16) | q16;
                ctx.cpu.sr.setNzvc(.{ .n = quotient < 0, .z = quotient == 0 });
            }
        }

        /// ORI/ANDI/SUBI/ADDI/EORI/CMPI: immediate always the source, `ea` the
        /// destination (Dn or memory), except CMPI which only reads `ea` and
        /// discards the result. `write=false` in `calcEa` matches the RMW
        /// convention `opAlu2` already uses: the initial read never recomputes
        /// the address, so its fault timing is the "read" shape even when a
        /// write follows.
        fn opImmediateAlu(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const mn = instr.mnemonic;
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);

            const imm = switch (size) {
                .byte => @as(u32, @as(u8, @truncate(try fetch16(ctx)))),
                .word => @as(u32, try fetch16(ctx)),
                .long => try fetch32(ctx),
            };

            // Charged word-sized either way; a faulting memory read never
            // reaches long's extra ALU pass or a write-back extra, added
            // below only once the read (if any) has actually succeeded.
            ctx.cpu.cycles += mode.cycles(size);
            errdefer if (size == .long) {
                ctx.cpu.cycles -= 4;
            };
            const operand = try RmwOperand.read(ctx, mode, reg, size);
            const old = operand.old;

            const kind: AluOp = switch (mn) {
                .ori => .or_,
                .andi => .and_,
                .eori => .eor,
                .addi => .add,
                .subi => .sub,
                .cmpi => .cmp,
                else => unreachable,
            };
            const r = aluCompute(kind, old, imm, size);

            if (mode == .data_reg) {
                // CMPI.l to Dn takes a smaller version of the usual long
                // bonus: it reads but never writes, so there's no write-back
                // extra to fold it into.
                if (size == .long) ctx.cpu.cycles += if (mn == .cmpi) @as(u8, 2) else 4;
            } else if (r.writes) {
                ctx.cpu.cycles += if (size == .long) @as(u8, 8) else 4;
            }

            if (r.writes) try operand.writeBack(ctx, size, r.value);
            if (r.affects_x) ctx.cpu.sr.setNzvcx(r.cc) else ctx.cpu.sr.setNzvc(r.cc);
        }

        /// ORI/ANDI/EORI to CCR/SR: fixed opcodes, immediate word follows.
        /// The `toSR` forms are privileged: a user-mode attempt faults before
        /// the immediate word is even fetched.
        fn opImmediateSr(ctx: *Ctx, instr: decode.Instr) Fault!void {
            const mn = instr.mnemonic;
            const is_sr = switch (mn) {
                .ori_sr, .andi_sr, .eori_sr => true,
                else => false,
            };

            if (is_sr and !ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }

            var imm = try fetch16(ctx);
            if (!is_sr) imm &= 0xFF;

            const old = ctx.cpu.sr.toInt();
            const result: u16 = switch (mn) {
                .ori_ccr, .ori_sr => old | imm,
                .andi_ccr, .andi_sr => old & imm,
                .eori_ccr, .eori_sr => old ^ imm,
                else => unreachable,
            };
            // CCR forms only ever touch the low byte; the upper byte (incl. S) carries over untouched.
            const v = if (is_sr) result else (old & 0xFF00) | (result & 0xFF);

            const new_sr = cpu_mod.StatusRegister.fromInt(v);
            if (is_sr) ctx.cpu.setSupervisor(new_sr.s);
            ctx.cpu.sr = new_sr;
        }

        /// A long read at a predecrement address used by ADDX/SUBX's memory
        /// form: faults low-word-first (addr+2), read bit set, same as the
        /// rest of that instruction's dual-predecrement bus ordering.
        fn predecLongRead(ctx: *Ctx, addr: u32) Fault!u32 {
            if (addr & 1 != 0) return addressError(ctx, addr +% 2, true, false);
            return read32(ctx, addr, false);
        }

        /// ADDX/SUBX: Dy,Dx (register-register) or -(Ay),-(Ax) (memory pair,
        /// bit 3 of the opcode set). Unlike ADD/SUB, the Z flag can only be
        /// cleared here, never set (`flags.addx`/`subx`'s `old_z` chaining),
        /// so multi-word extended arithmetic reads as zero only if every
        /// word in the chain was.
        fn opAddSubX(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const is_sub = instr.mnemonic == .subx;
            const rx: u3 = @truncate(op >> 9);
            const ry: u3 = @truncate(op);
            const mem_form = op & 0x0008 != 0;

            const old_z = ctx.cpu.sr.z;
            const x_in = ctx.cpu.sr.x;

            var dst_addr: u32 = undefined;
            var src: u32 = undefined;
            var dst: u32 = undefined;
            if (mem_form and size == .long) {
                // Long size only: each operand's predecrement commits only
                // if *that* operand's own access succeeds — unlike a lone
                // predecrement read elsewhere, which always commits before
                // the fault. Its bus cycle also goes low-word-first (fault
                // address is addr+2, matching a long *write*'s ordering)
                // even though it's only ever read -- and its fault frame's
                // PC follows the write-side prefetch rule too, for the same
                // reason.
                const src_addr = ctx.cpu.a[ry] -% stackAdjust(ry, size);
                markPrefetch(ctx, ctx.cpu.pc, .addr_predec, false);
                // `base_cycles` charges the full two-operand success total
                // up front; a fault on the source read never reaches the
                // destination access at all, so it must give back 20 of
                // that (the destination's own would-have-been EA/access
                // cost) once the flat exception overhead is added on top.
                src = predecLongRead(ctx, src_addr) catch |e| {
                    ctx.cpu.cycles -= 20;
                    return e;
                };
                ctx.cpu.a[ry] = src_addr;

                // Source already succeeded here, so only the smaller
                // remainder (12) needs unwinding: the difference between
                // "both operands" and "source operand" cost, not the whole
                // destination-side share source-fault gives back above.
                dst_addr = calcEa(ctx, .addr_predec, rx, size, true) catch |e| {
                    ctx.cpu.cycles -= 12;
                    ctx.fault.?.read = true;
                    return e;
                };
                dst = try readAt(ctx, dst_addr, size, false);
            } else if (mem_form) {
                const src_addr = try calcEa(ctx, .addr_predec, ry, size, false);
                // Byte accesses never fault (no alignment check), so these
                // adjustments are no-ops for `.byte`; only `.word` can reach
                // them. Same shape as the long case above: source-fault
                // gives back more (8) than a fault after the source already
                // succeeded (4).
                src = readAt(ctx, src_addr, size, false) catch |e| {
                    if (size == .word) ctx.cpu.cycles -= 8;
                    return e;
                };
                dst_addr = try calcEa(ctx, .addr_predec, rx, size, true);
                dst = readAt(ctx, dst_addr, size, false) catch |e| {
                    if (size == .word) ctx.cpu.cycles -= 4;
                    return e;
                };
            } else {
                src = ctx.cpu.d[ry] & size.mask();
                dst = ctx.cpu.d[rx] & size.mask();
            }

            const cc = if (is_sub) flags.subx(dst, src, x_in, size, old_z) else flags.addx(dst, src, x_in, size, old_z);
            const x = @intFromBool(x_in);
            const value = if (is_sub) (dst -% src -% x) & size.mask() else (dst +% src +% x) & size.mask();

            if (mem_form) {
                try writeAt(ctx, dst_addr, size, value);
            } else {
                setReg(&ctx.cpu.d[rx], size, value);
            }
            ctx.cpu.sr.setNzvcx(cc);
        }

        /// ABCD/SBCD: byte-only, register or -(Ay),-(Ax) pair form — same
        /// shape ADDX/SUBX use, minus the long-size predecrement-pair special
        /// case above since these never go past byte.
        fn opBcd(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const rx: u3 = @truncate(op >> 9);
            const ry: u3 = @truncate(op);
            const mem_form = op & 0x0008 != 0;

            const old_z = ctx.cpu.sr.z;
            const x_in = ctx.cpu.sr.x;

            var dst_addr: u32 = undefined;
            var src: u8 = undefined;
            var dst: u8 = undefined;
            if (mem_form) {
                const src_addr = try calcEa(ctx, .addr_predec, ry, .byte, false);
                src = @truncate(try readAt(ctx, src_addr, .byte, false));
                dst_addr = try calcEa(ctx, .addr_predec, rx, .byte, true);
                dst = @truncate(try readAt(ctx, dst_addr, .byte, false));
            } else {
                src = @truncate(ctx.cpu.d[ry]);
                dst = @truncate(ctx.cpu.d[rx]);
            }

            const r = if (instr.mnemonic == .abcd)
                flags.bcdAdd(dst, src, x_in, old_z)
            else
                flags.bcdSub(dst, src, x_in, old_z);

            if (mem_form) {
                try writeAt(ctx, dst_addr, .byte, r.value);
            } else {
                setReg(&ctx.cpu.d[rx], .byte, r.value);
            }
            ctx.cpu.sr.setNzvcx(r.cc);
        }

        /// EXG: swaps two full 32-bit registers, no flags affected. The
        /// 5-bit mode field (bits 7-3) picks which register file each side
        /// comes from; Dx,Ay swaps across the D/A files, the other two forms
        /// stay within one.
        fn opExg(ctx: *Ctx, op: u16) void {
            const rx: u3 = @truncate(op >> 9);
            const ry: u3 = @truncate(op);
            const exg_mode: u5 = @truncate(op >> 3);
            const px = switch (exg_mode) {
                0b01001 => &ctx.cpu.a[rx],
                else => &ctx.cpu.d[rx],
            };
            const py = switch (exg_mode) {
                0b01000 => &ctx.cpu.d[ry],
                else => &ctx.cpu.a[ry],
            };
            const tmp = px.*;
            px.* = py.*;
            py.* = tmp;
        }

        /// CMPM: (Ay)+,(Ax)+ — both operands postincrement independently, no
        /// writeback, source read before destination. The destination read
        /// passes `write=true` to `calcEa` even though nothing is written
        /// back: empirically, a misaligned destination access leaves Ax
        /// un-incremented (fault detected before the register updates),
        /// unlike the source, which increments unconditionally before its
        /// own bus cycle. `write=true` also flips the fault frame's R/W bit
        /// to "write", which is wrong here — it's a read — so it's patched
        /// back after the fact if a fault was raised.
        ///
        /// Long size only, the source read commits a partial +2 (not the
        /// full +4, not 0) when its own address is misaligned: empirically
        /// the first word's postincrement lands before the second word's
        /// access faults.
        fn opCmpm(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const ax: u3 = @truncate(op >> 9);
            const ay: u3 = @truncate(op);

            // Each (An)+ read can independently fault; charge its own EA
            // cost here and unwind to the word-equivalent (-4 for long) only
            // on that read's own fault, since a fault on the second read
            // must leave the first (successful) read's full cost charged.
            ctx.cpu.cycles += EaMode.addr_postinc.cycles(size);
            const src = if (size == .long) blk: {
                const addr = ctx.cpu.a[ay];
                if (addr & 1 != 0) {
                    ctx.cpu.cycles -= 4;
                    ctx.cpu.a[ay] +%= 2;
                    // Same bus-timing quirk as ADDX/SUBX's long predecrement
                    // pair: this read's fault frame follows the write-side
                    // prefetch rule, not a plain read's.
                    markPrefetch(ctx, ctx.cpu.pc, .addr_postinc, false);
                    return addressError(ctx, addr, true, false);
                }
                ctx.cpu.a[ay] +%= 4;
                break :blk try read32(ctx, addr, false);
            } else blk: {
                const src_addr = try calcEa(ctx, .addr_postinc, ay, size, false);
                // Same write-side prefetch timing as the long case above,
                // even though this word/byte fault surfaces later, in the
                // actual bus access rather than inside `calcEa` itself.
                markPrefetch(ctx, ctx.cpu.pc, .addr_postinc, false);
                break :blk try readAt(ctx, src_addr, size, false);
            };
            ctx.cpu.cycles += EaMode.addr_postinc.cycles(size);
            const dst_addr = calcEa(ctx, .addr_postinc, ax, size, true) catch |e| {
                if (size == .long) ctx.cpu.cycles -= 4;
                ctx.fault.?.read = true;
                return e;
            };
            const dst = try readAt(ctx, dst_addr, size, false);

            ctx.cpu.sr.setNzvc(flags.sub(dst, src, size));
        }

        fn shiftKind(mn: decode.Mnemonic) flags.ShiftKind {
            return switch (mn) {
                .asl => .asl,
                .asr => .asr,
                .lsl => .lsl,
                .lsr => .lsr,
                .rol => .rol,
                .ror => .ror,
                .roxl => .roxl,
                .roxr => .roxr,
                else => unreachable,
            };
        }

        /// ASx/LSx/ROx/ROXx. The register form's i/r bit (5) picks whether
        /// the count is the encoded immediate (1-8, 0 means 8) or a
        /// register's value (mod 64, free via truncation to u6). The memory
        /// form always shifts exactly one bit and has no count field.
        fn opShift(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const kind = shiftKind(instr.mnemonic);
            const x_in = ctx.cpu.sr.x;

            const is_memory = (op >> 6) & 0b11 == 0b11;
            if (is_memory) {
                const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
                const reg: u3 = @truncate(op);
                // Full EA cost, not the predecrement-discounted `destCycles`:
                // the read and the write-back share one address, so a
                // faulting read is the only fault point, and it always
                // gives back a flat 4 once the exception overhead lands.
                ctx.cpu.cycles += mode.cycles(.word);
                const addr = try calcEa(ctx, mode, reg, .word, false);
                const old = readAt(ctx, addr, .word, false) catch |e| {
                    ctx.cpu.cycles -= 4;
                    return e;
                };
                const r = flags.shift(kind, old, 1, .word, x_in);
                try writeAt(ctx, addr, .word, r.value);
                ctx.cpu.sr.setNzvc(.{ .n = r.n, .z = r.z, .v = r.v, .c = r.c });
                if (r.x) |x| ctx.cpu.sr.x = x;
                return;
            }

            const size = instr.size;
            const dn: u3 = @truncate(op);
            const count: u6 = if (op & 0x0020 != 0) blk: {
                const creg: u3 = @truncate(op >> 9);
                break :blk @truncate(ctx.cpu.d[creg]);
            } else blk: {
                const field: u3 = @truncate(op >> 9);
                break :blk if (field == 0) 8 else field;
            };
            ctx.cpu.cycles += @as(u64, count) * 2;

            const old = ctx.cpu.d[dn] & size.mask();
            const r = flags.shift(kind, old, count, size, x_in);
            setReg(&ctx.cpu.d[dn], size, r.value);
            ctx.cpu.sr.setNzvc(.{ .n = r.n, .z = r.z, .v = r.v, .c = r.c });
            if (r.x) |x| ctx.cpu.sr.x = x;
        }

        /// BTST/BCHG/BCLR/BSET. The bit number is a runtime value either way
        /// (an extension word for the static form, Dn for the dynamic one),
        /// so decode couldn't split them; bit 8 does it here too.
        fn opBitOp(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);

            const bit_num: u32 = if (op & 0x0100 == 0)
                try fetch16(ctx) & 0xFF
            else
                ctx.cpu.d[@as(u3, @truncate(op >> 9))];

            // Dn: bit# mod 32, tests/writes the whole register.
            if (mode == .data_reg) {
                const bit: u5 = @truncate(bit_num);
                // Hardware quirk: touching the upper word of the register
                // costs 2 more cycles than the lower one — BCHG/BCLR/BSET
                // only (BTST never writes back, so it skips the extra access).
                if (instr.mnemonic != .btst and bit >= 16) ctx.cpu.cycles += 2;
                const mask: u32 = @as(u32, 1) << bit;
                const old = ctx.cpu.d[reg];
                ctx.cpu.sr.z = old & mask == 0;
                const v: u32 = switch (instr.mnemonic) {
                    .btst => old,
                    .bchg => old ^ mask,
                    .bclr => old & ~mask,
                    .bset => old | mask,
                    else => unreachable,
                };
                if (instr.mnemonic != .btst) setReg(&ctx.cpu.d[reg], .long, v);
                return;
            }

            // Memory: bit# mod 8, always a single byte. Only BTST's ea class
            // allows immediate, which (unlike every other mode) has no
            // address to compute — read the ext word directly instead.
            const bit: u3 = @truncate(bit_num);
            const mask: u8 = @as(u8, 1) << bit;
            if (mode == .immediate) {
                // BTST's one genuinely odd case: testing a bit in a literal
                // costs 2 more than an ordinary byte immediate fetch
                // (confirmed against SST cycle data).
                ctx.cpu.cycles += mode.cycles(.byte) + 2;
                const old: u8 = @truncate(try fetch16(ctx));
                ctx.cpu.sr.z = old & mask == 0;
                return;
            }

            // Bit ops' ea timing table matches the general (source) cost, not
            // the RMW destination one: -(An) still pays the full 6, unlike
            // CLR/NEG/NOT next door (confirmed against SST cycle data).
            ctx.cpu.cycles += mode.cycles(.byte);
            const addr = try calcEa(ctx, mode, reg, .byte, false);
            const old: u8 = @truncate(try readAt(ctx, addr, .byte, mode.isProgram()));
            ctx.cpu.sr.z = old & mask == 0;
            if (instr.mnemonic == .btst) return;
            const v: u8 = switch (instr.mnemonic) {
                .bchg => old ^ mask,
                .bclr => old & ~mask,
                .bset => old | mask,
                else => unreachable,
            };
            try writeAt(ctx, addr, .byte, v);
        }

        /// MOVEP: alternates bytes of Dn (high first) with word-spaced bytes
        /// of memory starting at d16(An). Bit 7 picks direction; size lives
        /// in `instr.size`.
        fn opMovep(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const dn: u3 = @truncate(op >> 9);
            const an: u3 = @truncate(op);
            const to_mem = op & 0x0080 != 0;
            const d: i16 = @bitCast(try fetch16(ctx));
            const base = ctx.cpu.a[an] +% @as(u32, @bitCast(@as(i32, d)));

            const n: u32 = if (instr.size == .long) 4 else 2;
            if (to_mem) {
                var i: u32 = 0;
                while (i < n) : (i += 1) {
                    const shift: u5 = @intCast((n - 1 - i) * 8);
                    write8(ctx, base +% i * 2, @truncate(ctx.cpu.d[dn] >> shift));
                }
            } else {
                var v: u32 = 0;
                var i: u32 = 0;
                while (i < n) : (i += 1) v = (v << 8) | read8(ctx, base +% i * 2);
                setReg(&ctx.cpu.d[dn], instr.size, v);
            }
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

        fn opSwap(ctx: *Ctx, op: u16) void {
            const reg: u3 = @truncate(op);
            const v = ctx.cpu.d[reg];
            const swapped = (v << 16) | (v >> 16);
            ctx.cpu.d[reg] = swapped;
            ctx.cpu.sr.setNzvc(flags.logic(swapped, .long));
        }

        /// EXT.w sign extends the low byte into the low word (upper word
        /// untouched); EXT.l sign extends the low word across the whole
        /// register.
        fn opExt(ctx: *Ctx, op: u16, instr: decode.Instr) void {
            const reg: u3 = @truncate(op);
            const v = if (instr.size == .word)
                signExtend(ctx.cpu.d[reg] & 0xFF, .byte)
            else
                signExtend(ctx.cpu.d[reg] & 0xFFFF, .word);
            setReg(&ctx.cpu.d[reg], instr.size, v);
            ctx.cpu.sr.setNzvc(flags.logic(v, instr.size));
        }

        /// PEA: like `opLea` but pushes the computed address rather than
        /// loading it into an address register.
        fn opPea(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const addr = try calcEa(ctx, mode, @truncate(op), .long, false);
            try push32(ctx, addr);
        }

        /// LINK An,#d16: push An, An := SP, SP += d16. Pushing first and
        /// reading `a[7]` as the value before `push32` decrements it is what
        /// makes `LINK A7` self-consistent with no special case: the pushed
        /// value is the old SP, and "An := SP" for An=A7 is then a no-op on
        /// top of the push, matching the documented hardware quirk.
        fn opLink(ctx: *Ctx, op: u16) Fault!void {
            const reg: u3 = @truncate(op);
            const disp: i16 = @bitCast(try fetch16(ctx));
            try push32(ctx, ctx.cpu.a[reg]);
            ctx.cpu.a[reg] = ctx.cpu.a[7];
            ctx.cpu.a[7] +%= @as(u32, @bitCast(@as(i32, disp)));
        }

        /// UNLK An: SP := An, An := (SP)+. Reading through `An` rather than
        /// through `a[7]` (i.e. not committing "SP := An" up front) matters
        /// only when the read faults: on a real address error the stacked
        /// frame is built from the *original* SP, not one already clobbered
        /// with the (possibly odd) An value, so both writes must land
        /// together only once the read has actually succeeded. That also
        /// makes `UNLK A7` fall out for free: reg==7 means the final
        /// `a[reg] = v` assignment overwrites the `a[7] = sp +% 4` one, so A7
        /// ends up holding the popped value, matching hardware.
        fn opUnlk(ctx: *Ctx, op: u16) Fault!void {
            const reg: u3 = @truncate(op);
            const sp = ctx.cpu.a[reg];
            // Empirically follows the write-side prefetch timing (see
            // `markPrefetch`), not a plain read's, despite reading An's
            // contents rather than writing anything.
            markPrefetch(ctx, ctx.cpu.pc, .addr_ind, false);
            // A faulting read gives back a flat 4 once the automatic
            // exception overhead lands.
            const v = read32(ctx, sp, false) catch |e| {
                ctx.cpu.cycles -= 4;
                return e;
            };
            ctx.cpu.a[7] = sp +% 4;
            ctx.cpu.a[reg] = v;
        }

        /// Jump before push: on real hardware an odd target faults with no
        /// return address ever pushed, so the target has to be committed
        /// (and validated) before the stack is touched.
        fn opJsr(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const addr = try calcEa(ctx, mode, @truncate(op), .long, false);
            // Unlike a data read (see `markReadFault`), JSR's EA computation
            // never reads the target as data, and empirically always shows
            // the fully-advanced post-EA PC in the fault frame, regardless
            // of addressing mode -- so this overrides `calcEa`'s baked-in
            // mark uniformly rather than following its mode split.
            ctx.prefetch = .{ .ir = null, .pc = ctx.cpu.pc };
            const ret = ctx.cpu.pc;
            // An odd target faults for the same total as JMP's own fault at
            // that EA (nothing has been pushed yet to distinguish them), so
            // JSR's extra base cost (the not-yet-attempted push) has to be
            // given back here once the exception overhead lands.
            jump(ctx, addr) catch |e| {
                ctx.cpu.cycles -= 8;
                return e;
            };
            try push32(ctx, ret);
        }

        fn opJmp(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const pc_before = ctx.cpu.pc;
            const addr = try calcEa(ctx, mode, @truncate(op), .long, false);
            // JMP is JSR's mirror: always rolls back to the pre-EA PC,
            // regardless of addressing mode.
            ctx.prefetch = .{ .ir = null, .pc = pc_before };
            try jump(ctx, addr);
        }

        /// Real hardware does a dummy read of the destination before writing
        /// SR there (well-documented MOVE-from-SR quirk), so an odd address
        /// faults as a read, not a write: matters for the stacked SSW.
        fn opMoveFromSr(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const v: u32 = ctx.cpu.sr.toInt();
            if (mode == .data_reg) {
                setReg(&ctx.cpu.d[reg], .word, v);
            } else {
                // Full EA cost, not the predecrement-discounted `destCycles`:
                // the dummy read below makes this destination behave like a
                // read, not a plain write (same shape as `opMoveToCcr`).
                ctx.cpu.cycles += mode.cycles(.word);
                // `write=false`: the dummy read makes this destination
                // behave like a read for postinc/predec's fault-vs-register
                // ordering, matching the bus cycle that actually faults.
                const addr = try calcEa(ctx, mode, reg, .word, false);
                // A faulting read gives back a flat 4, the same for every EA
                // kind, once the automatic exception overhead lands (same
                // shape as `opMoveToCcr`).
                _ = read16(ctx, addr, false) catch |e| {
                    ctx.cpu.cycles -= 4;
                    return e;
                };
                try writeAt(ctx, addr, .word, v);
            }
        }

        fn opMoveToCcr(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            ctx.cpu.cycles += mode.cycles(.word);
            // A faulting read gives back a flat 8, the same for every EA
            // kind, once the automatic exception overhead lands (same shape
            // as `opMoveToSr`).
            const v = readEa(ctx, mode, reg, .word) catch |e| {
                ctx.cpu.cycles -= 8;
                return e;
            };
            const full = (ctx.cpu.sr.toInt() & 0xFF00) | (v & 0xFF);
            ctx.cpu.sr = cpu_mod.StatusRegister.fromInt(@truncate(full));
        }

        /// Privileged: a user-mode attempt faults before the source operand
        /// is even read, and before any of this instruction's own cost is
        /// charged — dispatch's flat `base_cycles` add happens unconditionally
        /// ahead of the handler, so a privilege violation has to give it back.
        fn opMoveToSr(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            if (!ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            ctx.cpu.cycles += mode.cycles(.word);
            // A faulting read gives back a flat 8, the same for every EA
            // kind, once the automatic exception overhead lands.
            const v = readEa(ctx, mode, reg, .word) catch |e| {
                ctx.cpu.cycles -= 8;
                return e;
            };
            const new_sr = cpu_mod.StatusRegister.fromInt(@truncate(v));
            ctx.cpu.setSupervisor(new_sr.s);
            ctx.cpu.sr = new_sr;
        }

        /// MOVE USP: bit 3 of the opcode picks direction, since `Instr` has
        /// no direction field (same convention as MOVEP). Same privilege-fault
        /// cycle giveback as `opMoveToSr`.
        fn opMoveUsp(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            if (!ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }
            const reg: u3 = @truncate(op);
            if (op & 0x0008 != 0) {
                ctx.cpu.a[reg] = ctx.cpu.usp;
            } else {
                ctx.cpu.usp = ctx.cpu.a[reg];
            }
        }

        fn opTrap(ctx: *Ctx, op: u16) Fault!void {
            const n: u8 = @truncate(op & 0xF);
            ctx.trap_vector = @enumFromInt(@intFromEnum(Exception.trap_0) + n);
            return Fault.Trap;
        }

        fn opTrapv(ctx: *Ctx) Fault!void {
            if (!ctx.cpu.sr.v) return;
            ctx.trap_vector = .trapv;
            return Fault.Trap;
        }

        /// CHK: faults if Dn (as a signed word) is negative or exceeds the
        /// ea bound. The documented "N set if Dn<0, cleared if Dn>bound" is
        /// just what a
        /// plain `TST Dn` already gives: real hardware sets NZVC from Dn
        /// alone (V/C always clear, same as `flags.logic`), not from a
        /// comparison against the bound.
        fn opChk(ctx: *Ctx, op: u16) Fault!void {
            const dn: u3 = @truncate(op >> 9);
            const ea_mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const ea_reg: u3 = @truncate(op);
            ctx.cpu.cycles += ea_mode.cycles(.word);
            // A faulting read gives back a flat 6, the same for every EA
            // kind, once the automatic exception overhead lands.
            const bound: i16 = @bitCast(@as(u16, @truncate(readEa(ctx, ea_mode, ea_reg, .word) catch |e| {
                ctx.cpu.cycles -= 6;
                return e;
            })));
            const v: i16 = @bitCast(@as(u16, @truncate(ctx.cpu.d[dn])));
            ctx.cpu.sr.setNzvc(flags.logic(ctx.cpu.d[dn], .word));
            if (v < 0 or v > bound) {
                // The trap itself is decided by the correct signed compare
                // above, but its cost isn't: hardware re-derives "greater
                // than" from a raw 16-bit `Dn - bound` ALU op and takes the
                // fast exit whenever that raw subtract overflows OR comes
                // out strictly positive — even when the *true* signed
                // result disagrees (Dn very negative, bound very positive
                // wraps the 16-bit subtract back to a false-positive
                // "greater"). Every other combination takes the slow exit,
                // 2 cycles more.
                const dn_bits: u16 = @bitCast(v);
                const bound_bits: u16 = @bitCast(bound);
                const raw = dn_bits -% bound_bits;
                const n = raw & 0x8000 != 0;
                const z = raw == 0;
                const overflow = (dn_bits ^ bound_bits) & 0x8000 != 0 and (dn_bits ^ raw) & 0x8000 != 0;
                const fast = overflow or (!n and !z);
                ctx.chk_delta = if (fast) 12 else 10;
                return Fault.Chk;
            }
        }

        /// STOP freezes the bus: no further prefetch happens, so unlike every
        /// other instruction the PC does not settle 4 bytes past where it
        /// started (the harness's uniform `pc_prefetch_offset` correction
        /// assumes prefetch resumes at the new PC, which here it never
        /// does). Rewinding by 4 undoes both this fetch and the opcode
        /// fetch `execute` already did, landing PC back on STOP itself.
        fn opStop(ctx: *Ctx, instr: decode.Instr) Fault!void {
            if (!ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }
            const v = try fetch16(ctx);
            const new_sr = cpu_mod.StatusRegister.fromInt(v);
            ctx.cpu.setSupervisor(new_sr.s);
            ctx.cpu.sr = new_sr;
            ctx.cpu.stopped = true;
            ctx.cpu.pc -%= 4;
        }

        /// RESET pulses the reset line for external devices. This core has
        /// none to reset, so it is host-facing only: privileged, and costs
        /// cycles, but has no further effect (real hardware doesn't reset
        /// itself either).
        fn opReset(ctx: *Ctx, instr: decode.Instr) Fault!void {
            if (!ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }
        }

        /// RTE: pops SR then PC (word then long, matching the frame `frame`
        /// builds). Only the base 68000's format-less frame exists here, so
        /// this is a plain pop with no format-word dispatch. `jump`, not a
        /// plain assignment, for the same reason RTS/JMP/JSR use it: the
        /// implied prefetch at the restored PC happens immediately, so an
        /// odd popped PC faults within this instruction, not the next.
        fn opRte(ctx: *Ctx, instr: decode.Instr) Fault!void {
            if (!ctx.cpu.sr.s) {
                ctx.cpu.cycles -= instr.base_cycles;
                return Fault.PrivilegeViolation;
            }
            const sr_word = try pop16(ctx);
            const pc = try pop32(ctx);
            const new_sr = cpu_mod.StatusRegister.fromInt(sr_word);
            ctx.cpu.setSupervisor(new_sr.s);
            ctx.cpu.sr = new_sr;
            // Like RTS, the fault frame shows RTE's own pre-jump PC, not
            // the popped (bad) target.
            ctx.prefetch = .{ .ir = null, .pc = ctx.cpu.pc };
            try jump(ctx, pc);
        }

        /// RTR: pops CCR then PC. Unlike RTE, only the CCR byte is restored
        /// (the upper SR byte, and supervisor mode, are untouched), so it
        /// needs no privilege check. `jump`, as in `opRte`, for the implied
        /// prefetch at the restored PC.
        fn opRtr(ctx: *Ctx) Fault!void {
            const ccr_word = try pop16(ctx);
            const pc = try pop32(ctx);
            const full = (ctx.cpu.sr.toInt() & 0xFF00) | (ccr_word & 0xFF);
            ctx.cpu.sr = cpu_mod.StatusRegister.fromInt(full);
            // Same as RTE/RTS: fault frame shows the pre-jump PC.
            ctx.prefetch = .{ .ir = null, .pc = ctx.cpu.pc };
            try jump(ctx, pc);
        }

        /// TAS: byte read-modify-write, same address-once shape as
        /// `opAluSingle`'s RMW cases. Tests the old value into N/Z (clearing
        /// V/C), then unconditionally sets bit 7 on write-back.
        fn opTas(ctx: *Ctx, op: u16) Fault!void {
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const operand = try RmwOperand.read(ctx, mode, reg, .byte);
            const old: u8 = @truncate(operand.old);
            ctx.cpu.sr.setNzvc(flags.logic(old, .byte));
            const new_v = old | 0x80;
            try operand.writeBack(ctx, .byte, new_v);
        }

        /// MOVEM: register list <-> memory. Every mode but predecrement maps
        /// mask bit i (0-indexed from the LSB) to register i (0-7 = D0-D7,
        /// 8-15 = A0-A7) and walks addresses ascending. Predecrement
        /// (store only) reverses the mapping (bit i -> register 15-i, so
        /// bit 0 = A7) while still walking the mask LSB-first, which
        /// produces descending register order over descending addresses —
        /// landing on the same D0-lowest memory layout the other modes read
        /// back with their own bit order.
        fn opMovem(ctx: *Ctx, op: u16, instr: decode.Instr) Fault!void {
            const size = instr.size;
            const load = op & 0x0400 != 0;
            const mode = EaMode.decode(@truncate(op >> 3), @truncate(op)).?;
            const reg: u3 = @truncate(op);
            const mask = try fetch16(ctx);
            const predec = mode == .addr_predec;
            const postinc = mode == .addr_postinc;
            const orig_an = ctx.cpu.a[reg];

            // A misaligned EA always faults on the very first register
            // transfer attempted (stepping by an even amount never changes
            // parity), so store's fault total is a flat 4 more than its
            // base — load's already matches with no adjustment.
            errdefer if (!load) {
                ctx.cpu.cycles += 4;
            };

            // Non-indexing modes compute the base address once; predec/postinc
            // instead step it once *per register* via `movemStep`, since each
            // transfer is its own bus cycle with its own predecrement/
            // postincrement. Unlike a single-operand instruction's (An)/-(An)
            // (see `calcEa`), MOVEM always checks alignment before committing
            // the register update for every combination, including word
            // reads - there's no "increments before the bus cycle" quirk here.
            var addr: u32 = if (predec or postinc) 0 else try calcEa(ctx, mode, reg, size, false);

            // Every register transfer is its own bus cycle, so a fault on
            // any of them (not just the first) needs marking -- but
            // empirically they all follow the same write-side-style flat
            // bonus (see `markPrefetch`) over the PC as it sits once the
            // mask word and any EA extension words are already fetched,
            // regardless of which register in the list actually faults.
            const pc_after_setup = ctx.cpu.pc;
            markPrefetch(ctx, pc_after_setup, .addr_ind, false);

            var count: u32 = 0;
            var i: u5 = 0;
            while (i < 16) : (i += 1) {
                if (mask & (@as(u16, 1) << @truncate(i)) == 0) continue;
                const which: u5 = if (predec) 15 - i else i;
                const rn: u3 = @truncate(which);
                const is_addr = which >= 8;
                count += 1;

                const cur = if (predec or postinc) try movemStep(ctx, reg, size, predec, load) else blk: {
                    const a = addr;
                    addr +%= size.bytes();
                    break :blk a;
                };

                if (load) {
                    const v = signExtend(try readAt(ctx, cur, size, mode.isProgram()), size);
                    // (An)+ loading An itself: the auto-incremented address
                    // movemStep already wrote into An wins over the value
                    // just read from memory, unlike the reverse (store) case.
                    const skip_writeback = is_addr and rn == reg and postinc;
                    if (!skip_writeback) {
                        if (is_addr) ctx.cpu.a[rn] = v else setReg(&ctx.cpu.d[rn], .long, v);
                    }
                } else {
                    // -(An) storing An itself: real hardware stores the
                    // register's value from *before* the instruction
                    // started, not the (partially or fully) decremented
                    // address, despite An having already been decremented
                    // by the time its own slot is written.
                    const v = if (is_addr and rn == reg and predec)
                        orig_an
                    else if (is_addr) ctx.cpu.a[rn] else ctx.cpu.d[rn];
                    try writeAt(ctx, cur, size, v);
                }
            }

            ctx.cpu.cycles += @as(u64, count) * (if (size == .long) @as(u64, 8) else 4);
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
                .bra => {
                    // Unlike BSR (which also pushes a return address), a
                    // plain branch's fault frame shows its own pre-branch
                    // PC, not the bad target -- as if the branch had never
                    // been attempted.
                    ctx.prefetch = .{ .ir = null, .pc = base };
                    try jump(ctx, target);
                },
                .bsr => {
                    // The return address is written before the new PC is
                    // prefetched, so an odd target still leaves it on the stack.
                    try push32(ctx, c.pc);
                    try jump(ctx, target);
                },
                .bcc => {
                    if (flags.testCondition(c.sr, @truncate(op >> 8))) {
                        c.cycles += 2; // taken is 10, base covers the not-taken 8
                        ctx.prefetch = .{ .ir = null, .pc = base };
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
            // The PC is loaded first and the prefetch from it is what
            // faults, so by default the address error frame stacks the
            // target, not the return address -- true for BSR/JMP/JSR's
            // successful-EA case. Callers where that's empirically wrong
            // (Bcc/DBcc, RTS, RTE, RTR -- the frame shows their own
            // pre-jump PC instead, as if the jump had never been
            // attempted) override `ctx.prefetch` themselves before calling
            // this, which takes priority in the fault path.
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

test "neg.w -(a0) negates memory, decrements first, and sets X" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x4460); // neg.w -(a0)
    bus.write16(0x0FFE, 0x1234);
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1000;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x0FFE), c.a[0]);
    try std.testing.expectEqual(@as(u16, 0xEDCC), bus.read16(0x0FFE));
    try std.testing.expect(c.sr.x and c.sr.c and !c.sr.z);
}

test "clr.l d0 zeroes the register and leaves X alone" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x4280); // clr.l d0
    var c = Cpu{ .pc = 0x400 };
    c.d[0] = 0xDEAD_BEEF;
    c.sr.x = true;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0), c.d[0]);
    try std.testing.expect(c.sr.z and !c.sr.n and !c.sr.v and !c.sr.c and c.sr.x);
}

test "tst.b does not write back" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x4A10); // tst.b (a0)
    bus.write8(0x1000, 0x80);
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1000;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u8, 0x80), bus.read8(0x1000));
    try std.testing.expect(c.sr.n and !c.sr.z);
}

test "addq/subq: An destination is a plain long add with no flags" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x5548); // subq.w #2,a0
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x1000;
    c.sr.z = true; // untouched: An destination never affects flags

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x0FFE), c.a[0]);
    try std.testing.expect(c.sr.z);
}

test "addq: Dn destination sets flags normally" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x5A00); // addq.b #5,d0 (field 101 = 5)
    var c = Cpu{ .pc = 0x400 };
    c.d[0] = 0xFF; // -1 as a byte

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x04), c.d[0]);
    try std.testing.expect(c.sr.c and c.sr.x and !c.sr.z);
}

test "scc writes all-ones or all-zero based on the condition" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x57C0); // seq d0
    var c = Cpu{ .pc = 0x400 };
    c.sr.z = true;
    c.d[0] = 0xAAAA_AAAA;

    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0xAAAA_AAFF), c.d[0]);
}

test "dbcc loops until the counter expires, then falls through" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x51C8); // dbf d0, *-2 (branch back to itself)
    bus.write16(0x402, 0xFFFE); // displacement -2
    var c = Cpu{ .pc = 0x400 };
    c.d[0] = 1;

    TestCore.step(&c, &bus); // count 1 -> 0, branches back
    try std.testing.expectEqual(@as(u32, 0), c.d[0]);
    try std.testing.expectEqual(@as(u32, 0x400), c.pc);

    TestCore.step(&c, &bus); // count 0 -> -1, expires, falls through
    try std.testing.expectEqual(@as(u32, 0xFFFF), c.d[0]);
    try std.testing.expectEqual(@as(u32, 0x404), c.pc);
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

test "trace fires one instruction after T is sampled, outranking interrupts" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x26, 0x3000); // trace vector (9 * 4 = 0x24)
    bus.write16(0x400, 0x4E71); // nop
    var c = Cpu{ .pc = 0x400, .sr = .{ .s = true, .t = true } };
    c.a[7] = 0x1800;

    // The traced instruction still executes and advances PC normally.
    TestCore.step(&c, &bus);
    try std.testing.expectEqual(@as(u32, 0x402), c.pc);
    try std.testing.expect(c.trace_pending);

    // A pending interrupt does not preempt the trace that was sampled first.
    TestCore.setIpl(&c, 7);
    TestCore.step(&c, &bus);

    try std.testing.expectEqual(@as(u32, 0x3000), c.pc);
    try std.testing.expect(!c.sr.t);
    try std.testing.expect(!c.trace_pending);
    try std.testing.expectEqual(@as(u32, 0x1800 - 6), c.a[7]); // 3-word frame
    try std.testing.expectEqual(@as(u16, 0x402), bus.read16(0x1800 - 2)); // saved pc low
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

test "ori to ccr sets flags from an immediate, leaving the upper byte alone" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x400, 0x003C); // ori #imm,ccr
    bus.write16(0x402, 0x0014); // Z (bit2) | X (bit4)
    var c = Cpu{ .pc = 0x400 };
    c.sr.s = true; // supervisor, so the frame path in setSupervisor isn't hit
    c.sr.n = true;

    TestCore.step(&c, &bus);

    try std.testing.expect(c.sr.z and c.sr.x and c.sr.n);
    try std.testing.expect(c.sr.s); // untouched: ORI to CCR never affects the upper byte
}

test "addx.l -(Ay),-(Ax): a misaligned source leaves both operands uncommitted" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000);

    bus.write16(0x400, 0xD189); // addx.l -(a1),-(a0)
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x2000;
    c.a[1] = 0x1001; // odd: -4 lands on 0x0FFD
    c.a[7] = 0x1800;

    TestCore.step(&c, &bus);

    try std.testing.expect(c.sr.s); // address error taken
    try std.testing.expectEqual(@as(u32, 0x1001), c.a[1]); // source never committed
    try std.testing.expectEqual(@as(u32, 0x2000), c.a[0]); // destination never reached
}

test "addx.l -(Ay),-(Ax): a misaligned destination commits the source but not itself" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000);

    bus.write16(0x400, 0xD189); // addx.l -(a1),-(a0)
    var c = Cpu{ .pc = 0x400 };
    c.a[0] = 0x2001; // odd: -4 lands on 0x1FFD
    c.a[1] = 0x1004; // aligned, reads cleanly from zeroed ram
    c.a[7] = 0x1800;

    TestCore.step(&c, &bus);

    try std.testing.expect(c.sr.s);
    try std.testing.expectEqual(@as(u32, 0x1000), c.a[1]); // source's own access succeeded
    try std.testing.expectEqual(@as(u32, 0x2001), c.a[0]); // destination never commits on fault
}

test "cmpm.l (Ay)+,(Ax)+: a misaligned destination leaves it uncommitted after the source already advanced" {
    var bus = try testBus(std.testing.allocator);
    defer std.testing.allocator.free(bus.ram);

    bus.write16(0x0C, 0x0000);
    bus.write16(0x0E, 0x2000);

    bus.write16(0x400, 0xB38C); // cmpm.l (a4)+,(a1)+
    var c = Cpu{ .pc = 0x400 };
    c.a[4] = 0x1000; // aligned source
    c.a[1] = 0x2001; // odd destination
    c.a[7] = 0x1800;

    TestCore.step(&c, &bus);

    try std.testing.expect(c.sr.s);
    try std.testing.expectEqual(@as(u32, 0x1004), c.a[4]); // source postincrements unconditionally
    try std.testing.expectEqual(@as(u32, 0x2001), c.a[1]); // destination never commits on fault
}
