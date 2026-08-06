//! CPU data objects. Pure data — no logic beyond trivial accessors that only
//! reinterpret the bits already here. Execution lives in `core.zig`.

const std = @import("std");

/// Condition code register, the low byte of `StatusRegister`.
pub const Ccr = struct {
    c: bool = false,
    v: bool = false,
    z: bool = false,
    n: bool = false,
    x: bool = false,
};

/// MC68000 status register. Field order is LSB-first, matching the hardware
/// layout, so `@bitCast` is the whole of MOVE to/from SR and the exception
/// frame encoding.
pub const StatusRegister = packed struct(u16) {
    c: bool = false, // 0
    v: bool = false, // 1
    z: bool = false, // 2
    n: bool = false, // 3
    x: bool = false, // 4
    _pad0: u3 = 0, // 5..7
    ipl: u3 = 7, // 8..10  interrupt priority mask
    _pad1: u2 = 0, // 11..12
    s: bool = true, // 13     supervisor
    _pad2: u1 = 0, // 14
    t: bool = false, // 15     trace

    pub fn toInt(sr: StatusRegister) u16 {
        return @bitCast(sr);
    }

    /// Unused bits read back as zero on a real 68000, so they are dropped here
    /// rather than at every call site that writes SR.
    pub fn fromInt(v: u16) StatusRegister {
        var sr: StatusRegister = @bitCast(v);
        sr._pad0 = 0;
        sr._pad1 = 0;
        sr._pad2 = 0;
        return sr;
    }

    /// Writes N, Z, V and C but leaves X alone. Moves and logic operations do
    /// not affect X.
    pub fn setNzvc(sr: *StatusRegister, v: Ccr) void {
        sr.c = v.c;
        sr.v = v.v;
        sr.z = v.z;
        sr.n = v.n;
    }

    /// Like `setNzvc` but also writes X. The arithmetic ops (ADD, SUB, NEG,
    /// and the X-chain forms) affect X; MOVE and the logic ops do not.
    pub fn setNzvcx(sr: *StatusRegister, v: Ccr) void {
        sr.setNzvc(v);
        sr.x = v.x;
    }
};

pub const Size = enum(u2) {
    byte,
    word,
    long,

    pub fn bytes(s: Size) u3 {
        return switch (s) {
            .byte => 1,
            .word => 2,
            .long => 4,
        };
    }

    pub fn mask(s: Size) u32 {
        return switch (s) {
            .byte => 0xFF,
            .word => 0xFFFF,
            .long => 0xFFFF_FFFF,
        };
    }

    pub fn msb(s: Size) u32 {
        return switch (s) {
            .byte => 0x80,
            .word => 0x8000,
            .long => 0x8000_0000,
        };
    }

    /// MOVE encodes size differently from everything else.
    pub fn fromMoveField(v: u2) ?Size {
        return switch (v) {
            0b01 => .byte,
            0b11 => .word,
            0b10 => .long,
            0b00 => null,
        };
    }
};

/// Exception vector numbers. The value is the vector number; the vector's
/// address in memory is `vectorAddr`.
pub const Exception = enum(u8) {
    reset = 0,
    bus_error = 2,
    address_error = 3,
    illegal_instruction = 4,
    zero_divide = 5,
    chk = 6,
    trapv = 7,
    privilege_violation = 8,
    trace = 9,
    line_a = 10,
    line_f = 11,
    uninitialized_interrupt = 15,
    spurious_interrupt = 24,
    autovector_1 = 25,
    autovector_7 = 31,
    trap_0 = 32,
    trap_15 = 47,
    _,

    pub fn autovector(level: u3) Exception {
        std.debug.assert(level != 0);
        return @enumFromInt(24 + @as(u8, level));
    }

    pub fn vectorAddr(e: Exception) u32 {
        return @as(u32, @intFromEnum(e)) * 4;
    }

    /// Group 0 exceptions push the 7-word frame instead of the 3-word one.
    pub fn isGroup0(e: Exception) bool {
        return switch (e) {
            .bus_error, .address_error => true,
            else => false,
        };
    }
};

/// Special status word layout for the group 0 exception frame: R/W, I/N and
/// the three FC pins live in the low bits; bits 5..15 are undefined on
/// hardware but MAME leaves the instruction register there, so tests check it.
pub const ssw_rw_read: u16 = 0x10;
pub const ssw_fc_program: u16 = 0x2;
pub const ssw_fc_data: u16 = 0x1;
pub const ssw_fc_supervisor: u16 = 0x4;
pub const ssw_ir_residue_mask: u16 = 0xFFE0;
/// R/W + FC occupy the low 5 bits; the rest is prefetch/IR residue.
pub const ssw_field_mask: u16 = 0x1F;

/// Extra state pushed by a group 0 (bus/address error) exception frame.
pub const Group0Info = struct {
    access_addr: u32,
    ir: u16,
    read: bool,
    /// Program space vs data space, for the function-code word.
    program: bool,
};

/// The entire CPU state. Plain and copyable: `const snapshot = cpu.*` is a
/// complete save state.
pub const Cpu = struct {
    d: [8]u32 = @splat(0),
    /// a[7] is always the *active* stack pointer for the current privilege mode.
    a: [8]u32 = @splat(0),
    /// The inactive stack pointer. Swapped with a[7] by `setSupervisor`.
    usp: u32 = 0,
    pc: u32 = 0,
    sr: StatusRegister = .{},
    /// STOP executed: nothing happens until an interrupt arrives.
    stopped: bool = false,
    /// T was set when the last instruction started, so a trace exception is due
    /// at the next instruction boundary. Deferring it this way puts the
    /// boundary where the hardware puts it: the traced instruction completes
    /// fully, and trace outranks a pending interrupt.
    trace_pending: bool = false,
    /// Double fault. Dead until reset.
    halted: bool = false,
    /// Interrupt level asserted on IPL0-2 by the host. 0 means none.
    pending_ipl: u3 = 0,
    cycles: u64 = 0,

    /// The only place the stack pointers swap.
    pub fn setSupervisor(cpu: *Cpu, s: bool) void {
        if (cpu.sr.s == s) return;
        std.mem.swap(u32, &cpu.a[7], &cpu.usp);
        cpu.sr.s = s;
    }

    /// Supervisor stack pointer regardless of current mode. For the test
    /// harness, which names both stacks explicitly.
    pub fn ssp(cpu: *const Cpu) u32 {
        return if (cpu.sr.s) cpu.a[7] else cpu.usp;
    }

    /// User stack pointer regardless of current mode.
    pub fn userSp(cpu: *const Cpu) u32 {
        return if (cpu.sr.s) cpu.usp else cpu.a[7];
    }
};

test "SR round-trips and drops unused bits" {
    const sr = StatusRegister.fromInt(0xFFFF);
    try std.testing.expectEqual(@as(u16, 0xA71F), sr.toInt());
    try std.testing.expect(sr.t and sr.s and sr.ipl == 7 and sr.x);

    try std.testing.expectEqual(@as(u16, 0x2700), (StatusRegister{}).toInt());
}

test "setSupervisor swaps stack pointers symmetrically" {
    var cpu = Cpu{ .sr = .{ .s = true } };
    cpu.a[7] = 0x8000; // ssp
    cpu.usp = 0x1000;

    cpu.setSupervisor(false);
    try std.testing.expectEqual(@as(u32, 0x1000), cpu.a[7]);
    try std.testing.expectEqual(@as(u32, 0x8000), cpu.ssp());

    cpu.setSupervisor(true);
    try std.testing.expectEqual(@as(u32, 0x8000), cpu.a[7]);
    try std.testing.expectEqual(@as(u32, 0x1000), cpu.userSp());

    // Setting the mode it is already in must not swap.
    cpu.setSupervisor(true);
    try std.testing.expectEqual(@as(u32, 0x8000), cpu.a[7]);
}

test "Size helpers" {
    try std.testing.expectEqual(@as(u32, 0xFFFF), Size.word.mask());
    try std.testing.expectEqual(@as(u32, 0x8000_0000), Size.long.msb());
    try std.testing.expectEqual(@as(?Size, .byte), Size.fromMoveField(0b01));
}

test "exception vector addresses" {
    try std.testing.expectEqual(@as(u32, 0x80), Exception.trap_0.vectorAddr());
    try std.testing.expectEqual(@as(u32, 0x7C), Exception.autovector(7).vectorAddr());
    try std.testing.expect(Exception.address_error.isGroup0());
}
