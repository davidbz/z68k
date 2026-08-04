//! Opcode decoding. The decoder's output is data: a 65536-entry table built at
//! comptime from the pure function `decodeOne`.
//!
//! Musashi generates its dispatch with an external C program (`m68kmake`).
//! Zig's comptime does the same job in-language, so the "generator" is just
//! `decodeOne` — ordinary readable Zig that anyone can step through.

const std = @import("std");
const cpu = @import("cpu.zig");
const Size = cpu.Size;

pub const EaMode = enum {
    data_reg, // Dn
    addr_reg, // An
    addr_ind, // (An)
    addr_postinc, // (An)+
    addr_predec, // -(An)
    addr_disp, // d16(An)
    addr_index, // d8(An,Xn.sz)
    abs_word, // (xxx).W
    abs_long, // (xxx).L
    pc_disp, // d16(PC)
    pc_index, // d8(PC,Xn.sz)
    immediate, // #imm

    /// `null` for the reserved mode-7 register encodings.
    pub fn decode(mode: u3, reg: u3) ?EaMode {
        return switch (mode) {
            0 => .data_reg,
            1 => .addr_reg,
            2 => .addr_ind,
            3 => .addr_postinc,
            4 => .addr_predec,
            5 => .addr_disp,
            6 => .addr_index,
            7 => switch (reg) {
                0 => .abs_word,
                1 => .abs_long,
                2 => .pc_disp,
                3 => .pc_index,
                4 => .immediate,
                else => null,
            },
        };
    }

    /// PC-relative operands live in program space, which the FC pins expose
    /// and a group 0 exception frame records.
    pub fn isProgram(m: EaMode) bool {
        return m == .pc_disp or m == .pc_index;
    }

    /// Cycles to compute the address and fetch the operand, added to an
    /// instruction's base cost. (M68000UM addressing-mode timing table.)
    pub fn cycles(m: EaMode, size: Size) u8 {
        const long = size == .long;
        return switch (m) {
            .data_reg, .addr_reg => 0,
            .addr_ind, .addr_postinc => if (long) 8 else 4,
            .addr_predec => if (long) 10 else 6,
            .addr_disp, .abs_word, .pc_disp => if (long) 12 else 8,
            .addr_index, .pc_index => if (long) 14 else 10,
            .abs_long => if (long) 16 else 12,
            .immediate => if (long) 8 else 4,
        };
    }

    /// Cost as a destination. Identical to `cycles` except that a write skips
    /// the two extra cycles a predecrement source spends computing its address
    /// ahead of the bus cycle.
    pub fn destCycles(m: EaMode, size: Size) u8 {
        return switch (m) {
            .addr_predec => if (size == .long) 8 else 4,
            else => m.cycles(size),
        };
    }
};

pub const EaClass = std.EnumSet(EaMode);

fn classOf(comptime modes: []const EaMode) EaClass {
    var set = EaClass.initEmpty();
    for (modes) |m| set.insert(m);
    return set;
}

/// Addressing-mode classes from the M68000 PRM. Used to reject illegal operand
/// combinations while the table is being built, so no runtime check is needed.
// TODO(M2): ea_data / ea_memory and their alterable intersections, with the
// families that restrict operands to them.
pub const ea_all: EaClass = EaClass.initFull();
pub const ea_control: EaClass = classOf(&.{
    .addr_ind, .addr_disp, .addr_index, .abs_word, .abs_long, .pc_disp, .pc_index,
});
pub const ea_alterable: EaClass = ea_all.differenceWith(classOf(&.{ .pc_disp, .pc_index, .immediate }));

pub const Mnemonic = enum {
    // Immediate group (0000)
    ori,
    ori_ccr,
    ori_sr,
    andi,
    andi_ccr,
    andi_sr,
    subi,
    addi,
    eori,
    eori_ccr,
    eori_sr,
    cmpi,
    btst,
    bchg,
    bclr,
    bset,
    movep,
    // Moves (0001/0010/0011)
    move,
    movea,
    moveq,
    // Misc (0100)
    negx,
    clr,
    neg,
    not,
    nbcd,
    swap,
    pea,
    ext,
    movem,
    tst,
    tas,
    illegal_insn,
    trap,
    link,
    unlk,
    move_usp,
    move_from_sr,
    move_to_ccr,
    move_to_sr,
    reset_insn,
    nop,
    stop,
    rte,
    rts,
    trapv,
    rtr,
    jsr,
    jmp,
    lea,
    chk,
    // Quick / conditional (0101, 0110)
    addq,
    subq,
    scc,
    dbcc,
    bra,
    bsr,
    bcc,
    // ALU (1000..1101)
    or_,
    divu,
    divs,
    sbcd,
    sub,
    suba,
    subx,
    cmp,
    cmpa,
    cmpm,
    eor,
    and_,
    mulu,
    muls,
    abcd,
    exg,
    add,
    adda,
    addx,
    // Shifts / rotates (1110)
    asl,
    asr,
    lsl,
    lsr,
    rol,
    ror,
    roxl,
    roxr,
    // Not an instruction
    illegal,
    line_a,
    line_f,
};

/// The decoded identity of one opcode word. `base_cycles` excludes the
/// addressing-mode cost, which `core` adds once it knows the effective address.
pub const Instr = struct {
    mnemonic: Mnemonic,
    size: Size = .word,
    base_cycles: u8 = 0,
};

const illegal: Instr = .{ .mnemonic = .illegal };

/// Pure opcode -> instruction identity. Adding an instruction family means
/// adding one branch here; `table` and the disassembler pick it up for free.
pub fn decodeOne(op: u16) Instr {
    const mode: u3 = @truncate(op >> 3);
    const reg: u3 = @truncate(op);

    return switch (op >> 12) {
        0b0001, 0b0011, 0b0010 => decodeMove(op),

        0b0100 => switch (op) {
            0x4E70 => .{ .mnemonic = .reset_insn, .base_cycles = 132 },
            0x4E71 => .{ .mnemonic = .nop, .base_cycles = 4 },
            0x4E72 => .{ .mnemonic = .stop, .base_cycles = 4 },
            0x4E73 => .{ .mnemonic = .rte, .base_cycles = 20 },
            0x4E75 => .{ .mnemonic = .rts, .base_cycles = 16 },
            0x4E76 => .{ .mnemonic = .trapv, .base_cycles = 4 },
            0x4E77 => .{ .mnemonic = .rtr, .base_cycles = 20 },
            0x4AFC => .{ .mnemonic = .illegal_insn },
            else => blk: {
                // LEA: 0100 rrr 111 mmm rrr, control EAs only.
                if (op & 0x01C0 == 0x01C0) {
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (!ea_control.contains(ea)) break :blk illegal;
                    // LEA computes an address without fetching an operand, so
                    // it has its own timings rather than the usual EA costs.
                    break :blk .{
                        .mnemonic = .lea,
                        .size = .long,
                        .base_cycles = switch (ea) {
                            .addr_ind => 4,
                            .addr_disp, .abs_word, .pc_disp => 8,
                            .addr_index, .abs_long, .pc_index => 12,
                            else => unreachable,
                        },
                    };
                }
                break :blk illegal; // TODO(M4): rest of the 0100 line
            },
        },

        0b0111 => if (op & 0x0100 == 0)
            .{ .mnemonic = .moveq, .size = .long, .base_cycles = 4 }
        else
            illegal,

        0b0110 => switch (@as(u4, @truncate(op >> 8))) {
            0b0000 => .{ .mnemonic = .bra, .base_cycles = 10 },
            0b0001 => .{ .mnemonic = .bsr, .base_cycles = 18 },
            else => .{ .mnemonic = .bcc, .base_cycles = 8 },
        },

        // TODO(M2/M3): 0000, 0101, 1000..1110 families. See DESIGN.md §4.8.
        // Instructions whose whole job is to raise an exception cost nothing on
        // their own: the vector's entry time already covers the opcode fetch.
        0b1010 => .{ .mnemonic = .line_a },
        0b1111 => .{ .mnemonic = .line_f },
        else => illegal,
    };
}

fn decodeMove(op: u16) Instr {
    const size = Size.fromMoveField(@truncate(op >> 12)) orelse return illegal;

    const src = EaMode.decode(@truncate(op >> 3), @truncate(op)) orelse return illegal;
    const dst = EaMode.decode(@truncate(op >> 6), @truncate(op >> 9)) orelse return illegal;

    // Byte-sized MOVE cannot touch an address register at either end.
    if (size == .byte and (src == .addr_reg or dst == .addr_reg)) return illegal;
    if (!ea_alterable.contains(dst)) return illegal;

    return .{
        .mnemonic = if (dst == .addr_reg) .movea else .move,
        .size = size,
        .base_cycles = 4,
    };
}

/// Every opcode word, decoded once at compile time.
pub const table: [65536]Instr = blk: {
    @setEvalBranchQuota(40_000_000);
    var t: [65536]Instr = undefined;
    for (&t, 0..) |*e, op| e.* = decodeOne(@intCast(op));
    break :blk t;
};

test "decode table is total and matches decodeOne" {
    // Every opcode maps to something; nothing is left undefined.
    for (0..65536) |op| {
        const o: u16 = @intCast(op);
        try std.testing.expectEqual(decodeOne(o), table[o]);
    }
}

test "spot-check known encodings" {
    try std.testing.expectEqual(Mnemonic.rts, table[0x4E75].mnemonic);
    try std.testing.expectEqual(Mnemonic.nop, table[0x4E71].mnemonic);
    try std.testing.expectEqual(Mnemonic.illegal_insn, table[0x4AFC].mnemonic);
    try std.testing.expectEqual(Mnemonic.line_a, table[0xA123].mnemonic);
    try std.testing.expectEqual(Mnemonic.line_f, table[0xF000].mnemonic);

    // move.w #imm,d0  =  0011 000 000 111 100
    try std.testing.expectEqual(Mnemonic.move, table[0x303C].mnemonic);
    try std.testing.expectEqual(Size.word, table[0x303C].size);

    // movea.l a0,a1 -> dst is An, so MOVEA
    try std.testing.expectEqual(Mnemonic.movea, table[0x2248].mnemonic);
    // moveq #1,d0
    try std.testing.expectEqual(Mnemonic.moveq, table[0x7001].mnemonic);
    // lea (a0),a1
    try std.testing.expectEqual(Mnemonic.lea, table[0x43D0].mnemonic);
}

test "illegal operand combinations are rejected at decode time" {
    // move.b a0,d0 — byte moves cannot source an address register.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x1008].mnemonic);
    // move.w d0,#imm — immediate is not an alterable destination.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x39C0].mnemonic);
    // lea #imm,a0 — immediate is not a control mode.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x41FC].mnemonic);
}

test "addressing mode classes" {
    try std.testing.expect(!ea_alterable.contains(.pc_disp));
    try std.testing.expect(ea_control.contains(.pc_disp));
    try std.testing.expect(!ea_control.contains(.addr_postinc));
    try std.testing.expectEqual(@as(?EaMode, null), EaMode.decode(7, 5));
}
