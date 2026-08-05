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
pub const ea_all: EaClass = EaClass.initFull();
pub const ea_control: EaClass = classOf(&.{
    .addr_ind, .addr_disp, .addr_index, .abs_word, .abs_long, .pc_disp, .pc_index,
});
pub const ea_alterable: EaClass = ea_all.differenceWith(classOf(&.{ .pc_disp, .pc_index, .immediate }));
/// Everything except An: on the base 68000, address registers never take
/// part in byte-sized operand or single-operand-data-op forms.
pub const ea_data: EaClass = ea_all.differenceWith(classOf(&.{.addr_reg}));
/// Dn plus alterable memory, no An/PC-relative/immediate. The destination
/// class shared by NEG/NEGX/NOT/CLR/TST and the whole immediate ALU group
/// (confirmed against Musashi's opcode table: those are exactly the forms
/// the base 68000 does not support for this family).
pub const ea_data_alterable: EaClass = ea_data.intersectWith(ea_alterable);
/// Alterable memory only: no Dn, no An. The destination class for the
/// Dn->ea direction of ADD/SUB/AND/OR (EOR uses `ea_data_alterable` instead,
/// since it allows a Dn destination too). Using this rather than a hand-
/// rolled reserved-bit check routes the ABCD/SBCD/EXG/MULU/MULS/CMPM/ADDX/
/// SUBX carve-outs sharing these opcode lines to `.illegal` for free: those
/// reserved encodings all decode to `data_reg` or `addr_reg` mode under the
/// general EA scheme, which this class already excludes.
pub const ea_memory_alterable: EaClass = ea_alterable.differenceWith(classOf(&.{ .data_reg, .addr_reg }));

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
        // Immediate group: ORI/ANDI/SUBI/ADDI/EORI/CMPI (0000 oooo ss mmm rrr)
        // plus six fixed-opcode CCR/SR variants sharing ORI/ANDI/EORI's prefix
        // at ea=immediate (mode 111, reg 100), where the "size" field (00/01)
        // that would otherwise select byte/word instead picks CCR vs SR.
        0b0000 => switch (op) {
            0x003C => .{ .mnemonic = .ori_ccr, .size = .byte, .base_cycles = 20 },
            0x007C => .{ .mnemonic = .ori_sr, .size = .word, .base_cycles = 20 },
            0x023C => .{ .mnemonic = .andi_ccr, .size = .byte, .base_cycles = 20 },
            0x027C => .{ .mnemonic = .andi_sr, .size = .word, .base_cycles = 20 },
            0x0A3C => .{ .mnemonic = .eori_ccr, .size = .byte, .base_cycles = 20 },
            0x0A7C => .{ .mnemonic = .eori_sr, .size = .word, .base_cycles = 20 },
            else => blk: {
                // Static bit op: 0000 1000 oo mmm rrr, bit# in the ext word.
                if (op & 0xFF00 == 0x0800) break :blk decodeBitOp(op, mode, reg, true);
                // Dynamic bit op / MOVEP: 0000 ddd1 oo mmm rrr. MOVEP always
                // has mode=001 (An direct), which a bit op's ea can never be
                // (An isn't in `ea_data`), so the two never collide.
                if (op & 0x0100 != 0) {
                    if (mode == 1) break :blk decodeMovep(op);
                    break :blk decodeBitOp(op, mode, reg, false);
                }
                break :blk decodeImmediateAlu(op, mode, reg);
            },
        },

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

                // NBCD: 0100 1000 00 mmm rrr, byte-only BCD negate. Its own
                // fixed slot at the 0x48 nibble's ss=00; ss=01 there is
                // SWAP/PEA, ss=10/11 EXT/MOVEM (below).
                if (op & 0xFFC0 == 0x4800) {
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (!ea_data_alterable.contains(ea)) break :blk illegal;
                    break :blk .{
                        .mnemonic = .nbcd,
                        .size = .byte,
                        .base_cycles = if (ea == .data_reg) @as(u8, 6) else 8,
                    };
                }

                // SWAP/PEA share 0x4840-0x487F: SWAP is the ea=data_reg
                // subset (Dn only, the mode PEA's control-only class already
                // excludes), PEA the rest.
                if (op & 0xFFC0 == 0x4840) {
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (ea == .data_reg) break :blk .{ .mnemonic = .swap, .base_cycles = 4 };
                    if (!ea_control.contains(ea)) break :blk illegal;
                    break :blk .{
                        .mnemonic = .pea,
                        .size = .long,
                        .base_cycles = switch (ea) {
                            .addr_ind => 12,
                            .addr_disp, .abs_word, .pc_disp => 16,
                            .addr_index, .abs_long, .pc_index => 20,
                            else => unreachable,
                        },
                    };
                }

                // EXT/MOVEM (reg->mem direction) share 0x4880-0x48FF: bit 6
                // picks word/long, EXT is the ea=data_reg subset (MOVEM's ea
                // never allows Dn/An anyway), MOVEM the rest.
                if (op & 0xFF80 == 0x4880) {
                    const size: Size = if (op & 0x0040 != 0) .long else .word;
                    if (mode == 0) break :blk .{ .mnemonic = .ext, .size = size, .base_cycles = 4 };
                    break :blk decodeMovem(mode, reg, size, false);
                }
                // MOVEM, mem->reg direction: 0x4C80-0x4CFF.
                if (op & 0xFF80 == 0x4C80) {
                    const size: Size = if (op & 0x0040 != 0) .long else .word;
                    break :blk decodeMovem(mode, reg, size, true);
                }

                // TRAP #n: 0x4E40-0x4E4F.
                if (op & 0xFFF0 == 0x4E40) break :blk .{ .mnemonic = .trap };
                // LINK/UNLK: 0x4E50-0x4E5F, bit 3 picks which.
                if (op & 0xFFF8 == 0x4E50) break :blk .{ .mnemonic = .link, .size = .long, .base_cycles = 16 };
                if (op & 0xFFF8 == 0x4E58) break :blk .{ .mnemonic = .unlk, .base_cycles = 12 };
                // MOVE USP: 0x4E60-0x4E6F, bit 3 picks direction (handler
                // re-reads it, same as MOVEP's direction bits).
                if (op & 0xFFF0 == 0x4E60) break :blk .{ .mnemonic = .move_usp, .size = .long, .base_cycles = 4 };

                // JSR/JMP: 0100 1110 1 d mmm rrr, control EAs only.
                if (op & 0xFF80 == 0x4E80) {
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (!ea_control.contains(ea)) break :blk illegal;
                    const is_jmp = op & 0x0040 != 0;
                    const jsr_costs = [_]u8{ 16, 18, 22, 18, 20, 18, 22 };
                    const jmp_costs = [_]u8{ 8, 10, 14, 10, 12, 10, 14 };
                    const idx: usize = switch (ea) {
                        .addr_ind => 0,
                        .addr_disp => 1,
                        .addr_index => 2,
                        .abs_word => 3,
                        .abs_long => 4,
                        .pc_disp => 5,
                        .pc_index => 6,
                        else => unreachable,
                    };
                    break :blk .{
                        .mnemonic = if (is_jmp) .jmp else .jsr,
                        .base_cycles = if (is_jmp) jmp_costs[idx] else jsr_costs[idx],
                    };
                }

                // CHK: 0100 ddd1 10 mmm rrr, word only on the base 68000.
                if (op & 0x01C0 == 0x0180) {
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (!ea_data.contains(ea)) break :blk illegal;
                    break :blk .{ .mnemonic = .chk, .size = .word, .base_cycles = 10 };
                }

                // Single-operand ALU forms: NEGX/CLR/NEG/NOT/TST, all shaped
                // 0100 oooo ssmmmrrr. Size field 11 is a reserved slot at four
                // of these prefixes, repurposed for MOVEfromSR/MOVEtoCCR/
                // MOVEtoSR/TAS (CLR's is genuinely unused).
                const mn: ?Mnemonic = switch (op & 0xFF00) {
                    0x4000 => .negx,
                    0x4200 => .clr,
                    0x4400 => .neg,
                    0x4600 => .not,
                    0x4A00 => .tst,
                    else => null,
                };
                if (mn) |m| {
                    const size_field: u2 = @truncate(op >> 6);
                    if (size_field == 0b11) {
                        break :blk switch (m) {
                            .negx => decodeMoveFromSr(mode, reg),
                            .neg => decodeMoveToCcr(mode, reg),
                            .not => decodeMoveToSr(mode, reg),
                            .tst => decodeTas(mode, reg),
                            else => illegal,
                        };
                    }
                    const size: Size = @enumFromInt(size_field);
                    const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                    if (!ea_data_alterable.contains(ea)) break :blk illegal;
                    break :blk .{
                        .mnemonic = m,
                        .size = size,
                        .base_cycles = switch (m) {
                            .tst => 4,
                            else => if (size == .long) @as(u8, 6) else 4,
                        },
                    };
                }

                break :blk illegal;
            },
        },

        0b0111 => if (op & 0x0100 == 0)
            .{ .mnemonic = .moveq, .size = .long, .base_cycles = 4 }
        else
            illegal,

        0b0101 => blk: {
            const size_field: u2 = @truncate(op >> 6);
            if (size_field == 0b11) {
                // Scc / DBcc share this region: 0101 cccc 11 mmm rrr, where
                // mode=001 (An) is otherwise unused by Scc and is DBcc instead.
                const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
                if (ea == .addr_reg) {
                    break :blk .{ .mnemonic = .dbcc, .base_cycles = 12 };
                }
                if (!ea_data_alterable.contains(ea)) break :blk illegal;
                // Dn baseline is 4; the handler adds 2 more at runtime when
                // the condition evaluates true (any condition, not just ST —
                // a real hardware quirk, not something decode time can know).
                // Memory destination is a flat 8 regardless.
                break :blk .{
                    .mnemonic = .scc,
                    .size = .byte,
                    .base_cycles = if (ea == .data_reg) @as(u8, 4) else 8,
                };
            }

            // ADDQ/SUBQ: 0101 ddd s ss mmm rrr (s: 0 = add, 1 = sub).
            const size: Size = @enumFromInt(size_field);
            const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
            // Byte can never touch An; word/long can (no flags, always long).
            const allowed = if (size == .byte) ea_data_alterable else ea_alterable;
            if (!allowed.contains(ea)) break :blk illegal;
            const is_sub = op & 0x0100 != 0;
            break :blk .{
                .mnemonic = if (is_sub) .subq else .addq,
                .size = size,
                // Dn baseline; the handler adds the addressing-mode cost on
                // top for a memory destination. An destination is always
                // treated as long, takes no flags, and (a real hardware
                // asymmetry) costs SUBQ.w double what ADDQ.w costs — a fixed
                // special case rather than something the usual EA cost table
                // composes cleanly.
                .base_cycles = switch (ea) {
                    .addr_reg => if (size == .long) @as(u8, 8) else if (is_sub) @as(u8, 8) else @as(u8, 4),
                    else => if (size == .long) @as(u8, 8) else @as(u8, 4),
                },
            };
        },

        0b0110 => switch (@as(u4, @truncate(op >> 8))) {
            0b0000 => .{ .mnemonic = .bra, .base_cycles = 10 },
            0b0001 => .{ .mnemonic = .bsr, .base_cycles = 18 },
            else => .{ .mnemonic = .bcc, .base_cycles = 8 },
        },

        // OR/SUB/AND/ADD share one shape: 1oo0 rrr ooo mmm rrr, where the
        // 3-bit opmode (bits 8-6) picks byte/word/long "er" (ea op Dn -> Dn,
        // 0-2), the A-form (ea -> An, 3 word / 7 long — AND/OR use those
        // slots for MULU/MULS and DIVU/DIVS instead, see `decodeAndOrLine`),
        // or byte/word/long "re" (Dn op ea -> ea, memory only, 4-6).
        0b1000 => decodeAndOrLine(op, mode, reg, .or_, .sbcd, null, .divu, .divs),
        0b1001 => decodeAluLine(op, mode, reg, .sub, .sub, .suba, .subx),
        0b1100 => decodeAndOrLine(op, mode, reg, .and_, .abcd, .exg, .mulu, .muls),
        0b1101 => decodeAluLine(op, mode, reg, .add, .add, .adda, .addx),

        // CMP/CMPA/EOR share the 1011 line the same way, except CMP has no
        // "re" direction (those opmodes are EOR instead) and CMPA costs 6 for
        // both word and long rather than ADDA/SUBA's 8/6.
        0b1011 => blk: {
            const opmode: u3 = @truncate(op >> 6);
            const ea = EaMode.decode(mode, reg) orelse break :blk illegal;
            switch (opmode) {
                0, 1, 2 => {
                    const size: Size = @enumFromInt(opmode);
                    const allowed = if (size == .byte) ea_data else ea_all;
                    if (!allowed.contains(ea)) break :blk illegal;
                    break :blk .{ .mnemonic = .cmp, .size = size, .base_cycles = if (size == .long) @as(u8, 6) else 4 };
                },
                3 => break :blk .{ .mnemonic = .cmpa, .size = .word, .base_cycles = 6 },
                4, 5, 6 => {
                    // CMPM reuses this direction's byte/word/long slots at
                    // ea==addr_reg: the (Ay)+,(Ax)+ pair form: a plain read,
                    // never a Dn destination, so it can't collide with EOR's
                    // own data_reg case just below.
                    const size: Size = @enumFromInt(opmode - 4);
                    if (ea == .addr_reg) {
                        break :blk .{
                            .mnemonic = .cmpm,
                            .size = size,
                            .base_cycles = if (size == .long) @as(u8, 20) else 12,
                        };
                    }
                    if (!ea_data_alterable.contains(ea)) break :blk illegal;
                    break :blk .{
                        .mnemonic = .eor,
                        .size = size,
                        // EOR.l to Dn is a real hardware outlier: 8, not the
                        // usual 6, since the register is read and written in
                        // the same cycle slot.
                        .base_cycles = switch (size) {
                            .long => if (ea == .data_reg) @as(u8, 8) else @as(u8, 6),
                            else => 4,
                        },
                    };
                },
                7 => break :blk .{ .mnemonic = .cmpa, .size = .long, .base_cycles = 6 },
            }
        },

        0b1110 => decodeShift(op, mode, reg),

        // Instructions whose whole job is to raise an exception cost nothing on
        // their own: the vector's entry time already covers the opcode fetch.
        0b1010 => .{ .mnemonic = .line_a },
        0b1111 => .{ .mnemonic = .line_f },
        else => illegal,
    };
}

/// OR/SUB/AND/ADD: 1oo0 rrr ooo mmm rrr. `a_mn` is `null` for AND/OR, whose
/// A-form slots are MULU/MULS and DIVU/DIVS instead (see `decodeAndOrLine`).
/// `x_mn` is `.addx`/`.subx` for the ADD/SUB lines, `null` for AND/OR (whose
/// reserved slots here are ABCD/SBCD/EXG instead): ADDX/SUBX share the
/// re-direction's
/// byte/word/long opmodes, distinguished from a normal memory destination by
/// `ea` decoding as `data_reg` (register-register form, bit 3 of the raw
/// opcode clear) or `addr_reg` (the `-(Ay),-(Ax)` memory-pair form, bit 3
/// set) — both otherwise-reserved encodings `ea_memory_alterable` already
/// excludes, so intercepting them here can't collide with a real re-direction
/// memory destination.
fn decodeAluLine(op: u16, mode: u3, reg: u3, er_mn: Mnemonic, re_mn: Mnemonic, a_mn: ?Mnemonic, x_mn: ?Mnemonic) Instr {
    const opmode: u3 = @truncate(op >> 6);
    const ea = EaMode.decode(mode, reg) orelse return illegal;

    return switch (opmode) {
        0, 1, 2 => blk: {
            const size: Size = @enumFromInt(opmode);
            const allowed = if (size == .byte) ea_data else ea_all;
            if (!allowed.contains(ea)) break :blk illegal;
            break :blk .{ .mnemonic = er_mn, .size = size, .base_cycles = if (size == .long) @as(u8, 6) else 4 };
        },
        3 => if (a_mn) |m| .{ .mnemonic = m, .size = .word, .base_cycles = 8 } else illegal,
        4, 5, 6 => blk: {
            const size: Size = @enumFromInt(opmode - 4);
            if (x_mn) |xm| {
                if (ea == .data_reg) {
                    break :blk .{ .mnemonic = xm, .size = size, .base_cycles = if (size == .long) @as(u8, 8) else 4 };
                }
                if (ea == .addr_reg) {
                    break :blk .{ .mnemonic = xm, .size = size, .base_cycles = if (size == .long) @as(u8, 30) else 18 };
                }
            }
            if (!ea_memory_alterable.contains(ea)) break :blk illegal;
            break :blk .{ .mnemonic = re_mn, .size = size, .base_cycles = if (size == .long) @as(u8, 6) else 4 };
        },
        7 => if (a_mn) |m| .{ .mnemonic = m, .size = .long, .base_cycles = 6 } else illegal,
    };
}

/// AND/OR's byte re-direction opmode (100) doubles as ABCD/SBCD's home: `ea`
/// decoding as `data_reg` is the Dy,Dx register form, `addr_reg` the
/// -(Ay),-(Ax) memory-pair form — same reserved-encoding trick `decodeAluLine`
/// uses for ADDX/SUBX, but ABCD/SBCD is byte-only (no word/long counterpart)
/// and costs more for the register form (6, not 4), so it gets its own
/// pre-check rather than another `decodeAluLine` parameter.
///
/// `exg_mn` is `.exg` on the AND line only: EXG's 5-bit mode field (bits
/// 7-3) is 01000/01001/10001 (Dx,Dy / Ax,Ay / Dx,Ay), with bit 8 always set
/// to mark the family. Checking bit 8 matters: without it the mode field
/// alone collides with ordinary word-form AND/OR opmodes (bits 8-6 = 001),
/// whose low 3 EA-mode bits can coincidentally match 01000's low 3 bits.
///
/// `u_mn`/`s_mn` (unsigned/signed MULU|DIVU vs MULS|DIVS) take opmodes 3/7,
/// the same slots ADDA/SUBA use on the other ALU lines — but unlike
/// ADDA/SUBA (same mnemonic, word vs long) the AND/OR lines use two
/// different mnemonics both at a fixed word size, and unlike ADDA/CMPA the
/// source excludes An direct (`ea_data`, not `ea_all`), so this can't reuse
/// `decodeAluLine`'s `a_mn`. The word-count-dependent part of MULU/MULS's
/// cost and DIVU/DIVS's dividend-dependent cost are runtime-only;
/// `base_cycles` here is just the fixed floor the handler adds to.
fn decodeAndOrLine(op: u16, mode: u3, reg: u3, mn: Mnemonic, bcd_mn: Mnemonic, exg_mn: ?Mnemonic, u_mn: Mnemonic, s_mn: Mnemonic) Instr {
    if (exg_mn) |em| {
        const exg_mode: u5 = @truncate(op >> 3);
        if (op & 0x0100 != 0 and (exg_mode == 0b01000 or exg_mode == 0b01001 or exg_mode == 0b10001)) {
            return .{ .mnemonic = em, .size = .long, .base_cycles = 6 };
        }
    }
    const opmode: u3 = @truncate(op >> 6);
    if (opmode == 0b100) {
        const ea = EaMode.decode(mode, reg) orelse return illegal;
        if (ea == .data_reg or ea == .addr_reg) {
            return .{ .mnemonic = bcd_mn, .size = .byte, .base_cycles = if (ea == .data_reg) @as(u8, 6) else 18 };
        }
    }
    if (opmode == 0b011 or opmode == 0b111) {
        const ea = EaMode.decode(mode, reg) orelse return illegal;
        if (!ea_data.contains(ea)) return illegal;
        const is_signed = opmode == 0b111;
        return .{
            .mnemonic = if (is_signed) s_mn else u_mn,
            .size = .word,
            .base_cycles = if (mn == .and_) @as(u8, 38) else @as(u8, 140),
        };
    }
    return decodeAluLine(op, mode, reg, mn, mn, null, null);
}

fn shiftMnemonic(ty: u2, left: bool) Mnemonic {
    return switch (ty) {
        0 => if (left) .asl else .asr,
        1 => if (left) .lsl else .lsr,
        2 => if (left) .roxl else .roxr,
        3 => if (left) .rol else .ror,
    };
}

/// ASx/LSx/ROx/ROXx: 1110 ccccc dss iii rrr for the register/immediate-count
/// form (size field 00/01/10 = byte/word/long, count/type in bits 11-9 and
/// 4-3). Size field 11 is repurposed as the memory form's marker: the type
/// field moves to bits 11-9 and the usual mode/reg pair becomes an EA (word
/// only, one bit at a time, memory-alterable — same destination class as
/// CLR/NEG, no Dn/An).
fn decodeShift(op: u16, mode: u3, reg: u3) Instr {
    const size_field: u2 = @truncate(op >> 6);
    const left = op & 0x0100 != 0;

    if (size_field == 0b11) {
        const ea = EaMode.decode(mode, reg) orelse return illegal;
        if (!ea_memory_alterable.contains(ea)) return illegal;
        const ty: u2 = @truncate(op >> 9);
        return .{ .mnemonic = shiftMnemonic(ty, left), .size = .word, .base_cycles = 8 };
    }

    const size: Size = @enumFromInt(size_field);
    const ty: u2 = @truncate(op >> 3);
    return .{
        .mnemonic = shiftMnemonic(ty, left),
        .size = size,
        // The shift count (immediate, or at runtime a register's value) adds
        // 2 cycles each; the handler applies that on top of this fixed base.
        .base_cycles = if (size == .long) @as(u8, 8) else 6,
    };
}

/// BTST/BCHG/BCLR/BSET, static (`#imm` bit number, an extension word fetched
/// at runtime) or dynamic (bit number in the ddd field's Dn). Either way the
/// bit number is a runtime value, so decode only picks mnemonic/ea/cost; the
/// handler does the mod-32-vs-mod-8 split once it knows whether `ea` is a Dn.
/// BTST never writes back, so unlike the other three it keeps `ea_data`'s
/// read-only forms (PC-relative, immediate) per the M68000 PRM's addressing
/// table for this family.
fn decodeBitOp(op: u16, mode: u3, reg: u3, static: bool) Instr {
    const sel: u2 = @truncate(op >> 6);
    const mn: Mnemonic = switch (sel) {
        0 => .btst,
        1 => .bchg,
        2 => .bclr,
        3 => .bset,
    };

    const ea = EaMode.decode(mode, reg) orelse return illegal;
    const allowed = if (mn == .btst) ea_data else ea_data_alterable;
    if (!allowed.contains(ea)) return illegal;

    const is_dn = ea == .data_reg;
    // M68000UM Table 3-19: register destination is cheaper than memory, and
    // the static forms pay 4 extra cycles to fetch the bit-number ext word.
    const base_cycles: u8 = switch (mn) {
        .btst => if (is_dn) (if (static) @as(u8, 10) else 6) else (if (static) @as(u8, 8) else 4),
        .bclr => if (is_dn) (if (static) @as(u8, 12) else 8) else (if (static) @as(u8, 12) else 8),
        .bchg, .bset => if (is_dn) (if (static) @as(u8, 10) else 6) else (if (static) @as(u8, 12) else 8),
        else => unreachable,
    };
    return .{ .mnemonic = mn, .size = .byte, .base_cycles = base_cycles };
}

/// MOVEP: 0000 ddd1 oo 001 aaa, transfers 2 or 4 bytes between Dn and
/// alternating bytes of memory at d16(An). Size/direction live in the two
/// `oo` bits (00/01 = word/long from memory, 10/11 = word/long to memory);
/// the handler re-reads them from `op` since `Instr` has no direction field.
fn decodeMovep(op: u16) Instr {
    const oo: u2 = @truncate(op >> 6);
    const size: Size = if (oo & 1 != 0) .long else .word;
    return .{ .mnemonic = .movep, .size = size, .base_cycles = if (size == .long) 24 else 16 };
}

/// ORI/ANDI/SUBI/ADDI/EORI/CMPI: 0000 oooo ss mmm rrr. The immediate operand
/// is always the source; `ea` (Dn or alterable memory, no An/PC-relative,
/// same class NEG/CLR/TST already use) is the destination, except for CMPI
/// which only reads it.
fn decodeImmediateAlu(op: u16, mode: u3, reg: u3) Instr {
    const size_field: u2 = @truncate(op >> 6);
    if (size_field == 0b11) return illegal;
    const size: Size = @enumFromInt(size_field);

    const mn: ?Mnemonic = switch (@as(u8, @truncate(op >> 8))) {
        0x00 => .ori,
        0x02 => .andi,
        0x04 => .subi,
        0x06 => .addi,
        0x0A => .eori,
        0x0C => .cmpi,
        else => null,
    };
    const m = mn orelse return illegal;

    const ea = EaMode.decode(mode, reg) orelse return illegal;
    if (!ea_data_alterable.contains(ea)) return illegal;

    // Dn destination cost includes the immediate fetch; ANDI.l and CMPI.l are
    // real hardware outliers (14, not the 16 the other long ops take). Memory
    // destination's own extra cost is added by the handler via the usual EA
    // cost table on top of this base.
    const base_dn: u8 = switch (size) {
        .byte, .word => 8,
        .long => if (m == .andi or m == .cmpi) @as(u8, 14) else @as(u8, 16),
    };
    const base_mem: u8 = switch (size) {
        .byte, .word => 8,
        .long => 12,
    };
    return .{
        .mnemonic = m,
        .size = size,
        .base_cycles = if (ea == .data_reg) base_dn else base_mem,
    };
}

/// MOVEM register list <-> memory. `load` picks the mem->reg direction
/// (control EAs plus postincrement); store (reg->mem) allows control EAs
/// plus predecrement instead. Base cost is a per-register 4, doubled for
/// long; the handler adds 2 more per register actually transferred plus
/// this floor already covers the fixed ea/extension-word overhead.
fn decodeMovem(mode: u3, reg: u3, size: Size, load: bool) Instr {
    const ea = EaMode.decode(mode, reg) orelse return illegal;
    const ok = if (load)
        ea_control.contains(ea) or ea == .addr_postinc
    else
        ea_control.intersectWith(ea_alterable).contains(ea) or ea == .addr_predec;
    if (!ok) return illegal;
    return .{ .mnemonic = .movem, .size = size, .base_cycles = if (load) 12 else 8 };
}

/// MOVE from SR: 0100000011 mmm rrr, the size=11 slot under NEGX's prefix.
fn decodeMoveFromSr(mode: u3, reg: u3) Instr {
    const ea = EaMode.decode(mode, reg) orelse return illegal;
    if (!ea_data_alterable.contains(ea)) return illegal;
    return .{ .mnemonic = .move_from_sr, .size = .word, .base_cycles = if (ea == .data_reg) @as(u8, 6) else 8 };
}

/// MOVE to CCR: 0100010011 mmm rrr, the size=11 slot under NEG's prefix.
fn decodeMoveToCcr(mode: u3, reg: u3) Instr {
    const ea = EaMode.decode(mode, reg) orelse return illegal;
    if (!ea_data.contains(ea)) return illegal;
    return .{ .mnemonic = .move_to_ccr, .size = .word, .base_cycles = 12 };
}

/// MOVE to SR: 0100011011 mmm rrr, the size=11 slot under NOT's prefix.
/// Privileged; the handler checks that.
fn decodeMoveToSr(mode: u3, reg: u3) Instr {
    const ea = EaMode.decode(mode, reg) orelse return illegal;
    if (!ea_data.contains(ea)) return illegal;
    return .{ .mnemonic = .move_to_sr, .size = .word, .base_cycles = 12 };
}

/// TAS: 0100101011 mmm rrr, the size=11 slot under TST's prefix.
fn decodeTas(mode: u3, reg: u3) Instr {
    const ea = EaMode.decode(mode, reg) orelse return illegal;
    if (!ea_data_alterable.contains(ea)) return illegal;
    return .{ .mnemonic = .tas, .size = .byte, .base_cycles = if (ea == .data_reg) @as(u8, 4) else 10 };
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

    // ea_data_alterable: Dn and alterable memory, never An/PC-relative/immediate.
    try std.testing.expect(ea_data_alterable.contains(.data_reg));
    try std.testing.expect(ea_data_alterable.contains(.addr_ind));
    try std.testing.expect(!ea_data_alterable.contains(.addr_reg));
    try std.testing.expect(!ea_data_alterable.contains(.pc_disp));
    try std.testing.expect(!ea_data_alterable.contains(.immediate));
}

test "quick and conditional forms (0101 line)" {
    // addq.b #1,d0
    try std.testing.expectEqual(Mnemonic.addq, table[0x5200].mnemonic);
    try std.testing.expectEqual(Size.byte, table[0x5200].size);
    // subq.w #1,a0 — An destination allowed for word/long, not byte.
    try std.testing.expectEqual(Mnemonic.subq, table[0x5348].mnemonic);
    try std.testing.expectEqual(@as(u8, 8), table[0x5348].base_cycles);
    // addq.w #1,a0 costs half what subq.w does to An (real asymmetry).
    try std.testing.expectEqual(@as(u8, 4), table[0x5248].base_cycles);
    // addq.b #1,a0 — byte can't touch An.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x5208].mnemonic);

    // seq d0 (cond=0111 EQ) / dbeq d0 share the 11-mmm-rrr region.
    try std.testing.expectEqual(Mnemonic.scc, table[0x57C0].mnemonic);
    try std.testing.expectEqual(Mnemonic.dbcc, table[0x57C8].mnemonic);
    // Dn destination baseline is 4 (the true-condition +2 bonus is a runtime
    // decision in core.zig, not something decode time can know).
    try std.testing.expectEqual(@as(u8, 4), table[0x50C0].base_cycles);
    try std.testing.expectEqual(@as(u8, 4), table[0x51C0].base_cycles);
}

test "single-operand ALU forms" {
    // clr.b d0
    try std.testing.expectEqual(Mnemonic.clr, table[0x4200].mnemonic);
    try std.testing.expectEqual(Size.byte, table[0x4200].size);
    // neg.w d0
    try std.testing.expectEqual(Mnemonic.neg, table[0x4440].mnemonic);
    // negx.l d0
    try std.testing.expectEqual(Mnemonic.negx, table[0x4080].mnemonic);
    // not.b d0
    try std.testing.expectEqual(Mnemonic.not, table[0x4600].mnemonic);
    // tst.l d0
    try std.testing.expectEqual(Mnemonic.tst, table[0x4A80].mnemonic);

    // tst.l a0 — An is not a valid TST operand on the base 68000.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x4A88].mnemonic);
    // clr.b #imm-shaped bits (mode=111,reg=100) — immediate not alterable.
    try std.testing.expectEqual(Mnemonic.illegal, table[0x423C].mnemonic);
    // size field 11 under negx's prefix is MOVE from SR, not our family.
    try std.testing.expectEqual(Mnemonic.move_from_sr, table[0x40C0].mnemonic);
    // size field 11 under tst's prefix is TAS.
    try std.testing.expectEqual(Mnemonic.tas, table[0x4AC0].mnemonic);
}
