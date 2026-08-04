//! Condition-code computation. Every function here is pure: values in, result
//! and flags out, no `Cpu` in sight. This is where 68000 emulators most often
//! go wrong, so it is the part that is cheapest to test in isolation.
//!
//! Carry and overflow are derived from operand signs and a widened result
//! rather than from bit-twiddling identities. Slightly more work per op, and
//! obviously correct at a glance.

const std = @import("std");
const cpu = @import("cpu.zig");
const Ccr = cpu.Ccr;
const Size = cpu.Size;

fn isNeg(v: u32, size: Size) bool {
    return v & size.msb() != 0;
}

/// N and Z only. The shape used by MOVE, AND, OR, EOR, NOT, TST, ... which all
/// clear V and C.
pub fn logic(value: u32, size: Size) Ccr {
    const v = value & size.mask();
    return .{ .n = isNeg(v, size), .z = v == 0, .v = false, .c = false };
}

// TODO(M2): ALU flag functions (add/sub/cmp/addx/subx), added with the
// handlers that call them.

/// The 16 condition codes, shared by Bcc, Scc and DBcc.
pub fn testCondition(sr: cpu.StatusRegister, cond: u4) bool {
    return switch (cond) {
        0x0 => true, // T
        0x1 => false, // F
        0x2 => !sr.c and !sr.z, // HI
        0x3 => sr.c or sr.z, // LS
        0x4 => !sr.c, // CC/HS
        0x5 => sr.c, // CS/LO
        0x6 => !sr.z, // NE
        0x7 => sr.z, // EQ
        0x8 => !sr.v, // VC
        0x9 => sr.v, // VS
        0xA => !sr.n, // PL
        0xB => sr.n, // MI
        0xC => sr.n == sr.v, // GE
        0xD => sr.n != sr.v, // LT
        0xE => !sr.z and sr.n == sr.v, // GT
        0xF => sr.z or sr.n != sr.v, // LE
    };
}

// TODO(M3): shift/rotate flags (ASx/LSx/ROx/ROXx). Their C/X/V rules differ per
// variant and at count 0, so they get their own functions and their own tests.

test "condition codes" {
    const eq = cpu.StatusRegister{ .z = true };
    try std.testing.expect(testCondition(eq, 0x7)); // EQ
    try std.testing.expect(!testCondition(eq, 0x6)); // NE
    try std.testing.expect(testCondition(eq, 0xF)); // LE
    try std.testing.expect(!testCondition(eq, 0xE)); // GT
    try std.testing.expect(testCondition(eq, 0x3)); // LS

    // N != V is LT; GT additionally requires !Z.
    const lt = cpu.StatusRegister{ .n = true };
    try std.testing.expect(testCondition(lt, 0xD));
    try std.testing.expect(!testCondition(lt, 0xC));

    const gt = cpu.StatusRegister{};
    try std.testing.expect(testCondition(gt, 0xE));
    try std.testing.expect(testCondition(gt, 0x0)); // T
    try std.testing.expect(!testCondition(gt, 0x1)); // F
}

test "logic clears V and C" {
    const c = logic(0xFFFF_8000, .word);
    try std.testing.expect(c.n and !c.z and !c.v and !c.c);
    try std.testing.expect(logic(0xFFFF_0000, .word).z);
}
