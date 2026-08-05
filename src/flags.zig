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

/// N, Z, V, C from `a + b`; X mirrors C. Shared by ADD/ADDI/ADDQ (and CMP's
/// mirror-image, SUB, below). Just `addx` with no carry in and Z free to be
/// set outright (`old_z = true`).
pub fn add(a: u32, b: u32, size: Size) Ccr {
    return addx(a, b, false, size, true);
}

/// N, Z, V, C from `a - b`; X mirrors C. Shared by SUB/SUBI/SUBQ/CMP/CMPI/
/// CMPA/CMPM (compares just discard the result) and NEG (`a = 0`). Just
/// `subx` with no borrow in and Z free to be set outright.
pub fn sub(a: u32, b: u32, size: Size) Ccr {
    return subx(a, b, false, size, true);
}

/// Like `add`, but folds in an incoming X (carry chain) and only ever
/// *clears* Z, never sets it — so a chain of ADDX across a multi-word value
/// reads as zero only if every word in the chain was zero.
pub fn addx(a: u32, b: u32, x_in: bool, size: Size, old_z: bool) Ccr {
    const mask = size.mask();
    const am = a & mask;
    const bm = b & mask;
    const sum: u64 = @as(u64, am) + @as(u64, bm) + @intFromBool(x_in);
    const result: u32 = @as(u32, @truncate(sum)) & mask;
    const carry = sum > mask;
    const an = isNeg(am, size);
    const bn = isNeg(bm, size);
    const rn = isNeg(result, size);
    const v = (an == bn) and (rn != an);
    return .{ .n = rn, .z = result == 0 and old_z, .v = v, .c = carry, .x = carry };
}

/// Like `sub`, but folds in an incoming X (borrow chain) and only ever
/// *clears* Z. Shared by SUBX and NEGX (`a = 0`).
pub fn subx(a: u32, b: u32, x_in: bool, size: Size, old_z: bool) Ccr {
    const mask = size.mask();
    const am: u64 = a & mask;
    const bm: u64 = b & mask;
    const subtrahend = bm + @intFromBool(x_in);
    const result: u32 = @as(u32, @truncate(am -% subtrahend)) & mask;
    const borrow = subtrahend > am;
    const an = isNeg(@truncate(am), size);
    const bn = isNeg(@truncate(bm), size);
    const rn = isNeg(result, size);
    const v = (an != bn) and (rn != an);
    return .{ .n = rn, .z = result == 0 and old_z, .v = v, .c = borrow, .x = borrow };
}

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

test "add: signed overflow and unsigned carry" {
    // 0x7FFF + 1 overflows into the sign bit: V set, N set, C clear.
    const ov = add(0x7FFF, 1, .word);
    try std.testing.expect(ov.v and ov.n and !ov.c and !ov.z);

    // 0xFFFF + 1 carries out and wraps to zero: C and Z set, V clear.
    const wrap = add(0xFFFF, 1, .word);
    try std.testing.expect(wrap.c and wrap.z and !wrap.v and wrap.x);

    // Long add wraps the full u32 the same way.
    const long_wrap = add(0xFFFF_FFFF, 1, .long);
    try std.testing.expect(long_wrap.c and long_wrap.z);
}

test "sub: signed overflow and borrow" {
    // 0x80 - 1 on a byte: most-negative minus one overflows (V), no borrow.
    const ov = sub(0x80, 1, .byte);
    try std.testing.expect(ov.v and !ov.n and !ov.c);

    // 0 - 1 borrows: C/X set, result wraps to all-ones, N set.
    const borrow = sub(0, 1, .byte);
    try std.testing.expect(borrow.c and borrow.x and borrow.n and !borrow.v);

    try std.testing.expect(sub(5, 5, .long).z);
}

test "addx/subx accumulate Z across a chain, never set it back" {
    // First word is zero with no carry in: Z true.
    const w0 = addx(0, 0, false, .word, true);
    try std.testing.expect(w0.z);

    // Second word is also zero, carry in false: Z stays true (old_z and true).
    const w1 = addx(0, 0, false, .word, w0.z);
    try std.testing.expect(w1.z);

    // A nonzero word breaks the chain, and a later zero word can't revive it.
    const nz = addx(1, 0, false, .word, true);
    try std.testing.expect(!nz.z);
    const after = addx(0, 0, false, .word, nz.z);
    try std.testing.expect(!after.z);

    // subx borrow chain: 0 - 0 - x_in(1) borrows.
    const b = subx(0, 0, true, .byte, true);
    try std.testing.expect(b.c and b.x and !b.z);
}
