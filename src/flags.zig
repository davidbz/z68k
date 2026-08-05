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

/// Packed-BCD add/sub result: N and V are famously "undefined" per the
/// Motorola manual, but ABCD/SBCD/NBCD conformance data pins down exactly
/// what real hardware does, so it's reproduced verbatim (via Musashi's
/// widely cross-checked port of it) rather than left to guesswork. Z follows
/// ADDX/SUBX's sticky convention: only ever cleared, letting a multi-byte BCD
/// chain read as zero solely when every byte in it was zero.
pub const BcdResult = struct { value: u8, cc: Ccr };

/// ABCD: `a + b + x`, corrected one decimal digit at a time. V is set exactly
/// when the decimal correction flips the result negative that the plain binary
/// `a +% b +% x` wasn't (the classic 6800-family DAA quirk) — confirmed bit
/// for bit against conformance data, not derivable from the manual.
pub fn bcdAdd(a: u8, b: u8, x_in: bool, old_z: bool) BcdResult {
    const a16: u16 = a;
    const b16: u16 = b;
    var res: u16 = (a16 & 0xF) +% (b16 & 0xF) +% @intFromBool(x_in);
    if (res > 9) res +%= 6;
    res +%= (a16 & 0xF0) +% (b16 & 0xF0);
    const carry = res > 0x9F;
    if (carry) res -%= 0xA0;
    const value: u8 = @truncate(res);
    const raw = a +% b +% @as(u8, @intFromBool(x_in));
    return .{
        .value = value,
        .cc = .{ .n = res & 0x80 != 0, .z = value == 0 and old_z, .v = res & 0x80 != 0 and raw & 0x80 == 0, .c = carry, .x = carry },
    };
}

/// SBCD: `a - b - x`, corrected against the plain binary subtraction (unlike
/// `bcdAdd`, which corrects a nibble-built intermediate). The value's high
/// correction (0x60) is gated on the raw unsigned byte borrow (`a < b + x`).
/// The C/X output is a *different*, more inclusive condition than that value
/// correction: it also fires when `a == b + x` exactly (raw == 0) but a low
/// nibble borrow still occurs — real hardware reports a carry-out there even
/// though the raw byte compare shows no borrow. Confirmed against
/// conformance data (register- and memory-form); not derivable from the
/// manual. V mirrors `bcdAdd`'s quirk in the other direction: set when the
/// correction pulls a negative raw subtraction back to non-negative.
pub fn bcdSub(a: u8, b: u8, x_in: bool, old_z: bool) BcdResult {
    const x: u8 = @intFromBool(x_in);
    const raw = a -% b -% x;
    const low_borrow = (a & 0xF) < (b & 0xF) + x;
    const full_borrow = @as(u16, a) < @as(u16, b) + x;
    var corf: u8 = 0;
    if (low_borrow) corf +%= 6;
    if (full_borrow) corf +%= 0x60;
    const value = raw -% corf;
    const corf_low: i16 = if (low_borrow) 6 else 0;
    const a_wide: i16 = @as(i16, a) - @as(i16, b) - x;
    const carry = a_wide < corf_low;
    return .{
        .value = value,
        .cc = .{ .n = value & 0x80 != 0, .z = value == 0 and old_z, .v = raw & 0x80 != 0 and value & 0x80 == 0, .c = carry, .x = carry },
    };
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

/// The eight shift/rotate variants, sharing one implementation since they
/// differ only in which bit feeds in and whether X participates.
pub const ShiftKind = enum { asl, asr, lsl, lsr, rol, ror, roxl, roxr };

pub const ShiftResult = struct {
    value: u32,
    n: bool,
    z: bool,
    v: bool,
    c: bool,
    /// `null` means "leave X alone": ROL/ROR never touch it, and ASx/LSx
    /// don't either at count 0 (only C is cleared then — a real hardware
    /// asymmetry with the count>0 case, where X mirrors C).
    x: ?bool,
};

/// Shifts/rotates one bit at a time, `count` times — matching the real
/// microcode (and its 2-cycles-per-count cost) rather than a closed-form
/// bit-twiddle, which would have to special-case ASL's V flag anyway (set if
/// the sign bit's value changes on *any* individual step, not just start vs
/// end).
pub fn shift(kind: ShiftKind, value: u32, count: u6, size: Size, x_in: bool) ShiftResult {
    const mask = size.mask();
    var v = value & mask;

    if (count == 0) {
        // Only ROX kinds still reflect X in C at count 0 (the rotate ring
        // includes X, so "no rotation" leaves C = X); every other kind
        // clears C outright. X itself is never touched by a count-0 shift.
        return .{
            .value = v,
            .n = isNeg(v, size),
            .z = v == 0,
            .v = false,
            .c = switch (kind) {
                .roxl, .roxr => x_in,
                else => false,
            },
            .x = null,
        };
    }

    const msb = size.msb();
    var carry: bool = x_in;
    var overflow = false;

    switch (kind) {
        .asl => {
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                const before = v & msb != 0;
                carry = before;
                v = (v << 1) & mask;
                if (before != (v & msb != 0)) overflow = true;
            }
        },
        .lsl => {
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                carry = v & msb != 0;
                v = (v << 1) & mask;
            }
        },
        .asr => {
            const sign = v & msb != 0;
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                carry = v & 1 != 0;
                v = ((v >> 1) | (if (sign) msb else 0)) & mask;
            }
        },
        .lsr => {
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                carry = v & 1 != 0;
                v = (v >> 1) & mask;
            }
        },
        .rol => {
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                const out = v & msb != 0;
                v = ((v << 1) | @intFromBool(out)) & mask;
                carry = out;
            }
        },
        .ror => {
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                const out = v & 1 != 0;
                v = ((v >> 1) | (if (out) msb else 0)) & mask;
                carry = out;
            }
        },
        .roxl => {
            var x = x_in;
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                const out = v & msb != 0;
                v = ((v << 1) | @intFromBool(x)) & mask;
                x = out;
            }
            carry = x;
        },
        .roxr => {
            var x = x_in;
            var i: u6 = 0;
            while (i < count) : (i += 1) {
                const out = v & 1 != 0;
                v = ((v >> 1) | (if (x) msb else 0)) & mask;
                x = out;
            }
            carry = x;
        },
    }

    return .{
        .value = v,
        .n = isNeg(v, size),
        .z = v == 0,
        .v = overflow,
        .c = carry,
        .x = switch (kind) {
            .rol, .ror => null,
            else => carry,
        },
    };
}

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

test "shift: count 0 clears C (and, for ROX, mirrors X) but never touches X itself" {
    const asl0 = shift(.asl, 0x12, 0, .byte, true);
    try std.testing.expect(!asl0.c and asl0.x == null and asl0.value == 0x12);

    const roxl0 = shift(.roxl, 0, 0, .byte, true);
    try std.testing.expect(roxl0.c and roxl0.x == null); // C mirrors X, X itself untouched
}

test "shift: asl sets V when the sign bit changes mid-shift, C/X to the last bit out" {
    // 0x40 << 1 = 0x80: crosses from positive to negative mid-shift.
    const cross = shift(.asl, 0x40, 1, .byte, false);
    try std.testing.expect(cross.v and cross.n and !cross.c);
    try std.testing.expectEqual(@as(bool, false), cross.x.?);

    // 0xC0 << 1 = 0x80: stays negative throughout, no overflow.
    const stay = shift(.asl, 0xC0, 1, .byte, false);
    try std.testing.expect(!stay.v and stay.c); // bit shifted out (bit7=1) is the carry
}

test "shift: asr sign-extends and never sets V" {
    const r = shift(.asr, 0x80, 4, .byte, false);
    try std.testing.expectEqual(@as(u32, 0xF8), r.value);
    try std.testing.expect(!r.v);
}

test "shift: rol/ror never write X even at count > 0" {
    const r = shift(.rol, 0x80, 1, .byte, false);
    try std.testing.expectEqual(@as(u32, 0x01), r.value);
    try std.testing.expect(r.c and r.x == null);
}

test "shift: roxl rotates X into the vacated bit and back out the other end" {
    // 0x00 rotated left through X=1: bit0 becomes 1, X becomes the old bit7 (0).
    const r = shift(.roxl, 0x00, 1, .byte, true);
    try std.testing.expectEqual(@as(u32, 0x01), r.value);
    try std.testing.expect(!r.c and r.x.? == false);
}

test "shift: lsr/lsl clear V and shift in zero" {
    const l = shift(.lsl, 0xFF, 1, .byte, false);
    try std.testing.expectEqual(@as(u32, 0xFE), l.value);
    try std.testing.expect(l.c and !l.v);

    const r = shift(.lsr, 0x01, 1, .byte, false);
    try std.testing.expectEqual(@as(u32, 0x00), r.value);
    try std.testing.expect(r.c and r.z);
}
