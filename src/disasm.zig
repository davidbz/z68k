//! Disassembler. Pure and independent of `Cpu`: it reads words through a
//! caller-supplied closure, so the debugger can disassemble a memory snapshot
//! without a running core.
//!
//! Shares `decode.decodeOne`, so an instruction added to the decoder shows up
//! here as soon as it gets a formatting case.

const std = @import("std");
const cpu = @import("cpu.zig");
const decode = @import("decode.zig");

const Size = cpu.Size;
const EaMode = decode.EaMode;

pub const Line = struct {
    buf: [64]u8 = undefined,
    len: u8 = 0,
    /// Instruction length in bytes, for advancing to the next one.
    bytes: u8 = 2,

    pub fn text(l: *const Line) []const u8 {
        return l.buf[0..l.len];
    }
};

/// `read16` takes an address and returns the word there.
pub fn disasm(read16: anytype, pc: u32) Line {
    var line = Line{};
    const op = read16(pc);
    const instr = decode.table[op];

    const suffix: []const u8 = switch (instr.size) {
        .byte => "b",
        .word => "w",
        .long => "l",
    };

    // TODO(M6): operand formatting. Until each family has a case here the
    // mnemonic alone is still more useful than a raw word.
    const s = switch (instr.mnemonic) {
        .illegal => std.fmt.bufPrint(&line.buf, "dc.w    ${x:0>4}", .{op}),
        .nop => std.fmt.bufPrint(&line.buf, "nop", .{}),
        .rts => std.fmt.bufPrint(&line.buf, "rts", .{}),
        .moveq => std.fmt.bufPrint(&line.buf, "moveq   #{d},d{d}", .{
            @as(i8, @bitCast(@as(u8, @truncate(op)))),
            @as(u3, @truncate(op >> 9)),
        }),
        else => std.fmt.bufPrint(&line.buf, "{s}.{s}", .{ @tagName(instr.mnemonic), suffix }),
    } catch line.buf[0..0];

    line.len = @intCast(s.len);
    return line;
}

test "disassembles what it knows and falls back to dc.w" {
    const words = struct {
        fn read(addr: u32) u16 {
            return switch (addr) {
                0 => 0x4E71, // nop
                2 => 0x70FF, // moveq #-1,d0
                else => 0x4E40, // trap, not decoded yet (TODO M4)
            };
        }
    }.read;

    try std.testing.expectEqualStrings("nop", disasm(words, 0).text());
    try std.testing.expectEqualStrings("moveq   #-1,d0", disasm(words, 2).text());
    try std.testing.expectEqualStrings("dc.w    $4e40", disasm(words, 4).text());
}
