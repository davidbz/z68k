//! z68k — a Motorola 68000 core.
//!
//! Data lives in `cpu` and `decode`; logic lives in `core`, `flags` and
//! `disasm`. Nothing here allocates. See DESIGN.md.
//!
//!     var bus = FlatBus{ .ram = ram };
//!     const M68k = core.Core(FlatBus);
//!     var c: Cpu = .{};
//!     M68k.reset(&c, &bus);
//!     _ = M68k.run(&c, &bus, 100_000);

const std = @import("std");

pub const cpu = @import("cpu.zig");
pub const decode = @import("decode.zig");
pub const flags = @import("flags.zig");
pub const core = @import("core.zig");
pub const disasm = @import("disasm.zig");

pub const Cpu = cpu.Cpu;
pub const StatusRegister = cpu.StatusRegister;
pub const Size = cpu.Size;
pub const Exception = cpu.Exception;
pub const Core = core.Core;
pub const FlatBus = core.FlatBus;

test {
    std.testing.refAllDecls(@This());
}
