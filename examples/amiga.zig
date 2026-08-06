//! Boot a real Amiga ROM against the core.
//!
//!     tools/fetch_diagrom.sh        # once, downloads roms/diagrom.rom
//!     zig build amiga               # boot it
//!     zig build amiga -- other.rom 5  # another ROM, 5 emulated seconds
//!
//! Not an emulator: there is no Agnus, Denise or Paula here, just enough of an
//! A500 memory map for ROM code to run — chip RAM, the OVL bit that hides it
//! at reset, a beam position that advances with the cycle count, and the
//! serial transmitter wired to stdout. That is the whole point: the SST suite
//! proves each instruction in isolation, and this proves the core can carry
//! several hundred million of them in sequence without drifting.
//!
//! DiagROM (diagrom.com, freely distributable) is the intended target: it
//! narrates itself over the serial port, so its output is the test result.

const std = @import("std");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;
const Core = m68k.Core(Amiga);

const default_rom = "roms/diagrom.rom"; // tools/fetch_diagrom.sh puts it here

const chip_ram_bytes = 512 << 10;
const rom_bytes = 512 << 10;
const rom_base = 0xF8_0000;
/// OVL, set at reset, mirrors the ROM over the bottom megabyte so the vector
/// table the CPU fetches at address 0 is the ROM's.
const ovl_limit = 0x10_0000;

// PAL: 7.09 MHz CPU, two cycles per color clock, 227 color clocks a line.
const cpu_hz = 7_093_790.0;
const cycles_per_line = 454;
const lines_per_frame = 313;
const color_clocks_per_line = 227;

const Amiga = struct {
    rom: []const u8,
    chip: []u8,
    cpu: *const Cpu,
    ovl: bool = true,
    /// CIA-A port A: bit 0 is OVL, the rest read back as pulled high (no
    /// floppy, no fire buttons pressed).
    cia_a_pra: u8 = 0xFF,
    serial_bytes: u64 = 0,

    pub fn read8(b: *Amiga, addr: u24) u8 {
        return switch (addr) {
            0x00_0000...0x1F_FFFF => blk: {
                if (b.ovl and addr < ovl_limit) break :blk b.rom[addr % rom_bytes];
                break :blk b.chip[addr % chip_ram_bytes];
            },
            0xBF_0000...0xBF_FFFF => b.ciaRead(addr),
            0xDF_F000...0xDF_FFFF => @truncate(b.customRead(addr & 0x1FE) >> @intCast(8 * (~addr & 1))),
            rom_base...0xFF_FFFF => b.rom[(addr - rom_base) % rom_bytes],
            else => 0xFF, // unmapped: the data bus floats high
        };
    }

    pub fn read16(b: *Amiga, addr: u24) u16 {
        return switch (addr) {
            0x00_0000...0x1F_FFFF => blk: {
                if (b.ovl and addr < ovl_limit) break :blk rd16(b.rom, addr % rom_bytes);
                break :blk rd16(b.chip, addr % chip_ram_bytes);
            },
            0xDF_F000...0xDF_FFFF => b.customRead(addr & 0x1FE),
            rom_base...0xFF_FFFF => rd16(b.rom, (addr - rom_base) % rom_bytes),
            // A CIA sits on half the data bus, so the other half floats.
            else => @as(u16, b.read8(addr)) << 8 | 0xFF,
        };
    }

    pub fn write8(b: *Amiga, addr: u24, v: u8) void {
        switch (addr) {
            0x00_0000...0x1F_FFFF => {
                if (b.ovl and addr < ovl_limit) return; // ROM under the overlay
                b.chip[addr % chip_ram_bytes] = v;
            },
            0xBF_0000...0xBF_FFFF => b.ciaWrite(addr, v),
            0xDF_F000...0xDF_FFFF => {
                // Byte writes to a custom register still present a full word;
                // only the serial data register cares what is in it.
                b.customWrite(addr & 0x1FE, v);
            },
            else => {},
        }
    }

    pub fn write16(b: *Amiga, addr: u24, v: u16) void {
        switch (addr) {
            0x00_0000...0x1F_FFFF => {
                if (b.ovl and addr < ovl_limit) return;
                wr16(b.chip, addr % chip_ram_bytes, v);
            },
            0xDF_F000...0xDF_FFFF => b.customWrite(addr & 0x1FE, v),
            else => b.write8(addr, @truncate(v >> 8)),
        }
    }

    // ------------------------------------------------------------------- CIAs

    /// Both CIAs decode their register from A8-A11. A12 low selects CIA-A (odd
    /// addresses, $BFEx01), A13 low selects CIA-B (even ones, $BFDx00).
    fn ciaRead(b: *Amiga, addr: u24) u8 {
        const reg = (addr >> 8) & 0xF;
        const is_a = addr & 0x1000 == 0;
        return switch (reg) {
            // Only OVL and the power LED are outputs; the floppy status lines
            // and both fire buttons are inputs, idle high.
            0x0 => if (is_a) b.cia_a_pra & 0x03 | 0xFC else 0xFF,
            0xD => 0x00, // ICR: nothing pending, and reading clears it anyway
            else => 0xFF,
        };
    }

    fn ciaWrite(b: *Amiga, addr: u24, v: u8) void {
        const reg = (addr >> 8) & 0xF;
        const is_a = addr & 0x1000 == 0;
        if (is_a and reg == 0x0) {
            b.cia_a_pra = v;
            b.ovl = v & 1 != 0;
        }
    }

    // ----------------------------------------------------------------- custom

    fn customRead(b: *Amiga, reg: u24) u16 {
        const cycles = b.cpu.cycles;
        const vpos: u16 = @intCast((cycles / cycles_per_line) % lines_per_frame);
        const hpos: u16 = @intCast((cycles / 2) % color_clocks_per_line);
        return switch (reg) {
            0x002 => 0x0000, // DMACONR
            0x004 => vpos >> 8, // VPOSR: V8 only, short frames
            0x006 => vpos << 8 | hpos, // VHPOSR
            0x010 => 0x0000, // ADKCONR
            0x018 => 0x3000, // SERDATR: TBE | TSRE, transmitter always idle
            0x01C => 0x0000, // INTENAR
            0x01E => 0x0001, // INTREQR: TBE
            else => 0x0000,
        };
    }

    fn customWrite(b: *Amiga, reg: u24, v: u16) void {
        if (reg != 0x030) return; // SERDAT
        // The stop bit rides above the data; everything below it is the byte.
        std.debug.print("{c}", .{@as(u8, @truncate(v))});
        b.serial_bytes += 1;
    }
};

fn rd16(mem: []const u8, addr: usize) u16 {
    return std.mem.readInt(u16, mem[addr..][0..2], .big);
}

fn wr16(mem: []u8, addr: usize, v: u16) void {
    std.mem.writeInt(u16, mem[addr..][0..2], v, .big);
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;

    var args = std.process.Args.Iterator.init(init.minimal.args);
    _ = args.skip();
    const path = args.next() orelse default_rom;
    // Wall-clock seconds of emulated CPU time, at the PAL 7.09 MHz clock.
    // DiagROM needs about 25 to finish its power-on sequence.
    const seconds = if (args.next()) |s| try std.fmt.parseFloat(f64, s) else 30.0;

    const image = std.Io.Dir.cwd().readFileAlloc(io, path, gpa, .limited(2 << 20)) catch |err| {
        std.debug.print(
            "cannot read {s}: {t}\nRun tools/fetch_diagrom.sh first, or pass a ROM path.\n",
            .{ path, err },
        );
        return err;
    };
    defer gpa.free(image);
    if (image.len < rom_bytes) {
        std.debug.print("{s}: {d} bytes, need at least {d}\n", .{ path, image.len, rom_bytes });
        return error.RomTooSmall;
    }

    const chip = try gpa.alloc(u8, chip_ram_bytes);
    defer gpa.free(chip);
    @memset(chip, 0);

    var c = Cpu{};
    var bus = Amiga{ .rom = image[0..rom_bytes], .chip = chip, .cpu = &c };
    Core.reset(&c, &bus);
    std.debug.print("reset: pc={x:0>6} ssp={x:0>8}\n", .{ c.pc, c.a[7] });

    const budget: u64 = @intFromFloat(seconds * cpu_hz);
    // A line at a time: nothing here needs finer granularity, and a halted CPU
    // (double bus fault) is worth stopping for rather than idling out the clock.
    while (c.cycles < budget and !c.halted) _ = Core.run(&c, &bus, cycles_per_line);

    std.debug.print("\n--- stopped after {d} cycles ({d:.2}s emulated)\n", .{
        c.cycles, @as(f64, @floatFromInt(c.cycles)) / cpu_hz,
    });
    std.debug.print("pc={x:0>6} sr={x:0>4} halted={} stopped={} serial={d} bytes\n", .{
        c.pc, c.sr.toInt(), c.halted, c.stopped, bus.serial_bytes,
    });
}
