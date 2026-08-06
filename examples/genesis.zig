//! Boot a real Genesis / Mega Drive ROM against the core, and draw it.
//!
//!     zig build genesis -Doptimize=ReleaseFast              # roms/Sonic-the-Hedgehog.bin
//!     zig build genesis -- roms/other.bin
//!     zig build genesis -- --shot 600 shot.png              # headless: N frames, then a PNG
//!
//! The machine the CPU was written for. This is the whole rest of it apart
//! from sound: cartridge, 64 KiB of work RAM, the VDP in `genesis_vdp.zig`,
//! a controller, and the interrupt and frame timing that ties them together.
//! The Z80, YM2612 and PSG are stubs — silent, but present enough that the
//! ROM's handshakes complete instead of hanging.
//!
//! Keys: arrows, A/S/D = A/B/C, Enter = Start.

const std = @import("std");
const m68k = @import("m68k");
const vdp = @import("genesis_vdp.zig");

const rl = @cImport(@cInclude("raylib.h"));

const Cpu = m68k.Cpu;
const Core = m68k.Core(Genesis);

const default_rom = "roms/Sonic-the-Hedgehog.bin";
const scale = 3;

// NTSC: a 53.693175 MHz master clock, seven of which make a 68000 cycle, and
// 3420 of which make a scanline.
const mclk_per_cpu = 7;
const mclk_per_line = 3420;
const lines_per_frame = 262;
const cpu_hz = 53_693_175.0 / @as(f64, mclk_per_cpu);

// Controller bits, active high here and inverted on the way out to the ROM.
const btn_up = 0x01;
const btn_down = 0x02;
const btn_left = 0x04;
const btn_right = 0x08;
const btn_b = 0x10;
const btn_c = 0x20;
const btn_a = 0x40;
const btn_start = 0x80;

const Genesis = struct {
    rom: []const u8,
    cpu: *const Cpu,
    ram: [64 << 10]u8 = @splat(0),
    /// The Z80's RAM. Nothing runs it; the 68k still fills it with a driver.
    zram: [8 << 10]u8 = @splat(0),
    v: vdp.Vdp = .{},

    buttons: u8 = 0,
    pad_ctrl: u8 = 0,
    pad_data: u8 = 0,

    line: u32 = 0,
    /// Cycle count when the current scanline started, for the H counter.
    line_start: u64 = 0,
    /// A scanline isn't a whole number of CPU cycles; the remainder is carried
    /// here so a frame comes out at the right length instead of 0.1% short.
    mclk_debt: u64 = 0,

    // -------------------------------------------------------------- memory map

    pub fn read8(g: *Genesis, addr: u24) u8 {
        return switch (addr) {
            0x00_0000...0x3F_FFFF => if (addr < g.rom.len) g.rom[addr] else 0xFF,
            0xA0_0000...0xA0_3FFF => g.zram[addr & 0x1FFF],
            0xA0_4000...0xA0_5FFF => 0, // YM2612: never busy
            0xA1_0000...0xA1_001F => g.ioRead(addr),
            // Z80 bus request and reset. The bus is always granted at once:
            // bit 0 low means the 68k has it.
            0xA1_1100...0xA1_11FF => 0x00,
            0xC0_0000...0xC0_000F => @truncate(g.read16(addr & ~@as(u24, 1)) >> @intCast(8 * (~addr & 1))),
            0xE0_0000...0xFF_FFFF => g.ram[addr & 0xFFFF],
            else => 0xFF,
        };
    }

    pub fn read16(g: *Genesis, addr: u24) u16 {
        return switch (addr) {
            0x00_0000...0x3F_FFFF => if (addr + 1 < g.rom.len) rd16(g.rom, addr) else 0xFFFF,
            0xA0_0000...0xA0_3FFF => rd16(&g.zram, addr & 0x1FFF),
            0xC0_0000...0xC0_0003 => g.v.readData(),
            0xC0_0004...0xC0_0007 => g.v.readStatus(),
            0xC0_0008...0xC0_000F => g.hvCounter(),
            0xE0_0000...0xFF_FFFF => rd16(&g.ram, addr & 0xFFFF),
            // Everything else on this bus is a byte-wide device, and answers
            // a word read with the same byte in both halves.
            else => @as(u16, g.read8(addr)) << 8 | g.read8(addr),
        };
    }

    pub fn write8(g: *Genesis, addr: u24, val: u8) void {
        switch (addr) {
            0xA0_0000...0xA0_3FFF => g.zram[addr & 0x1FFF] = val,
            0xA1_0000...0xA1_001F => g.ioWrite(addr, val),
            // A byte write to a VDP port still presents a full word, with the
            // byte in both halves.
            0xC0_0000...0xC0_000F => g.write16(addr & ~@as(u24, 1), @as(u16, val) << 8 | val),
            0xE0_0000...0xFF_FFFF => g.ram[addr & 0xFFFF] = val,
            else => {},
        }
    }

    pub fn write16(g: *Genesis, addr: u24, val: u16) void {
        switch (addr) {
            0xA0_0000...0xA0_3FFF => wr16(&g.zram, addr & 0x1FFF, val),
            0xC0_0000...0xC0_0003 => g.v.writeData(val),
            0xC0_0004...0xC0_0007 => {
                g.v.writeControl(val);
                if (g.v.dma_request) g.dmaFrom68k();
            },
            0xE0_0000...0xFF_FFFF => wr16(&g.ram, addr & 0xFFFF, val),
            else => g.write8(addr, @truncate(val >> 8)),
        }
    }

    /// The one DMA mode the VDP can't run on its own: its source is out here.
    fn dmaFrom68k(g: *Genesis) void {
        g.v.dma_request = false;
        var src = g.v.dmaSource();
        var len = g.v.dmaLength();
        while (len > 0) : (len -= 1) {
            g.v.writeTarget(g.read16(@truncate(src)));
            // The source counter wraps inside its own 128 KiB bank.
            src = (src & 0xFF_0000) | ((src + 2) & 0xFFFF);
        }
        g.v.dmaDone();
    }

    // --------------------------------------------------------------------- I/O

    /// The H counter runs 0..$FF across a line here rather than the real
    /// blanking-aware ramp; games use it as an entropy source, not a clock.
    fn hvCounter(g: *Genesis) u16 {
        const into_line = g.cpu.cycles -| g.line_start;
        const h: u16 = @truncate(into_line * 256 / (mclk_per_line / mclk_per_cpu));
        return @as(u16, @truncate(g.line)) << 8 | (h & 0xFF);
    }

    fn ioRead(g: *Genesis, addr: u24) u8 {
        return switch (addr & 0x1F) {
            0x01 => 0xA0, // export, NTSC, no expansion, VA0
            0x03 => g.padByte(),
            0x05, 0x07 => if (g.pad_ctrl & 0x40 != 0) 0x7F else 0x3F, // no pad 2 or 3
            0x09 => g.pad_ctrl,
            else => 0x00,
        };
    }

    fn ioWrite(g: *Genesis, addr: u24, val: u8) void {
        switch (addr & 0x1F) {
            0x03 => g.pad_data = val,
            0x09 => g.pad_ctrl = val,
            else => {},
        }
    }

    /// Three-button pad. TH is an output the ROM toggles to pick which half of
    /// the pad it sees; every button reads low when pressed.
    fn padByte(g: *Genesis) u8 {
        const th = g.pad_ctrl & 0x40 == 0 or g.pad_data & 0x40 != 0;
        const b = g.buttons;
        const low: u8 = if (th)
            b & 0x3F // C B R L D U
        else
            (b & (btn_up | btn_down)) | ((b & btn_a) >> 2) | ((b & btn_start) >> 2);
        return (if (th) @as(u8, 0x40) else 0) | (~low & 0x3F);
    }

    // ----------------------------------------------------------------- timing

    fn runFrame(g: *Genesis, c: *Cpu) void {
        g.line = 0;
        while (g.line < lines_per_frame) : (g.line += 1) {
            // The H-int counter reloads throughout blanking and counts down
            // once per line over the active display.
            if (g.line <= vdp.height) {
                if (g.v.hint_counter == 0) {
                    g.v.hint_counter = g.v.hintReload();
                    g.v.hint_irq = true;
                } else g.v.hint_counter -= 1;
            } else g.v.hint_counter = g.v.hintReload();

            g.mclk_debt += mclk_per_line;
            const budget = g.mclk_debt / mclk_per_cpu;
            g.mclk_debt %= mclk_per_cpu;
            g.line_start = c.cycles;
            g.runLine(c, budget);

            if (g.line < vdp.height) g.v.renderLine(g.line);
            if (g.line == vdp.height) {
                g.v.in_vblank = true;
                g.v.vint_flag = true;
                g.v.vint_irq = true;
            }
            if (g.line == lines_per_frame - 1) g.v.in_vblank = false;
        }
    }

    fn runLine(g: *Genesis, c: *Cpu, budget: u64) void {
        const end = c.cycles + budget;
        while (c.cycles < end and !c.halted) {
            const level: u3 = if (g.v.vint_irq and g.v.vintEnabled())
                6
            else if (g.v.hint_irq and g.v.hintEnabled())
                4
            else
                0;
            Core.setIpl(c, level);
            const mask_before = c.sr.ipl;
            Core.step(c, g);
            // ponytail: the acknowledge is inferred rather than driven — the
            // core has no ack hook, so an interrupt counts as taken once the
            // mask has risen to its level and PC has landed on the handler the
            // autovector names. The VDP is the only source, so nothing else is
            // in a position to fake that.
            if (level != 0 and mask_before < level and c.sr.ipl == level and
                c.pc == g.vectorFor(level))
            {
                if (level == 6) g.v.vint_irq = false else g.v.hint_irq = false;
            }
        }
    }

    fn vectorFor(g: *Genesis, level: u3) u32 {
        const at: u24 = @intCast(m68k.Exception.autovector(level).vectorAddr());
        return @as(u32, g.read16(at)) << 16 | g.read16(at + 2);
    }
};

fn rd16(mem: []const u8, addr: usize) u16 {
    return std.mem.readInt(u16, mem[addr..][0..2], .big);
}

fn wr16(mem: []u8, addr: usize, val: u16) void {
    std.mem.writeInt(u16, mem[addr..][0..2], val, .big);
}

fn pollInput(g: *Genesis) void {
    var b: u8 = 0;
    if (rl.IsKeyDown(rl.KEY_UP)) b |= btn_up;
    if (rl.IsKeyDown(rl.KEY_DOWN)) b |= btn_down;
    if (rl.IsKeyDown(rl.KEY_LEFT)) b |= btn_left;
    if (rl.IsKeyDown(rl.KEY_RIGHT)) b |= btn_right;
    if (rl.IsKeyDown(rl.KEY_A)) b |= btn_a;
    if (rl.IsKeyDown(rl.KEY_S)) b |= btn_b;
    if (rl.IsKeyDown(rl.KEY_D)) b |= btn_c;
    if (rl.IsKeyDown(rl.KEY_ENTER)) b |= btn_start;
    g.buttons = b;
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;

    var args = std.process.Args.Iterator.init(init.minimal.args);
    _ = args.skip();
    var path: []const u8 = default_rom;
    var shot_frames: ?u32 = null;
    var shot_path: [:0]const u8 = "shot.png";
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "--shot")) {
            shot_frames = try std.fmt.parseInt(u32, args.next() orelse "60", 10);
            if (args.next()) |p| shot_path = try gpa.dupeZ(u8, p);
        } else path = arg;
    }

    const image = std.Io.Dir.cwd().readFileAlloc(io, path, gpa, .limited(4 << 20)) catch |err| {
        std.debug.print("cannot read {s}: {t}\n", .{ path, err });
        return err;
    };
    defer gpa.free(image);

    var c = Cpu{};
    // A megabyte of VDP and RAM: too much for the stack.
    const g = try gpa.create(Genesis);
    defer gpa.destroy(g);
    g.* = .{ .rom = image, .cpu = &c };

    Core.reset(&c, g);
    std.debug.print("{s}: {d} KiB, reset pc={x:0>6} sp={x:0>8}\n", .{
        path, image.len >> 10, c.pc, c.a[7],
    });

    var frames: u32 = 0;
    if (shot_frames) |n| {
        // Headless: no window, no GL — ExportImage only touches the pixels.
        while (frames < n and !c.halted) : (frames += 1) g.runFrame(&c);
        _ = rl.ExportImage(.{
            .data = &g.v.fb,
            .width = vdp.width,
            .height = vdp.height,
            .mipmaps = 1,
            .format = rl.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
        }, shot_path.ptr);
    } else {
        rl.InitWindow(vdp.width * scale, vdp.height * scale, "z68k — Genesis");
        if (!rl.IsWindowReady()) {
            // Closing a window that never opened is a segfault, so leave first.
            std.debug.print("no window (no display?); try --shot N out.png instead\n", .{});
            return error.NoDisplay;
        }
        defer rl.CloseWindow();
        rl.SetTargetFPS(60);
        const tex = rl.LoadTextureFromImage(.{
            .data = &g.v.fb,
            .width = vdp.width,
            .height = vdp.height,
            .mipmaps = 1,
            .format = rl.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
        });
        while (!rl.WindowShouldClose() and !c.halted) : (frames += 1) {
            pollInput(g);
            g.runFrame(&c);
            rl.UpdateTexture(tex, &g.v.fb);
            rl.BeginDrawing();
            rl.DrawTexturePro(
                tex,
                .{ .x = 0, .y = 0, .width = vdp.width, .height = vdp.height },
                .{ .x = 0, .y = 0, .width = vdp.width * scale, .height = vdp.height * scale },
                .{ .x = 0, .y = 0 },
                0,
                .{ .r = 255, .g = 255, .b = 255, .a = 255 },
            );
            rl.EndDrawing();
        }
    }

    std.debug.print("{d} frames, {d} cycles ({d:.2}s emulated), pc={x:0>6} sr={x:0>4} halted={}\n", .{
        frames, c.cycles, @as(f64, @floatFromInt(c.cycles)) / cpu_hz, c.pc, c.sr.toInt(), c.halted,
    });
}
