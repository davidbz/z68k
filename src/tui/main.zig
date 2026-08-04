//! Debugger TUI.
//!
//!     zig build dbg -- program.bin [load-address]
//!
//! Uses libvaxis's low-level API rather than the vxfw widget framework: a
//! debugger wants exact control over a fixed pane layout, and none of the
//! widgets fit that better than writing the cells directly.

const std = @import("std");
const vaxis = @import("vaxis");
const m68k = @import("m68k");

const Cpu = m68k.Cpu;
const FlatBus = m68k.FlatBus;
const Core = m68k.Core(FlatBus);

const ram_size = 0x100_0000;
const default_load = 0x400;

/// Everything the debugger knows that the CPU does not.
const Debug = struct {
    cpu: Cpu = .{},
    /// Snapshot from before the last step, for change highlighting.
    prev: Cpu = .{},
    breakpoints: std.ArrayList(u32) = .empty,
    mem_cursor: u32 = 0,
    running: bool = false,
    quit: bool = false,

    fn hasBreakpoint(d: *const Debug, addr: u32) bool {
        return std.mem.indexOfScalar(u32, d.breakpoints.items, addr) != null;
    }

    fn toggleBreakpoint(d: *Debug, gpa: std.mem.Allocator, addr: u32) !void {
        if (std.mem.indexOfScalar(u32, d.breakpoints.items, addr)) |i| {
            _ = d.breakpoints.orderedRemove(i);
        } else {
            try d.breakpoints.append(gpa, addr);
        }
    }

    fn step(d: *Debug, bus: *FlatBus) void {
        d.prev = d.cpu;
        Core.step(&d.cpu, bus);
    }

    /// One instruction at a time so breakpoints are checked between them.
    fn resume_(d: *Debug, bus: *FlatBus, max: usize) void {
        for (0..max) |_| {
            d.step(bus);
            if (d.cpu.halted or d.hasBreakpoint(d.cpu.pc)) {
                d.running = false;
                return;
            }
        }
    }
};

const changed = vaxis.Style{ .fg = .{ .index = 3 }, .bold = true };
const dim = vaxis.Style{ .fg = .{ .index = 8 } };
const accent = vaxis.Style{ .fg = .{ .index = 4 } };

// vaxis cells keep slices into the text they are given until `vx.render`, so
// every line drawn in a frame must stay alive for the whole frame: draw
// functions allocate from a per-frame arena instead of stack buffers.
fn drawRegisters(win: vaxis.Window, d: *const Debug, fa: std.mem.Allocator) void {
    for (0..8) |i| {
        const line = std.fmt.allocPrint(fa, "D{d} {x:0>8}  A{d} {x:0>8}", .{
            i, d.cpu.d[i], i, d.cpu.a[i],
        }) catch continue;
        const style: vaxis.Style = if (d.cpu.d[i] != d.prev.d[i] or d.cpu.a[i] != d.prev.a[i]) changed else .{};
        _ = win.printSegment(.{ .text = line, .style = style }, .{ .row_offset = @intCast(i) });
    }

    const sr = d.cpu.sr;
    const pc = std.fmt.allocPrint(fa, "PC {x:0>8}", .{d.cpu.pc}) catch return;
    _ = win.printSegment(.{ .text = pc, .style = accent }, .{ .row_offset = 9 });

    const f = std.fmt.allocPrint(fa, "SR {x:0>4} {c}{c}{d} {c}{c}{c}{c}{c}", .{
        sr.toInt(),
        @as(u8, if (sr.t) 'T' else '-'),
        @as(u8, if (sr.s) 'S' else '-'),
        sr.ipl,
        @as(u8, if (sr.x) 'X' else '-'),
        @as(u8, if (sr.n) 'N' else '-'),
        @as(u8, if (sr.z) 'Z' else '-'),
        @as(u8, if (sr.v) 'V' else '-'),
        @as(u8, if (sr.c) 'C' else '-'),
    }) catch return;
    _ = win.printSegment(.{ .text = f }, .{ .row_offset = 10 });

    if (d.cpu.halted) {
        _ = win.printSegment(.{ .text = "HALTED", .style = .{ .fg = .{ .index = 1 } } }, .{ .row_offset = 12 });
    } else if (d.cpu.stopped) {
        _ = win.printSegment(.{ .text = "STOPPED", .style = dim }, .{ .row_offset = 12 });
    }
}

fn drawDisasm(win: vaxis.Window, d: *const Debug, bus: *FlatBus, fa: std.mem.Allocator) void {
    const Reader = struct {
        var b: *FlatBus = undefined;
        fn read(addr: u32) u16 {
            if (addr + 1 >= b.ram.len) return 0;
            return b.read16(@truncate(addr & ~@as(u32, 1)));
        }
    };
    Reader.b = bus;

    var addr = d.cpu.pc;
    var row: u16 = 0;
    while (row < win.height) : (row += 1) {
        const line = m68k.disasm.disasm(Reader.read, addr);
        const text = std.fmt.allocPrint(fa, "{c}{c} {x:0>6}: {s}", .{
            @as(u8, if (d.hasBreakpoint(addr)) '*' else ' '),
            @as(u8, if (addr == d.cpu.pc) '>' else ' '),
            addr,
            line.text(),
        }) catch continue;
        const style: vaxis.Style = if (addr == d.cpu.pc) accent else .{};
        _ = win.printSegment(.{ .text = text, .style = style }, .{ .row_offset = row });
        addr +%= line.bytes;
    }
}

fn drawMemory(win: vaxis.Window, d: *const Debug, bus: *const FlatBus, fa: std.mem.Allocator) void {
    var row: u16 = 0;
    while (row < win.height) : (row += 1) {
        const base = d.mem_cursor +% row * 16;
        if (base + 16 > bus.ram.len) return;

        var line: [64]u8 = undefined;
        var used = (std.fmt.bufPrint(&line, "{x:0>6}:", .{base}) catch return).len;
        for (bus.ram[base..][0..16]) |byte| {
            used += (std.fmt.bufPrint(line[used..], " {x:0>2}", .{byte}) catch return).len;
        }
        const text = fa.dupe(u8, line[0..used]) catch return;
        _ = win.printSegment(.{ .text = text }, .{ .row_offset = row });
    }
}

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;

    var args = std.process.Args.Iterator.init(init.minimal.args);
    _ = args.skip();
    const program = args.next();
    const load_addr: u32 = if (args.next()) |a|
        std.fmt.parseInt(u32, a, 0) catch default_load
    else
        default_load;

    const ram = try gpa.alloc(u8, ram_size);
    defer gpa.free(ram);
    @memset(ram, 0);
    var bus = FlatBus{ .ram = ram };

    var d = Debug{};
    defer d.breakpoints.deinit(gpa);

    if (program) |path| {
        const image = try std.Io.Dir.cwd().readFileAlloc(io, path, gpa, .limited(ram_size));
        defer gpa.free(image);
        @memcpy(ram[load_addr..][0..image.len], image);
        d.cpu.pc = load_addr;
        d.cpu.a[7] = load_addr -% 0x100;
    } else {
        // No program: sit at the reset vector so the panes have something real.
        Core.reset(&d.cpu, &bus);
    }
    d.prev = d.cpu;
    d.mem_cursor = d.cpu.pc & ~@as(u32, 0xF);

    var tty_buf: [4096]u8 = undefined;
    var tty = try vaxis.Tty.init(io, &tty_buf);
    defer tty.deinit();

    var vx = try vaxis.init(io, gpa, init.environ_map, .{});
    defer vx.deinit(gpa, tty.writer());

    var loop: vaxis.Loop(vaxis.Event) = .init(io, &tty, &vx);
    try loop.start();
    defer loop.stop();

    try vx.enterAltScreen(tty.writer());
    try vx.queryTerminal(tty.writer(), .fromNanoseconds(1 * std.time.ns_per_s));

    // Frame text lives here; see drawRegisters.
    var arena = std.heap.ArenaAllocator.init(gpa);
    defer arena.deinit();

    while (!d.quit) {
        const event = try loop.nextEvent();
        switch (event) {
            .key_press => |key| {
                if (key.matches('q', .{}) or key.matches('c', .{ .ctrl = true })) d.quit = true;
                if (key.matches('s', .{})) d.step(&bus);
                if (key.matches('r', .{})) {
                    d.running = true;
                    d.resume_(&bus, 1_000_000);
                }
                if (key.matches('b', .{})) try d.toggleBreakpoint(gpa, d.cpu.pc);
                if (key.matches('m', .{})) d.mem_cursor +%= 0x100;
                if (key.matches('n', .{})) d.mem_cursor -%= 0x100;
                if (key.matches('g', .{})) d.mem_cursor = d.cpu.pc & ~@as(u32, 0xF);
            },
            .winsize => |ws| try vx.resize(gpa, tty.writer(), ws),
            else => {},
        }

        _ = arena.reset(.retain_capacity);
        const fa = arena.allocator();

        const win = vx.window();
        win.clear();

        const body_h = if (win.height > 12) win.height - 12 else 1;

        const regs = win.child(.{
            .width = 30,
            .height = body_h,
            .border = .{ .where = .all, .style = dim },
        });
        drawRegisters(regs, &d, fa);

        const code = win.child(.{
            .x_off = 30,
            .width = if (win.width > 30) win.width - 30 else 1,
            .height = body_h,
            .border = .{ .where = .all, .style = dim },
        });
        drawDisasm(code, &d, &bus, fa);

        const mem = win.child(.{
            .y_off = body_h,
            .height = if (win.height > body_h + 1) win.height - body_h - 1 else 1,
            .border = .{ .where = .all, .style = dim },
        });
        drawMemory(mem, &d, &bus, fa);

        var status: [128]u8 = undefined;
        const s = std.fmt.bufPrint(&status, " cycles {d}  bp {d}   [s]tep [r]un [b]reak [m/n] mem [g]oto pc [q]uit", .{
            d.cpu.cycles, d.breakpoints.items.len,
        }) catch " ";
        _ = win.printSegment(.{ .text = s, .style = dim }, .{ .row_offset = win.height -| 1 });

        try vx.render(tty.writer());
    }
}
