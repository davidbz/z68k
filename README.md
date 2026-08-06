# z68k

A Motorola 68000 CPU emulator written in Zig, built as a standalone, reusable
library. Intended to power a Sega Genesis / Mega Drive emulator, but has no
knowledge of any particular machine: the host supplies the bus.

Accuracy is validated against the MAME-generated
[SingleStepTests/m68000](https://github.com/SingleStepTests/m68000) suite, at
the level of architectural state (registers, SR, PC, memory) and total
per-instruction cycle counts. Per-cycle bus activity isn't modelled; a
prefetch-queue approximation reproduces the fault-frame effects it has,
including the group 0 exception frame's PC, instruction register and
mid-microcode CCR. See [DESIGN.md](DESIGN.md) for the rationale.

## Status

The instruction set is complete and passes the conformance suite outright:
**all 127 files, 317,500 cases, exact on both architectural state and cycle
counts**, with nothing excluded, skipped or masked. That covers every family
— MOVE/MOVEA/MOVEQ, LEA, branches and returns, NEG/NEGX/NOT/CLR/TST,
Scc/DBcc, ADD/SUB/AND/OR/EOR/CMP and their address forms, ADDX/SUBX/CMPM, the
immediate/CCR/SR logic ops, ASx/LSx/ROx/ROXx, BTST/BCHG/BCLR/BSET, MOVEP,
ABCD/SBCD/NBCD, EXG, MULU/MULS/DIVU/DIVS, CHK, EXT, JMP/JSR, LINK/UNLK,
MOVE to/from CCR/SR/USP, MOVEM, PEA, RESET, RTE, RTR, STOP, SWAP, TAS, TRAP,
TRAPV — plus the exception machinery and data-dependent divide timing.

Remaining work (milestone M6): the disassembler prints mnemonics without
operands, so the debugger TUI is usable but rough.

## Requirements

- Zig 0.16.0

No other dependencies for the core. The debugger TUI pulls in
[libvaxis](https://github.com/rockorager/libvaxis) via the Zig package manager.

## Building

```sh
zig build          # library + executables into zig-out/
zig build test     # unit tests
```

## Running the conformance suite

The test data (~2 GB) is fetched once into `testdata/` (gitignored):

```sh
tools/fetch_tests.sh
zig build sst              # run everything
zig build sst -- MOVE      # only files whose name contains "MOVE"
```

The suite reports two tiers per file — `state` (full architectural match) and
`cycles` (exact cycle count) — and fails if either falls short of every case.
Nothing is masked or excused, so any regression turns it red.

## Running the debugger

```sh
zig build dbg -- program.bin [load-address]
```

Loads a raw binary (default address `0x400`) into a flat 16 MiB RAM and opens
a TUI with register, disassembly, and memory panes. Keys: `s` step, `r` run,
`b` toggle breakpoint at PC, `m`/`n` scroll memory, `g` jump memory view to PC,
`q` quit.

## Using the library

The core is a dependency-free Zig module named `m68k`. It never allocates; the
host owns the memory and implements the bus as a plain struct — no vtable, the
core is generic over the bus type.

```zig
const m68k = @import("m68k");

// FlatBus.ram is []u8, host-owned; addr is u24 so 16 MiB is the max, not a
// minimum. reset() reads SSP from ram[0..4] and PC from ram[4..8], so both
// the reset vector and the program need to be loaded before calling it.
var ram = try allocator.alloc(u8, 16 * 1024 * 1024);
var bus = m68k.FlatBus{ .ram = ram };   // or any type with read8/16, write8/16
const Core = m68k.Core(m68k.FlatBus);

var cpu = m68k.Cpu{};
Core.reset(&cpu, &bus);
_ = Core.run(&cpu, &bus, 100_000);      // run a ~100k cycle timeslice
```

`Cpu` is plain copyable data: `const snapshot = cpu;` is a complete save state.

## Architecture

```
src/root.zig       public surface
src/cpu.zig        Cpu, StatusRegister, Size, Exception     (data)
src/decode.zig     EaMode, comptime 64K-entry decode table  (data + pure fns)
src/flags.zig      condition-code computation               (pure fns)
src/core.zig       Core(BusT): step/run/reset/EA/exceptions (logic)
src/disasm.zig     disassembler                             (pure fns)
src/harness.zig    SingleStepTests conformance runner
src/tui/main.zig   libvaxis debugger
```

Design in one paragraph: all state lives in plain copyable structs and all
logic in free functions taking `*Cpu` plus a bus, so snapshots are memcpy.
Where Musashi generates its dispatch with an external code generator, z68k
builds a 65,536-entry decode table at comptime from an ordinary pure function,
and dispatches with a `switch` that inlines across the bus type. The full
design document, including the test plan and milestone gates, is
[DESIGN.md](DESIGN.md).

## References

- [SingleStepTests/m68000](https://github.com/SingleStepTests/m68000) — MAME-generated per-opcode conformance tests
- [Musashi](https://github.com/kstenerud/Musashi) — architecture reference
- [rocket68](https://github.com/habedi/rocket68) — API-shape reference
- [libvaxis](https://github.com/rockorager/libvaxis) — TUI library
- M68000UM — Motorola 68000 User's Manual (instruction set, cycle tables, exception frames)

## License

[MIT](LICENSE)
