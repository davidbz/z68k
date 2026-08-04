# z68k

A Motorola 68000 CPU emulator written in Zig, built as a standalone, reusable
library. Intended to power a Sega Genesis / Mega Drive emulator, but has no
knowledge of any particular machine: the host supplies the bus.

Accuracy is validated against the MAME-generated
[SingleStepTests/m68000](https://github.com/SingleStepTests/m68000) suite, at
the level of architectural state (registers, SR, PC, memory) and total
per-instruction cycle counts. The prefetch queue and per-cycle bus activity are
not modelled; see [DESIGN.md](DESIGN.md) for the rationale and the one known
divergence this causes.

## Status

Early development (milestone M1 of 6): decode table, MOVE/MOVEA/MOVEQ, LEA,
branches, RTS, and the exception machinery (address errors, illegal
instructions, interrupts, trace). Across the 13 applicable conformance files
(32,500 cases), all failures are the single documented prefetch gap;
unexplained failures are 0, and every state-passing case also passes the
cycle tier. Files for not-yet-implemented families are skipped and counted.

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

The suite reports three tiers per file: `state` (full architectural match),
`cycles` (exact cycle count, of the state-matching cases), and `aerr-pc`
(the known prefetch-derived gap, DESIGN.md §5.4). The build fails only on
unexplained failures, so the known gap stays visible without masking
regressions.

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
