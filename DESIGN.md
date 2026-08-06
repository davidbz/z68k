# z68k — Motorola 68000 Emulator in Zig

Design document. Target: a standalone, reusable MC68000 core, later consumed by a
Sega Genesis emulator (separate repo). Toolchain: Zig 0.16.0.

## 1. Goals & Principles

- **Data-oriented design.** All state lives in plain, copyable data structs. All
  logic lives in free functions taking `*Cpu` plus a bus. No methods hiding
  state, no globals, no allocation inside the core. A `Cpu` can be memcpy'd for
  snapshot/rewind.
- **Library first** (lesson from rocket68): the core is a Zig module with zero
  dependencies. The test harness is a separate executable that depends on it.
- **Accuracy target: architectural state + total cycle counts.** Registers,
  memory, SR, PC, and per-instruction cycle totals are validated against
  [SingleStepTests/m68000](https://github.com/SingleStepTests/m68000). The
  prefetch queue and per-cycle bus *timing* are not modeled — the Genesis
  doesn't need them, and they roughly double the core's complexity. Which
  data-space bus cycles ran, in what order, is checked separately (§5.6).
- **Comptime replaces codegen** (lesson from Musashi): Musashi generates its 64K
  opcode handlers with a build-time C program (`m68kmake`). Zig's `comptime`
  builds the same 65536-entry dispatch table from declarative pattern data at
  compile time, in-language.

## 2. Architecture Overview

```
                 ┌─────────────────────────────┐
                 │  m68k module (no deps)      │
                 │                             │
                 │  cpu.zig     data objects   │   ┌─────────────┐
                 │  decode.zig  opcode → instr │◀──│ SST harness │
                 │  flags.zig   pure CCR logic │   │ harness.zig │
                 │  core.zig    step/run/traps │   └─────────────┘
                 └─────────────────────────────┘
                            ▲
                       bus: anytype
                 (flat RAM in tests, Genesis
                  mapper later — comptime, no vtable)
```

File layout:

```
build.zig, build.zig.zon   module `m68k`; exe `z68k-sst`; steps `test`, `sst`
src/root.zig               public surface
src/cpu.zig                Cpu, StatusRegister, Size, Exception   (data)
src/decode.zig             pattern tables, EaMode, comptime table (data + pure fns)
src/flags.zig              pure flag computation                  (pure fns)
src/core.zig               Core(BusT): step/run/reset/EA/traps    (logic)
src/harness.zig            SingleStepTests runner
tools/fetch_tests.sh       clone + decode test data → testdata/ (gitignored)
```

## 3. Data Objects

### 3.1 `Cpu` — the entire CPU state

```zig
pub const Cpu = struct {
    d: [8]u32,              // data registers D0–D7
    a: [8]u32,              // address registers; a[7] is the ACTIVE stack pointer
    usp: u32,               // shadow of the INACTIVE stack pointer
    pc: u32,
    sr: StatusRegister,
    stopped: bool = false,  // STOP instruction: waiting for interrupt
    trace_pending: bool = false, // T was set at instruction start; trace due next step
    halted: bool = false,   // double fault: dead until reset
    pending_ipl: u3 = 0,    // asserted interrupt priority level, 0 = none
    cycles: u64 = 0,        // total cycles executed
};
```

Stack-pointer convention (matches the SingleStepTests state images): `a[7]`
always holds the stack pointer for the *current* privilege mode; `usp` holds
the other one. The swap happens in exactly one function, `setSupervisor`.

### 3.2 `StatusRegister`

```zig
pub const StatusRegister = packed struct(u16) {
    c: bool = false,        // bit 0  carry
    v: bool = false,        // bit 1  overflow
    z: bool = false,        // bit 2  zero
    n: bool = false,        // bit 3  negative
    x: bool = false,        // bit 4  extend
    _pad0: u3 = 0,
    ipl: u3 = 7,            // bits 8–10  interrupt priority mask
    _pad1: u2 = 0,
    s: bool = true,         // bit 13  supervisor
    _pad2: u1 = 0,
    t: bool = false,        // bit 15  trace
};
```

`packed struct(u16)` gives free `@bitCast` conversion for `MOVE to/from SR`,
exception frames, and test comparison — no manual bit packing anywhere.
Writes to SR mask out the pad bits (real 68000 reads them back as 0).

### 3.3 `Size`

```zig
pub const Size = enum(u2) {
    byte, word, long,

    pub fn bytes(s: Size) u3;        // 1, 2, 4
    pub fn mask(s: Size) u32;        // 0xFF, 0xFFFF, 0xFFFFFFFF
    pub fn msb(s: Size) u32;         // 0x80, 0x8000, 0x80000000
};
```

### 3.4 `EaMode` — addressing modes

```zig
pub const EaMode = enum {
    data_reg,        // Dn
    addr_reg,        // An
    addr_ind,        // (An)
    addr_postinc,    // (An)+
    addr_predec,     // -(An)
    addr_disp,       // d16(An)
    addr_index,      // d8(An,Xn.size)
    abs_word,        // (xxx).W
    abs_long,        // (xxx).L
    pc_disp,         // d16(PC)
    pc_index,        // d8(PC,Xn.size)
    immediate,       // #imm

    pub fn decode(mode: u3, reg: u3) ?EaMode;   // null = illegal encoding
};

/// Addressing-mode classes from the 68000 PRM, used by the decoder to
/// reject illegal operand combinations at table-build time. Only the classes
/// with a consumer exist; ea_data/ea_memory and their alterable intersections
/// arrive with the M2 families that restrict operands to them.
pub const EaClass = std.EnumSet(EaMode);
pub const ea_control: EaClass;     // memory minus (An)+/-(An)/#imm
pub const ea_alterable: EaClass;   // all except PC-relative and #imm
```

### 3.5 Decode table

The decoder output is **data**: a 65536-entry table mapping every possible
opcode word to an instruction identity.

```zig
pub const Mnemonic = enum { move, movea, moveq, /* … ~90 entries … */, illegal, line_a, line_f };

pub const Instr = struct {
    mnemonic: Mnemonic,
    size: Size = .word,     // meaningless for unsized instructions
    base_cycles: u8 = 0,    // base cost; EA cost added at runtime
};

pub fn decodeOne(op: u16) Instr;                // pure, no state
pub const table: [65536]Instr = comptime …;     // map decodeOne over every word
```

`decodeOne` is an ordinary pure function that switches on the opcode's top
nibble and narrows from there, rejecting illegal operand combinations against
the `EaClass` sets of §3.4. `table` is just `decodeOne` mapped over all 65536
words at comptime (it needs a raised `@setEvalBranchQuota`). This replaces
Musashi's entire `m68kmake` generator, and because the decoder is a plain
function it is directly unit-testable without the table.

Dispatch is a `switch` on `instr.mnemonic` inside `Core(BusT)` (§4.1) rather
than a function-pointer table. A pointer table would have to be rebuilt per bus
type and defeats inlining across the call; the `switch` lets the compiler see
the whole instruction body, and it keeps the decode table pure data that the
disassembler can share.

**Timing lives in the table wherever it is a pure function of the opcode.**
`LEA`, for instance, computes an address without fetching an operand, so it
carries its own per-mode cost in `base_cycles` instead of using the shared EA
timings. Instructions whose only job is to raise an exception carry zero: the
vector's entry cost already covers the opcode fetch.

### 3.6 `Exception`

```zig
pub const Exception = enum(u8) {
    reset = 0,
    bus_error = 2,
    address_error = 3,
    illegal_instruction = 4,
    zero_divide = 5,
    chk = 6,
    trapv = 7,
    privilege_violation = 8,
    trace = 9,
    line_a = 10,
    line_f = 11,
    uninitialized_interrupt = 15,
    spurious_interrupt = 24,
    autovector_1 = 25, // … through autovector_7 = 31
    trap_0 = 32,       // … through trap_15 = 47
    _,

    pub fn vectorAddr(e: Exception) u32;   // @intFromEnum(e) * 4
};

/// Extra data for the group-0 (address error) 7-word stack frame.
pub const Group0Info = struct {
    access_addr: u32,
    ir: u16,            // opcode being executed
    read: bool,
    program: bool,      // program vs data space
};
```

### 3.7 Bus interface (comptime contract, not a type)

Any type with these four functions works as a bus:

```zig
read8(self: *B, addr: u24) u8
read16(self: *B, addr: u24) u16      // addr guaranteed even by the core
write8(self: *B, addr: u24, v: u8)
write16(self: *B, addr: u24, v: u16) // addr guaranteed even by the core
```

- 32-bit accesses are composed from two 16-bit accesses **in the core**
  (matches the real 16-bit bus; gets address-wrap behavior right once).
- Alignment is the core's job: a word/long access to an odd address raises an
  address error *before* the bus sees it. The bus never observes odd word
  addresses.
- Address space is 24 bits (A0–A23), enforced by the `u24` parameter type.

Test bus: a flat `[]u8` of 16 MiB. Genesis bus later: ROM/RAM/VDP/IO mapper —
same contract, zero changes to the core.

### 3.8 Harness data (`harness.zig`)

Mirrors the layout of a state block in the binary container (§5.2). Field order
is the file's register order, because the reader walks it positionally:

```zig
pub const State = struct {
    d: [8]u32,
    a: [7]u32,              // a7 is not stored; usp and ssp are, separately
    usp: u32, ssp: u32, sr: u32, pc: u32,
    prefetch: [2]u32,
    ram: []const RamWord,   // { addr: u32, value: u16 } — the bus is 16 bits wide
};

pub const TestCase = struct {
    name: []const u8,
    initial: State,
    final: State,
    length: u32,            // total cycles, from the transaction block header
    accesses: []const Access, // the data-space bus cycles, in order (§5.6)
};
```

Both `State`s point into buffers owned by the decoder and are overwritten by the
next test — nothing is allocated per case.

## 4. Business Logic Flows

All core logic lives in a generic namespace:

```zig
pub fn Core(comptime BusT: type) type {
    return struct {
        pub fn reset(cpu: *Cpu, bus: *BusT) void;
        pub fn step(cpu: *Cpu, bus: *BusT) void;
        pub fn run(cpu: *Cpu, bus: *BusT, budget: u64) u64;
        // … internal: EA, operand access, exception entry, handlers
    };
}
```

### 4.1 `step` — one instruction

```
step(cpu, bus):
  1. if halted → consume 4 cycles, return
  2. if trace_pending → clear it, enterException(.trace), return
  3. if pending_ipl == 7 (NMI) or pending_ipl > sr.ipl → takeInterrupt
  4. if stopped → consume 4 cycles, return          (only interrupts leave STOP)
  5. tracing := sr.t                                (sampled BEFORE execution)
  6. opcode := fetch16(cpu, bus)                    (odd PC → address error)
  7. switch on table[opcode].mnemonic               (EA + read + operate +
                                                     writeback + flags + cycles)
  8. on Fault → enterException with the right vector
  9. trace_pending := tracing
```

**Trace is deferred to the next `step`, not taken at the end of this one.** The
traced instruction completes and the step returns; the exception is taken at the
following instruction boundary, ahead of any pending interrupt. That is where
the hardware — and MAME, which generates the conformance data — puts the
boundary, and getting it wrong fails every single trace-enabled test.

Faults propagate as a Zig error union rather than by calling `enterException`
mid-handler:

```zig
pub const Fault = error{ AddressError, IllegalInstruction, LineA, LineF };
// M2 adds Privilege, ZeroDivide and Trap with the instructions that raise them.
```

`step` catches it and enters the matching vector. The payload a group 0 frame
needs (faulting address, R/W, program vs data space) rides in `Ctx`, a transient
per-instruction struct holding `*Cpu`, `*BusT`, the current opcode, and the
fault info. `Ctx` is deliberately **not** part of `Cpu`: `Cpu` stays a clean
snapshot of architectural state.

Illegal-instruction-class faults (illegal, line A, line F — later privilege)
stack the address of the *offending* instruction, so the catch rewinds `pc` to
where the instruction started before entering the exception.

### 4.2 `run` — timeslice (Musashi model, what the Genesis will call)

```zig
pub fn run(cpu: *Cpu, bus: *BusT, budget: u64) u64 {
    const start = cpu.cycles;
    while (cpu.cycles - start < budget) step(cpu, bus);
    return cpu.cycles - start;      // may overshoot; caller carries the debt
}
```

### 4.3 `reset`

Like real hardware (and rocket68): `sr = .{ .s = true, .ipl = 7 }`,
`stopped/halted = false`, `a[7] ← read32(0)`, `pc ← read32(4)`, +40 cycles.

### 4.4 Effective-address flow

```zig
fn calcEa (ctx: *Ctx, mode: EaMode, reg: u3, size: Size, write: bool) Fault!u32;
fn readEa (ctx: *Ctx, mode: EaMode, reg: u3, size: Size) Fault!u32;
fn writeEa(ctx: *Ctx, mode: EaMode, reg: u3, size: Size, v: u32) Fault!void;
```

- Pre-decrement/post-increment side effects happen inside `calcEa`; **byte
  operations on A7 adjust by 2** (stack stays word-aligned).
- Read-modify-write instructions call `calcEa` once and reuse the address —
  post-inc/pre-dec must not fire twice.
- The brief extension word (index modes) is parsed in one helper:
  `d8 + (Xn.W sign-extended | Xn.L)`.
- PC-relative modes use the address of the extension word as the base (this is
  where the real chip's prefetch becomes architecturally visible; no queue
  needed to get it right). They are also **program-space** accesses, which the
  function-code bits of a group 0 frame record.
- `calcEa`'s `write` flag exists for the misaligned-fault cases, where the
  conformance data pins down when the register writeback happens: a word
  post-increment **read** updates the register before the bus cycle, but a
  **write** — and a **long read**, whose first word access faults — leaves it
  unchanged. A long predecrement **write** goes low word first, so its fault
  address is `addr+2` and the register is not written back; reads go high
  word first and fault with the register already decremented.
- A faulting long operand only completed its first word access, so it pays
  the word EA cost, not the long one (predecrement destinations excepted:
  their cost discount already covers the first access).
- **Each operand's cycle cost is charged before its own bus cycle**, so an
  instruction that faults partway through still pays for the work it did. A
  destination `-(An)` skips the two extra cycles a `-(An)` *source* spends
  computing its address, which is why `EaMode` has both `cycles` and
  `destCycles`.
- EA cycle costs are a data table added to `Instr.base_cycles`:

| mode        | b/w | l  | | mode      | b/w | l  |
|-------------|-----|----|-|-----------|-----|----|
| Dn, An      | 0   | 0  | | (xxx).W   | 8   | 12 |
| (An)        | 4   | 8  | | (xxx).L   | 12  | 16 |
| (An)+       | 4   | 8  | | d16(PC)   | 8   | 12 |
| -(An)       | 6   | 10 | | d8(PC,Xn) | 10  | 14 |
| d16(An)     | 8   | 12 | | #imm      | 4   | 8  |
| d8(An,Xn)   | 10  | 14 | |           |     |    |

### 4.5 Flag logic (`flags.zig` — pure, the most test-critical code)

No `Cpu` access; values in, `Ccr` out. Fully unit-tested in isolation. Only
what has a caller exists:

```zig
pub fn logic(value: u32, size: Size) Ccr;   // N/Z, clears V/C — MOVE, AND, TST, …
pub fn testCondition(sr: StatusRegister, cond: u4) bool;  // the 16 Bcc/Scc/DBcc codes
```

M2 adds `add`/`sub`/`cmp`/`addx`/`subx` (C,V from sign analysis; the X variants
only ever *clear* Z, for multi-precision chains); M3 adds the shift/rotate
flags, whose C/X/V rules differ per variant and at count 0.

### 4.6 Exception entry

```zig
fn enterException(cpu: *Cpu, bus: *BusT, e: Exception, g0: ?Group0Info) void;
```

1. `old_sr := cpu.sr`; `setSupervisor(cpu, true)`; `sr.t = false`
2. Group 1/2: push PC (long), push `old_sr` (word).
   Group 0 (bus/address error): additionally push IR, access address (long),
   and the special status word — the 7-word frame. Only the low five bits of
   that word (R/W, I/N, the three function-code bits) are defined; the upper
   eleven hold whatever the chip last had in its instruction register, which
   MAME reproduces and the conformance tests check. See §5.4.
3. `pc ← read32(vectorAddr(e))`
4. `cycles += cost[e]` (34 for group 1/2, 50 for address error, 44 + 4·n for
   interrupts, exact values from the M68000UM table).
5. Address error *while entering* an address-error handler → `halted = true`.

### 4.7 Interrupts

```zig
pub fn setIpl(cpu: *Cpu, level: u3) void;   // host-facing; just sets pending_ipl
```

`step` takes the interrupt when `pending_ipl > sr.ipl` or `pending_ipl == 7`
(level 7 is non-maskable, edge-triggered on the transition — the harness for
this is a unit test, SST doesn't cover it). Acknowledge is always
**autovector** (`Exception.autovector_N`); the Genesis has no vectored
peripherals. `sr.ipl ← level` on entry; `stopped` cleared.

### 4.8 Instruction families

Each family is one handler function; the decode table routes all its encodings
there. Cycle bases come from Musashi's tables / the M68000UM.

| Family | Opcode pattern (bits 15–12) | Notes |
|---|---|---|
| ORI/ANDI/SUBI/ADDI/EORI/CMPI, `#imm,SR/CCR` forms | `0000` | privilege check on SR forms |
| BTST/BCHG/BCLR/BSET (static & dynamic) | `0000` | bit index mod 32 (Dn) / mod 8 (mem) |
| MOVEP | `0000` | word/long, alternating bytes |
| MOVE.B/W/L, MOVEA | `0001/0011/0010` | the big one: full src EA × dst EA matrix |
| CLR/NEG/NEGX/NOT/TST/NBCD/TAS | `0100` | TAS sets flags then writes bit 7 |
| MOVE to/from SR/CCR, MOVE USP | `0100` | from-SR unprivileged on 68000 |
| EXT/SWAP/PEA/LEA/LINK/UNLK | `0100` | |
| MOVEM | `0100` | reg-mask order reverses for -(An) |
| TRAP/TRAPV/CHK/STOP/RESET/RTE/RTS/RTR/NOP/ILLEGAL | `0100` | privilege checks |
| JSR/JMP | `0100` | control EAs only |
| ADDQ/SUBQ/Scc/DBcc | `0101` | ADDQ/SUBQ to An: no flags, always long |
| Bcc/BSR/BRA | `0110` | 8-bit disp; 0x00 → 16-bit form |
| MOVEQ | `0111` | sign-extended 8-bit → long |
| OR/DIVU/DIVS/SBCD | `1000` | division: cycle-accurate bit-serial simulation, not a table (`divuCycles`/`divsCycles`) |
| SUB/SUBA/SUBX | `1001` | |
| CMP/CMPA/CMPM/EOR | `1011` | |
| AND/MULU/MULS/ABCD/EXG | `1100` | MULU cost = 38 + 2·popcount(src) |
| ADD/ADDA/ADDX | `1101` | |
| ASx/LSx/ROx/ROXx (reg & mem forms) | `1110` | reg form: 6/8 + 2·count |
| line-A / line-F | `1010/1111` | → vectors 10/11 |

## 5. Test Plan

### 5.1 Unit tests (inline `test` blocks, `zig build test`)

- **flags.zig**: condition-code truth table, N/Z/V-clear behaviour of `logic`.
  Each M2/M3 flag function lands with its own edge cases — `0x7FFF + 1`
  overflow, borrow chains, ADDX never-set-Z, every shift variant at counts 0,
  1, size, >size, 63.
- **decode.zig**: every one of the 65536 opcodes maps to a handler or
  `.illegal` (table totality); spot-checks of known encodings
  (`0x4E75` = RTS, `0x303C` = `move.w #imm,d0`, …); EA class matrices reject
  e.g. `addq #1,d8(pc)`.
- **cpu.zig**: SR pack/unpack round-trip, pad-bit masking, `setSupervisor`
  swap symmetry.
- **core.zig**: reset flow, interrupt masking matrix (ipl × pending level),
  STOP wake-up, byte-on-A7 ±2 quirk.

### 5.2 Conformance: SingleStepTests/m68000 (`zig build sst`)

Data: `tools/fetch_tests.sh` clones the repo into `testdata/` (gitignored).

`v1/*.json.bin` is **not** compressed JSON — it is a simple tagged binary
container (length-prefixed blocks behind magic numbers, little-endian). The
harness decodes it directly. That drops the Python dependency of upstream's
`decode.py`, the multi-gigabyte intermediate JSON, and the JSON parse; one test
is decoded at a time into reusable buffers, so the suite runs in fixed memory.

Per test case:
1. Build a flat 16 MiB test bus; apply `initial.ram` pairs and registers.
   `a[7]`/`usp` chosen from `ssp`/`usp` by `initial.sr` S-bit.
2. **PC prefetch correction**: the recorded PC is MAME's next-prefetch address
   — execution start + 4. Set `cpu.pc = initial.pc − 4`; compare against
   `final.pc − 4`.
3. `step()` once.
4. Compare: d0–d7, a0–a6, both stack pointers, SR, PC, all `final.ram` pairs.
   Nothing is masked or excused, including every word of an exception frame.
5. Cycle check: `cpu.cycles == length` — reported as a **separate tier** so a
   regression says which kind it is.
6. Bus check: the data-space cycles the core drove against the ones MAME
   recorded — a third tier (§5.6).

All 127 files run; nothing is excluded or skipped. `TAS` and `TRAPV` were
excluded for a while on upstream's word (its README calls TAS's read-modify-write
timing and TRAPV's trigger condition faulty). Measured against this core both
pass 2500/2500 on both tiers: the divergences were ours — `opTas` never charged
its EA cost, and `opTrapv` charged a 4-cycle base on top of the trap's own 34.
A filter argument (`zig build sst -- MOVE`) narrows the run by filename
substring, nothing else.

Reporting is three numbers per file plus the first failure of each kind, with
expected-vs-actual detail. The build gates on all three being complete: every
case must match state, cycles *and* data-space bus cycles, so any regression
turns `zig build sst` red.

### 5.3 CI

- `zig build test` and the full SST suite on every pull request
  (`.github/workflows/ci.yml`). The ~2 GB fetch is done per run; the Zig build
  cache is keyed on the `build.zig.zon` hash.

### 5.4 Group 0 fault frames

The 7-word address-error frame is the one place where "architectural state"
leaks microcode timing, and it is fully modelled here — no field is excused.

**PC (+10)** comes from the prefetch mark (`Ctx.prefetch`): handlers that can
fault past the point where a real 68000 has already fetched the next opcode
call `markPrefetch`/`markReadFault`, and the fault path uses the marked PC
instead of the natural one. Each addressing mode's/instruction's mark point
was derived empirically from conformance data, per instruction rather than as
one shared rule — the same mode can need a different mark depending on who
uses it (JSR's `(xxx).w` EA marks the post-EA PC, JMP's identical EA marks
the pre-EA PC, since one pushes a return address into the fault-safe path and
the other doesn't).

**IR (+6), and with it the SSW's upper 11 bits of IR residue**, is the opcode
still executing for every family and every mode but one: a **word-sized MOVE
to `-(An)`** is slow enough that the next opcode has already been latched by
fault time, so the frame shows the word at the (unmodified) PC instead. That
is the only value `Ctx.prefetch.ir` ever overrides, and it costs 4 cycles.
Long-sized predecrement is not slow enough in the same way, nor is any other
destination mode. An earlier attempt to apply "the word at the marked PC"
*unconditionally* is what made this look unmodellable: it dropped MOVE.w from
2380/2500 to 1013/2500, because most faulting cases do keep the original
opcode.

**CCR, live and stacked at +8**, freezes wherever MOVE's microcode got to
before the destination write faulted, which the data pins down per operand
pair (`opMove`). Byte and word moves have always finished the flag update by
then. A long move has too, *unless* the destination needed address work the
source didn't pay for:

| long MOVE, source | destination | what the frame shows |
|---|---|---|
| `Dn`/`An`/`#imm` | `(An)`, `(An)+` | CCR untouched |
| `Dn`/`An`/`#imm` | `d(An)`, `d(An,Xn)` | N and Z from the full long; V/C not yet cleared |
| memory | `(An)`, `(An)+`, `(xxx).l` | N and Z as if the value were a *word* — the low word is the last thing the source read pushed through the ALU |

Everything else lands the full N/Z/V/C. A long immediate behaves like a
register source: it arrives as two prefetch words and never crosses the data
bus into the ALU.

### 5.5 Milestone gates

| Milestone | Scope | Gate |
|---|---|---|
| M1 | decode table, MOVE/MOVEA/MOVEQ, harness runs | MOVE* files pass state tier |
| M2 | ALU families + flags | arithmetic/logic files pass state tier |
| M3 | shifts, bits, BCD, EXG, MUL/DIV | their files pass state tier |
| M4 | prefetch queue; remaining exceptions | **all** files pass state tier |
| M5 | cycle tables tightened | all files pass cycle tier |

**Current status: all milestones done.** All 127 upstream files, 317500 cases,
pass both tiers exactly — no exclusions, no skips, no masked fields (§5.2).

The rest of this section records the rules that were derived from conformance
data rather than from the manual, because the code implementing them looks
arbitrary without them.

**Flags.** ABCD's decimal carry threshold is `res > 0x9F`, not the `> 0x99`
textbook DAA threshold. SBCD's C/X output is a strictly more inclusive
condition than the one gating its value's high-nibble correction: it also
fires when the raw operands are exactly equal but a low-nibble borrow still
occurs. DIVU/DIVS set N unconditionally on quotient overflow, regardless of
the (discarded) quotient's actual sign. See `flags.zig`'s `bcdAdd`/`bcdSub`
and `core.zig`'s `opDivu`/`opDivs`. MOVE's fault-time CCR staging is in §5.4.

**Bus order.** ADDX/SUBX/CMPM's `-(An)`/`(An)+` forms follow a per-operand
"commit only on your own success" rule: a misaligned access on one operand can
leave *that* operand's address register uncommitted while the other,
already-succeeded operand's update stands — unlike the single-operand model's
"always commit, fault after." Long-sized CMPM's source additionally
partial-commits +2 (not 0, not +4) when its own address is odd. These live as
instruction-specific code in `opAddSubX`/`opCmpm` rather than in `calcEa`,
since they don't hold for the single-operand modes that function serves.

**Prefetch marks.** Several control-flow families need bespoke mark points
rather than a mode-keyed rule: JMP marks the pre-EA PC while JSR (identical
addressing modes) marks the post-EA PC, since JSR's return-address push
changes what the fault-safe path should show; BRA/Bcc-taken mark their own
pre-branch PC ("as if the branch had never been attempted") while BSR keeps
the target, since only BSR also pushes a return address; DBcc's mandatory
16-bit displacement fetch marks the natural post-fetch PC, unlike Bcc's
optional 8-bit form; RTS/RTE/RTR mark their own pre-jump PC rather than the
popped target; MOVEM marks once per instruction, applied to whichever
register in the transfer list actually faults.

**Cycles.** The compositional model (base + EA cost) needed three
instruction-specific corrections to reach the cycle tier:

- **DIVU/DIVS** are a direct port of Jorge Cwik's cycle-accurate simulation of
  the divide microcode — a 15-iteration bit-serial restoring-division loop
  (`divuCycles`/`divsCycles`) — not a closed-form table, since the real cost
  is data-dependent on the dividend/divisor bit pattern, not just magnitude.
- **MOVEfromSR**'s memory form is not a MOVE-shaped write: it reads the
  destination first (like `MOVEtoCCR`), so it uses the full `cycles` table
  plus a flat `-4` fault correction, not `destCycles`' predecrement discount.
- **MOVE**'s destination-fault correction is `-4` at long size, plus a further
  `-4` for `abs_long` at any size once the source read used the bus (anything
  but a free `Dn`/`An`), since its two extension words then overlap that
  access differently. `-(An)` gives nothing back — it pays its full cost even
  when it faults, its first access being the one `destCycles`' discount covers
  — and at word size it *adds* 4 for the next-opcode prefetch that the same
  fault makes visible in IR (§5.4).

### 5.6 Bus tier: which cycles ran, in what order

Matching final state does not mean the right bus cycles happened. A read from
the wrong address that happens to hold the right value, a write pair emitted
back to front, a word access where the chip drives two bytes — all of them are
invisible once the instruction has retired. The recorded transaction block
already carries what is needed to see them, so the harness decodes it rather
than skipping it.

What is compared, per case: every **data-space** cycle, in order — address
(even; the chip has no A0), direction, the two data strobes, and the driven
half or halves of the data bus. What is not:

- **Program space.** Prefetches, PC-relative operand reads, and the harness's
  own opcode re-read for the fault frame. No prefetch queue is modelled
  (§1), so this core's program cycles legitimately differ. The core tells the
  bus which space each access is in through an optional `setProgram` hook —
  `if (@hasDecl(BusT, "setProgram"))`, so the documented four-function contract
  (§3.7) is unchanged for hosts that don't want it. Exception **vector fetches
  are supervisor data**, not program, which is why `enterException` reads them
  with `program = false`.
- **Per-transaction timing.** Idle cycles and each transaction's duration are
  dropped; only the total feeds the cycle tier.
- **Address-error cycles** (upstream kinds 4 and 5), where AS is never asserted
  and no bus cycle happens.
- **The undriven half of the data bus.** A byte write mirrors the byte onto
  both halves on hardware; only the strobed lane is meaningful.

The tier gates like the other two: 317,500 of 317,500. It started at 227,669
and every gap was a real divergence, not an over-strict comparison. What it
found, and what the fixes settled:

| Class | What the data showed | Where |
|---|---|---|
| **Byte reads through `d(PC)`** | Reported as data space because `read8` had no space parameter. Seven byte families, ~60 cases each. | `read8` |
| **Frame push order** | The microcode does not push a frame in stack order. It writes the low word of each stacked long, then an unrelated word, then the high word: PC low, SR, PC high for the 3-word frame, and the same interleave twice over for the 7-word one. Worth 76,420 cases on its own. | `frame` |
| **Long word order** | Not one rule. A plain `MOVE.l` to memory, a stack push (JSR/BSR/PEA/LINK) and every ordinary long read go **high word first**. Read-modify-write write-back, `MOVE.l`/`MOVEM.l` to `-(An)`, and both `ADDX.l`/`SUBX.l -(An)` operand reads go **low word first**. Derived per family from the recordings — a blanket swap looked plausible and broke the state tier, because read order decides which half commits before an address error. | `write32Low`/`read32Low` and their callers |
| **MOVEM's trailing read** | Loading registers runs one word read past the end of the list and discards it; the flat `base_cycles` already paid for it, so only the bus saw it missing. | `opMovem` |
| **Missing RMW read** | `Scc` with a memory destination wrote without reading first. Same shape as the `MOVEfromSR` note in §5.5, which cycle counts caught instead. | `opScc` |

The word-order split is the one to remember: order is state-visible on reads
(a fault mid-long leaves the already-read half committed) and invisible on
writes (both halves are aligned by then, or the operand read faulted first).

## 6. References

- [SingleStepTests/m68000](https://github.com/SingleStepTests/m68000) — MAME-generated per-opcode JSON conformance tests
- [Musashi](https://github.com/kstenerud/Musashi) — architecture reference: generated dispatch, memory callbacks, timeslices
- [rocket68](https://github.com/habedi/rocket68) — API-shape reference: single-struct state, library-first, reset semantics
- M68000UM — Motorola 68000 User's Manual (instruction set, cycle tables, exception frames)
