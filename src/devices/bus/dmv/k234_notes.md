# K234 (68008) module — emulation notes and change derivation

This note records *why* the K234 emulation was changed, and which primary
sources each change is derived from, so the changes can be reviewed as
evidence-based rather than guesswork. It covers `k230.cpp` / `k230.h`
(the K234 device) and the two K234-relevant lines in `ncr/dmv.cpp`.

All register, port and address references below are taken from NCR's own
documentation and from the original NCR software, listed under
"Primary sources".

## The K234 hardware

The K234 is the customer-installable 68008 processor module for the NCR
Decision Mate V. It plugs into slot J7/J7A and shares the DM V's base
64 KB of RAM with the mainboard Z80; the two processors never run the bus
at the same time. (K234 "System Information" sheet, parts list, and
pin-assignment table; schematic K234 017-0033573 Rev. A.)

Relevant facts, all from the K234 "System Information" sheet:
- 68008, 8-bit data bus, 8 MHz clock (`XTAL 16 MHz / 2`).
- "In the DM V environment as much [as] 512 Kbyte linear address space."
- "Memory mapped I/O."
- D1 = 68008, D2 = 74LS245 octal bus transceiver, D4/D7 = programmed PALs,
  D9/D10 = 74LS74 dual D-FF, D8 = 74LS38 quad NAND, D5 = 74LS321 oscillator.

## SNR — the processor hand-off doorbell

### What SNR is

The two processors hand the shared bus back and forth through a single
"SNR" doorbell, decoded on the K234 at the `$d8-$df` window. On the
68008 side this is memory-mapped (`$fffd8-$fffdf`); on the Z80 side it is
I/O port `$df`. A write (or read) to that window on either processor
toggles the doorbell: it must *start* the other processor and *stop* the
one that rang it.

Source: the SNR decode sits in the K234 PALs (D7 on the schematic drives
the processor/bus-control group: `HOLDA16`, `THOLD`, `INTAK`, `RESETOUT`,
etc.); the `$d8-$df` placement matches both the original MAME handler
install and the addresses used by the NCR software below.

### Direction evidence, from the NCR formatter source

The decisive evidence for the *direction* of the hand-off is NCR's own
formatter, which drives both sides of SNR explicitly:

1. **68008 -> Z80 (hand the bus to the Z80).** `FORMAT68.S`, at label
   `cont`, immediately after copying the Z80 overlay (`FORMAT.DAT`) down
   to `$9147`:

   ```
   cont:   move.b  (a1)+,(a0)+
           dbra    d0,cont
           move.b  #$7f,$fffdf        * must end before 911a
   ```

   NCR's own comment `* must end before 911a` is the key: after the 68008
   writes SNR it must stop *before* executing the next instruction. The
   68008 is parked and the Z80 takes over.

2. **Z80 -> 68008 (hand the bus back / start the 68008).** `FORM68.Z80`,
   at label `goto68k`:

   ```
   goto68k:
           ...
           ld      hl,4051h
           ld      (6),hl             ; set 68008 reset PC vector low word
           out     0dfh,a             ; ring SNR -> start the 68008
           jp      alles
   ```

   The Z80 pre-loads the 68008 reset vector in shared RAM (`($0004)`
   high word, `($0006)` low word => PC `$00005140`) and then rings SNR
   from port `$df`. The 68008 must come out of reset and begin executing
   at its reset vector.

So SNR is genuinely *bidirectional through one address*, and the correct
behaviour depends on which processor is currently running. That is the
behaviour the code now models with `m_await_back`.

### The bug that was fixed (exception $03 at $911A)

The previous handler did, for both read and write:

```
m_snr = ASSERT_LINE;
m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
m_maincpu->reset();          // synchronous, inside the access handler
out_thold(ASSERT_LINE);
```

Two problems, both confirmed against the sources and against MAME runs:

- **Wrong direction.** It always *started* the 68008, even when the
  68008 itself rang SNR to hand *back* to the Z80 (case 1 above). The
  68008 was never parked.

- **Synchronous reset mid-instruction.** Calling `m_maincpu->reset()`
  from inside the memory access that is still being executed re-entered
  the 68008 core and left it in an inconsistent state. The observed
  result was `Exception $03 at user address $0000911A` — an address
  error reported at the instruction *right after* the SNR store at
  `$9112`, exactly the instruction NCR's `* must end before 911a`
  comment says must never run. (Reproduced in MAME `dmv`; the SNR store
  is visible at `$9112: move.b #$7f,$fffdf` in the 68008 disassembly,
  and the abort banner names `$911A`.)

### The fix

`snr_toggle()` now tracks hand-off direction in `m_await_back`:

- Z80 -> 68008: schedule the 68008 release on a zero-delay timer
  (`snr_reset`) so the current instruction retires before `reset()` runs.
  This removes the mid-instruction re-entrancy that caused exception $03.
- 68008 -> Z80: park the 68008 (`HALT` asserted, `m_snr` cleared) and
  release the Z80 via `THOLD` immediately, so the 68008 does not execute
  past the SNR store ("must end before 911a").

`snr_r()` additionally guards against MAME debugger / side-effect-free
reads so that merely inspecting the `$d8-$df` window in the debugger does
not trigger a hand-off.

`switch16_w()` also clears `m_await_back`, so a `SWITCH16` transition
re-arms the next SNR as a fresh Z80 -> 68008 start.

The deferred-timer idiom matches other DMV bus devices in this directory
(e.g. `k210.cpp` uses `TIMER_CALLBACK_MEMBER` the same way).

## ncr/dmv.cpp — port $14 read, and floppy sound

### Port $14 must be readable

`FORMAT68.S` turns the drive motor on and polls status through the FDC
latch at `$fff00+$13` / `$fff00+$14` (the 68008 view of mainboard ports
`$13`/`$14`):

```
moton:  move.b  $fff00+$13,d0        * get byte from port 13
        move.b  d0,$fff00+$14        * switch disk motor on
        btst    #0,d0
        beq     return               * return if motor on
```

and the Z80 overlay waits on the same latch:

```
warten: in      a,14h
        in      a,50h
        cp      010h
        jr      z,warten
```

The mainboard decodes port `$14` for both read and write (it mirrors the
system-status byte that is also at `$13`; `sys_status_r` already provides
that byte). Previously `dmv.cpp` mapped `$14` as **write-only**
(`fdd_motor_w`), so the Z80 `IN A,($14)` in the format path read an
unmapped port and the poll loop never terminated. Mapping `$14` as
`rw(sys_status_r, fdd_motor_w)` lets the read return the real status
byte, matching the hardware the NCR code expects.

### Floppy sound

`enable_sound(true)` was added to both `i8272:0` / `i8272:1`
`FLOPPY_CONNECTOR`s — a cosmetic parity fix with other MAME floppy
systems; it has no bearing on the format logic.

## Not committed: the DMA diagnostic

During investigation a `logerror` "DMA spy" was temporarily added to the
mainboard `program_w` path to trace am9517 DMA writes into the
`$4000-$6FFF` OS region. That was a debugging probe, not a correctness
change, and is intentionally **not** part of this commit. The separate
question of whether the FDC->DMA transfer terminates correctly (terminal
count / EOP handling in the format path) is still under investigation and
is tracked outside this change.

## Primary sources

- NCR "DECISION MATE V — SYSTEM INFORMATION", K234 MC 68008 Processor
  Board: description, parts list, 68008 pin-assignment table.
- K234 schematic, sheet "68008 Processor Board (K234) 017-0033573 Rev. A"
  (PAL D4/D7 output groups, SNR / processor-control signals).
- NCR system block diagram (Z-80A + DMA + shared RAM/ROM, Figure 2.1).
- DMV expansion-slot pinout (SNR / THOLD / HOLDA16 / SWITCH16 signals),
  as documented in `dmvbus.cpp`.
- NCR formatter sources:
  - `FORMAT68.S` (68008 side: self-copy of the Z80 overlay to `$9147`,
    `move.b #$7f,$fffdf  * must end before 911a`, motor/status via
    `$fff00+$13/$14`, FDC via `$fff00+$50/$51`).
  - `FORM68.Z80` (Z80 overlay: `goto68k` reset-vector set-up and
    `out 0dfh` hand-back).
- MAME `dmv` runs reproducing `Exception $03 at $0000911A`, with the SNR
  store visible at `$9112` in the 68008 disassembly.
