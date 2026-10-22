// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    NCR 9010

    8085A-based business system (about 1981), 256 KB RAM behind a page MMU,
    8" flexible disk drives, optional fixed disk, character video 80x24.
    Typical operating system: SB-80 (Lifeboat Associates, CP/M 2.2 compatible).

    Boards (from photos of a machine at Schroeder):
      M210-02  Common Control Module: P8085A-2, 27.648 MHz oscillator,
               2716 EPROMs "1023" (ROM 1, J1) and "1024" (ROM 2, F1),
               empty third socket G1, 93419 64x9 bipolar RAM (MMU map),
               CCM indicator board (8 LEDs "17".."10" = bits 7-0, RUN LED)
      M217-02  memory board, 36 x 64Kx1 DRAM (4 of 8 rows fitted, parity)
      M215-01  discrete DMA board
      M223-06  video board, no CRTC (TTL timing), Intel D2608 "6-1043026" and
               EPROM "9709" (character generator?), MMI 6331 PROM, 18 x 1Kx1 SRAM
      I/O Processor: microcoded, 2 x Am2901, 5 x MMI 6341 PROMs (1117..1120, 1122)
      Flex Disk IF: discrete drive interface, PROMs "502" and "177"

    The two boot EPROMs are stored with inverted data lines.

    Front panel (right of the screen, NCR GmbH Augsburg, class 3274,
    model 1201-4301; the drive cabinet is class 7646, model 0201-2390):
    FLEX DRIVE STATUS with a green READY and a red BUSY lamp per drive 0-3,
    a MONIT key, COMM and POWER lamps, power switch and brightness wheel.
    READY follows a mounted disk, BUSY lights while the flex controller
    works on the drive and for a short head unload delay afterwards.  The
    COMM lamp stays dark (no communication hardware is emulated); RUN on
    the CCM indicator is lit while the machine is on.

    MONIT: the key sets bit 6 of port 05 and pulls TRAP.  The boot ROM has
    no RAM vector for TRAP (SB-80 leaves 0024h alone), so the TRAP
    acknowledge is assumed to switch the boot ROM in, which the emulation
    does.  The ROM handler (0024h -> 0742h) saves the registers to 3FE1h-
    3FEAh; with bit 6 set it acknowledges with OUT 05 and enters the ROM
    monitor ("MONITOR?", commands M R G I E F C T L Z), with bit 7 set it
    shows 18h on the CCM LEDs and stops (power fail?), otherwise it
    restarts.  The monitor keeps its command table and work area at
    3F80h-3FFFh, which is only set up at power on: before an operating
    system is loaded it works; after SB-80 has used that memory it may not.

    Memory map as seen by the 8085 (derived from the boot ROM and SBIOS 1.8):
      0000-0FFF  boot ROM while bit 0 of port 04 is set (writes always go to RAM)
      0000-FFFF  RAM through the MMU (16 x 4 KB pages, 64 physical pages)
      FC00-FCFF  memory mapped I/O, mirrors I/O ports 00-FF
      FD00-FDFF  flexible disk controller (I/O Processor)

    MMU: writing port 04 with bit 2 set arms a map write; the next memory
    write cycle does not reach memory.  Its address supplies the data:
      A15-A12 logical page, A11-A10 map set, A9-A8 = 01 (page register),
      A7-A2 = inverted physical page.  Writes with A9-A8 != 01 are control
      writes of unknown meaning (the ROM writes address 0000 after loading
      the maps).  Only map set 0 is ever used by the ROM and SB-80, so map
      set selection is not emulated.

    I/O ports (boot ROM, SB-80 and the NCR I-8100 Series pocket reference,
    whose port map the 9010 shares):
      00   R  interrupt status (bits 5-3 internal level 4/5/6, bit 6 maskable
              interrupts disabled, bit 7 internal interrupt waiting)
      01  RW  CCM indicator LEDs (POST step / firmware error code); also the
              data port of the byte link to a DMV (data inverted, see below).
              The 8100 has its indicator lights on 06 instead.
      02  RW  byte link status: bits 1-0 = 3 byte received, 2 ready to send
      03  RW  interval timer count (RST 5 at 0028h reads it)
      04   W  01 enable on-board memory (boot ROM), 00 disable, bit 2 arm
              MMU write (8100: "enable memory address translator control")
      05  RW  nonmaskable interrupt (TRAP) status: bit 0 transfer error,
              bit 1 parity error, bit 2 time-out, bit 3 on-board memory
              enabled, bit 6 MONIT key, bit 7 power down (error code 18);
              writing 00 resets it
      12   W  written 00 at reset
      21   R  workstation status / mode switches (bit 7 = 0 ROM test modes)
      23   R  configuration switches SW1 (bits 3-0) and SW2 (bits 7-4)
      27   W  workstation interface reset
      28-2F   keyboard: 28 data, 29 status (10 data, 20 entry not accepted,
              40 break) / clear ENA, 2A/2B enable/disable interrupt, 2C reset
              service request, 2D diagnostic clear, 2E bell, 2F reset
      30-37   parallel printer 1 (NCR 6441): 30 data, 31 status (bits C6 =
              error), 32 enable interrupt / read data turnaround, 33 disable
              interrupt, 37 reset; SBIOS 1.8 polls it, LST: = P1 by default
      41-47   data cartridge, load device "2" (41 command: 00 rewind,
              01 unload, 02 tapemark, 07 write, 08 read, 09 retension,
              0A-3A track, 0E/1E unit; 42 diagnostic status, 43/46 int,
              45 functional status, 47 reset service request; not emulated)
      61-67   NCR 6560 fixed disk controller, load device "1" (not emulated,
              reports "not ready", see below)
      8C-8F   flex disk DMA      \
      94-97   data cartridge DMA  | byte 0 A7-A0, byte 1 A15-A8, byte 2 count
      9C-9F   cassette DMA        | F7-F0, byte 3 C1 C0 A17 A16 F11-F8 with
      A4-A7   fixed disk DMA     /  C1C0 = 10 write memory, 11 read memory.
              The DMA addresses physical memory (18 bits) past the MMU.
      C0-C4   CRT: cursor X, cursor Y, display char (auto increment),
              control character / attribute, display enable
    8100 options at FDxx, not known on the 9010 and not emulated: FDA0-FDA7
    message level adapter M250-05 (synchronous BSC, V.24 modem control;
    probably the 9010 COMM option), FDD0-FDD7 auxiliary printer interface
    (SBIOS printer P2, NCR 4504 visual record printer), FDE0-FDE3 time of
    day clock, FCE0-FCF9 cassette.

    Flex disk controller (FD01 command / sense 1, FD02 sense 2 / int on,
    FD03 int off, FD04 reset service request, FD07 reset):
      the CPU writes four bytes: function, unit (bits 2-1) / side (bit 0) /
      double density (bit 7), cylinder, sector.  Functions: 0 restore,
      1 write format, 2 read format (IDs), 3 write data, 4 read data,
      5 write deleted sector, 6 format defective track, 7 verify, 8 DMA
      byte out, 9 DMA byte in, A byte turnaround, B data turnaround; the
      high nibble selects ASCII/EBCDIC conversion (8, C).  0-4 and 7 are
      emulated.  Data moves by DMA for the full count; the controller
      continues with the next sector and the next cylinder (same head)
      until the count is exhausted.  Sense 1: bit 7 busy, bit 6 service
      request, bit 5 deleted data mark, bits 3-0 result (0 diagnostics
      failed, 1 soft error, 2 ok, 3 invalid instruction, 4 not ready,
      5 media changed since the last command (not generated; the ROM
      restores again), 7 verify error, 8 beyond track 73 sector 26).
      Sense 2: 40 DMA parity, 20 CRC, 10 DMA time-out, 08 data mark not
      found, 04 sector not found, 02 seek error, 01 write fault.  SB-80
      only tests for 2.  The real I/O Processor microcode is not dumped,
      so this is a high-level emulation driving an FD1793.

    Disk format (SB-80): cylinder 0 head 0 FM 26 x 128 ("NCR FMT-" ID
    block), all other tracks MFM 26 x 256.  Logical track 1 = cylinder 72,
    logical track 2 = cylinder 73 (boot sector, BIOS, DOS); the boot ROM
    reads cylinder 72 sectors 1-26 and cylinder 73 sectors 1-6 to 0100h and
    requires a zero 8-bit sum over 0100h-20FFh.

    Fixed disk controller (SBIOS 1.8 / 1.7, boot ROM load device "1"):
      NCR 6560 cartridge disk (8100 series: one fixed and one removable
      4.9 MB disk, 35 ms average access, 312 KB/s per Datapro 1980; these
      figures match a CDC 9427H Hawk, not verified).  FC61 command / sense 1, FC62 sense 2 / int on, FC63 int off,
      FC64 reset service request, FC67 reset.
      The CPU writes four bytes, waiting for bit 7 (busy) to clear between
      them: command (00 restore, 01 seek, 02 write format, 03 read format,
      04 write, 05 read, 06 time disk, 07 seek RT, 08 fill buffer,
      09 dump buffer, 0A data / 0B byte turnaround), unit (software unit
      0-7 = drive 1-4, disk 0/1; SB-80 uses 0 and 1 as C: and D:), block
      number low, block number high (00-3F).  Sense 1: 80 busy, 40 poll,
      10 write protected, 08 not ready, 04 invalid instruction, 02 ok,
      01 soft error; sense 2: 80 alternate track used, 40 DMA parity,
      20 CRC, 10 missing clock, 08 no data sync, 04 sector not found,
      02 drive fault, 01 seek error.  Blocks are 512 bytes, addressed
      linearly (BIOS: track * 24 + sector - 1), so the controller handles
      the drive geometry itself.  Completion sets bit 6 (service request),
      result = status & 1F (02 ok, 08 not ready).  DMA on FCA4-FCA7 with
      the same layout as the flex disk DMA.  SB-80 drives C: and D: are one
      unit each: 24 x 512 bytes per track, 4 KB blocks, 1200 blocks
      (4.8 MB), 512 directory entries, 1 reserved track.  Block 11 holds
      an alternate track table (6 entries, lookup disabled in SBIOS 1.8).
      The boot ROM loads blocks 12-19 to 0100h and 20-27 to 1100h (same
      checksum rule as the flex disk); CONFIG bit 7 selects the unit.
      The controller board is not in the photos.

    DMV link (SB-80 "Fix Disk Version 1.7 WITH INTERRUPT"): no hardware
    interrupts; the BIOS hooks the BDOS entry and the console wait loops
    and polls port 02.  When a byte is pending (status 3) it receives a
    disk request block over port 01 (inverted data), runs it on the local
    fixed disk and transfers 512 byte sectors over the link, making the
    9010 a fixed disk server for a DMV (see SEND9010/EMPF9010 and
    SENDDMV/EMPFDMV, DMV side on ports C0-C3).  The link is reported idle.
    The link software writes and reads port 01 (OUT 1 / IN 1, data
    complemented) with OUT 2 / IN 2 as handshake, so it is a byte wide
    parallel link, presumably on the CCM header J1 that carries the port 01
    latch to the indicator board.  Neither the boot ROM nor SB-80 executes
    SIM, and RIM only appears in the TRAP handler (IE recovery), so the
    8085 SID/SOD pins are unused by the known software.

    TODO:
    - dump character generator, video PROMs, IOP microcode, Flex IF PROMs
    - real video timing, attributes, cursor shape
    - keyboard is HLE (ASCII + 80h-9Fh function codes from the SB-80 key table)
    - DMV byte link (ports 01/02)
    - interrupts (keyboard, disk), serial port, fixed disk,
      load device 2, third ROM socket ("EXTFW" extension firmware at 1000h)

****************************************************************************/

#include "emu.h"

#include "cpu/i8085/i8085.h"
#include "imagedev/floppy.h"
#include "imagedev/printer.h"
#include "machine/keyboard.h"
#include "machine/wd_fdc.h"
#include "sound/beep.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include "formats/imd_dsk.h"

#include "ncr9010.lh"

#define LOG_MMU     (1U << 1)
#define LOG_FLEX    (1U << 2)
#define LOG_IO      (1U << 3)

#define VERBOSE (0)
#include "logmacro.h"

#define LOGMMU(...)  LOGMASKED(LOG_MMU,  __VA_ARGS__)
#define LOGFLEX(...) LOGMASKED(LOG_FLEX, __VA_ARGS__)
#define LOGIO(...)   LOGMASKED(LOG_IO,   __VA_ARGS__)


namespace {

// substitute character set (public domain 8x8 font by Daniel Hepper / Marcel
// Sondaar), used only while the real character generator is not dumped
const u8 subst_chargen[128 * 8] =
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0x18, 0x18, 0x00, 0x18, 0x00,
	0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c, 0x6c, 0xfe, 0x6c, 0xfe, 0x6c, 0x6c, 0x00,
	0x30, 0x7c, 0xc0, 0x78, 0x0c, 0xf8, 0x30, 0x00, 0x00, 0xc6, 0xcc, 0x18, 0x30, 0x66, 0xc6, 0x00,
	0x38, 0x6c, 0x38, 0x76, 0xdc, 0xcc, 0x76, 0x00, 0x60, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x18, 0x30, 0x60, 0x60, 0x60, 0x30, 0x18, 0x00, 0x60, 0x30, 0x18, 0x18, 0x18, 0x30, 0x60, 0x00,
	0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00, 0x00, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80, 0x00,
	0x7c, 0xc6, 0xce, 0xde, 0xf6, 0xe6, 0x7c, 0x00, 0x30, 0x70, 0x30, 0x30, 0x30, 0x30, 0xfc, 0x00,
	0x78, 0xcc, 0x0c, 0x38, 0x60, 0xcc, 0xfc, 0x00, 0x78, 0xcc, 0x0c, 0x38, 0x0c, 0xcc, 0x78, 0x00,
	0x1c, 0x3c, 0x6c, 0xcc, 0xfe, 0x0c, 0x1e, 0x00, 0xfc, 0xc0, 0xf8, 0x0c, 0x0c, 0xcc, 0x78, 0x00,
	0x38, 0x60, 0xc0, 0xf8, 0xcc, 0xcc, 0x78, 0x00, 0xfc, 0xcc, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x00,
	0x78, 0xcc, 0xcc, 0x78, 0xcc, 0xcc, 0x78, 0x00, 0x78, 0xcc, 0xcc, 0x7c, 0x0c, 0x18, 0x70, 0x00,
	0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x60,
	0x18, 0x30, 0x60, 0xc0, 0x60, 0x30, 0x18, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0xfc, 0x00, 0x00,
	0x60, 0x30, 0x18, 0x0c, 0x18, 0x30, 0x60, 0x00, 0x78, 0xcc, 0x0c, 0x18, 0x30, 0x00, 0x30, 0x00,
	0x7c, 0xc6, 0xde, 0xde, 0xde, 0xc0, 0x78, 0x00, 0x30, 0x78, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0x00,
	0xfc, 0x66, 0x66, 0x7c, 0x66, 0x66, 0xfc, 0x00, 0x3c, 0x66, 0xc0, 0xc0, 0xc0, 0x66, 0x3c, 0x00,
	0xf8, 0x6c, 0x66, 0x66, 0x66, 0x6c, 0xf8, 0x00, 0xfe, 0x62, 0x68, 0x78, 0x68, 0x62, 0xfe, 0x00,
	0xfe, 0x62, 0x68, 0x78, 0x68, 0x60, 0xf0, 0x00, 0x3c, 0x66, 0xc0, 0xc0, 0xce, 0x66, 0x3e, 0x00,
	0xcc, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0xcc, 0x00, 0x78, 0x30, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00,
	0x1e, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0x78, 0x00, 0xe6, 0x66, 0x6c, 0x78, 0x6c, 0x66, 0xe6, 0x00,
	0xf0, 0x60, 0x60, 0x60, 0x62, 0x66, 0xfe, 0x00, 0xc6, 0xee, 0xfe, 0xfe, 0xd6, 0xc6, 0xc6, 0x00,
	0xc6, 0xe6, 0xf6, 0xde, 0xce, 0xc6, 0xc6, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0x6c, 0x38, 0x00,
	0xfc, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xf0, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0xdc, 0x78, 0x1c, 0x00,
	0xfc, 0x66, 0x66, 0x7c, 0x6c, 0x66, 0xe6, 0x00, 0x78, 0xcc, 0xe0, 0x70, 0x1c, 0xcc, 0x78, 0x00,
	0xfc, 0xb4, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xfc, 0x00,
	0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x00, 0xc6, 0xc6, 0xc6, 0xd6, 0xfe, 0xee, 0xc6, 0x00,
	0xc6, 0xc6, 0x6c, 0x38, 0x38, 0x6c, 0xc6, 0x00, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x30, 0x78, 0x00,
	0xfe, 0xc6, 0x8c, 0x18, 0x32, 0x66, 0xfe, 0x00, 0x78, 0x60, 0x60, 0x60, 0x60, 0x60, 0x78, 0x00,
	0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x02, 0x00, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x78, 0x00,
	0x10, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
	0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0x76, 0x00,
	0xe0, 0x60, 0x60, 0x7c, 0x66, 0x66, 0xdc, 0x00, 0x00, 0x00, 0x78, 0xcc, 0xc0, 0xcc, 0x78, 0x00,
	0x1c, 0x0c, 0x0c, 0x7c, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x78, 0xcc, 0xfc, 0xc0, 0x78, 0x00,
	0x38, 0x6c, 0x60, 0xf0, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0x7c, 0x0c, 0xf8,
	0xe0, 0x60, 0x6c, 0x76, 0x66, 0x66, 0xe6, 0x00, 0x30, 0x00, 0x70, 0x30, 0x30, 0x30, 0x78, 0x00,
	0x0c, 0x00, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0x78, 0xe0, 0x60, 0x66, 0x6c, 0x78, 0x6c, 0xe6, 0x00,
	0x70, 0x30, 0x30, 0x30, 0x30, 0x30, 0x78, 0x00, 0x00, 0x00, 0xcc, 0xfe, 0xfe, 0xd6, 0xc6, 0x00,
	0x00, 0x00, 0xf8, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0x78, 0x00,
	0x00, 0x00, 0xdc, 0x66, 0x66, 0x7c, 0x60, 0xf0, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0x7c, 0x0c, 0x1e,
	0x00, 0x00, 0xdc, 0x76, 0x66, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x7c, 0xc0, 0x78, 0x0c, 0xf8, 0x00,
	0x10, 0x30, 0x7c, 0x30, 0x30, 0x34, 0x18, 0x00, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00,
	0x00, 0x00, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x00, 0x00, 0x00, 0xc6, 0xd6, 0xfe, 0xfe, 0x6c, 0x00,
	0x00, 0x00, 0xc6, 0x6c, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0xf8,
	0x00, 0x00, 0xfc, 0x98, 0x30, 0x64, 0xfc, 0x00, 0x1c, 0x30, 0x30, 0xe0, 0x30, 0x30, 0x1c, 0x00,
	0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00, 0xe0, 0x30, 0x30, 0x1c, 0x30, 0x30, 0xe0, 0x00,
	0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


class ncr9010_state : public driver_device
{
public:
	ncr9010_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_fdc(*this, "fdc")
		, m_floppy(*this, "fdc:%u", 0U)
		, m_printer(*this, "printer")
		, m_bell(*this, "bell")
		, m_palette(*this, "palette")
		, m_rom(*this, "maincpu")
		, m_chargen(*this, "chargen")
		, m_mode(*this, "MODE")
		, m_config(*this, "CONFIG")
		, m_ccm_leds(*this, "ccm_led%u", 0U)
		, m_ccm_run(*this, "ccm_run")
		, m_ready_leds(*this, "flex_ready%u", 0U)
		, m_busy_leds(*this, "flex_busy%u", 0U)
		, m_comm_led(*this, "comm_led")
		, m_power_led(*this, "power_led")
	{ }

	DECLARE_INPUT_CHANGED_MEMBER(monit_pressed);

	void ncr9010(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void video_start() override ATTR_COLD;

private:
	static constexpr unsigned SECTORS_PER_TRACK = 26;

	enum : u8
	{
		FX_IDLE = 0,
		FX_RESTORE,
		FX_RECAL,
		FX_SEEK,
		FX_XFER,
		FX_READID,
		FX_FORMAT
	};

	// primary status (FD01 bits 3-0) and secondary status (FD02) after the I-8100 pocket reference
	enum : u8
	{
		RES_SOFT     = 0x01,
		RES_OK       = 0x02,
		RES_BADCMD   = 0x03,
		RES_NOTREADY = 0x04,
		RES_CHANGED  = 0x05, // media changed since the last command (not generated)
		RES_VERIFY   = 0x07
	};

	enum : u8
	{
		ST2_WFAULT   = 0x01,
		ST2_SEEK     = 0x02,
		ST2_RNF      = 0x04,
		ST2_NOMARK   = 0x08,
		ST2_DMATIME  = 0x10,
		ST2_CRC      = 0x20,
		ST2_PARITY   = 0x40
	};

	required_device<i8085a_cpu_device> m_maincpu;
	required_device<fd1793_device> m_fdc;
	required_device_array<floppy_connector, 4> m_floppy;
	required_device<printer_image_device> m_printer;
	required_device<beep_device> m_bell;
	required_device<palette_device> m_palette;
	required_region_ptr<u8> m_rom;
	required_region_ptr<u8> m_chargen;
	required_ioport m_mode;
	required_ioport m_config;
	output_finder<8> m_ccm_leds;
	output_finder<> m_ccm_run;
	output_finder<4> m_ready_leds;
	output_finder<4> m_busy_leds;
	output_finder<> m_comm_led;
	output_finder<> m_power_led;

	std::unique_ptr<u8[]> m_ram;
	u8 m_map[4][16];
	u8 m_memctrl;
	bool m_map_arm;
	u8 m_trap_cause;

	// video
	u8 m_vram[2048];
	u8 m_vattr[2048];
	u8 m_cur_x, m_cur_y;
	u8 m_crt_ctrl;
	bool m_subst_font;

	// keyboard
	u8 m_kbd_fifo[16];
	u8 m_kbd_head, m_kbd_count;

	// flex disk
	u8 m_fx_cmdbuf[4];
	u8 m_fx_cmdcnt;
	u8 m_fx_status, m_fx_status2;
	u8 m_fx_state;
	u8 m_fx_cmd, m_fx_unit, m_fx_head, m_fx_cyl, m_fx_sec;
	bool m_fx_dd;
	u8 m_fx_curcyl[4];
	u8 m_fx_dmareg[4];
	u32 m_fx_addr;
	u16 m_fx_count;
	u8 m_fx_inten;
	std::vector<u8> m_fx_trackbuf;
	unsigned m_fx_trackpos, m_fx_ids;
	u8 m_fx_idbuf[6];
	unsigned m_fx_idpos;
	floppy_image_device *m_fx_floppy;
	emu_timer *m_fx_timer;
	emu_timer *m_hld_timer;
	emu_timer *m_panel_timer;
	emu_timer *m_bell_timer;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	static void floppy_formats(format_registration &fr);

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	u32 phys(offs_t offset) const { return (u32(m_map[0][offset >> 12]) << 12) | (offset & 0x0fff); }
	// the DMA board addresses physical memory with 18 bits and bypasses the MMU
	u8 dma_r(u32 addr) { return m_ram[addr & 0x3ffff]; }
	void dma_w(u32 addr, u8 data) { m_ram[addr & 0x3ffff] = data; }

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void kbd_put(u8 data);

	u8 flex_r(offs_t offset);
	void flex_w(offs_t offset, u8 data);
	void flex_command();
	void flex_finish(u8 result, u8 status2 = 0);
	void flex_select();
	void flex_seek();
	void flex_start_transfer();
	void flex_build_track();
	void fdc_intrq_w(int state);
	void fdc_drq_w(int state);
	TIMER_CALLBACK_MEMBER(flex_step);
	TIMER_CALLBACK_MEMBER(head_unload);
	TIMER_CALLBACK_MEMBER(panel_update);
	TIMER_CALLBACK_MEMBER(bell_off) { m_bell->set_state(0); }
	IRQ_CALLBACK_MEMBER(irq_ack);
};


/***************************************************************************
    MEMORY
***************************************************************************/

u8 ncr9010_state::mem_r(offs_t offset)
{
	if ((offset < 0x1000) && BIT(m_memctrl, 0))
		return m_rom[offset];

	if ((offset >= 0xfc00) && (offset < 0xfe00))
	{
		if (offset < 0xfd00)
			return io_r(offset & 0xff);
		return flex_r(offset & 0xff);
	}

	return m_ram[phys(offset)];
}

void ncr9010_state::mem_w(offs_t offset, u8 data)
{
	if (m_map_arm)
	{
		m_map_arm = false;
		if (((offset >> 8) & 3) == 1)
		{
			u8 const set = (offset >> 10) & 3;
			u8 const page = offset >> 12;
			m_map[set][page] = ~(offset >> 2) & 0x3f;
			LOGMMU("map set %u page %X -> %02X\n", set, page, m_map[set][page]);
		}
		else
		{
			LOGMMU("%s: MMU control write %04X = %02X\n", machine().describe_context(), offset, data);
		}
		return;
	}

	if ((offset >= 0xfc00) && (offset < 0xfe00))
	{
		if (offset < 0xfd00)
			io_w(offset & 0xff, data);
		else
			flex_w(offset & 0xff, data);
		return;
	}

	m_ram[phys(offset)] = data;
}

void ncr9010_state::mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(ncr9010_state::mem_r), FUNC(ncr9010_state::mem_w));
}

void ncr9010_state::io_map(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(ncr9010_state::io_r), FUNC(ncr9010_state::io_w));
}


/***************************************************************************
    I/O
***************************************************************************/

u8 ncr9010_state::io_r(offs_t offset)
{
	switch (offset)
	{
	case 0x05: // TRAP cause
		return m_trap_cause;

	case 0x21:
		return m_mode->read();

	case 0x23:
		return m_config->read();

	case 0x28: // keyboard data
		if (m_kbd_count)
		{
			u8 const data = m_kbd_fifo[m_kbd_head];
			if (!machine().side_effects_disabled())
			{
				m_kbd_head = (m_kbd_head + 1) % std::size(m_kbd_fifo);
				m_kbd_count--;
			}
			return data;
		}
		return 0x00;

	case 0x29: // keyboard status
		return m_kbd_count ? 0x10 : 0x00;

	case 0x31: // printer 1 status, 0 = no error
		return 0x00;

	case 0x02: // DMV link status: idle (nothing received, not ready to send)
		return 0x00;

	case 0x41: // load device 2 status
		return 0x00;

	case 0x61: // fixed disk: service request + "not ready"
		return 0x44;

	case 0x8c: case 0x8d: case 0x8e: case 0x8f:
		return m_fx_dmareg[offset & 3];

	case 0xc0:
		return m_cur_x;
	case 0xc1:
		return m_cur_y;
	case 0xc2:
		return m_vram[(m_cur_y * 80 + m_cur_x) & 0x7ff];
	case 0xc3:
		return m_vattr[(m_cur_y * 80 + m_cur_x) & 0x7ff];

	default:
		if (!machine().side_effects_disabled())
			LOGIO("%s: unknown port read %02X\n", machine().describe_context(), offset);
		return 0xff;
	}
}

void ncr9010_state::io_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0x01: // CCM indicator
		for (int i = 0; i < 8; i++)
			m_ccm_leds[i] = BIT(data, i);
		break;

	case 0x05: // acknowledge TRAP
		m_trap_cause = 0;
		m_maincpu->set_input_line(I8085_TRAP_LINE, CLEAR_LINE);
		break;

	case 0x04: // memory control
		m_memctrl = data;
		m_map_arm = BIT(data, 2);
		break;

	case 0x2e: // keyboard bell
		m_bell->set_state(1);
		m_bell_timer->adjust(attotime::from_msec(150)); // length not verified
		break;

	case 0x28: case 0x29: case 0x2a: case 0x2b:
	case 0x2c: case 0x2d: case 0x2f: // keyboard data/control
		if (offset == 0x2f)
			m_kbd_head = m_kbd_count = 0;
		break;

	case 0x30: // printer 1 data
		m_printer->output(data);
		break;

	case 0x8c: case 0x8d: case 0x8e: case 0x8f:
		m_fx_dmareg[offset & 3] = data;
		break;

	case 0xc0:
		m_cur_x = data;
		break;
	case 0xc1:
		m_cur_y = data;
		break;
	case 0xc2:
		m_vram[(m_cur_y * 80 + m_cur_x) & 0x7ff] = data;
		if (++m_cur_x >= 80)
		{
			m_cur_x = 0;
			if (++m_cur_y >= 24)
				m_cur_y = 0;
		}
		break;
	case 0xc3:
		m_vattr[(m_cur_y * 80 + m_cur_x) & 0x7ff] = data;
		break;
	case 0xc4:
		m_crt_ctrl = data;
		break;

	default:
		LOGIO("%s: port write %02X = %02X\n", machine().describe_context(), offset, data);
		break;
	}
}


/***************************************************************************
    KEYBOARD (HLE)
***************************************************************************/

void ncr9010_state::kbd_put(u8 data)
{
	switch (data)
	{
	case 0x0d: data = 0x88; break; // RETURN
	case 0x08: data = 0x8b; break; // backspace
	case 0x1b: data = 0x80; break; // ESC
	case 0x09: data = 0x86; break; // TAB
	default: break;
	}

	if (m_kbd_count < std::size(m_kbd_fifo))
	{
		m_kbd_fifo[(m_kbd_head + m_kbd_count) % std::size(m_kbd_fifo)] = data;
		m_kbd_count++;
	}
}


/***************************************************************************
    VIDEO
***************************************************************************/

void ncr9010_state::video_start()
{
	m_subst_font = true;
	for (unsigned i = 0; i < 0x400; i++)
		if (m_chargen[i])
			m_subst_font = false;
}

u32 ncr9010_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	pen_t const *const pens = m_palette->pens();
	bool const blink = BIT(screen.frame_number(), 4);

	bitmap.fill(pens[0], cliprect);
	if (!BIT(m_crt_ctrl, 2))
		return 0;

	for (int y = 0; y < 24; y++)
	{
		for (int x = 0; x < 80; x++)
		{
			unsigned const pos = y * 80 + x;
			u8 const chr = m_vram[pos];
			bool const inv = BIT(chr, 7) || (m_vattr[pos] & 0x01);
			bool const cursor = (x == m_cur_x) && (y == m_cur_y) && blink;
			u8 const *const gfx = m_subst_font ? &subst_chargen[(chr & 0x7f) * 8] : &m_chargen[(chr & 0x7f) * 8];

			for (int ra = 0; ra < 10; ra++)
			{
				u8 bits = (ra < 8) ? gfx[ra] : 0;
				if (inv)
					bits = ~bits;
				if (cursor && (ra == 9))
					bits = 0xff;

				u32 *const p = &bitmap.pix(y * 10 + ra, x * 8);
				for (int b = 0; b < 8; b++)
					p[b] = pens[BIT(bits, 7 - b)];
			}
		}
	}

	return 0;
}


/***************************************************************************
    FLEX DISK CONTROLLER (HLE of the I/O Processor)
***************************************************************************/

u8 ncr9010_state::flex_r(offs_t offset)
{
	switch (offset)
	{
	case 0x01:
		return m_fx_status;
	case 0x02:
		return m_fx_status2;
	default:
		if (!machine().side_effects_disabled())
			LOGFLEX("%s: flex read %02X\n", machine().describe_context(), offset);
		return 0xff;
	}
}

void ncr9010_state::flex_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0x01: // command byte
		if (BIT(m_fx_status, 7))
			break;
		m_fx_cmdbuf[m_fx_cmdcnt++] = data;
		if (m_fx_cmdcnt == 4)
		{
			m_fx_cmdcnt = 0;
			flex_command();
		}
		break;

	case 0x02: // interrupt on
		m_fx_inten = 1;
		break;

	case 0x03: // interrupt off
		m_fx_inten = 0;
		break;

	case 0x04: // reset service request
		m_fx_status &= ~0x40;
		break;

	case 0x07: // reset controller
		m_fx_cmdcnt = 0;
		m_fx_state = FX_IDLE;
		m_fx_status = 0x00;
		break;

	default:
		LOGFLEX("%s: flex write %02X = %02X\n", machine().describe_context(), offset, data);
		break;
	}
}

void ncr9010_state::flex_select()
{
	m_fx_floppy = (m_fx_unit < 4) ? m_floppy[m_fx_unit]->get_device() : nullptr;
	m_fdc->set_floppy(m_fx_floppy);
	if (m_fx_floppy)
	{
		m_fx_floppy->mon_w(0);
		m_fx_floppy->ss_w(m_fx_head);
	}
	m_fdc->dden_w(m_fx_dd ? 0 : 1);
}

void ncr9010_state::flex_command()
{
	m_fx_cmd = m_fx_cmdbuf[0] & 0x0f; // high nibble: ASCII/EBCDIC conversion (not emulated)
	m_fx_unit = (m_fx_cmdbuf[1] >> 1) & 3;
	m_fx_head = BIT(m_fx_cmdbuf[1], 0);
	m_fx_dd = BIT(m_fx_cmdbuf[1], 7);
	m_fx_cyl = m_fx_cmdbuf[2];
	m_fx_sec = m_fx_cmdbuf[3];
	m_fx_addr = m_fx_dmareg[0] | (m_fx_dmareg[1] << 8) | (u32(BIT(m_fx_dmareg[3], 4, 2)) << 16);
	m_fx_count = m_fx_dmareg[2] | ((m_fx_dmareg[3] & 0x0f) << 8);
	if (!m_fx_count)
		m_fx_count = 0x1000;

	LOGFLEX("flex cmd %02X unit %u head %u %s cyl %u sec %u dma %05X count %03X dir %02X\n",
			m_fx_cmd, m_fx_unit, m_fx_head, m_fx_dd ? "MFM" : "FM", m_fx_cyl, m_fx_sec,
			m_fx_addr, m_fx_count, m_fx_dmareg[3] & 0xc0);

	m_fx_status = 0x80;
	m_fx_status2 = 0x00;
	flex_select();

	m_hld_timer->adjust(attotime::never);
	for (unsigned i = 0; i < 4; i++)
		m_busy_leds[i] = (i == m_fx_unit) && m_fx_floppy && m_fx_floppy->exists();

	if (!m_fx_floppy || !m_fx_floppy->exists())
	{
		flex_finish(RES_NOTREADY);
		return;
	}

	switch (m_fx_cmd)
	{
	case 0x00: // restore
		m_fx_state = FX_RESTORE;
		m_fdc->cmd_w(0x00);
		break;

	case 0x01: // write format
	case 0x02: // read format (IDs)
	case 0x03: // write data
	case 0x04: // read data
	case 0x07: // verify
		if (m_fx_cmd == 0x01)
			flex_build_track();
		m_fx_ids = 0;
		if (m_fx_curcyl[m_fx_unit] == 0xff)
		{
			// head position unknown: recalibrate first
			m_fx_state = FX_RECAL;
			m_fdc->cmd_w(0x00);
		}
		else if (m_fx_curcyl[m_fx_unit] != m_fx_cyl)
			flex_seek();
		else
			flex_start_transfer();
		break;

	default:
		logerror("flex: unsupported command %02X\n", m_fx_cmd);
		flex_finish(RES_BADCMD);
		break;
	}
}

void ncr9010_state::flex_seek()
{
	m_fx_state = FX_SEEK;
	m_fdc->track_w(m_fx_curcyl[m_fx_unit]);
	m_fdc->data_w(m_fx_cyl);
	m_fdc->cmd_w(0x10);
}

void ncr9010_state::flex_start_transfer()
{
	m_fdc->track_w(m_fx_cyl);
	switch (m_fx_cmd)
	{
	case 0x01:
		m_fx_state = FX_FORMAT;
		m_fx_trackpos = 0;
		m_fdc->cmd_w(0xf0);
		break;

	case 0x02:
		m_fx_state = FX_READID;
		m_fx_idpos = 0;
		m_fdc->cmd_w(0xc0);
		break;

	default:
		m_fx_state = FX_XFER;
		m_fdc->sector_w(m_fx_sec);
		m_fdc->cmd_w((m_fx_cmd == 0x03) ? 0xa0 : 0x80);
		break;
	}
}

// IBM style track for the "write format" command; DMA supplies 4 ID bytes per sector
void ncr9010_state::flex_build_track()
{
	std::vector<u8> &t = m_fx_trackbuf;
	t.clear();
	auto put = [&t] (u8 val, unsigned n) { t.insert(t.end(), n, val); };

	u8 ids[SECTORS_PER_TRACK * 4];
	for (unsigned i = 0; i < std::size(ids); i++)
		ids[i] = dma_r(m_fx_addr + i);

	if (m_fx_dd)
	{
		put(0x4e, 80); put(0x00, 12); put(0xf6, 3); put(0xfc, 1); put(0x4e, 50);
		for (unsigned s = 0; s < SECTORS_PER_TRACK; s++)
		{
			put(0x00, 12); put(0xf5, 3); put(0xfe, 1);
			t.insert(t.end(), &ids[s * 4], &ids[s * 4 + 4]);
			put(0xf7, 1); put(0x4e, 22); put(0x00, 12); put(0xf5, 3); put(0xfb, 1);
			put(0xe5, 128 << (ids[s * 4 + 3] & 3)); put(0xf7, 1); put(0x4e, 54);
		}
	}
	else
	{
		put(0xff, 40); put(0x00, 6); put(0xfc, 1); put(0xff, 26);
		for (unsigned s = 0; s < SECTORS_PER_TRACK; s++)
		{
			put(0x00, 6); put(0xfe, 1);
			t.insert(t.end(), &ids[s * 4], &ids[s * 4 + 4]);
			put(0xf7, 1); put(0xff, 11); put(0x00, 6); put(0xfb, 1);
			put(0xe5, 128 << (ids[s * 4 + 3] & 3)); put(0xf7, 1); put(0xff, 27);
		}
	}
	m_fx_count = 0;
}

void ncr9010_state::fdc_drq_w(int state)
{
	if (!state)
		return;

	switch (m_fx_state)
	{
	case FX_XFER:
		if (m_fx_cmd == 0x03)
		{
			u8 data = 0x00;
			if (m_fx_count)
			{
				data = dma_r(m_fx_addr++);
				m_fx_count--;
			}
			m_fdc->data_w(data);
		}
		else
		{
			u8 const data = m_fdc->data_r();
			if (m_fx_count && (m_fx_cmd == 0x04))
			{
				dma_w(m_fx_addr++, data);
				m_fx_count--;
			}
			else if (m_fx_count)
				m_fx_count--;
		}
		break;

	case FX_READID:
	{
		u8 const data = m_fdc->data_r();
		if (m_fx_idpos < 6)
			m_fx_idbuf[m_fx_idpos++] = data;
		break;
	}

	case FX_FORMAT:
		m_fdc->data_w((m_fx_trackpos < m_fx_trackbuf.size()) ? m_fx_trackbuf[m_fx_trackpos++] : (m_fx_dd ? 0x4e : 0xff));
		break;

	default:
		m_fdc->data_r();
		break;
	}
}

void ncr9010_state::fdc_intrq_w(int state)
{
	if (state && (m_fx_state != FX_IDLE))
		m_fx_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(ncr9010_state::flex_step)
{
	u8 const st = m_fdc->status_r();

	switch (m_fx_state)
	{
	case FX_RESTORE:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else if (!BIT(st, 2))
			flex_finish(RES_SOFT, ST2_SEEK);
		else
		{
			m_fx_curcyl[m_fx_unit] = 0;
			flex_finish(RES_OK);
		}
		break;

	case FX_RECAL:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else if (!BIT(st, 2))
			flex_finish(RES_SOFT, ST2_SEEK);
		else
		{
			m_fx_curcyl[m_fx_unit] = 0;
			if (m_fx_cyl)
				flex_seek();
			else
				flex_start_transfer();
		}
		break;

	case FX_SEEK:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else
		{
			m_fx_curcyl[m_fx_unit] = m_fx_cyl;
			flex_start_transfer();
		}
		break;

	case FX_XFER:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else if (BIT(st, 6) && (m_fx_cmd == 0x03))
			flex_finish(RES_SOFT, ST2_WFAULT);
		else if (BIT(st, 4))
			flex_finish(RES_SOFT, ST2_RNF);
		else if (BIT(st, 3))
			flex_finish((m_fx_cmd == 0x07) ? RES_VERIFY : RES_SOFT, ST2_CRC);
		else if (BIT(st, 2))
			flex_finish(RES_SOFT, ST2_DMATIME);
		else if (!m_fx_count)
			flex_finish(RES_OK);
		else
		{
			// multi-sector transfer: next sector, then next cylinder on the same head
			if (++m_fx_sec > SECTORS_PER_TRACK)
			{
				m_fx_sec = 1;
				m_fx_cyl++;
				flex_seek();
			}
			else
				flex_start_transfer();
		}
		break;

	case FX_READID:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else if (BIT(st, 4))
			flex_finish(RES_SOFT, ST2_RNF);
		else
		{
			for (unsigned i = 0; (i < 4) && m_fx_count; i++, m_fx_count--)
				dma_w(m_fx_addr++, m_fx_idbuf[i]);
			if (m_fx_count && (++m_fx_ids < SECTORS_PER_TRACK))
				flex_start_transfer();
			else
				flex_finish(RES_OK);
		}
		break;

	case FX_FORMAT:
		if (BIT(st, 7))
			flex_finish(RES_NOTREADY);
		else if (BIT(st, 6))
			flex_finish(RES_SOFT, ST2_WFAULT);
		else
			flex_finish(RES_OK);
		break;

	default:
		break;
	}
}

void ncr9010_state::flex_finish(u8 result, u8 status2)
{
	LOGFLEX("flex result %02X/%02X\n", result, status2);
	m_fx_status2 = status2;
	m_fx_state = FX_IDLE;
	m_fx_status = 0x40 | result;
	m_hld_timer->adjust(attotime::from_msec(300)); // head unload delay
}

TIMER_CALLBACK_MEMBER(ncr9010_state::head_unload)
{
	if (m_fx_state == FX_IDLE)
		for (auto &led : m_busy_leds)
			led = 0;
}


/***************************************************************************
    FRONT PANEL
***************************************************************************/

TIMER_CALLBACK_MEMBER(ncr9010_state::panel_update)
{
	for (unsigned i = 0; i < 4; i++)
	{
		floppy_image_device *const floppy = m_floppy[i]->get_device();
		m_ready_leds[i] = floppy && floppy->exists();
	}
}

INPUT_CHANGED_MEMBER(ncr9010_state::monit_pressed)
{
	if (newval && !BIT(m_trap_cause, 6))
	{
		m_trap_cause |= 0x40;
		m_maincpu->set_input_line(I8085_TRAP_LINE, ASSERT_LINE);
	}
}

IRQ_CALLBACK_MEMBER(ncr9010_state::irq_ack)
{
	// assumed: taking TRAP switches the boot ROM in (its handler is at 0024h)
	if (irqline == I8085_TRAP_LINE)
		m_memctrl |= 0x01;
	return 0xff;
}


/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( ncr9010 )
	PORT_START("MODE")
	PORT_DIPNAME( 0x80, 0x80, "Mode" )
	PORT_DIPSETTING(    0x80, "Normal" )
	PORT_DIPSETTING(    0x00, "ROM test (selected by CONFIG)" )
	PORT_DIPNAME( 0x04, 0x00, "Start" )
	PORT_DIPSETTING(    0x00, "Memory test, keep memory" )
	PORT_DIPSETTING(    0x04, "Clear memory / RESTORE? prompt" )
	PORT_BIT( 0x7b, 0x00, IPT_UNUSED )

	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MONIT") PORT_CODE(KEYCODE_MENU) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(ncr9010_state::monit_pressed), 0)

	// configuration switches, names after the I-8100 pocket reference
	PORT_START("CONFIG")
	PORT_DIPNAME( 0x01, 0x01, "CRT" ) PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x01, "12-inch (80 x 24)" )
	PORT_DIPSETTING(    0x00, "9-inch (32 x 16)" )
	PORT_DIPUNUSED_DIPLOC( 0x02, 0x00, "SW1:2" )
	PORT_DIPNAME( 0x04, 0x04, "Flexible disk is primary load device" ) PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x08, 0x00, "Fixed disk is primary load device" ) PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x10, 0x00, "Data cartridge is primary load device" ) PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x20, 0x00, "Battery back-up for CCM memory" ) PORT_DIPLOCATION("SW2:2") // enables the RESTORE? prompt
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Yes ) )
	PORT_DIPUNUSED_DIPLOC( 0x40, 0x00, "SW2:3" )
	PORT_DIPNAME( 0x80, 0x00, "Fixed disk unit (fixed disk load device only)" ) PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, "Disk 0" )
	PORT_DIPSETTING(    0x80, "Disk 1" )
INPUT_PORTS_END


/***************************************************************************
    MACHINE
***************************************************************************/

void ncr9010_state::machine_start()
{
	m_ram = make_unique_clear<u8[]>(0x40000);
	m_fx_timer = timer_alloc(FUNC(ncr9010_state::flex_step), this);
	m_hld_timer = timer_alloc(FUNC(ncr9010_state::head_unload), this);
	m_panel_timer = timer_alloc(FUNC(ncr9010_state::panel_update), this);
	m_bell_timer = timer_alloc(FUNC(ncr9010_state::bell_off), this);
	m_panel_timer->adjust(attotime::from_hz(20), 0, attotime::from_hz(20));

	m_power_led = 1;
	m_ccm_run = 1;
	m_comm_led = 0;

	std::fill(std::begin(m_vram), std::end(m_vram), 0x20);
	std::fill(std::begin(m_vattr), std::end(m_vattr), 0x00);

	save_pointer(NAME(m_ram), 0x40000);
	save_item(NAME(m_map));
	save_item(NAME(m_memctrl));
	save_item(NAME(m_map_arm));
	save_item(NAME(m_trap_cause));
	save_item(NAME(m_vram));
	save_item(NAME(m_vattr));
	save_item(NAME(m_cur_x));
	save_item(NAME(m_cur_y));
	save_item(NAME(m_crt_ctrl));
	save_item(NAME(m_kbd_fifo));
	save_item(NAME(m_kbd_head));
	save_item(NAME(m_kbd_count));
	save_item(NAME(m_fx_cmdbuf));
	save_item(NAME(m_fx_cmdcnt));
	save_item(NAME(m_fx_status));
	save_item(NAME(m_fx_status2));
	save_item(NAME(m_fx_state));
	save_item(NAME(m_fx_cmd));
	save_item(NAME(m_fx_unit));
	save_item(NAME(m_fx_head));
	save_item(NAME(m_fx_cyl));
	save_item(NAME(m_fx_sec));
	save_item(NAME(m_fx_dd));
	save_item(NAME(m_fx_curcyl));
	save_item(NAME(m_fx_dmareg));
	save_item(NAME(m_fx_addr));
	save_item(NAME(m_fx_count));
	save_item(NAME(m_fx_inten));
	save_item(NAME(m_fx_trackpos));
	save_item(NAME(m_fx_ids));
	save_item(NAME(m_fx_idbuf));
	save_item(NAME(m_fx_idpos));
}

void ncr9010_state::machine_reset()
{
	// MMU maps come up as identity (the ROM uses 3F00h-3FFFh before loading them)
	for (auto &set : m_map)
		for (unsigned i = 0; i < 16; i++)
			set[i] = i;
	m_memctrl = 0x01;
	m_map_arm = false;
	m_trap_cause = 0;
	m_maincpu->set_input_line(I8085_TRAP_LINE, CLEAR_LINE);
	for (auto &led : m_busy_leds)
		led = 0;

	m_cur_x = m_cur_y = 0;
	m_crt_ctrl = 0;
	m_kbd_head = m_kbd_count = 0;

	m_fx_cmdcnt = 0;
	m_fx_status = 0x00;
	m_fx_status2 = 0x00;
	m_fx_state = FX_IDLE;
	m_fx_inten = 0;
	std::fill(std::begin(m_fx_curcyl), std::end(m_fx_curcyl), 0xff);
	std::fill(std::begin(m_fx_dmareg), std::end(m_fx_dmareg), 0);
	m_fx_floppy = nullptr;

	// 8" drives spin all the time
	for (auto &f : m_floppy)
		if (f->get_device())
			f->get_device()->mon_w(0);
}

void ncr9010_state::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_IMD_FORMAT);
}

static void ncr9010_floppies(device_slot_interface &device)
{
	device.option_add("8dsdd", FLOPPY_8_DSDD);
}

void ncr9010_state::ncr9010(machine_config &config)
{
	I8085A(config, m_maincpu, 27.648_MHz_XTAL / 4); // divider not verified
	m_maincpu->set_addrmap(AS_PROGRAM, &ncr9010_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &ncr9010_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(ncr9010_state::irq_ack));

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_color(rgb_t::green());
	screen.set_refresh_hz(60);
	screen.set_vblank_time(ATTOSECONDS_IN_USEC(2500));
	screen.set_size(640, 240);
	screen.set_visarea_full();
	screen.set_screen_update(FUNC(ncr9010_state::screen_update));
	PALETTE(config, m_palette, palette_device::MONOCHROME);

	generic_keyboard_device &keyboard(GENERIC_KEYBOARD(config, "keyboard", 0));
	keyboard.set_keyboard_callback(FUNC(ncr9010_state::kbd_put));

	// HLE of the I/O Processor + Flex Disk IF
	FD1793(config, m_fdc, 2_MHz_XTAL);
	m_fdc->set_disable_motor_control(true);
	m_fdc->intrq_wr_callback().set(FUNC(ncr9010_state::fdc_intrq_w));
	m_fdc->drq_wr_callback().set(FUNC(ncr9010_state::fdc_drq_w));
	FLOPPY_CONNECTOR(config, m_floppy[0], ncr9010_floppies, "8dsdd", ncr9010_state::floppy_formats).enable_sound(true);
	FLOPPY_CONNECTOR(config, m_floppy[1], ncr9010_floppies, "8dsdd", ncr9010_state::floppy_formats).enable_sound(true);
	FLOPPY_CONNECTOR(config, m_floppy[2], ncr9010_floppies, nullptr, ncr9010_state::floppy_formats);
	FLOPPY_CONNECTOR(config, m_floppy[3], ncr9010_floppies, nullptr, ncr9010_state::floppy_formats);

	PRINTER(config, m_printer, 0);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_bell, 1000).add_route(ALL_OUTPUTS, "mono", 0.5); // frequency not verified

	config.set_default_layout(layout_ncr9010);
}


/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

ROM_START( ncr9010 )
	ROM_REGION( 0x1000, "maincpu", ROMREGION_INVERT ) // data lines inverted
	ROM_LOAD( "1023.j1", 0x0000, 0x0800, CRC(072a6f95) SHA1(8502f68640a8ecd7529632ca369f78181b3cf35a) )
	ROM_LOAD( "1024.f1", 0x0800, 0x0800, CRC(1f336b9a) SHA1(0c36d83bf27e99fef25f573c2df41dad370e9a08) )

	ROM_REGION( 0x800, "chargen", ROMREGION_ERASE00 ) // video board, contents unknown
	ROM_LOAD( "6-1043026.q5", 0x000, 0x400, NO_DUMP ) // Intel D2608
	ROM_LOAD( "9709.r5",      0x400, 0x400, NO_DUMP ) // type hidden under label, size unknown

	ROM_REGION( 0x20, "vprom", 0 )
	ROM_LOAD( "6331.d2", 0x00, 0x20, NO_DUMP )

	ROM_REGION( 0xa00, "iop", 0 ) // I/O Processor microcode, MMI 6341 (512 x 8)
	ROM_LOAD( "1117.bin", 0x000, 0x200, NO_DUMP )
	ROM_LOAD( "1118.bin", 0x200, 0x200, NO_DUMP )
	ROM_LOAD( "1119.bin", 0x400, 0x200, NO_DUMP )
	ROM_LOAD( "1120.bin", 0x600, 0x200, NO_DUMP )
	ROM_LOAD( "1122.bin", 0x800, 0x200, NO_DUMP )

	ROM_REGION( 0x400, "flexif", 0 ) // Flex Disk IF, type and size unknown
	ROM_LOAD( "502.c3", 0x000, 0x200, NO_DUMP )
	ROM_LOAD( "177.h4", 0x200, 0x100, NO_DUMP )
ROM_END

} // anonymous namespace


//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY  FULLNAME     FLAGS
COMP( 1981, ncr9010, 0,      0,      ncr9010, ncr9010, ncr9010_state, empty_init, "NCR",   "NCR 9010",  MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
