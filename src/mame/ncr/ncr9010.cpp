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
               empty third socket G1, 93419 64x9 bipolar RAM, CCM indicator LEDs
      M217-02  memory board, 36 x 64Kx1 DRAM (4 of 8 rows fitted, parity)
      M215-01  discrete DMA board
      M223-06  video board, no CRTC (TTL timing), Intel D2608 "6-1043026" and
               EPROM "9709" (character generator?), MMI 6331 PROM, 18 x 1Kx1 SRAM
      I/O Processor: microcoded, 2 x Am2901, 5 x MMI 6341 PROMs (1117..1120, 1122)
      Flex Disk IF: discrete drive interface, PROMs "502" and "177"

    The two boot EPROMs are stored with inverted data lines.

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

    I/O ports used by the boot ROM and SB-80:
      01  RW  CCM indicator LEDs (POST step / error codes); also the data
              port of the byte link to a DMV (data inverted, see below)
      02  RW  byte link status: bits 1-0 = 3 byte received, 2 ready to send
      04   W  memory control: bit 0 boot ROM enable, bit 2 arm MMU write
      05  RW  interrupt / status (used by the TRAP monitor, not emulated)
      12   W  written 00 at reset
      21   R  mode switches (bit 7 = 0 selects ROM test modes)
      23   R  configuration switches (default boot device, screen format)
      28-2F   keyboard (data, status 10=data 20=not accepted 40=break, control)
      30-37   parallel printer 1 (data 30, status 31, disable int 33, reset 37)
      41-47   load device "2" (unknown; not emulated)
      60-67   fixed disk controller (not emulated, reports "not ready")
      8C-8F   flex disk DMA: address L/H, count L, count H (bits 3-0) + direction
      A4-A7   fixed disk DMA
      C0-C4   CRT: cursor X, cursor Y, display char (auto increment),
              control character / attribute, display enable

    Flex disk controller (FD01 command/status, FD02 status 2 / int on,
    FD03 int off, FD04 reset service request, FD07 reset):
      the CPU writes four bytes: command, unit (bits 2-1) / side (bit 0) /
      double density (bit 7), cylinder, sector.  00 restore, 01 format
      track, 02 read IDs, 03 write, 04 read, 07 verify.  Data moves by DMA
      for the full count; the controller continues with the next sector and
      the next cylinder (same head) until the count is exhausted.  Status:
      bit 7 busy, bit 6 service request, bits 3-0 result (2 = ok, 4 = not
      ready, 5 = retry restore).  The real I/O Processor microcode is not
      dumped, so this is a high-level emulation driving an FD1793.

    Disk format (SB-80): cylinder 0 head 0 FM 26 x 128 ("NCR FMT-" ID
    block), all other tracks MFM 26 x 256.  Logical track 1 = cylinder 72,
    logical track 2 = cylinder 73 (boot sector, BIOS, DOS); the boot ROM
    reads cylinder 72 sectors 1-26 and cylinder 73 sectors 1-6 to 0100h and
    requires a zero 8-bit sum over 0100h-20FFh.

    Fixed disk controller (SBIOS 1.8 / 1.7, boot ROM load device "1"):
      FC61 command/status, FC62 status 2, FC64 reset service request.
      The CPU writes four bytes, waiting for bit 7 (busy) to clear between
      them: command (00 restore, 04 write, 05 read), unit (0/1), block
      number low, block number high.  Blocks are 512 bytes, addressed
      linearly (BIOS: track * 24 + sector - 1), so the controller handles
      the drive geometry itself.  Completion sets bit 6 (service request),
      result = status & 1F (02 ok, 08 not ready).  DMA on FCA4-FCA7 with
      the same layout as the flex disk DMA.  SB-80 drives C: and D: are one
      unit each: 24 x 512 bytes per track, 4 KB blocks, 1200 blocks
      (4.8 MB), 512 directory entries, 1 reserved track.  Block 11 holds
      an alternate track table (6 entries, lookup disabled in SBIOS 1.8).
      The boot ROM loads blocks 12-19 to 0100h and 20-27 to 1100h (same
      checksum rule as the flex disk); CONFIG bit 7 selects the unit.
      The controller hardware is unknown (no board in the photos).  Its
      protocol matches the I/O Processor style of the flex disk controller,
      not a WD1001 task file and not the Corvus disk command set.

    DMV link (SB-80 "Fix Disk Version 1.7 WITH INTERRUPT"): no hardware
    interrupts; the BIOS hooks the BDOS entry and the console wait loops
    and polls port 02.  When a byte is pending (status 3) it receives a
    disk request block over port 01 (inverted data), runs it on the local
    fixed disk and transfers 512 byte sectors over the link, making the
    9010 a fixed disk server for a DMV (see SEND9010/EMPF9010 and
    SENDDMV/EMPFDMV, DMV side on ports C0-C3).  The link is reported idle.

    TODO:
    - dump character generator, video PROMs, IOP microcode, Flex IF PROMs
    - real video timing, attributes, cursor shape
    - keyboard is HLE (ASCII + 80h-9Fh function codes from the SB-80 key table)
    - DMV byte link (ports 01/02)
    - interrupts (TRAP monitor, keyboard, disk), serial port, fixed disk,
      load device 2, third ROM socket ("EXTFW" extension firmware at 1000h)

****************************************************************************/

#include "emu.h"

#include "cpu/i8085/i8085.h"
#include "imagedev/floppy.h"
#include "imagedev/printer.h"
#include "machine/keyboard.h"
#include "machine/wd_fdc.h"

#include "emupal.h"
#include "screen.h"

#include "formats/imd_dsk.h"

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
		, m_palette(*this, "palette")
		, m_rom(*this, "maincpu")
		, m_chargen(*this, "chargen")
		, m_mode(*this, "MODE")
		, m_config(*this, "CONFIG")
		, m_leds(*this, "led%u", 0U)
	{ }

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

	enum : u8
	{
		RES_OK       = 0x02,
		RES_NOTREADY = 0x04,
		RES_SEEK     = 0x05,
		RES_CRC      = 0x08,
		RES_RNF      = 0x0a,
		RES_LOST     = 0x0b,
		RES_WPROT    = 0x0c,
		RES_BADCMD   = 0x0e
	};

	required_device<i8085a_cpu_device> m_maincpu;
	required_device<fd1793_device> m_fdc;
	required_device_array<floppy_connector, 4> m_floppy;
	required_device<printer_image_device> m_printer;
	required_device<palette_device> m_palette;
	required_region_ptr<u8> m_rom;
	required_region_ptr<u8> m_chargen;
	required_ioport m_mode;
	required_ioport m_config;
	output_finder<8> m_leds;

	std::unique_ptr<u8[]> m_ram;
	u8 m_map[4][16];
	u8 m_memctrl;
	bool m_map_arm;

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
	u16 m_fx_addr, m_fx_count;
	u8 m_fx_inten;
	std::vector<u8> m_fx_trackbuf;
	unsigned m_fx_trackpos, m_fx_ids;
	u8 m_fx_idbuf[6];
	unsigned m_fx_idpos;
	floppy_image_device *m_fx_floppy;
	emu_timer *m_fx_timer;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	static void floppy_formats(format_registration &fr);

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	u32 phys(offs_t offset) const { return (u32(m_map[0][offset >> 12]) << 12) | (offset & 0x0fff); }
	u8 dma_r(u16 addr) { return mem_r(addr); }
	void dma_w(u16 addr, u8 data) { mem_w(addr, data); }

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void kbd_put(u8 data);

	u8 flex_r(offs_t offset);
	void flex_w(offs_t offset, u8 data);
	void flex_command();
	void flex_finish(u8 result);
	void flex_select();
	void flex_seek();
	void flex_start_transfer();
	void flex_build_track();
	void fdc_intrq_w(int state);
	void fdc_drq_w(int state);
	TIMER_CALLBACK_MEMBER(flex_step);
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
			m_leds[i] = BIT(data, i);
		break;

	case 0x04: // memory control
		m_memctrl = data;
		m_map_arm = BIT(data, 2);
		break;

	case 0x28: case 0x29: case 0x2a: case 0x2b:
	case 0x2c: case 0x2d: case 0x2e: case 0x2f: // keyboard control
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
	m_fx_cmd = m_fx_cmdbuf[0];
	m_fx_unit = (m_fx_cmdbuf[1] >> 1) & 3;
	m_fx_head = BIT(m_fx_cmdbuf[1], 0);
	m_fx_dd = BIT(m_fx_cmdbuf[1], 7);
	m_fx_cyl = m_fx_cmdbuf[2];
	m_fx_sec = m_fx_cmdbuf[3];
	m_fx_addr = m_fx_dmareg[0] | (m_fx_dmareg[1] << 8);
	m_fx_count = m_fx_dmareg[2] | ((m_fx_dmareg[3] & 0x0f) << 8);
	if (!m_fx_count)
		m_fx_count = 0x1000;

	LOGFLEX("flex cmd %02X unit %u head %u %s cyl %u sec %u dma %04X count %03X dir %02X\n",
			m_fx_cmd, m_fx_unit, m_fx_head, m_fx_dd ? "MFM" : "FM", m_fx_cyl, m_fx_sec,
			m_fx_addr, m_fx_count, m_fx_dmareg[3] & 0xc0);

	m_fx_status = 0x80;
	flex_select();

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
			flex_finish(RES_SEEK);
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
			flex_finish(RES_SEEK);
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
			flex_finish(RES_WPROT);
		else if (BIT(st, 4))
			flex_finish(RES_RNF);
		else if (BIT(st, 3))
			flex_finish(RES_CRC);
		else if (BIT(st, 2))
			flex_finish(RES_LOST);
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
			flex_finish(RES_RNF);
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
			flex_finish(RES_WPROT);
		else
			flex_finish(RES_OK);
		break;

	default:
		break;
	}
}

void ncr9010_state::flex_finish(u8 result)
{
	LOGFLEX("flex result %02X\n", result);
	m_fx_state = FX_IDLE;
	m_fx_status = 0x40 | result;
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

	PORT_START("CONFIG")
	PORT_DIPNAME( 0x5e, 0x04, "Default load device" )
	PORT_DIPSETTING(    0x04, "3 - Flex disk" )
	PORT_DIPSETTING(    0x08, "1 - Fixed disk" )
	PORT_DIPSETTING(    0x10, "2 - Load device 2" )
	PORT_DIPSETTING(    0x00, "None (ask)" )
	PORT_DIPNAME( 0x01, 0x01, "Screen format" )
	PORT_DIPSETTING(    0x01, "80 x 24" )
	PORT_DIPSETTING(    0x00, "32 x 16" )
	PORT_DIPNAME( 0x20, 0x00, "RESTORE? prompt" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, "Fixed disk boot sector" )
	PORT_DIPSETTING(    0x00, "0" )
	PORT_DIPSETTING(    0x80, "1" )
INPUT_PORTS_END


/***************************************************************************
    MACHINE
***************************************************************************/

void ncr9010_state::machine_start()
{
	m_ram = make_unique_clear<u8[]>(0x40000);
	m_fx_timer = timer_alloc(FUNC(ncr9010_state::flex_step), this);

	std::fill(std::begin(m_vram), std::end(m_vram), 0x20);
	std::fill(std::begin(m_vattr), std::end(m_vattr), 0x00);

	save_pointer(NAME(m_ram), 0x40000);
	save_item(NAME(m_map));
	save_item(NAME(m_memctrl));
	save_item(NAME(m_map_arm));
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
COMP( 1981, ncr9010, 0,      0,      ncr9010, ncr9010, ncr9010_state, empty_init, "NCR",   "NCR 9010",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND | MACHINE_IMPERFECT_GRAPHICS )
