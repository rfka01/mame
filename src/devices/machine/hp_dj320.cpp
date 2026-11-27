// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

HP DeskJet 320 Inkjet Printer, 1994
Hardware Info by Guru
---------------------

HP Product Number C2634A
The HP Deskjet 320 is a portable inkjet printer.
600 by 300 dpi with REt (Resolution Enhancement technology)
Expandable using special slot and thin half-length PCMCIA-like cards (but not PCMCIA-compatible)

Ink Carts:
 - Black printing using cartridge HP 33 (51633M)
 OR
 - Color printing using HP 25 (51625A) tri-color (CMY) cartridge, black is produced by mixing all 3 colors

PCB#: C2621-80089 Rev B
Sticker: C2634-60021  3445
         B-3434  SINGAPORE

PCB Parts
---------

HP All-In-One Custom CPU/Print Engine 1RH3-0001 94376 Korea 9427014 (QFP100)
 - Note this all-in-1 chip is likely to be very similar or the same as used in some other printers such as Deskjet 300 and 400 series.
 - CPU Core is Z80-based. Possibly an off-the-shelf ROM-less Z80-code-compatible microcontroller.
 - Possible manufacturer Toshiba as this is essentially a TMPZ84C015-class device.
27C040 4M-bit EPROM labelled 'C2634-80052 REV A' at U8 (Program Code)
SOP32 4M-bit mask ROM MB834000 marked 'C2621-80086' at U9 (Font Data)
16MHz Crystal (Z80 core likely runs at 8MHz or 16MHz)
Oki M548128-10 128K PSRAM
NEC A1603C Quad N-Channel Power Mosfet Array
93C46 EEPROM at U25 on bottom side of PCB
ST 7705 Reset IC
2x LM339 Quad Open-Collector Comparator
58-0134 (M) CTAP435 (SOIC16, M is Motorola symbol), possibly logic signals from MCU go here to control L6202/motor
74HC273 8-bit D Flip-Flop
2x 74HCT240 Octal 3-State Buffers/Line Driver
74HC574 8-bit D Transparent Latch with 3-State Outputs
8x Texas Instruments 49A6EHT 210895 (SOIC16) tied to 2x 16 pin connectors that would join to the print head (logic-level gate drivers)
ST L6202 DMOS Full-Bridge Motor Driver
6x Push Button
9x LED
7x Connectors
36 pin Centronics Parallel Port
Custom Slot for additional font carts
 - Dumped cart details:
                       PCB#: 90TH512KP1:0
                             SMART (C) 1995

                       Small PCB with 1x Intel E28F020-150 Flash TSOP32 at U1
                       Location for another TSOP32 at U2 (not populated)

                       Label:
                       X552 Font Cartridge For Deskjet Portable
                       Courier  12
                       CG Times 12
                       Contains fonts in PC Cyrillic, ISO-8859/5
                       and Windows Latin Cyr. Symbolsets
                       5181-7100; B2589A Opt. ACB
                       (C) Hewlett Packard 1993

**********************************************************************

    Emulated: the Z80 core with the memory banking, the timers, the host
    port, the carriage (DC motor and encoder), the service station and paper
    steppers, the EEPROM and the print engine for printing with the internal
    fonts, the raster graphics (75 to 600 dpi, black and CMY with the colour
    cartridge), draft quality and the font card slot (bus/hp_djp_card).

    REt (the manual: 600 by 300 addressable dots per inch for black, curve
    smoothing, DeskJet 320 and 340 only): the internal and card fonts hold
    glyphs on the 600 dpi grid (no nozzle fires on two neighbouring
    columns).  Host raster data goes into the print buffer unchanged; at
    ESC * t 600 R the pass gets E0h instead of E1h and each pixel takes one
    600 dpi column (same byte layout).  No smoothing of raster data was
    found in the firmware.

    Cartridges: the door label lists the 51633A and 51608A black and the
    51625A colour cartridges.  The cartridge answers four ID nozzles; the
    firmware's ID table knows 0 (0A, type 3), 1 (BE, type 0), 2 (BD, type
    1), 9 (0H, type 0) and 12 (0F, type 0).  A job for black (internal
    command 66h) wants type 0, a colour job (67h) type 1; ID 2 prints the
    colour planes, so it is the 51625A.  Which black cartridge gives which
    of the other IDs is not known.  After a change of the type the firmware
    asks for the change cartridge procedure (Setup).  The black IDs print
    with the same command lists; only port 17h differs (written at 0xFDF4,
    value from the pass descriptor): 31h for ID 1 and the colour cartridge,
    45h for IDs 9 and 12, 28h in draft for all of them.  Probably the firing
    pulse width or energy (not verified, not emulated).

    Memory map, from the firmware:
    0000-7FFF  program ROM, first 32K
    8000-BFFF  window: ports 51h (bits 0-3) and 41h (bits 0-3) select a 64K
               block, port 41h bits 4-5 the 16K page in it
               blocks 00-07 program ROM, 08-09 PSRAM, 18-1F font card,
               40-47 font ROM
    C000-FFFF  RAM

    Interrupts (IM 1): status at port 09h, mask in port 08h.
      bit 6  host data received (written into RAM by the chip)
      bit 5  timer compare (ports 02h/03h)
      bit 1  system tick (counter overflow); acknowledged with port 08h bit 4,
             or by writing port 02h (the only acknowledge in the charge mode)

    Power: on battery the chip powers up when Power is pressed.  With the AC
    adapter plugged in the firmware starts in charge mode (printer off) and is
    switched on with Power; the emulation presses it after power on.

    Control panel: port 60h writes drive eight lights, active low: bit 0
    Menu, 1 Change Cartridge, 2 Draft, 3 Busy, 4 Landscape, 5 Compressed,
    6 Online, 7 Power; port 61h bit 6 is the Charge light.  The firmware's
    error light table at 0x576D agrees (code 9 carriage: Draft, code 10
    service station: Change Cartridge, battery codes: Charge and Power);
    Menu for bit 0 is inferred.  The keys are read in five columns: the
    column strobe is written to port 60h, port 60h bit 7 reads the key (low:
    pressed).

    Menu mode: the settings are chosen on a printed menu card on the frame
    behind the access door; the carriage is the pointer.  Groups on the card,
    from right to left (firmware table at 0x1C636, nine bytes per group, the
    last byte is the number of settings), settings with the value the self
    test prints, 0 first:

      BATTERY CONTROL   Quick Charge, Discharge (an action, not stored; order
                        not verified)
      MEDIA SOURCE      Sheet Feeder, Manual Feed
      MEDIA SIZE        Letter, A4, Legal, Executive
      CR DEF            CR=CR, CR=CR+LF
      PERF SKIP         On, Off
      TEXT SCALE        Off, On
      CHARACTER SET     0 PC-8 USA, 1 HP Roman8, 2 PC-8 DN, 3 UK, 4 Germany,
                        5 France, 6 Italy, 7 Norway 1, 8 Sweden, 9 Spain,
                        10 ASCII, 11 Portugal, 12 PC-850, 13 ECMA-94,
                        14 Legal, 15 Card Default

    The keys in Menu mode (key table at 0x1DF6F, also on a label inside the
    door): Online = Group (the first press moves the pointer to the first
    group), Font = pointer left, Draft = pointer right, LF/FF = Select (saved
    to the EEPROM, the carriage moves a little to and fro), Setup = Exit;
    Power keeps its function.  The pointer steps 174 encoder counts per
    setting and 120 more between groups.  Without a key press the firmware
    leaves Menu mode after about 30 seconds.  The self test (Font held at
    power on) prints the stored settings.

    The door label gives Change Cartridge = Setup pressed and Menu = Setup
    pressed and held.  The firmware (0x58F1) reports Font, Draft, Online and
    key 5 at once; Setup and LF/FF start a 30 tick timer (0xE388, about 0.8
    s): released before it runs out is a press, else a hold.  Presses and
    holds use separate key tables (selectors 0x1DC82 and 0x1DD42), Setup
    held calls 0x8029 (Menu).  TEXT SCALE On is EEPROM word 0 bit 14 and
    sets the line spacing to 94 percent (1/6 inch lines 94/600 inch apart).

**********************************************************************/

#include "emu.h"
#include "hp_dj320.h"

#include "hpdj320.lh"

#define LOG_PORT (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(HP_DJ320, hp_dj320_device, "hpdj320", "HP DeskJet 320")


ROM_START( hpdj320 )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD("c2634-80052.u8", 0x00000, 0x80000, CRC(e99a27d1) SHA1(d0ed7b439634656b246c6ebed1ed378ad35d36ae))

	ROM_REGION(0x80000, "fonts", 0)
	ROM_LOAD("c2621-80086.u9", 0x00000, 0x80000, CRC(2d4c96de) SHA1(ac15fd23bc7f010979c47ed7131586f497beac9b))

	ROM_REGION16_BE(0x80, "eeprom", 0)
	ROM_LOAD("93c46.u25", 0x00, 0x80, CRC(61ee3e3a) SHA1(83cbfee3dbc69aa1e7d477d957b94c54920ee512))
ROM_END

const tiny_rom_entry *hp_dj320_device::device_rom_region() const
{
	return ROM_NAME( hpdj320 );
}


static INPUT_PORTS_START( hpdj320 )
	PORT_START("KEYS")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Power")
	// the other keys in the order of the scan columns (port 60h bits 0-4), found
	// with the power on key combinations of the service manual
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Setup")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("LF/FF")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Draft")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Font")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Online")

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x00, "Power source" )
	PORT_CONFSETTING(    0x00, "AC adapter" )
	PORT_CONFSETTING(    0x01, "Battery" )
	// the four bit ID the cartridge returns (firmware table at 0xF320: name, type);
	// after a change of the type the firmware wants the change cartridge procedure
	PORT_CONFNAME( 0xf0, 0x10, "Cartridge ID" )
	PORT_CONFSETTING(    0x10, "1 (BE, type 0)" )
	PORT_CONFSETTING(    0x20, "2 (BD, type 1)" )
	PORT_CONFSETTING(    0x00, "0 (0A, type 3)" )
	PORT_CONFSETTING(    0x90, "9 (0H, type 0)" )
	PORT_CONFSETTING(    0xc0, "12 (0F, type 0)" )
	PORT_CONFSETTING(    0xf0, "15 (not in the table)" )
INPUT_PORTS_END

ioport_constructor hp_dj320_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( hpdj320 );
}


hp_dj320_device::hp_dj320_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_DJ320, tag, owner, clock),
	m_maincpu(*this, "maincpu"),
	m_eeprom(*this, "eeprom"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_card(*this, "card"),
	m_rom(*this, "maincpu"),
	m_fonts(*this, "fonts"),
	m_keys(*this, "KEYS"),
	m_config(*this, "CONFIG"),
	m_leds(*this, "hpdj320_led%u", 0U),
	m_busy_cb(*this),
	m_ack_cb(*this),
	m_perror_cb(*this),
	m_fault_cb(*this),
	m_select_cb(*this),
	m_irq_status(0),
	m_compare(0),
	m_data(0),
	m_strobe(1)
{
}


void hp_dj320_device::device_add_mconfig(machine_config &config)
{
	Z80(config, m_maincpu, XTAL(16'000'000) / 2);   // placeholder
	m_maincpu->set_addrmap(AS_PROGRAM, &hp_dj320_device::mem_map);
	m_maincpu->set_addrmap(AS_IO, &hp_dj320_device::io_map);

	EEPROM_93C46_16BIT(config, m_eeprom);

	config.set_default_layout(layout_hpdj320);

	HP_DJP_CARD_SLOT(config, m_card, hp_djp_cards, nullptr);

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	printer_dot_style dots = printer_dot_style::inkjet(0.0050);
	dots.view_diameter = 0.9 / VDPI;
	m_bitmap_printer->set_dot_style(dots);
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0c, bitmap_printer_device::PAPER_LETTER);   // A4 and Letter sheets
	m_bitmap_printer->set_sheet_top(0.0);
}


void hp_dj320_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0xbfff).rw(FUNC(hp_dj320_device::window_r), FUNC(hp_dj320_device::window_w));
	map(0xc000, 0xffff).lrw8(
			NAME([this] (offs_t offset) { return m_ram[offset]; }),
			NAME([this] (offs_t offset, u8 data) { m_ram[offset] = data; }));
}

void hp_dj320_device::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0xff).rw(FUNC(hp_dj320_device::port_r), FUNC(hp_dj320_device::port_w));
}


void hp_dj320_device::device_start()
{
	m_psram = std::make_unique<u8[]>(0x20000);
	m_ram = std::make_unique<u8[]>(0x4000);
	std::fill_n(m_port, std::size(m_port), 0);

	m_timer = timer_alloc(FUNC(hp_dj320_device::timer_match), this);
	m_tick_timer = timer_alloc(FUNC(hp_dj320_device::tick), this);
	m_host_timer = timer_alloc(FUNC(hp_dj320_device::host_idle), this);
	m_cr_timer = timer_alloc(FUNC(hp_dj320_device::cr_tick), this);
	m_nmi_timer = timer_alloc(FUNC(hp_dj320_device::servo_tick), this);
	m_cr_pos = CR_PARK;
	m_cr_vel = 0.0;
	m_cr_offset = 0;
	m_cr_time = attotime::zero;
	m_nmi_flag = false;
	m_ss_phase = -1;
	m_pf_phase = -1;
	m_sheet = false;
	m_sheet_lead = 0;
	m_sheet_len = 3300;
	m_sheet_inked = false;
	m_sheet_time = attotime::zero;
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_sheet));
	save_item(NAME(m_sheet_len));
	save_item(NAME(m_sheet_inked));
	save_item(NAME(m_sheet_time));
	save_item(NAME(m_sheet_lead));
	save_item(NAME(m_engine_block));
	save_item(NAME(m_pen_sense));
	save_item(NAME(m_engine_mode));
	m_ss_pos = SS_STEPS / 2;
	save_item(NAME(m_ss_phase));
	save_item(NAME(m_ss_pos));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_vel));
	save_item(NAME(m_cr_offset));
	save_item(NAME(m_cr_time));
	save_item(NAME(m_nmi_flag));
	m_ack_timer = timer_alloc(FUNC(hp_dj320_device::ack_end), this);

	save_pointer(NAME(m_psram), 0x20000);
	save_pointer(NAME(m_ram), 0x4000);
	save_item(NAME(m_port));
	save_item(NAME(m_irq_status));
	save_item(NAME(m_compare));
	save_item(NAME(m_counter_base));
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_pending));
}

void hp_dj320_device::device_reset()
{
	std::fill_n(m_port, std::size(m_port), 0);
	m_irq_status = 0;
	m_engine_block = 0;
	m_pen_sense = false;
	m_pending = false;
	m_engine_mode = 0;
	m_counter_base = machine().time();
	m_compare = 0;
	update_timer();
	m_tick_timer->adjust(attotime::from_ticks(0x10000, TIMER_CLOCK), 0, attotime::from_ticks(0x10000, TIMER_CLOCK));
	update_irq();

	m_host_timer->adjust(attotime::never);
	m_cr_time = machine().time();
	m_cr_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(100));
	// servo interrupt (NMI): placeholder rate, the counter divided by 8192 fits the move speeds
	m_nmi_timer->adjust(attotime::from_ticks(8192, TIMER_CLOCK), 0, attotime::from_ticks(8192, TIMER_CLOCK));
	m_busy_cb(1);
	m_ack_cb(1);
	m_select_cb(1);
	m_fault_cb(1);
	m_perror_cb(0);
}


u8 *hp_dj320_device::window_ptr(offs_t offset, bool write)
{
	// 64K block number from port 51h (high) and 41h (low), 16K page from port 41h bits 4-5
	unsigned const block = ((m_port[0x51] & 0x0f) << 4) | (m_port[0x41] & 0x0f);
	offs_t const addr = (offs_t(block & 7) << 16) | (offs_t((m_port[0x41] >> 4) & 3) << 14) | offset;

	if (block < 0x08)
		return write ? nullptr : &m_rom[addr];
	if (block < 0x0a)
		return &m_psram[addr & 0x1ffff];
	if ((block >= 0x40) && (block < 0x48))
		return write ? nullptr : &m_fonts[addr];
	// font card: blocks 18-1F (the firmware also looks at 48-7F)
	if ((block >= 0x18) && (block < 0x20))
	{
		u8 const *const p = m_card->page(((block - 0x18) << 2) | ((m_port[0x41] >> 4) & 3));
		return (p && !write) ? const_cast<u8 *>(&p[offset]) : nullptr;
	}
	return nullptr;
}

u8 hp_dj320_device::window_r(offs_t offset)
{
	u8 const *const p = window_ptr(offset, false);
	return p ? *p : 0xff;
}

void hp_dj320_device::window_w(offs_t offset, u8 data)
{
	u8 *const p = window_ptr(offset, true);
	if (p)
		*p = data;
}


u16 hp_dj320_device::counter() const
{
	return u16((machine().time() - m_counter_base).as_ticks(TIMER_CLOCK));
}

void hp_dj320_device::update_timer()
{
	u16 const delta = m_compare - counter();
	m_timer->adjust(attotime::from_ticks(delta ? delta : 0x10000, TIMER_CLOCK));
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::timer_match)
{
	m_irq_status |= 0x20;
	update_irq();
	m_timer->adjust(attotime::from_ticks(0x10000, TIMER_CLOCK));
}

void hp_dj320_device::paper_step(int dir)
{
	if (!m_sheet)
		return;

	m_sheet_lead += dir;
	if (m_sheet_inked)
	{
		// the paper view follows the lowest nozzle row; the page is filed when its
		// trailing edge has passed the nozzles
		m_bitmap_printer->m_ypos = std::max(sheet_row() + 50, 0);
		if (m_bitmap_printer->check_new_page())
			m_sheet_inked = false;
	}
	if (m_sheet_lead - m_sheet_len > PAPER_OUT)
	{
		LOG("sheet out\n");
		if (m_sheet_inked)
		{
			// file the page now
			m_bitmap_printer->m_ypos = m_sheet_len + 1000;
			m_bitmap_printer->check_new_page();
		}
		m_sheet = false;
		m_sheet_inked = false;
		m_sheet_time = machine().time();
	}
}

bool hp_dj320_device::paper_sensor() const
{
	return m_sheet && (m_sheet_lead >= 0) && (m_sheet_lead - m_sheet_len < 0);
}


//**************************************************************************
//  Print engine
//**************************************************************************

// Port 07h:06h is the encoder count where a pass starts, port 1Ch:1Dh points to
// the pass's command list and port 0Ah bit 7 gives the direction (set: towards
// lower counts).  The engine fires a column every second encoder count.  The
// command list (built by the firmware at 0x14000) holds:
//   000ggggg gggggggg wwwwwwww hhhh dddd  glyph: w columns of font data, header
//                                         bytes at h, dot bytes at d; the gap g
//                                         counts from the end of the glyph
//                                         before the previous one
//   010ggggg gggggggg 00 00 00 nn nn      blank
//   001xxxxx m nnnn g aaaa f l            bitmap: m = 1 spits into the service
//                                         station, m = 2 prints raster data
//                                         (see below)
//   1100bbbb, 1101bbbb                    font memory block, low and high nibble
//   100mmmmm                              glyph mode: 2 prints only the columns
//                                         whose header bit 7 is set (compressed
//                                         pitch), w then counts those columns
//   111xxvvr                              vv + 1 = nozzles per raster row (1 at
//                                         300 dpi, 2, 3, 4 at 75 dpi), r = 0 for
//                                         600 dpi raster (one column per pixel)
//   101xxxxx                              not known (no visible effect yet)
// Every list sets the block first; the glyph mode starts at 0 in each list.
//   011xxxxx                              end of list
// Each column has a header byte: bits 0-6 tell which of seven bytes (8 rows each,
// MSB on top) follow in the dot data, bit 7 marks the columns kept in compressed
// pitch.  A pass towards lower counts gets the
// address of the last header byte and of the last dot byte and reads backwards.
//
// Raster data (mode 2) lies in groups of eight pixels: for each group one byte
// per raster row (rows of (l-f)/(vv+1) nozzles), MSB on the left.  n is the number of columns
// fired, g the columns per group (16: a 300 dpi pixel takes two 600 dpi columns,
// fired once),
// a the address in the font memory block.  A pass towards lower counts gets the
// address of the last byte and reads backwards.

u8 hp_dj320_device::engine_byte(unsigned block, u32 offset) const
{
	u32 const addr = (u32(block) << 16) + offset;
	block = addr >> 16;
	if (block < 0x08)
		return m_rom[addr & 0x7ffff];
	if (block < 0x0a)
		return m_psram[addr & 0x1ffff];
	if ((block >= 0x40) && (block < 0x48))
		return m_fonts[addr & 0x7ffff];
	if ((block >= 0x18) && (block < 0x20))
	{
		u8 const *const p = m_card->page(((block - 0x18) << 2) | ((addr >> 14) & 3));
		return p ? p[addr & 0x3fff] : 0xff;
	}
	return 0x00;
}

void hp_dj320_device::plot(int x, int y, u32 ink)
{
	if ((x < 0) || (x >= int(PAPER_WIDTH * HDPI)) || (y < 0) || (y >= m_sheet_len))
		return;
	// inks mix subtractively
	m_bitmap_printer->pix(y, x) &= ink;
}

// Cartridge identification: the firmware fires the nozzles 46 to 49 one at a
// time with a spit pattern (one byte per nozzle at D072h, bit 7 fires) and
// reads after each firing whether the cartridge answered on port 0Eh bit 7;
// reading port 0Fh clears that latch.  Nozzle 46 gives the most significant
// bit of the four bit ID (firmware at 0xEC0E, ID table at 0xF320).
void hp_dj320_device::sense_pen(u16 ptr)
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u8 const id = (m_config->read() >> 4) & 0x0f;
	for (int n = 0; n < 0x40; n++)
	{
		u8 const b = space.read_byte(ptr);
		u8 const op = b >> 5;
		if (op == 3)
			break;
		if (op == 1)
		{
			u8 const mode = space.read_byte(ptr + 1);
			u16 const addr = space.read_byte(ptr + 5) | (space.read_byte(ptr + 6) << 8);
			if (mode == 1)
				for (int k = 0; k < 4; k++)
					if (BIT(space.read_byte(addr + 46 + k), 7) && BIT(id, 3 - k))
						m_pen_sense = true;
			ptr += 9;
		}
		else
			ptr += ((op == 0) || (op == 2)) ? 7 : 1;
	}
}

void hp_dj320_device::print_pass()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u16 const start = (m_port[0x07] << 8) | m_port[0x06];
	u16 ptr = (m_port[0x1c] << 8) | m_port[0x1d];
	bool const down = BIT(m_port[0x0a], 7);

	sense_pen(ptr);
	if (!m_sheet)
		return;

	LOG("pass at %d %s, list %04x, paper row %d\n", start, down ? "down" : "up", ptr, sheet_row());
	int const x0 = (int(start) + (down ? DOWN_SHIFT : 0) - PAPER_LEFT) / 2 + SHEET_LEFT;
	if (!m_sheet_inked)
	{
		// the first ink on a sheet: the paper view follows it from now on
		m_bitmap_printer->m_ypos = std::max(sheet_row() + 50, 0);
		m_bitmap_printer->check_new_page();
		m_bitmap_printer->m_ypos = std::max(sheet_row() + 50, 0);
	}
	int const y0 = sheet_row();
	int end[2] = { -1, -1 };
	m_engine_mode = 0;   // every list starts in the normal glyph mode
	m_engine_rows = 1;
	m_engine_600 = false;
	// port 00h bit 2 is set on every pass towards lower counts (F0h/F1h up, F4h
	// down); bit 1 is set for draft quality (150 by 300 dpi, Draft key or
	// ESC ( s 1 Q; ESC * o -1 M is ignored), with
	// the same command lists and port 17h at 28h instead of 31h.  In draft each
	// nozzle fires on every fourth column instead of every second; the glyphs
	// keep their alternating columns, so every second dot of a row stays out
	// (assumed)
	bool const draft = BIT(m_port[0x00], 1);
	for (int n = 0; n < 0x200; n++)
	{
		u8 const b = space.read_byte(ptr);
		u8 const op = b >> 5;
		if ((op == 0) || (op == 2))
		{
			int const gap = ((b & 0x1f) << 8) | space.read_byte(ptr + 1);
			int const x = end[0] + gap + 1;
			int w = 0;
			if (op == 0)
			{
				w = space.read_byte(ptr + 2);
				u16 const hdr = space.read_word(ptr + 3);
				u16 const dat = space.read_word(ptr + 5);
				bool const compressed = (m_engine_mode == 2);
				int dp = 0;
				for (int c = 0, n = 0; (c < w) && (n < 0x400); n++)
				{
					u8 const h = engine_byte(m_engine_block, down ? (hdr - n) : (hdr + n));
					bool const fire = !compressed || BIT(h, 7);
					int const px = down ? (x0 - x - c) : (x0 + x + c);
					bool const skip = draft && (((x + c) % 4) >= 2);
					for (int i = 0; i < 7; i++)
					{
						int const lane = down ? (6 - i) : i;
						if (BIT(h, lane))
						{
							u8 const v = engine_byte(m_engine_block, down ? (dat - dp) : (dat + dp));
							dp++;
							if (fire && !skip)
								for (int bit = 0; bit < 8; bit++)
									if (BIT(v, 7 - bit))
									{
										plot(px, y0 + lane * 8 + bit);
										m_sheet_inked = true;
									}
						}
					}
					if (fire)
						c++;
				}
			}
			end[0] = end[1];
			end[1] = x + w - 1;
			ptr += 7;
		}
		else if (op == 3)
		{
			break;
		}
		else if (op == 1)
		{
			// bitmap; spitting into the service station leaves no ink on the paper
			u8 const mode = space.read_byte(ptr + 1);
			int const cols = space.read_byte(ptr + 2) | (space.read_byte(ptr + 3) << 8);
			int const group = space.read_byte(ptr + 4);
			u16 const addr = space.read_byte(ptr + 5) | (space.read_byte(ptr + 6) << 8);
			int const first = space.read_byte(ptr + 7);
			int const last = space.read_byte(ptr + 8);
			int const rows = last - first;
			int const vrep = m_engine_rows;
			if ((mode == 2) && (group >= 8) && (rows >= vrep))
			{
				// E0h (600 dpi, ESC * t 600 R): one column per pixel, the
				// bytes keep their layout (one byte per row for 8 pixels)
				int const per_pixel = m_engine_600 ? 1 : (group / 8);
				// the colour cartridge (ID 2) has three groups of 16 nozzles, 20
				// rows apart: yellow, magenta, cyan from the top; the magenta
				// nozzles sit 36 columns (1/600 inch) to the left of the others
				// (both from the offsets the firmware gives the colour planes)
				// the odd nozzles of each group sit in a second column, the firmware
				// shifts their data by COLOUR_STAGGER columns
				bool const colour = (((m_config->read() >> 4) & 0x0f) == 2);
				static u32 const inks[3] = { 0xffff00, 0xff00ff, 0x00ffff };
				for (int c = 0; c < cols; c++)
				{
					// at 300 dpi and below a pixel fires once (on every second
					// 600 dpi column), in draft on every fourth column (assumed)
					if (!m_engine_600 && ((c % (draft ? 4 : 2)) != 0))
						continue;
					int const g = c / (8 * per_pixel);
					int const k = (c % (8 * per_pixel)) / per_pixel;
					int const px = down ? (x0 - 1 - c) : (x0 + 1 + c);
					int const lines = rows / vrep;
					for (int r = 0; r < lines; r++)
					{
						int const j = g * lines + r;
						u8 const v = engine_byte(m_engine_block, down ? (addr - j) : (addr + j));
						if (BIT(v, down ? k : (7 - k)))
						{
							for (int n = 0; n < vrep; n++)
							{
								int const row = first + (down ? (lines - 1 - r) : r) * vrep + n;
								if (colour && (row < 48))
									plot(px - ((row / 16) == 1 ? 36 : 0) - (BIT(row, 0) ? COLOUR_STAGGER : 0), y0 + (row / 16) * 20 + (row % 16), inks[row / 16]);
								else
									plot(px, y0 + row);
							}
							m_sheet_inked = true;
						}
					}
				}
			}
			ptr += 9;
		}
		else
		{
			if ((b & 0xf0) == 0xc0)
				m_engine_block = (m_engine_block & 0xf0) | (b & 0x0f);
			else if ((b & 0xf0) == 0xd0)
				m_engine_block = (m_engine_block & 0x0f) | ((b & 0x0f) << 4);
			else if ((b & 0xe0) == 0x80)
				m_engine_mode = b & 0x1f;
			else if ((b & 0xe0) == 0xe0)
			{
				m_engine_rows = ((b >> 1) & 3) + 1;
				m_engine_600 = !BIT(b, 0);
			}
			ptr++;
		}
	}
	if (m_sheet_inked)
		m_bitmap_printer->m_ypos = std::max(sheet_row() + 50, 0);
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::cr_tick)
{
	update_carriage();
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::servo_tick)
{
	// runs while the carriage servo is enabled (port 00h bit 6, placeholder)
	if (!BIT(m_port[0x00], 6))
		return;
	m_nmi_flag = true;
	m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::zero);
}

void hp_dj320_device::update_carriage()
{
	attotime const now = machine().time();
	double dt = (now - m_cr_time).as_double();
	m_cr_time = now;
	u8 const pwm = m_port[0x15];
	double duty = ((pwm >> 4) * 13 + (pwm & 0x0f)) / 208.0;
	if (!BIT(m_port[0x00], 6))
		duty = 0.0;
	double const drive = BIT(m_port[0x14], 7) ? -duty : duty;
	while (dt > 0.0)
	{
		double const h = std::min(dt, 0.0002);
		dt -= h;
		double target = drive * CR_VMAX;
		if (m_cr_pos < 0.0)
			target -= m_cr_pos * (0.75 * CR_VMAX / CR_SPRING_HOLD);   // spring pushes back
		m_cr_vel += (target - m_cr_vel) * h / CR_TAU;
		m_cr_pos += m_cr_vel * h;
		if (m_cr_pos < -CR_SPRING)
		{
			m_cr_pos = -CR_SPRING;
			m_cr_vel = 0.0;
		}
		if (m_cr_pos > CR_TRAVEL)
		{
			m_cr_pos = CR_TRAVEL;
			m_cr_vel = 0.0;
		}
	}
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::tick)
{
	// counter overflow
	m_irq_status |= 0x02;
	update_irq();
}

void hp_dj320_device::update_irq()
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (m_irq_status & m_port[0x08] & 0xef) ? ASSERT_LINE : CLEAR_LINE);
}


u8 hp_dj320_device::port_r(offs_t offset)
{
	u8 data = m_port[offset];

	switch (offset)
	{
	case 0x09:
		data = m_irq_status;
		break;

	case 0x0c: case 0x0d:
		// free running counter
		data = u8(counter() >> ((offset - 0x0c) * 8));
		break;

	case 0x0f:
		// clears the cartridge answer latch
		if (!machine().side_effects_disabled())
			m_pen_sense = false;
		break;

	case 0x0e:
		// bit 7: the cartridge answered a firing (see sense_pen)
		data = (data & 0x7f) | (m_pen_sense ? 0x80 : 0x00);
		// bit 3: paper sensor
		{
			if (!m_sheet && m_bitmap_printer->sheet_in() && ((machine().time() - m_sheet_time) > attotime::from_seconds(2)))
			{
				// a sheet pushed into the slot up to the rollers (placeholder: two seconds after the last one left)
				m_sheet = true;
				m_sheet_lead = PAPER_INSERT;
				m_sheet_len = ((m_bitmap_printer->ioport("PAPER")->read() & 0x07) == 3) ? 3508 : 3300;
				m_sheet_inked = false;
			}
			data = (data & ~0x08) | (paper_sensor() ? 0x08 : 0x00);
		}
		break;

	case 0x04: case 0x05:
		data = u8(encoder() >> ((offset - 4) * 8));
		break;

	case 0x49:
		// bit 0: servo interrupt pending
		data = m_nmi_flag ? 0x01 : 0x00;
		if (!machine().side_effects_disabled())
			m_nmi_flag = false;
		break;

	case 0x61:
		// inputs: bit 7 power good (placeholder: always on AC power), bit 0 service station index
		data = 0x80 | ((m_ss_pos < SS_SENSOR) ? 0x01 : 0x00);
		break;

	case 0x60:
	{
		// bit 0: Power key; the chip powers up when it is pressed, and the
		// firmware waits for its release.  The emulation presses it at power on
		// (on AC power a second time to leave the charge mode).
		u8 const keys = m_keys->read();
		bool const battery = BIT(m_config->read(), 0);
		attotime const now = machine().time();
		data = 0x00;
		if (BIT(keys, 0) || (now < attotime::from_msec(300)) ||
				(!battery && (now > attotime::from_msec(4000)) && (now < attotime::from_msec(4500))))
			data |= 0x01;
		// bits 1-2: battery comparators (low: charged)
		// bits 3-4: power source (both high: battery)
		if (battery)
			data |= 0x18;
		// bit 5: EEPROM data out, inverted
		if (!m_eeprom->do_read())
			data |= 0x20;
		// bit 6: unknown (high)
		data |= 0x40;
		// bit 7: the key in the column selected by the last write (low: pressed)
		for (int i = 0; i < 5; i++)
			if ((m_port[0x60] == (1 << i)) && BIT(keys, i + 1))
				data |= 0x80;
		data ^= 0x80;
		break;
	}
	}

	if (!machine().side_effects_disabled() && (offset != 0x41) && (offset != 0x51) && (offset != 0x60) && (offset != 0x08) && (offset != 0x09) && (offset != 0x04) && (offset != 0x05) && (offset != 0x49))
		LOGMASKED(LOG_PORT, "%s: port %02x read %02x\n", machine().describe_context(), offset, data);
	return data;
}

void hp_dj320_device::port_w(offs_t offset, u8 data)
{
	if ((offset != 0x0e) && (offset != 0x0f) && (offset != 0x41) && (offset != 0x51) && (offset != 0x61) && (offset != 0x08) && (offset != 0x60) && (offset != 0x02) && (offset != 0x03) && (offset != 0x14) && (offset != 0x15))
		LOGMASKED(LOG_PORT, "%s: port %02x write %02x\n", machine().describe_context(), offset, data);
	if ((offset == 0x00) || (offset == 0x14) || (offset == 0x15))
		update_carriage();
	m_port[offset] = data;

	switch (offset)
	{
	case 0x07:
		// the last register of a pass set up
		print_pass();
		break;

	case 0x4b:
	{
		// service station stepper on the high nibble (half step sequence 6 4 C 8 9 1 3 2)
		static constexpr u8 SEQ[8] = { 0x6, 0x4, 0xc, 0x8, 0x9, 0x1, 0x3, 0x2 };
		int phase = -1;
		for (int i = 0; i < 8; i++)
			if ((data >> 4) == SEQ[i])
				phase = i;
		if ((phase >= 0) && (m_ss_phase >= 0))
		{
			int const d = (phase - m_ss_phase) & 7;
			if (d == 1)
				m_ss_pos = (m_ss_pos + 1) % SS_STEPS;
			else if (d == 7)
				m_ss_pos = (m_ss_pos + SS_STEPS - 1) % SS_STEPS;
		}
		if (phase >= 0)
			m_ss_phase = phase;

		// paper feed stepper on the low nibble (full step sequence 9 C 6 3)
		static constexpr u8 PSEQ[4] = { 0x9, 0xc, 0x6, 0x3 };
		int pphase = -1;
		for (int i = 0; i < 4; i++)
			if ((data & 0x0f) == PSEQ[i])
				pphase = i;
		if ((pphase >= 0) && (m_pf_phase >= 0))
		{
			int const d = (pphase - m_pf_phase) & 3;
			if (d == 1)
				paper_step(1);
			else if (d == 3)
				paper_step(-1);
		}
		if (pphase >= 0)
			m_pf_phase = pphase;
		break;
	}

	case 0x04: case 0x05:
		// the encoder counter is preset
		update_carriage();
		m_cr_offset = s32(m_cr_pos) - ((m_port[0x05] << 8) | m_port[0x04]);
		break;

	case 0x02: case 0x03:
		m_compare = (m_port[0x03] << 8) | m_port[0x02];
		m_irq_status &= ~0x20;
		if (offset == 0x02)
			m_irq_status &= ~0x02;   // the charge mode interrupt handler acknowledges the tick this way
		update_timer();
		update_irq();
		break;

	case 0x60:
		// panel lights, active low (the key scan briefly writes the column
		// strobes 01h-10h in between, see port_r)
		for (int i = 0; i < 8; i++)
			m_leds[i] = BIT(~data, i);
		break;

	case 0x61:
		// bit 6: Charge light
		m_leds[8] = BIT(data, 6);
		// EEPROM: bit 3 chip select, bit 4 clock, bit 5 data in
		m_eeprom->cs_write(BIT(data, 3));
		m_eeprom->di_write(BIT(data, 5));
		m_eeprom->clk_write(BIT(data, 4));
		break;

	case 0x4a:
		// bit 6 enables the host DMA; the interrupt handler clears it, which
		// also clears the interrupt
		if (!BIT(data, 6))
		{
			m_irq_status &= ~0x40;
			m_host_timer->adjust(attotime::never);
			update_irq();
		}
		take_byte();
		update_busy();
		break;

	case 0x5c: case 0x5d: case 0x5e: case 0x5f:
		take_byte();
		update_busy();
		break;

	case 0x08:
		// bit 4 written high acknowledges the pending interrupts
		if (BIT(data, 4))
			m_irq_status &= ~0x02;
		update_irq();
		break;
	}
}


void hp_dj320_device::strobe_w(int state)
{
	// the chip latches the byte on the rising edge of /STROBE and keeps BUSY
	// up until the DMA has stored it
	if (!m_strobe && state)
	{
		m_pending = true;
		take_byte();
	}
	m_strobe = state;
	update_busy();
}

void hp_dj320_device::take_byte()
{
	if (m_pending && host_ready())
	{
		m_pending = false;
		u16 ptr = (m_port[0x5f] << 8) | m_port[0x5e];
		m_maincpu->space(AS_PROGRAM).write_byte(ptr, m_data);
		ptr++;
		m_port[0x5e] = ptr & 0xff;
		m_port[0x5f] = ptr >> 8;
		if (ptr == ((m_port[0x5d] << 8) | m_port[0x5c]))
		{
			// buffer full
			m_host_timer->adjust(attotime::never);
			m_irq_status |= 0x40;
			update_irq();
		}
		else
		{
			// interrupt when the host pauses (placeholder unit for the count in ports 4Ch/4Dh)
			m_host_timer->adjust(attotime::from_usec(((m_port[0x4d] << 8) | m_port[0x4c]) & 0x3fff));
		}
		m_ack_cb(0);
		m_ack_timer->adjust(attotime::from_usec(5));
	}
}

bool hp_dj320_device::host_ready() const
{
	u16 const ptr = (m_port[0x5f] << 8) | m_port[0x5e];
	return BIT(m_port[0x4a], 6) && !BIT(m_irq_status, 6) && (ptr != ((m_port[0x5d] << 8) | m_port[0x5c]));
}

void hp_dj320_device::update_busy()
{
	m_busy_cb((host_ready() && !m_pending) ? 0 : 1);
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::host_idle)
{
	m_irq_status |= 0x40;
	update_irq();
	update_busy();
}

TIMER_CALLBACK_MEMBER(hp_dj320_device::ack_end)
{
	m_ack_cb(1);
}

void hp_dj320_device::init_w(int state)
{
}
