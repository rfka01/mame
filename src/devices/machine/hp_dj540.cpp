// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

HP Deskjet 540, Hewlett Packard, September 1994
Hardware Info By Guru
---------------------

HP Product#: C2162
PCB#: C2162-80151
Sticker: C2162-60157 3512 A-3505-32 S'PORE

Main components are....
- HP 1RM7-0001 QFP160 all-in-1 custom chip
- 4Mbit SOP40 mask ROM. Firmware check reveals the CPU core is 68000-based.
- Space for a SOP44 ROM but not populated
- 24C02 SOIC8 EEPROM. Dump reveals this is a Deskwriter which is the Apple version of the Deskjet series.
- Toshiba TC514260BJ-80 RAM
- Crystal 15CKSS5BT (maybe 15MHz?)
- UC3843 PWM power supply controller
- Motorola XC79185FN PLCC44 (possibly motor / head driver)
- Printer Port
- DC power jack
- Several connectors

---------------------

    HP DeskJet 540 inkjet printer (C2162A), 1994

    The firmware is the DeskJet 540's (device ID MFG:HEWLETT-PACKARD;
    MDL:DESKJET 540;CMD:MLC,PCL,PML, revision 2.39.1.12 of 94/09/13), with
    pSOS 4.1; it writes "DeskWriter" into the EEPROM itself as the default
    AppleTalk name.  Black printing is 600 x 300 dpi, colour 300 x 300 dpi
    with the optional colour cartridge, which takes the place of the black
    one.

    The custom chip as far as the firmware shows (68000 addresses; all of it
    worked out from the firmware, none of it documented):

      000000-07FFFF  ROM; RAM 512 KiB at 780000-7FFFFF, mirrored at 400000
      FFFFEF/ED/F1   interrupt enable, pending (active low), acknowledge
                     (bits written as 0 are cleared); vectors 64 + n: n = 1
                     system tick, 2 stepper timer, 3 encoder compare,
                     4 host interface (FFFFEB & FFFFDF), 5 carriage servo
                     (pending bit 4)
      FFFD06         system tick period, FFFE0F bit 7 runs it (100 Hz)
      FFFD0C/0E      stepper timer reload and start value, FFFE0D bit 7
      FFFD0A         carriage servo period (400 Hz), FFFE09 bit 7
      FFFD02         carriage encoder counter (600 per inch, writable)
      FFFD00         encoder compare: interrupt 3 when the counter reaches
                     it; a print pass runs from one match to the next
      FFFE03/07      carriage motor PWM and direction (1 up, 2 down)
      FFFE0B/11      steppers: paper feed and service station, half steps
                     1 9 8 A 2 6 4 5 forward
      FFF911-917     ports A-D, direction registers FFF919-91F (1 = output):
                     A bits 3/4 SDA/SCL of the 24C02, bits 6/7 pen ID pulses;
                     C bits 0/1 Power and Resume lamps (active low, read high
                     through the lamps), bit 3 Resume (high = pressed), bit 4
                     access door (high = closed), bit 5 paper sensor, bit 6
                     Power (low = pressed); D host lines (bit 1 BUSY)
      FFF900         host data latch; the host DMA stores bytes at FFF53C up
                     to FFF538 while FFFFE9 bit 1 is set, FFFFEB bit 0 = idle
      FFF600-60F     nozzles to fire on a write to FFFB0B (pen ID), the
                     answer is FFFB01 bit 1
      FFF800-81F     16 rows of 16 dots; writing FFFD12 stores them turned
                     into 16 columns of the print buffer at FFF524
      FFF400-47F     glyph engine, started by FFFA08 (see glyph_op)
      FFF500-517     print pass: three pairs of column pointers, FFFB03 bit 0
                     starts the pass at the next compare match, bit 3 runs
                     towards lower counts, bit 6 fires every other count

    Panel: Power and Resume keys and lamps, the access door.  Power held and
    Resume pressed n times: 1 self test (also Resume held while ready),
    2-4 demo pages (not in this ROM), 5 diagnostic test, 7 cartridge
    cleaning.  Resume alone prints a page that waits for a form feed.

    Not emulated: the REt edge smoothing of the glyph engine (dots are
    doubled across), the other host modes (nibble, ECP, MLC), the service
    station's effect on print quality.

**********************************************************************/

#include "emu.h"
#include "hp_dj540.h"

#include "hpdj540.lh"

//#define VERBOSE 1
#include "logmacro.h"


DEFINE_DEVICE_TYPE(HP_DJ540, hp_dj540_device, "hpdj540", "HP DeskJet 540")


ROM_START( hpdj540 )
	ROM_REGION16_BE(0x80000, "maincpu", 0)
	ROM_LOAD16_WORD_SWAP("1818-6057.u14", 0x00000, 0x80000, CRC(d5308d85) SHA1(486fd19f72864492bdf0e38441b9a93f755b72ba))

	ROM_REGION(0x100, "eeprom", 0)
	ROM_LOAD("24c02.u10", 0x000, 0x100, CRC(0b079058) SHA1(744d1baff9c28ecdb9adeb657ca6d623458fe29d))
ROM_END


hp_dj540_device::hp_dj540_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_DJ540, tag, owner, clock),
	m_maincpu(*this, "maincpu"),
	m_eeprom(*this, "eeprom"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_keys(*this, "KEYS"),
	m_config(*this, "CONFIG"),
	m_led(*this, "hpdj540_led%u", 0U),
	m_busy_cb(*this),
	m_ack_cb(*this),
	m_perror_cb(*this),
	m_fault_cb(*this),
	m_select_cb(*this),
	m_data(0),
	m_strobe(1)
{
}

const tiny_rom_entry *hp_dj540_device::device_rom_region() const
{
	return ROM_NAME(hpdj540);
}

void hp_dj540_device::mem_map(address_map &map)
{
	map.global_mask(0xffffff);
	map(0x000000, 0x07ffff).rom().region("maincpu", 0);
	map(0x400000, 0x47ffff).mirror(0x380000).ram();
	map(0xfff400, 0xffffff).rw(FUNC(hp_dj540_device::reg_r), FUNC(hp_dj540_device::reg_w));
}

void hp_dj540_device::cpu_space_map(address_map &map)
{
	map(0xfffff1 + 2 * IRQ_LEVEL, 0xfffff1 + 2 * IRQ_LEVEL).r(FUNC(hp_dj540_device::irq_vector_r));
}

void hp_dj540_device::device_add_mconfig(machine_config &config)
{
	config.set_default_layout(layout_hpdj540);

	M68000(config, m_maincpu, 15'000'000);    // crystal marked 15CKSS5BT (assumed 15 MHz)
	m_maincpu->set_addrmap(AS_PROGRAM, &hp_dj540_device::mem_map);
	m_maincpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &hp_dj540_device::cpu_space_map);

	I2C_24C02(config, m_eeprom);

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	printer_dot_style dots = printer_dot_style::inkjet(0.0050);
	dots.view_diameter = 0.9 / VDPI;
	m_bitmap_printer->set_dot_style(dots);
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0c, bitmap_printer_device::PAPER_LETTER);   // A4 and Letter sheets
	m_bitmap_printer->set_sheet_top(0.0);
}

INPUT_PORTS_START( hpdj540 )
	PORT_START("KEYS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Power")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Resume")

	PORT_START("CONFIG")
	PORT_CONFNAME(0x03, 0x01, "Print cartridge")
	PORT_CONFSETTING(0x00, DEF_STR(None))
	PORT_CONFSETTING(0x01, "Black (HP 51626A)")
	PORT_CONFSETTING(0x02, "Colour (HP 51625A)")
	PORT_CONFNAME(0x04, 0x00, "Paper in tray")
	PORT_CONFSETTING(0x00, DEF_STR(Yes))
	PORT_CONFSETTING(0x04, DEF_STR(No))
	PORT_CONFNAME(0x08, 0x00, "Access door")
	PORT_CONFSETTING(0x00, "Closed")
	PORT_CONFSETTING(0x08, "Open")
INPUT_PORTS_END

ioport_constructor hp_dj540_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( hpdj540 );
}

void hp_dj540_device::device_start()
{
	m_tick_timer = timer_alloc(FUNC(hp_dj540_device::tick), this);
	m_step_timer = timer_alloc(FUNC(hp_dj540_device::step_tick), this);
	m_servo_timer = timer_alloc(FUNC(hp_dj540_device::servo_tick), this);
	m_encoder_timer = timer_alloc(FUNC(hp_dj540_device::encoder_tick), this);
	m_ack_timer = timer_alloc(FUNC(hp_dj540_device::ack_end), this);

	std::fill(std::begin(m_reg), std::end(m_reg), 0);
	m_irq_pending = 0;
	m_host_irq = false;
	m_host_status = 0;
	m_byte_waiting = false;
	m_pen_sense = false;
	m_phase[0] = m_phase[1] = -1;
	m_cr_pos = CR_PARK;
	m_cr_vel = 0.0;
	m_cr_offset = 0;
	m_cr_last = s32(CR_PARK);
	m_cr_time = attotime::zero;
	m_cmp_armed = false;
	m_printing = false;
	m_odd_count = false;
	m_sheet = false;
	m_sheet_lead = 0;
	m_sheet_len = 3300;
	m_paper_step_time = attotime::zero;
	m_widen_last = 0;

	save_item(NAME(m_reg));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_host_irq));
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_host_status));
	save_item(NAME(m_byte_waiting));
	save_item(NAME(m_pen_sense));
	save_item(NAME(m_phase));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_vel));
	save_item(NAME(m_cr_offset));
	save_item(NAME(m_cr_last));
	save_item(NAME(m_cr_time));
	save_item(NAME(m_cmp_armed));
	save_item(NAME(m_printing));
	save_item(NAME(m_odd_count));
	save_item(NAME(m_sheet));
	save_item(NAME(m_sheet_lead));
	save_item(NAME(m_sheet_len));
	save_item(NAME(m_paper_step_time));
	save_item(NAME(m_widen_last));

	// the carriage is followed in steps of 0.1 ms so that the compare
	// interrupts and the columns of a pass come at the right encoder counts
	m_encoder_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(100));
}

void hp_dj540_device::device_reset()
{
	m_irq_pending = 0;
	m_tick_timer->adjust(attotime::never);
	m_step_timer->adjust(attotime::never);
	m_servo_timer->adjust(attotime::never);
	update_irq();

	// status lines other than BUSY are not driven by the firmware model
	m_select_cb(1);
	m_fault_cb(1);
	m_perror_cb(0);
	m_ack_cb(1);
	update_busy();
}


//**************************************************************************
//  Interrupts and timers
//**************************************************************************

void hp_dj540_device::set_irq(int bit, bool state)
{
	if (state)
		m_irq_pending |= 1 << bit;
	else
		m_irq_pending &= ~(1 << bit);
	update_irq();
}

void hp_dj540_device::update_irq()
{
	m_maincpu->set_input_line(IRQ_LEVEL, ((m_irq_pending & m_reg[0xbef]) || m_host_irq) ? ASSERT_LINE : CLEAR_LINE);
}

// vectors: pending bits 1-3 give 64 + bit, the host interface 68, bit 4 (the
// carriage servo) 69, as the firmware installs its handlers
u8 hp_dj540_device::irq_vector_r()
{
	u8 const active = m_irq_pending & m_reg[0xbef];
	for (int bit = 0; bit < 4; bit++)
		if (BIT(active, bit))
			return 64 + bit;
	if (m_host_irq)
		return 68;
	for (int bit = 4; bit < 8; bit++)
		if (BIT(active, bit))
			return 65 + bit;
	return 24;      // spurious
}

TIMER_CALLBACK_MEMBER(hp_dj540_device::tick)
{
	set_irq(1, true);
}

void hp_dj540_device::update_tick()
{
	u16 const period = reg16(0xfffd06);
	if (BIT(m_reg[0xa0f], 7) && period)
	{
		attotime const t = attotime::from_ticks(period, TIMER_CLOCK);
		if (m_tick_timer->expire().is_never())
			m_tick_timer->adjust(t, 0, t);
	}
	else
		m_tick_timer->adjust(attotime::never);
}

// stepper timer: interrupt 2 after FFFD0E counts, then every FFFD0C counts
void hp_dj540_device::update_step_timer()
{
	if (BIT(m_reg[0xa0d], 7))
	{
		if (m_step_timer->expire().is_never())
		{
			u16 const n = reg16(0xfffd0e);
			m_step_timer->adjust(attotime::from_ticks(n ? n : 0x10000, TIMER_CLOCK));
		}
	}
	else
		m_step_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(hp_dj540_device::step_tick)
{
	set_irq(2, true);
	u16 const n = reg16(0xfffd0c);
	m_step_timer->adjust(attotime::from_ticks(n ? n : 0x10000, TIMER_CLOCK));
}

// carriage servo timer: interrupt bit 4 every FFFD0A counts
void hp_dj540_device::update_servo_timer()
{
	if (BIT(m_reg[0xa09], 7))
	{
		if (m_servo_timer->expire().is_never())
		{
			u16 const n = reg16(0xfffd0a);
			m_servo_timer->adjust(attotime::from_ticks(n ? n : 0x10000, TIMER_CLOCK));
		}
	}
	else
		m_servo_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(hp_dj540_device::servo_tick)
{
	update_carriage();
	set_irq(4, true);
	u16 const n = reg16(0xfffd0a);
	m_servo_timer->adjust(attotime::from_ticks(n ? n : 0x10000, TIMER_CLOCK));
}

TIMER_CALLBACK_MEMBER(hp_dj540_device::encoder_tick)
{
	update_carriage();
}


//**************************************************************************
//  Carriage and print passes
//**************************************************************************

// the carriage motor is modelled as a speed that follows the PWM duty (high
// nibble * 13 + low nibble out of 208) and direction; the firmware homes by
// driving into the end stops and stops on the position error
void hp_dj540_device::update_carriage()
{
	attotime const now = machine().time();
	double dt = (now - m_cr_time).as_double();
	m_cr_time = now;
	u8 const pwm = m_reg[0xa03];
	double duty = ((pwm >> 4) * 13 + (pwm & 0x0f)) / 208.0;
	if (!BIT(m_reg[0xa09], 7))
		duty = 0.0;
	double const drive = (m_reg[0xa07] == 1) ? duty : (m_reg[0xa07] == 2) ? -duty : 0.0;
	while (dt > 0.0)
	{
		double const h = std::min(dt, 0.0002);
		dt -= h;
		m_cr_vel += (drive * CR_VMAX - m_cr_vel) * h / CR_TAU;
		m_cr_pos += m_cr_vel * h;
		if (m_cr_pos < 0.0)
		{
			m_cr_pos = 0.0;
			m_cr_vel = 0.0;
		}
		if (m_cr_pos > CR_TRAVEL)
		{
			m_cr_pos = CR_TRAVEL;
			m_cr_vel = 0.0;
		}
		s32 const c = s32(m_cr_pos);
		while (c != m_cr_last)
		{
			m_cr_last += (c > m_cr_last) ? 1 : -1;
			encoder_count(u16(m_cr_last - m_cr_offset));
		}
	}
}

void hp_dj540_device::encoder_count(u16 count)
{
	if (m_printing)
		print_column(count);
	if (m_cmp_armed && (count == reg16(0xfffd00)))
	{
		m_cmp_armed = false;
		set_irq(3, true);
		// the first match of a pass (FFFB03 bits 0-1 set) starts the nozzle data, the next ends it
		if (m_printing)
			m_printing = false;
		else if ((m_reg[0x703] & 0x03) == 0x03)
		{
			m_printing = true;
			m_odd_count = false;
		}
	}
}

// a pass: for each encoder count the head fires a column (600 dpi), in colour
// passes every other count (300 dpi).  Columns are 8 bytes, four words of 16
// nozzles; FFF500-FFF517 are three pairs of column pointers, one for each
// nozzle group, moved on by a column before each column.  The second pointer of
// a pair serves the other nozzle column, 20 columns behind; it also clears the
// printed column: the swath manager only writes the groups that have dots, and
// a buffer used two passes before would otherwise print again (fitted: without
// this, colour pages print a second time 32 rows lower)
void hp_dj540_device::print_column(u16 count)
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	bool const down = BIT(m_reg[0x703], 3);
	bool const half = BIT(m_reg[0x703], 6);
	if (half)
	{
		m_odd_count = !m_odd_count;
		if (!m_odd_count)
			return;
	}
	int const x = count + (down ? REVERSE_OFFSET : 0);
	bool const colour = pen() == PEN_COLOUR;
	for (int g = 0; g < 3; g++)
	{
		offs_t const a = 0xfff500 + 8 * g;
		u32 const pa = reg32(a) + (down ? -8 : 8);
		u32 const pb = reg32(a + 4) + (down ? -8 : 8);
		set_reg32(a, pa);
		set_reg32(a + 4, pb);
		u16 const w = space.read_word((pa + 2 * g) & 0xfffffe);
		space.write_word((pb + 2 * g) & 0xfffffe, 0);
		if (!m_sheet)
			continue;
		for (int b = 0; b < 16; b++)
		{
			if (BIT(w, 15 - b))
			{
				int const ink = colour ? (g + 1) : 0;
				int const row = 16 * g + b + (colour ? COLOUR_DY[g] : 0);
				int const cx = x + (colour ? COLOUR_DX[g] : 0);
				plot(cx, row, ink);
				if (half)
					plot(cx + (down ? -1 : 1), row, ink);
			}
		}
	}
}

void hp_dj540_device::plot(int x, int row, int ink)
{
	// ink 0 black, 1 yellow, 2 magenta, 3 cyan; overlapping inks mix
	static constexpr u32 INK[4] = { 0x000000, 0xffff00, 0xff00ff, 0x00ffff };
	int const px = x - PAPER_LEFT + SHEET_LEFT;
	int const y = sheet_row() + row;
	if ((px < 0) || (px >= int(PAPER_WIDTH * HDPI)) || (y < 0) || (y >= m_sheet_len))
		return;
	u32 &p = m_bitmap_printer->pix(y, px);
	u32 const c = INK[ink];
	p = (p & 0xff000000) | std::min(p & 0xff0000, c & 0xff0000) | std::min(p & 0xff00, c & 0xff00) | std::min(p & 0xff, c & 0xff);
}


//**************************************************************************
//  Glyph engine and column transposer
//**************************************************************************

// FFFA08 starts the glyph engine; the destination is FFF52C, the pitch
// FFF520 and the number of words FFFA0A:
//  010E: FFF40C-FFF433 hold 20 words, two rows of context, 16 rows, two rows of
//        context; the output turns them through 90 degrees with every dot
//        doubled across (16 rows give a word, 16 dots 32 words); the first
//        block of a glyph (FFFA0C set) starts two words in
//  014E: the 32 words of FFF440-FFF47F (see widen_window) go to memory
//  000E: FFF410-FFF42F, 16 words, turned through 90 degrees into 16 words
void hp_dj540_device::glyph_op()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u16 const op = reg16(0xfffa08);
	u32 dest = reg32(0xfff52c);
	u32 const pitch = reg32(0xfff520);
	int const rows = reg16(0xfffa0a);
	auto win = [this] (int i) { return reg16(0xfff40c + 2 * i); };

	if (op == 0x014e)
	{
		for (int i = 0; i < std::min(rows, 32); i++)
			space.write_word((dest + 2 * i) & 0xfffffe, reg16(0xfff440 + 2 * i));
	}
	else if (op == 0x010e)
	{
		int start = 0;
		if (reg16(0xfffa0c))
		{
			start = 2;
			m_reg[0x60c] = m_reg[0x60d] = 0;
		}
		for (int i = start; (i < 32) && (i - start < rows); i++)
		{
			int const dot = i / 2 - 1;
			u16 w = 0;
			if ((dot >= 0) && (dot < 16))
				for (int r = 0; r < 16; r++)
					if (BIT(win(r + 2), 15 - dot))
						w |= 0x8000 >> r;
			space.write_word(dest & 0xfffffe, w);
			dest += pitch;
		}
	}
	else
	{
		for (int i = 0; i < std::min(rows, 16); i++)
		{
			u16 w = 0;
			for (int r = 0; r < 16; r++)
				if (BIT(win(r + 2), 15 - i))
					w |= 0x8000 >> r;
			space.write_word(dest & 0xfffffe, w);
			dest += pitch;
		}
	}
}

// with FFF400 bit 1 set, writing the last of the 20 window words (FFF40C-FFF433)
// turns the 16 rows into 32 columns of 600 dpi, a word per column with the top
// row in bit 15.  The result lags by two columns: FFF440-FFF443 hold the last two
// columns of the window before, FFF444-FFF47F the first 30 of this one (fitted to
// the swath manager at 0x23CD4, which walks along a band of rows a word at a
// time).  The real engine smooths edges (REt) with the help of the rows of
// context; how is not known, so every dot is simply doubled.
void hp_dj540_device::widen_window()
{
	u16 col[32];
	for (int c = 0; c < 32; c++)
	{
		u16 w = 0;
		for (int r = 0; r < 16; r++)
			if (BIT(reg16(0xfff40c + 2 * (r + 2)), 15 - c / 2))
				w |= 0x8000 >> r;
		col[c] = w;
	}
	auto put = [this] (int i, u16 v) { m_reg[0x040 + 2 * i] = v >> 8; m_reg[0x041 + 2 * i] = v; };
	put(0, m_widen_last >> 16);
	put(1, m_widen_last);
	for (int c = 0; c < 30; c++)
		put(c + 2, col[c]);
	m_widen_last = (u32(col[30]) << 16) | col[31];
}

// FFF800-FFF81F take 16 rows of 16 dots; writing FFFD12 stores them turned
// through 90 degrees at FFF524 as 16 columns 8 bytes apart, and moves FFF524 on
void hp_dj540_device::transpose()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u32 const dest = reg32(0xfff524);
	for (int c = 0; c < 16; c++)
	{
		u16 w = 0;
		for (int r = 0; r < 16; r++)
			if (BIT(reg16(0xfff800 + 2 * r), 15 - c))
				w |= 0x8000 >> r;
		space.write_word((dest + 8 * c) & 0xfffffe, w);
	}
	set_reg32(0xfff524, dest + 0x80);
}


//**************************************************************************
//  Pen, steppers and paper
//**************************************************************************

int hp_dj540_device::pen() const
{
	return m_config->read() & 0x03;
}

// pen ID: the firmware fires nozzle addresses 2E/2F (FFF600-FFF60F, two words for
// each group of 16 addresses, odd addresses in the first) and reads FFFB01 bit 1
// (assumed: the black cartridge answers on address 31, the colour one on 30)
void hp_dj540_device::fire()
{
	auto bit = [this] (int n)
	{
		int const w = 0x200 + 4 * (n >> 4) + (BIT(n, 0) ? 0 : 2);
		return BIT((m_reg[w] << 8) | m_reg[w + 1], 15 - (n & 15));
	};
	m_pen_sense = ((pen() == PEN_BLACK) && bit(0x31)) || ((pen() == PEN_COLOUR) && bit(0x30));
}

void hp_dj540_device::step(int motor, u8 coils)
{
	static const u8 seq[8] = { 0x1, 0x9, 0x8, 0xa, 0x2, 0x6, 0x4, 0x5 };
	int ph = -1;
	for (int i = 0; i < 8; i++)
		if (seq[i] == coils)
			ph = i;
	if (ph < 0)
		return;
	if ((motor == 0) && (m_phase[0] >= 0))
	{
		int const d = (ph - m_phase[0]) & 7;
		if (d == 1)
			paper_step(1);
		else if (d == 7)
			paper_step(-1);
	}
	m_phase[motor] = ph;
}

// paper path: the paper motor feeds 300 half steps per inch.  A sheet is picked
// from the tray when the motor starts forward with the carriage at the right end
// and no sheet in the path; the sensor sees the sheet between its edges
void hp_dj540_device::paper_step(int dir)
{
	attotime const now = machine().time();
	bool const from_rest = (now - m_paper_step_time) > attotime::from_msec(50);
	m_paper_step_time = now;
	if (!m_sheet)
	{
		if ((dir > 0) && from_rest && !BIT(m_config->read(), 2) && (m_cr_pos > PICK_CARRIAGE))
		{
			m_sheet = true;
			m_sheet_lead = -PICK_DIST;
			m_sheet_len = ((m_bitmap_printer->ioport("PAPER")->read() & 0x07) == 3) ? 3508 : 3300;
			LOG("pick\n");
		}
		return;
	}
	m_sheet_lead += dir;
	// the paper view follows the lowest nozzle row; the page is filed when its
	// trailing edge has passed the nozzles
	m_bitmap_printer->m_ypos = std::max(sheet_row() + HEAD_ROWS, 0);
	if (m_bitmap_printer->check_new_page())
	{
		m_sheet = false;
		LOG("sheet out\n");
	}
	m_bitmap_printer->m_ypos = std::max(sheet_row() + HEAD_ROWS, 0);
}

bool hp_dj540_device::paper_sensor() const
{
	return m_sheet && (m_sheet_lead >= 0) && (m_sheet_lead < m_sheet_len);
}


//**************************************************************************
//  Registers and ports
//**************************************************************************

u8 hp_dj540_device::reg_r(offs_t offset)
{
	u8 data = m_reg[offset];
	switch (offset + 0xfff400)
	{
	case 0xffffed: data = ~m_irq_pending; break;    // active low
	case 0xffffeb: data = host_status(); break;
	case 0xfffb01: data = (data & ~0x02) | (m_pen_sense ? 0x02 : 0x00); break;
	case 0xfffd02: case 0xfffd03:
		{
			if (!machine().side_effects_disabled())
				update_carriage();
			u16 const c = s32(m_cr_pos) - m_cr_offset;
			data = (offset & 1) ? (c & 0xff) : (c >> 8);
		}
		break;
	case 0xfff905: data = 0xff; break;      // host control lines (assumed idle)
	case 0xfff900:
		if (!machine().side_effects_disabled())
		{
			m_host_status &= ~0x08;
			m_byte_waiting = false;
			update_host();
		}
		break;
	case 0xfff911: case 0xfff913: case 0xfff915: case 0xfff917:
		{
			int const port = (offset - 0x511) >> 1;
			u8 const dir = m_reg[0x519 + 2 * port];
			data = (m_reg[offset] & dir) | (port_in(port) & ~dir);
		}
		break;
	}
	return data;
}

void hp_dj540_device::reg_w(offs_t offset, u8 data)
{
	if ((offset == 0xa03) || (offset == 0xa07) || (offset == 0xa09))
		update_carriage();
	m_reg[offset] = data;
	switch (offset + 0xfff400)
	{
	case 0xffffef: update_irq(); break;
	case 0xfffff1: m_irq_pending &= data; update_irq(); break;
	case 0xffffdf: update_host(); break;
	case 0xffffe9: dma_waiting(); update_host(); break;
	case 0xfffe0d: update_step_timer(); break;
	case 0xfffe09: update_servo_timer(); break;
	case 0xfffd06: case 0xfffd07: case 0xfffe0f: update_tick(); break;
	case 0xfffd13: transpose(); break;
	case 0xfffa09: glyph_op(); break;
	case 0xfff433: if (BIT(m_reg[0x001], 1)) widen_window(); break;
	case 0xfffd01:
		update_carriage();
		m_cmp_armed = true;
		break;
	case 0xfffd03:
		update_carriage();
		m_cr_offset = s32(m_cr_pos) - reg16(0xfffd02);
		break;
	case 0xfffb0b: fire(); break;
	case 0xfffe0b: step(0, data & 0x0f); break;
	case 0xfffe11: step(1, data & 0x0f); break;
	case 0xfff538: case 0xfff539: case 0xfff53a: case 0xfff53b:
	case 0xfff53c: case 0xfff53d: case 0xfff53e: case 0xfff53f:
		m_host_status &= ~0x01;
		if ((offset == 0x13b) || (offset == 0x13f))
			dma_waiting();
		update_host();
		break;
	case 0xfff911: case 0xfff913: case 0xfff915: case 0xfff917:
		port_changed((offset - 0x511) >> 1);
		break;
	case 0xfff919: case 0xfff91b: case 0xfff91d: case 0xfff91f:
		port_changed((offset - 0x519) >> 1);
		break;
	}
}

u8 hp_dj540_device::port_in(int port)
{
	switch (port)
	{
	case 0:     // bit 3 SDA of the 24C02
		return m_eeprom->read_sda() ? 0xff : 0xf7;
	case 2:
		{
			u8 const keys = m_keys->read();
			u8 const cfg = m_config->read();
			u8 r = 0x43;        // lamp lines read high, Power released
			// Power is held for a moment at power on, as when the printer is switched on
			if (BIT(keys, 0) || (machine().time() < attotime::from_msec(300)))
				r &= ~0x40;
			if (BIT(keys, 1))
				r |= 0x08;
			if (!BIT(cfg, 3))
				r |= 0x10;
			if (paper_sensor())
				r |= 0x20;
			return r;
		}
	default:
		return 0x00;
	}
}

void hp_dj540_device::port_changed(int port)
{
	u8 const dir = m_reg[0x519 + 2 * port];
	u8 const out = m_reg[0x511 + 2 * port] | ~dir;     // inputs float high
	switch (port)
	{
	case 0:
		m_eeprom->write_scl(BIT(out, 4));
		m_eeprom->write_sda(BIT(out, 3));
		// pen ID pulses: the black cartridge answers on bit 7, the colour one on bit 6 (assumed)
		if (BIT(dir, 6) && BIT(m_reg[0x511], 6))
			m_pen_sense = pen() == PEN_COLOUR;
		if (BIT(dir, 7) && BIT(m_reg[0x511], 7))
			m_pen_sense = pen() == PEN_BLACK;
		break;
	case 2:
		m_led[0] = BIT(out, 0) ? 0 : 1;     // Power
		m_led[1] = BIT(out, 1) ? 0 : 1;     // Resume
		break;
	case 3:
		update_busy();
		break;
	}
}


//**************************************************************************
//  Host interface
//**************************************************************************

// a byte is stored by DMA at FFF53C (counting up) while FFFFE9 bit 1 is set and
// FFF53C differs from FFF538; otherwise it waits in FFF900 (FFFFEB bit 3) until
// the firmware reads it or the DMA can take it
void hp_dj540_device::strobe_w(int state)
{
	if (m_strobe && !state)
	{
		m_reg[0x500] = m_data;
		u32 const cur = reg32(0xfff53c), end = reg32(0xfff538);
		if (BIT(m_reg[0xbe9], 1) && (cur != end))
		{
			m_maincpu->space(AS_PROGRAM).write_byte(cur & 0xffffff, m_data);
			set_reg32(0xfff53c, cur + 1);
			m_ack_cb(0);
			m_ack_timer->adjust(attotime::from_usec(5));
		}
		else
		{
			m_host_status |= 0x08;
			m_byte_waiting = true;
		}
		update_host();
	}
	m_strobe = state;
}

void hp_dj540_device::dma_waiting()
{
	u32 const cur = reg32(0xfff53c), end = reg32(0xfff538);
	if (m_byte_waiting && BIT(m_reg[0xbe9], 1) && (cur != end))
	{
		m_maincpu->space(AS_PROGRAM).write_byte(cur & 0xffffff, m_reg[0x500]);
		set_reg32(0xfff53c, cur + 1);
		m_byte_waiting = false;
		m_host_status &= ~0x08;
		m_ack_cb(0);
		m_ack_timer->adjust(attotime::from_usec(5));
	}
}

TIMER_CALLBACK_MEMBER(hp_dj540_device::ack_end)
{
	m_ack_cb(1);
}

u8 hp_dj540_device::host_status() const
{
	bool const dma_busy = BIT(m_reg[0xbe9], 1) && (reg32(0xfff53c) != reg32(0xfff538));
	return (m_host_status & ~0x01) | (dma_busy ? 0x00 : 0x01);
}

void hp_dj540_device::update_host()
{
	m_host_irq = (host_status() & m_reg[0xbdf]) != 0;
	update_irq();
	update_busy();
}

// BUSY: a byte waiting, or the firmware's busy line (port D bit 1)
void hp_dj540_device::update_busy()
{
	m_busy_cb((m_byte_waiting || BIT(m_reg[0x517], 1)) ? 1 : 0);
}

void hp_dj540_device::init_w(int state)
{
}
