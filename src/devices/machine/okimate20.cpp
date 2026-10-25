// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Okidata Okimate 20 thermal transfer colour printer: logic board and
    mechanism

    Logic board after the Sams ComputerFacts CP34 (model EN3211):
    8031 (Q6) at 11.059 MHz, 16 KiB ROM (Q1) on the Plug 'n Print module,
    8 KiB RAM (Q2), M60307 (Q3) printer gate array: print head shift
    register, motor phases PM PH1-4 shared by the line feed and the
    carriage motor, motor drive, ready lamp, sensors and darkness slide.
    M60306 (Q4) at 0000: interval timer, print head solenoid and the
    parallel interface.  The module also decodes the M60307: c000 on the
    Commodore module, 8000 on the IBM PC module.

      program 0000-3fff  ROM
      data    0000-1fff  M60306
      data    4000-5fff  RAM

    M60307, as far as derived from the firmware:
      0 w    head data, three writes for 24 elements (first byte = top 8, lsb at the top)
      0 r    bits 1-4: darkness slide (one contact low), bit 5: ribbon sensor (opaque)
      1 w    bit 0: heat pulse (the rising edge fires the latched elements)
      1 r    bit 0: select key (low = pressed), bit 3: module (0 = parallel / serial interface),
             bit 4: paper, bit 6: home sensor (low at home)
      2 w    carriage step period in 57.6 kHz clocks; a read acknowledges INT1
      4 w    bit 6: away from home, bit 7: stepping (clear holds the phases),
             bits 4-5: motor current (?), bit 1: falling edge starts the step generator
      5 w    bits 0-1: paper motor phases (Gray code, 1/144 inch per step),
             bits 2-3: line feed / carriage motor drive, bit 6: ready lamp
      6 w    bit 5: step generator running, bits 0-1 both set: one interrupt per
             character cell of 12 steps instead of one per step
    M60306 (timer and head):
      0      status (bit 1: interval elapsed), a write clears it
      3      bit 0: head down (solenoid hold)
      4-5    interval count

    Colour ribbon: ESC EM starts a line of three passes (yellow, magenta,
    cyan).  The firmware finds the black marker by moving the carriage home
    with the head up while watching the ribbon sensor, after advancing the
    ribbon in steps of 194; the panels then follow at a fixed pitch.

    TODO:
    - the ribbon geometry (marker length, clear strip, sensor position)
      is fitted to the firmware, not measured
    - heat time and darkness have no effect on the dots

**********************************************************************/

#include "emu.h"
#include "okimate20.h"

#include "okimat20.lh"

#define LOG_IO   (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

// carriage step clock (guess: crystal / 192 = 57.6 kHz): the speed tables then give 40 cps in
// correspondence and 80 cps in data processing quality as specified, and the end of the
// acceleration ramp matches the cruise speed
constexpr XTAL STEP_CLOCK = 11.0592_MHz_XTAL / 192;
// M60306 interval timer clock (guess: ALE / 16, about 22 ms for the firmware's count of 2500)
constexpr XTAL Q4_CLOCK = 11.0592_MHz_XTAL / 96;
constexpr int STEPS_PER_CELL = 12;      // carriage steps of 1/120 inch per interrupt while printing

// paper and resolution
constexpr double PAPER_WIDTH = 10.0;    // inches
constexpr double PAPER_HEIGHT = 11.0;
constexpr int HDPI = 240;
constexpr int VDPI = 144;
constexpr int DOT_WIDTH = 2;            // pixels at HDPI: a heat pulse covers one carriage step
constexpr double STEP_INCH = 1.0 / 120; // carriage step (guess)
constexpr double LEFT_OFFSET = 0.3;     // paper edge to the home position, inches (guess)

// colour ribbon, in carriage steps: a black marker, then yellow, magenta and cyan panels of one
// line each and a clear strip.  The firmware finds the marker with the sensor, then expects the
// panels at a fixed pitch of 1020 steps; the lengths of the marker and the clear strip are guesses
// that let the next marker be found right after the cyan pass.
constexpr int RIB_MARK = 60;
constexpr int RIB_PANEL = 1020;
constexpr int RIB_PERIOD = 3192;
constexpr int RIB_HEAD = 138;           // ribbon between the head and the point seen by the sensor

} // anonymous namespace


INPUT_PORTS_START( okimate20 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Okimate 20 Select")
	PORT_CONFNAME( 0x1e, 0x1a, "Darkness" )
	PORT_CONFSETTING(    0x1c, "1 (light)" )
	PORT_CONFSETTING(    0x1a, "2" )
	PORT_CONFSETTING(    0x16, "3" )
	PORT_CONFSETTING(    0x0e, "4 (dark)" )

	PORT_START("SENSORS")
	PORT_CONFNAME( 0x10, 0x10, "Paper" )
	PORT_CONFSETTING(    0x10, "Loaded" )
	PORT_CONFSETTING(    0x00, "Out" )

	PORT_START("CONF")
	PORT_CONFNAME( 0x03, 0x00, "Ribbon" )
	PORT_CONFSETTING(    0x00, "Black" )
	PORT_CONFSETTING(    0x01, "Colour" )
	PORT_CONFSETTING(    0x02, "None (thermal paper)" )
INPUT_PORTS_END


okimate20_base_device::okimate20_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, offs_t io_base) :
	device_t(mconfig, type, tag, owner, clock),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_panel(*this, "PANEL"),
	m_sensors(*this, "SENSORS"),
	m_conf(*this, "CONF"),
	m_ready_led(*this, "okimate_ready_led"),
	m_io_base(io_base),
	m_head(0),
	m_pf_phase(0),
	m_cr_pos(240),
	m_cr_dir(1),
	m_cr_count(0),
	m_rib_pos(0),
	m_q4_status(0),
	m_q4_count(0),
	m_q4_ctrl(0)
{
}

void okimate20_base_device::prog_map(address_map &map)
{
	map(0x0000, 0x3fff).rom().region("maincpu", 0);
}

void okimate20_base_device::data_map(address_map &map)
{
	map(0x0000, 0x0007).mirror(0x1ff8).rw(FUNC(okimate20_base_device::q4_r), FUNC(okimate20_base_device::q4_w));
	map(0x4000, 0x5fff).ram();
	map(m_io_base, m_io_base + 7).rw(FUNC(okimate20_base_device::io_r), FUNC(okimate20_base_device::io_w));
}

void okimate20_base_device::device_add_mconfig(machine_config &config)
{
	I8031(config, m_maincpu, 11.0592_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &okimate20_base_device::prog_map);
	m_maincpu->set_addrmap(AS_DATA, &okimate20_base_device::data_map);
	m_maincpu->port_in_cb<1>().set([this] () { return port1_r(); });
	m_maincpu->port_out_cb<1>().set([this] (u8 data) { port1_w(data); });
	m_maincpu->port_in_cb<3>().set([this] () { return port3_r(); });
	m_maincpu->port_out_cb<3>().set([this] (u8 data) { port3_w(data); });

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_dot_style(printer_dot_style::thermal(1.0 / HDPI, 1.0 / VDPI));

	config.set_default_layout(layout_okimat20);
}

void okimate20_base_device::device_start()
{
	m_step_timer = timer_alloc(FUNC(okimate20_base_device::step_tick), this);
	m_q4_timer = timer_alloc(FUNC(okimate20_base_device::q4_tick), this);

	std::fill(std::begin(m_io), std::end(m_io), 0);

	save_item(NAME(m_io));
	save_item(NAME(m_head));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_count));
	save_item(NAME(m_cr_start));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_rib_pos));
	save_item(NAME(m_q4_status));
	save_item(NAME(m_q4_count));
	save_item(NAME(m_q4_ctrl));
}

void okimate20_base_device::device_reset()
{
}

void okimate20_base_device::fire()
{
	int const x = int((head_pos() * STEP_INCH + LEFT_OFFSET) * HDPI);
	LOGMASKED(LOG_IO, "FIRE rib %d pos %.1f head %06x c004 %02x c006 %02x t %s y %d\n", m_rib_pos, head_pos(), m_head & 0xffffff, m_io[4], m_io[6], machine().time().as_string(6), m_bitmap_printer->m_ypos);
	int const color = ink();
	if (color < 0)
		return;
	for (int e = 0; e < 24; e++)
	{
		if (BIT(m_head, (2 - e / 8) * 8 + (e & 7)))     // first byte = top elements, lsb first
		{
			int const y = m_bitmap_printer->m_ypos + e;
			for (int dx = 0; dx < DOT_WIDTH; dx++)
				m_bitmap_printer->draw_pixel(x + dx, y, add_ink(m_bitmap_printer->get_pixel(x + dx, y), color));
		}
	}
}

void okimate20_base_device::paper_step(u8 data)
{
	// c005 bits 0-1: paper motor, Gray code 00 01 11 10
	static const int gray[4] = { 0, 1, 3, 2 };
	int const phase = gray[data & 3];
	int const delta = (phase - m_pf_phase) & 3;
	m_pf_phase = phase;
	if (delta == 1)
		m_bitmap_printer->m_ypos++;
	else if (delta == 3)
		m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
	else
		return;
	m_bitmap_printer->check_new_page();
}

TIMER_CALLBACK_MEMBER(okimate20_base_device::step_tick)
{
	// carriage step generator: one step of 1/120 inch per period of c002 clocks while c006 bit 5 is set;
	// it interrupts after every step, or after every 12th step (one character cell) while c006 bits 0-1 are set
	if (!BIT(m_io[6], 5))
	{
		// stopped: the generator halts with one more interrupt but no further step; the firmware
		// counts the steps of a move from these interrupts and returns from the last pass of a
		// colour line by count, so a step here shifts every following line to the right
		m_maincpu->set_input_line(MCS51_INT1_LINE, ASSERT_LINE);
		m_cr_count = 0;
		return;
	}
	int const dir = BIT(m_io[4], 6) ? 1 : -1;   // c004 bit 6 = away from home
	if (BIT(m_io[4], 7))                        // c004 bit 7 clear: phases held (start of a move)
	{
		bool const old = marker();
		m_cr_pos += dir;
		if ((dir > 0) && BIT(m_q4_ctrl, 0))
			m_rib_pos++;                        // with the head down the ribbon stays on the paper and unwinds
		if (old != marker())
			LOGMASKED(LOG_IO, "marker %d rib %d pos %d\n", marker(), m_rib_pos, m_cr_pos);
	}
	m_cr_dir = dir;
	m_cr_start = machine().time();
	m_bitmap_printer->setheadpos(int((m_cr_pos * STEP_INCH + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);
	int divide = 1;
	if ((m_io[6] & 0x03) == 0x03)
		divide = BIT(m_io[6], 3) ? STEPS_PER_CELL : STEPS_PER_CELL / 3;
	if (++m_cr_count >= divide)
	{
		m_cr_count = 0;
		m_maincpu->set_input_line(MCS51_INT1_LINE, ASSERT_LINE);
	}
	m_cr_period = attotime::from_ticks(std::max(1, int(m_io[2])), STEP_CLOCK);
	m_step_timer->adjust(m_cr_period);
}

bool okimate20_base_device::marker() const
{
	if (!BIT(m_conf->read(), 0))
		return true;            // a black ribbon covers the sensor
	// The sensor sits at the home end of the frame.  With the head down the used ribbon is laid on
	// the paper behind the head, so the sensor sees the point that was under the head at its own
	// position: moving the carriage home with the head up scans along the ribbon.
	switch (m_conf->read() & 0x03)
	{
	case 0:
		return true;            // the black ribbon is opaque
	case 1:
		return (((m_rib_pos - m_cr_pos) % RIB_PERIOD + RIB_PERIOD) % RIB_PERIOD) < RIB_MARK;
	default:
		return false;
	}
}

int okimate20_base_device::ink() const
{
	// the ribbon panel under the head: 0 yellow, 1 magenta, 2 cyan, 3 black, -1 none
	if (!BIT(m_q4_ctrl, 0))
		return -1;              // head up
	if ((m_conf->read() & 0x03) != 1)
		return 3;
	int const p = ((m_rib_pos - RIB_HEAD) % RIB_PERIOD + RIB_PERIOD) % RIB_PERIOD;
	if (p < RIB_MARK)
		return 3;
	int const panel = (p - RIB_MARK) / RIB_PANEL;
	return (panel < 3) ? panel : -1;    // past the cyan panel: clear strip
}

u32 okimate20_base_device::ink_mix(int inks)
{
	// colour of the wax inks in the set (bit 0 yellow .. bit 3 black) on white paper
	static constexpr u32 colors[4] = { 0xfff200, 0xec008c, 0x00aeef, 0x202020 };
	u32 mix = 0xffffff;
	for (int i = 0; i < 4; i++)
	{
		if (BIT(inks, i))
		{
			u32 m = 0;
			for (int shift = 0; shift < 24; shift += 8)
				m |= ((((mix >> shift) & 0xff) * ((colors[i] >> shift) & 0xff)) / 255) << shift;
			mix = m;
		}
	}
	return mix;
}

u32 okimate20_base_device::add_ink(u32 paper, int ink)
{
	// the head melts the wax off the ribbon onto the paper; inks of different panels mix, but
	// a dot transferred again in the same ink (the band overlap some programs print) stays as it was
	paper &= 0xffffff;
	for (int inks = 0; inks < 16; inks++)
	{
		if (ink_mix(inks) == paper)
			return BIT(inks, ink) ? paper : ink_mix(inks | (1 << ink));
	}
	return ink_mix(1 << ink);
}

double okimate20_base_device::head_pos() const
{
	// the carriage moves smoothly between interrupts
	if (!BIT(m_io[6], 5) || m_cr_period.is_zero())
		return m_cr_pos;
	double const f = std::min(1.0, (machine().time() - m_cr_start).as_double() / m_cr_period.as_double());
	return m_cr_pos + m_cr_dir * f;
}

// M60306 interface gate array: interval timer, which paces the firmware's housekeeping
// (lamp blinking, key debouncing, delays), and head solenoid; the module handles the rest
u8 okimate20_base_device::q4_r(offs_t offset)
{
	u8 const data = module_r(offset);
	if (offset == 0)
		return (data & ~0x02) | (m_q4_status & 0x02);  // bit 1: interval elapsed
	return data;
}

void okimate20_base_device::q4_w(offs_t offset, u8 data)
{
	module_w(offset, data);
	if (offset == 3)
		m_q4_ctrl = data;
	if (offset == 2 || offset == 3)
		LOGMASKED(LOG_IO, "%s q4_w %d = %02x (pos %d rib %d)\n", machine().describe_context(), offset, data, m_cr_pos, m_rib_pos);
	switch (offset)
	{
	case 0: // command: clears the status
		m_q4_status &= ~0x02;
		break;
	case 4:
		m_q4_count = (m_q4_count & 0xff00) | data;
		break;
	case 5:
		// timer count high (the top bits look like 8155 timer mode bits), starts the interval
		m_q4_count = (m_q4_count & 0x00ff) | ((data & 0x3f) << 8);
		m_q4_timer->adjust(attotime::from_ticks(std::max<u16>(m_q4_count, 1), Q4_CLOCK));
		break;
	}
}

TIMER_CALLBACK_MEMBER(okimate20_base_device::q4_tick)
{
	m_q4_status |= 0x02;
}

u8 okimate20_base_device::io_r(offs_t offset)
{
	u8 data = 0xff;
	switch (offset)
	{
	case 0:
		// bits 1-4: darkness slide (one contact low), bit 5: ribbon marker (?)
		data = 0xc1 | (m_panel->read() & 0x1e) | (marker() ? 0x20 : 0x00);
		break;
	case 1:
		// bit 0: select key (low = pressed), bit 3: module, bit 4: paper, bit 5 must be high,
		// bit 6: home sensor (low at home)
		data = 0xa6 | (m_panel->read() & 0x01) | (module_id() & 0x08) | (m_sensors->read() & 0x10) | ((m_cr_pos > 0) ? 0x40 : 0x00);
		break;
	case 2:
		// acknowledges the carriage step interrupt
		if (!machine().side_effects_disabled())
			m_maincpu->set_input_line(MCS51_INT1_LINE, CLEAR_LINE);
		break;
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s io_r %d = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void okimate20_base_device::io_w(offs_t offset, u8 data)
{
	{
		LOGMASKED(LOG_IO, "%s io_w %d = %02x (pos %d)\n", machine().describe_context(), offset, data, m_cr_pos);
	}
	u8 const old = m_io[offset];
	m_io[offset] = data;
	if (offset == 0)
		m_head = (m_head << 8) | data;      // three writes load the 24 elements
	else if ((offset == 1) && BIT(data, 0) && !BIT(old, 0))
		fire();                             // heat pulse starts
	else if (offset == 5)
	{
		paper_step(data);
		m_ready_led = BIT(data, 6);
	}
	else if ((offset == 6) && ((old ^ data) & 0x03))
		m_cr_count = 0;                     // the interrupt divider restarts
	if ((offset == 4) && BIT(old, 1) && !BIT(data, 1))
	{
		// provisional: start the step generator
		m_cr_period = attotime::from_ticks(std::max(1, int(m_io[2])), STEP_CLOCK);
		m_cr_start = machine().time();
		m_step_timer->adjust(m_cr_period);
	}
}

