// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Okidata Okimate 20 thermal transfer colour printer, Commodore version

    Logic board after the Sams ComputerFacts CP34 (model EN3211):
    8031 (Q6) at 11.059 MHz, 16 KiB ROM (Q1, here CO2.5G1, Bitstream font),
    8 KiB RAM (Q2), M60307 (Q3) printer gate array: print head shift
    register, motor phases PM PH1-4 shared by the line feed and the
    carriage motor, motor drive, ready lamp, sensors and darkness slide.
    The EN3211 has a second gate array (M60306, Q4) at 0000 for the serial
    or parallel interface; the Commodore firmware talks to the serial bus
    directly through port 3 and uses Q4 only for its interval timer and
    the print head solenoid.
    Other Plug 'n Print modules carry a PROM at f000 (P1.7 low) which takes
    INT0; not emulated.
    The Commodore MCS 810 is the same printer with firmware CO 02.5 (CO2.5G1
    differs in a few bytes); the MCS 820 is the Okimate 20 with the IBM PC
    parallel and serial module.

      program 0000-3fff  ROM
      data    0000-1fff  M60306
      data    4000-5fff  RAM
      data    c000-c007  M60307

      P1.0-5    function switches (low = on), switch 6 selects device 4 or 5
      P3.0      serial bus DATA in (inverted)
      P3.1      serial bus DATA out (high pulls DATA low)
      P3.4      serial bus ATN in (inverted)
      P3.5      serial bus CLK in (inverted)
      INT1      carriage step (M60307)

    M60307, as far as derived from the firmware:
      c000 w    head data, three writes for 24 elements (first byte = top 8, lsb at the top)
      c000 r    bits 1-4: darkness slide (one contact low), bit 5: ribbon sensor (opaque)
      c001 w    bit 0: heat pulse (the rising edge fires the latched elements)
      c001 r    bit 0: select key (low = pressed), bit 4: paper, bit 6: home sensor (low at home)
      c002 w    carriage step period in 57.6 kHz clocks; a read acknowledges INT1
      c004 w    bit 6: away from home, bit 7: stepping (clear holds the phases),
                bits 4-5: motor current (?), bit 1: falling edge starts the step generator
      c005 w    bits 0-1: paper motor phases (Gray code, 1/144 inch per step),
                bits 2-3: line feed / carriage motor drive, bit 6: ready lamp
      c006 w    bit 5: step generator running, bits 0-1 both set: one interrupt per
                character cell of 12 steps instead of one per step
    M60306:
      0000      status (bit 1: interval elapsed), a write clears it
      0002      bit 4: head solenoid pull-in
      0003      bit 0: head down (solenoid hold)
      0004-0005 interval count

    Colour ribbon: ESC EM starts a line of three passes (yellow, magenta,
    cyan).  The firmware finds the black marker by moving the carriage home
    with the head up while watching the ribbon sensor, after advancing the
    ribbon in steps of 194; the panels then follow at a fixed pitch.

    TODO:
    - the ribbon geometry (marker length, clear strip, sensor position)
      is fitted to the firmware, not measured
    - meanings of function switches 1-5 for the Commodore firmware
    - heat time and darkness have no effect on the dots

**********************************************************************/

#include "emu.h"
#include "okimate20.h"
#include "mcs810.h"

#include "cpu/mcs51/i8051.h"
#include "machine/bitmap_printer.h"

#include "okimat20.lh"

#define LOG_IO   (1U << 1)
#define LOG_PORT (1U << 2)

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

class okimate20_device : public device_t, public device_cbm_iec_interface
{
public:
	okimate20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static constexpr flags_type emulation_flags() { return flags::NOT_WORKING; }

protected:
	okimate20_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void cbm_iec_reset(int state) override;

private:
	void prog_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;

	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);
	u8 q4_r(offs_t offset);
	void q4_w(offs_t offset, u8 data);

	TIMER_CALLBACK_MEMBER(step_tick);
	TIMER_CALLBACK_MEMBER(q4_tick);

	void fire();
	void paper_step(u8 data);
	double head_pos() const;
	bool marker() const;
	int ink() const;
	static u32 ink_mix(int inks);
	static u32 add_ink(u32 paper, int ink);

	required_device<i8031_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_panel;
	required_ioport m_sensors;
	required_ioport m_sw;
	required_ioport m_conf;
	output_finder<> m_ready_led;

	emu_timer *m_step_timer;
	emu_timer *m_q4_timer;

	u8 m_io[8];             // M60307 registers as written
	u8 m_p1, m_p3, m_p3in;
	u32 m_head;             // 24 head elements, shifted in through c000
	int m_pf_phase;         // paper motor phase (Gray code)
	int m_cr_pos;           // carriage position, steps from the home sensor edge
	int m_cr_dir;
	int m_cr_count;         // steps since the last interrupt
	attotime m_cr_start;
	attotime m_cr_period;
	int m_rib_pos;          // ribbon taken from the supply spool, carriage steps
	u8 m_q4_status;
	u16 m_q4_count;
	u8 m_q4_ctrl;           // offset 3 bit 0: head down
};


// Commodore MCS 810: the Okimate 20 with Commodore's firmware revision CO 02.5
class mcs810_device : public okimate20_device
{
public:
	mcs810_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		okimate20_device(mconfig, MCS810, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};


ROM_START( okimat20 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("co25g1.bin", 0x0000, 0x4000, CRC(050b4590) SHA1(a7558326cb0c7879424f9a5d861ed4e6da5d189f))
ROM_END

ROM_START( mcs810 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("mcs810-65-1115.bin", 0x0000, 0x4000, CRC(7d88d6f5) SHA1(b62396eba1910bc809979dd5444a16c7ffd431c5))
ROM_END

const tiny_rom_entry *okimate20_device::device_rom_region() const
{
	return ROM_NAME( okimat20 );
}

const tiny_rom_entry *mcs810_device::device_rom_region() const
{
	return ROM_NAME( mcs810 );
}


INPUT_PORTS_START( okimat20 )
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

	PORT_START("SW")    // function switches, read through P1 (1 = ON); meanings for the Commodore firmware to be confirmed
	PORT_DIPNAME( 0x01, 0x00, "SW1" )             PORT_DIPLOCATION("SW:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "SW2" )             PORT_DIPLOCATION("SW:2")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "SW3" )             PORT_DIPLOCATION("SW:3")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "SW4" )             PORT_DIPLOCATION("SW:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "SW5" )             PORT_DIPLOCATION("SW:5")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, "Device number" )   PORT_DIPLOCATION("SW:6")
	PORT_DIPSETTING(    0x00, "4" )
	PORT_DIPSETTING(    0x20, "5" )
INPUT_PORTS_END

ioport_constructor okimate20_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( okimat20 );
}


okimate20_device::okimate20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	okimate20_device(mconfig, OKIMATE20, tag, owner, clock)
{
}

okimate20_device::okimate20_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_cbm_iec_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_panel(*this, "PANEL"),
	m_sensors(*this, "SENSORS"),
	m_sw(*this, "SW"),
	m_conf(*this, "CONF"),
	m_ready_led(*this, "okimate_ready_led"),
	m_p1(0xff),
	m_p3(0xff),
	m_p3in(0),
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

void okimate20_device::prog_map(address_map &map)
{
	map(0x0000, 0x3fff).rom().region("maincpu", 0);
}

void okimate20_device::data_map(address_map &map)
{
	map(0x0000, 0x0007).mirror(0x1ff8).rw(FUNC(okimate20_device::q4_r), FUNC(okimate20_device::q4_w));
	map(0x4000, 0x5fff).ram();
	map(0xc000, 0xc007).rw(FUNC(okimate20_device::io_r), FUNC(okimate20_device::io_w));
}

void okimate20_device::device_add_mconfig(machine_config &config)
{
	I8031(config, m_maincpu, 11.0592_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &okimate20_device::prog_map);
	m_maincpu->set_addrmap(AS_DATA, &okimate20_device::data_map);
	// P1: function switches (low = ON, mapping to be verified), P1.7 high: no module ROM
	m_maincpu->port_in_cb<1>().set([this] () { return u8(~m_sw->read() | 0x80); });
	m_maincpu->port_out_cb<1>().set([this] (u8 data) { if (data != m_p1) LOGMASKED(LOG_PORT, "%s P1 %02x\n", machine().describe_context(), data); m_p1 = data; });
	m_maincpu->port_in_cb<3>().set([this] () {
		// serial bus lines through inverting buffers
		u8 const v = u8(0xce | (m_bus->data_r() ? 0x00 : 0x01) | (m_bus->atn_r() ? 0x00 : 0x10) | (m_bus->clk_r() ? 0x00 : 0x20));
		if (v != m_p3in && !machine().side_effects_disabled())
		{
			LOGMASKED(LOG_PORT, "%s P3 in %02x  t %s\n", machine().describe_context(), v, machine().time().as_string(6));
			m_p3in = v;
		}
		return v;
	});
	m_maincpu->port_out_cb<3>().set([this] (u8 data) {
		if (data != m_p3)
			LOGMASKED(LOG_PORT, "%s P3 %02x atn %d clk %d data %d  t %s\n", machine().describe_context(), data, m_bus->atn_r(), m_bus->clk_r(), m_bus->data_r(), machine().time().as_string(6));
		m_p3 = data;
		m_bus->data_w(this, !BIT(data, 1));
	});

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_dot_style(printer_dot_style::thermal(1.0 / HDPI, 1.0 / VDPI));

	config.set_default_layout(layout_okimat20);
}

void okimate20_device::device_start()
{
	m_step_timer = timer_alloc(FUNC(okimate20_device::step_tick), this);
	m_q4_timer = timer_alloc(FUNC(okimate20_device::q4_tick), this);

	std::fill(std::begin(m_io), std::end(m_io), 0);

	save_item(NAME(m_io));
	save_item(NAME(m_p1));
	save_item(NAME(m_p3));
	save_item(NAME(m_p3in));
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

void okimate20_device::device_reset()
{
	m_p3 = 0xff;
}

void okimate20_device::cbm_iec_reset(int state)
{
	if (!state)
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
}

void okimate20_device::fire()
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

void okimate20_device::paper_step(u8 data)
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

TIMER_CALLBACK_MEMBER(okimate20_device::step_tick)
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
	int const divide = ((m_io[6] & 0x03) == 0x03) ? STEPS_PER_CELL : 1;
	if (++m_cr_count >= divide)
	{
		m_cr_count = 0;
		m_maincpu->set_input_line(MCS51_INT1_LINE, ASSERT_LINE);
	}
	m_cr_period = attotime::from_ticks(std::max(1, int(m_io[2])), STEP_CLOCK);
	m_step_timer->adjust(m_cr_period);
}

bool okimate20_device::marker() const
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

int okimate20_device::ink() const
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

u32 okimate20_device::ink_mix(int inks)
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

u32 okimate20_device::add_ink(u32 paper, int ink)
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

double okimate20_device::head_pos() const
{
	// the carriage moves smoothly between interrupts
	if (!BIT(m_io[6], 5) || m_cr_period.is_zero())
		return m_cr_pos;
	double const f = std::min(1.0, (machine().time() - m_cr_start).as_double() / m_cr_period.as_double());
	return m_cr_pos + m_cr_dir * f;
}

// M60306 interface gate array: only its interval timer, which paces the
// firmware's housekeeping (lamp blinking, key debouncing, delays), is used here
u8 okimate20_device::q4_r(offs_t offset)
{
	u8 data = 0xff;
	if (offset == 0)
		data = m_q4_status;     // bit 1: interval elapsed
	return data;
}

void okimate20_device::q4_w(offs_t offset, u8 data)
{
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

TIMER_CALLBACK_MEMBER(okimate20_device::q4_tick)
{
	m_q4_status |= 0x02;
}

u8 okimate20_device::io_r(offs_t offset)
{
	u8 data = 0xff;
	switch (offset)
	{
	case 0:
		// bits 1-4: darkness slide (one contact low), bit 5: ribbon marker (?)
		data = 0xc1 | (m_panel->read() & 0x1e) | (marker() ? 0x20 : 0x00);
		break;
	case 1:
		// bit 0: select key (low = pressed), bit 4: paper, bit 5 must be high, bit 6: home sensor (low at home)
		data = 0xae | (m_panel->read() & 0x01) | (m_sensors->read() & 0x10) | ((m_cr_pos > 0) ? 0x40 : 0x00);
		break;
	case 2:
		// acknowledges the carriage step interrupt
		if (!machine().side_effects_disabled())
			m_maincpu->set_input_line(MCS51_INT1_LINE, CLEAR_LINE);
		break;
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s io_r c00%d = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void okimate20_device::io_w(offs_t offset, u8 data)
{
	{
		LOGMASKED(LOG_IO, "%s io_w c00%d = %02x (pos %d)\n", machine().describe_context(), offset, data, m_cr_pos);
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

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(OKIMATE20, device_cbm_iec_interface, okimate20_device, "okimat20", "Okidata Okimate 20 (Commodore)")
DEFINE_DEVICE_TYPE_PRIVATE(MCS810,    device_cbm_iec_interface, mcs810_device,    "mcs810",   "Commodore MCS 810 Color Printer")
