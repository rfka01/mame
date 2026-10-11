// license:BSD-3-Clause
// copyright-holders:Dirk Best, rfka01
/**********************************************************************

    Epson LX-800 9-pin dot matrix printer

    CPU uPD7810HG at 14.7456 MHz, 32 KiB ROM, 8 KiB RAM, E05A03 gate array.

      0000-7fff     ROM
      8000-9fff     RAM
      c000-dfff     E05A03 gate array (8 registers, mirrored)

      PA0 out       carriage motor drive (1) or hold (0) voltage
      PA2 out       paper feed motor drive (1) or hold (0) voltage
      PA3 in        LINE FEED key (0 = pressed)
      PA4 in        FORM FEED key (0 = pressed)
      PA5 in        paper end sensor (0 = no paper)
      PA7 in        P/S signal from the optional interface (1 = parallel)
      PB0-PB7 in    DIP switch 1
      PC2 out       ON LINE lamp (0 = on)
      PC3 in        ON LINE key, also INT2
      PC4 out       Centronics /ERROR
      PC5 out       Centronics /ACK
      PC6 (CO0)     FIRE, needle pulse width
      PC7 (CO1)     buzzer
      AN0-AN2       DIP switch 2-1 to 2-3
      AN3           line feed after CR (high): /AUTO FEED XT or DIP switch 2-4
      AN4           /SLCT IN
      AN5           +24 V supervision

    Carriage: 4-phase stepper, 1 step = 1/60", 2-2 phase drive for draft,
    1-1 phase for NLQ. Print column 1 is 25 steps right of the home sensor.
    Paper: 4-phase stepper, 1 step = 1/216".

**********************************************************************/

#include "emu.h"
#include "epson_lx800.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "machine/e05a03.h"
#include "sound/beep.h"

#include "speaker.h"

#include "lx800.lh"

#define LOG_MOTOR (1U << 1)
#define LOG_FIRE  (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_MOTOR | LOG_FIRE)
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 1.0 / 3;   // paper edge to the home sensor edge: column 1 at 3/4"

constexpr double SHEET_SENSOR = 433.0 / 216 - 22.0 / 25.4;   // paper end sensor to print line: the
										// loading feed of 433 steps leaves the first line 22 mm from the top
constexpr double CR_LAG = 0.000555;     // the head lags the rotor by this time (s); found from
										// bidirectional draft (900 pps) and NLQ (450 pps) printing
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 560;           // right stop

class epson_lx800_device : public device_t, public device_centronics_peripheral_interface
{
public:
	// construction/destruction
	epson_lx800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// Centronics
	virtual void input_strobe(int state) override { m_ga->strobe_w(state); }
	virtual void input_init(int state) override;
	virtual void input_autofd(int state) override { m_autofd = state; }
	virtual void input_select_in(int state) override { m_slctin = state; }
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

	DECLARE_INPUT_CHANGED_MEMBER(online_key);
	DECLARE_INPUT_CHANGED_MEMBER(power_switch);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

private:
	void lx800_mem(address_map &map) ATTR_COLD;

	u8 pa_r();
	void pa_w(u8 data);
	u8 pc_r();
	void pc_w(u8 data);
	void co0_w(int state);

	u8 an0_r();
	u8 an1_r();
	u8 an2_r();
	u8 an3_r();
	u8 an4_r();
	u8 an5_r();

	void cr_motor_w(u8 data);
	void pf_motor_w(u8 data);
	void reso_w(int state);

	void restart();
	void carriage_step(int dir);
	double head_x() const;
	void fire();
	u8 keys_held() const;

	required_device<upd7810_device> m_maincpu;
	required_device<e05a03_device> m_ga;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<beep_device> m_beep;
	required_ioport m_panel;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paperout_led;

	// Centronics
	u8 m_data;
	int m_autofd;
	int m_slctin;
	bool m_init;

	u8 m_pa;
	u8 m_pc;
	int m_co0;

	// carriage
	int m_cr_pos;           // 1/60" steps right of the home sensor edge
	int m_cr_angle;         // electrical angle of the motor phases, in half steps
	int m_cr_half;          // carriage position in half steps
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_last_change;
	attotime m_cr_period;

	// paper
	int m_pf_angle;
	int m_pf_half;

	attotime m_power_on;
	bool m_power_cycled;
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( lx800 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("lx800.ic3c", 0x0000, 0x8000, CRC(da06c45b) SHA1(9618c940dd10d5b43cd1edd5763b90e6447de667) )
ROM_END

const tiny_rom_entry *epson_lx800_device::device_rom_region() const
{
	return ROM_NAME( lx800 );
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

INPUT_PORTS_START( epson_lx800 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("LX-800 ON LINE") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx800_device::online_key), 0)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("LX-800 FORM FEED")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("LX-800 LINE FEED")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("LX-800 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx800_device::power_switch), 0)

	PORT_START("DIPSW1")
	PORT_DIPNAME(0x01, 0x00, "Typeface")                      PORT_DIPLOCATION("SW 1:!1")
	PORT_DIPSETTING(   0x00, "Normal")
	PORT_DIPSETTING(   0x01, "Condensed")
	PORT_DIPNAME(0x02, 0x00, "Shape of zero")                 PORT_DIPLOCATION("SW 1:!2")
	PORT_DIPSETTING(   0x00, "Not slashed")
	PORT_DIPSETTING(   0x02, "Slashed")
	PORT_DIPNAME(0x04, 0x00, "Character table")               PORT_DIPLOCATION("SW 1:!3")
	PORT_DIPSETTING(   0x00, "Italics")
	PORT_DIPSETTING(   0x04, "Graphics")
	PORT_DIPNAME(0x08, 0x08, "Paper-out detection")           PORT_DIPLOCATION("SW 1:!4")
	PORT_DIPSETTING(   0x00, "Invalid")
	PORT_DIPSETTING(   0x08, "Valid")
	PORT_DIPNAME(0x10, 0x00, "Printing quality")              PORT_DIPLOCATION("SW 1:!5")
	PORT_DIPSETTING(   0x00, "Draft")
	PORT_DIPSETTING(   0x10, "NLQ")
	PORT_DIPNAME(0xe0, 0xe0, "International character set")   PORT_DIPLOCATION("SW 1:!6,!7,!8")
	PORT_DIPSETTING(   0xe0, "USA")
	PORT_DIPSETTING(   0x60, "France")
	PORT_DIPSETTING(   0xa0, "Germany")
	PORT_DIPSETTING(   0x20, "UK")
	PORT_DIPSETTING(   0xc0, "Denmark")
	PORT_DIPSETTING(   0x40, "Sweden")
	PORT_DIPSETTING(   0x80, "Italy")
	PORT_DIPSETTING(   0x00, "Spain")

	PORT_START("DIPSW2")
	PORT_DIPNAME(0x01, 0x00, "Page length")                   PORT_DIPLOCATION("SW 2:!1")
	PORT_DIPSETTING(   0x00, "11\"")
	PORT_DIPSETTING(   0x01, "12\"")
	PORT_DIPNAME(0x02, 0x00, "Cut-sheet feeder mode")         PORT_DIPLOCATION("SW 2:!2")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Skip over perforation")         PORT_DIPLOCATION("SW 2:!3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, "1\"")
	PORT_DIPNAME(0x08, 0x00, "Auto line feed")                PORT_DIPLOCATION("SW 2:!4")
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) // controlled by /AUTO FEED XT signal (Centronics pin 14)
	PORT_DIPSETTING(   0x08, DEF_STR(On))

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x02, "LINE FEED (self test)" )
	PORT_CONFSETTING(    0x04, "FORM FEED (self test, NLQ)" )
	PORT_CONFSETTING(    0x06, "LINE FEED + FORM FEED (hex dump)" )
INPUT_PORTS_END

ioport_constructor epson_lx800_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( epson_lx800 );
}


//**************************************************************************
//  DEVICE
//**************************************************************************

epson_lx800_device::epson_lx800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, EPSON_LX800, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_ga(*this, "ic3b"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_beep(*this, "beeper"),
	m_panel(*this, "PANEL"),
	m_dsw1(*this, "DIPSW1"),
	m_dsw2(*this, "DIPSW2"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "online_led"),
	m_ready_led(*this, "ready_led"),
	m_paperout_led(*this, "paperout_led"),
	m_data(0),
	m_autofd(1),
	m_slctin(1),
	m_init(true),
	m_pa(0),
	m_pc(0xff),
	m_co0(1),
	m_cr_pos(100),
	m_cr_angle(-1),
	m_cr_half(200),
	m_cr_dir(1),
	m_pf_angle(-1),
	m_pf_half(0),
	m_power_cycled(false)
{
}

void epson_lx800_device::lx800_mem(address_map &map)
{
	map(0x0000, 0x7fff).rom(); /* 32k firmware */
	map(0x8000, 0x9fff).ram(); /* 8k external RAM */
	map(0xa000, 0xbfff).noprw(); /* not used */
	map(0xc000, 0xc007).mirror(0x1ff8).rw(m_ga, FUNC(e05a03_device::read), FUNC(e05a03_device::write));
	map(0xe000, 0xfeff).noprw(); /* not used */
}

void epson_lx800_device::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 14.7456_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_lx800_device::lx800_mem);
	m_maincpu->pa_in_cb().set(FUNC(epson_lx800_device::pa_r));
	m_maincpu->pa_out_cb().set(FUNC(epson_lx800_device::pa_w));
	m_maincpu->pb_in_cb().set_ioport("DIPSW1");
	m_maincpu->pc_in_cb().set(FUNC(epson_lx800_device::pc_r));
	m_maincpu->pc_out_cb().set(FUNC(epson_lx800_device::pc_w));
	m_maincpu->co0_func().set(FUNC(epson_lx800_device::co0_w));
	m_maincpu->co1_func().set(m_beep, FUNC(beep_device::set_state));
	m_maincpu->an0_func().set(FUNC(epson_lx800_device::an0_r));
	m_maincpu->an1_func().set(FUNC(epson_lx800_device::an1_r));
	m_maincpu->an2_func().set(FUNC(epson_lx800_device::an2_r));
	m_maincpu->an3_func().set(FUNC(epson_lx800_device::an3_r));
	m_maincpu->an4_func().set(FUNC(epson_lx800_device::an4_r));
	m_maincpu->an5_func().set(FUNC(epson_lx800_device::an5_r));
	m_maincpu->an6_func().set_constant(0xff);
	m_maincpu->an7_func().set_constant(0xff);

	E05A03(config, m_ga);
	m_ga->busy_wr_callback().set([this] (int state) { output_busy(state); m_ready_led = !state; });
	m_ga->pe_wr_callback().set([this] (int state) { output_perror(state); });
	m_ga->pe_lp_wr_callback().set([this] (int state) { m_paperout_led = state; });
	m_ga->reso_wr_callback().set(FUNC(epson_lx800_device::reso_w));
	m_ga->data_rd_callback().set([this] () { return m_data; });
	m_ga->cr_motor_wr_callback().set(FUNC(epson_lx800_device::cr_motor_w));
	m_ga->pf_motor_wr_callback().set(FUNC(epson_lx800_device::pf_motor_w));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);
	m_bitmap_printer->set_sheet_load_offset(SHEET_SENSOR);
	m_bitmap_printer->set_sheet_in_at_power_on(true);
	m_bitmap_printer->set_sheet_top(22.0 / 25.4);

	config.set_default_layout(layout_lx800);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 3250).add_route(ALL_OUTPUTS, "mono", 0.3);
}

void epson_lx800_device::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_autofd));
	save_item(NAME(m_slctin));
	save_item(NAME(m_init));
	save_item(NAME(m_pa));
	save_item(NAME(m_pc));
	save_item(NAME(m_co0));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_angle));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_last_change));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_angle));
	save_item(NAME(m_pf_half));
	save_item(NAME(m_cr_half));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void epson_lx800_device::device_reset()
{
	m_beep->set_state(0);
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_cr_last_change = m_cr_last_step;
	m_power_on = machine().time();
	m_power_cycled = false;
	m_ga->home_w(m_cr_pos >= 0);  // low in the home zone
	m_maincpu->set_input_line(UPD7810_INTF2, CLEAR_LINE);
	output_ack(1);
	output_fault(1);
	output_select(1);
}

void epson_lx800_device::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_ga->reset();
	m_power_on = machine().time();
	m_power_cycled = false;
}

INPUT_CHANGED_MEMBER(epson_lx800_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

INPUT_CHANGED_MEMBER(epson_lx800_device::online_key)
{
	// the ON LINE key also drives INT2
	m_maincpu->set_input_line(UPD7810_INTF2, newval ? CLEAR_LINE : ASSERT_LINE);
}

u8 epson_lx800_device::keys_held() const
{
	// keys held while the printer is switched on with its power switch (bits as in PANEL)
	static u8 const held[4] = { 0x00, 0x04, 0x02, 0x06 };
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return held[(m_config->read() >> 1) & 3];
	return 0;
}

void epson_lx800_device::input_init(int state)
{
	// /INIT resets the printer controller
	if (m_init && !state)
		restart();
	m_init = state;
}

void epson_lx800_device::reso_w(int state)
{
	LOG("gate array reset output\n");
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

u8 epson_lx800_device::pa_r()
{
	u8 const keys = m_panel->read() & ~keys_held();
	bool const paper = BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();

	u8 data = 0x47 | 0x80;          // parallel interface
	if (BIT(keys, 2))
		data |= 0x08;               // LINE FEED
	if (BIT(keys, 1))
		data |= 0x10;               // FORM FEED
	if (paper)
		data |= 0x20;
	return data;
}

void epson_lx800_device::pa_w(u8 data)
{
	m_pa = data;
}

u8 epson_lx800_device::pc_r()
{
	return 0xff;
}

void epson_lx800_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	m_online_led = !BIT(data, 2);
	output_select(!BIT(data, 2));
	output_fault(BIT(data, 4));
	if ((old ^ data) & 0x20)
		output_ack(BIT(data, 5));
}

void epson_lx800_device::co0_w(int state)
{
	// the needles fire while FIRE is low
	if (m_co0 && !state)
		fire();
	m_co0 = state;
}

u8 epson_lx800_device::an0_r()
{
	return BIT(m_dsw2->read(), 0) ? 0x00 : 0xff;
}

u8 epson_lx800_device::an1_r()
{
	return BIT(m_dsw2->read(), 1) ? 0x00 : 0xff;
}

u8 epson_lx800_device::an2_r()
{
	return BIT(m_dsw2->read(), 2) ? 0x00 : 0xff;
}

u8 epson_lx800_device::an3_r()
{
	// line feed after CR: high when /AUTO FEED XT is low or DIP switch 2-4 is on
	return (BIT(m_dsw2->read(), 3) || !m_autofd) ? 0xff : 0x00;
}

u8 epson_lx800_device::an4_r()
{
	return 0x00;    // /SLCT IN, printer always selected
}

u8 epson_lx800_device::an5_r()
{
	return 0xcb;    // +24 V
}


//-------------------------------------------------
//  motors and printhead
//-------------------------------------------------

namespace {

// electrical angle (in half steps, 0-7) of the energised phases, -1 if none or invalid;
// the windings are on bits 3 (A), 1 (B), 2 (/A) and 0 (/B)
int phase_angle(u8 phases)
{
	static int const angle[16] = {
		-1,  6,  2, -1,  4,  5,  3, -1,
		 0,  7,  1, -1, -1, -1, -1, -1 };
	return angle[phases & 0x0f];
}

// change of the angle, -4..3 half steps
int angle_delta(int from, int to)
{
	return ((to - from + 4) & 7) - 4;
}

} // anonymous namespace

void epson_lx800_device::cr_motor_w(u8 data)
{
	LOGMASKED(LOG_MOTOR, "%s CR %x PA %02x pos %d\n", machine().time().as_string(), data, m_pa, m_cr_pos);

	// the carriage phases are active low; a rising angle moves the carriage to the right
	int const angle = phase_angle(~data);
	if (angle < 0)
		return;
	if (m_cr_angle >= 0)
	{
		int const delta = angle_delta(m_cr_angle, angle);
		if (delta)
		{
			m_cr_half = std::clamp(m_cr_half + delta, CR_LEFT * 2, CR_RIGHT * 2 + 1);   // the motor slips at the stops
			carriage_step((delta > 0) ? 1 : -1);
		}
	}
	m_cr_angle = angle;
}

void epson_lx800_device::pf_motor_w(u8 data)
{
	LOGMASKED(LOG_MOTOR, "%s PF %x PA %02x\n", machine().time().as_string(), data, m_pa);

	int const angle = phase_angle(data);
	if (angle < 0)
		return;
	if (m_pf_angle >= 0)
	{
		int const old = m_pf_half >> 1;
		m_pf_half -= angle_delta(m_pf_angle, angle);   // a falling angle feeds the paper forward
		int const now = m_pf_half >> 1;
		if (now != old)
		{
			if (now > old)
				m_bitmap_printer->m_ypos++;
			else
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
	}
	m_pf_angle = angle;
}

void epson_lx800_device::carriage_step(int dir)
{
	// a 2-2 phase step is written in two halves a few microseconds apart;
	// the second half belongs to the same step
	attotime const now = machine().time();
	if ((now - m_cr_last_change) > attotime::from_usec(50))
	{
		attotime const since = now - m_cr_last_step;
		m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
		m_cr_last_step = now;
		m_cr_dir = dir;
	}
	m_cr_last_change = now;

	m_cr_pos = m_cr_half >> 1;
	m_ga->home_w(m_cr_pos >= 0);  // low in the home zone
	m_bitmap_printer->setheadpos(int((m_cr_pos / 60.0 + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);
}

double epson_lx800_device::head_x() const
{
	// a step starts the move to the new rotor position (two phases on lie half a
	// step from one phase on), which is reached at the next step when the
	// carriage runs at constant speed
	double const pos = m_cr_half / 2.0;
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return pos;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return pos - m_cr_dir * (1.0 - frac + CR_LAG / m_cr_period.as_double());
}

void epson_lx800_device::fire()
{
	u16 const head = m_ga->printhead();
	LOGMASKED(LOG_FIRE, "%s fire %03x x %.2f y %d\n", machine().time().as_string(), head, head_x(), m_bitmap_printer->m_ypos);
	if (!head)
		return;

	int const x = int(std::lround((head_x() / 60.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles are 1/72" = 3 paper steps apart
	for (int needle = 0; needle < 9; needle++)
		if (BIT(head, needle))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + needle * (VDPI / 72), 0x000000);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_LX800, device_centronics_peripheral_interface, epson_lx800_device, "lx800", "Epson LX-800")
