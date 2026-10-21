// license:BSD-3-Clause
// copyright-holders:Golden Child, rfka01
/***************************************************************************

    Epson RX-80 9-pin dot matrix printer

    uPD7810 at 11 MHz, 8 KiB program ROM (M64200CA) on the external bus,
    256 bytes of internal RAM only (the line buffer is FF00-FF88).

    Ports (after the Sams Computerfacts CP9 schematic and the firmware):
      PA0           /ERROR                PA1 in   paper end sensor (1 = paper)
      PA2 in        DIP SW2-3 and /AUTO FEED XT (0 = line feed after CR)
      PA3 in        DIP SW2-2             PA4      /ACK
      PA5           0 pulse: clear the strobe flip-flop
      PA6           BUSY                  PA7      PD source: 0 = Centronics data, 1 = DIP SW1
      PB1:PB0       carriage motor phases (1 step = 1/60"), switched on at each PTS pulse
      PB3:PB2       paper stepper, Gray code, 1 step = 1/216"
      PB4..PB6      motor supply
      PB7           needle 9, active low
      PC0           0 = clear the ON LINE flip-flop (off line)
      PC2 in        ON LINE flip-flop /Q (0 = on line)
      PC3 in        strobe flip-flop (0 = data received)
      PC4 in        home sensor
      PC5, INT1     print timing sensor (PTS) on the carriage motor
      PC6 (CO0)     head drive: low for the needle pulse, started by PTS
      PC7 (CO1)     buzzer
      PD            data bus: Centronics data, or DIP SW1 with SW1-1 on PD7
      PF            needles 1..8, active low
      AN0           head supply voltage, sets the needle pulse length
      AN2, AN3      FORM FEED and LINE FEED keys (low = pressed)
      AN4           interface type (high = parallel)
      AN6, AN7      DIP SW2-1 and SW2-4 (low = ON)

    Holding LINE FEED while the printer is switched on runs the self test,
    holding both keys the hex dump; select them under "Keys held at power on"
    and use the printer's power switch input, which restarts only the printer.

    TODO:
    - serial interface board

***************************************************************************/

#include "emu.h"
#include "epson_rx80.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "sound/spkrdev.h"

#include "speaker.h"

#include <cmath>
#include <numbers>

#include "epson_rx80.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.75;    // paper edge to the home sensor edge
constexpr int CR_DPI = 60;             // carriage steps per inch
constexpr double CR_LEFT = -40.0;       // left stop, steps from the home sensor edge
constexpr double CR_RIGHT = 1040.0;     // right stop

// carriage motor: the rotor angle (in steps) follows the torque of the energised
// phase against viscous and dry friction; PTS is high for a part of every step
constexpr double CR_TORQUE = 60000.0;   // steps/s^2 at full current
constexpr double CR_DAMP = 70.0;        // 1/s
constexpr double CR_FRICTION = 2000.0;  // steps/s^2
constexpr double CR_DT = 10e-6;         // integration step
constexpr double CR_TICK = 50e-6;       // sensor sampling while the carriage moves
constexpr double PTS_ON = 0.5;          // PTS window within a step
constexpr double PTS_WIDTH = 0.25;

// Gray code phase index of a 2-bit stepper port value (00 01 11 10 = forward)
constexpr int gray_phase[4] = { 0, 1, 3, 2 };

class epson_rx80_device : public device_t, public device_centronics_peripheral_interface
{
public:
	// construction/destruction
	epson_rx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// Centronics
	virtual void input_init(int state) override;
	virtual void input_strobe(int state) override;
	virtual void input_autofd(int state) override { m_autofd = state; }
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
	void rx80_mem(address_map &map) ATTR_COLD;

	u8 pa_r();
	void pa_w(u8 data);
	void pb_w(u8 data);
	u8 pc_r();
	void pc_w(u8 data);
	u8 pd_r();
	void pf_w(u8 data);
	void co0_w(int state);
	void co1_w(int state);
	u8 an2_r();
	u8 an3_r();
	u8 an6_r();
	u8 an7_r();

	TIMER_CALLBACK_MEMBER(carriage_tick);

	void restart();
	void update_busy();
	void set_pts(int state);
	void carriage_update();
	double carriage_torque() const;
	void fire_needles();
	bool paper_loaded();
	u8 keys_held() const;

	required_device<upd7810_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<speaker_sound_device> m_speaker;
	required_ioport m_panel;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paper_led;

	emu_timer *m_cr_timer;

	// Centronics
	u8 m_data;              // data latch 4A
	bool m_strobe_ff;       // set by /STROBE, cleared by PA5
	int m_strobe;
	bool m_init;
	int m_autofd;           // /AUTO FEED XT level
	bool m_online;

	// outputs
	u8 m_pa;
	u8 m_pb;
	u8 m_pc;
	u8 m_pf;
	int m_co0;
	bool m_co0_reset;       // CO0 still to be raised after a reset

	// carriage
	double m_cr_pos;        // rotor angle in steps right of the home sensor edge
	double m_cr_vel;        // steps/s
	attotime m_cr_time;     // time of m_cr_pos
	int m_pts;              // print timing sensor level

	// paper
	int m_pf_phase;

	attotime m_power_on;
	bool m_power_cycled;    // switched on with the power switch, not by a machine reset or /INIT
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( epson_rx80 )
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("rx80_2764.bin", 0x0000, 0x2000, CRC(5206104a) SHA1(3e304f5331181aedb321d3db23a9387e3cfacf0c))
ROM_END

const tiny_rom_entry *epson_rx80_device::device_rom_region() const
{
	return ROM_NAME(epson_rx80);
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( epson_rx80 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("RX-80 ON LINE") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_rx80_device::online_key), 0)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("RX-80 FORM FEED")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("RX-80 LINE FEED")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("RX-80 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_rx80_device::power_switch), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x04, "LINE FEED (self test)" )
	PORT_CONFSETTING(    0x06, "LINE FEED and FORM FEED (hex dump)" )

	PORT_START("DSW1")  // read through PD with PA7 high, ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Character pitch" )            PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, "Condensed" )
	PORT_DIPSETTING(    0x01, "Normal" )
	PORT_DIPNAME( 0x02, 0x02, "Graphics characters" )        PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "Control codes" )
	PORT_DIPSETTING(    0x02, "Graphics" )
	PORT_DIPNAME( 0x04, 0x04, "Buzzer" )                     PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, "Form length" )                PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, "12\"" )
	PORT_DIPSETTING(    0x08, "11\"" )
	PORT_DIPNAME( 0x10, 0x10, "Paper-end detector" )         PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x10, "Enabled" )
	PORT_DIPNAME( 0xe0, 0x00, "International characters" )   PORT_DIPLOCATION("SW1:6,7,8")
	PORT_DIPSETTING(    0x00, "USA" )
	PORT_DIPSETTING(    0x80, "France" )
	PORT_DIPSETTING(    0x40, "Germany" )
	PORT_DIPSETTING(    0xc0, "England" )
	PORT_DIPSETTING(    0x20, "Denmark" )
	PORT_DIPSETTING(    0xa0, "Sweden" )
	PORT_DIPSETTING(    0x60, "Italy" )
	PORT_DIPSETTING(    0xe0, "Spain" )

	PORT_START("DSW2")  // ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Slashed zero" )               PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPNAME( 0x02, 0x00, "Printer select" )             PORT_DIPLOCATION("SW2:2")
	PORT_DIPSETTING(    0x00, "Fixed" )
	PORT_DIPSETTING(    0x02, "DC1/DC3" )
	PORT_DIPNAME( 0x04, 0x04, "Auto line feed with CR" )     PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(    0x00, "Always" )
	PORT_DIPSETTING(    0x04, "From the host" )
	PORT_DIPNAME( 0x08, 0x08, "Skip over perforation" )      PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, "1\"" )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
INPUT_PORTS_END

ioport_constructor epson_rx80_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(epson_rx80);
}


//**************************************************************************
//  DEVICE
//**************************************************************************

epson_rx80_device::epson_rx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, EPSON_RX80, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_speaker(*this, "buzzer"),
	m_panel(*this, "PANEL"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "rx80_online_led"),
	m_ready_led(*this, "rx80_ready_led"),
	m_paper_led(*this, "rx80_paper_led"),
	m_cr_timer(nullptr),
	m_data(0),
	m_strobe_ff(false),
	m_strobe(1),
	m_init(true),
	m_autofd(1),
	m_online(true),
	m_pa(0xff),
	m_pb(0xff),
	m_pc(0xff),
	m_pf(0xff),
	m_co0(1),
	m_co0_reset(true),
	m_cr_pos(200.0),
	m_cr_vel(0.0),
	m_pts(0),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void epson_rx80_device::rx80_mem(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0);
}

void epson_rx80_device::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 11_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_rx80_device::rx80_mem);
	m_maincpu->pa_in_cb().set(FUNC(epson_rx80_device::pa_r));
	m_maincpu->pa_out_cb().set(FUNC(epson_rx80_device::pa_w));
	m_maincpu->pb_out_cb().set(FUNC(epson_rx80_device::pb_w));
	m_maincpu->pc_in_cb().set(FUNC(epson_rx80_device::pc_r));
	m_maincpu->pc_out_cb().set(FUNC(epson_rx80_device::pc_w));
	m_maincpu->pd_in_cb().set(FUNC(epson_rx80_device::pd_r));
	m_maincpu->pf_out_cb().set(FUNC(epson_rx80_device::pf_w));
	m_maincpu->co0_func().set(FUNC(epson_rx80_device::co0_w));
	m_maincpu->co1_func().set(FUNC(epson_rx80_device::co1_w));
	m_maincpu->an0_func().set_constant(0xc0);   // head supply
	m_maincpu->an1_func().set_constant(0xff);
	m_maincpu->an2_func().set(FUNC(epson_rx80_device::an2_r));
	m_maincpu->an3_func().set(FUNC(epson_rx80_device::an3_r));
	m_maincpu->an4_func().set_constant(0xff);   // parallel interface
	m_maincpu->an5_func().set_constant(0xff);
	m_maincpu->an6_func().set(FUNC(epson_rx80_device::an6_r));
	m_maincpu->an7_func().set(FUNC(epson_rx80_device::an7_r));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_epson_rx80);

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.3);
}

void epson_rx80_device::device_start()
{

	m_cr_timer = timer_alloc(FUNC(epson_rx80_device::carriage_tick), this);

	save_item(NAME(m_data));
	save_item(NAME(m_strobe_ff));
	save_item(NAME(m_strobe));
	save_item(NAME(m_init));
	save_item(NAME(m_autofd));
	save_item(NAME(m_online));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_pf));
	save_item(NAME(m_co0));
	save_item(NAME(m_co0_reset));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_vel));
	save_item(NAME(m_cr_time));
	save_item(NAME(m_pts));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void epson_rx80_device::device_reset()
{
	m_pa = m_pb = m_pc = m_pf = 0xff;
	m_strobe_ff = false;
	m_online = true;
	m_co0_reset = true;
	m_power_on = machine().time();
	m_power_cycled = false;
	m_cr_vel = 0.0;
	m_cr_time = machine().time();
	m_cr_timer->adjust(attotime::zero, 0, attotime::from_double(CR_TICK));
	set_pts(0);
	update_busy();
	output_ack(1);
	output_fault(1);
}

void epson_rx80_device::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_pa = m_pb = m_pc = m_pf = 0xff;
	m_strobe_ff = false;
	m_online = true;
	m_co0_reset = true;
	m_power_on = machine().time();
	m_power_cycled = false;
	update_busy();
}

INPUT_CHANGED_MEMBER(epson_rx80_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

INPUT_CHANGED_MEMBER(epson_rx80_device::online_key)
{
	// the key toggles the ON LINE flip-flop 9C, unless PC0 holds it cleared
	if (newval && BIT(m_pc, 0))
	{
		m_online = !m_online;
		update_busy();
	}
}

u8 epson_rx80_device::keys_held() const
{
	// keys held while the printer is switched on with its power switch (bits as in PANEL)
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return m_config->read() & 0x06;
	return 0;
}

bool epson_rx80_device::paper_loaded()
{
	bool const paper = BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
	m_paper_led = !paper;
	output_perror(!paper);
	return paper;
}


//-------------------------------------------------
//  Centronics
//-------------------------------------------------

void epson_rx80_device::input_strobe(int state)
{
	// the data latch 4A follows the data lines while /STROBE is low,
	// the falling edge sets the strobe flip-flop
	if (m_strobe && !state)
	{
		m_strobe_ff = true;
		update_busy();
	}
	m_strobe = state;
}

void epson_rx80_device::input_init(int state)
{
	// /INIT resets the printer controller
	if (m_init && !state)
		restart();
	m_init = state;
}

void epson_rx80_device::update_busy()
{
	// BUSY: strobe flip-flop, PA6 or off line
	bool const busy = m_strobe_ff || BIT(m_pa, 6) || !m_online;
	output_busy(busy);
	output_select(m_online);
	m_ready_led = !busy;
	m_online_led = m_online;
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

u8 epson_rx80_device::pa_r()
{
	// PA1 paper end sensor, PA2 DIP SW2-3 and /AUTO FEED XT, PA3 DIP SW2-2
	u8 const dsw2 = m_dsw2->read();
	u8 data = 0xf1;
	if (paper_loaded())
		data |= 0x02;
	if (m_autofd && BIT(dsw2, 2))
		data |= 0x04;
	if (BIT(dsw2, 1))
		data |= 0x08;
	return data;
}

void epson_rx80_device::pa_w(u8 data)
{
	u8 const old = m_pa;
	m_pa = data;

	// the firmware takes CO0 high as idle and waits for it before it loads the
	// first needle column, but never raises it itself; the CPU core starts with
	// CO0 low, so raise it when the firmware first writes PA after a reset
	if (m_co0_reset)
	{
		m_co0_reset = false;
		m_maincpu->set_state_int(UPD7810_CO0, 1);
		m_co0 = 1;
	}

	if ((old ^ data) & 0x01)
		output_fault(BIT(data, 0));
	if ((old ^ data) & 0x10)
		output_ack(BIT(data, 4));
	if (!BIT(data, 5))
		m_strobe_ff = false;
	if ((old ^ data) & 0x60)
		update_busy();
}

void epson_rx80_device::pb_w(u8 data)
{
	u8 const old = m_pb;
	m_pb = data;

	// carriage motor: phases on PB1:PB0, current on PB4/PB5
	if ((old ^ data) & 0x33)
	{
		m_pb = old;
		carriage_update();
		m_pb = data;
		if (m_cr_timer->expire().is_never())
			m_cr_timer->adjust(attotime::from_double(CR_TICK), 0, attotime::from_double(CR_TICK));
		LOG("carriage pb %02x at %.3f v %.0f\n", data & 0x33, m_cr_pos, m_cr_vel);
	}

	// paper stepper: Gray code on PB3:PB2
	if ((old ^ data) & 0x0c)
	{
		int const phase = gray_phase[(data >> 2) & 3];
		if (m_pf_phase >= 0)
		{
			int const delta = (phase - m_pf_phase) & 3;
			if (delta == 3)
				m_bitmap_printer->m_ypos++;
			else if (delta == 1)
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
		m_pf_phase = phase;
	}
}

u8 epson_rx80_device::pc_r()
{
	// PC2 ON LINE flip-flop /Q, PC3 strobe flip-flop, PC4 home sensor, PC5 PTS
	u8 data = 0xc3;
	if (!m_online)
		data |= 0x04;
	if (!m_strobe_ff)
		data |= 0x08;
	carriage_update();
	if (m_cr_pos >= 0)
		data |= 0x10;
	if (m_pts)
		data |= 0x20;
	return data;
}

void epson_rx80_device::pc_w(u8 data)
{
	m_pc = data;

	// PC0 low clears the ON LINE flip-flop
	if (!BIT(data, 0) && m_online)
	{
		m_online = false;
		update_busy();
	}
}

u8 epson_rx80_device::pd_r()
{
	// PA7 selects the Centronics data latch or DIP switch 1 (SW1-1 on PD7)
	if (BIT(m_pa, 7))
		return bitswap<8>(m_dsw1->read(), 0, 1, 2, 3, 4, 5, 6, 7);
	return m_data;
}

void epson_rx80_device::pf_w(u8 data)
{
	m_pf = data;
}

void epson_rx80_device::co0_w(int state)
{
	// CO0 goes low at the PTS pulse for the needle pulse
	if (m_co0 && !state)
		fire_needles();
	m_co0 = state;
}

void epson_rx80_device::co1_w(int state)
{
	m_speaker->level_w(state);
}

u8 epson_rx80_device::an2_r()
{
	return ((m_panel->read() | keys_held()) & 0x02) ? 0x00 : 0xff;
}

u8 epson_rx80_device::an3_r()
{
	return ((m_panel->read() | keys_held()) & 0x04) ? 0x00 : 0xff;
}

u8 epson_rx80_device::an6_r()
{
	return BIT(m_dsw2->read(), 0) ? 0xff : 0x00;
}

u8 epson_rx80_device::an7_r()
{
	return BIT(m_dsw2->read(), 3) ? 0xff : 0x00;
}


//-------------------------------------------------
//  carriage and print timing sensor
//-------------------------------------------------

void epson_rx80_device::set_pts(int state)
{
	// the PTS signal goes to PC5 and INT1
	m_pts = state;
	m_maincpu->set_input_line(UPD7810_INTF1, state ? ASSERT_LINE : CLEAR_LINE);
}

double epson_rx80_device::carriage_torque() const
{
	// PB4 low: full current, PB5 low: holding current
	double const current = (BIT(m_pb, 4) ? 0.0 : 1.0) + (BIT(m_pb, 5) ? 0.0 : 0.3);
	if (current == 0.0)
		return 0.0;

	// the phase pattern 00 01 11 10 moves the carriage to the right, one step each
	int const eq = gray_phase[m_pb & 3];
	return current * CR_TORQUE * std::sin(std::numbers::pi / 2 * (eq - m_cr_pos));
}

void epson_rx80_device::carriage_update()
{
	attotime const now = machine().time();
	double dt = (now - m_cr_time).as_double();
	m_cr_time = now;
	if (dt <= 0.0)
		return;

	if (m_cr_vel == 0.0 && std::abs(carriage_torque()) <= CR_FRICTION)
	{
		// at rest until the phases or the current change
		m_cr_timer->adjust(attotime::never);
		return;
	}

	while (dt > 0.0)
	{
		double const h = std::min(dt, CR_DT);
		dt -= h;
		double acc = carriage_torque() - CR_DAMP * m_cr_vel;
		// dry friction
		double const fr = (m_cr_vel > 0.0) ? -CR_FRICTION : (m_cr_vel < 0.0) ? CR_FRICTION : 0.0;
		double const v = m_cr_vel + (acc + fr) * h;
		if ((m_cr_vel > 0.0 && v < 0.0 && acc <= CR_FRICTION) || (m_cr_vel < 0.0 && v > 0.0 && acc >= -CR_FRICTION) || (m_cr_vel == 0.0 && std::abs(acc) <= CR_FRICTION))
			m_cr_vel = 0.0;
		else
			m_cr_vel = v;
		m_cr_pos += m_cr_vel * h;
		if (m_cr_pos < CR_LEFT || m_cr_pos > CR_RIGHT)
		{
			m_cr_pos = std::clamp(m_cr_pos, CR_LEFT, CR_RIGHT);
			m_cr_vel = 0.0;
		}
	}

	m_bitmap_printer->setheadpos(int((m_cr_pos / CR_DPI + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);

	double const frac = m_cr_pos - std::floor(m_cr_pos);
	int const pts = (frac >= PTS_ON && frac < PTS_ON + PTS_WIDTH) ? 1 : 0;
	if (pts != m_pts)
		set_pts(pts);
}

TIMER_CALLBACK_MEMBER(epson_rx80_device::carriage_tick)
{
	carriage_update();
}

void epson_rx80_device::fire_needles()
{
	carriage_update();
	int const x = int(std::lround((m_cr_pos / CR_DPI + LEFT_OFFSET) * HDPI));
	LOG("fire pf %02x pb7 %d at %.2f\n", m_pf, BIT(m_pb, 7), m_cr_pos);
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles 1..8 on PF7..PF0, needle 9 on PB7, active low; 1/72" = 3 paper steps apart
	u16 const needles = ~((m_pf << 1) | BIT(m_pb, 7)) & 0x1ff;
	for (int i = 0; i < 9; i++)
		if (BIT(needles, 8 - i))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + i * (VDPI / 72), 0x000000);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_RX80, device_centronics_peripheral_interface, epson_rx80_device, "epson_rx80", "Epson RX-80")
