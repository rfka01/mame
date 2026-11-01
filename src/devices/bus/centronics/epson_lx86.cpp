// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Epson LX-86 9-pin dot matrix printer (1986)

    Main board Y56021100001:
      uPD7810HG CPU
      EPROM "U5 LX4" (16 KiB)
      SRM2016 2 KiB RAM
      E05020AA gate array (Centronics latch, needle latch)
      D4584, 74LS32, 74LS75, 7407, TL494 (head and motor supply)

    The crystal is not readable on the photos. 11.0592 MHz gives exactly
    the specified 120 cps in draft (720 carriage steps per second).

    Memory map:
      0000-3fff     EPROM
      a000-a7ff     RAM
      c000-c00f     gate array
        c000 r      Centronics data, latched at the falling edge of /STROBE
        c001 r      bit 0: data latched (cleared by reading c000)
        c002 w      needles 1-8 (bit 7 = needle 1), 0 = fire
        c003 w      needle 9 in bit 7, 0 = fire
        c004 r      optional serial interface board
        c005 w      unknown (written once at power on)

    Ports:
      PA0-PA1 out   carriage stepper, Gray code, 1 step = 1/60"
      PA2 out       carriage motor current
      PA3 in        unused here
      PA4 in        paper end sensor (1 = paper), also drives the PAPER OUT lamp
      PA5-PA7 in    DIP switches 1-8, 1-7, 1-6 (country)
      PB0-PB1 out   paper stepper, Gray code, 1 step = 1/216"
      PB2 out       paper motor current
      PB3 out       Centronics BUSY
      PB4 in        DIP switch 2-3 (auto line feed)
      PB5 in        LINE FEED/DRAFT key (0 = pressed)
      PB6 out       Centronics /ERROR (low when off line or after an error)
      PB7 in        FORM FEED/NLQ key (0 = pressed)
      PC0 out       Centronics /ACK
      PC2 in        DIP switch 2-2 (printer select: 0 = always selected,
                    1 = wait for DC1)
      PC3 in        ON LINE key (0 = pressed), also INT2
      PC4 in        carriage home sensor (0 = home zone)
      PC5 out       1 = off line (ON LINE lamp off)
      PC6 (CO0)     needle pulse: the latched needles fire on the falling edge
      PC7 (CO1)     buzzer; DIP switch 2-4 (silent) pulls the line low, and
                    the firmware reads it back before it beeps
      INT1          carriage timing pulse, one per step
      AN0           head supply voltage (the needle pulse width depends on it)
      AN1           DIP switch 2-1 (slashed zero)
      AN2           serial interface board present (low)
      AN3-AN7       DIP switches 1-1 to 1-5

    DIP switches read low when ON (the firmware compares the analog inputs
    against 80h).

    Power-on keys: LF/DRAFT = draft self test, FF/NLQ = NLQ self test, both =
    hex dump.

    TODO:
    - where the carriage timing pulse comes from (here: one per step)
    - the gate array register c005 and the optional serial board

***************************************************************************/

#include "emu.h"
#include "epson_lx86.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "sound/spkrdev.h"

#include "speaker.h"

#include "epson_lx86.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.72;    // paper edge to the home sensor edge: column 1 at 3/4"
constexpr int CR_EDGE = 40;             // home sensor edge (moving right), steps from the left stop
constexpr int CR_HYST = 6;              // moving left, the sensor switches this many steps further left (the
										// firmware stops 20 steps left of the edge seen when moving right, 13 after
										// the edge seen when moving left)
constexpr int CR_RIGHT = CR_EDGE + 560; // right stop
constexpr double CR_LEAD = 1.0 / (3 * 720); // the dots land this much time (s) ahead of the step count
										// in the direction of travel (found from bidirectional draft printing)


class epson_lx86_device : public device_t, public device_centronics_peripheral_interface
{
public:
	epson_lx86_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	virtual void input_strobe(int state) override;
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
	DECLARE_INPUT_CHANGED_MEMBER(paper_changed) { update_outputs(); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);
	u8 pa_r();
	u8 pb_r();
	u8 pc_r();
	void pa_w(u8 data);
	void pb_w(u8 data);
	void pc_w(u8 data);
	void co0_w(int state);
	void co1_w(int state);
	u8 an_r(int channel);

	TIMER_CALLBACK_MEMBER(pulse_end);

	bool paper_present() const;
	bool keys_held(u8 mask) const;
	void update_outputs();
	double head_x() const;
	void fire();

	required_device<upd7810_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<speaker_sound_device> m_speaker;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_panel;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paper_led;

	emu_timer *m_pulse_timer;

	// Centronics
	u8 m_data;
	u8 m_latch;
	bool m_strobe;
	bool m_flag;

	u8 m_pa;
	u8 m_pb;
	u8 m_pc;
	u8 m_needles[2];
	int m_co0;
	int m_co1;

	// carriage (1/60" steps) and paper (1/216" steps)
	int m_cr_pos;
	int m_cr_phase;
	int m_cr_dir;
	bool m_home;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pf_phase;

	attotime m_power_on;
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( epson_lx86 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("epson_lx_86.bin", 0x0000, 0x4000, CRC(252d36c4) SHA1(bf303553a7b7be4dd5d75ca57df3d12a87c944cc))
ROM_END

const tiny_rom_entry *epson_lx86_device::device_rom_region() const
{
	return ROM_NAME( epson_lx86 );
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( epson_lx86 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("LX-86 ON LINE") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx86_device::online_key), 0)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("LX-86 FF/NLQ")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("LX-86 LF/DRAFT")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("LX-86 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx86_device::power_switch), 0)

	PORT_START("DSW1")      // ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Print width" )            PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, "Condensed" )
	PORT_DIPSETTING(    0x01, "Pica" )
	PORT_DIPNAME( 0x02, 0x02, "Character table" )        PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "Graphics" )
	PORT_DIPSETTING(    0x02, "Standard" )
	PORT_DIPNAME( 0x04, 0x04, "Cut-sheet feeder" )       PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, "Active" )
	PORT_DIPSETTING(    0x04, "Inactive" )
	PORT_DIPNAME( 0x08, 0x08, "Form length" )            PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, "12\"" )
	PORT_DIPSETTING(    0x08, "11\"" )
	PORT_DIPNAME( 0x10, 0x10, "Paper-out sensor" )       PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, "Inactive" )
	PORT_DIPSETTING(    0x10, "Active" )
	PORT_DIPNAME( 0xe0, 0x00, "Country" )                PORT_DIPLOCATION("SW1:6,7,8")
	PORT_DIPSETTING(    0x00, "U.S.A." )
	PORT_DIPSETTING(    0x80, "France" )
	PORT_DIPSETTING(    0x40, "Germany" )
	PORT_DIPSETTING(    0xc0, "United Kingdom" )
	PORT_DIPSETTING(    0x20, "Denmark/Norway" )
	PORT_DIPSETTING(    0xa0, "Sweden" )
	PORT_DIPSETTING(    0x60, "Italy" )
	PORT_DIPSETTING(    0xe0, "Spain" )

	PORT_START("DSW2")      // ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Slashed zero" )           PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPNAME( 0x02, 0x00, "Printer select" )         PORT_DIPLOCATION("SW2:2")
	PORT_DIPSETTING(    0x00, "Fixed (always selected)" )
	PORT_DIPSETTING(    0x02, "By DC1/DC3" )
	PORT_DIPNAME( 0x04, 0x04, "Automatic line feed" )    PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(    0x00, "CR+LF" )
	PORT_DIPSETTING(    0x04, "CR only" )
	PORT_DIPNAME( 0x08, 0x08, "Beeper" )                 PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, "Silent" )
	PORT_DIPSETTING(    0x08, "Sounds" )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" ) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx86_device::paper_changed), 0)
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, DEF_STR( None ) )
	PORT_CONFSETTING(    0x04, "LF/DRAFT (draft self test)" )
	PORT_CONFSETTING(    0x02, "FF/NLQ (NLQ self test)" )
	PORT_CONFSETTING(    0x06, "Both (hex dump)" )
INPUT_PORTS_END

ioport_constructor epson_lx86_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( epson_lx86 );
}


//**************************************************************************
//  DEVICE
//**************************************************************************

epson_lx86_device::epson_lx86_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, EPSON_LX86, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_speaker(*this, "speaker"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_panel(*this, "PANEL"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "lx86_online_led"),
	m_ready_led(*this, "lx86_ready_led"),
	m_paper_led(*this, "lx86_paper_led"),
	m_pulse_timer(nullptr),
	m_data(0),
	m_latch(0),
	m_strobe(true),
	m_flag(false),
	m_pa(0xff),
	m_pb(0xff),
	m_pc(0xff),
	m_needles{ 0xff, 0xff },
	m_co0(1),
	m_co1(0),
	m_cr_pos(CR_EDGE + 30),
	m_cr_phase(0),
	m_cr_dir(1),
	m_home(false),
	m_pf_phase(0)
{
}

void epson_lx86_device::mem_map(address_map &map)
{
	map(0x0000, 0x3fff).rom().region("maincpu", 0);
	map(0xa000, 0xa7ff).ram();
	map(0xc000, 0xc00f).rw(FUNC(epson_lx86_device::ga_r), FUNC(epson_lx86_device::ga_w));
}

void epson_lx86_device::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 11.0592_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_lx86_device::mem_map);
	m_maincpu->pa_in_cb().set(FUNC(epson_lx86_device::pa_r));
	m_maincpu->pb_in_cb().set(FUNC(epson_lx86_device::pb_r));
	m_maincpu->pc_in_cb().set(FUNC(epson_lx86_device::pc_r));
	m_maincpu->pa_out_cb().set(FUNC(epson_lx86_device::pa_w));
	m_maincpu->pb_out_cb().set(FUNC(epson_lx86_device::pb_w));
	m_maincpu->pc_out_cb().set(FUNC(epson_lx86_device::pc_w));
	m_maincpu->co0_func().set(FUNC(epson_lx86_device::co0_w));
	m_maincpu->co1_func().set(FUNC(epson_lx86_device::co1_w));
	m_maincpu->an0_func().set([this] () { return an_r(0); });
	m_maincpu->an1_func().set([this] () { return an_r(1); });
	m_maincpu->an2_func().set([this] () { return an_r(2); });
	m_maincpu->an3_func().set([this] () { return an_r(3); });
	m_maincpu->an4_func().set([this] () { return an_r(4); });
	m_maincpu->an5_func().set([this] () { return an_r(5); });
	m_maincpu->an6_func().set([this] () { return an_r(6); });
	m_maincpu->an7_func().set([this] () { return an_r(7); });

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_epson_lx86);

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.5);
}

void epson_lx86_device::device_start()
{
	m_pulse_timer = timer_alloc(FUNC(epson_lx86_device::pulse_end), this);

	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_flag));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_needles));
	save_item(NAME(m_co0));
	save_item(NAME(m_co1));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_home));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
}

void epson_lx86_device::device_reset()
{
	m_flag = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	m_cr_last_step = machine().time();
	m_cr_period = attotime::never;
	m_power_on = machine().time();
	output_ack(1);
	update_outputs();
}

INPUT_CHANGED_MEMBER(epson_lx86_device::online_key)
{
	// the key also drives INT2 (falling edge on PC3)
	m_maincpu->set_input_line(UPD7810_INTF2, newval ? ASSERT_LINE : CLEAR_LINE);
}

INPUT_CHANGED_MEMBER(epson_lx86_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
		m_flag = false;
		m_power_on = machine().time();
		update_outputs();
	}
}


//-------------------------------------------------
//  board I/O
//-------------------------------------------------

bool epson_lx86_device::paper_present() const
{
	return BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
}

bool epson_lx86_device::keys_held(u8 mask) const
{
	// the keys chosen in the configuration are held for the first seconds
	return (m_config->read() & mask) && ((machine().time() - m_power_on) < attotime::from_seconds(2));
}

u8 epson_lx86_device::ga_r(offs_t offset)
{
	switch (offset)
	{
	case 0:
		if (!machine().side_effects_disabled())
		{
			m_flag = false;
			update_outputs();
		}
		return m_latch;

	case 1:
		return m_flag ? 0xff : 0xfe;

	default:
		if (!machine().side_effects_disabled())
			LOG("%s gate array read %x\n", machine().describe_context(), offset);
		return 0xff;   // c004: no serial board
	}
}

void epson_lx86_device::ga_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 2:
	case 3:
		m_needles[offset - 2] = data;
		break;

	default:
		LOG("%s gate array write %x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
}

void epson_lx86_device::input_strobe(int state)
{
	// the gate array latches the data at the falling edge of /STROBE
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_flag = true;
		update_outputs();
	}
	m_strobe = state;
}

u8 epson_lx86_device::an_r(int channel)
{
	u8 const dsw1 = m_dsw1->read(), dsw2 = m_dsw2->read();
	switch (channel)
	{
	case 0:  return 0xc8;                                  // head supply voltage (valid range bd-f9)
	case 1:  return BIT(dsw2, 0) ? 0xff : 0x00;            // SW2-1
	case 2:  return 0xff;                                  // no serial board
	default: return BIT(dsw1, channel - 3) ? 0xff : 0x00;  // SW1-1..5
	}
}

u8 epson_lx86_device::pa_r()
{
	u8 data = 0x0f | (m_dsw1->read() & 0xe0);   // country on PA5-PA7
	if (paper_present())
		data |= 0x10;
	return data;
}

u8 epson_lx86_device::pb_r()
{
	u8 const panel = m_panel->read();
	u8 data = 0xff;
	if (!BIT(m_dsw2->read(), 2))
		data &= ~0x10;
	if (BIT(panel, 2) || keys_held(0x04))
		data &= ~0x20;  // LF/DRAFT
	if (BIT(panel, 1) || keys_held(0x02))
		data &= ~0x80;  // FF/NLQ
	return data;
}

u8 epson_lx86_device::pc_r()
{
	u8 data = 0x63;
	if (BIT(m_dsw2->read(), 1))
		data |= 0x04;
	if (!BIT(m_panel->read(), 0))
		data |= 0x08;   // ON LINE key
	if (!m_home)
		data |= 0x10;   // home sensor
	if (BIT(m_dsw2->read(), 3))
		data |= 0x80;   // the beeper switch holds the line low
	return data;
}

void epson_lx86_device::update_outputs()
{
	bool const online = !BIT(m_pc, 5);
	bool const busy = m_flag || BIT(m_pb, 3);
	bool const paper = paper_present();

	output_busy(busy);
	output_select(online);
	output_fault(BIT(m_pb, 6));
	output_perror(!paper);

	m_online_led = online ? 1 : 0;
	m_ready_led = busy ? 0 : 1;
	m_paper_led = paper ? 0 : 1;
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

void epson_lx86_device::pa_w(u8 data)
{
	u8 const old = m_pa;
	m_pa = data;

	// carriage stepper, Gray code on PA1:PA0
	if ((old ^ data) & 0x03)
	{
		static int const gray[4] = { 0, 1, 3, 2 };
		int const phase = gray[data & 3];
		int const delta = (phase - m_cr_phase) & 3;
		m_cr_phase = phase;
		if (delta == 1 || delta == 3)
		{
			int const dir = (delta == 1) ? 1 : -1;
			attotime const now = machine().time();
			attotime const since = now - m_cr_last_step;
			m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
			m_cr_last_step = now;
			m_cr_dir = dir;
			m_cr_pos = std::clamp(m_cr_pos + dir, 0, CR_RIGHT);
			if (m_cr_pos <= CR_EDGE - CR_HYST)
				m_home = true;
			else if (m_cr_pos > CR_EDGE)
				m_home = false;
			m_bitmap_printer->setheadpos(int((m_cr_pos - CR_EDGE) / 60.0 * HDPI + LEFT_OFFSET * HDPI), m_bitmap_printer->m_ypos);

			// timing pulse on INT1
			m_maincpu->set_input_line(UPD7810_INTF1, ASSERT_LINE);
			m_pulse_timer->adjust(attotime::from_usec(200));
		}
	}
}

TIMER_CALLBACK_MEMBER(epson_lx86_device::pulse_end)
{
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
}

void epson_lx86_device::pb_w(u8 data)
{
	u8 const old = m_pb;
	m_pb = data;

	// paper stepper, Gray code on PB1:PB0
	if ((old ^ data) & 0x03)
	{
		static int const gray[4] = { 2, 1, 3, 0 };  // forward: 11 01 00 10
		int const phase = gray[data & 3];
		int const delta = (phase - m_pf_phase) & 3;
		m_pf_phase = phase;
		if (delta == 1)
			m_bitmap_printer->m_ypos++;
		else if (delta == 3)
			m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
		if (delta == 1 || delta == 3)
		{
			m_bitmap_printer->check_new_page();
			update_outputs();
		}
	}

	if ((old ^ data) & 0x48)
		update_outputs();
}

void epson_lx86_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	if ((old ^ data) & 0x01)
		output_ack(BIT(data, 0));
	if ((old ^ data) & 0x20)
		update_outputs();
}

void epson_lx86_device::co0_w(int state)
{
	if (m_co0 && !state)
		fire();
	m_co0 = state;
}

void epson_lx86_device::co1_w(int state)
{
	m_co1 = state;
	m_speaker->level_w((state && BIT(m_dsw2->read(), 3)) ? 1 : 0);
}


//-------------------------------------------------
//  printing
//-------------------------------------------------

double epson_lx86_device::head_x() const
{
	// steps from the home sensor edge; the carriage moves on between two steps
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos - CR_EDGE;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - CR_EDGE - m_cr_dir * (1.0 - frac);
}

void epson_lx86_device::fire()
{
	u16 const head = u16(~m_needles[0] & 0xff) << 1 | BIT(~m_needles[1], 7);   // bit 8 = needle 1
	if (!head)
		return;

	double const lead = m_cr_period.is_never() ? 0.0 : CR_LEAD / m_cr_period.as_double();
	int const x = int(std::lround((head_x() + m_cr_dir * lead) / 60.0 * HDPI + LEFT_OFFSET * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles are 1/72" = 3 paper steps apart
	int const y = m_bitmap_printer->m_ypos;
	for (int needle = 0; needle < 9; needle++)
		if (BIT(head, 8 - needle))
			m_bitmap_printer->draw_pixel(x, y + needle * (VDPI / 72), 0x000000);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_LX86, device_centronics_peripheral_interface, epson_lx86_device, "lx86", "Epson LX-86")
