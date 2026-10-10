// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Star NL-10 9-pin dot matrix printer (1986)

    Main board "NL-10 CONTROL" (8093 1511-2):
      HD63A03YP CPU, 4.9152 MHz ceramic resonator
      HM6264P-15 8 KiB RAM
      74LS257 x2 (DIP switch / panel multiplexers), HD74LS374 (needle latch),
      74LS139 (decoder), 74LS123, 74LS05/04/32
    "NL-10 DRIVER" board: needle and motor drivers
    The interface is a plug-in cartridge that also carries the firmware.
    Dumped: the IBM PC parallel cartridge "NL-10 I/F-P" (8093 1531-1) with
    a uPD27256 EPROM labelled "NLI 1.6" and a Mitsubishi M54610P.

    Memory map (from the firmware):
      0000          w   watchdog retrigger (not emulated)
      1000          r   buzzer pulse
      2000          r   DIP switches (ON = 0)
      2000          w   needle latch, bit 7 = top needle, 1 = fire
      2001          r   panel keys (active low) and lever switches
      4000          r   Centronics data latch
      4000          w   clear the "data ready" latch
      6000-7fff         RAM
      8000-ffff         cartridge EPROM

    Ports:
      P20 in   data ready (0 = byte latched)      P50 out  needle 9 (0 = fire)
      P21 out  carriage motor current             P51 out  /ACK
      P22 out  0 = paper out                      P52 in   paper sensor (1 = paper)
      P23 out  0 = fatal error                    P53 out  indicator (blinks on paper out)
      P24 out  1 = offline                        P54 out  BOLD LED (0 = on)
      P25 out  needle strobe (Tout2, active low)  P55 out  1 = busy / offline
      P26 in   home sensor (0 = home zone)        P56-57   mode LED code
      P27 out  interface enable (?)
      P60-61   carriage stepper (2 phase, Gray code), 1 step = 1/60"
      P62      paper motor off, P63 motor power save
      P64-67   paper stepper (half steps of 1/432")

    Dot columns are timed by the firmware (not tied to carriage steps); the
    head position is interpolated between carriage steps.

    Power-on key combinations: ON LINE = short self test, PAPER FEED =
    long self test, MODE + PAPER FEED = hex dump, MODE = panel lock,
    BOLD = NLQ.  Hold the keys (or select them under "Keys held at power
    on") and use the printer's power switch input, which restarts only
    the printer.

    TODO:
    - which of P24/P55 is the ON LINE LED and which drives BUSY/SELECT
    - meaning of panel input bit 4 (an active-high stop input)
    - watchdog (write to 0000)

***************************************************************************/

#include "emu.h"
#include "star_nl10.h"

#include "speaker.h"

#include "star_nl10.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 432;               // vertical resolution (one paper half step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 12.0;
constexpr double LEFT_OFFSET = 0.75;    // paper edge to the home sensor edge

constexpr int CR_EDGE = 40;             // home sensor edge, steps from the left stop
constexpr int CR_RIGHT = CR_EDGE + 500; // right stop

// paper stepper P67..P64 in forward order (half steps)
const u8 pf_sequence[8] = { 0xb, 0x9, 0xd, 0x5, 0x7, 0x6, 0xe, 0xa };

} // anonymous namespace


DEFINE_DEVICE_TYPE(STAR_NL10, star_nl10_device, "nl10", "Star NL-10")


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( star_nl10 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("star_nl-10_nli_1.6_ibm.bin", 0x0000, 0x8000, CRC(8e9fb286) SHA1(0769645da54e1368755c908f9f8580e7eb01abdf))
ROM_END

const tiny_rom_entry *star_nl10_device::device_rom_region() const
{
	return ROM_NAME( star_nl10 );
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( star_nl10 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("NL-10 ON LINE")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("NL-10 PAPER FEED")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("NL-10 BOLD")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("NL-10 MODE")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("NL-10 TOF")
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_UNUSED )   // levers, see CONFIG
	PORT_BIT( 0x100, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("NL-10 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(star_nl10_device::power_switch), 0)

	PORT_START("DIP")
	PORT_DIPNAME( 0x01, 0x00, "Input buffer" )               PORT_DIPLOCATION("SW:1")
	PORT_DIPSETTING(    0x00, "5 KB" )
	PORT_DIPSETTING(    0x01, "1 KB, download characters" )
	PORT_DIPNAME( 0x02, 0x00, "Paper-out detector" )         PORT_DIPLOCATION("SW:2")
	PORT_DIPSETTING(    0x00, "Enabled" )
	PORT_DIPSETTING(    0x02, "Disabled" )
	PORT_DIPNAME( 0x04, 0x00, "Auto line feed with CR" )     PORT_DIPLOCATION("SW:3")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "IBM character set" )          PORT_DIPLOCATION("SW:4")
	PORT_DIPSETTING(    0x00, "Set 1" )
	PORT_DIPSETTING(    0x08, "Set 2" )
	PORT_DIPNAME( 0x10, 0x10, "Page length" )                PORT_DIPLOCATION("SW:5")
	PORT_DIPSETTING(    0x00, "11\"" )
	PORT_DIPSETTING(    0x10, "12\"" )
	PORT_DIPNAME( 0xe0, 0x00, "International characters" )   PORT_DIPLOCATION("SW:6,7,8")
	PORT_DIPSETTING(    0x00, "USA" )
	PORT_DIPSETTING(    0x20, "France" )
	PORT_DIPSETTING(    0x40, "Germany" )
	PORT_DIPSETTING(    0x60, "England" )
	PORT_DIPSETTING(    0x80, "Denmark" )
	PORT_DIPSETTING(    0xa0, "Sweden" )
	PORT_DIPSETTING(    0xc0, "Italy" )
	PORT_DIPSETTING(    0xe0, "Spain" )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x02, 0x00, "Paper release lever" )
	PORT_CONFSETTING(    0x00, "Tractor (fanfold)" )
	PORT_CONFSETTING(    0x02, "Friction (single sheets)" )
	PORT_CONFNAME( 0x1c, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x04, "ON LINE (short self test)" )
	PORT_CONFSETTING(    0x08, "PAPER FEED (long self test)" )
	PORT_CONFSETTING(    0x0c, "MODE + PAPER FEED (hex dump)" )
	PORT_CONFSETTING(    0x10, "MODE (panel lock)" )
	PORT_CONFSETTING(    0x14, "BOLD (NLQ default)" )
INPUT_PORTS_END

ioport_constructor star_nl10_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( star_nl10 );
}


//**************************************************************************
//  DEVICE
//**************************************************************************

star_nl10_device::star_nl10_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, STAR_NL10, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_speaker(*this, "speaker"),
	m_dip(*this, "DIP"),
	m_panel(*this, "PANEL"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "nl10_online_led"),
	m_bold_led(*this, "nl10_bold_led"),
	m_mode_led(*this, "nl10_mode_led%u", 0U),
	m_data(0),
	m_latch(0),
	m_strobe(true),
	m_data_ready(false),
	m_needles(0),
	m_port2(0xff),
	m_port5(0xff),
	m_port6(0xff),
	m_speaker_level(false),
	m_cr_pos(CR_EDGE - 20),
	m_cr_phase(0),
	m_cr_dir(1),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void star_nl10_device::mem_map(address_map &map)
{
	map(0x0000, 0x0000).nopw();     // watchdog
	map(0x1000, 0x1000).r(FUNC(star_nl10_device::buzzer_r));
	map(0x2000, 0x2000).rw(FUNC(star_nl10_device::dip_r), FUNC(star_nl10_device::needles_w));
	map(0x2001, 0x2001).r(FUNC(star_nl10_device::panel_r));
	map(0x4000, 0x4000).rw(FUNC(star_nl10_device::cent_data_r), FUNC(star_nl10_device::cent_clear_w));
	map(0x6000, 0x7fff).ram();
	map(0x8000, 0xffff).rom().region("maincpu", 0);
}

void star_nl10_device::device_add_mconfig(machine_config &config)
{
	HD6303Y(config, m_maincpu, 4.9152_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &star_nl10_device::mem_map);
	m_maincpu->in_p2_cb().set(FUNC(star_nl10_device::port2_r));
	m_maincpu->out_p2_cb().set(FUNC(star_nl10_device::port2_w));
	m_maincpu->in_p5_cb().set(FUNC(star_nl10_device::port5_r));
	m_maincpu->out_p5_cb().set(FUNC(star_nl10_device::port5_w));
	m_maincpu->out_p6_cb().set(FUNC(star_nl10_device::port6_w));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());

	config.set_default_layout(layout_star_nl10);

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.5);
}

void star_nl10_device::device_start()
{

	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_data_ready));
	save_item(NAME(m_needles));
	save_item(NAME(m_port2));
	save_item(NAME(m_port5));
	save_item(NAME(m_port6));
	save_item(NAME(m_speaker_level));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void star_nl10_device::device_reset()
{
	m_data_ready = false;
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_power_on = machine().time();
	m_power_cycled = false;
	update_busy();
	output_fault(1);
	output_ack(1);
}

INPUT_CHANGED_MEMBER(star_nl10_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
		m_data_ready = false;
		m_power_on = machine().time();
		m_power_cycled = true;
		update_busy();
	}
}


//-------------------------------------------------
//  board I/O
//-------------------------------------------------

u8 star_nl10_device::buzzer_r()
{
	if (!machine().side_effects_disabled())
	{
		m_speaker_level = !m_speaker_level;
		m_speaker->level_w(m_speaker_level);
	}
	return 0xff;
}

u8 star_nl10_device::dip_r()
{
	return m_dip->read();
}

u8 star_nl10_device::panel_r()
{
	u32 const cfg = m_config->read();
	u8 data = m_panel->read() & 0x2f;

	// levers: tractor = both switches 1, friction = both 0
	if (!BIT(cfg, 1))
		data |= 0xc0;

	// keys held while the printer is switched on with its power switch
	// (the firmware looks after homing the carriage)
	static u8 const held[8] = { 0x00, 0x01, 0x02, 0x0a, 0x08, 0x04, 0x00, 0x00 };
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(6)))
		data &= ~held[(cfg >> 2) & 7];
	return data;
}

void star_nl10_device::needles_w(u8 data)
{
	m_needles = data;
}

u8 star_nl10_device::cent_data_r()
{
	return m_latch;
}

void star_nl10_device::cent_clear_w(u8 data)
{
	m_data_ready = false;
	update_busy();
}

void star_nl10_device::input_strobe(int state)
{
	// the cartridge latches the data on the falling edge of /STROBE
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_data_ready = true;
		update_busy();
	}
	m_strobe = state;
}

void star_nl10_device::update_busy()
{
	output_busy(m_data_ready || BIT(m_port5, 5));
	output_select(!BIT(m_port5, 5));
	output_perror(!BIT(m_port2, 2));
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

u8 star_nl10_device::port2_r()
{
	u8 data = 0xff;
	if (m_data_ready)
		data &= ~0x01;
	if (m_cr_pos <= CR_EDGE)
		data &= ~0x40;      // home sensor
	return data;
}

void star_nl10_device::port2_w(u8 data)
{
	u8 const old = m_port2;
	m_port2 = data;

	// needle strobe (Tout2) falling edge fires the needles
	if (BIT(old, 5) && !BIT(data, 5))
		fire(m_needles);

	m_online_led = !BIT(data, 4);
	if ((old ^ data) & 0x04)
		update_busy();
}

u8 star_nl10_device::port5_r()
{
	return BIT(m_config->read(), 0) ? 0xff : 0xfb;  // P52: paper sensor
}

void star_nl10_device::port5_w(u8 data)
{
	u8 const old = m_port5;
	m_port5 = data;

	if (BIT(old, 1) != BIT(data, 1))
		output_ack(BIT(data, 1));

	m_bold_led = !BIT(data, 4);
	for (int i = 0; i < 4; i++)
		m_mode_led[i] = (((data >> 6) & 3) == (3 - i)) ? 1 : 0;   // 0 draft 80, 1 elite 96, 2 condensed 136, 3 NLQ
	if (BIT(old, 5) != BIT(data, 5))
		update_busy();
}

void star_nl10_device::port6_w(u8 data)
{
	u8 const old = m_port6;
	m_port6 = data;

	// carriage stepper: Gray code on P61:P60
	if ((old ^ data) & 0x03)
	{
		static int const gray[4] = { 2, 3, 1, 0 };  // P61:P60 = 00 01 10 11
		int const phase = gray[data & 3];
		int const delta = (phase - m_cr_phase) & 3;
		m_cr_phase = phase;
		if (delta == 1 || delta == 3)
		{
			int const dir = (delta == 1) ? -1 : 1;
			attotime const now = machine().time();
			attotime const since = now - m_cr_last_step;
			m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
			m_cr_last_step = now;
			m_cr_dir = dir;
			m_cr_pos = std::clamp(m_cr_pos + dir, 0, CR_RIGHT);
			m_bitmap_printer->setheadpos(int((m_cr_pos - CR_EDGE) / 60.0 * HDPI + LEFT_OFFSET * HDPI), m_bitmap_printer->m_ypos);
		}
	}

	// paper stepper: half steps on P67..P64
	if ((old ^ data) & 0xf0)
	{
		int idx = -1;
		for (int i = 0; i < 8; i++)
			if (pf_sequence[i] == (data >> 4))
				idx = i;
		if (idx >= 0)
		{
			if (m_pf_phase >= 0)
			{
				int const delta = (idx - m_pf_phase) & 7;
				if (delta == 1)
					m_bitmap_printer->m_ypos++;
				else if (delta == 7)
					m_bitmap_printer->m_ypos = std::max(0, m_bitmap_printer->m_ypos - 1);
				m_bitmap_printer->check_new_page();
			}
			m_pf_phase = idx;
		}
	}
}


//-------------------------------------------------
//  printing
//-------------------------------------------------

double star_nl10_device::head_x() const
{
	// a step starts the move to the new position, which is reached at the
	// next step when the carriage runs at constant speed
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - m_cr_dir * (1.0 - frac);
}

void star_nl10_device::fire(u8 pins)
{
	bool const pin9 = !BIT(m_port5, 0);
	if (!pins && !pin9)
		return;

	int const x = int(std::lround((head_x() - CR_EDGE) / 60.0 * HDPI + LEFT_OFFSET * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	int const y = m_bitmap_printer->m_ypos;
	for (int i = 0; i < 8; i++)
		if (BIT(pins, 7 - i))
			m_bitmap_printer->draw_pixel(x, y + i * (VDPI / 72), 0x000000);
	if (pin9)
		m_bitmap_printer->draw_pixel(x, y + 8 * (VDPI / 72), 0x000000);
}
