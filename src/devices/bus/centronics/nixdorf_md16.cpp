// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Nixdorf MD 16 portable ink-jet printer (about 1988, made by Chinon)

    Board: NEC uPD78C10G CPU, Chinon M70H010 gate array, Chinon
    TC53257P-K900 32 KiB mask ROM, Sony CXK5864M 8 KiB RAM, Sanyo LB1233,
    2x Toshiba TD62308AP stepper drivers, JRC 2903 comparator, 8 DIP
    switches read through resistor ladders on the A/D inputs.
    Print head with 12 nozzles at 1/96" pitch; NLQ is printed in two
    passes, the second one 1/192" lower.

    Memory map (from the firmware):
      0000-7fff  ROM
      8000       gate array: write = stepper phases (bits 3-0, active low,
                 shared by both motors), Paper Empty LED (bit 4) and
                 On Line LED (bit 5), both active low;
                 read = Centronics data latch
      c000-dfff  RAM

    Ports:
      PA5-0, PB5-0  nozzles (one bit per nozzle, fired in six sub-phases)
      PA6           carriage motor driver enable
      PA7           paper motor driver enable
      PB6           FORM FEED key        PC3  ON LINE key      PC6  LINE FEED key
      PC0           /ACK pulse           PC1  1 = paper present
      PC2           pulse when going offline (resets the interface logic?)
      PC4/PC5       1 = offline          PC7  error indicator
      INT1          Centronics STROBE (polled)
      AN0           carriage home sensor (high in the home zone)
      AN1, AN4-7    DIP switches, two per input
      AN2           paper sensor (high = no paper)
      AN3           supply / head supervision

    The carriage is a stepper (1 full step = 1/40", 4 nozzle columns);
    columns are timed by the extended timer, so the head position is
    interpolated between steps.

    The crystal is not visible on the photos; 12 MHz gives plausible speeds.

    Power-on keys: FORM FEED = demonstration print, LINE FEED = banner
    mode, ON LINE with DIP switch setting 0x66 or 0x99 = factory test.

***************************************************************************/

#include "emu.h"
#include "nixdorf_md16.h"

#include "nixdorf_md16.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 160;               // nozzle column grid
constexpr int VDPI = 384;               // paper half step
constexpr double PAPER_WIDTH = 9.5;
constexpr double PAPER_HEIGHT = 12.0;
constexpr double LEFT_OFFSET = -0.5;   // paper edge to the carriage's left stop
constexpr double CR_LAG = -0.73;         // mechanical lag of the head behind the rotor, in steps

constexpr int CR_EDGE = 70;             // home sensor edge in steps from the left stop
constexpr int CR_RIGHT = CR_EDGE + 460;

const u8 cr_sequence[4] = { 0x0c, 0x06, 0x03, 0x09 };
const u8 pf_sequence[8] = { 0x0c, 0x0d, 0x09, 0x0b, 0x03, 0x07, 0x06, 0x0e };

} // anonymous namespace


DEFINE_DEVICE_TYPE(NIXDORF_MD16, nixdorf_md16_device, "md16", "Nixdorf MD 16")


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( nixdorf_md16 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("tc53257p-k900.ic8", 0x0000, 0x8000, CRC(df7c7ccf) SHA1(83b7b4b42c7fc2ff95f62ae1d09a783d87a9d6c4))
ROM_END

const tiny_rom_entry *nixdorf_md16_device::device_rom_region() const
{
	return ROM_NAME( nixdorf_md16 );
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( nixdorf_md16 )
	PORT_START("KEYS")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MD 16 ON LINE")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MD 16 LINE FEED")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MD 16 FORM FEED")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MD 16 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(nixdorf_md16_device::power_switch), 0)

	// as seen by the firmware: 1 = function selected
	PORT_START("DIP")
	PORT_DIPNAME( 0x0001, 0x0000, "Auto line feed with CR" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0001, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "IBM character set" )
	PORT_DIPSETTING(      0x0000, "Set 1" )
	PORT_DIPSETTING(      0x0002, "Set 2" )
	PORT_DIPNAME( 0x0004, 0x0000, "Skip over perforation" )
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0004, "1\"" )
	PORT_DIPNAME( 0x0008, 0x0008, "Page length" )
	PORT_DIPSETTING(      0x0000, "11\"" )
	PORT_DIPSETTING(      0x0008, "12\"" )
	PORT_DIPNAME( 0x0010, 0x0000, "Print quality at power on" )
	PORT_DIPSETTING(      0x0000, "Draft" )
	PORT_DIPSETTING(      0x0010, "NLQ" )
	PORT_DIPNAME( 0x0020, 0x0000, "Emulation" )
	PORT_DIPSETTING(      0x0000, "Epson" )
	PORT_DIPSETTING(      0x0020, "IBM" )
	PORT_DIPNAME( 0x10c0, 0x0000, "International characters" )
	PORT_DIPSETTING(      0x0000, "USA" )
	PORT_DIPSETTING(      0x0040, "France" )
	PORT_DIPSETTING(      0x0080, "Germany" )
	PORT_DIPSETTING(      0x00c0, "England" )
	PORT_DIPSETTING(      0x1000, "Denmark" )
	PORT_DIPSETTING(      0x1040, "Sweden" )
	PORT_DIPSETTING(      0x1080, "Italy" )
	PORT_DIPSETTING(      0x10c0, "Spain" )
	PORT_DIPNAME( 0x2000, 0x0000, "Home calibration (AN1 upper)" )
	PORT_DIPSETTING(      0x0000, "Normal" )
	PORT_DIPSETTING(      0x2000, "Alternative" )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x02, "FORM FEED (demonstration)" )
	PORT_CONFSETTING(    0x04, "LINE FEED (banner mode)" )
	PORT_CONFSETTING(    0x06, "ON LINE (factory test)" )
INPUT_PORTS_END

ioport_constructor nixdorf_md16_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( nixdorf_md16 );
}


//**************************************************************************
//  DEVICE
//**************************************************************************

nixdorf_md16_device::nixdorf_md16_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NIXDORF_MD16, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_keys(*this, "KEYS"),
	m_dip(*this, "DIP"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "md16_online_led"),
	m_paper_led(*this, "md16_paper_led"),
	m_error_led(*this, "md16_error_led"),
	m_data(0),
	m_latch(0),
	m_strobe(true),
	m_busy(false),
	m_ga(0x3f),
	m_pa(0),
	m_pc(0xff),
	m_cr_pos(CR_EDGE - 42),
	m_cr_phase(-1),
	m_cr_dir(1),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void nixdorf_md16_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0x8000).mirror(0x3fff).rw(FUNC(nixdorf_md16_device::ga_r), FUNC(nixdorf_md16_device::ga_w));
	map(0xc000, 0xdfff).ram();
}

void nixdorf_md16_device::device_add_mconfig(machine_config &config)
{
	UPD78C10(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &nixdorf_md16_device::mem_map);
	m_maincpu->pa_out_cb().set(FUNC(nixdorf_md16_device::pa_w));
	m_maincpu->pb_in_cb().set(FUNC(nixdorf_md16_device::pb_r));
	m_maincpu->pb_out_cb().set(FUNC(nixdorf_md16_device::pb_w));
	m_maincpu->pc_in_cb().set(FUNC(nixdorf_md16_device::pc_r));
	m_maincpu->pc_out_cb().set(FUNC(nixdorf_md16_device::pc_w));
	m_maincpu->an0_func().set(FUNC(nixdorf_md16_device::an0_r));
	m_maincpu->an1_func().set(FUNC(nixdorf_md16_device::an1_r));
	m_maincpu->an2_func().set(FUNC(nixdorf_md16_device::an2_r));
	m_maincpu->an3_func().set(FUNC(nixdorf_md16_device::an3_r));
	m_maincpu->an4_func().set(FUNC(nixdorf_md16_device::dip_r<4>));
	m_maincpu->an5_func().set(FUNC(nixdorf_md16_device::dip_r<5>));
	m_maincpu->an6_func().set(FUNC(nixdorf_md16_device::dip_r<6>));
	m_maincpu->an7_func().set(FUNC(nixdorf_md16_device::dip_r<7>));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::inkjet(0.0118));
	m_bitmap_printer->set_paper_view(3.0);

	config.set_default_layout(layout_nixdorf_md16);
}

void nixdorf_md16_device::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_busy));
	save_item(NAME(m_ga));
	save_item(NAME(m_pa));
	save_item(NAME(m_pc));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void nixdorf_md16_device::device_reset()
{
	m_busy = false;
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_power_on = machine().time();
	m_power_cycled = false;
	output_fault(1);
	output_ack(1);
	update_status();
}

INPUT_CHANGED_MEMBER(nixdorf_md16_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
		m_busy = false;
		m_power_on = machine().time();
		m_power_cycled = true;
		update_status();
	}
}

bool nixdorf_md16_device::keys_held(u8 mask) const
{
	static u8 const held[4] = { 0x00, 0x04, 0x02, 0x01 };
	if (m_keys->read() & mask)
		return true;
	// keys held while the printer is switched on with its power switch
	return m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)) && (held[(m_config->read() >> 1) & 3] & mask);
}


//-------------------------------------------------
//  gate array
//-------------------------------------------------

u8 nixdorf_md16_device::ga_r()
{
	return m_latch;
}

void nixdorf_md16_device::ga_w(u8 data)
{
	m_ga = data;
	m_paper_led = !BIT(data, 4);
	m_online_led = !BIT(data, 5);
	motor_phase(data & 0x0f);
}

void nixdorf_md16_device::input_strobe(int state)
{
	// the gate array latches the data and requests INT1 on /STROBE
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_busy = true;
		update_status();
		m_maincpu->set_input_line(UPD7810_INTF1, ASSERT_LINE);
	}
	else if (!m_strobe && state)
		m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	m_strobe = state;
}

void nixdorf_md16_device::update_status()
{
	output_busy(m_busy || BIT(m_pc, 4));
	output_select(!BIT(m_pc, 5));
	output_perror(!BIT(m_pc, 1));
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

void nixdorf_md16_device::pa_w(u8 data)
{
	u8 const old = m_pa;
	m_pa = data;
	fire(0, data & 0x3f);

	// a driver that is switched on pulls its motor to the current phase
	if ((data & ~old) & 0xc0)
		motor_phase(m_ga & 0x0f);
}

u8 nixdorf_md16_device::pb_r()
{
	return keys_held(0x04) ? 0xbf : 0xff;      // PB6: FORM FEED
}

void nixdorf_md16_device::pb_w(u8 data)
{
	fire(1, data & 0x3f);
}

u8 nixdorf_md16_device::pc_r()
{
	u8 data = 0xff;
	if (keys_held(0x01))
		data &= ~0x08;      // PC3: ON LINE
	if (keys_held(0x02))
		data &= ~0x40;      // PC6: LINE FEED
	return data;
}

void nixdorf_md16_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	// /ACK pulse also releases BUSY
	if (BIT(old, 0) != BIT(data, 0))
	{
		output_ack(BIT(data, 0));
		if (!BIT(data, 0))
			m_busy = false;
	}
	if (BIT(old, 2) && !BIT(data, 2))
		m_busy = false;

	m_error_led = !BIT(data, 7);
	update_status();
}

u8 nixdorf_md16_device::an0_r()
{
	return (m_cr_pos <= CR_EDGE) ? 0xe0 : 0x20;     // home sensor
}

u8 nixdorf_md16_device::an2_r()
{
	return BIT(m_config->read(), 0) ? 0x20 : 0xe0;  // paper sensor
}

u8 nixdorf_md16_device::an3_r()
{
	return 0x60;    // supply / head supervision: normal
}

// two switches per input through a resistor ladder
static u8 ladder(bool upper, bool lower)
{
	return (upper && lower) ? 0x30 : upper ? 0x90 : lower ? 0x5a : 0xff;
}

u8 nixdorf_md16_device::an1_r()
{
	u32 const dip = m_dip->read();
	return ladder(BIT(dip, 12), BIT(dip, 13));
}

template <int N> u8 nixdorf_md16_device::dip_r()
{
	u32 const dip = m_dip->read();
	int const bit = 7 - (N - 4) * 2;
	return ladder(BIT(dip, bit - 1), BIT(dip, bit));
}


//-------------------------------------------------
//  mechanics
//-------------------------------------------------

void nixdorf_md16_device::motor_phase(u8 phases)
{
	if (BIT(m_pa, 7))
	{
		// paper motor, half steps of 1/384"
		for (int i = 0; i < 8; i++)
		{
			if (pf_sequence[i] != phases)
				continue;
			if (m_pf_phase >= 0)
			{
				// the firmware uses full steps (two entries) for normal feeds
				int const delta = (i - m_pf_phase) & 7;
				if (delta == 1 || delta == 2)
					m_bitmap_printer->m_ypos += delta;
				else if (delta == 6 || delta == 7)
					m_bitmap_printer->m_ypos = std::max(0, m_bitmap_printer->m_ypos - (8 - delta));
				m_bitmap_printer->check_new_page();
			}
			m_pf_phase = i;
		}
	}
	else
	{
		// carriage motor, full steps of 1/40"
		for (int i = 0; i < 4; i++)
		{
			if (cr_sequence[i] != phases)
				continue;
			if (m_cr_phase >= 0)
			{
				int const delta = (i - m_cr_phase) & 3;
				if (delta == 1 || delta == 3)
				{
					int const dir = (delta == 1) ? 1 : -1;
					attotime const now = machine().time();
					attotime const since = now - m_cr_last_step;
					m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
					m_cr_last_step = now;
					m_cr_dir = dir;
					m_cr_pos = std::clamp(m_cr_pos + dir, 0, CR_RIGHT);
					m_bitmap_printer->setheadpos(int(m_cr_pos / 40.0 * HDPI + LEFT_OFFSET * HDPI), m_bitmap_printer->m_ypos);
				}
			}
			m_cr_phase = i;
		}
	}
}

double nixdorf_md16_device::head_x() const
{
	// a step starts the move to the new position, which is reached at the next step
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - m_cr_dir * (1.0 - frac + CR_LAG);
}

void nixdorf_md16_device::fire(int port, u8 data)
{
	// PB5..0 = rows 0,6,1,7,2,8; PA5..0 = rows 3,9,4,10,5,11
	static int const rows[2][6] = { { 11, 5, 10, 4, 9, 3 }, { 8, 2, 7, 1, 6, 0 } };
	if (!data)
		return;

	int const x = int(std::lround(head_x() / 40.0 * HDPI + LEFT_OFFSET * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	int const y = m_bitmap_printer->m_ypos;
	for (int b = 0; b < 6; b++)
		if (BIT(data, b))
			m_bitmap_printer->draw_pixel(x, y + rows[port][b] * (VDPI / 96), 0x000000);
}
