// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Citizen 200GX 9-pin dot matrix printer

    CPU uPD7810, 32 KiB program ROM (W000-030, June 1990), 32 KiB character
    ROM (W100-010) seen in 8 KiB banks, 16 KiB RAM, Citizen gate array.
    Everything below is derived from the firmware.

      0000-7fff     program ROM
      8000-9fff     character ROM, bank selected by PB0-PB1
      a000-dfff     RAM
      e000-e00f     gate array

    Gate array:
      e000 w        needles 1-8 (bit 0 = top), r status: bit 2 = data byte latched
      e00a w        needle 9 in bit 0; the write fires the head
      e00b/e00c w   needle pulse width (negative count; e00b ed for printing)
      e001 w        bits 0-3 carriage motor, bits 4-7 paper feed motor phases
                    (active low, half-step tables at 0081 and 3d81)
      e002 w        bits 0-3 phases of a third motor stepped only at power on,
                    bit 5 BUSY, bit 6 /ACK pulse, bit 7 (?)
      e003 w        bit 6 set: bits 0-3 lamps beside the keys, keys read on PA4-PA7;
                    bit 6 clear: menu switch read on PA5-PA6
      e004 r        Centronics data latch
      e006 w        carriage motor current (?), e007 w baud rate divider
      e009 w        releases the data latch

    Ports:
      PA0 in        home sensor (0 = home)
      PA1 in        paper end sensor (1 = no paper)
      PA2 in        switches of the optional serial interface, shifted in with PB0/PB1
      PA3 in        /AUTO FEED XT, inverted: line feed after CR
      PA4-PA7 in    QUIET, ON LINE, PARK/LOAD, LF/FF keys (0 = pressed); menu switch on PA5-PA6
      PB0-PB1 out   character ROM bank; PB2 selects the DIP switch row; PB4-PB6 carriage motor (?)
      PC5 in        second paper sensor (1 = no paper)
      PC7 in        (?) held high
      AN0-AN5       DIP switches SW1 and SW2, two per input through a resistor network
      AN6           head temperature (?)
      TI            timing clock for the needle (timer 0) and carriage (timer 1) interrupts

    Carriage: 1/60" full steps, the head fires twice per step in draft (120 dpi).
    Paper: 1/432" half steps, a line feed of n/216" takes 2n half steps.

    TODO:
    - /INIT, the third motor (paper path?), PC7, serial interface option
    - keys held at power on (self test, hex dump)
    - crystal and TI clock are guesses that give the specified 160 cps

**********************************************************************/

#include "emu.h"
#include "citizen_200gx.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"

#include "citizen_200gx.lh"

#define LOG_GA    (1U << 1)
#define LOG_PORT  (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_GA | LOG_PORT)
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;
constexpr int VDPI = 216;
constexpr double PAPER_WIDTH = 9.5;
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.25;    // paper edge to the home position, inches (guess)

class citizen_200gx_device : public device_t, public device_centronics_peripheral_interface
{
public:
	citizen_200gx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_strobe(int state) override;
	virtual void input_init(int state) override { m_init = state; }
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

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);
	u8 cg_r(offs_t offset);

	u8 pa_r();
	void pa_w(u8 data);
	u8 pb_r();
	void pb_w(u8 data);
	u8 pc_r();
	void pc_w(u8 data);

	required_device<upd7810_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_region_ptr<u8> m_cg;
	required_ioport m_panel;
	required_ioport m_sw1;
	required_ioport m_sw2;
	required_ioport m_config;
	output_finder<4> m_key_led;
	output_finder<> m_paper_led;

	u8 m_data;
	u8 m_latch;
	bool m_full;
	int m_strobe;
	int m_init;
	int m_autofd;
	int m_slctin;
	u8 m_ga[16];
	u8 m_pa, m_pb, m_pc;

	// carriage and paper motors, in half steps
	int m_cr_angle;
	int m_cr_half;
	int m_cr_from;          // position at the previous step
	attotime m_cr_step_time;
	attotime m_cr_period;
	int m_pf_angle;
	int m_pf_half;

	void cr_motor_w(u8 data);
	void pf_motor_w(u8 data);
	void update_busy();
	void fire();
	double head_x() const;
	bool paper() const { return BIT(m_config->read(), 2) && m_bitmap_printer->paper_present(); }
	u8 dip_pair(int first, int second) const;
	u8 an_r(int channel);
};


ROM_START( c200gx )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("w000-030.bin", 0x0000, 0x8000, CRC(4a0c80ea) SHA1(ba5f27f7001f9d37b3fad786493b71893430e9c3))

	ROM_REGION(0x8000, "cg", 0)
	ROM_LOAD("w100-010.bin", 0x0000, 0x8000, CRC(0fe70957) SHA1(184141f747cc57c9f381aefad22ecd863d4be7f9))
ROM_END

const tiny_rom_entry *citizen_200gx_device::device_rom_region() const
{
	return ROM_NAME( c200gx );
}


INPUT_PORTS_START( c200gx )
	PORT_START("PANEL")     // read on PA4-PA7 while e003 bit 6 is set (0 = pressed)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("200GX QUIET")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("200GX ON LINE")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("200GX PARK/LOAD")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("200GX LF/FF")

	PORT_START("SW1")
	PORT_DIPNAME( 0x01, 0x00, "Emulation" )                     PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, "Epson" )
	PORT_DIPSETTING(    0x01, "IBM" )
	PORT_DIPNAME( 0x02, 0x00, "Character set (Epson) / set (IBM)" ) PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "Italic / Set 1" )
	PORT_DIPSETTING(    0x02, "Graphics / Set 2" )
	PORT_DIPNAME( 0x04, 0x00, "Zero" )                          PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, "Unslashed" )
	PORT_DIPSETTING(    0x04, "Slashed" )
	PORT_DIPNAME( 0x08, 0x00, "Auto line feed" )                PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "Skip perforation (Epson) / auto CR (IBM)" ) PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0xe0, 0x00, "International set (Epson) / code page (IBM)" ) PORT_DIPLOCATION("SW1:6,7,8")
	PORT_DIPSETTING(    0x00, "USA / 437" )
	PORT_DIPSETTING(    0x20, "France / 850" )
	PORT_DIPSETTING(    0x40, "Germany / 860" )
	PORT_DIPSETTING(    0x60, "England / 863" )
	PORT_DIPSETTING(    0x80, "Denmark I / 865" )
	PORT_DIPSETTING(    0xa0, "Sweden / Scandinavian" )
	PORT_DIPSETTING(    0xc0, "Italy" )
	PORT_DIPSETTING(    0xe0, "Spain" )

	PORT_START("SW2")
	PORT_DIPNAME( 0x03, 0x00, "Default font" )                  PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x00, "Draft" )
	PORT_DIPSETTING(    0x01, "Roman" )
	PORT_DIPSETTING(    0x02, "Sans Serif" )
	PORT_DIPSETTING(    0x03, "Courier" )
	PORT_DIPNAME( 0x0c, 0x00, "Form length" )                   PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(    0x00, "11\"" )
	PORT_DIPSETTING(    0x04, "12\"" )
	PORT_DIPSETTING(    0x08, "11 2/3\"" )
	PORT_DIPSETTING(    0x0c, "8\"" )
	PORT_DIPNAME( 0x10, 0x00, "Top of form" )                   PORT_DIPLOCATION("SW2:5")
	PORT_DIPSETTING(    0x00, "1\"" )
	PORT_DIPSETTING(    0x10, "9/16\"" )
	PORT_DIPNAME( 0x20, 0x00, "Automatic sheet feeder" )        PORT_DIPLOCATION("SW2:6")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, "Printing direction" )            PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(    0x00, "Unidirectional" )
	PORT_DIPSETTING(    0x40, "Bidirectional" )
	PORT_DIPNAME( 0x80, 0x00, "Colour printing" )               PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x04, 0x04, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x04, "Loaded" )
	PORT_CONFNAME( 0x03, 0x00, "Command-Vue II menu switch" )    // PA5, PA6 while e003 bit 6 is clear
	PORT_CONFSETTING(    0x02, "INSTALL" )
	PORT_CONFSETTING(    0x03, "FONT" )
	PORT_CONFSETTING(    0x01, "PITCH" )
	PORT_CONFSETTING(    0x00, "ON LINE" )
INPUT_PORTS_END

ioport_constructor citizen_200gx_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( c200gx );
}


citizen_200gx_device::citizen_200gx_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CITIZEN_200GX, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_cg(*this, "cg"),
	m_panel(*this, "PANEL"),
	m_sw1(*this, "SW1"),
	m_sw2(*this, "SW2"),
	m_config(*this, "CONFIG"),
	m_key_led(*this, "c200gx_key_led%u", 0U),
	m_paper_led(*this, "c200gx_paper_led"),
	m_data(0),
	m_latch(0),
	m_full(false),
	m_strobe(1),
	m_init(1),
	m_autofd(1),
	m_slctin(1),
	m_pa(0xff), m_pb(0), m_pc(0xff),
	m_cr_angle(-1), m_cr_half(100), m_cr_from(100),
	m_pf_angle(-1), m_pf_half(0)
{
}

void citizen_200gx_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0x9fff).r(FUNC(citizen_200gx_device::cg_r));
	map(0xa000, 0xdfff).ram();
	map(0xe000, 0xe00f).mirror(0x0ff0).rw(FUNC(citizen_200gx_device::ga_r), FUNC(citizen_200gx_device::ga_w));
}

void citizen_200gx_device::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 11.0592_MHz_XTAL);   // crystal not known
	m_maincpu->set_addrmap(AS_PROGRAM, &citizen_200gx_device::mem_map);
	// timing clock for the head and the carriage at TI (guess: crystal / 192 gives the specified
	// 160 cps at 10 cpi with the firmware's cruise period of 60 counts per 1/60" step)
	m_maincpu->set_ti_clock(11.0592_MHz_XTAL / 192);
	m_maincpu->pa_in_cb().set(FUNC(citizen_200gx_device::pa_r));
	m_maincpu->pa_out_cb().set(FUNC(citizen_200gx_device::pa_w));
	m_maincpu->pb_in_cb().set(FUNC(citizen_200gx_device::pb_r));
	m_maincpu->pb_out_cb().set(FUNC(citizen_200gx_device::pb_w));
	m_maincpu->pc_in_cb().set(FUNC(citizen_200gx_device::pc_r));
	m_maincpu->pc_out_cb().set(FUNC(citizen_200gx_device::pc_w));
	m_maincpu->an0_func().set([this] () { return an_r(0); });
	m_maincpu->an1_func().set([this] () { return an_r(1); });
	m_maincpu->an2_func().set([this] () { return an_r(2); });
	m_maincpu->an3_func().set([this] () { return an_r(3); });
	m_maincpu->an4_func().set([this] () { return an_r(4); });
	m_maincpu->an5_func().set([this] () { return an_r(5); });
	m_maincpu->an6_func().set_constant(0x80);   // head temperature (?)
	m_maincpu->an7_func().set_constant(0x80);

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x03, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_citizen_200gx);
}

void citizen_200gx_device::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_full));
	save_item(NAME(m_strobe));
	save_item(NAME(m_init));
	save_item(NAME(m_autofd));
	save_item(NAME(m_slctin));
	save_item(NAME(m_ga));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_cr_angle));
	save_item(NAME(m_cr_half));
	save_item(NAME(m_cr_from));
	save_item(NAME(m_cr_step_time));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_angle));
	save_item(NAME(m_pf_half));
}

void citizen_200gx_device::device_reset()
{
	m_cr_period = attotime::never;
	m_cr_step_time = machine().time();
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
}

void citizen_200gx_device::input_strobe(int state)
{
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_full = true;
		update_busy();
		LOG("strobe %02x\n", m_latch);
	}
	m_strobe = state;
}

u8 citizen_200gx_device::cg_r(offs_t offset)
{
	return m_cg[((m_pb & 0x03) << 13) | offset];
}

u8 citizen_200gx_device::ga_r(offs_t offset)
{
	u8 data = 0xff;
	switch (offset)
	{
	case 0:
		data = 0xfa | (m_full ? 0x04 : 0x00);   // bit 2: data latched
		break;
	case 4:
		data = m_latch;
		break;
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_GA, "%s GA r %x = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void citizen_200gx_device::ga_w(offs_t offset, u8 data)
{
	if (offset != 0 && offset != 10)
		LOGMASKED(LOG_GA, "%s GA w %x = %02x\n", machine().describe_context(), offset, data);
	u8 const old = m_ga[offset];
	m_ga[offset] = data;
	switch (offset)
	{
	case 1:
		cr_motor_w(data & 0x0f);
		pf_motor_w(data >> 4);
		break;
	case 2:
		if ((old ^ data) & 0x40)
			output_ack(BIT(data, 6) ? 0 : 1);
		update_busy();
		break;
	case 3:
		// bit 6 set: bits 0-3 light the lamps beside the keys (ON LINE, PARK/LOAD, LF/FF, QUIET)
		// and the keys are read; bit 6 clear: the menu switch is read, the lamps are dark
		if (BIT(data, 6))
			for (int i = 0; i < 4; i++)
				m_key_led[i] = BIT(data, i);
		break;
	case 9:
		m_full = false;
		update_busy();
		break;
	case 10:
		fire();
		break;
	}
}

double citizen_200gx_device::head_x() const
{
	// carriage position in half steps (1/120"), interpolated within the current step period
	if (m_cr_period.is_never())
		return m_cr_half;
	double const frac = std::min(1.0, (machine().time() - m_cr_step_time).as_double() / m_cr_period.as_double());
	return m_cr_from + (m_cr_half - m_cr_from) * frac;
}

void citizen_200gx_device::update_busy()
{
	output_busy((m_full || BIT(m_ga[2], 5)) ? 1 : 0);
}

void citizen_200gx_device::fire()
{
	u16 const head = m_ga[0] | (m_ga[10] << 8);
	// e00b: needle pulse width as a negative count; printing uses 19, the warm-up of a cold
	// head at power on 6, too short to strike the ribbon (threshold is a guess)
	if ((0x100 - m_ga[11]) < 10)
		return;
	LOG("fire %04x cr %d y %d t %s\n", head, m_cr_half, m_bitmap_printer->m_ypos, machine().time().as_string(6));
	int const x = int(std::lround((head_x() / 120.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;
	for (int needle = 0; needle < 9; needle++)
		if (BIT(head, needle))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + needle * (VDPI / 72), 0x000000);
}

namespace {

// electrical angle in half steps (0-7) of the energised windings (active low), -1 if none;
// the carriage half-step table (0081) energises bits 0, 3, 1, 2 in turn
int cr_angle(u8 phases)
{
	static int const table[16] = {
		-1, -1, -1, -1, -1,  2,  0,  1,
		-1,  4,  6,  5, -1,  3,  7, -1 };
	return table[phases & 0x0f];
}

int angle_delta(int from, int to)
{
	return ((to - from + 4) & 7) - 4;
}

} // anonymous namespace

void citizen_200gx_device::cr_motor_w(u8 data)
{
	LOGMASKED(LOG_GA, "CR %x half %d t %s\n", data, m_cr_half, machine().time().as_string(6));
	int const angle = cr_angle(data);
	if (angle < 0)
		return;
	if (m_cr_angle >= 0)
	{
		int const delta = angle_delta(m_cr_angle, angle);
		if (delta)
		{
			// the carriage runs from the previous rotor position to the new one during a step period
			attotime const now = machine().time();
			attotime const since = now - m_cr_step_time;
			m_cr_period = (since < attotime::from_msec(10)) ? since : attotime::never;
			m_cr_step_time = now;
			m_cr_from = m_cr_half;
			m_cr_half += delta;
		}
	}
	m_cr_angle = angle;
}

void citizen_200gx_device::pf_motor_w(u8 data)
{
	int const angle = cr_angle(data);
	if (angle < 0)
		return;
	if (m_pf_angle >= 0)
	{
		int const old = m_pf_half >> 1;
		m_pf_half += angle_delta(m_pf_angle, angle);
		int const now = m_pf_half >> 1;
		if (now > old)
			m_bitmap_printer->m_ypos++;
		else if (now < old)
			m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
		m_bitmap_printer->check_new_page();
	}
	m_pf_angle = angle;
}

u8 citizen_200gx_device::dip_pair(int first, int second) const
{
	// two DIP switches on one A/D input through a resistor network; the firmware's thresholds are
	// b8, 94 and 71: above b8 both off, 94-b7 the second on, 72-93 the first on, up to 71 both on
	static u8 const level[4] = { 0xf0, 0x80, 0xa5, 0x50 };
	return level[(first ? 1 : 0) | (second ? 2 : 0)];
}

u8 citizen_200gx_device::an_r(int channel)
{
	u8 const sw1 = m_sw1->read();
	u8 const sw2 = m_sw2->read();
	if (BIT(m_pb, 2))
	{
		// PB2 high: SW2-5 to SW2-8 on AN0-AN3, on below 39
		return (channel < 4) ? (BIT(sw2, 4 + channel) ? 0x20 : 0xf0) : 0xf0;
	}
	switch (channel)
	{
	case 0: return dip_pair(BIT(sw1, 0), BIT(sw1, 6));
	case 1: return dip_pair(BIT(sw1, 1), BIT(sw1, 7));
	case 2: return dip_pair(BIT(sw1, 2), BIT(sw2, 0));
	case 3: return dip_pair(BIT(sw1, 3), BIT(sw2, 1));
	case 4: return dip_pair(BIT(sw1, 4), BIT(sw2, 2));
	case 5: return dip_pair(BIT(sw1, 5), BIT(sw2, 3));
	}
	return 0xf0;
}

u8 citizen_200gx_device::pa_r()
{
	u8 data = 0xf4;
	if (!m_autofd)
		data |= 0x08;       // PA3: /AUTO FEED XT (inverted), line feed after CR
	if (m_cr_half > 0)
		data |= 0x01;       // PA0: home sensor, low at home
	if (!paper())
		data |= 0x02;       // PA1: paper end sensor, high without paper
	if (!machine().side_effects_disabled())
		m_paper_led = !paper();     // the PAPER OUT lamp follows the sensor
	if (BIT(m_ga[3], 6))
		data = (data & 0x0f) | (m_panel->read() & 0xf0);    // keys
	else
		data = (data & 0x9f) | ((~m_config->read() & 0x03) << 5);    // slide switch
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_PORT, "%s PA r = %02x\n", machine().describe_context(), data);
	return data;
}

void citizen_200gx_device::pa_w(u8 data)
{
	LOGMASKED(LOG_PORT, "%s PA w %02x\n", machine().describe_context(), data);
	m_pa = data;
}

u8 citizen_200gx_device::pb_r()
{
	return m_pb;
}

void citizen_200gx_device::pb_w(u8 data)
{
	if ((data ^ m_pb) & ~0x07)
		LOGMASKED(LOG_PORT, "%s PB w %02x\n", machine().describe_context(), data);
	m_pb = data;
}

u8 citizen_200gx_device::pc_r()
{
	u8 data = 0xdf;
	if (!paper())
		data |= 0x20;       // PC5: second paper sensor, high without paper
	return data;
}

void citizen_200gx_device::pc_w(u8 data)
{
	m_pc = data;
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(CITIZEN_200GX, device_centronics_peripheral_interface, citizen_200gx_device, "c200gx", "Citizen 200GX")
