// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Star Gemini-10X 9-pin dot matrix printer (1983)

    Main board (Sams ComputerFacts CP13):
      IC11  uPD7801G-176 CPU with 4 KiB mask ROM, 4 MHz crystal
      IC16  2764 EPROM (IC12, a 2364 mask ROM, is the alternative)
      IC13  TMM2016 2 KiB RAM
      IC17  M5L8253P-5 timer, clocked from the CPU (2 MHz)
      IC9   74LS42 address decoder (A15-A12)
      IC2, IC5  74LS257 DIP switch / panel multiplexers
      IC19  74LS374 needle latch, IC4 74LS74 strobe flip-flop
      IC23  74LS123 watchdog that disables the needle drivers

    Memory map:
      0000-0fff     internal ROM
      1000-2fff     2764: 1000-1fff program (upper half), 2000-2fff
                    character generator (lower half)
      3000-37ff     RAM (mirrored at 3800)
      4000-4003     8253
      5000          r: Centronics data, w: clear the strobe flip-flop
      6000/6001     r: switches and keys (A0 selects the half),
                    w: needle latch (needles 1-8, 1 = fire)
      7000          w: latch of unknown use (written with small values
                    while the carriage accelerates and brakes)

    Ports:
      PA0-PA2 out   paper motor control (PA2 pulses between the steps)
      PA3-PA4 out   paper stepper, Gray code, 1 step = 1/144"
      PA5 out       8253 GATE0: needle pulse (rising edge fires)
      PA6 out       8253 GATE1: dot column timer
      PA7 out       8253 GATE2
      PB0 out       needle 9, 0 = fire
      PB1-PB2 out   carriage stepper, Gray code, 1 step = 1/120"
      PB3-PB4 out   0 when on line (interface status)
      PB5 out       buzzer
      PB6 in        unused here
      PB7 in        home sensor, 0 = home zone
      PC0 in        serial interface RxD (bit banged, INT1 start bit)
      PC4 out       serial interface handshake
      PC5 out       Centronics /ACK (positive pulse, inverted)
      PC6 out       1 = on line
      PC7 in        interface select: 1 = parallel, 0 = serial board
      INT1          strobe flip-flop (Centronics) / start bit (serial)
      INT2          8253 OUT1 and OUT2 (both one-shots finished)
      SI            serial board identification (all ones = no board)

    Switches at 6000: D0 ON LINE, D1 LF, D2 FF (0 = pressed), D3 paper
    switch (0 = paper out), D4-D7 DSW1-1..4; at 6001: D0 paper out line
    (passed on by DSW2-1, else high), D1-D3 DSW2-2..4, D4-D7 DSW1-5..8.
    Switches read 0 when ON.

    The dot columns are timed by the 8253, the carriage by the CPU timer;
    the firmware fires one column per carriage step in 10 CPI.

    Power-on key: LF = self test (repeats while LF is held).

    TODO:
    - the latch at 7000
    - needle pulse width (8253 OUT0) and the watchdog are not modelled;
      the needles fire on the rising edge of PA5
    - optional serial interface board

***************************************************************************/

#include "emu.h"
#include "star_gem10x.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "machine/pit8253.h"
#include "sound/spkrdev.h"

#include "speaker.h"

#include "star_gem10x.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 144;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 12.0;
constexpr double LEFT_OFFSET = 0.5;     // paper edge to the first print column
constexpr int CR_EDGE = 40;             // home sensor edge, steps from the left stop
constexpr int CR_RIGHT = CR_EDGE + 1060; // right stop (80 columns = 960 steps)
constexpr double CR_SHIFT = 0.5;        // the dots land half a step further in the direction
										// of travel (found from bidirectional printing)


class star_gem10x_device : public device_t, public device_centronics_peripheral_interface
{
public:
	star_gem10x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	virtual void input_strobe(int state) override;
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

	DECLARE_INPUT_CHANGED_MEMBER(power_switch);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 pb_r();
	u8 pc_r();
	void pa_w(u8 data);
	void pb_w(u8 data);
	void pc_w(u8 data);
	u8 cent_r();
	void cent_w(u8 data);
	u8 sel_r(offs_t offset);
	void needles_w(u8 data);
	void latch7_w(u8 data);

	void update_int2();
	void update_outputs();
	bool paper_present() const;
	double head_x() const;
	void fire();

	required_device<upd7801_device> m_maincpu;
	required_device<pit8253_device> m_pit;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<speaker_sound_device> m_speaker;
	required_region_ptr<u8> m_ext;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_panel;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paper_led;

	// Centronics
	u8 m_data;
	u8 m_latch;
	bool m_strobe;
	bool m_strobe_ff;

	u8 m_pa;
	u8 m_pb;
	u8 m_pc;
	u8 m_needles;
	int m_out1;
	int m_out2;

	// carriage (1/120" steps) and paper (1/144" steps)
	int m_cr_pos;
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pf_phase;

	attotime m_power_on;
	bool m_power_cycled;
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( star_gem10x )
	ROM_REGION(0x1000, "maincpu", 0)
	ROM_DEFAULT_BIOS("v1")
	ROM_SYSTEM_BIOS(0, "v1", "Version 1")
	ROMX_LOAD("star_gx10_2732.bin", 0x0000, 0x1000, CRC(463d2ac7) SHA1(248f6c6e2de149cd3027c0310c5dc48454b87211), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "v14", "Version 1.4")
	ROMX_LOAD("star_gx10_2732_open_windowupd7800.bin", 0x0000, 0x1000, CRC(0aeb50a5) SHA1(4ad98032060422a159f162fdc0ae934e6214eb71), ROM_BIOS(1))

	ROM_REGION(0x2000, "ext", 0)
	ROMX_LOAD("star_gx10_2764.bin", 0x0000, 0x2000, CRC(9f7dd771) SHA1(f2abaea6d8fe39253b18c076a134762f81a94441), ROM_BIOS(0))
	ROMX_LOAD("stargx10_14_s.bin", 0x0000, 0x2000, CRC(8dc73dfe) SHA1(68a4b45f6cec024e623e76d42fcfb495116ba57b), ROM_BIOS(1))
ROM_END

const tiny_rom_entry *star_gem10x_device::device_rom_region() const
{
	return ROM_NAME( star_gem10x );
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( star_gem10x )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Gemini-10X ON LINE")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Gemini-10X L.F")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Gemini-10X F.F")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("Gemini-10X Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(star_gem10x_device::power_switch), 0)

	PORT_START("DSW1")      // ON = 0
	PORT_DIPNAME( 0x09, 0x00, "Character set" )          PORT_DIPLOCATION("DSW1:1,4")
	PORT_DIPSETTING(    0x00, "Standard ASCII" )
	PORT_DIPSETTING(    0x08, "Download" )
	PORT_DIPSETTING(    0x01, "Italic ASCII" )
	PORT_DIPNAME( 0x02, 0x00, "Print mode" )             PORT_DIPLOCATION("DSW1:2")
	PORT_DIPSETTING(    0x00, "Normal" )
	PORT_DIPSETTING(    0x02, "Emphasized" )
	PORT_DIPNAME( 0x04, 0x00, "Character pitch" )        PORT_DIPLOCATION("DSW1:3")
	PORT_DIPSETTING(    0x00, "10 CPI" )
	PORT_DIPSETTING(    0x04, "17 CPI" )
	PORT_DIPNAME( 0x10, 0x00, "Line spacing" )           PORT_DIPLOCATION("DSW1:5")
	PORT_DIPSETTING(    0x00, "1/6\"" )
	PORT_DIPSETTING(    0x10, "1/8\"" )
	PORT_DIPNAME( 0xe0, 0x00, "Country / form length" )  PORT_DIPLOCATION("DSW1:6,7,8")
	PORT_DIPSETTING(    0x00, "U.S.A. / 11\"" )
	PORT_DIPSETTING(    0x20, "England / 11\"" )
	PORT_DIPSETTING(    0x40, "Germany / 12\"" )
	PORT_DIPSETTING(    0x60, "Denmark / 12\"" )
	PORT_DIPSETTING(    0x80, "France / 12\"" )
	PORT_DIPSETTING(    0xa0, "Sweden / 12\"" )
	PORT_DIPSETTING(    0xc0, "Italy / 12\"" )
	PORT_DIPSETTING(    0xe0, "Spain / 12\"" )

	PORT_START("DSW2")      // ON = 0
	PORT_DIPNAME( 0x01, 0x00, "Paper-out signal" )       PORT_DIPLOCATION("DSW2:1")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPNAME( 0x02, 0x02, "Buffer full printing" )   PORT_DIPLOCATION("DSW2:2")
	PORT_DIPSETTING(    0x00, "CR ignored until full or LF" )
	PORT_DIPSETTING(    0x02, "Print on CR" )
	PORT_DIPNAME( 0x04, 0x04, "Interface" )              PORT_DIPLOCATION("DSW2:3")
	PORT_DIPSETTING(    0x00, "7 bit" )
	PORT_DIPSETTING(    0x04, "8 bit" )
	PORT_DIPNAME( 0x08, 0x08, "Auto line feed with CR" ) PORT_DIPLOCATION("DSW2:4")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x02, 0x00, "L.F held at power on (self test)" )
	PORT_CONFSETTING(    0x00, DEF_STR( No ) )
	PORT_CONFSETTING(    0x02, DEF_STR( Yes ) )
INPUT_PORTS_END

ioport_constructor star_gem10x_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( star_gem10x );
}


//**************************************************************************
//  DEVICE
//**************************************************************************

star_gem10x_device::star_gem10x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, STAR_GEMINI10X, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_pit(*this, "pit"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_speaker(*this, "speaker"),
	m_ext(*this, "ext"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_panel(*this, "PANEL"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "gem10x_online_led"),
	m_ready_led(*this, "gem10x_ready_led"),
	m_paper_led(*this, "gem10x_paper_led"),
	m_data(0),
	m_latch(0),
	m_strobe(true),
	m_strobe_ff(false),
	m_pa(0xff),
	m_pb(0xff),
	m_pc(0xff),
	m_needles(0),
	m_out1(1),
	m_out2(1),
	m_cr_pos(CR_EDGE + 30),
	m_cr_phase(0),
	m_cr_dir(1),
	m_pf_phase(0),
	m_power_cycled(false)
{
}

void star_gem10x_device::mem_map(address_map &map)
{
	map(0x1000, 0x2fff).lr8(NAME([this] (offs_t offset) { return m_ext[(offset + 0x1000) & 0x1fff]; }));
	map(0x3000, 0x37ff).mirror(0x0800).ram();
	map(0x4000, 0x4003).mirror(0x0ffc).rw(m_pit, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0x5000, 0x5000).mirror(0x0fff).rw(FUNC(star_gem10x_device::cent_r), FUNC(star_gem10x_device::cent_w));
	map(0x6000, 0x6001).mirror(0x0ffe).rw(FUNC(star_gem10x_device::sel_r), FUNC(star_gem10x_device::needles_w));
	map(0x7000, 0x7000).mirror(0x0fff).w(FUNC(star_gem10x_device::latch7_w));
}

void star_gem10x_device::device_add_mconfig(machine_config &config)
{
	UPD7801(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &star_gem10x_device::mem_map);
	m_maincpu->pb_in_cb().set(FUNC(star_gem10x_device::pb_r));
	m_maincpu->pc_in_cb().set(FUNC(star_gem10x_device::pc_r));
	m_maincpu->pa_out_cb().set(FUNC(star_gem10x_device::pa_w));
	m_maincpu->pb_out_cb().set(FUNC(star_gem10x_device::pb_w));
	m_maincpu->pc_out_cb().set(FUNC(star_gem10x_device::pc_w));

	PIT8253(config, m_pit);
	m_pit->set_clk<0>(4_MHz_XTAL / 2);
	m_pit->set_clk<1>(4_MHz_XTAL / 2);
	m_pit->set_clk<2>(4_MHz_XTAL / 2);
	m_pit->out_handler<1>().set([this] (int state) { m_out1 = state; update_int2(); });
	m_pit->out_handler<2>().set([this] (int state) { m_out2 = state; update_int2(); });

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_star_gem10x);

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.5);
}

void star_gem10x_device::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_strobe_ff));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_needles));
	save_item(NAME(m_out1));
	save_item(NAME(m_out2));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void star_gem10x_device::device_reset()
{
	m_strobe_ff = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	m_cr_last_step = machine().time();
	m_cr_period = attotime::never;
	m_power_on = machine().time();
	m_power_cycled = false;
	output_fault(1);
	output_ack(1);
	update_outputs();
}

INPUT_CHANGED_MEMBER(star_gem10x_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
		m_strobe_ff = false;
		m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
		m_power_on = machine().time();
		m_power_cycled = true;
		update_outputs();
	}
}


//-------------------------------------------------
//  board I/O
//-------------------------------------------------

bool star_gem10x_device::paper_present() const
{
	return BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
}

u8 star_gem10x_device::sel_r(offs_t offset)
{
	u8 const dsw1 = m_dsw1->read(), dsw2 = m_dsw2->read();
	u8 panel = m_panel->read() & 0x07;

	// L.F held while the printer is switched on starts the self test
	if (BIT(m_config->read(), 1) && m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(8)))
		panel &= ~0x02;

	// the paper out switch pulls its line low without paper; DSW2-1 passes it
	// on to the Centronics PE line (and the B input), otherwise that is high
	bool const paper = paper_present();
	bool const pe_line = BIT(dsw2, 0) || paper;

	if (!offset)    // 74LS257 A inputs: ON LINE, LF, FF, paper switch; DSW1-1..4
		return panel | (paper ? 0x08 : 0x00) | ((dsw1 & 0x0f) << 4);
	else            // B inputs: paper out line, DSW2-2..4; DSW1-5..8
		return (pe_line ? 0x01 : 0x00) | (dsw2 & 0x0e) | (dsw1 & 0xf0);
}

void star_gem10x_device::needles_w(u8 data)
{
	m_needles = data;
}

void star_gem10x_device::latch7_w(u8 data)
{
	LOG("%s 7000 = %02x\n", machine().describe_context(), data);
}

u8 star_gem10x_device::cent_r()
{
	return m_latch;
}

void star_gem10x_device::cent_w(u8 data)
{
	// writing to the data port clears the strobe flip-flop
	m_strobe_ff = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	update_outputs();
}

void star_gem10x_device::input_strobe(int state)
{
	// IC4 (74LS74) latches /STROBE, its output is INT1
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_strobe_ff = true;
		m_maincpu->set_input_line(UPD7810_INTF1, ASSERT_LINE);
		update_outputs();
	}
	m_strobe = state;
}

void star_gem10x_device::update_int2()
{
	// INT2: the dot column timer (and counter 2) have run out
	m_maincpu->set_input_line(UPD7810_INTF2, (m_out1 && m_out2) ? ASSERT_LINE : CLEAR_LINE);
}

void star_gem10x_device::update_outputs()
{
	bool const online = BIT(m_pc, 6);
	bool const paper = paper_present();
	bool const pe = !(BIT(m_dsw2->read(), 0) || paper);

	output_busy(m_strobe_ff || !online);
	output_select(online);
	output_perror(pe);

	m_online_led = online ? 1 : 0;
	m_ready_led = (online && !m_strobe_ff) ? 1 : 0;
	m_paper_led = paper ? 0 : 1;
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

u8 star_gem10x_device::pb_r()
{
	u8 data = 0xff;
	if (m_cr_pos <= CR_EDGE)
		data &= ~0x80;      // home sensor
	return data;
}

u8 star_gem10x_device::pc_r()
{
	return 0xff;            // PC7 = 1: parallel interface, PC0 = RxD idle
}

void star_gem10x_device::pa_w(u8 data)
{
	u8 const old = m_pa;
	m_pa = data;

	// 8253 gates: counter 0 needle pulse, 1 dot column, 2 used with the carriage
	m_pit->write_gate0(BIT(data, 5));
	m_pit->write_gate1(BIT(data, 6));
	m_pit->write_gate2(BIT(data, 7));

	if (!BIT(old, 5) && BIT(data, 5))
		fire();

	// paper stepper, Gray code on PA4:PA3
	if ((old ^ data) & 0x18)
	{
		static int const gray[4] = { 2, 1, 3, 0 };  // forward: 11 01 00 10
		int const phase = gray[(data >> 3) & 3];
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
}

void star_gem10x_device::pb_w(u8 data)
{
	u8 const old = m_pb;
	m_pb = data;

	// carriage stepper, Gray code on PB2:PB1
	if ((old ^ data) & 0x06)
	{
		static int const gray[4] = { 0, 1, 3, 2 };
		int const phase = gray[(data >> 1) & 3];
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
			m_bitmap_printer->setheadpos(int((m_cr_pos - CR_EDGE) / 120.0 * HDPI + LEFT_OFFSET * HDPI), m_bitmap_printer->m_ypos);
		}
	}

	if ((old ^ data) & 0x20)
		m_speaker->level_w(BIT(data, 5));
}

void star_gem10x_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	if ((old ^ data) & 0x20)
		output_ack(!BIT(data, 5));
	if ((old ^ data) & 0x40)
		update_outputs();
}


//-------------------------------------------------
//  printing
//-------------------------------------------------

double star_gem10x_device::head_x() const
{
	// steps from the home sensor edge; the carriage moves on between two steps
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos - CR_EDGE;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - CR_EDGE - m_cr_dir * (1.0 - frac);
}

void star_gem10x_device::fire()
{
	bool const pin9 = !BIT(m_pb, 0);
	if (!m_needles && !pin9)
		return;

	int const x = int(std::lround((head_x() + m_cr_dir * CR_SHIFT) / 120.0 * HDPI + LEFT_OFFSET * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles 1/72" apart, two paper steps
	int const y = m_bitmap_printer->m_ypos;
	for (int i = 0; i < 8; i++)
		if (BIT(m_needles, i))
			m_bitmap_printer->draw_pixel(x, y + i * 2, 0x000000);
	if (pin9)
		m_bitmap_printer->draw_pixel(x, y + 16, 0x000000);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(STAR_GEMINI10X, device_centronics_peripheral_interface, star_gem10x_device, "gem10x", "Star Gemini-10X")
