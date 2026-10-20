// license:BSD-3-Clause
// copyright-holders:Golden Child, rfka01
/***************************************************************************

    Epson MX-80 9-pin dot matrix printer

    Main CPU: 8039 (or 8049 with its mask ROM) at 6 MHz, 6 KiB program ROM.
    Slave CPU: 8041 at 6 MHz, runs the carriage and paper feed steppers.
    8155: 256 bytes line buffer, Centronics input port, needle drivers.

    ROM sets:
      mx80      Graftrax-80: 8039 + three 2716 (3B = 000-7ff, 1B/2B = the
                upper 2 KiB bank selected by P24)
      mx80dp    Dots Perfect upgrade: one 27128, four 4 KiB banks selected by
                P21:P20 on the falling edge of PROG

    Main CPU:
      P10           ON LINE lamp
      P11..P14 in   8041 P20 (/PRAV), P23 (/ERR), P21 (/DIRAV), P22 (DIR)
      P15           /ACK             P16    BUSY (with the 8155 BF output)
      P17 in        1 = no line feed after CR (/AUTO FEED XT, DIP SW2-3)
      P23..P20      device select, active low: P23 8155, P22 8041,
                    P21 DIP switch 1, P20 control panel (read with INS A,BUS)
      P24           upper ROM bank      P25    8155 IO/M, 8041 A0
      P26           head trigger, a falling edge starts the 555 needle pulse
      P27           Centronics /ERROR
      T0            serial data input (PA7)
      T1            PTS sensor
      INT           8155 port A INTR

    8155 (ALT 3):
      PA            Centronics data, latched by /STROBE on PC2
      PC0           INTR to the 8049     PC1    BF (BUSY)
      PC3           SLCT                 PC4    needle 9
      PB7..PB0      needles 1..8, active low

    8041:
      P11:P10       carriage stepper, Gray code, 1 step = 1/60"
      P13:P12       carriage motor current
      P15:P14       paper stepper, Gray code, 1 step = 1/216"
      P16           paper motor current
      P17           buzzer, active low
      P23..P20      status to the 8049
      P24..P26      DIP SW2-4, SW2-2, SW2-1
      P27           1 = parallel interface
      T0            PTS sensor, one cycle per carriage step
      T1            home sensor

    Control panel (INS A,BUS with P20 low): bit 0 ON LINE, bit 1 paper
    end sensor (1 = paper), bit 2 LINE FEED, bit 3 FORM FEED; ON LINE is
    active high, the feed keys are active low.  Holding LINE FEED while the printer is switched on runs the
    self test; select it under "Keys held at power on" and use the printer's
    power switch input, which restarts only the printer.

    TODO:
    - serial interface board
    - the 8049 mask ROM (MX-80 F/T III socket 3B) with the 2332 takes data
      but does not print with this 8041; its own 8041 is not dumped
    - ESC 4 does not select italics with the Graftrax ROMs
    - DIP switch functions come from the MX-80 manuals and the firmware

***************************************************************************/

#include "emu.h"
#include "epson_mx80.h"

#include "cpu/mcs48/mcs48.h"
#include "machine/bitmap_printer.h"
#include "machine/i8155.h"
#include "sound/beep.h"

#include "speaker.h"

#include "epson_mx80.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.75;    // paper edge to the home sensor edge

// the carriage motor takes CR_MOVE for a step and dwells for the rest of a longer
// step period; the head lags the rotor by CR_SHIFT steps plus CR_TLAG seconds
constexpr double CR_MOVE = 0.00208;
constexpr double CR_SHIFT = 0.0;
constexpr double CR_TLAG = 0.0;
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 520;           // right stop

// Gray code phase index of a 2-bit stepper port value (00 01 11 10 = forward)
constexpr int gray_phase[4] = { 0, 1, 3, 2 };

class epson_mx80_device : public device_t, public device_centronics_peripheral_interface
{
public:
	// construction/destruction
	epson_mx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

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

	DECLARE_INPUT_CHANGED_MEMBER(power_switch);

protected:
	epson_mx80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

	virtual void mx80_prog(address_map &map) ATTR_COLD;
	virtual void update_rom_bank() { m_rombank->set_entry(BIT(m_p2, 4) ? 0 : 1); }   // P24 = 1: first bank

	required_device<i8039_device> m_maincpu;
	optional_memory_bank m_rombank;
	u8 m_p2;

private:
	void mx80_io(address_map &map) ATTR_COLD;

	// main CPU
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);
	TIMER_CALLBACK_MEMBER(slave_write_sync);
	u8 bus_r();
	u8 p1_r();
	void p1_w(u8 data);
	void p2_w(u8 data);
	int t0_r();

	// 8155
	u8 pa_r();
	void pb_w(u8 data);
	void pc_w(u8 data);

	// slave CPU
	void slave_p1_w(u8 data);
	u8 slave_p2_r();
	void slave_p2_w(u8 data);
	int home_r();
	int pts_r();

	TIMER_CALLBACK_MEMBER(pts_edge);

	void restart();
	void update_busy();
	void set_pts(int state);
	void carriage_step(int dir);
	double head_x() const;
	void fire_needles();
	bool paper_loaded();
	u8 keys_held() const;

	required_device<i8041ah_device> m_slavecpu;
	required_device<i8155_device> m_i8155;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<beep_device> m_beep;
	required_ioport m_panel;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paper_led;

	emu_timer *m_pts_timer;

	// Centronics
	u8 m_data;
	bool m_init;
	int m_autofd;           // /AUTO FEED XT level

	// outputs
	u8 m_p1;
	u8 m_pb;
	u8 m_pc;
	u8 m_slave_p1;
	u8 m_slave_p2;

	// carriage
	int m_cr_pos;           // 1/60" steps right of the home sensor edge
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pts;              // print timing sensor level
	int m_pts_pending;      // PTS edges still due for the last step: 2 = rise and fall, 1 = fall
	attotime m_pts_fall;    // delay of the falling edge after the rising edge

	// paper
	int m_pf_phase;

	attotime m_power_on;
	bool m_power_cycled;    // switched on with the power switch, not by a machine reset or /INIT
};


class epson_mx80dp_device : public epson_mx80_device
{
public:
	epson_mx80dp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void mx80_prog(address_map &map) override ATTR_COLD;
	virtual void update_rom_bank() override { }

private:
	void prog_w(int state);

	int m_prog;
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( epson_mx80 )
	ROM_REGION(0x1800, "maincpu", 0)  // Graftrax-80
	ROM_LOAD("mx80_graftrax_rom_3.bin", 0x0000, 0x0800, CRC(f1d1c0e7) SHA1(1f4e24575c412368f0b628d2a276c9943debd0ae))  // 3B
	ROM_LOAD("mx80_graftrax_rom_1.bin", 0x0800, 0x0800, CRC(0843ea56) SHA1(23948acb55760ddbfc59c57c1553b2e733983201))  // 1B, P24 = 1
	ROM_LOAD("mx80_graftrax_rom_2.bin", 0x1000, 0x0800, CRC(8435f945) SHA1(c1f0a74bf66d114d0f349bec27c7756997352602))  // 2B, P24 = 0

	ROM_REGION(0x400, "slavecpu", 0)
	ROM_LOAD("8041_mx80.bin", 0x0000, 0x0400, CRC(5844ef51) SHA1(1025d34b3ab684a06589b5890c604ce114399d23))
ROM_END

ROM_START( epson_mx80dp )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("dots_perfect_mx80_27128.bin", 0x0000, 0x4000, CRC(49bae08f) SHA1(702b3fd3e8fefd983306028915a49cb26880bb16))

	ROM_REGION(0x400, "slavecpu", 0)
	ROM_LOAD("8041_mx80.bin", 0x0000, 0x0400, CRC(5844ef51) SHA1(1025d34b3ab684a06589b5890c604ce114399d23))
ROM_END

const tiny_rom_entry *epson_mx80_device::device_rom_region() const
{
	return ROM_NAME(epson_mx80);
}

const tiny_rom_entry *epson_mx80dp_device::device_rom_region() const
{
	return ROM_NAME(epson_mx80dp);
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( epson_mx80_common )
	PORT_START("PANEL")  // read with P20 low; ON LINE 1 = pressed, LINE FEED and FORM FEED 0 = pressed
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MX-80 ON LINE")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("MX-80 LINE FEED")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("MX-80 FORM FEED")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("MX-80 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_mx80_device::power_switch), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x02, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x02, "LINE FEED (self test)" )
INPUT_PORTS_END

static INPUT_PORTS_START( epson_mx80 )
	PORT_INCLUDE(epson_mx80_common)

	PORT_START("DSW1")  // read with P21 low, ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Line spacing" )               PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, "1/8\"" )
	PORT_DIPSETTING(    0x01, "1/6\"" )
	PORT_DIPNAME( 0x02, 0x02, "Form length" )                PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "12\"" )
	PORT_DIPSETTING(    0x02, "11\"" )
	PORT_DIPNAME( 0x0c, 0x0c, "Character size" )             PORT_DIPLOCATION("SW1:3,4")
	PORT_DIPSETTING(    0x0c, "Normal (80)" )
	PORT_DIPSETTING(    0x04, "Condensed (132)" )
	PORT_DIPSETTING(    0x00, "Emphasized (80)" )
	PORT_DIPNAME( 0x10, 0x10, "Always leave OFF" )           PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPNAME( 0x20, 0x20, "Paper-end detector" )         PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x20, "Enabled" )
	PORT_DIPNAME( 0x40, 0x00, "International characters" )   PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x00, "USA/France/Germany/England" )
	PORT_DIPSETTING(    0x40, "Denmark/Sweden/Italy/Spain" )
	PORT_DIPNAME( 0x80, 0x00, "/SLCT IN" )                   PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x00, "Fixed low" )
	PORT_DIPSETTING(    0x80, "From the host" )

	PORT_START("DSW2")  // SW2-1, 2 and 4 read by the 8041, SW2-3 on the 8049 P17; ON = 0
	PORT_DIPNAME( 0x03, 0x00, "International characters" )   PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x00, "USA/Denmark" )
	PORT_DIPSETTING(    0x01, "Germany/Italy" )
	PORT_DIPSETTING(    0x02, "France/Sweden" )
	PORT_DIPSETTING(    0x03, "England/Spain" )
	PORT_DIPNAME( 0x04, 0x04, "Auto line feed with CR" )     PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(    0x00, "Always" )
	PORT_DIPSETTING(    0x04, "From the host" )
	PORT_DIPNAME( 0x08, 0x00, "Skip over perforation" )      PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, "1\"" )
INPUT_PORTS_END

static INPUT_PORTS_START( epson_mx80dp )
	PORT_INCLUDE(epson_mx80_common)

	PORT_START("DSW1")  // read with P21 low, ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Compressed" )                 PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPNAME( 0x02, 0x02, "Character set" )              PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "IBM" )
	PORT_DIPSETTING(    0x02, "Epson" )
	PORT_DIPNAME( 0x04, 0x04, "Paper-end detector" )         PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x04, "Enabled" )
	PORT_DIPNAME( 0x08, 0x08, "Italic" )                     PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPNAME( 0x10, 0x10, "Emphasized" )                 PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPNAME( 0x20, 0x00, "Buzzer" )                     PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPNAME( 0x40, 0x40, "Slashed zero" )               PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPNAME( 0x80, 0x00, "/SLCT IN" )                   PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x00, "Fixed low" )
	PORT_DIPSETTING(    0x80, "From the host" )

	PORT_START("DSW2")  // SW2-1, 2 and 4 read by the 8041, SW2-3 on the 8049 P17; ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Paper width" )                PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(    0x00, "13.5\"" )
	PORT_DIPSETTING(    0x01, "8.5\"" )
	PORT_DIPNAME( 0x02, 0x02, "Print mode" )                 PORT_DIPLOCATION("SW2:2")
	PORT_DIPSETTING(    0x00, "NLQ" )
	PORT_DIPSETTING(    0x02, "Draft" )
	PORT_DIPNAME( 0x04, 0x04, "Auto line feed with CR" )     PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(    0x00, "Always" )
	PORT_DIPSETTING(    0x04, "From the host" )
	PORT_DIPNAME( 0x08, 0x00, "Skip over perforation" )      PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, "1\"" )
INPUT_PORTS_END

ioport_constructor epson_mx80_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(epson_mx80);
}

ioport_constructor epson_mx80dp_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(epson_mx80dp);
}


//**************************************************************************
//  DEVICE
//**************************************************************************

epson_mx80_device::epson_mx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	epson_mx80_device(mconfig, EPSON_MX80, tag, owner, clock)
{
}

epson_mx80_device::epson_mx80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_rombank(*this, "rombank"),
	m_p2(0xff),
	m_slavecpu(*this, "slavecpu"),
	m_i8155(*this, "i8155"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_beep(*this, "beep"),
	m_panel(*this, "PANEL"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "mx80_online_led"),
	m_ready_led(*this, "mx80_ready_led"),
	m_paper_led(*this, "mx80_paper_led"),
	m_pts_timer(nullptr),
	m_data(0),
	m_init(true),
	m_autofd(1),
	m_p1(0xff),
	m_pb(0xff),
	m_pc(0xff),
	m_slave_p1(0xff),
	m_slave_p2(0xff),
	m_cr_pos(-20),
	m_cr_phase(0),
	m_cr_dir(1),
	m_pts(0),
	m_pts_pending(0),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void epson_mx80_device::mx80_prog(address_map &map)
{
	map(0x000, 0x7ff).rom().region("maincpu", 0);
	map(0x800, 0xfff).bankr(m_rombank);
}

void epson_mx80_device::mx80_io(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(epson_mx80_device::io_r), FUNC(epson_mx80_device::io_w));
}

void epson_mx80_device::device_add_mconfig(machine_config &config)
{
	I8039(config, m_maincpu, 6_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_mx80_device::mx80_prog);
	m_maincpu->set_addrmap(AS_IO, &epson_mx80_device::mx80_io);
	m_maincpu->p1_in_cb().set(FUNC(epson_mx80_device::p1_r));
	m_maincpu->p1_out_cb().set(FUNC(epson_mx80_device::p1_w));
	m_maincpu->p2_in_cb().set_constant(0xff);
	m_maincpu->p2_out_cb().set(FUNC(epson_mx80_device::p2_w));
	m_maincpu->bus_in_cb().set(FUNC(epson_mx80_device::bus_r));
	m_maincpu->t0_in_cb().set(FUNC(epson_mx80_device::t0_r));
	m_maincpu->t1_in_cb().set(FUNC(epson_mx80_device::pts_r));

	I8041AH(config, m_slavecpu, 6_MHz_XTAL);
	m_slavecpu->p1_in_cb().set_constant(0xff);
	m_slavecpu->p1_out_cb().set(FUNC(epson_mx80_device::slave_p1_w));
	m_slavecpu->p2_in_cb().set(FUNC(epson_mx80_device::slave_p2_r));
	m_slavecpu->p2_out_cb().set(FUNC(epson_mx80_device::slave_p2_w));
	m_slavecpu->t0_in_cb().set(FUNC(epson_mx80_device::pts_r));
	m_slavecpu->t1_in_cb().set(FUNC(epson_mx80_device::home_r));

	I8155(config, m_i8155, 6_MHz_XTAL / 15);  // TIMER IN unused
	m_i8155->in_pa_callback().set(FUNC(epson_mx80_device::pa_r));
	m_i8155->out_pb_callback().set(FUNC(epson_mx80_device::pb_w));
	m_i8155->out_pc_callback().set(FUNC(epson_mx80_device::pc_w));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_epson_mx80);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 2000).add_route(ALL_OUTPUTS, "mono", 0.3);
}

void epson_mx80_device::device_start()
{

	if (m_rombank)
		m_rombank->configure_entries(0, 2, memregion("maincpu")->base() + 0x800, 0x800);

	m_pts_timer = timer_alloc(FUNC(epson_mx80_device::pts_edge), this);

	save_item(NAME(m_p2));
	save_item(NAME(m_data));
	save_item(NAME(m_init));
	save_item(NAME(m_autofd));
	save_item(NAME(m_p1));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_slave_p1));
	save_item(NAME(m_slave_p2));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pts));
	save_item(NAME(m_pts_pending));
	save_item(NAME(m_pts_fall));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void epson_mx80_device::device_reset()
{
	m_p1 = 0xff;
	m_p2 = 0xff;
	update_rom_bank();
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_power_on = machine().time();
	m_power_cycled = false;
	m_pts_pending = 0;
	m_pts_timer->adjust(attotime::never);
	set_pts(0);
	update_busy();
	output_ack(1);
}

void epson_mx80_device::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_slavecpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_i8155->reset();
	m_p1 = 0xff;
	m_p2 = 0xff;
	update_rom_bank();
	m_power_on = machine().time();
	m_power_cycled = false;
	update_busy();
}

INPUT_CHANGED_MEMBER(epson_mx80_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

u8 epson_mx80_device::keys_held() const
{
	// keys held while the printer is switched on with its power switch (bits as in PANEL)
	if (m_power_cycled && BIT(m_config->read(), 1) && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return 0x04;
	return 0;
}

bool epson_mx80_device::paper_loaded()
{
	bool const paper = BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
	m_paper_led = !paper;
	output_perror(!paper);
	return paper;
}


//-------------------------------------------------
//  Centronics
//-------------------------------------------------

void epson_mx80_device::input_strobe(int state)
{
	// /STROBE goes to the 8155 PC2: its falling edge latches the data and sets BF,
	// its rising edge raises INTR
	m_i8155->stb_a_w(state);
}

void epson_mx80_device::input_init(int state)
{
	// /INIT resets the printer controller
	if (m_init && !state)
		restart();
	m_init = state;
}

void epson_mx80_device::update_busy()
{
	// BUSY is the 8155 buffer full flag or the 8049 P16
	bool const busy = BIT(m_pc, 1) || BIT(m_p1, 6);
	output_busy(busy);
	m_ready_led = !busy;
}


//-------------------------------------------------
//  main CPU
//-------------------------------------------------

u8 epson_mx80_device::io_r(offs_t offset)
{
	// devices are selected by P23..P20, active low
	u8 data = 0xff;
	if (!BIT(m_p2, 3))
		data &= BIT(m_p2, 5) ? m_i8155->io_r(offset) : m_i8155->memory_r(offset);
	if (!BIT(m_p2, 2))
		data &= m_slavecpu->upi41_master_r(BIT(m_p2, 5));
	if (!BIT(m_p2, 1) || !BIT(m_p2, 0))
		data &= bus_r();
	return data;
}

void epson_mx80_device::io_w(offs_t offset, u8 data)
{
	if (!BIT(m_p2, 3))
	{
		if (BIT(m_p2, 5))
			m_i8155->io_w(offset, data);
		else
			m_i8155->memory_w(offset, data);
	}
	if (!BIT(m_p2, 2))
	{
		// pass A0 and data packed into param
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(epson_mx80_device::slave_write_sync), this), (BIT(m_p2, 5) << 8) | data);
	}
}

TIMER_CALLBACK_MEMBER(epson_mx80_device::slave_write_sync)
{
	m_slavecpu->upi41_master_w(u32(param) >> 8, u32(param) & 0xff);
}

u8 epson_mx80_device::bus_r()
{
	// the 74LS367 buffers of DIP switch 1 (P21 low) and of the control panel (P20 low)
	u8 data = 0xff;
	if (!BIT(m_p2, 1))
		data &= m_dsw1->read();
	if (!BIT(m_p2, 0))
	{
		u8 const keys = m_panel->read() & ~keys_held();
		data &= 0xf0 | (keys & 0x0d) | (paper_loaded() ? 0x02 : 0x00);
	}
	return data;
}

u8 epson_mx80_device::p1_r()
{
	// P11..P14: 8041 status, P17: 0 = line feed after CR
	u8 data = 0xe1;
	data |= BIT(m_slave_p2, 0) << 1;   // /PRAV
	data |= BIT(m_slave_p2, 3) << 2;   // /ERR
	data |= BIT(m_slave_p2, 1) << 3;   // /DIRAV
	data |= BIT(m_slave_p2, 2) << 4;   // DIR
	if (!m_autofd || !BIT(m_dsw2->read(), 2))
		data &= ~0x80;
	return data;
}

void epson_mx80_device::p1_w(u8 data)
{
	u8 const old = m_p1;
	m_p1 = data;

	m_online_led = BIT(data, 0);
	if ((old ^ data) & 0x20)
		output_ack(BIT(data, 5));
	if ((old ^ data) & 0x40)
		update_busy();
}

void epson_mx80_device::p2_w(u8 data)
{
	u8 const old = m_p2;
	m_p2 = data;

	if ((old ^ data) & 0x10)
		update_rom_bank();

	// head trigger: the falling edge starts the needle pulse
	if (BIT(old, 6) && !BIT(data, 6))
		fire_needles();

	if ((old ^ data) & 0x80)
		output_fault(BIT(data, 7));
}

int epson_mx80_device::t0_r()
{
	// serial interface data, shared with PA7
	return BIT(m_data, 7);
}


//-------------------------------------------------
//  8155
//-------------------------------------------------

u8 epson_mx80_device::pa_r()
{
	return m_data;
}

void epson_mx80_device::pb_w(u8 data)
{
	m_pb = data;
}

void epson_mx80_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	// PC0: INTR to the 8049 /INT
	if ((old ^ data) & 0x01)
		m_maincpu->set_input_line(MCS48_INPUT_IRQ, BIT(data, 0) ? ASSERT_LINE : CLEAR_LINE);
	if ((old ^ data) & 0x02)
		update_busy();
	if ((old ^ data) & 0x08)
		output_select(BIT(data, 3));
}


//-------------------------------------------------
//  slave CPU
//-------------------------------------------------

void epson_mx80_device::slave_p1_w(u8 data)
{
	u8 const old = m_slave_p1;
	m_slave_p1 = data;

	// carriage stepper: Gray code on P11:P10
	if ((old ^ data) & 0x03)
	{
		int const phase = gray_phase[data & 3];
		int const delta = (phase - m_cr_phase) & 3;
		m_cr_phase = phase;
		if (delta == 1 || delta == 3)
			carriage_step((delta == 1) ? 1 : -1);
	}

	// paper stepper: Gray code on P15:P14
	if ((old ^ data) & 0x30)
	{
		int const phase = gray_phase[(data >> 4) & 3];
		if (m_pf_phase >= 0)
		{
			int const delta = (phase - m_pf_phase) & 3;
			if (delta == 1)
				m_bitmap_printer->m_ypos++;
			else if (delta == 3)
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
		m_pf_phase = phase;
	}

	if ((old ^ data) & 0x80)
		m_beep->set_state(!BIT(data, 7));
}

u8 epson_mx80_device::slave_p2_r()
{
	// P24..P26: DIP SW2-4, SW2-2, SW2-1; P27: 1 = parallel interface
	u8 const dsw2 = m_dsw2->read();
	return 0x8f | (BIT(dsw2, 3) << 4) | (BIT(dsw2, 1) << 5) | (BIT(dsw2, 0) << 6);
}

void epson_mx80_device::slave_p2_w(u8 data)
{
	m_slave_p2 = data;
}

int epson_mx80_device::home_r()
{
	return (m_cr_pos < 0) ? 0 : 1;
}

int epson_mx80_device::pts_r()
{
	return m_pts;
}


//-------------------------------------------------
//  carriage and print timing sensor
//-------------------------------------------------

void epson_mx80_device::set_pts(int state)
{
	// the PTS signal goes to the 8041 T0 and the 8049 T1 inputs
	m_pts = state;
}

TIMER_CALLBACK_MEMBER(epson_mx80_device::pts_edge)
{
	if (m_pts_pending == 2)
	{
		set_pts(1);
		m_pts_pending = 1;
		m_pts_timer->adjust(m_pts_fall);
	}
	else if (m_pts_pending == 1)
	{
		set_pts(0);
		m_pts_pending = 0;
	}
}

void epson_mx80_device::carriage_step(int dir)
{
	attotime const now = machine().time();
	attotime const since = now - m_cr_last_step;

	// one PTS cycle per step; edges still due for the previous step are produced right away
	if (m_pts_pending == 2)
	{
		set_pts(1);
		m_pts_pending = 1;
		m_pts_fall = attotime::from_usec(100);
		m_pts_timer->adjust(m_pts_fall);
	}

	m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
	m_cr_last_step = now;
	m_cr_dir = dir;
	m_cr_pos = std::clamp(m_cr_pos + dir, CR_LEFT, CR_RIGHT);
	m_bitmap_printer->setheadpos(int((m_cr_pos / 60.0 + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);

	// the sensor slot passes halfway through the step and stays open for 30% of it
	attotime const period = m_cr_period.is_never() ? attotime::from_double(CR_MOVE) : std::min(m_cr_period, attotime::from_double(CR_MOVE));
	attotime rise = period / 2;
	if (m_pts_pending)
	{
		// finish the previous cycle first
		if (m_pts_pending == 1)
		{
			set_pts(0);
			m_pts_pending = 0;
		}
		rise = std::max(rise, attotime::from_usec(200));
	}
	m_pts_fall = std::max(attotime::from_usec(150), period * 3 / 10);
	m_pts_pending = 2;
	m_pts_timer->adjust(rise);
}

double epson_mx80_device::head_x() const
{
	// a step starts the move to the new position, which is reached at the
	// next step when the carriage runs at full speed, after CR_MOVE when slower
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos;
	double const move = std::min(m_cr_period.as_double(), CR_MOVE);
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / move);
	return m_cr_pos - m_cr_dir * (1.0 - frac + CR_SHIFT + CR_TLAG / m_cr_period.as_double());
}

void epson_mx80_device::fire_needles()
{
	int const x = int(std::lround((head_x() / 60.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles 1..8 on PB7..PB0, needle 9 on PC4, active low; 1/72" = 3 paper steps apart
	u16 const needles = ~((m_pb << 1) | BIT(m_pc, 4)) & 0x1ff;
	for (int i = 0; i < 9; i++)
		if (BIT(needles, 8 - i))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + i * (VDPI / 72), 0x000000);
}


//-------------------------------------------------
//  Dots Perfect
//-------------------------------------------------

epson_mx80dp_device::epson_mx80dp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	epson_mx80_device(mconfig, EPSON_MX80DP, tag, owner, clock),
	m_prog(1)
{
}

void epson_mx80dp_device::mx80_prog(address_map &map)
{
	map(0x000, 0xfff).bankr(m_rombank);
}

void epson_mx80dp_device::device_add_mconfig(machine_config &config)
{
	epson_mx80_device::device_add_mconfig(config);
	m_maincpu->prog_out_cb().set(FUNC(epson_mx80dp_device::prog_w));
}

void epson_mx80dp_device::device_start()
{
	epson_mx80_device::device_start();
	m_rombank->configure_entries(0, 4, memregion("maincpu")->base(), 0x1000);
	save_item(NAME(m_prog));
}

void epson_mx80dp_device::device_reset()
{
	epson_mx80_device::device_reset();
	m_rombank->set_entry(0);
}

void epson_mx80dp_device::prog_w(int state)
{
	// the bank latch takes P21:P20 on the falling edge of PROG
	if (m_prog && !state)
		m_rombank->set_entry(m_p2 & 3);
	m_prog = state;
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_MX80,   device_centronics_peripheral_interface, epson_mx80_device,   "epson_mx80",   "Epson MX-80 (Graftrax-80)")
DEFINE_DEVICE_TYPE_PRIVATE(EPSON_MX80DP, device_centronics_peripheral_interface, epson_mx80dp_device, "epson_mx80dp", "Epson MX-80 (Dots Perfect)")
