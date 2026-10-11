// license:BSD-3-Clause
// copyright-holders:Golden Child, rfka01
/***************************************************************************

    Epson FX-80 9-pin dot matrix printer
    Epson JX-80 9-pin colour dot matrix printer

    Main CPU: uPD7810H6 at 10 MHz, 16 KiB mask ROM, 6 KiB external RAM.
    Slave CPU: 8042AH at 11 MHz, runs the carriage stepper.
    The JX-80 adds the VX0B board: 8 KiB RAM and the colour ribbon shift
    motor.

    Main CPU (from the firmware):
      0000-3fff     ROM
      8000-97ff     RAM
      d000-d001     8042 UPI data / status+command
      d800          Centronics data latch
      PA7..PA0      needles 1..8 (PA7 = top), active low
      PB0 out       BUSY             PB1 out      /ACK
      PB2 out       driver inhibit (errors, power fail)
      PB3 in        0 = serial interface board fitted
      PB4 in        1 = line feed after CR (/AUTO FEED XT or DIP SW2-4)
      PB5 in        FORM FEED key    PB6 in       LINE FEED key
      PB7 out       needle 9, active low
      PC2 out       paper motor energise
      PC5 (CI)      PTS sensor: its falling edge clears the event counter;
                    the dot columns are timed from it (INTE1)
      PC7:PC6 out   paper stepper, Gray code, 1 step = 1/216"
      INT1          Centronics strobe
      AN0           needle pulse width compensation
      AN1           0 = wait for DC1 after reset
      AN2           analog supervision (must be mid-range)
      AN3           paper end sensor (0 = paper out)
      AN4           page length (1 = 12")
      AN5           0 = cut sheet mode
      AN6           0 = skip over perforation
      AN7           ON LINE key (0 = pressed, falling edge flag)

    8042:
      P1            DIP switch 1 (0 = ON)
      P21:P20       carriage stepper, Gray code, 1 step = 1/60"
      P24..P22      carriage motor current (all 1 = off)
      P25           error (written by the main CPU)
      P26           ON LINE lamp
      P27           buzzer, active low
      T0            home sensor (0 = home zone)
      T1            PTS sensor, one cycle per carriage step

    JX-80 VX0B:
      c800 r        bit 7 = colour ribbon home sensor
      c801 w        bits 7-6 ribbon shift stepper (Gray code), bit 5 energise

    Power-on key combinations: LINE FEED = self test, FORM FEED = hex
    dump.  Hold the keys (or select them under "Keys held at power on")
    and use the printer's power switch input, which restarts only the
    printer.

    TODO:
    - DIP switch 2 is read through AN1/AN4-AN6; switch numbers and
      polarity are not verified
    - optional serial interface board
    - CI polarity relative to the 8042 T1 input

***************************************************************************/

#include "emu.h"
#include "epson_fx80.h"

#include "cpu/mcs48/mcs48.h"
#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "sound/beep.h"

#include "speaker.h"

#include "epson_fx80.lh"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.75;    // paper edge to the home sensor edge

constexpr double CR_LAG = 0.25;         // lag of the head behind the rotor, in steps
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 520;           // right stop

// Gray code phase index of a 2-bit stepper port value (00 01 11 10 = forward)
constexpr int gray_phase[4] = { 0, 1, 3, 2 };

// JX-80 ribbon bands
constexpr u32 ribbon_color[4] = { 0x000000, 0xd8306c, 0x2c74c8, 0xf0d028 };   // black, red, blue, yellow
constexpr int RIBBON_BAND = 36;         // shift motor steps per colour band

class epson_fx80_device : public device_t, public device_centronics_peripheral_interface
{
public:
	// construction/destruction
	epson_fx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

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
	epson_fx80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

	void fx80_mem(address_map &map) ATTR_COLD;

	u32 ink_color() const { return ribbon_color[m_ribbon_band]; }
	virtual void update_ribbon() { }

	required_device<upd7810_device> m_maincpu;
	required_device<i8042ah_device> m_slavecpu;
	required_device<bitmap_printer_device> m_bitmap_printer;

	int m_ribbon_band;      // colour band in front of the needles (JX-80)

private:
	// main CPU
	u8 slave_r(offs_t offset);
	void slave_w(offs_t offset, u8 data);
	TIMER_CALLBACK_MEMBER(slave_write_sync);
	u8 cent_data_r();
	void pa_w(u8 data);
	u8 pb_r();
	void pb_w(u8 data);
	void pc_w(u8 data);
	u8 an1_r();
	u8 an3_r();
	u8 an4_r();
	u8 an5_r();
	u8 an6_r();
	u8 an7_r();

	// slave CPU
	u8 slave_p1_r();
	void slave_p2_w(u8 data);
	int home_r();
	int pts_r();

	TIMER_CALLBACK_MEMBER(pts_edge);

	void restart();
	void update_busy();
	void set_pts(int state);
	void carriage_step(int dir);
	double head_x() const;
	void fire(int needle);
	u8 keys_held() const;

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
	u8 m_latch;
	bool m_strobe;
	bool m_init;
	int m_autofd;           // /AUTO FEED XT level
	bool m_strobe_latch;    // set by /STROBE, cleared by /ACK; holds BUSY and drives INT1

	// outputs
	u8 m_pa;
	u8 m_pb;
	u8 m_pc;
	u8 m_p2;

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


class epson_jx80_device : public epson_fx80_device
{
public:
	// construction/destruction
	epson_jx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	virtual void update_ribbon() override;

private:
	void jx80_mem(address_map &map) ATTR_COLD;
	u8 vxob_r();
	void vxob_w(u8 data);

	u8 m_vxob;
	int m_ribbon_pos;       // shift motor steps from the black band
	int m_ribbon_phase;
};


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START( epson_fx80 )
	ROM_REGION(0x4000, "maincpu", 0)  // 16K rom for upd7810 FX-80
	// this is from an fx80+ (found with a M20214GA which according to the tech manual goes with an fx80+)
	ROM_LOAD("epson_8426k9_m1206ba029_read_as_27c128.bin", 0x0000, 0x4000, CRC(ff5c2b1e) SHA1(e1e38c3e4864e60f701939e23331360b76603a24))

	ROM_REGION(0x800, "slavecpu", 0)  // 2K rom for 8042
	ROM_LOAD("epson_fx_c42040kb_8042ah.bin", 0x0000, 0x800, CRC(3e9d08c1) SHA1(d5074f60497cc75d40996e6cef63231d3a3697f1))
ROM_END

ROM_START( epson_jx80 )
	ROM_REGION(0x4000, "maincpu", 0)  // JX-80 rom
	ROM_LOAD("jx80_a4_fs5_27128.bin", 0x0000, 0x4000, CRC(2925a47b) SHA1(1864d3561491d7dca78ac2cd13a023460f551184))

	ROM_REGION(0x800, "slavecpu", 0)  // 2K rom for 8042
	ROM_LOAD("epson_fx_c42040kb_8042ah.bin", 0x0000, 0x800, CRC(3e9d08c1) SHA1(d5074f60497cc75d40996e6cef63231d3a3697f1))
ROM_END

const tiny_rom_entry *epson_fx80_device::device_rom_region() const
{
	return ROM_NAME(epson_fx80);
}

const tiny_rom_entry *epson_jx80_device::device_rom_region() const
{
	return ROM_NAME(epson_jx80);
}


//**************************************************************************
//  INPUT PORTS
//**************************************************************************

static INPUT_PORTS_START( epson_fx80 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-80 ON LINE")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-80 FORM FEED")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-80 LINE FEED")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("FX-80 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_fx80_device::power_switch), 0)

	PORT_START("DSW1")  // read by the 8042, ON = 0
	PORT_DIPNAME( 0x01, 0x01, "Characters per line" )        PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(    0x00, "132 (compressed)" )
	PORT_DIPSETTING(    0x01, "80" )
	PORT_DIPNAME( 0x02, 0x02, "Zero" )                       PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(    0x00, "Slashed" )
	PORT_DIPSETTING(    0x02, "Not slashed" )
	PORT_DIPNAME( 0x04, 0x04, "Paper-end detector" )         PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x04, "Enabled" )
	PORT_DIPNAME( 0x08, 0x00, "Input buffer" )               PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x00, "2 KB" )
	PORT_DIPSETTING(    0x08, "None (download characters)" )
	PORT_DIPNAME( 0x10, 0x10, "Print mode" )                 PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(    0x00, "Emphasized" )
	PORT_DIPSETTING(    0x10, "Normal" )
	PORT_DIPNAME( 0xe0, 0x00, "International characters" )   PORT_DIPLOCATION("SW1:6,7,8")
	PORT_DIPSETTING(    0x00, "USA" )
	PORT_DIPSETTING(    0x20, "France" )
	PORT_DIPSETTING(    0x40, "Germany" )
	PORT_DIPSETTING(    0x60, "England" )
	PORT_DIPSETTING(    0x80, "Denmark" )
	PORT_DIPSETTING(    0xa0, "Sweden" )
	PORT_DIPSETTING(    0xc0, "Italy" )
	PORT_DIPSETTING(    0xe0, "Spain" )

	PORT_START("DSW2")  // read through the A/D converter, functions from the firmware
	PORT_DIPNAME( 0x01, 0x00, "Page length" )
	PORT_DIPSETTING(    0x00, "11\"" )
	PORT_DIPSETTING(    0x01, "12\"" )
	PORT_DIPNAME( 0x02, 0x00, "Skip over perforation" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "Cut sheet mode" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "Wait for DC1 after reset" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "Auto line feed with CR" )     PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x02, "LINE FEED (self test)" )
	PORT_CONFSETTING(    0x04, "FORM FEED (hex dump)" )
INPUT_PORTS_END

ioport_constructor epson_fx80_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(epson_fx80);
}


//**************************************************************************
//  DEVICE
//**************************************************************************

epson_fx80_device::epson_fx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	epson_fx80_device(mconfig, EPSON_FX80, tag, owner, clock)
{
}

epson_fx80_device::epson_fx80_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_slavecpu(*this, "slavecpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_ribbon_band(0),
	m_beep(*this, "beep"),
	m_panel(*this, "PANEL"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "fx80_online_led"),
	m_ready_led(*this, "fx80_ready_led"),
	m_paper_led(*this, "fx80_paper_led"),
	m_pts_timer(nullptr),
	m_data(0),
	m_latch(0),
	m_strobe(true),
	m_init(true),
	m_autofd(1),
	m_strobe_latch(false),
	m_pa(0xff),
	m_pb(0xff),
	m_pc(0xff),
	m_p2(0xff),
	m_cr_pos(-20),
	m_cr_phase(0),
	m_cr_dir(1),
	m_pts(0),
	m_pts_pending(0),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void epson_fx80_device::fx80_mem(address_map &map)
{
	map(0x0000, 0x3fff).rom().region("maincpu", 0);
	map(0x8000, 0x97ff).ram(); // external RAM 4K + 2K = 0x1800
	map(0xd000, 0xd001).mirror(0x07fe).rw(FUNC(epson_fx80_device::slave_r), FUNC(epson_fx80_device::slave_w));
	map(0xd800, 0xd800).mirror(0x07ff).r(FUNC(epson_fx80_device::cent_data_r));
}

void epson_fx80_device::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 10_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_fx80_device::fx80_mem);
	m_maincpu->pa_out_cb().set(FUNC(epson_fx80_device::pa_w));
	m_maincpu->pb_in_cb().set(FUNC(epson_fx80_device::pb_r));
	m_maincpu->pb_out_cb().set(FUNC(epson_fx80_device::pb_w));
	m_maincpu->pc_in_cb().set_constant(0xff);
	m_maincpu->pc_out_cb().set(FUNC(epson_fx80_device::pc_w));
	m_maincpu->an0_func().set_constant(0x80);   // needle pulse width compensation
	m_maincpu->an1_func().set(FUNC(epson_fx80_device::an1_r));
	m_maincpu->an2_func().set_constant(0x80);   // supervised voltage, in range
	m_maincpu->an3_func().set(FUNC(epson_fx80_device::an3_r));
	m_maincpu->an4_func().set(FUNC(epson_fx80_device::an4_r));
	m_maincpu->an5_func().set(FUNC(epson_fx80_device::an5_r));
	m_maincpu->an6_func().set(FUNC(epson_fx80_device::an6_r));
	m_maincpu->an7_func().set(FUNC(epson_fx80_device::an7_r));

	I8042AH(config, m_slavecpu, 11_MHz_XTAL);
	m_slavecpu->p1_in_cb().set(FUNC(epson_fx80_device::slave_p1_r));
	m_slavecpu->p2_in_cb().set_constant(0xff);
	m_slavecpu->p2_out_cb().set(FUNC(epson_fx80_device::slave_p2_w));
	m_slavecpu->t0_in_cb().set(FUNC(epson_fx80_device::home_r));
	m_slavecpu->t1_in_cb().set(FUNC(epson_fx80_device::pts_r));


	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_epson_fx80);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 2000).add_route(ALL_OUTPUTS, "mono", 0.3);
}

void epson_fx80_device::device_start()
{

	m_pts_timer = timer_alloc(FUNC(epson_fx80_device::pts_edge), this);

	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_init));
	save_item(NAME(m_autofd));
	save_item(NAME(m_strobe_latch));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_p2));
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
	save_item(NAME(m_ribbon_band));
}

void epson_fx80_device::device_reset()
{
	m_strobe_latch = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
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

void epson_fx80_device::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_slavecpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_strobe_latch = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	m_power_on = machine().time();
	m_power_cycled = false;
	update_busy();
}

INPUT_CHANGED_MEMBER(epson_fx80_device::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

u8 epson_fx80_device::keys_held() const
{
	// keys held while the printer is switched on with its power switch (bits as in PANEL)
	static u8 const held[4] = { 0x00, 0x04, 0x02, 0x00 };
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return held[(m_config->read() >> 1) & 3];
	return 0;
}


//-------------------------------------------------
//  Centronics
//-------------------------------------------------

void epson_fx80_device::input_strobe(int state)
{
	// the interface latches the data on the falling edge of /STROBE
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_strobe_latch = true;
		m_maincpu->set_input_line(UPD7810_INTF1, ASSERT_LINE);
		update_busy();
	}
	m_strobe = state;
}

void epson_fx80_device::input_init(int state)
{
	// /INIT resets the printer controller
	if (m_init && !state)
		restart();
	m_init = state;
}

u8 epson_fx80_device::cent_data_r()
{
	return m_latch;
}

void epson_fx80_device::update_busy()
{
	bool const busy = BIT(m_pb, 0) || m_strobe_latch;
	output_busy(busy);
	m_ready_led = !busy;
}


//-------------------------------------------------
//  main CPU
//-------------------------------------------------

u8 epson_fx80_device::slave_r(offs_t offset)
{
	return m_slavecpu->upi41_master_r(offset);
}

void epson_fx80_device::slave_w(offs_t offset, u8 data)
{
	// pass offset and data packed into param
	machine().scheduler().synchronize(timer_expired_delegate(FUNC(epson_fx80_device::slave_write_sync), this), (offset << 8) | data);
}

TIMER_CALLBACK_MEMBER(epson_fx80_device::slave_write_sync)
{
	m_slavecpu->upi41_master_w(u32(param) >> 8, u32(param) & 0xff);
}

void epson_fx80_device::pa_w(u8 data)
{
	// needles 1..8, active low: a falling bit fires its needle
	u8 const fired = m_pa & ~data;
	m_pa = data;
	for (int i = 0; i < 8; i++)
		if (BIT(fired, 7 - i))
			fire(i);
}

u8 epson_fx80_device::pb_r()
{
	u8 const keys = m_panel->read() & ~keys_held();
	u8 data = 0xff;
	// PB4: 1 = line feed after CR, from /AUTO FEED XT or DIP switch 2-4
	if (m_autofd && !BIT(m_dsw2->read(), 4))
		data &= ~0x10;
	if (!BIT(keys, 1))
		data &= ~0x20;      // FORM FEED
	if (!BIT(keys, 2))
		data &= ~0x40;      // LINE FEED
	return data;
}

void epson_fx80_device::pb_w(u8 data)
{
	u8 const old = m_pb;
	m_pb = data;

	// needle 9
	if (BIT(old, 7) && !BIT(data, 7))
		fire(8);

	// the /ACK pulse also clears the strobe latch
	if (BIT(old, 1) != BIT(data, 1))
	{
		output_ack(BIT(data, 1));
		if (!BIT(data, 1) && m_strobe_latch)
		{
			m_strobe_latch = false;
			m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
		}
	}
	if ((old ^ data) & 0x03)
		update_busy();
}

void epson_fx80_device::pc_w(u8 data)
{
	u8 const old = m_pc;
	m_pc = data;

	// paper stepper: Gray code on PC7:PC6, energised by PC2
	if ((old ^ data) & 0xc0)
	{
		int const phase = gray_phase[data >> 6];
		if (BIT(data, 2) && m_pf_phase >= 0)
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
}

u8 epson_fx80_device::an1_r()
{
	return BIT(m_dsw2->read(), 3) ? 0x00 : 0xff;
}

u8 epson_fx80_device::an3_r()
{
	bool const paper = BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
	m_paper_led = !paper;
	output_perror(!paper);
	return paper ? 0xff : 0x00;
}

u8 epson_fx80_device::an4_r()
{
	return BIT(m_dsw2->read(), 0) ? 0xff : 0x00;
}

u8 epson_fx80_device::an5_r()
{
	return BIT(m_dsw2->read(), 2) ? 0x00 : 0xff;
}

u8 epson_fx80_device::an6_r()
{
	return BIT(m_dsw2->read(), 1) ? 0x00 : 0xff;
}

u8 epson_fx80_device::an7_r()
{
	return BIT(m_panel->read() & ~keys_held(), 0) ? 0xff : 0x00;
}


//-------------------------------------------------
//  slave CPU
//-------------------------------------------------

u8 epson_fx80_device::slave_p1_r()
{
	return m_dsw1->read();
}

void epson_fx80_device::slave_p2_w(u8 data)
{
	u8 const old = m_p2;
	m_p2 = data;

	// carriage stepper: Gray code on P21:P20, energised unless P24..P22 are all 1
	if ((old ^ data) & 0x03)
	{
		int const phase = gray_phase[data & 3];
		int const delta = (phase - m_cr_phase) & 3;
		m_cr_phase = phase;
		if (((data & 0x1c) != 0x1c) && (delta == 1 || delta == 3))
			carriage_step((delta == 1) ? 1 : -1);
	}

	if ((old ^ data) & 0x80)
		m_beep->set_state(!BIT(data, 7));
	m_online_led = BIT(data, 6);
	output_select(BIT(data, 6));
	output_fault(!BIT(data, 5));
}

int epson_fx80_device::home_r()
{
	return (m_cr_pos < 0) ? 0 : 1;
}

int epson_fx80_device::pts_r()
{
	return m_pts;
}


//-------------------------------------------------
//  carriage and print timing sensor
//-------------------------------------------------

void epson_fx80_device::set_pts(int state)
{
	// the PTS signal goes to the 8042 T1 input and to the uPD7810 CI input
	m_pts = state;
	m_maincpu->set_input_line(UPD7810_CI_LINE, state);
}

TIMER_CALLBACK_MEMBER(epson_fx80_device::pts_edge)
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

void epson_fx80_device::carriage_step(int dir)
{
	attotime const now = machine().time();
	attotime const since = now - m_cr_last_step;

	// one PTS cycle per step; edges still due for the previous step are produced right away
	if (m_pts_pending == 2)
	{
		set_pts(1);
		m_pts_pending = 1;
		m_pts_fall = attotime::from_usec(60);
		m_pts_timer->adjust(m_pts_fall);
	}

	m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
	m_cr_last_step = now;
	m_cr_dir = dir;
	m_cr_pos = std::clamp(m_cr_pos + dir, CR_LEFT, CR_RIGHT);
	m_bitmap_printer->setheadpos(int((m_cr_pos / 60.0 + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);

	// the sensor slot passes halfway through the step and stays open for 30% of it
	attotime const period = m_cr_period.is_never() ? attotime::from_msec(3) : std::min(m_cr_period, attotime::from_msec(3));
	attotime rise = period / 2;
	if (m_pts_pending)
	{
		// finish the previous cycle first
		if (m_pts_pending == 1)
		{
			set_pts(0);
			m_pts_pending = 0;
		}
		rise = std::max(rise, attotime::from_usec(120));
	}
	m_pts_fall = std::max(attotime::from_usec(100), period * 3 / 10);
	m_pts_pending = 2;
	m_pts_timer->adjust(rise);
}

double epson_fx80_device::head_x() const
{
	// a step starts the move to the new position, which is reached at the
	// next step when the carriage runs at constant speed
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - m_cr_dir * (1.0 - frac + CR_LAG);
}

void epson_fx80_device::fire(int needle)
{
	update_ribbon();

	int const x = int(std::lround((head_x() / 60.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles are 1/72" = 3 paper steps apart; colours mix on the paper
	int const y = m_bitmap_printer->m_ypos + needle * (VDPI / 72);
	u32 const ink = ink_color();
	u32 color = ink;
	if (ink)
	{
		u32 const paper = m_bitmap_printer->get_pixel(x, y) & 0xffffff;
		color = 0;
		for (int shift = 0; shift < 24; shift += 8)
			color |= ((((paper >> shift) & 0xff) * ((ink >> shift) & 0xff)) / 255) << shift;
	}
	m_bitmap_printer->draw_pixel(x, y, color);
}


//-------------------------------------------------
//  JX-80
//-------------------------------------------------

epson_jx80_device::epson_jx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	epson_fx80_device(mconfig, EPSON_JX80, tag, owner, clock),
	m_vxob(0),
	m_ribbon_pos(0),
	m_ribbon_phase(0)
{
}

void epson_jx80_device::jx80_mem(address_map &map)
{
	fx80_mem(map);
	map(0x6000, 0x7fff).ram(); // external RAM VXOB board
	map(0xc800, 0xc800).r(FUNC(epson_jx80_device::vxob_r));
	map(0xc801, 0xc801).w(FUNC(epson_jx80_device::vxob_w));
}

void epson_jx80_device::device_add_mconfig(machine_config &config)
{
	epson_fx80_device::device_add_mconfig(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_jx80_device::jx80_mem);
}

void epson_jx80_device::device_start()
{
	epson_fx80_device::device_start();
	save_item(NAME(m_vxob));
	save_item(NAME(m_ribbon_pos));
	save_item(NAME(m_ribbon_phase));
}

void epson_jx80_device::device_reset()
{
	epson_fx80_device::device_reset();
	update_ribbon();
}

u8 epson_jx80_device::vxob_r()
{
	// bit 7: colour ribbon home sensor, 1 = black band
	return (m_ribbon_pos <= RIBBON_BAND / 2) ? 0x80 : 0x00;
}

void epson_jx80_device::vxob_w(u8 data)
{
	u8 const old = m_vxob;
	m_vxob = data;

	// ribbon shift stepper: Gray code on bits 7-6, energised by bit 5
	if ((old ^ data) & 0xc0)
	{
		int const phase = gray_phase[data >> 6];
		int const delta = (phase - m_ribbon_phase) & 3;
		m_ribbon_phase = phase;
		if (BIT(data, 5) && (delta == 1 || delta == 3))
		{
			m_ribbon_pos = std::clamp(m_ribbon_pos + ((delta == 1) ? 1 : -1), -RIBBON_BAND, 4 * RIBBON_BAND);
			update_ribbon();
		}
	}
}

void epson_jx80_device::update_ribbon()
{
	m_ribbon_band = std::clamp((m_ribbon_pos + RIBBON_BAND / 2) / RIBBON_BAND, 0, 3);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_FX80, device_centronics_peripheral_interface, epson_fx80_device, "epson_fx80", "Epson FX-80")
DEFINE_DEVICE_TYPE_PRIVATE(EPSON_JX80, device_centronics_peripheral_interface, epson_jx80_device, "epson_jx80", "Epson JX-80")
