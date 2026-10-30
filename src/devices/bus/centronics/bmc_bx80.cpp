// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    BMC BX-80 / Microbee Admate DP-80 9-pin dot matrix printer

    Shinwa Digital board 21100188 (1984): 8039 with 8 KiB program ROM,
    8155 RAM/IO/timer, 8041 slave for the motors, gate arrays D6101C-045
    and D6101C-048.

    Main CPU 8039 (8 MHz):
      P20 low, INS A,BUS: paper sensor and panel keys (gate array 1)
      P21 low, INS A,BUS: DIP switch SW1
      P22 low: 8041 (P25 = A0), P23 low: 8155 (P25 = IO/M)
      P24: A12 of the program ROM, inverted
      P10 = ON LINE lamp, P27 = READY lamp (both toggle with the ON LINE key)
      P11-P14 <- slave P21, P23, P20, P22; P15 = /ACK, P16 = BUSY; P17 <- SW2-3
      INT <- 8155 PC0 (INTR A)
    8155: PA = Centronics data (strobed input), PB7-PB0 = needles 1-8,
      PC4 = needle 9 (all active low)
    Slave 8041 (6 MHz): P10/P11 carriage stepper, P14/P15 paper feed stepper,
      P17 = buzzer; T0 = PTS, T1 = home; P20-P23 status to the main CPU,
      P24 <- SW2-4

    The panel lamp drive, the buzzer frequency and the PTS and home sensor
    timing are not known from the schematic and are modelled.

    TODO:
    - paper-out sensor (always reports paper)
    - SW2-1/SW2-2 wiring (not used by the firmware according to the manual)

**********************************************************************/

#include "emu.h"
#include "bmc_bx80.h"

#include "cpu/mcs48/mcs48.h"
#include "machine/bitmap_printer.h"
#include "machine/i8155.h"
#include "sound/beep.h"

#include "speaker.h"

#include "bmc_bx80.lh"

#define LOG_IO    (1U << 1)
#define LOG_PORT  (1U << 2)
#define LOG_SLAVE (1U << 3)

//#define VERBOSE (LOG_GENERAL | LOG_IO | LOG_PORT | LOG_SLAVE)
#include "logmacro.h"


namespace {

constexpr int gray_phase[4] = { 0, 1, 3, 2 };
constexpr int HDPI = 240;
constexpr int VDPI = 216;
constexpr double PAPER_WIDTH = 9.5;
constexpr double PAPER_HEIGHT = 11.0;
constexpr double CR_STEPS_PER_INCH = 640.0 / 7.5;   // manual: 640 dots in a 7.5 inch line
constexpr double LEFT_OFFSET = 0.75;         // paper edge to the home sensor edge (guess)
constexpr double CR_MOVE = 0.00104;     // time the carriage takes for a step (guess, as FX-80)
constexpr double DOT_LAG = 1.13;        // steps the dot lands behind the step position while moving:
                                        // stepper load angle less needle flight, fitted so both directions line up
constexpr int NEEDLE_MIN = 150;        // shorter drive pulses do not reach the paper (guess)
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 720;           // right stop

class bmc_bx80_device : public device_t, public device_centronics_peripheral_interface
{
public:
	bmc_bx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_strobe(int state) override { m_strobe = state; m_8155->stb_a_w(state); }
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
	void prog_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 io_r(offs_t offset);
	u8 bus_r();
	void slave_p1_w(u8 data);
	void slave_p2_w(u8 data);
	void carriage_step(int dir);
	void needles_w(u16 needles);
	double head_x() const;
	void set_pts(int state);
	TIMER_CALLBACK_MEMBER(pts_edge);
	void io_w(offs_t offset, u8 data);

	required_device<i8039_device> m_maincpu;
	required_device<i8041a_device> m_slavecpu;
	required_device<i8155_device> m_8155;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_region_ptr<u8> m_rom;
	required_device<beep_device> m_beep;
	required_ioport m_panel;
	required_ioport m_sw1;
	required_ioport m_sw2;
	output_finder<4> m_led;

	u8 m_data;
	int m_strobe;
	u8 m_p1;
	u8 m_p2;
	u8 m_s1;
	u8 m_s2;

	emu_timer *m_pts_timer;
	int m_pts;
	int m_pts_pending;
	attotime m_pts_fall;
	int m_cr_pos;
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pf_phase;
	int m_pf_pos;
	u8 m_pb;
	u8 m_pc;
	u16 m_needles;
	attotime m_fire_t[9];
	double m_fire_x[9];
};


ROM_START( bx80 )
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("m5m2364.bin", 0x0000, 0x2000, CRC(3f6608ae) SHA1(dcd098efa9ee98672fcac206fdd0b15456b594b4))

	ROM_REGION(0x400, "slavecpu", 0)
	ROM_LOAD("d8041ahc-042.bin", 0x000, 0x400, CRC(5c7e08c3) SHA1(cd59306b40b3e48e824a8579927a63a23472b645))
ROM_END

const tiny_rom_entry *bmc_bx80_device::device_rom_region() const
{
	return ROM_NAME( bx80 );
}


INPUT_PORTS_START( bx80 )
	PORT_START("PANEL")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_UNUSED)   // paper sensor, 0 = paper present
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("On Line") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Line Feed") PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Form Feed") PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("SW1")
	PORT_DIPNAME(0x01, 0x01, "Character pitch") PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(   0x01, "10 cpi")
	PORT_DIPSETTING(   0x00, "17.16 cpi (compressed)")
	PORT_DIPUNUSED_DIPLOC(0x02, 0x02, "SW1:2")
	PORT_DIPNAME(0x04, 0x04, "Paper-out detection") PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(   0x04, "Stops printing")
	PORT_DIPSETTING(   0x00, "Ignored")
	PORT_DIPNAME(0x08, 0x08, "Typeface") PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(   0x08, "Normal")
	PORT_DIPSETTING(   0x00, "Italic")
	PORT_DIPNAME(0x10, 0x10, "Print mode") PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(   0x10, "Normal")
	PORT_DIPSETTING(   0x00, "Emphasized")
	PORT_DIPNAME(0x20, 0x00, "Buzzer") PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(   0x20, DEF_STR(Off))
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x40, "Zero") PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(   0x40, "Normal")
	PORT_DIPSETTING(   0x00, "Slashed")
	PORT_DIPNAME(0x80, 0x00, "Select") PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(   0x80, "By host")
	PORT_DIPSETTING(   0x00, "Always selected")

	PORT_START("SW2")
	PORT_DIPUNUSED_DIPLOC(0x01, 0x01, "SW2:1")
	PORT_DIPUNUSED_DIPLOC(0x02, 0x02, "SW2:2")
	PORT_DIPNAME(0x04, 0x04, "Auto LF with CR") PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(   0x04, DEF_STR(Off))
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x08, "Skip over perforation") PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(   0x08, DEF_STR(Off))
	PORT_DIPSETTING(   0x00, "1 inch")
INPUT_PORTS_END

ioport_constructor bmc_bx80_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( bx80 );
}


bmc_bx80_device::bmc_bx80_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, BMC_BX80, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_slavecpu(*this, "slavecpu"),
	m_8155(*this, "i8155"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_rom(*this, "maincpu"),
	m_beep(*this, "beep"),
	m_panel(*this, "PANEL"),
	m_sw1(*this, "SW1"),
	m_sw2(*this, "SW2"),
	m_led(*this, "bx80_led%u", 0U),
	m_data(0),
	m_strobe(1)
{
}

void bmc_bx80_device::prog_map(address_map &map)
{
	// P24 = 0 selects the upper half of the 2764 (A12 inverted)
	map(0x000, 0xfff).lr8(NAME([this] (offs_t offset) { return m_rom[(BIT(~m_p2, 4) << 12) | offset]; }));
}

void bmc_bx80_device::io_map(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(bmc_bx80_device::io_r), FUNC(bmc_bx80_device::io_w));
}

void bmc_bx80_device::device_add_mconfig(machine_config &config)
{
	I8039(config, m_maincpu, 8_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &bmc_bx80_device::prog_map);
	m_maincpu->set_addrmap(AS_IO, &bmc_bx80_device::io_map);
	m_maincpu->p1_in_cb().set([this] () {
		// P11-P14 from the slave P2, P17 = SW2-3
		return (m_p1 & 0x61) | (BIT(m_s2, 1) << 1) | (BIT(m_s2, 3) << 2) | (BIT(m_s2, 0) << 3) | (BIT(m_s2, 2) << 4) | (BIT(m_sw2->read(), 2) << 7);
	});
	m_maincpu->p1_out_cb().set([this] (u8 data) {
		if (data != m_p1) LOGMASKED(LOG_PORT, "%s P1 w %02x\n", machine().describe_context(), data);
		m_p1 = data;
		m_led[1] = BIT(data, 0);     // ON LINE
		output_ack(BIT(data, 5));
		output_busy(BIT(data, 6));
	});
	m_maincpu->p2_in_cb().set([this] () { u8 const d = 0xff; if (!machine().side_effects_disabled()) LOGMASKED(LOG_PORT, "%s P2 r\n", machine().describe_context()); return d; });
	m_maincpu->p2_out_cb().set([this] (u8 data) {
		if (data != m_p2) LOGMASKED(LOG_PORT, "%s P2 w %02x\n", machine().describe_context(), data);
		m_p2 = data;
		m_led[2] = BIT(data, 7);     // READY
	});
	m_maincpu->t0_in_cb().set([this] () { return 1; });
	m_maincpu->t1_in_cb().set([this] () { return m_pts; });   // guess: PTS also to the main CPU
	m_maincpu->bus_in_cb().set(FUNC(bmc_bx80_device::bus_r));

	I8041A(config, m_slavecpu, 6_MHz_XTAL);
	m_slavecpu->p1_in_cb().set([this] () { return m_s1; });
	m_slavecpu->p1_out_cb().set(FUNC(bmc_bx80_device::slave_p1_w));
	m_slavecpu->p2_in_cb().set([this] () { return (m_s2 & 0x0f) | 0xe0 | (BIT(m_sw2->read(), 3) << 4); });   // P24 = SW2-4
	m_slavecpu->p2_out_cb().set(FUNC(bmc_bx80_device::slave_p2_w));
	m_slavecpu->t0_in_cb().set([this] () { return m_pts; });                       // PTS sensor
	m_slavecpu->t1_in_cb().set([this] () { return (m_cr_pos < 0) ? 0 : 1; });   // home sensor (polarity guessed)

	I8155(config, m_8155, 8_MHz_XTAL / 3);
	m_8155->in_pa_callback().set([this] () { return m_data; });
	m_8155->out_pb_callback().set([this] (u8 data) {
		LOGMASKED(LOG_PORT, "%s PB w %02x\n", machine().describe_context(), data);
		m_pb = data;
		needles_w((u16(~m_pb & 0xff) << 1) | BIT(~m_pc, 4));
	});
	m_8155->out_pc_callback().set([this] (u8 data) {
		LOGMASKED(LOG_PORT, "%s PC w %02x\n", machine().describe_context(), data);
		m_pc = data;
		needles_w((u16(~m_pb & 0xff) << 1) | BIT(~m_pc, 4));
		m_maincpu->set_input_line(MCS48_INPUT_IRQ, BIT(data, 0) ? ASSERT_LINE : CLEAR_LINE);   // INTR A (guess: straight to /INT)
	});
	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x03, bitmap_printer_device::PAPER_FANFOLD_11);


	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 2000).add_route(ALL_OUTPUTS, "mono", 0.25);   // frequency guessed

	config.set_default_layout(layout_bmc_bx80);
}

void bmc_bx80_device::device_start()
{
	m_p1 = m_p2 = 0xff;
	m_s1 = m_s2 = 0xff;
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_p1));
	save_item(NAME(m_p2));
	save_item(NAME(m_s1));
	save_item(NAME(m_s2));

	m_pts_timer = timer_alloc(FUNC(bmc_bx80_device::pts_edge), this);
	m_pts = 0;
	m_pts_pending = 0;
	m_cr_pos = 200;
	m_cr_phase = 0;
	m_cr_dir = 1;
	m_cr_period = attotime::never;
	m_cr_last_step = attotime::zero;
	m_pf_phase = 0;
	m_pf_pos = 0;
	m_pb = 0xff;
	m_pc = 0xff;
	m_needles = 0;
	for (int i = 0; i < 9; i++) { m_fire_t[i] = attotime::zero; m_fire_x[i] = 0; }
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_needles));
	save_item(NAME(m_fire_t));
	save_item(NAME(m_fire_x));
	save_item(NAME(m_pts));
	save_item(NAME(m_pts_pending));
	save_item(NAME(m_pts_fall));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_pf_pos));
}

void bmc_bx80_device::slave_p1_w(u8 data)
{
	u8 const old = m_s1;
	m_s1 = data;
	if (data != old)
		LOGMASKED(LOG_SLAVE, "%s S P1 w %02x\n", machine().describe_context(), data);

	// P17 low = buzzer
	m_beep->set_state(BIT(~data, 7));

	// P11:P10 carriage stepper, Gray code
	if ((old ^ data) & 0x03)
	{
		int const phase = gray_phase[data & 3];
		int const delta = (phase - m_cr_phase) & 3;
		m_cr_phase = phase;
		if (delta == 1 || delta == 3)
			carriage_step((delta == 1) ? 1 : -1);
	}

	// P15:P14 paper feed stepper, Gray code
	if ((old ^ data) & 0x30)
	{
		int const phase = gray_phase[(data >> 4) & 3];
		int const delta = (phase - m_pf_phase) & 3;
		m_pf_phase = phase;
		if (delta == 1 || delta == 3)
		{
			m_pf_pos += (delta == 1) ? 1 : -1;
			LOGMASKED(LOG_SLAVE, "PF %d\n", m_pf_pos);
			if (delta == 1)
				m_bitmap_printer->m_ypos++;
			else
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
	}
}

void bmc_bx80_device::slave_p2_w(u8 data)
{
	if (data != m_s2)
		LOGMASKED(LOG_SLAVE, "%s S P2 w %02x\n", machine().describe_context(), data);
	m_s2 = data;
}

void bmc_bx80_device::set_pts(int state)
{
	m_pts = state;
}

TIMER_CALLBACK_MEMBER(bmc_bx80_device::pts_edge)
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

void bmc_bx80_device::carriage_step(int dir)
{
	attotime const now = machine().time();
	attotime const since = now - m_cr_last_step;

	// one PTS cycle per step (model taken from the FX-80)
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
	LOGMASKED(LOG_SLAVE, "CR %d\n", m_cr_pos);
	m_bitmap_printer->setheadpos(int((m_cr_pos / CR_STEPS_PER_INCH + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);

	attotime const period = m_cr_period.is_never() ? attotime::from_double(CR_MOVE) : std::min(m_cr_period, attotime::from_double(CR_MOVE));
	attotime rise = period / 2;
	if (m_pts_pending)
	{
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

void bmc_bx80_device::device_reset()
{
	m_led[0] = 1;   // POWER
	m_led[3] = 0;   // PAPER OUT: the paper sensor always sees paper
	output_perror(0);
}

double bmc_bx80_device::head_x() const
{
	// the carriage moves to the new step position within CR_MOVE
	if (m_cr_period.is_never())
		return m_cr_pos;
	double const move = std::min(m_cr_period.as_double(), CR_MOVE);
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / move);
	return m_cr_pos - m_cr_dir * (1.0 - frac);
}

void bmc_bx80_device::needles_w(u16 needles)
{
	// PB7..PB0 = needles 1..8, PC4 = needle 9, all active low
	u16 const changed = needles ^ m_needles;
	m_needles = needles;
	if (!changed)
		return;
	attotime const now = machine().time();
	for (int pin = 0; pin < 9; pin++)
	{
		if (!BIT(changed, 8 - pin))
			continue;
		if (BIT(needles, 8 - pin))
		{
			m_fire_t[pin] = now;
			m_fire_x[pin] = head_x() - (m_cr_period.is_never() ? 0.0 : m_cr_dir * DOT_LAG);
		}
		else
		{
			attotime const len = now - m_fire_t[pin];
			if (len < attotime::from_usec(NEEDLE_MIN))
				continue;
			int const x = int(std::lround((m_fire_x[pin] / CR_STEPS_PER_INCH + LEFT_OFFSET) * HDPI));
			if (x >= 0 && x < int(PAPER_WIDTH * HDPI))
				m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + pin * (VDPI / 72), 0x000000);
		}
	}
}

u8 bmc_bx80_device::bus_r()
{
	// INS A,BUS: gate array 1 puts switches or sensors on the bus (P20 / P21 low)
	u8 data = 0xff;
	if (!BIT(m_p2, 0))
		data &= m_panel->read();
	if (!BIT(m_p2, 1))
		data &= m_sw1->read();
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s BUS r = %02x P1=%02x P2=%02x\n", machine().describe_context(), data, m_p1, m_p2);
	return data;
}

u8 bmc_bx80_device::io_r(offs_t offset)
{
	u8 data = 0xff;
	if (!BIT(m_p2, 2))
		data &= m_slavecpu->upi41_master_r(BIT(m_p2, 5));   // P22 = /CS, P25 = A0
	if (!BIT(m_p2, 3))
		data &= BIT(m_p2, 5) ? m_8155->io_r(offset) : m_8155->memory_r(offset);   // P23 = /CE, P25 = IO/M
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s IO r %02x = %02x P1=%02x P2=%02x\n", machine().describe_context(), offset, data, m_p1, m_p2);
	return data;
}

void bmc_bx80_device::io_w(offs_t offset, u8 data)
{
	if (!BIT(m_p2, 2))
		m_slavecpu->upi41_master_w(BIT(m_p2, 5), data);
	if (!BIT(m_p2, 3))
	{
		if (BIT(m_p2, 5))
			m_8155->io_w(offset, data);
		else
			m_8155->memory_w(offset, data);
	}
	LOGMASKED(LOG_IO, "%s IO w %02x = %02x P1=%02x P2=%02x\n", machine().describe_context(), offset, data, m_p1, m_p2);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(BMC_BX80, device_centronics_peripheral_interface, bmc_bx80_device, "bx80", "BMC BX-80")
