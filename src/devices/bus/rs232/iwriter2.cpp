// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Apple ImageWriter II 9-pin dot matrix printer (serial, RS-422)

    TEC logic board: TEC EC-A041 custom chip (uPD7807 core with the
    printer control logic), EC-A042, 32 KiB ROM, 8 KiB RAM, AM26LS30/32
    line drivers, DIP switches SW1 (8) and SW2 (4 + 2) read through 74LS158.
    Connects to the printer port of the Macintosh (SCC channel B, HSKi = CTS)
    or any other serial port.

    The EC-A041's printer logic is not documented; its model here is derived
    from the firmware (ROMREV 04).  Crystal frequency not verified.

      0000-7fff   ROM
      8000-9fff   option card (AppleTalk, 32K buffer), not emulated
      a000-a0ff   RAM, dot column ring buffer read by the carriage engine
      a100-a1ff   RAM, carriage step period table read by the carriage engine
      a200-bfff   RAM (b800-bfff: input buffer)
      fe00-fe7f   EC-A041 registers
      ff00-ffff   uPD7807 internal RAM

      PA0-3       paper motor phases, latched by a write to fe01 (fe00: off)
      PA4         error lamp
      PA5         select lamp
      PA6/PA7     print quality lamps (draft PA7, correspondence PA6, NLQ both)
      PB0-3       colour ribbon shift motor phases, latched by a write to fe03
      PC6         handshake output (DTR), high = busy
      PT0-3       keys select, line feed, form feed, print quality (low = pressed)
      PT4         front cover (?), PT5 paper sensor (low = paper),
      PT6         colour ribbon (low), PT7 unknown
      TI          614.4 kHz baud clock
      fe04 read   SW2 (inverted, low nibble)
      fe60 read   SW1 (inverted)

    Carriage engine (fe60-fe76), as far as the firmware shows it:
      fe60/61     first step period word; the table at a100 follows, one word
                  per step: bit 15 = ramp (take the next word after the step),
                  bits 15/14 clear = cruise (hold until fe64/65 + 2 steps are
                  done), bit 14 = constant speed (every step raises status bit 0),
                  bit 13 = end of the table / dwell: no steps, status bit 0 after
                  each period; bits 11-0 = period in engine clocks
      fe62 / fe63 start / stop
      fe64/65     steps before the deceleration (also written during a move)
      fe66/67     distance from the start to the first dot column, 1/2880 inch
                  (ffff = no printing)
      fe72        bit 0 run, bit 2 towards the home sensor (left), others ?
      fe61 read   bits 0-3 status (acknowledged by writes to fe6c-fe6f):
                  0 end of ramp / dwell / step, 1 end of the dot data,
                  2 half of the ring buffer read; bit 4 = 0 in the home zone
      INT1        status bits 0-3
    Dot columns are 16-bit words in the ring buffer starting at a000: low byte
    pins 1-8, high byte bit 0 pin 9 (all active low), bits 6-1 distance to the
    next column - 1 in 1/2880 inch, bit 7 clear marks the end.
    Not understood yet: fe68, fe6a/6b, fe70/71 (hammer timing?), fe73-fe76.

**********************************************************************/

#include "emu.h"
#include "iwriter2.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"

#include "iwriter2.lh"

#define LOG_IO   (1U << 1)
#define LOG_ASIC (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_IO | LOG_ASIC)
#include "logmacro.h"


namespace {

// carriage engine time base (guess: crystal / 64, gives about 250 cps in draft)
constexpr u32 CR_CLOCK = 14'745'600 / 64;

// head positions are kept in 1/2880 inch, the unit of the firmware; a carriage step is 1/160 inch
constexpr int UNITS_PER_INCH = 2880;
constexpr int UNITS_PER_STEP = 18;
// the head lags behind the motor by a constant distance and a time; both are fitted to the
// bidirectional alignment the firmware gives in draft, correspondence and NLQ
constexpr double CR_SHIFT = 18.8;       // units
constexpr double CR_LAG = 0.000153;     // seconds
// colour ribbon bands, from the black band at the lower end of the shift travel
constexpr u32 ribbon_color[4] = { 0x000000, 0x2c74c8, 0xd8306c, 0xf0d028 };   // black, blue, red, yellow
constexpr int RIBBON_BAND = 13;         // shift motor steps per band

constexpr int HOME_EDGE = -20;          // home sensor edge, steps from the carriage zero position (guess)

// paper
constexpr double PAPER_WIDTH = 10.0;    // inches
constexpr double PAPER_HEIGHT = 11.0;
constexpr int HDPI = 320;               // 9 units per pixel
constexpr int VDPI = 144;               // paper feed step
constexpr double LEFT_OFFSET = 0.4;     // paper edge to the carriage zero position, inches (guess)

class iwriter2_device : public device_t, public device_rs232_port_interface
{
public:
	iwriter2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override { m_rxd = state; }

	DECLARE_INPUT_CHANGED_MEMBER(power_switch);
	void set_power(bool on);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 option_r(offs_t offset);
	void option_w(offs_t offset, u8 data);
	u8 asic_r(offs_t offset);
	void asic_w(offs_t offset, u8 data);
	u8 pt_r();
	void pa_w(u8 data);
	void pc_w(u8 data);

	TIMER_CALLBACK_MEMBER(cr_tick);
	void dots_step();
	// fe64/fe65: steps before the deceleration, the firmware extends a running move
	int cruise_steps() const { return (m_asic[0x64] | ((m_asic[0x65] & 0x0f) << 8)) + 2; }
	void fire(u16 w, double pos);
	void update_head();
	void paper_step(u8 pattern);
	void ribbon_step(u8 pattern);
	u32 ink_color() const;
	void update_irq();

	required_device<upd7807_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	output_finder<> m_power_led;
	output_finder<> m_error_led;
	output_finder<> m_select_led;
	output_finder<> m_draft_led;
	output_finder<> m_quality_led;
	required_ioport m_panel;
	required_ioport m_sw1;
	required_ioport m_sw2;
	required_ioport m_sensors;

	emu_timer *m_cr_timer;

	int m_rxd;
	u8 m_pa;
	u8 m_pb;
	u8 m_pc;
	bool m_power;
	u8 m_asic[0x80];
	u8 m_irq_status;    // fe61 bits 0-3
	int m_cr_pos;       // carriage position, steps right of the home sensor edge
	bool m_cr_run;
	u16 m_cr_word;      // current step period word
	u16 m_cr_ptr;       // ramp table pointer
	int m_cr_count;     // steps since the start
	bool m_dot_run;
	u8 m_dot_ptr;       // ring buffer offset
	int m_dot_units;    // head position in dot sub-units since the start of the dot data
	int m_dot_next;     // position of the next column
	int m_dot_cols;
	int m_dot_min, m_dot_max;   // logging
	int m_dot_origin;   // head position of the start of the dot data, units
	int m_dot_dir;
	int m_pf_phase;     // paper motor half-step position
	int m_pf_halfsteps; // paper position in half steps
	int m_rb_phase;     // ribbon shift motor phase
	int m_rb_pos;       // ribbon shift position, steps from the black band
};


ROM_START( iwriter2 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("iwriter2.bin", 0x0000, 0x8000, CRC(8f5d3363) SHA1(e7dee1661cd0fa72350b32a60abd4625a4f3ccff))
ROM_END

const tiny_rom_entry *iwriter2_device::device_rom_region() const
{
	return ROM_NAME( iwriter2 );
}


INPUT_PORTS_START( iwriter2 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("ImageWriter II Select")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("ImageWriter II Line Feed")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("ImageWriter II Form Feed")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("ImageWriter II Print Quality")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("ImageWriter II On/Off") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(iwriter2_device::power_switch), 0)

	PORT_START("SENSORS")   // port T bits 4-7
	PORT_CONFNAME( 0x10, 0x10, "Front cover" )  // guess: printing stops when low
	PORT_CONFSETTING(    0x00, "Open" )
	PORT_CONFSETTING(    0x10, "Closed" )
	PORT_CONFNAME( 0x20, 0x00, "Paper" )
	PORT_CONFSETTING(    0x00, "Loaded" )
	PORT_CONFSETTING(    0x20, "Out" )
	PORT_CONFNAME( 0x40, 0x40, "Ribbon" )
	PORT_CONFSETTING(    0x40, "Black" )
	PORT_CONFSETTING(    0x00, "Colour" )
	PORT_DIPNAME( 0x80, 0x80, "PT7" )           // unknown, read at power-on
	PORT_DIPSETTING(    0x00, "0" )
	PORT_DIPSETTING(    0x80, "1" )

	// DIP switches as in the Technical Reference Manual, table 2-1 (1 = closed/ON, as printed by
	// the self test); defaults are the US factory settings, DIPSW1(00000100) DIPSW2(1100)
	PORT_START("SW1")
	PORT_DIPNAME( 0x07, 0x00, "Language" )                      PORT_DIPLOCATION("SW1:!1,!2,!3")
	PORT_DIPSETTING(    0x00, "American" )
	PORT_DIPSETTING(    0x01, "Italian" )
	PORT_DIPSETTING(    0x02, "Danish" )
	PORT_DIPSETTING(    0x03, "British" )
	PORT_DIPSETTING(    0x04, "German" )
	PORT_DIPSETTING(    0x05, "Swedish" )
	PORT_DIPSETTING(    0x06, "French" )
	PORT_DIPSETTING(    0x07, "Spanish" )
	PORT_DIPNAME( 0x08, 0x00, "Form length" )                   PORT_DIPLOCATION("SW1:!4")
	PORT_DIPSETTING(    0x00, "11 inches" )
	PORT_DIPSETTING(    0x08, "12 inches" )
	PORT_DIPNAME( 0x10, 0x00, "Perforation skip" )              PORT_DIPLOCATION("SW1:!5")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x60, 0x20, "Character pitch" )               PORT_DIPLOCATION("SW1:!6,!7")
	PORT_DIPSETTING(    0x00, "10 cpi (Pica)" )
	PORT_DIPSETTING(    0x20, "12 cpi (Elite)" )
	PORT_DIPSETTING(    0x40, "17 cpi (Ultracondensed)" )
	PORT_DIPSETTING(    0x60, "160 dpi (Elite proportional)" )
	PORT_DIPNAME( 0x80, 0x00, "Line feed after CR" )            PORT_DIPLOCATION("SW1:!8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("SW2")   // 2-5 and 2-6 (hammer timing) are not read through fe04
	PORT_DIPNAME( 0x03, 0x03, "Baud rate" )                     PORT_DIPLOCATION("SW2:!1,!2")
	PORT_DIPSETTING(    0x00, "300" )
	PORT_DIPSETTING(    0x01, "1200" )
	PORT_DIPSETTING(    0x02, "2400" )
	PORT_DIPSETTING(    0x03, "9600" )
	PORT_DIPNAME( 0x04, 0x00, "Protocol" )                      PORT_DIPLOCATION("SW2:!3")
	PORT_DIPSETTING(    0x00, "Hardware handshake" )
	PORT_DIPSETTING(    0x04, "XON/XOFF" )
	PORT_DIPNAME( 0x08, 0x00, "Option card" )                   PORT_DIPLOCATION("SW2:!4")   // no option card is emulated, enabling it hangs the firmware
	PORT_DIPSETTING(    0x00, "Disabled" )
	PORT_DIPSETTING(    0x08, "Enabled" )
INPUT_PORTS_END

ioport_constructor iwriter2_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( iwriter2 );
}


iwriter2_device::iwriter2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, APPLE_IMAGEWRITER2, tag, owner, clock),
	device_rs232_port_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_power_led(*this, "iw2_power_led"),
	m_error_led(*this, "iw2_error_led"),
	m_select_led(*this, "iw2_select_led"),
	m_draft_led(*this, "iw2_draft_led"),
	m_quality_led(*this, "iw2_quality_led"),
	m_panel(*this, "PANEL"),
	m_sw1(*this, "SW1"),
	m_sw2(*this, "SW2"),
	m_sensors(*this, "SENSORS"),
	m_rxd(1),
	m_pa(0),
	m_pb(0),
	m_pc(0),
	m_power(true),
	m_irq_status(0),
	m_cr_pos(100),
	m_cr_run(false),
	m_cr_word(0),
	m_cr_ptr(0),
	m_cr_count(0),
	m_dot_run(false),
	m_dot_ptr(0),
	m_dot_units(0),
	m_dot_next(0),
	m_dot_cols(0),
	m_dot_origin(0),
	m_dot_dir(1),
	m_pf_phase(0),
	m_pf_halfsteps(0),
	m_rb_phase(0),
	m_rb_pos(0)
{
}

void iwriter2_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0x9fff).rw(FUNC(iwriter2_device::option_r), FUNC(iwriter2_device::option_w));
	map(0xa000, 0xbfff).ram();
	map(0xfe00, 0xfe7f).rw(FUNC(iwriter2_device::asic_r), FUNC(iwriter2_device::asic_w));
}

void iwriter2_device::device_add_mconfig(machine_config &config)
{
	UPD7807(config, m_maincpu, 14.7456_MHz_XTAL);   // crystal not verified
	m_maincpu->set_addrmap(AS_PROGRAM, &iwriter2_device::mem_map);
	m_maincpu->set_ti_clock(14.7456_MHz_XTAL / 24);   // 614.4 kHz baud clock
	m_maincpu->pa_out_cb().set(FUNC(iwriter2_device::pa_w));
	m_maincpu->pb_out_cb().set([this] (u8 data) { m_pb = data; });
	m_maincpu->pc_out_cb().set(FUNC(iwriter2_device::pc_w));
	m_maincpu->pc_in_cb().set([this] () { return u8(0xff); });
	m_maincpu->pt_in_cb().set(FUNC(iwriter2_device::pt_r));
	m_maincpu->txd_func().set([this] (int state) { output_rxd(state); });
	m_maincpu->rxd_func().set([this] () { return m_rxd; });

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);

	config.set_default_layout(layout_iwriter2);
}

void iwriter2_device::device_start()
{

	m_cr_timer = timer_alloc(FUNC(iwriter2_device::cr_tick), this);

	std::fill(std::begin(m_asic), std::end(m_asic), 0);

	save_item(NAME(m_rxd));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_power));
	save_item(NAME(m_asic));
	save_item(NAME(m_irq_status));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_run));
	save_item(NAME(m_cr_word));
	save_item(NAME(m_cr_ptr));
	save_item(NAME(m_cr_count));
	save_item(NAME(m_dot_run));
	save_item(NAME(m_dot_ptr));
	save_item(NAME(m_dot_units));
	save_item(NAME(m_dot_next));
	save_item(NAME(m_dot_cols));
	save_item(NAME(m_dot_origin));
	save_item(NAME(m_dot_dir));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_pf_halfsteps));
	save_item(NAME(m_rb_phase));
	save_item(NAME(m_rb_pos));
}

void iwriter2_device::device_reset()
{
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
	m_irq_status = 0;
	m_cr_run = false;
	m_cr_timer->adjust(attotime::never);
	update_irq();
	set_power(m_power);
}

INPUT_CHANGED_MEMBER(iwriter2_device::power_switch)
{
	// the on/off key toggles the power; the printer is on when the machine starts
	if (newval)
		set_power(!m_power);
}

void iwriter2_device::set_power(bool on)
{
	m_power = on;
	m_power_led = on;
	output_dsr(on ? 0 : 1);
	m_maincpu->set_input_line(INPUT_LINE_RESET, on ? CLEAR_LINE : ASSERT_LINE);
	if (!on)
	{
		// the motors stop and the lamps go out; the head stays where it is
		m_cr_run = false;
		m_dot_run = false;
		m_cr_timer->adjust(attotime::never);
		m_irq_status = 0;
		update_irq();
		pa_w(0);
		pc_w(0x40);     // not ready
	}
}

u8 iwriter2_device::pt_r()
{
	return (m_panel->read() & 0x0f) | (m_sensors->read() & 0xf0);
}

void iwriter2_device::pa_w(u8 data)
{
	// PA0-3: paper motor phases (latched by fe01), PA4: error lamp, PA5: select lamp,
	// PA6/PA7: print quality lamps (draft: PA7, correspondence: PA6, NLQ: both)
	m_pa = data;
	m_error_led = BIT(data, 4);
	m_select_led = BIT(data, 5);
	m_quality_led = BIT(data, 6);
	m_draft_led = BIT(data, 7);
}

void iwriter2_device::pc_w(u8 data)
{
	// PC6: handshake output (DTR at the interface), high = buffer full
	if ((data ^ m_pc) & ~0x40)
		LOGMASKED(LOG_IO, "%s PC %02x\n", machine().describe_context(), data);
	m_pc = data;
	output_cts(BIT(data, 6));
}

u8 iwriter2_device::option_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s option_r %04x\n", machine().describe_context(), 0x8000 + offset);
	return 0xff;
}

void iwriter2_device::option_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_IO, "%s option_w %04x = %02x\n", machine().describe_context(), 0x8000 + offset, data);
}

void iwriter2_device::update_irq()
{
	m_maincpu->set_input_line(UPD7810_INTF1, (m_irq_status & 0x0f) ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(iwriter2_device::cr_tick)
{
	// carriage engine (hypothesis, see header)
	u8 const ctrl = m_asic[0x72];
	auto &sp = m_maincpu->space(AS_PROGRAM);

	if ((m_cr_word & 0x2000) || !BIT(ctrl, 0))
	{
		// hold: no step, report the end of the dwell period
		m_irq_status |= 0x01;
	}
	else
	{
		int const dir = BIT(ctrl, 2) ? -1 : 1;
		m_cr_pos += dir;
		m_cr_count++;
		update_head();
		if (m_cr_word & 0x4000)
			m_irq_status |= 0x01;   // constant speed: every step is reported
		dots_step();

		if (m_cr_word & 0x8000)
		{
			m_cr_word = sp.read_word(m_cr_ptr);
			m_cr_ptr += 2;
		}
		else if (!(m_cr_word & 0x4000) && (m_cr_count >= cruise_steps()))
		{
			m_cr_word = sp.read_word(m_cr_ptr);
			m_cr_ptr += 2;
		}
		if (m_cr_word & 0x2000)
			LOGMASKED(LOG_ASIC, "carriage end of ramp after %d steps, pos %d\n", m_cr_count, m_cr_pos);
	}
	update_irq();
	m_cr_timer->adjust(attotime::from_ticks(std::max<u32>(m_cr_word & 0x0fff, 1), CR_CLOCK));
}

void iwriter2_device::fire(u16 w, double pos)
{
	// low byte: pins 1-8 (top first), high byte bit 0: pin 9, active low
	u16 const pins = (~w & 0xff) | (BIT(~w, 8) << 8);
	if (!pins)
		return;
	m_dot_min = std::min(m_dot_min, int(pos));
	m_dot_max = std::max(m_dot_max, int(pos));
	int const x = int((pos / double(UNITS_PER_INCH) + LEFT_OFFSET) * HDPI);
	u32 const ink = ink_color();
	for (int pin = 0; pin < 9; pin++)
	{
		if (BIT(pins, pin))
		{
			// colours mix on the paper
			int const y = m_bitmap_printer->m_ypos + pin * (VDPI / 72);
			u32 const paper = m_bitmap_printer->get_pixel(x, y);
			u32 color = 0;
			for (int shift = 0; shift < 24; shift += 8)
				color |= ((((paper >> shift) & 0xff) * ((ink >> shift) & 0xff)) / 255) << shift;
			m_bitmap_printer->draw_pixel(x, y, color);
		}
	}
}

void iwriter2_device::update_head()
{
	m_bitmap_printer->setheadpos(int((m_cr_pos * UNITS_PER_STEP / double(UNITS_PER_INCH) + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);
}

void iwriter2_device::paper_step(u8 pattern)
{
	static const u8 phases[8] = { 0x0c, 0x0d, 0x09, 0x0b, 0x03, 0x07, 0x06, 0x0e };
	for (int i = 0; i < 8; i++)
	{
		if (phases[i] == (pattern & 0x0f))
		{
			int delta = (i - m_pf_phase) & 7;
			if (delta > 4)
				delta -= 8;
			m_pf_phase = i;
			// the table is stepped by two half steps, one full step feeds 1/144 inch;
			// a decreasing table index feeds the paper forward
			int const rows = ((m_pf_halfsteps - delta) >> 1) - (m_pf_halfsteps >> 1);
			m_pf_halfsteps -= delta;
			m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos + rows);
			m_bitmap_printer->check_new_page();
			update_head();
			break;
		}
	}
}

void iwriter2_device::ribbon_step(u8 pattern)
{
	// full steps 03 09 0c 06; at power-on the firmware drives 52 steps against the end stop
	// on the yellow side, then 39 steps back to black; the colour bands are 13 steps apart
	static const u8 phases[4] = { 0x03, 0x09, 0x0c, 0x06 };
	for (int i = 0; i < 4; i++)
	{
		if (phases[i] == (pattern & 0x0f))
		{
			int const delta = (i - m_rb_phase) & 3;
			m_rb_phase = i;
			if (delta == 1)
				m_rb_pos = std::min(m_rb_pos + 1, RIBBON_BAND * 3);
			else if (delta == 3)
				m_rb_pos = std::max(m_rb_pos - 1, 0);
			break;
		}
	}
}

u32 iwriter2_device::ink_color() const
{
	if (BIT(m_sensors->read(), 6))
		return 0x000000;    // black ribbon
	return ribbon_color[std::clamp((m_rb_pos + RIBBON_BAND / 2) / RIBBON_BAND, 0, 3)];
}

void iwriter2_device::dots_step()
{
	// dot column stream from the ring buffer at a000-a0ff (hypothesis)
	if (!m_dot_run)
		return;
	m_dot_units += UNITS_PER_STEP;
	if (m_dot_units < 0)
		return;
	auto &sp = m_maincpu->space(AS_PROGRAM);
	while (m_dot_run && (m_dot_units >= m_dot_next))
	{
		u16 const w = sp.read_word(0xa000 | m_dot_ptr);
		u8 const old = m_dot_ptr;
		m_dot_ptr += 2;
		if ((old ^ m_dot_ptr) & 0x80)
			m_irq_status |= 0x04;   // half of the ring consumed
		m_dot_cols++;
		// where the head is when the column is fired
		double const speed = double(UNITS_PER_STEP) * CR_CLOCK / std::max<u32>(m_cr_word & 0x0fff, 1);
		fire(w, m_dot_origin + m_dot_dir * (m_dot_next - (CR_SHIFT + speed * CR_LAG)));
		if (!BIT(w, 15))
		{
			m_dot_run = false;
			m_irq_status |= 0x02;   // end of the dot data
			LOGMASKED(LOG_ASIC, "dots end after %d columns, pos %d, dir %d, dots %d..%d, period %d\n", m_dot_cols, m_cr_pos, m_dot_dir, m_dot_min, m_dot_max, m_cr_word & 0x0fff);
		}
		m_dot_next += ((w >> 9) & 0x3f) + 1;
	}
}

u8 iwriter2_device::asic_r(offs_t offset)
{
	u8 data = m_asic[offset];
	switch (offset)
	{
	case 0x04: data = ~m_sw2->read(); break;
	case 0x60: data = ~m_sw1->read(); break;
	case 0x61: data = m_irq_status | ((m_cr_pos > HOME_EDGE) ? 0x10 : 0x00); break;   // bit 4 low in the home zone
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_ASIC, "%s asic_r fe%02x = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void iwriter2_device::asic_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_ASIC, "%s asic_w fe%02x = %02x  (PA %02x PB %02x)\n", machine().describe_context(), offset, data, m_pa, m_pb);
	m_asic[offset] = data;
	switch (offset)
	{
	case 0x62:  // start the carriage
		LOGMASKED(LOG_ASIC, "carriage start: fe60=%04x fe64=%04x fe66=%04x fe72=%02x pos %d\n",
				m_asic[0x60] | (m_asic[0x61] << 8), m_asic[0x64] | (m_asic[0x65] << 8), m_asic[0x66] | (m_asic[0x67] << 8), m_asic[0x72], m_cr_pos);
		m_cr_run = true;
		m_cr_word = m_asic[0x60] | (m_asic[0x61] << 8);
		m_cr_ptr = 0xa100;     // fe60 is the first step, the table at a100 follows
		m_cr_count = 0;
		{
			u16 const start = m_asic[0x66] | (m_asic[0x67] << 8);
			m_dot_run = start != 0xffff;
			if (m_dot_run)
				m_dot_ptr = 0;
			m_dot_units = -int(start);
			m_dot_dir = BIT(m_asic[0x72], 2) ? -1 : 1;
			m_dot_origin = m_cr_pos * UNITS_PER_STEP + m_dot_dir * int(start);
			m_dot_next = 0;
			m_dot_cols = 0;
			m_dot_min = INT_MAX;
			m_dot_max = INT_MIN;
		}
		m_cr_timer->adjust(attotime::from_ticks(std::max<u32>(m_cr_word & 0x0fff, 1), CR_CLOCK));
		break;
	case 0x01:  // paper motor phases from PA0-3
		paper_step(m_pa);
		break;
	case 0x03:  // ribbon shift motor phases from PB0-3
		ribbon_step(m_pb);
		break;
	case 0x63:  // stop the carriage
		m_cr_run = false;
		m_dot_run = false;
		m_cr_timer->adjust(attotime::never);
		break;
	case 0x6c: case 0x6d: case 0x6e: case 0x6f:
		m_irq_status &= ~(1 << (offset - 0x6c));
		update_irq();
		break;
	}
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(APPLE_IMAGEWRITER2, device_rs232_port_interface, iwriter2_device, "iwriter2", "Apple ImageWriter II")
