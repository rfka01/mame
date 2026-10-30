// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Epson FX-880 9-pin dot matrix printer

    C229MAIN board: TMP96C141AF at 14.7456 MHz, E05B50 gate array,
    AT93C46 EEPROM, 1 Mbit PSRAM, 2 Mbit program ROM (G21397).

      000480-007fff  PSRAM (first 32 KiB, short addresses)
      008000-03ffff  program ROM
      040000-0400ff  E05B50 gate array
      080000-09ffff  PSRAM
      100000-13ffff  program ROM
      448000-        CS0: Type-B option interface

    TMP96C141 ports:
      P33   release lever detector (1 = friction feed)
      P34   platen gap detector (1 = gap -1 to 3)
      P36   front PE detector (front paper path)
      P40   rear PE detector (0 = paper)
      P53   carriage home detector (0 = home)
      P60-63  carriage motor phases to the SLA7024M, active low (PG0)
      P64-67  paper feed motor phases through the gate array (PG1)
      P70-73  EEPROM CS, SK, DI, DO
      P82   TO4: needle pulse timing (or the buzzer tone) to the gate array
      AN0   head thermistor, AN1 +35 V monitor

    Gate array registers used here:
      20      parallel data latch (read by micro DMA on INT4)
      22/23   hold/release BUSY
      2b      host control lines
      40      panel keys (bits 0-3, 0 = pressed; bit 6 = a key is down)
      41      LEDs, active low: 1 Pause, 2 Paper Out, 4 Tear Off/Bin,
              5 and 7 Font
      50/51   needle data: 50 bit 7 = pin 1 (top) .. bit 0 = pin 8,
              51 bit 7 = pin 9
      53      bit 0: needle drive enable
      56/57   needle pulse width

    The EEPROM factory area holds what Epson's adjustment program sets,
    among it the carriage width (byte 70h: 1 = 80 columns, otherwise
    136); the firmware shares its code with the wide FX-1180.

    Not emulated: single sheets and the front paper path, the Type-B
    option interface, the head temperature (the thermistor reads cold).

**********************************************************************/

#include "emu.h"
#include "epson_fx880.h"

#include "cpu/tlcs900/tmp96c141.h"
#include "machine/bitmap_printer.h"
#include "machine/eepromser.h"
#include "sound/dac.h"
#include "speaker.h"

#include "epson_fx880.lh"

#define LOG_GA    (1U << 1)
#define LOG_PORT  (1U << 2)
#define LOG_CR    (1U << 3)
#define LOG_PF    (1U << 4)
#define LOG_HEAD  (1U << 5)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap (carriage half step)
constexpr int VDPI = 216;               // vertical resolution (four paper feed half steps)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr int LEFT_STEPS = 60;          // paper edge to the carriage home position, in half steps (guess)
constexpr double NEEDLE_FLIGHT = 135e-6; // fire to impact (s): the firmware fires early by as much; found from
										// bidirectional draft after graphics (aligned from 70 to 190 us)
constexpr int REAR_PE = 400;            // rear PE detector below the print line, in paper half steps (guess)

// AT93C46 contents after the factory reset (Tear Off/Bin and Load/Eject held at power on),
// with the 80 column carriage set in the factory area as the adjustment program does
const u8 default_eeprom[0x80] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

class epson_fx880_device : public device_t, public device_centronics_peripheral_interface
{
public:
	epson_fx880_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_strobe(int state) override;
	virtual void input_init(int state) override;
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
	TIMER_CALLBACK_MEMBER(ack_release);

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);
	u8 p3_r();
	u8 p4_r();
	u8 p5_r();
	u8 p7_r();
	void p6_w(offs_t offset, u8 data, u8 mem_mask);
	void p7_w(u8 data);
	void p8_w(u8 data);
	double head_x() const;
	void strike();
	void update_leds(u8 data);

	required_device<tmp96c141_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<dac_bit_interface> m_beep;
	required_ioport m_panel;
	required_ioport m_config;
	output_finder<5> m_led;

	std::unique_ptr<u8[]> m_ram;
	emu_timer *m_ack_timer;
	u8 m_data;
	u8 m_latch;
	bool m_busy;
	bool m_pending;
	bool m_hold;
	int m_strobe;
	int m_init;
	int m_autofd;
	int m_slctin;
	u8 m_ga[0x100];
	int m_to4;

	// carriage: position in half steps (1/240") from the home sensor edge
	int m_cr_angle;
	int m_cr_pos;
	int m_cr_from;
	attotime m_cr_step_time;
	attotime m_cr_period;

	// paper feed: half steps (1/864"); the paper's top edge relative to the print line
	int m_pf_angle;
	int m_pf_sub;
	int m_paper_pos;
};


INPUT_PORTS_START( fx880 )
	PORT_START("PANEL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-880 Pause")
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-880 Load/Eject")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-880 LF/FF")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FX-880 Tear Off/Bin")

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x00, "Release lever" )
	PORT_CONFSETTING(    0x00, "Tractor" )
	PORT_CONFSETTING(    0x01, "Friction" )
	PORT_CONFNAME( 0x02, 0x02, "Platen gap lever" )
	PORT_CONFSETTING(    0x02, "-1 to 3" )
	PORT_CONFSETTING(    0x00, "4 to 6 (copies)" )
INPUT_PORTS_END

ioport_constructor epson_fx880_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( fx880 );
}


ROM_START( fx880 )
	ROM_REGION(0x80000, "maincpu", 0)
	ROM_LOAD("g21397.ic7", 0x00000, 0x80000, CRC(ca6fcf8a) SHA1(65e7d686791ce40f69fe86372b5dde9cdb8e0e2e))   // 2 Mbit program in a 4 Mbit dump
ROM_END

const tiny_rom_entry *epson_fx880_device::device_rom_region() const
{
	return ROM_NAME( fx880 );
}


epson_fx880_device::epson_fx880_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, EPSON_FX880, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_eeprom(*this, "eeprom"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_beep(*this, "beeper"),
	m_panel(*this, "PANEL"),
	m_config(*this, "CONFIG"),
	m_led(*this, "fx880_led%u", 0U),
	m_data(0),
	m_latch(0),
	m_strobe(1),
	m_init(1),
	m_autofd(1),
	m_slctin(1)
{
}

void epson_fx880_device::mem_map(address_map &map)
{
	map(0x000480, 0x007fff).lrw8(
			NAME([this] (offs_t offset) { return m_ram[0x480 + offset]; }),
			NAME([this] (offs_t offset, u8 data) { m_ram[0x480 + offset] = data; }));
	map(0x008000, 0x03ffff).rom().region("maincpu", 0x8000);
	map(0x040000, 0x0400ff).rw(FUNC(epson_fx880_device::ga_r), FUNC(epson_fx880_device::ga_w));
	map(0x080000, 0x09ffff).lrw8(
			NAME([this] (offs_t offset) { return m_ram[offset]; }),
			NAME([this] (offs_t offset, u8 data) { m_ram[offset] = data; }));
	map(0x100000, 0x13ffff).rom().region("maincpu", 0);
	map(0x448000, 0x44ffff).lr8(NAME([] () { return 0xff; })).nopw();   // Type-B option slot (empty)
}

void epson_fx880_device::device_add_mconfig(machine_config &config)
{
	TMP96C141(config, m_maincpu, 14.7456_MHz_XTAL);
	m_maincpu->set_am8_16(1);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_fx880_device::mem_map);
	m_maincpu->port_read<3>().set(FUNC(epson_fx880_device::p3_r));
	m_maincpu->port_read<4>().set(FUNC(epson_fx880_device::p4_r));
	m_maincpu->port_read<5>().set(FUNC(epson_fx880_device::p5_r));
	m_maincpu->port_read<7>().set(FUNC(epson_fx880_device::p7_r));
	m_maincpu->port_write<6>().set(FUNC(epson_fx880_device::p6_w));
	m_maincpu->port_write<7>().set(FUNC(epson_fx880_device::p7_w));
	m_maincpu->port_write<8>().set(FUNC(epson_fx880_device::p8_w));
	m_maincpu->an_read<0>().set_constant(0x200);   // head thermistor: cold head
	m_maincpu->an_read<1>().set_constant(0x3c0);   // +35 V through a 4.7 V zener diode

	EEPROM_93C46_8BIT(config, m_eeprom).default_data(default_eeprom, sizeof(default_eeprom));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x03, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_epson_fx880);

	SPEAKER(config, "mono").front_center();
	DAC_1BIT(config, m_beep, 0).add_route(ALL_OUTPUTS, "mono", 0.25);
}

void epson_fx880_device::device_start()
{
	m_ram = std::make_unique<u8[]>(0x20000);
	m_ack_timer = timer_alloc(FUNC(epson_fx880_device::ack_release), this);

	m_to4 = 1;
	m_cr_angle = -1;
	m_cr_pos = 200;     // somewhere right of home until the firmware finds it
	m_cr_from = m_cr_pos;
	m_cr_step_time = attotime::zero;
	m_cr_period = attotime::never;
	m_pf_angle = -1;
	m_pf_sub = 0;
	m_paper_pos = 0;    // fanfold paper loaded

	save_pointer(NAME(m_ram), 0x20000);
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_busy));
	save_item(NAME(m_pending));
	save_item(NAME(m_hold));
	save_item(NAME(m_strobe));
	save_item(NAME(m_init));
	save_item(NAME(m_autofd));
	save_item(NAME(m_slctin));
	save_item(NAME(m_ga));
	save_item(NAME(m_to4));
	save_item(NAME(m_cr_angle));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_from));
	save_item(NAME(m_cr_step_time));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_angle));
	save_item(NAME(m_pf_sub));
	save_item(NAME(m_paper_pos));
}

void epson_fx880_device::device_reset()
{
	// BUSY until the firmware opens the interface
	m_busy = true;
	m_pending = false;
	m_hold = true;
	output_busy(1);
	memset(m_ga, 0, sizeof(m_ga));
	update_leds(0x00);
	m_beep->write(0);
}


//**************************************************************************
//  PARALLEL INTERFACE
//**************************************************************************

void epson_fx880_device::input_init(int state)
{
	// the gate array requests an interrupt on INT6 when the host pulls /INIT low
	if (m_init && !state)
	{
		m_maincpu->set_input_line(TLCS900_INT6, ASSERT_LINE);
		m_maincpu->set_input_line(TLCS900_INT6, CLEAR_LINE);
	}
	m_init = state;
}

void epson_fx880_device::input_strobe(int state)
{
	if (m_strobe && !state && !m_busy)
	{
		// the gate array latches the byte, raises BUSY and interrupts on INT4
		m_latch = m_data;
		LOG("strobe %02x\n", m_latch);
		m_busy = true;
		m_pending = true;
		output_busy(1);
		m_maincpu->set_input_line(TLCS900_INT4, ASSERT_LINE);
		m_maincpu->set_input_line(TLCS900_INT4, CLEAR_LINE);
	}
	m_strobe = state;
}

TIMER_CALLBACK_MEMBER(epson_fx880_device::ack_release)
{
	if (m_hold || m_pending)
		return;
	m_busy = false;
	output_ack(0);
	output_busy(0);
	output_ack(1);
}


//**************************************************************************
//  GATE ARRAY
//**************************************************************************

u8 epson_fx880_device::ga_r(offs_t offset)
{
	u8 data = m_ga[offset];
	switch (offset)
	{
	case 0x20:
		data = m_latch;
		if (!machine().side_effects_disabled() && m_pending)
		{
			// BUSY drops with the /ACK pulse a little later, unless 0x22 holds it
			m_pending = false;
			m_ack_timer->adjust(attotime::from_usec(2));
		}
		break;

	case 0x2b:
		// host control lines (bit assignment guessed)
		data = (m_strobe << 0) | (m_init << 1) | (m_slctin << 2) | (m_autofd << 3);
		break;

	case 0x40:
	{
		// panel keys, 0 = pressed; bit 6 tells that one is down
		u8 const keys = m_panel->read() & 0x0f;
		data = ((keys != 0x0f) ? 0x40 : 0x00) | keys;
		break;
	}
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_GA, "%s GA r %02x = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void epson_fx880_device::ga_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_GA, "%s GA w %02x = %02x\n", machine().describe_context(), offset, data);
	m_ga[offset] = data;
	switch (offset)
	{
	case 0x22:
		m_hold = true;
		break;

	case 0x23:
		m_hold = false;
		if (m_pending)
		{
			// byte still unread: request it again
			m_maincpu->set_input_line(TLCS900_INT4, ASSERT_LINE);
			m_maincpu->set_input_line(TLCS900_INT4, CLEAR_LINE);
		}
		else if (m_busy)
			ack_release(0);
		break;

	case 0x41:
		update_leds(data);
		break;
	}
}

void epson_fx880_device::update_leds(u8 data)
{
	m_led[0] = BIT(~data, 1);   // Pause
	m_led[1] = BIT(~data, 2);   // Paper Out
	m_led[2] = BIT(~data, 4);   // Tear Off/Bin
	m_led[3] = BIT(~data, 5);   // Font, left
	m_led[4] = BIT(~data, 7);   // Font, right
}


//**************************************************************************
//  PORTS
//**************************************************************************

u8 epson_fx880_device::p3_r()
{
	// P33: release lever, P34: platen gap lever, P36: front PE (no paper on the front path)
	u8 const cfg = m_config->read();
	return 0xe7 | (BIT(cfg, 0) << 3) | (BIT(cfg, 1) << 4);
}

u8 epson_fx880_device::p4_r()
{
	// P40: rear PE detector, 0 = paper; parked paper clears it
	return (m_paper_pos > -REAR_PE) ? 0xfe : 0xff;
}

u8 epson_fx880_device::p5_r()
{
	// P53: carriage home detector
	return (m_cr_pos <= 0) ? 0xf7 : 0xff;
}

u8 epson_fx880_device::p7_r()
{
	// P73: EEPROM DO
	return m_eeprom->do_read() ? 0xff : 0xf7;
}

// half-step angle 0-7 of a 4-phase motor from its energised phases, -1 if none
static int phase_angle(u8 phases)
{
	switch (phases & 0x0f)
	{
	case 0x1: return 0;
	case 0x3: return 1;
	case 0x2: return 2;
	case 0x6: return 3;
	case 0x4: return 4;
	case 0xc: return 5;
	case 0x8: return 6;
	case 0x9: return 7;
	default:  return -1;
	}
}

void epson_fx880_device::p6_w(offs_t offset, u8 data, u8 mem_mask)
{
	// P60-P63: carriage motor phases, active low (SLA7024M inputs)
	int const angle = phase_angle(~data & mem_mask & 0x0f);
	if (angle >= 0)
	{
		if (m_cr_angle >= 0)
		{
			int const delta = ((angle - m_cr_angle + 4) & 7) - 4;
			if (delta)
			{
				// the carriage runs from the previous rotor position to the new one during a step period
				attotime const now = machine().time();
				attotime const since = now - m_cr_step_time;
				m_cr_period = (since < attotime::from_msec(10)) ? since : attotime::never;
				m_cr_step_time = now;
				m_cr_from = m_cr_pos;
				m_cr_pos = std::max(m_cr_pos - delta, -20);   // positive phase steps run towards home
				m_bitmap_printer->setheadpos(int((m_cr_pos + LEFT_STEPS) * HDPI / 240), m_bitmap_printer->m_ypos);
				LOGMASKED(LOG_CR, "%s CR %d\n", machine().describe_context(), m_cr_pos);
			}
		}
		m_cr_angle = angle;
	}

	// P64-P67: paper feed motor phases through the gate array
	int const pf = phase_angle((data & mem_mask) >> 4);
	if (pf >= 0)
	{
		if (m_pf_angle >= 0 && pf != m_pf_angle)
		{
			// forward feed runs the phases backwards; 864 half steps per inch, a bitmap row every four
			int const delta = ((pf - m_pf_angle + 4) & 7) - 4;
			m_paper_pos -= delta;
			m_pf_sub -= delta;
			while (m_pf_sub >= 4)
			{
				m_pf_sub -= 4;
				m_bitmap_printer->m_ypos++;
			}
			while (m_pf_sub <= -4)
			{
				m_pf_sub += 4;
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			}
			m_bitmap_printer->check_new_page();
			LOGMASKED(LOG_PF, "%s PF %d\n", machine().describe_context(), m_paper_pos);
		}
		m_pf_angle = pf;
	}
}

void epson_fx880_device::p7_w(u8 data)
{
	m_eeprom->cs_write(BIT(data, 0));
	m_eeprom->di_write(BIT(data, 2));
	m_eeprom->clk_write(BIT(data, 1));
}

void epson_fx880_device::p8_w(u8 data)
{
	// P82/TO4: needle drive timing to the gate array, which pulses the needles selected in
	// 0x50/0x51 for the time in 0x56/0x57 while 0x53 bit 0 is set. The buzzer is the head
	// itself: all needles driven at 1.5 kHz with pulses too short to strike the ribbon
	// (strike threshold guessed between the beep's 5Dh and the shortest printing pulse)
	int const to4 = BIT(data, 2);
	bool const enabled = BIT(m_ga[0x53], 0);
	bool const buzz = enabled && ((m_ga[0x56] | (m_ga[0x57] << 8)) < 0x80);
	if (m_to4 != to4)
	{
		if (m_to4 && !to4 && enabled && !buzz)
		{
			LOGMASKED(LOG_HEAD, "fire cr %d data %02x%02x\n", m_cr_pos, m_ga[0x50], m_ga[0x51]);
			strike();
			m_ga[0x50] = 0;
			m_ga[0x51] = 0;
		}
		m_to4 = to4;
	}
	m_beep->write(buzz ? to4 : 0);
}


//**************************************************************************
//  PRINT HEAD
//**************************************************************************

double epson_fx880_device::head_x() const
{
	// carriage position in half steps (1/240"), interpolated within the current step period,
	// where the wire reaches the paper: the head moves on during the needle's flight
	if (m_cr_period.is_never())
		return m_cr_pos;
	double const period = m_cr_period.as_double();
	double const frac = std::min(1.0, (machine().time() - m_cr_step_time).as_double() / period);
	return m_cr_from + (m_cr_pos - m_cr_from) * (frac + NEEDLE_FLIGHT / period);
}

void epson_fx880_device::strike()
{
	// 0x50 bit 7 = pin 1 (top) .. bit 0 = pin 8, 0x51 bit 7 = pin 9
	u16 const head = (m_ga[0x50] << 1) | BIT(m_ga[0x51], 7);
	int const x = int(std::lround((head_x() + LEFT_STEPS) * HDPI / 240.0));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;
	for (int pin = 0; pin < 9; pin++)
		if (BIT(head, 8 - pin))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + pin * (VDPI / 72), 0x000000);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_FX880, device_centronics_peripheral_interface, epson_fx880_device, "fx880", "Epson FX-880")
