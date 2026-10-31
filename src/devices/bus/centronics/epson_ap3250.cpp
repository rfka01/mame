// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Epson ActionPrinter 3250 (LQ-100) 24-pin dot matrix printer

    Board C092 MAIN: NEC uPD78C10, 32 KiB SRAM, 2 Mbit mask ROM,
    gate array E05A71CA, X24C01 EEPROM.

    Memory map (from the firmware):
      0000-5fff  ROM 00000-05fff
      6000-7fff  ROM window, 8 KiB bank selected by FE02
      8000-fdff  SRAM
      fe00-fe0f  E05A71
      ff00-ffff  internal RAM

    E05A71 (as far as the firmware shows):
      FE00  bit 5 head on, bit 4 right-to-left pass (head bytes taken in reverse order)
      FE01  bits 3-0 paper feed motor (half steps), bits 6-4 motor currents?
      FE02  ROM bank
      FE03  bit 0 data latched, bit 1 rising edge releases BUSY, bit 2 ACK
      FE04  Centronics data
      FE0B  needle data, three bytes for 24 needles

    Ports:
      PA3-PA0 carriage motor, PA4 FONT/YES, PA5 FORM FEED/NO,
      PA6 paper sensor (1 = paper), PA7 carriage home (1 = home)
      PB1/PB0 EEPROM SCL/SDA, PB2 paper release lever (1 = single sheets),
      PB3/PB4 font lamps, PB5 /AUTO FEED?, PB6 READY lamp, PB7 /ERROR?
      PC6 = CO0 needle drive pulse (low), width from the A/D readings

    Single sheets come from the cassette: the firmware turns the paper
    feed motor back to pick a sheet and feeds it until the paper sensor
    sees it, then on to the top of form. With "always loaded" sheets the
    cassette is modelled here, the bitmap printer files a page whenever
    a sheet's trailing edge passes the print line.

    TODO:
    - push tractor (continuous paper): the firmware keeps to single sheet
      page lengths with the release lever at continuous
    - A/D inputs (head temperature, voltages) are fixed at mid scale
    - crystal frequency not known

**********************************************************************/

#include "emu.h"
#include "epson_ap3250.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "machine/i2cmem.h"

#include "epson_ap3250.lh"

#define LOG_GA    (1U << 1)
#define LOG_PORT  (1U << 2)
#define LOG_CR    (1U << 3)
#define LOG_HEAD  (1U << 4)
#define LOG_PF    (1U << 5)

//#define VERBOSE (LOG_GENERAL | LOG_HEAD | LOG_PF | LOG_CR)
#include "logmacro.h"


namespace {

constexpr int HDPI = 360;
constexpr int VDPI = 360;
constexpr double PAPER_WIDTH = 9.5;     // widest paper (fanfold with the push tractor)
constexpr double PAPER_HEIGHT = 11.0;
constexpr double CR_PER_INCH = 120.0;   // carriage half steps per inch (from character widths)
constexpr double LEFT_OFFSET = 0.2;     // home position to the left end of the mechanism, inches: centres 80 columns on a Letter sheet (guess)
constexpr double EVEN_OFFSET = 4.0;     // even needle column, half steps right of the odd one (from the firmware timing)
constexpr double DOT_LAG = 1.3;          // half steps the dots land behind the step position while moving,
                                        // fitted so that both directions line up
constexpr int NEEDLE_MIN = 150;         // shortest drive pulse that prints, us (beeper pulses are about 50 us)
constexpr double SHEET_TOP = 8.5 / 25.4;    // first print line of a sheet (manual: TOF 8.5 mm)
constexpr double SHEET_SENSOR = 576.0 / 360 - SHEET_TOP;   // paper sensor to print line: with TOF +-0 the loading feed runs 576/360" from the sensor to the top line
constexpr double SHEET_PICK = 0.2;         // cassette to paper sensor, guess
constexpr double CR_MOVE = 0.0005;      // time for one half step of the carriage (guess)

class epson_ap3250_device : public device_t, public device_centronics_peripheral_interface
{
public:
	epson_ap3250_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_strobe(int state) override;
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

	u8 an_r(int ch) { return 0x80; }   // sensor voltages, values not known
	u8 pa_r();
	u8 pb_r();
	bool paper_sensor() const;
	bool always_loaded() const;
	int sheet_rows() const;
	void vsheet_step(int dir);
	void check_sheet();
	u8 pc_r();
	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);

	required_device<upd78c10_device> m_maincpu;
	required_memory_bank m_bank;
	required_device<i2cmem_device> m_eeprom;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_panel;
	required_ioport m_config;
	output_finder<3> m_led;

	void update_centronics();

	u8 m_ga[16];
	u8 m_data;
	u8 m_latch;
	int m_strobe;
	int m_busy;
	u8 m_pa, m_pb, m_pc, m_pd, m_pf;
	int m_cr_phase;
	int m_pf_phase;
	int m_pf_pos;
	u32 m_head;
	int m_cr_dir;
	bool m_sheet_in;
	int m_vsheet;
	attotime m_fire_start;
	double m_fire_x;
	u32 m_fire_head;
	attotime m_cr_last;
	attotime m_cr_period;

	double head_x() const;
	void fire_start();
	void fire_end();
	int m_cr_pos;
};


INPUT_PORTS_START( ap3250 )
	PORT_START("PANEL")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Font / Yes") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Form Feed / No") PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("CONFIG")
	PORT_CONFNAME(0x01, 0x01, "Paper loaded")
	PORT_CONFSETTING(   0x00, DEF_STR(No))
	PORT_CONFSETTING(   0x01, DEF_STR(Yes))
INPUT_PORTS_END

ioport_constructor epson_ap3250_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( ap3250 );
}


ROM_START( ap3250 )
	ROM_REGION(0x40000, "maincpu", 0)
	ROM_LOAD("ap3250.ic3", 0x00000, 0x40000, CRC(f9ffec2a) SHA1(2552ab9bb4fc1b796bd83393f15f6b4a25e395f8))

	// settings as the firmware writes them into an empty EEPROM, changed to the factory
	// settings given in the manual: TOF +-0 (byte 6), download off (byte 8), auto line feed off (byte 9)
	ROM_REGION(0x80, "eeprom", 0)
	ROM_FILL(0x00, 0x40, 0x00)
	ROM_FILL(0x06, 0x01, 0x03)
	ROM_FILL(0x08, 0x01, 0x01)
	ROM_FILL(0x09, 0x01, 0x01)
	ROM_FILL(0x40, 0x40, 0xff)
ROM_END

const tiny_rom_entry *epson_ap3250_device::device_rom_region() const
{
	return ROM_NAME( ap3250 );
}


epson_ap3250_device::epson_ap3250_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, EPSON_AP3250, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bank(*this, "bank"),
	m_eeprom(*this, "eeprom"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_panel(*this, "PANEL"),
	m_config(*this, "CONFIG"),
	m_led(*this, "ap3250_led%u", 0U)
{
}

void epson_ap3250_device::mem_map(address_map &map)
{
	map(0x0000, 0x5fff).rom().region("maincpu", 0);
	map(0x6000, 0x7fff).bankr(m_bank);
	map(0x8000, 0xfdff).ram();
	map(0xfe00, 0xfe0f).rw(FUNC(epson_ap3250_device::ga_r), FUNC(epson_ap3250_device::ga_w));
}

void epson_ap3250_device::device_add_mconfig(machine_config &config)
{
	UPD78C10(config, m_maincpu, 14.7456_MHz_XTAL);   // crystal not known
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_ap3250_device::mem_map);
	m_maincpu->pa_in_cb().set(FUNC(epson_ap3250_device::pa_r));
	m_maincpu->pb_in_cb().set(FUNC(epson_ap3250_device::pb_r));
	m_maincpu->pc_in_cb().set(FUNC(epson_ap3250_device::pc_r));
	m_maincpu->pa_out_cb().set([this] (u8 data) {
		// PA3-PA0: carriage motor, eight states per cycle
		static constexpr int cr_seq[16] = { 0, 6, 7, -1, 2, 4, -1, 3, -1, 5, -1, -1, 1, -1, -1, -1 };
		int const ph = cr_seq[data & 0x0f];
		if (((data ^ m_pa) & 0x0f) && ph >= 0)
		{
			int const d = (ph - m_cr_phase) & 7;
			int delta = 0;
			if (d == 1 || d == 2) delta = d;
			else if (d == 7 || d == 6) delta = d - 8;
			if (delta)
			{
				attotime const now = machine().time();
				int const dir = delta > 0 ? 1 : -1;
				attotime const since = now - m_cr_last;
				m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since / std::abs(delta) : attotime::never;
				m_cr_dir = dir;
				m_cr_last = now;
				m_cr_pos += delta;
				m_bitmap_printer->setheadpos(int(std::lround((m_cr_pos / CR_PER_INCH + LEFT_OFFSET) * HDPI)), m_bitmap_printer->m_ypos);
			}
			m_cr_phase = ph;
			LOGMASKED(LOG_CR, "CR %d\n", m_cr_pos);
		}
		if ((data ^ m_pa) & 0xf0) LOGMASKED(LOG_PORT, "%s PA w %02x\n", machine().describe_context(), data);
		m_pa = data;
	});
	m_maincpu->pb_out_cb().set([this] (u8 data) {
		if ((data ^ m_pb) & 0xfc) LOGMASKED(LOG_PORT, "%s PB w %02x\n", machine().describe_context(), data);
		m_pb = data;
		m_led[0] = BIT(~data, 4);   // left font lamp (PB4, lit by switching the pin to output low)
		m_led[1] = BIT(~data, 3);   // right font lamp (PB3)
		m_led[2] = BIT(~data, 6);   // READY
		m_eeprom->write_sda(BIT(data, 0));   // PB0 = SDA, PB1 = SCL
		m_eeprom->write_scl(BIT(data, 1));
	});
	m_maincpu->pc_out_cb().set([this] (u8 data) {
		if ((data ^ m_pc) & ~0x40) LOGMASKED(LOG_PORT, "%s PC w %02x\n", machine().describe_context(), data);
		if (!BIT(data, 6) && BIT(m_pc, 6))   // PC6 = CO0, low while the head is driven
			fire_start();
		if (BIT(data, 6) && !BIT(m_pc, 6))
			fire_end();
		m_pc = data;
	});
	m_maincpu->pd_out_cb().set([this] (u8 data) { m_pd = data; });
	m_maincpu->pf_out_cb().set([this] (u8 data) { m_pf = data; });
	I2C_X24C01(config, m_eeprom);

	config.set_default_layout(layout_epson_ap3250);

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_24pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_LETTER);   // cassette; fanfold with the push tractor
	m_bitmap_printer->set_sheet_load_offset(SHEET_SENSOR);
	m_bitmap_printer->set_sheet_top(SHEET_TOP);

	m_maincpu->an0_func().set([this] () { return an_r(0); });
	m_maincpu->an1_func().set([this] () { return an_r(1); });
	m_maincpu->an2_func().set([this] () { return an_r(2); });
	m_maincpu->an3_func().set([this] () { return an_r(3); });
	m_maincpu->an4_func().set([this] () { return an_r(4); });
	m_maincpu->an5_func().set([this] () { return an_r(5); });
	m_maincpu->an6_func().set([this] () { return an_r(6); });
	m_maincpu->an7_func().set([this] () { return an_r(7); });
}

void epson_ap3250_device::device_start()
{
	m_bank->configure_entries(0, 0x20, memregion("maincpu")->base(), 0x2000);
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	m_data = 0;
	m_latch = 0;
	m_strobe = 1;
	m_busy = 1;
	save_item(NAME(m_busy));
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	m_pa = m_pb = m_pc = m_pd = m_pf = 0xff;
	m_cr_phase = 0;
	m_vsheet = -int(std::lround((SHEET_SENSOR + SHEET_PICK) * VDPI));
	m_sheet_in = false;
	save_item(NAME(m_vsheet));
	save_item(NAME(m_sheet_in));
	m_pf_phase = 0;
	m_cr_dir = 1;
	m_fire_start = attotime::zero;
	m_fire_x = 0;
	m_fire_head = 0;
	save_item(NAME(m_fire_start));
	save_item(NAME(m_fire_x));
	save_item(NAME(m_fire_head));
	m_cr_last = attotime::zero;
	m_cr_period = attotime::never;
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last));
	save_item(NAME(m_cr_period));
	m_pf_pos = 0;
	m_head = 0;
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_pf_pos));
	save_item(NAME(m_head));
	m_cr_pos = 400;
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_ga));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_pd));
	save_item(NAME(m_pf));
}

void epson_ap3250_device::device_reset()
{
	m_bank->set_entry(3);
}

u8 epson_ap3250_device::pa_r()
{
	// PA4 = FONT/YES, PA5 = FORM FEED/NO, PA6 = paper sensor (1 = paper), PA7 = carriage home sensor (1 = home)
	u8 d = (m_panel->read() & 0x70) | 0x0f;
	if (!paper_sensor())   // PA6: paper sensor, 1 = paper
		d &= ~0x40;
	if (m_cr_pos <= 0)
		d |= 0x80;
	return d;
}

void epson_ap3250_device::check_sheet()
{
	// a sheet from the cassette starts out short of the paper sensor; the loading feed brings it up
	bool const in = m_bitmap_printer->sheet_handling() && m_bitmap_printer->sheet_in();
	if (in && !m_sheet_in)
		m_bitmap_printer->m_ypos -= int(std::lround(SHEET_PICK * VDPI));
	m_sheet_in = in;
}

bool epson_ap3250_device::always_loaded() const
{
	return m_bitmap_printer->single_sheet() && !m_bitmap_printer->sheet_handling();
}

int epson_ap3250_device::sheet_rows() const
{
	u32 const fmt = m_bitmap_printer->ioport("PAPER")->read() & 0x07;
	return int(std::lround(((fmt == 3) ? 297.0 / 25.4 : 11.0) * VDPI));
}

bool epson_ap3250_device::paper_sensor() const
{
	if (!BIT(m_config->read(), 0))
		return false;
	int const sensor = int(std::lround(SHEET_SENSOR * VDPI));

	// always loaded single sheets: the cassette is modelled here, the bitmap printer
	// starts each page at the top of form, where the firmware's loading feed puts it
	if (always_loaded())
		return (m_vsheet >= -sensor) && (m_vsheet < sheet_rows() - sensor);

	// a sheet pulled back below the sensor is no longer seen
	return m_bitmap_printer->paper_present() && (m_bitmap_printer->m_ypos >= -sensor);
}

void epson_ap3250_device::vsheet_step(int dir)
{
	// always loaded single sheets: position of the sheet in the cassette paper path
	if (!always_loaded())
		return;
	int const pick = -int(std::lround((SHEET_SENSOR + SHEET_PICK) * VDPI));
	int const rows = sheet_rows();
	m_vsheet += dir;
	if (m_vsheet >= rows + int(std::lround(SHEET_SENSOR * VDPI)))
		m_vsheet = pick;   // ejected: the next sheet waits in the cassette
	if (m_vsheet <= rows)
	{
		// at the trailing edge the bitmap printer files the page
		m_bitmap_printer->m_ypos = m_vsheet;
		m_bitmap_printer->check_new_page();
	}
}

u8 epson_ap3250_device::pb_r()
{
	if (!machine().side_effects_disabled())
		check_sheet();
	// PB0 = EEPROM SDA, PB2 = paper release lever (1 = single sheets, 0 = continuous), PB5 = /AUTO FEED from the host? (only read for the auto line feed setting)
	u8 d = 0xfa | m_eeprom->read_sda();
	if (m_bitmap_printer->single_sheet())   // paper release lever: 1 = single sheets
		d |= 0x04;
	return d;
}

u8 epson_ap3250_device::pc_r()
{
	u8 const d = 0xff;
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_PORT, "%s PC r %02x\n", machine().describe_context(), d);
	return d;
}

double epson_ap3250_device::head_x() const
{
	// the carriage reaches a new half step within CR_MOVE
	if (m_cr_period.is_never())
		return m_cr_pos;
	double const move = std::min(m_cr_period.as_double(), CR_MOVE);
	double const frac = std::min(1.0, (machine().time() - m_cr_last).as_double() / move);
	return m_cr_pos - m_cr_dir * (1.0 - frac);
}

void epson_ap3250_device::fire_start()
{
	// FE0B shifts in three bytes; bit 23 = needle 1 (top), odd needles in the left column.
	// FE00 bit 4 (set for right-to-left passes) makes the gate array take the bytes in reverse order.
	m_fire_start = machine().time();
	m_fire_x = head_x() - (m_cr_period.is_never() ? 0.0 : m_cr_dir * DOT_LAG);
	m_fire_head = BIT(m_ga[0], 4) ? (((m_head & 0xff) << 16) | (m_head & 0xff00) | (m_head >> 16)) : m_head;
}

void epson_ap3250_device::fire_end()
{
	// the beeper drives the head with pulses too short to reach the paper
	attotime const len = machine().time() - m_fire_start;
	LOGMASKED(LOG_HEAD, "FIRE x=%.2f pf=%d head=%06x %d us\n", m_fire_x, m_pf_pos, m_fire_head, int(len.as_double() * 1e6));
	if (len < attotime::from_usec(NEEDLE_MIN))
		return;
	for (int n = 0; n < 24; n++)
	{
		if (!BIT(m_fire_head, 23 - n))
			continue;
		double const cx = m_fire_x + ((n & 1) ? EVEN_OFFSET : 0.0);
		int const x = int(std::lround((cx / CR_PER_INCH + LEFT_OFFSET) * HDPI));
		if (x >= 0 && x < int(PAPER_WIDTH * HDPI))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + n * (VDPI / 180), 0x000000);
	}
}

void epson_ap3250_device::input_strobe(int state)
{
	// falling edge: the gate array latches the data, flags it and goes busy
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_ga[3] |= 0x01;
		m_busy = 1;
		update_centronics();
	}
	m_strobe = state;
}

void epson_ap3250_device::update_centronics()
{
	// FE03: bit 0 = data latched, bit 1 rising = BUSY off, bit 2 = ACK (guess);
	// BUSY is set by the strobe
	output_busy(m_busy);
	output_ack(BIT(~m_ga[3], 2));
}

u8 epson_ap3250_device::ga_r(offs_t offset)
{
	u8 data = m_ga[offset];
	if (offset == 4)
	{
		data = m_latch;
		if (!machine().side_effects_disabled())
			m_ga[3] &= ~0x01;
	}
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_GA, "%s GA r %x = %02x\n", machine().describe_context(), offset, data);
	return data;
}

void epson_ap3250_device::ga_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_GA, "%s GA w %x = %02x\n", machine().describe_context(), offset, data);
	if (offset == 3)
	{
		if (BIT(data, 1) && !BIT(m_ga[3], 1))   // a rising edge of bit 1 clears BUSY
			m_busy = 0;
		m_ga[3] = (m_ga[3] & 0x01) | (data & ~0x01);
		update_centronics();
		return;
	}
	if (offset == 0x0b)
		m_head = ((m_head << 8) | data) & 0xffffff;
	if (offset == 1 && ((data ^ m_ga[1]) & 0x0f))
	{
		// FE01 bits 3-0: paper feed motor, half steps a 2 6 4 5 1 9 8
		static constexpr int pf_seq[16] = { -1, 5, 1, -1, 3, 4, 2, -1, 7, 6, 0, -1, -1, -1, -1, -1 };
		int const ph = pf_seq[data & 0x0f];
		if (ph >= 0)
		{
			int const d = (ph - m_pf_phase) & 7;
			int delta = 0;
			if (d == 1 || d == 2) delta = -d;            // the sequence runs backwards when feeding
			else if (d == 7 || d == 6) delta = 8 - d;
			m_pf_phase = ph;
			check_sheet();
			for (int i = 0; i < std::abs(delta); i++)
			{
				// one half step = 1/360 inch = one pixel
				m_pf_pos += delta > 0 ? 1 : -1;
				if (always_loaded())
				{
					vsheet_step(delta > 0 ? 1 : -1);
					continue;
				}
				if (delta > 0)
					m_bitmap_printer->m_ypos++;
				else
					m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
				m_bitmap_printer->check_new_page();
			}
			LOGMASKED(LOG_PF, "PF %d y %d paper %d v %d\n", m_pf_pos, m_bitmap_printer->m_ypos, paper_sensor(), m_vsheet);
		}
	}
	m_ga[offset] = data;
	if (offset == 2)
		m_bank->set_entry(data & 0x1f);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_AP3250, device_centronics_peripheral_interface, epson_ap3250_device, "ap3250", "Epson ActionPrinter 3250")
