// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX Printer and Alphacom 32 printer mechanisms

    The ZX Printer has two styli on a belt; each pass across the
    aluminium coated paper is one row of 256 dots. An encoder disc gives
    1024 pulses across the printing width, divided by four for the dot
    clock. The computer drives the printer directly through port FB
    (A2 low):

      read   D0  encoder: a new dot position has been reached since the
                 last write
             D6  0 = printer present
             D7  stylus on the paper at the left edge (start of a row)
      write  D1  slow motor (the last rows of a line)
             D2  motor off
             D7  stylus power (burn a dot)

    The Alphacom 32 is a thermal printer that connects to the same port.
    It was also sold as the Timex Sinclair 2040.

    The timing follows IMC's xz80 model (as used by Fuse): a dot takes
    220 Spectrum T-states (440 at slow speed), a row 384 dot times of
    which 256 are on the paper, starting 64 dot times after the row.
    A dot is burnt while the stylus passes it with the power on. The ROMs
    switch the power off right after setting the last dot of a row, while
    the stylus is still over it; a dot switched on and off again at the
    same position therefore counts as burnt (otherwise the right edge of
    COPY pictures was lost).

    Alphacom 32 keys (user's manual): OFF takes the printer out of the
    printing mode, ON/ADVANCE puts it back into the printing mode and feeds
    the paper while it is held. Holding ON/ADVANCE, pressing OFF once and
    releasing ON/ADVANCE starts the self test, which prints a line of the
    numeral 8 and a line of the numeral 1 again and again; OFF stops it,
    ON/ADVANCE returns to the printing mode.

    The self test printout of the identical Timex Sinclair 2040 (video
    "Printing a Sinus Curve with the Timex Sinclair 2040" by Retroguy)
    shows rows of 32 characters, 8s and 1s in turn without a gap, in a
    character set like the ZX Spectrum's; the self test uses the Spectrum
    glyphs of 8 and 1 (the Alphacom 32 logic is not dumped).

    Assumptions: out of the printing mode the printer ignores the
    computer's writes; the printer starts in the printing mode.

**********************************************************************/

#include "emu.h"
#include "zxprinter.h"

#include "alphacom32.lh"
#include "zxprinter.lh"


namespace {

constexpr int DPI = 72;                 // 256 dots on about 3.6 inches
constexpr double PAPER_WIDTH = 4.0;     // ZX Printer and Alphacom 32 rolls
constexpr double PAPER_HEIGHT = 11.0;   // length of the filed pages
constexpr int LEFT = int((PAPER_WIDTH * DPI - 256) / 2);
constexpr int ROW = 384;                // dot times per row
constexpr int START = 64;               // dot times before the left edge
constexpr int END = ROW - START;        // row wraps here

constexpr u32 SPECTRUM_CLOCK = 3'500'000;  // the T-states of the timing model

// Alphacom 32 self test characters (the ZX Spectrum glyphs of 8 and 1)
constexpr u8 TEST_GLYPHS[2][8] = {
	{ 0x00, 0x3c, 0x42, 0x3c, 0x42, 0x42, 0x3c, 0x00 },    // 8
	{ 0x00, 0x18, 0x28, 0x08, 0x08, 0x08, 0x3e, 0x00 } };  // 1

} // anonymous namespace


DEFINE_DEVICE_TYPE(ZX_PRINTER, zx_printer_device, "zx_printer", "Sinclair ZX Printer mechanism")
DEFINE_DEVICE_TYPE(ALPHACOM32, alphacom32_device, "alphacom32", "Alphacom 32 mechanism")


static INPUT_PORTS_START( zx_printer )
	PORT_START("BUTTONS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Paper Feed") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(zx_printer_device::feed_w), 0)
INPUT_PORTS_END

static INPUT_PORTS_START( alphacom32 )
	PORT_START("BUTTONS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("ON/ADVANCE") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(zx_printer_device::feed_w), 0)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("OFF") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(zx_printer_device::off_w), 0)
INPUT_PORTS_END

ioport_constructor zx_printer_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( zx_printer );
}

ioport_constructor alphacom32_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( alphacom32 );
}


zx_printer_device::zx_printer_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_buttons(*this, "BUTTONS")
{
}

zx_printer_device::zx_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	zx_printer_device(mconfig, ZX_PRINTER, tag, owner, clock)
{
}

alphacom32_device::alphacom32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	zx_printer_device(mconfig, ALPHACOM32, tag, owner, clock)
{
}


void zx_printer_device::device_add_mconfig(machine_config &config)
{
	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * DPI), int(PAPER_HEIGHT * DPI), DPI, DPI);
	m_bitmap_printer->set_continuous_feed(true);
	m_bitmap_printer->set_paper_view(3.0);
	if (type() != ALPHACOM32)
		m_bitmap_printer->set_paper_texture(&zx_printer_device::aluminium_paper);

	if (type() == ALPHACOM32)
		config.set_default_layout(layout_alphacom32);
	else
		config.set_default_layout(layout_zxprinter);
}

void zx_printer_device::device_start()
{
	m_timer = timer_alloc(FUNC(zx_printer_device::tick), this);
	m_test_timer = timer_alloc(FUNC(zx_printer_device::test_row), this);

	save_item(NAME(m_host));
	save_item(NAME(m_speed));
	save_item(NAME(m_new_speed));
	save_item(NAME(m_row_start));
	save_item(NAME(m_pixel));
	save_item(NAME(m_filled));
	save_item(NAME(m_stylus));
	save_item(NAME(m_on_x));
	save_item(NAME(m_row_done));
	save_item(NAME(m_row));
	save_item(NAME(m_off));
	save_item(NAME(m_test));
	save_item(NAME(m_test_armed));
	save_item(NAME(m_test_row));
}

void zx_printer_device::device_reset()
{
	m_host = false;
	m_speed = 0;
	m_new_speed = 0;
	m_row_start = attotime::zero;
	m_pixel = -1;
	m_filled = 0;
	m_stylus = 0;
	m_on_x = -1;
	m_row_done = false;
	std::fill(std::begin(m_row), std::end(m_row), 0);
	m_timer->adjust(attotime::never);
	m_off = false;
	m_test = false;
	m_test_armed = false;
	m_test_row = 0;
	m_test_timer->adjust(attotime::never);
}


void zx_printer_device::aluminium_paper(printer_page_canvas &page)
{
	// the aluminium coating of the ZX Printer paper on the PDF pages: silver grey,
	// with the streaks of the rolled coating running along the paper
	auto noise = [] (u32 i, u32 seed)
	{
		u32 h = i * 0x9e3779b1u ^ seed * 0x85ebca77u;
		h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; h *= 0x297a2d39u; h ^= h >> 15;
		return float(h >> 8) * (2.0f / 16777216.0f) - 1.0f;
	};
	auto smooth = [&noise] (double x, u32 seed)
	{
		int const i = int(std::floor(x));
		float f = float(x - i);
		f = f * f * (3.0f - 2.0f * f);
		return noise(u32(i), seed) * (1.0f - f) + noise(u32(i + 1), seed) * f;
	};

	int const w = page.width(), h = page.height();
	double const dpi = page.dpi();
	std::vector<float> column(w);
	for (int x = 0; x < w; x++)
	{
		double const in = x / dpi;
		column[x] = 0.84f
				+ 0.020f * smooth(in / 0.9, 1)
				+ 0.014f * smooth(in / 0.22, 2)
				+ 0.010f * smooth(in / 0.055, 3)
				+ 0.008f * noise(u32(x), 4);
	}
	for (int y = 0; y < h; y++)
	{
		float const along = 1.0f + 0.012f * smooth(y / dpi / 2.5, 5);   // the coating is not quite even along the roll
		u8 *const row = page.row(y);
		int const ch = page.channels();
		for (int x = 0; x < w; x++)
			std::fill_n(&row[x * ch], ch, u8(std::lround(255.0f * std::clamp(column[x] * along, 0.0f, 1.0f))));
	}
}

attotime zx_printer_device::pixel_period(int speed) const
{
	return attotime::from_ticks((speed == 1) ? 440 : 220, SPECTRUM_CLOCK);
}

int zx_printer_device::current_x() const
{
	// dot position reached by the stylus in this row
	return int((machine().time() - m_row_start).as_double() / pixel_period(m_speed).as_double()) - START;
}

void zx_printer_device::output_row()
{
	// one pass of a stylus: a row of dots, then the paper moves on by one row
	// the spark burns the aluminium away down to the dark grey base layer; the thermal
	// paper of the Alphacom 32 turns black
	int const y = m_bitmap_printer->m_ypos;
	int const ink = (type() == ALPHACOM32) ? 0x000000 : 0x2c2c2c;
	for (int i = 0; i < DOTS; i++)
		if (m_row[i])
			m_bitmap_printer->draw_pixel(LEFT + i, y, ink);
	std::fill(std::begin(m_row), std::end(m_row), 0);
	m_bitmap_printer->m_ypos++;
	m_bitmap_printer->check_new_page();
	m_row_done = true;
}

void zx_printer_device::update()
{
	// bring the belt up to the present: the dots passed get the stylus state
	if (!m_speed)
		return;
	for (;;)
	{
		int const x = current_x();
		int const to = std::min(x, DOTS);
		for (int i = std::max(m_filled, 0); i < to; i++)
			m_row[i] |= m_stylus;
		m_filled = std::max(m_filled, to);
		if ((x >= DOTS) && !m_row_done)
			output_row();
		if (x < END)
			break;

		// next stylus, next row
		m_row_start += pixel_period(m_speed) * ROW;
		m_pixel = -1;
		m_filled = 0;
		m_row_done = false;
		if (m_new_speed)
		{
			// keep the position reached, measured at the new speed
			attotime const now = machine().time();
			int const passed = int((now - m_row_start).as_double() / pixel_period(m_speed).as_double());
			m_speed = m_new_speed;
			m_new_speed = 0;
			m_row_start = now - pixel_period(m_speed) * passed;
		}
	}
}

uint8_t zx_printer_device::read()
{
	if (!machine().side_effects_disabled())
		update();
	if (!m_speed)
		return 0x3e;

	int const x = current_x();
	uint8_t data = 0x3e;
	if (((x > -10) && (x < 0)) || m_stylus)
		data |= 0x80;   // stylus on the paper at the left edge
	if (x > m_pixel)
		data |= 0x01;   // encoder
	return data;
}

void zx_printer_device::write(uint8_t data)
{
	if (m_off || m_test)
		return;     // Alphacom 32 out of the printing mode
	int const speed = BIT(data, 1) ? 1 : 2;
	if (!m_host)
	{
		if (BIT(data, 2))
			return;
		update();
		// motor on: the next stylus arrives
		m_host = true;
		m_speed = speed;
		m_new_speed = 0;
		m_row_start = machine().time();
		m_pixel = -1;
		m_filled = 0;
		m_stylus = BIT(data, 7);
		m_row_done = false;
		m_timer->adjust(attotime::from_msec(20), 0, attotime::from_msec(20));
		return;
	}

	update();
	if (BIT(data, 2))
	{
		// motor off: the row in progress is finished
		int const x = current_x();
		if ((x >= 0) && (x < DOTS) && !m_row_done)
		{
			for (int i = std::max(m_filled, 0); i < DOTS; i++)
				m_row[i] |= m_stylus;
			output_row();
		}
		m_speed = 0;
		m_stylus = 0;
		m_host = false;
		if (BIT(m_buttons->read(), 0))
		{
			// the paper feed button keeps the motor running
			m_speed = 2;
			m_row_start = machine().time();
			m_pixel = -1;
			m_filled = 0;
			m_row_done = false;
		}
		else
			m_timer->adjust(attotime::never);
		return;
	}

	int const x = current_x();
	if (m_stylus && !BIT(data, 7) && (x == m_on_x) && (x >= 0) && (x < DOTS))
		m_row[x] = 1;   // switched off before the stylus left the dot it was switched on at: it is burnt
	if (!m_stylus && BIT(data, 7))
		m_on_x = x;
	m_stylus = BIT(data, 7);
	if ((x < 0) && (speed != m_speed))
	{
		// before the paper: the new speed applies at once, from the position reached
		attotime const now = machine().time();
		m_speed = speed;
		m_row_start = now - pixel_period(m_speed) * (x + START);
	}
	else if (x >= 0)
		m_new_speed = (speed != m_speed) ? speed : 0;   // takes effect at the next row
	m_pixel = std::max(x, -1);
}

INPUT_CHANGED_MEMBER(zx_printer_device::feed_w)
{
	if (!newval && m_test_armed)
	{
		// Alphacom 32: ON/ADVANCE released after OFF was pressed while it was held
		m_test_armed = false;
		update();
		m_speed = 0;
		m_stylus = 0;
		m_timer->adjust(attotime::never);
		m_test = true;
		m_test_row = 0;
		attotime const period = pixel_period(2) * ROW;
		m_test_timer->adjust(period, 0, period);
		return;
	}
	if (m_test)
		return;
	if (newval)
		m_off = false;  // Alphacom 32: back into the printing mode

	// the paper feed button runs the motor without printing
	if (newval)
	{
		if (!m_speed)
		{
			m_speed = 2;
			m_row_start = machine().time();
			m_pixel = -1;
			m_filled = 0;
			m_stylus = 0;
			m_row_done = false;
		}
		m_timer->adjust(attotime::from_msec(20), 0, attotime::from_msec(20));
	}
	else if (!m_host)
	{
		update();
		m_speed = 0;
		m_stylus = 0;
		m_timer->adjust(attotime::never);
	}
}

INPUT_CHANGED_MEMBER(zx_printer_device::off_w)
{
	// Alphacom 32 OFF key
	if (!newval)
		return;
	if (m_test)
	{
		// stops the self test, the printer stays out of the printing mode
		m_test = false;
		m_test_timer->adjust(attotime::never);
		m_off = true;
		return;
	}
	if (BIT(m_buttons->read(), 0))
	{
		// OFF while ON/ADVANCE is held: the self test starts when ON/ADVANCE is released
		m_test_armed = true;
		return;
	}
	m_off = true;
	if (!m_host)
		return;
	// the motor stops, the row in progress is finished
	update();
	int const x = current_x();
	if ((x >= 0) && (x < DOTS) && !m_row_done)
	{
		for (int i = std::max(m_filled, 0); i < DOTS; i++)
			m_row[i] |= m_stylus;
		output_row();
	}
	m_host = false;
	m_speed = 0;
	m_stylus = 0;
	m_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(zx_printer_device::test_row)
{
	// Alphacom 32 self test: lines of 32 characters, 8s and 1s in turn
	u8 const bits = TEST_GLYPHS[(m_test_row >> 3) & 1][m_test_row & 7];
	for (int i = 0; i < DOTS; i++)
		m_row[i] = BIT(bits, 7 - (i & 7));
	output_row();
	m_test_row = (m_test_row + 1) & 15;
}

TIMER_CALLBACK_MEMBER(zx_printer_device::tick)
{
	// the paper moves on while the motor runs, even without port accesses
	update();
}
