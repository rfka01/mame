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

    The Alphacom 32 is a thermal printer that answers the same port.

    The timing follows IMC's xz80 model (as used by Fuse): a dot takes
    220 Spectrum T-states (440 at slow speed), a row 384 dot times of
    which 256 are on the paper, starting 64 dot times after the row.

    Not emulated: the Alphacom 32 self test and its OFF button.

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

} // anonymous namespace


DEFINE_DEVICE_TYPE(ZX_PRINTER, zx_printer_device, "zx_printer", "Sinclair ZX Printer mechanism")
DEFINE_DEVICE_TYPE(ALPHACOM32, alphacom32_device, "alphacom32", "Alphacom 32 mechanism")


static INPUT_PORTS_START( zx_printer )
	PORT_START("BUTTONS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Paper Feed") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(zx_printer_device::feed_w), 0)
INPUT_PORTS_END

ioport_constructor zx_printer_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( zx_printer );
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

	if (type() == ALPHACOM32)
		config.set_default_layout(layout_alphacom32);
	else
		config.set_default_layout(layout_zxprinter);
}

void zx_printer_device::device_start()
{
	m_timer = timer_alloc(FUNC(zx_printer_device::tick), this);

	save_item(NAME(m_host));
	save_item(NAME(m_speed));
	save_item(NAME(m_new_speed));
	save_item(NAME(m_row_start));
	save_item(NAME(m_pixel));
	save_item(NAME(m_filled));
	save_item(NAME(m_stylus));
	save_item(NAME(m_row_done));
	save_item(NAME(m_row));
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
	m_row_done = false;
	std::fill(std::begin(m_row), std::end(m_row), 0);
	m_timer->adjust(attotime::never);
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
	int const ink = (type() == ALPHACOM32) ? 0x000000 : 0x2c2c30;
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
			m_row[i] = m_stylus;
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
				m_row[i] = m_stylus;
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

	m_stylus = BIT(data, 7);
	int const x = current_x();
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

TIMER_CALLBACK_MEMBER(zx_printer_device::tick)
{
	// the paper moves on while the motor runs, even without port accesses
	update();
}
