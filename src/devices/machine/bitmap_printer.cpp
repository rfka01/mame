// license:BSD-3-Clause
// copyright-holders: Golden Child
/*********************************************************************

    bitmap_printer.cpp

    Implementation of Bitmap Printer

**********************************************************************/

#include "emu.h"
#include "video.h"
#include "screen.h"
#include "emuopts.h"
#include "fileio.h"
#include "png.h"
#include "bitmap_printer.h"
#include "corestr.h"

#include <algorithm>
#include <cmath>
#include <numeric>

/***************************************************************************
    DEVICE DECLARATION
***************************************************************************/

DEFINE_DEVICE_TYPE(BITMAP_PRINTER, bitmap_printer_device, "bitmap_printer", "Bitmap Printer Device")

//**************************************************************************
//    INPUT PORTS
//**************************************************************************

#define PORT_ADJUSTER_16MASK(_default, _name)                   \
		configurer.field_alloc(IPT_ADJUSTER, (_default), 0xffff, (_name)); \
		configurer.field_set_min_max(0, 100);

INPUT_PORTS_START(bitmap_printer)
	PORT_START("DRAWMARKS")
	PORT_CONFNAME(0x3, 0x02, "Draw Inch Marks")
	PORT_CONFSETTING(0x0, "Off")
	PORT_CONFSETTING(0x1, "with marks")
	PORT_CONFSETTING(0x2, "with numbers")

	PORT_START("TOPMARGIN")
	PORT_ADJUSTER_16MASK(18, "Printer Top Margin")
	PORT_MINMAX(0,500)

	PORT_START("BOTTOMMARGIN")
	PORT_ADJUSTER_16MASK(18, "Printer Bottom Margin")
	PORT_MINMAX(0,500)

INPUT_PORTS_END


INPUT_PORTS_START(bitmap_printer_paper)
	PORT_INCLUDE(bitmap_printer)

	PORT_START("PAPER")
	PORT_CONFNAME(0x07, 0x00, "Paper format") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bitmap_printer_device::paper_changed), 0)
	PORT_CONFSETTING(0x00, "Printer default")
	PORT_CONFSETTING(0x01, "Fanfold 240 mm x 12\"")
	PORT_CONFSETTING(0x02, "Fanfold 9.5\" x 11\"")
	PORT_CONFSETTING(0x03, "A4 single sheets")
	PORT_CONFSETTING(0x04, "Letter single sheets")
	PORT_CONFNAME(0x08, 0x00, "Green bar fanfold paper") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bitmap_printer_device::paper_changed), 0)
	PORT_CONFSETTING(0x00, DEF_STR( No ))
	PORT_CONFSETTING(0x08, DEF_STR( Yes ))
	PORT_CONFNAME(0x50, 0x50, "Single sheet supply") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bitmap_printer_device::paper_changed), 0)
	PORT_CONFSETTING(0x50, "Always loaded (printer sees endless paper)")
	PORT_CONFSETTING(0x10, "Sheet feeder (paper sensor sees each sheet)")
	PORT_CONFSETTING(0x00, "Insert each sheet by hand")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Insert sheet") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bitmap_printer_device::insert_sheet), 0)
INPUT_PORTS_END

namespace {

struct paper_format
{
	double width, height;   // inches
	bool fanfold;           // with tractor strips
	double left;            // left edge in the mechanism, inches
};

// fanfold paper uses the full width of the mechanism (tractor strips at both edges);
// single sheets are put where the tear-off strip of fanfold paper would be
const paper_format paper_formats[bitmap_printer_device::PAPER_FORMATS] =
{
	{ 9.5,          12.0,          true,  0.0 },  // 240 mm x 12" fanfold
	{ 9.5,          11.0,          true,  0.0 },  // 9.5" x 11" fanfold
	{ 210.0 / 25.4, 297.0 / 25.4,  false, 0.5 },  // A4
	{ 8.5,          11.0,          false, 0.5 },  // US Letter
};

} // anonymous namespace

ioport_constructor bitmap_printer_device::device_input_ports() const
{
	return m_paper_mask ? INPUT_PORTS_NAME(bitmap_printer_paper) : INPUT_PORTS_NAME(bitmap_printer);
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void bitmap_printer_device::device_add_mconfig(machine_config &config)
{
	// video hardware (simulates paper)
	screen_device &screen(SCREEN(config, m_screen));
	screen.set_refresh_hz(60);
	screen.set_vblank_time(ATTOSECONDS_IN_USEC(0));
	configure_screen(screen);
	screen.set_screen_update(FUNC(bitmap_printer_device::screen_update_bitmap));

	// paper output: PDF document, or PNG snapshots when no file is mounted
	PDF_PRINTER(config, m_pdf);

	STEPPER(config, m_pf_stepper, (uint8_t) 0xa);
	STEPPER(config, m_cr_stepper, (uint8_t) 0xa);
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

bitmap_printer_device::bitmap_printer_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_cr_direction(1),
	m_xpos(0),
	m_ypos(0),
	m_screen(*this, "screen"),
	m_pdf(*this, "printout"),
	m_pf_stepper(*this, "pf_stepper"),
	m_cr_stepper(*this, "cr_stepper"),
	m_top_margin_ioport(*this, "TOPMARGIN"),
	m_bottom_margin_ioport(*this, "BOTTOMMARGIN"),
	m_draw_marks_ioport(*this, "DRAWMARKS"),
	m_paper_ioport(*this, "PAPER"),
	m_printhead_color(0x00EE00),
	m_printhead_bordercolor(0xEE0000),
	m_printhead_bordersize(2),
	m_printhead_xsize(10),
	m_printhead_ysize(20),
	m_page_dirty(0),
	m_paper_width(0),
	m_paper_height(0),
	m_hdpi(0),
	m_vdpi(0),
	m_clear_pos(0),
	m_newpage_flag(0),
	m_continuous_feed(false),
	m_feed_hi(0),
	m_feed_lo(0),
	m_roll_dirty(false),
	m_roll_collected(false),
	m_roll_first(0),
	m_roll_last(0),
	m_feed_forward(true),
	m_led_state{0,1,1,1,1},
	m_num_leds(1),
	m_pf_stepper_ratio0(1),
	m_pf_stepper_ratio1(1),
	m_cr_stepper_ratio0(1),
	m_cr_stepper_ratio1(1),
	m_dot_style_set(false),
	m_screen_height(DEFAULT_SCREEN_HEIGHT),
	m_view_serial(1),
	m_view_cached_serial(0),
	m_view_cached_scroll(0),
	m_paper_mask(0),
	m_paper_default(PAPER_FANFOLD_11),
	m_sheet_height(0),
	m_sheet_left(0),
	m_sheet_right(0),
	m_sheet_fanfold(true),
	m_greenbar(false),
	m_unlimited_sheets(true),
	m_always_loaded(true),
	m_sheet_loaded(true),
	m_load_offset(0.0),
	m_sheet_at_power_on(false),
	m_sheet_top(0.5),
	m_sensor_rows(0),
	m_feeder_timer(nullptr)
{
}

void bitmap_printer_device::set_paper_formats(u8 mask, int default_format)
{
	m_paper_mask = mask;
	m_paper_default = default_format;

	// the page bitmap must hold the longest sheet
	double height = 0.0;
	for (int i = 0; i < PAPER_FORMATS; i++)
		if (BIT(mask, i))
			height = std::max(height, paper_formats[i].height);
	m_paper_height = std::max(m_paper_height, int(std::ceil(height * m_vdpi)));
}

void bitmap_printer_device::apply_paper()
{
	if (!m_paper_mask)
		return;

	u32 const cfg = m_paper_ioport ? m_paper_ioport->read() : 0;
	int format = int(cfg & 0x07) - 1;
	if ((format < 0) || (format >= PAPER_FORMATS) || !BIT(m_paper_mask, format))
		format = m_paper_default;
	paper_format const &p = paper_formats[format];

	m_sheet_height = std::min(m_paper_height, int(std::lround(p.height * m_vdpi)));
	m_sheet_left = std::clamp(int(std::lround(p.left * m_hdpi)), 0, m_paper_width - 1);
	m_sheet_right = std::min(m_paper_width, m_sheet_left + int(std::lround(p.width * m_hdpi)));
	m_sheet_fanfold = p.fanfold;
	m_sensor_rows = int(std::lround(m_load_offset * m_vdpi));
	m_greenbar = p.fanfold && BIT(cfg, 3);
	m_unlimited_sheets = BIT(cfg, 4) || BIT(cfg, 6);
	m_always_loaded = BIT(cfg, 6);
	if (m_always_loaded)
		m_sheet_loaded = true;
	touch();
}

INPUT_CHANGED_MEMBER(bitmap_printer_device::paper_changed)
{
	apply_paper();
}

INPUT_CHANGED_MEMBER(bitmap_printer_device::insert_sheet)
{
	// a new single sheet is put into the printer
	if (newval && single_sheet() && !m_sheet_loaded)
		load_sheet();
}

void bitmap_printer_device::load_sheet()
{
	m_sheet_loaded = true;
	m_page_bitmap.fill(paper_color);
	m_clear_pos = 0;

	// the top edge of the new sheet rests at the paper sensor
	if (m_sensor_rows > 0)
	{
		m_newpage_flag = 0;
		m_ypos = -m_sensor_rows;
	}
	touch();
}

TIMER_CALLBACK_MEMBER(bitmap_printer_device::feed_sheet)
{
	if (!single_sheet() || !m_unlimited_sheets || m_sheet_loaded)
		return;

	// the feeder waits until the paper motor has finished ejecting the last sheet
	attotime const settle = attotime::from_msec(500);
	attotime const idle = machine().time() - m_last_feed;
	if (idle < settle)
		m_feeder_timer->adjust(settle - idle);
	else
		load_sheet();
}

namespace {

constexpr u32 platen_color = 0x3a3a3c;      // the mechanism behind and beside the paper
constexpr u32 greenbar_color = 0xd2ecd2;
constexpr u32 perforation_color = 0xc4c4c4;
constexpr u32 hole_file_color = 0xe4e4e4;   // tractor holes on the filed page
constexpr double STRIP = 0.5;               // tractor strip width, inches
constexpr double HOLE_PITCH = 0.5;
constexpr double HOLE_RADIUS = 5.0 / 64.0;  // 5/32" holes
constexpr double BAR = 1.0 / 6.0;            // green bar band height: one 6 lpi line (DIN computer paper, 4.23 mm)

} // anonymous namespace

// paper at a point of the sheet (inches from its top left); screen: holes show the platen
static u32 paper_point(double sx, double sy, double w, bool fanfold, bool greenbar, bool screen, bool top_row)
{
	if (fanfold)
	{
		// tractor holes
		double const hy = std::fmod(sy, HOLE_PITCH) - HOLE_PITCH / 2;
		for (double hx : { STRIP / 2, w - STRIP / 2 })
		{
			double const dx = sx - hx;
			if (dx * dx + hy * hy <= HOLE_RADIUS * HOLE_RADIUS)
				return screen ? platen_color : hole_file_color;
		}

		// the page's perforation, only on the screen where the next page follows
		if (screen && top_row && (std::fmod(sx, 0.125) < 0.075))
			return perforation_color;

		if (greenbar && (sx >= STRIP) && (sx < w - STRIP) && (std::fmod(sy, 2 * BAR) < BAR))
			return greenbar_color;
	}
	return 0xffffff;
}

void bitmap_printer_device::render_paper(bitmap_rgb32 &dst, int page_row0, int col0, bool screen)
{
	double const w = double(m_sheet_right - m_sheet_left) / m_hdpi;
	int const strip_l = m_sheet_left + int(std::lround(STRIP * m_hdpi));
	int const strip_r = m_sheet_right - int(std::lround(STRIP * m_hdpi)) - 1;

	for (int y = 0; y < dst.height(); y++)
	{
		u32 *const line = &dst.pix(y, 0);
		int row = page_row0 + y;
		bool on_sheet = true;
		if (m_sheet_fanfold)
			row = ((row % m_sheet_height) + m_sheet_height) % m_sheet_height;
		else
			on_sheet = (row >= 0) && (row < m_sheet_height);

		double const sy = (row + 0.5) / m_vdpi;
		bool const top_row = (row == 0);
		bool const dashed = std::fmod(sy, 0.125) < 0.075;   // vertical perforation of the strips

		for (int x = 0; x < dst.width(); x++)
		{
			int const col = col0 + x;
			if (!on_sheet || (col < m_sheet_left) || (col >= m_sheet_right))
			{
				line[x] = platen_color;
				continue;
			}
			if (m_sheet_fanfold && dashed && ((col == strip_l) || (col == strip_r)))
			{
				line[x] = perforation_color;
				continue;
			}
			line[x] = paper_point((col - m_sheet_left + 0.5) / m_hdpi, sy, w, m_sheet_fanfold, m_greenbar, screen, top_row);
		}
	}
}

void bitmap_printer_device::paper_canvas(printer_page_canvas &page)
{
	if (!m_sheet_fanfold)
		return;

	double const w = page.width_in(), h = page.height_in();

	if (m_greenbar)
		for (double y = 0; y < h; y += 2 * BAR)
			page.fill_rect(STRIP, y, w - 2 * STRIP, std::min(BAR, h - y), greenbar_color);

	for (double y = HOLE_PITCH / 2; y < h; y += HOLE_PITCH)
	{
		page.fill_circle(STRIP / 2, y, HOLE_RADIUS, hole_file_color);
		page.fill_circle(w - STRIP / 2, y, HOLE_RADIUS, hole_file_color);
	}

	double const t = 1.0 / page.dpi();
	for (double y = 0; y < h; y += 0.125)
	{
		page.fill_rect(STRIP, y, t, 0.075, perforation_color);
		page.fill_rect(w - STRIP - t, y, t, 0.075, perforation_color);
	}
}

void bitmap_printer_device::configure_screen(screen_device &screen)
{
	screen.set_size(m_paper_width, m_screen_height);
	screen.set_visarea(0, m_paper_width - 1, 0, m_screen_height - 1);

	// the paper's real proportions, also when the resolution differs horizontally and vertically
	if ((m_paper_width > 0) && (m_hdpi > 0) && (m_vdpi > 0))
	{
		u32 x = u32(m_paper_width) * u32(m_vdpi), y = u32(m_screen_height) * u32(m_hdpi);
		u32 const g = std::gcd(x, y);
		screen.set_physical_aspect(x / g, y / g);
	}
}

void bitmap_printer_device::set_paper_view(double inches)
{
	m_screen_height = std::max(64, int(std::lround(inches * m_vdpi)));
	screen_device *const screen = subdevice<screen_device>("screen");
	if (screen)
		configure_screen(*screen);
}

bitmap_printer_device::bitmap_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	bitmap_printer_device(mconfig, BITMAP_PRINTER, tag, owner, clock)
{
}

bitmap_printer_device::bitmap_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, int paper_width, int paper_height, int hdpi, int vdpi) :
	bitmap_printer_device(mconfig, tag, owner, u32(0))
{
	m_paper_width = paper_width;
	m_paper_height = paper_height;
	m_hdpi = hdpi;
	m_vdpi = vdpi;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void bitmap_printer_device::device_start()
{
	m_page_bitmap.allocate(m_paper_width, m_paper_height);
	m_page_bitmap.fill(paper_color);  // Start with a white piece of paper

	// file the page in progress when the PDF is closed (unmounted or exit)
	m_pdf->set_flush_handler([this] () { flush_to_pdf(); });

	if (m_continuous_feed)
	{
		m_roll_bitmap.allocate(m_paper_width, m_paper_height);
		m_roll_bitmap.fill(paper_color);
		save_item(NAME(m_roll_bitmap));
		save_item(NAME(m_roll_dirty));
		save_item(NAME(m_roll_collected));
		save_item(NAME(m_roll_first));
		save_item(NAME(m_roll_last));
		save_item(NAME(m_feed_forward));
	}

	save_item(NAME(m_page_bitmap));
	save_item(NAME(m_xpos));
	save_item(NAME(m_ypos));
	save_item(NAME(m_cr_direction));
	save_item(NAME(m_pf_stepper_ratio0));
	save_item(NAME(m_pf_stepper_ratio1));
	save_item(NAME(m_cr_stepper_ratio0));
	save_item(NAME(m_cr_stepper_ratio1));
	save_item(NAME(m_printhead_color));
	save_item(NAME(m_printhead_bordercolor));
	save_item(NAME(m_printhead_bordersize));
	save_item(NAME(m_printhead_xsize));
	save_item(NAME(m_printhead_ysize));
	save_item(NAME(m_page_dirty));
	save_item(NAME(m_paper_width));
	save_item(NAME(m_paper_height));
	save_item(NAME(m_hdpi));
	save_item(NAME(m_vdpi));
	save_item(NAME(m_clear_pos));
	save_item(NAME(m_newpage_flag));
	save_item(NAME(m_continuous_feed));
	save_item(NAME(m_feed_hi));
	save_item(NAME(m_feed_lo));
	save_item(NAME(m_sheet_loaded));

	m_feeder_timer = timer_alloc(FUNC(bitmap_printer_device::feed_sheet), this);
}

void bitmap_printer_device::device_stop()
{
	if (!m_continuous_feed)
	{
		// without a PDF, a page that was started is saved as a PNG snapshot on exit
		// (with a PDF the page is filed when the image is unloaded)
		if (!m_pdf->exists() && sheet_in())
		{
			clear_to_pos(page_rows() - 1);  // the rest of the previous page shown below the head
			for (int row = 0; row < page_rows(); row++)
			{
				if (row_has_ink(m_page_bitmap, row))
				{
					write_snapshot_to_file();
					break;
				}
			}
		}
		return;
	}

	flush_roll();

	int const live = m_feed_forward ? (m_feed_hi - m_paper_height + 1) : m_feed_lo;

	for (int row = 0; row < m_paper_height; row++)
	{
		if (row_has_ink(m_page_bitmap, row))
		{
			write_roll_page(m_page_bitmap, live);
			break;
		}
	}
}

void bitmap_printer_device::device_reset_after_children()
{
	m_ypos = sheet_start_row();

	if (m_continuous_feed)
		reset_feed_marks();
	else
		m_clear_pos = std::max(m_clear_pos, m_ypos + m_distfrombottom + 1);  // the paper up to the head is in place: ink printed before the first feed stays
}

void bitmap_printer_device::device_reset()
{
	// single sheets: the paper sensor sees the first sheet come in after power on
	apply_paper();
	m_sheet_loaded = !single_sheet() || m_always_loaded || m_sheet_at_power_on;
	if (!m_sheet_loaded)
		m_feeder_timer->adjust(attotime::from_msec(1500));
}

//-------------------------------------------------
//    SCREEN UPDATE FUNCTIONS
//-------------------------------------------------

int bitmap_printer_device::calc_scroll_y(bitmap_rgb32& bitmap)
{
	return bitmap.height() - m_distfrombottom - m_ypos;
}

int bitmap_printer_device::wrap_row(int y) const
{
	if (!m_continuous_feed)
		return y;

	int const h = m_paper_height;
	int const w = y % h;
	return (w < 0) ? w + h : w;
}

void bitmap_printer_device::reset_feed_marks()
{
	m_feed_hi = m_ypos + m_distfrombottom;
	m_feed_lo = m_ypos - m_distfrombottom;
}

void bitmap_printer_device::clear_roll_rows(int from_row, int to_row, u32 color)
{
	if (to_row < from_row)
		return;

	from_row = std::max(from_row, to_row - m_paper_height + 1);

	for (int y = from_row; y <= to_row; y++)
		m_page_bitmap.plot_box(0, wrap_row(y), m_paper_width, 1, color);
}

bool bitmap_printer_device::row_has_ink(bitmap_rgb32 &bitmap, int row) const
{
	u32 const *const line = &bitmap.pix(row, 0);
	return std::any_of(line, line + m_paper_width, [] (u32 pixel) { return pixel != paper_color; });
}

void bitmap_printer_device::write_roll_page(bitmap_rgb32 &bitmap, int first_row)
{
	// page height follows the printing, not the ring. Only the outside is
	// trimmed - blank rows between two parts of a plot are its layout
	int top = 0, bottom = m_paper_height - 1;

	while ((top <= bottom) && !row_has_ink(bitmap, wrap_row(first_row + top)))
		top++;

	if (top > bottom)
		return;  // nothing on this stretch of roll

	while (!row_has_ink(bitmap, wrap_row(first_row + bottom)))
		bottom--;

	// or a job that drew one line files a one-pixel-tall page
	top = std::max(top - m_distfrombottom, 0);
	bottom = std::min(bottom + m_distfrombottom, m_paper_height - 1);

	bitmap_rgb32 page(m_paper_width, bottom - top + 1);

	for (int y = top; y <= bottom; y++)
		std::copy_n(&bitmap.pix(wrap_row(first_row + y), 0), m_paper_width, &page.pix(y - top, 0));

	write_bitmap_to_file(page);
}

bool bitmap_printer_device::flush_roll()
{
	bool const saved = m_roll_dirty;

	if (m_roll_dirty)
		write_roll_page(m_roll_bitmap, m_roll_first);

	m_roll_bitmap.fill(paper_color);
	m_roll_dirty = false;
	m_roll_collected = false;

	return saved;
}

bool bitmap_printer_device::retire_roll_rows(int from_row, int to_row, bool forward)
{
	bool saved = false;

	if (to_row < from_row)
		return false;

	from_row = std::max(from_row, to_row - m_paper_height + 1);

	for (int i = 0; i <= to_row - from_row; i++)
	{
		int const src = forward ? (from_row + i - m_paper_height) : (to_row - i + m_paper_height);
		int const row = wrap_row(src);

		// the two directions retire from opposite ends of the roll, a whole
		// paper length apart, so a reversal mid-lap has to close the pending
		// image out rather than blend two stretches of roll into one page
		if (m_roll_collected && (src != m_roll_last + 1) && (src != m_roll_first - 1))
			saved |= flush_roll();

		std::copy_n(&m_page_bitmap.pix(row, 0), m_paper_width, &m_roll_bitmap.pix(row, 0));

		if (m_roll_collected)
		{
			m_roll_first = std::min(m_roll_first, src);
			m_roll_last = std::max(m_roll_last, src);
		}
		else
		{
			m_roll_first = m_roll_last = src;
			m_roll_collected = true;
		}

		if (row_has_ink(m_page_bitmap, row))
			m_roll_dirty = true;

		if (row == (forward ? m_paper_height - 1 : 0))
			saved |= flush_roll();
	}

	return saved;
}

uint32_t bitmap_printer_device::screen_update_bitmap(screen_device &screen,
							 bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	static constexpr u32 top_edge_color = 0xEEE8AA;
	static constexpr u32 bottom_edge_color = 0xEE8844;
	static constexpr u32 coverup_color = 0xDDDDDD;

	int scrolly = calc_scroll_y(bitmap);

	if (m_paper_mask)
	{
		// selectable paper: the sheet on the platen, with the ink stamped on it
		if ((m_view_cache.width() != bitmap.width()) || (m_view_cache.height() != bitmap.height()))
		{
			m_view_cache.allocate(bitmap.width(), bitmap.height());
			m_dot_view.allocate(bitmap.width(), bitmap.height());
			m_view_cached_serial = m_view_serial - 1;
		}
		if ((m_view_cached_serial != m_view_serial) || (m_view_cached_scroll != scrolly))
		{
			int const rows = page_rows();
			for (int y = 0; y < m_dot_view.height(); y++)
			{
				// above the top of the page: the end of the previous fanfold page
				int row = y - scrolly;
				if (m_sheet_fanfold && (row < 0))
					row += rows;
				u32 *const dst = &m_dot_view.pix(y, 0);
				if ((row >= 0) && (row < rows) && ((y - scrolly >= 0) || sheet_in()))
					std::copy_n(&m_page_bitmap.pix(row, 0), std::min(m_dot_view.width(), m_paper_width), dst);
				else
					std::fill_n(dst, m_dot_view.width(), paper_color);
			}
			if (sheet_in())
				render_paper(m_view_cache, -scrolly, 0, true);
			else
				m_view_cache.fill(platen_color);
			expand_dots(m_dot_view, m_view_cache);
			m_view_cached_serial = m_view_serial;
			m_view_cached_scroll = scrolly;
		}
		copybitmap(bitmap, m_view_cache, 0, 0, 0, 0, cliprect);
	}
	else if (m_dot_style_set)
	{
		// printers that plot dot centers get their dots drawn at full size, so the
		// paper looks like the printout rather than a sparse grid of single pixels;
		// redone only when the paper changed or moved
		if ((m_view_cache.width() != bitmap.width()) || (m_view_cache.height() != bitmap.height()))
		{
			m_view_cache.allocate(bitmap.width(), bitmap.height());
			m_dot_view.allocate(bitmap.width(), bitmap.height());
			m_view_cached_serial = m_view_serial - 1;
		}
		if ((m_view_cached_serial != m_view_serial) || (m_view_cached_scroll != scrolly))
		{
			copyscrollbitmap(m_dot_view, m_page_bitmap, 0, nullptr, 1, &scrolly, m_dot_view.cliprect());
			m_view_cache.fill(paper_color);
			expand_dots(m_dot_view, m_view_cache);
			m_view_cached_serial = m_view_serial;
			m_view_cached_scroll = scrolly;
		}
		copybitmap(bitmap, m_view_cache, 0, 0, 0, 0, cliprect);
	}
	else
		copyscrollbitmap(bitmap, m_page_bitmap, 0, nullptr, 1, &scrolly, cliprect);

	// an endless roll has no page edges, and the buffer wrap isn't a seam
	if (!m_continuous_feed && !m_paper_mask)
	{
		// draw a line on the very top of the top edge of page
		bitmap.plot_box(0, bitmap.height() - m_distfrombottom - m_ypos, m_paper_width, 2, top_edge_color);
		// draw a line on the bottom edge of page
		bitmap.plot_box(0, bitmap.height() - m_distfrombottom - m_ypos + m_paper_height, m_paper_width, 2, bottom_edge_color);
		// cover up visible parts of current page at the bottom
		bitmap.plot_box(0, bitmap.height() - m_distfrombottom - m_ypos + m_paper_height + 2, m_paper_width, m_distfrombottom, coverup_color);
	}

	draw_printhead(bitmap, std::max(m_xpos, 0) , bitmap.height() - m_distfrombottom);

	draw_inch_marks(bitmap);

	return 0;
}

void bitmap_printer_device::expand_dots(const bitmap_rgb32 &src, bitmap_rgb32 &dst)
{
	rectangle const &clip = dst.cliprect();

	int const rx = m_dot_style_set ? std::max(int(std::lround(m_dot_style.diameter * m_hdpi / 2.0)), 0) : 0;
	int const ry = m_dot_style_set ? std::max(int(std::lround(m_dot_style.diameter * m_vdpi / 2.0)), 0) : 0;

	for (int y = clip.min_y; y <= clip.max_y; y++)
	{
		u32 const *const line = &src.pix(y, 0);
		for (int x = clip.min_x; x <= clip.max_x; x++)
		{
			u32 const color = line[x];
			if ((color & 0xffffff) == paper_color)
				continue;

			// filled ellipse around the dot center
			for (int dy = -ry; dy <= ry; dy++)
			{
				int const py = y + dy;
				if ((py < clip.min_y) || (py > clip.max_y))
					continue;
				double const f = ry ? double(dy) / (ry + 0.5) : 0.0;
				int const w = int((rx + 0.5) * std::sqrt(std::max(0.0, 1.0 - f * f)));
				int const x0 = std::max(x - w, clip.min_x), x1 = std::min(x + w, clip.max_x);
				u32 *const out = &dst.pix(py, 0);
				for (int px = x0; px <= x1; px++)
					out[px] = color;
			}
		}
	}
}

//-------------------------------------------------
//    BITMAP CLEARING FUNCTIONS
//-------------------------------------------------

void bitmap_printer_device::clear_to_pos(int to_line, u32 color)
{
	int from_line = m_clear_pos;
	to_line = std::min(m_page_bitmap.height(), to_line);
	if (to_line >= from_line)
	{
		bitmap_clear_band(m_page_bitmap, from_line, to_line, color);
	}
	m_clear_pos = std::max(m_clear_pos, to_line + 1);
}

void bitmap_printer_device::bitmap_clear_band(int from_line, int to_line, u32 color)
{
	bitmap_clear_band(m_page_bitmap, from_line, to_line, color);
}

void bitmap_printer_device::bitmap_clear_band(bitmap_rgb32 &bitmap, int from_line, int to_line, u32 color)
{
	touch();
	bitmap.plot_box(0, from_line, m_paper_width, to_line - from_line + 1, color);
}

//-------------------------------------------------
//    PRINTHEAD FUNCTIONS
//-------------------------------------------------

void bitmap_printer_device::set_printhead_color(int headcolor, int bordcolor)
{
	m_printhead_color = headcolor;
	m_printhead_bordercolor = bordcolor;
}

void bitmap_printer_device::set_printhead_size(int xsize, int ysize, int bordersize)
{
	m_printhead_xsize = xsize;
	m_printhead_ysize = ysize;
	m_printhead_bordersize = bordersize;
}

void bitmap_printer_device::set_led_state(int led, int value)
{
	m_led_state[led] = value;
	m_num_leds = std::max(m_num_leds, led);
}

void bitmap_printer_device::setheadpos(int x, int y)
{
	if (m_xpos != x)
	{
		m_newpage_flag = 0;
	}
	m_xpos = x;
	m_ypos = y;
}

u32 bitmap_printer_device::dimcolor(u32 incolor, int factor)
{
	return  (((incolor & 0xff0000) >> 16) / factor << 16) |
			(((incolor & 0xff00) >> 8) / factor << 8) |
			(((incolor & 0xff) >> 0) / factor);
}

void bitmap_printer_device::draw_printhead(bitmap_rgb32 &bitmap, int x, int y)
{
	int bordx = m_printhead_bordersize;
	int bordy = m_printhead_bordersize;
	int offy = 9 + bordy;
	int sizex = m_printhead_xsize;
	int sizey = m_printhead_ysize;
	bitmap.plot_box(x - sizex / 2- bordx, y + offy - bordy, sizex + 2 * bordx, sizey + bordy * 2,
		m_led_state[0] ? m_printhead_bordercolor : dimcolor(m_printhead_bordercolor, 4));

	for (int i = 1; i <= m_num_leds; i++)
	bitmap.plot_box(x - sizex / 2, y + offy + ((i -1) * sizey / m_num_leds), sizex,
		((i+1) * sizey / m_num_leds) - (i * sizey / m_num_leds),
		m_led_state[i] ? m_printhead_color : dimcolor(m_printhead_color, 4));
}

//-------------------------------------------------
//    DRAW INCH MARKS AND NUMBERS
//-------------------------------------------------

void bitmap_printer_device::draw7seg(u8 data, bool is_digit, int x0, int y0, int width, int height, int thick, bitmap_rgb32 &bitmap, u32 color, u32 erasecolor)
{
	// pass nonzero erasecolor to erase blank segments
	const u8 pat[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };
	u8 seg = is_digit ? pat[data & 0xf] : data;

	if (BIT(seg,0) || erasecolor) bitmap.plot_box(x0,       y0,                  width, thick,       BIT(seg,0) ? color : erasecolor);
	if (BIT(seg,1) || erasecolor) bitmap.plot_box(x0+width, y0+thick,            thick, height,      BIT(seg,1) ? color : erasecolor);
	if (BIT(seg,2) || erasecolor) bitmap.plot_box(x0+width, y0+2*thick+height,   thick, height,      BIT(seg,2) ? color : erasecolor);
	if (BIT(seg,3) || erasecolor) bitmap.plot_box(x0,       y0+2*thick+2*height, width, thick,       BIT(seg,3) ? color : erasecolor);
	if (BIT(seg,4) || erasecolor) bitmap.plot_box(x0-thick, y0+2*thick+height,   thick, height,      BIT(seg,4) ? color : erasecolor);
	if (BIT(seg,5) || erasecolor) bitmap.plot_box(x0-thick, y0+thick,            thick, height,      BIT(seg,5) ? color : erasecolor);
	if (BIT(seg,6) || erasecolor) bitmap.plot_box(x0,       y0+thick+height,     width, thick,       BIT(seg,6) ? color : erasecolor);
	if (BIT(seg,7) || erasecolor) bitmap.plot_box(x0+width+thick, y0+2*thick+2*height, thick, thick, BIT(seg,7) ? color : erasecolor); // draw dot
}

void bitmap_printer_device::draw_number(int number, int x, int y, bitmap_rgb32& bitmap)
{
	std::string s(std::to_string(number));

	int width = std::max(m_hdpi / 15, 1);  // 1/10 of inch
	int height = std::max(m_vdpi / 30, 1);
	int thick = std::max(m_vdpi / 72, 1);

	for (int i = s.length() - 1; i >= 0; i--)
		draw7seg( s.at(i) - 0x30, true,
					x + ( i - s.length()) * (3 * width) - (width), y + height * 3 / 2,
					width, height, thick, bitmap, 0x000000, 0);
}

void bitmap_printer_device::draw_inch_marks(bitmap_rgb32& bitmap)
{
	static constexpr u32 dark_grey_color = 0x202020;
	static constexpr u32 light_grey_color = 0xc0c0c0;

	int drawmarks = m_draw_marks_ioport->read();
	if (!drawmarks) return;

	int const rows = m_paper_mask ? page_rows() : m_vdpi * 11;
	for (int i = 0; i < rows; i += m_vdpi / 4)
	{
		int adj_i = i + calc_scroll_y(bitmap) % page_rows();
		int barbase = m_vdpi / 6;
		int barwidth = ((i % m_vdpi) == 0) ? barbase * 2 : barbase;
		int barcolor = ((i % m_vdpi) == 0) ? dark_grey_color : light_grey_color;
		if (adj_i < bitmap.height())
		{
			bitmap.plot_box(bitmap.width() - 1 - barwidth, adj_i, barwidth, 1, barcolor);
			if ((i % m_vdpi) == 0)
			{
				if (drawmarks & 2)
					draw_number(i / m_vdpi, bitmap.width(), adj_i, bitmap);
			}
		}
	}
}

//-------------------------------------------------
//    DRAW PIXEL FUNCTIONS
//-------------------------------------------------

void bitmap_printer_device::draw_pixel(int x, int y, int pixelval)
{
	if (m_continuous_feed) y = wrap_row(y);
	else if (y >= m_page_bitmap.height()) y = m_page_bitmap.height() - 1;
	else if (y < 0) return;   // below the print line: no paper there yet   // below the print line: no paper there yet
	if (x >= m_page_bitmap.width()) x = m_page_bitmap.width() - 1;

	m_page_bitmap.pix(y, x) = pixelval;

	m_page_dirty = 1;
	touch();
};

int bitmap_printer_device::get_pixel(int x, int y)
{
	if (m_continuous_feed) y = wrap_row(y);
	else if (y >= m_page_bitmap.height()) y = m_page_bitmap.height() - 1;
	else if (y < 0) return paper_color;
	if (x >= m_page_bitmap.width()) x = m_page_bitmap.width() - 1;

	return m_page_bitmap.pix(y, x);
};

unsigned int& bitmap_printer_device::pix(int y, int x)    // reversed y x
{
	if (m_continuous_feed) y = wrap_row(y);
	else if (y >= m_page_bitmap.height()) y = m_page_bitmap.height() - 1;
	else if (y < 0) y = 0;
	if (x >= m_page_bitmap.width()) x = m_page_bitmap.width() - 1;

	touch();    // the caller may write through the reference
	return m_page_bitmap.pix(y,x);
};

//-------------------------------------------------
//    WRITE SNAPSHOT TO FILE
//-------------------------------------------------

void bitmap_printer_device::write_snapshot_to_file()
{
	if (!m_paper_mask)
	{
		write_bitmap_to_file(m_page_bitmap);
		return;
	}

	// only the sheet is filed, not the whole width and length of the mechanism
	bitmap_rgb32 sheet(m_sheet_right - m_sheet_left, m_sheet_height);
	for (int y = 0; y < m_sheet_height; y++)
		std::copy_n(&m_page_bitmap.pix(y, m_sheet_left), sheet.width(), &sheet.pix(y, 0));
	write_bitmap_to_file(sheet);
}

void bitmap_printer_device::flush_to_pdf()
{
	if (m_continuous_feed)
	{
		flush_roll();

		int const live = m_feed_forward ? (m_feed_hi - m_paper_height + 1) : m_feed_lo;
		for (int row = 0; row < m_paper_height; row++)
		{
			if (row_has_ink(m_page_bitmap, row))
			{
				write_roll_page(m_page_bitmap, live);
				break;
			}
		}
	}
	else if (sheet_in())
	{
		clear_to_pos(page_rows() - 1);  // the rest of the previous page shown below the head
		for (int row = 0; row < page_rows(); row++)
		{
			if (row_has_ink(m_page_bitmap, row))
			{
				write_snapshot_to_file();
				break;
			}
		}
	}

	// the sheet has been torn off
	m_page_bitmap.fill(paper_color);
	touch();
}

void bitmap_printer_device::write_bitmap_to_file(bitmap_rgb32 &bitmap)
{
	if (m_pdf->exists())
	{
		if (m_paper_mask)
			m_pdf->print_bitmap(bitmap, m_hdpi, m_vdpi, m_dot_style_set ? &m_dot_style : nullptr, paper_color,
					[this] (printer_page_canvas &page) { paper_canvas(page); }, m_greenbar);
		else
			m_pdf->print_bitmap(bitmap, m_hdpi, m_vdpi, m_dot_style_set ? &m_dot_style : nullptr, paper_color);
		return;
	}

	machine().popmessage("writing printer snapshot");

	emu_file file(machine().options().snapshot_directory(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
	std::error_condition const filerr = machine().video().open_next(file, "png");

	if (!filerr)
	{
		static const rgb_t png_palette[] = { rgb_t::white(), rgb_t::black() };

		// save the paper into a png; printers that plot dot centers get full-size dots
		bitmap_rgb32 dots;
		bitmap_rgb32 const *page = &bitmap;
		if (m_paper_mask)
		{
			// the sheet with its strips and bars, and the ink on it
			dots.allocate(bitmap.width(), bitmap.height());
			render_paper(dots, 0, m_sheet_left, false);
			expand_dots(bitmap, dots);
			page = &dots;
		}
		else if (m_dot_style_set)
		{
			dots.allocate(bitmap.width(), bitmap.height());
			dots.fill(paper_color);
			expand_dots(bitmap, dots);
			page = &dots;
		}

		// square pixels, so image viewers show the page in its real proportions
		if ((m_hdpi > 0) && (m_vdpi > 0) && (m_hdpi != m_vdpi))
		{
			int const dpi = std::max(m_hdpi, m_vdpi);
			bitmap_rgb32 square(page->width() * dpi / m_hdpi, page->height() * dpi / m_vdpi);
			for (int y = 0; y < square.height(); y++)
			{
				u32 const *const src = &page->pix(y * m_vdpi / dpi, 0);
				u32 *const dst = &square.pix(y, 0);
				for (int x = 0; x < square.width(); x++)
					dst[x] = src[x * m_hdpi / dpi];
			}
			util::png_write_bitmap(file, nullptr, square, 2, png_palette);
		}
		else
			util::png_write_bitmap(file, nullptr, *page, 2, png_palette);
	}
}

//-------------------------------------------------
//    STEPPER AND MARGIN FUNCTIONS
//-------------------------------------------------

int bitmap_printer_device::get_top_margin()    { return m_top_margin_ioport->read(); }

int bitmap_printer_device::sheet_start_row()
{
	// an always loaded single sheet starts where the printer puts a sheet it loads
	if (single_sheet() && m_always_loaded)
		return std::max(get_top_margin(), int(std::lround(m_sheet_top * m_vdpi)));
	return get_top_margin();
}
int bitmap_printer_device::get_bottom_margin() { return m_bottom_margin_ioport->read(); }

bool bitmap_printer_device::check_new_page()
{
	bool retval = false;
	m_last_feed = machine().time();

	if (m_continuous_feed)
	{
		// endless roll: m_ypos never resets, only the storage row wraps, and
		// fresh paper is decided in roll coordinates rather than buffer rows.
		// Y is bidirectional, so the pen coming back over a row it has
		// already drawn on must leave that ink alone
		int const head = m_ypos + m_distfrombottom;
		int const tail = m_ypos - m_distfrombottom;

		// rows are retired as the clear is about to recycle them. Filing them
		// on the lap change instead loses ink: the head leads the pen by
		// m_distfrombottom, so those rows go into the file blank and what the
		// pen draws on them afterwards is cleared a lap later. Either end can
		// be the one exposing new paper - the pf ratio's sign decides which.
		//
		// the live window can never be wider than the ring: let the two marks
		// drift more than a paper length apart and rows a lap apart share a
		// buffer row while both count as visited, so the pen silently
		// overdraws ink that no file ever received
		if (head > m_feed_hi)
		{
			retval |= retire_roll_rows(m_feed_hi + 1, head, true);
			clear_roll_rows(m_feed_hi + 1, head);
			m_feed_hi = head;
			m_feed_lo = std::max(m_feed_lo, m_feed_hi - m_paper_height + 1);
			m_feed_forward = true;
		}

		if (tail < m_feed_lo)
		{
			retval |= retire_roll_rows(tail, m_feed_lo - 1, false);
			clear_roll_rows(tail, m_feed_lo - 1);
			m_feed_lo = tail;
			m_feed_hi = std::min(m_feed_hi, m_feed_lo + m_paper_height - 1);
			m_feed_forward = false;
		}

		return retval;
	}

	// idea here is that you update the position, then check the page, this will do the saving of the page
	// if this routine returns true, means there's a new page and you should clear the yposition
	if (m_newpage_flag == 1)
	{
		// if you change m_ypos you have to change the stepper abs position too
		m_ypos = sheet_start_row();  // lock to the top of page until we seek horizontally
		m_pf_stepper->set_absolute_position(m_ypos / m_pf_stepper_ratio0 * m_pf_stepper_ratio1);
	}

	// If we are at the bottom of the page we will
	// write the page to a file, then erase the top part of the page
	// so we can still see the last page printed.
	if (m_ypos > page_rows() - 1 - get_bottom_margin())
	{
		// clear paper to bottom from current position
		clear_to_pos(page_rows() - 1, rgb_t::white());

		// save a snapshot (a sheet that has run out was filed when it did)
		if (sheet_in())
			write_snapshot_to_file();

		// a single sheet put in by hand or from a feeder: the paper sensor reports
		// the end of the sheet (an always loaded sheet follows without a gap)
		if (single_sheet() && !m_always_loaded)
		{
			m_sheet_loaded = false;
			if (m_unlimited_sheets)
				m_feeder_timer->adjust(attotime::from_msec(500));
		}

		m_newpage_flag = 1;

		// the next sheet takes the paper selected now
		int const old_rows = page_rows();
		apply_paper();
		if ((page_rows() != old_rows) || single_sheet())
			m_page_bitmap.fill(paper_color);   // a new single sheet is blank all over

		// clear page down to visible area, starting from the top of page
		m_clear_pos = 0;
		clear_to_pos(page_rows() - 1 - m_screen_height);

		// lock to the top of page until we seek horizontally: an always loaded sheet
		// comes in like one loaded by the printer, the rest of the paper feed is lost
		m_ypos = sheet_start_row();
		m_pf_stepper->set_absolute_position(m_ypos / m_pf_stepper_ratio0 * m_pf_stepper_ratio1);
		retval = true;
	}
	else
	{
		clear_to_pos ( m_ypos + m_distfrombottom);
	}
	return retval;
}

int bitmap_printer_device::update_stepper_delta(stepper_device * stepper, uint8_t pattern)
{
	int lastpos = stepper->get_absolute_position();
	stepper->update(pattern);
	int delta = stepper->get_absolute_position() - lastpos;
	return delta;
}

// When sending patterns to the update_cr_stepper and update_pf_stepper
// functions, the stepper device uses a "standard drive table"
// so you have to match that drive table by using a bitswap function.
// If the stepper drive is in the opposite direction, just reverse the
// bits in the bitswap.

void bitmap_printer_device::update_cr_stepper(int pattern)
{
	int delta = update_stepper_delta(m_cr_stepper, pattern);

	if (delta != 0)
	{
		m_newpage_flag = 0;

		if      (delta > 0) {m_cr_direction = 1;}
		else if (delta < 0) {m_cr_direction = -1;}
	}
	m_xpos = m_cr_stepper->get_absolute_position() * m_cr_stepper_ratio0 / m_cr_stepper_ratio1;
}

void bitmap_printer_device::update_pf_stepper(int pattern)
{
	update_stepper_delta(m_pf_stepper, pattern);
	m_ypos = m_pf_stepper->get_absolute_position() * m_pf_stepper_ratio0 / m_pf_stepper_ratio1;
	check_new_page();
}

void bitmap_printer_device::set_pf_stepper_ratio(int ratio0, int ratio1)
{
	m_pf_stepper_ratio0 = ratio0;
	m_pf_stepper_ratio1 = ratio1;
}

void bitmap_printer_device::set_cr_stepper_ratio(int ratio0, int ratio1)
{
	m_cr_stepper_ratio0 = ratio0;
	m_cr_stepper_ratio1 = ratio1;
}

