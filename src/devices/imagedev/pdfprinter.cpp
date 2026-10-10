// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    pdfprinter.cpp

    Printer output to PDF documents

    The PDF is written incrementally: a page is appended to the file as
    soon as it has been compressed (on a worker thread, so the emulation
    does not stall), the page tree, catalog and cross-reference table
    follow when the image is unloaded or MAME exits.

***************************************************************************/

#include "emu.h"
#include "pdfprinter.h"

#include "emuopts.h"
#include "fileio.h"
#include "png.h"
#include "video.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>


DEFINE_DEVICE_TYPE(PDF_PRINTER, pdf_printer_device, "pdf_printer", "Printer output (PDF)")


//**************************************************************************
//  DOT STYLES
//**************************************************************************

printer_dot_style printer_dot_style::impact_9pin()
{
	printer_dot_style s;
	s.diameter = 0.0140;        // ~0.30 mm needle plus the ink spreading into the paper
	s.density = 0.84f;
	s.density_jitter = 0.10f;
	s.softness = 0.9f;
	s.texture = 0.18f;
	return s;
}

printer_dot_style printer_dot_style::impact_24pin()
{
	printer_dot_style s;
	s.diameter = 0.0095;        // ~0.20 mm needle plus the ink spreading into the paper
	s.density = 0.84f;
	s.density_jitter = 0.10f;
	s.softness = 0.8f;
	s.texture = 0.15f;
	return s;
}

printer_dot_style printer_dot_style::inkjet(double diameter)
{
	printer_dot_style s;
	s.diameter = diameter;
	s.density = 0.92f;
	s.density_jitter = 0.03f;
	s.softness = 0.7f;
	s.texture = 0.04f;
	s.position_jitter = diameter * 0.08;
	s.spread = 0.35f;
	return s;
}

printer_dot_style printer_dot_style::pixel(double hdpi, double vdpi)
{
	printer_dot_style s;
	s.diameter = 1.15 / std::min(hdpi, vdpi);
	s.density = 0.85f;
	s.density_jitter = 0.05f;
	s.softness = 0.8f;
	s.texture = 0.06f;
	return s;
}


//**************************************************************************
//  PAGE CANVAS
//**************************************************************************

void printer_page_canvas::reset(double width_in, double height_in, int dpi, bool color)
{
	m_dpi = dpi;
	m_width_in = width_in;
	m_height_in = height_in;
	m_width = std::max(1, int(std::lround(width_in * dpi)));
	m_height = std::max(1, int(std::lround(height_in * dpi)));
	m_channels = color ? 3 : 1;
	m_pixels.assign(size_t(m_width) * m_height * m_channels, 0xff);
	m_dirty = false;
}

u32 printer_page_canvas::hash(u32 a, u32 b)
{
	u32 h = a * 0x9e3779b1u ^ (b + 0x7f4a7c15u) * 0x85ebca77u;
	h ^= h >> 15;
	h *= 0x2c1b3c6du;
	h ^= h >> 12;
	h *= 0x297a2d39u;
	h ^= h >> 15;
	return h;
}

void printer_page_canvas::dot(double x, double y, const printer_dot_style &style)
{
	if (m_pixels.empty())
		return;

	double cx = x * m_dpi;
	double cy = y * m_dpi;
	if (style.position_jitter > 0.0)
	{
		cx += (rnd() - 0.5f) * 2.0 * style.position_jitter * m_dpi;
		cy += (rnd() - 0.5f) * 2.0 * style.position_jitter * m_dpi;
	}

	double const r = std::max(0.5, style.diameter * m_dpi * 0.5);
	float const density = std::clamp(style.density * (1.0f + style.density_jitter * (rnd() * 2.0f - 1.0f)), 0.0f, 1.0f);
	float const soft = std::max(0.05f, style.softness);

	int const x0 = std::max(0, int(std::floor(cx - r - 1.5)));
	int const x1 = std::min(m_width - 1, int(std::ceil(cx + r + 1.5)));
	int const y0 = std::max(0, int(std::floor(cy - r - 1.5)));
	int const y1 = std::min(m_height - 1, int(std::ceil(cy + r + 1.5)));
	if ((x0 > x1) || (y0 > y1))
		return;

	u8 const ink[3] = { u8(style.color >> 16), u8(style.color >> 8), u8(style.color) };
	u8 const ink_lum = u8((ink[0] * 77 + ink[1] * 150 + ink[2] * 29) >> 8);

	for (int py = y0; py <= y1; py++)
	{
		u8 *const line = row(py);
		double const dy = py + 0.5 - cy;
		for (int px = x0; px <= x1; px++)
		{
			double const dx = px + 0.5 - cx;
			float edge = float(r - std::sqrt(dx * dx + dy * dy));
			if (style.spread > 0.0f)
				edge += style.spread * float(r) * 0.5f * (float(hash(px, py) & 0xffff) / 65535.0f - 0.5f);
			float a = std::clamp(edge / soft + 0.5f, 0.0f, 1.0f);
			if (a <= 0.0f)
				continue;
			if (style.texture > 0.0f)
			{
				// coarse weave of the ribbon / fibres of the paper
				float const t = float(hash(px >> 1, py >> 1) & 0xffff) / 65535.0f;
				a *= 1.0f - style.texture * t;
			}
			float const k = a * density;
			if (m_channels == 1)
			{
				u8 &p = line[px];
				p = u8(p - (p - ink_lum) * k);
			}
			else
			{
				u8 *const p = &line[px * 3];
				for (int c = 0; c < 3; c++)
					p[c] = u8(p[c] - (p[c] - ink[c]) * k);
			}
		}
	}
	m_dirty = true;
}

void printer_page_canvas::take(printer_page_canvas &dest)
{
	dest.m_dpi = m_dpi;
	dest.m_width = m_width;
	dest.m_height = m_height;
	dest.m_width_in = m_width_in;
	dest.m_height_in = m_height_in;
	dest.m_channels = m_channels;
	dest.m_dirty = m_dirty;
	dest.m_style = m_style;
	dest.m_pixels = std::move(m_pixels);
	m_pixels.clear();
	m_dirty = false;
}


//**************************************************************************
//  CONFIGURATION
//**************************************************************************

// PDF numbers always use a decimal point; printf-style %f follows the
// C locale, which may use a decimal comma (e.g. German Windows)
static std::string pdf_real(double v)
{
	long long const m = std::llround(v * 1000.0);
	unsigned long long const a = (m < 0) ? -m : m;
	return util::string_format("%s%u.%03u", (m < 0) ? "-" : "", unsigned(a / 1000), unsigned(a % 1000));
}

static INPUT_PORTS_START( pdf_printer )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x03, 0x00, "PDF image resolution" )
	PORT_CONFSETTING(    0x00, "Printer default" )
	PORT_CONFSETTING(    0x01, "300 dpi" )
	PORT_CONFSETTING(    0x02, "600 dpi" )
	PORT_CONFSETTING(    0x03, "720 dpi" )
INPUT_PORTS_END

ioport_constructor pdf_printer_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( pdf_printer );
}


//**************************************************************************
//  PAGE JOBS
//**************************************************************************

struct pdf_printer_device::page_job
{
	printer_page_canvas page;
	std::vector<u8> image;      // deflated, PNG "Up" predictor
	std::future<void> done;
};


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

pdf_printer_device::pdf_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, PDF_PRINTER, tag, owner, clock),
	device_image_interface(mconfig, *this),
	m_online_cb(*this),
	m_config(*this, "CONFIG"),
	m_document_open(false),
	m_file_pos(0)
{
}

pdf_printer_device::~pdf_printer_device()
{
	// never leave a worker running on a page that goes away
	for (auto &job : m_jobs)
		if (job->done.valid())
			job->done.wait();
}

void pdf_printer_device::device_start()
{
}

void pdf_printer_device::device_stop()
{
	if (m_document_open)
		finish_document();
}

int pdf_printer_device::output_dpi(int native_dpi) const
{
	switch (m_config->read() & 3)
	{
	case 1: return 300;
	case 2: return 600;
	case 3: return 720;
	default:
		if (native_dpi <= 0)
			return 300;
		// smallest integer multiple of the native resolution that reaches 300 dpi
		return native_dpi * std::max(1, (300 + native_dpi - 1) / native_dpi);
	}
}


//-------------------------------------------------
//  image interface
//-------------------------------------------------

std::pair<std::error_condition, std::string> pdf_printer_device::call_create(int format_type, util::option_resolution *format_options)
{
	return call_load();
}

std::pair<std::error_condition, std::string> pdf_printer_device::call_load()
{
	// an existing file is replaced, not overwritten in place
	fseek(0, SEEK_SET);
	image_core_file().truncate(0);
	begin_document();
	m_online_cb(true);
	return std::make_pair(std::error_condition(), std::string());
}

void pdf_printer_device::call_unload()
{
	if (m_document_open)
	{
		if (m_flush_handler)
			m_flush_handler();
		finish_document();
	}
	m_online_cb(false);
}


//-------------------------------------------------
//  filing pages
//-------------------------------------------------

void pdf_printer_device::eject_page(printer_page_canvas &page)
{
	if (!page.dirty())
		return;

	if (!exists() || !m_document_open)
	{
		write_png(page);
		page.reset(page.width_in(), page.height_in(), page.dpi(), page.is_color());
		return;
	}

	auto job = std::make_unique<page_job>();
	double const w = page.width_in(), h = page.height_in();
	int const dpi = page.dpi();
	bool const color = page.is_color();
	page.take(job->page);
	page.reset(w, h, dpi, color);

	page_job *const raw = job.get();
	job->done = std::async(std::launch::async, [raw] () { compress_page(*raw); });
	m_jobs.emplace_back(std::move(job));

	machine().popmessage("%s: page %d", basename() ? basename() : "printout", int(m_page_objects.size() + m_jobs.size()));
	flush_jobs(false);
}

void pdf_printer_device::print_bitmap(const bitmap_rgb32 &bitmap, int hdpi, int vdpi, const printer_dot_style *style, u32 paper_color)
{
	if ((hdpi <= 0) || (vdpi <= 0) || (bitmap.width() <= 0) || (bitmap.height() <= 0))
		return;
	paper_color &= 0xffffff;

	// colored ink anywhere?
	bool color = false;
	for (int y = 0; (y < bitmap.height()) && !color; y++)
	{
		u32 const *const line = &bitmap.pix(y, 0);
		for (int x = 0; x < bitmap.width(); x++)
		{
			u32 const p = line[x] & 0xffffff;
			if ((p != paper_color) && ((((p >> 16) & 0xff) != ((p >> 8) & 0xff)) || (((p >> 8) & 0xff) != (p & 0xff))))
			{
				color = true;
				break;
			}
		}
	}

	printer_dot_style s = style ? *style : printer_dot_style::pixel(hdpi, vdpi);
	printer_page_canvas page;
	page.reset(double(bitmap.width()) / hdpi, double(bitmap.height()) / vdpi, output_dpi(std::max(hdpi, vdpi)), color);
	for (int y = 0; y < bitmap.height(); y++)
	{
		u32 const *const line = &bitmap.pix(y, 0);
		for (int x = 0; x < bitmap.width(); x++)
		{
			u32 const p = line[x] & 0xffffff;
			if (p != paper_color)
			{
				s.color = p;
				page.dot((x + 0.5) / hdpi, (y + 0.5) / vdpi, s);
			}
		}
	}
	eject_page(page);
}


//-------------------------------------------------
//  worker side
//-------------------------------------------------

void pdf_printer_device::compress_page(page_job &job)
{
	printer_page_canvas const &page = job.page;
	int const h = page.height();
	size_t const rowbytes = size_t(page.width()) * page.channels();

	// PNG "Up" predictor rows: filter byte 2 followed by the difference to the row above
	std::vector<u8> raw(size_t(h) * (rowbytes + 1));
	for (int y = 0; y < h; y++)
	{
		u8 *const dst = &raw[size_t(y) * (rowbytes + 1)];
		u8 const *const cur = page.row(y);
		dst[0] = 2;
		if (y == 0)
			std::copy_n(cur, rowbytes, dst + 1);
		else
		{
			u8 const *const prev = page.row(y - 1);
			for (size_t i = 0; i < rowbytes; i++)
				dst[i + 1] = u8(cur[i] - prev[i]);
		}
	}
	uLongf len = compressBound(uLong(raw.size()));
	job.image.resize(len);
	if (compress2(job.image.data(), &len, raw.data(), uLong(raw.size()), 6) != Z_OK)
		len = 0;
	job.image.resize(len);
}


//-------------------------------------------------
//  PDF writer
//-------------------------------------------------

void pdf_printer_device::write_raw(std::string_view data)
{
	fwrite(data.data(), data.size());
	m_file_pos += data.size();
}

u32 pdf_printer_device::write_object(std::string_view body)
{
	m_offsets.push_back(m_file_pos);
	u32 const id = u32(m_offsets.size());
	write_raw(util::string_format("%u 0 obj\n", id));
	write_raw(body);
	write_raw("\nendobj\n");
	return id;
}

u32 pdf_printer_device::write_stream_object(std::string_view dict, const std::vector<u8> &data)
{
	m_offsets.push_back(m_file_pos);
	u32 const id = u32(m_offsets.size());
	write_raw(util::string_format("%u 0 obj\n<< %s /Length %u >>\nstream\n", id, dict, unsigned(data.size())));
	fwrite(data.data(), data.size());
	m_file_pos += data.size();
	write_raw("\nendstream\nendobj\n");
	return id;
}

void pdf_printer_device::begin_document()
{
	m_jobs.clear();
	m_offsets.clear();
	m_page_objects.clear();
	m_file_pos = 0;
	m_document_open = true;

	write_raw("%PDF-1.4\n%\xe2\xe3\xcf\xd3\n");

	// objects 1 (catalog) and 2 (page tree) are written at the end
	m_offsets.push_back(0);
	m_offsets.push_back(0);
}

void pdf_printer_device::flush_jobs(bool wait)
{
	while (!m_jobs.empty())
	{
		page_job &job = *m_jobs.front();
		if (!wait && (job.done.wait_for(std::chrono::seconds(0)) != std::future_status::ready))
			break;
		job.done.wait();
		write_job(job);
		m_jobs.pop_front();
	}
}

void pdf_printer_device::write_job(page_job &job)
{
	printer_page_canvas const &page = job.page;

	char dict[320];
	std::snprintf(dict, sizeof(dict),
			"/Type /XObject /Subtype /Image /Width %d /Height %d /ColorSpace /%s /BitsPerComponent 8 "
			"/Filter /FlateDecode /DecodeParms << /Predictor 15 /Colors %d /BitsPerComponent 8 /Columns %d >>",
			page.width(), page.height(), page.is_color() ? "DeviceRGB" : "DeviceGray", page.channels(), page.width());
	u32 const image = write_stream_object(dict, job.image);

	std::string const pw = pdf_real(page.width_in() * 72.0), ph = pdf_real(page.height_in() * 72.0);
	std::string const content = util::string_format("q %s 0 0 %s 0 0 cm /Im0 Do Q\n", pw, ph);
	u32 const contents = write_stream_object("", std::vector<u8>(content.begin(), content.end()));

	m_page_objects.push_back(write_object(util::string_format(
			"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %s %s] /Resources << /XObject << /Im0 %u 0 R >> >> /Contents %u 0 R >>",
			pw, ph, image, contents)));
}

void pdf_printer_device::finish_document()
{
	flush_jobs(true);

	std::string kids;
	for (u32 id : m_page_objects)
		kids += util::string_format("%u 0 R ", id);

	m_offsets[1] = m_file_pos;
	write_raw(util::string_format("2 0 obj\n<< /Type /Pages /Kids [ %s] /Count %u >>\nendobj\n", kids, unsigned(m_page_objects.size())));
	m_offsets[0] = m_file_pos;
	write_raw("1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n");

	u32 const info = write_object("<< /Producer (MAME) >>");

	u64 const xref = m_file_pos;
	std::string x = util::string_format("xref\n0 %u\n0000000000 65535 f \n", unsigned(m_offsets.size() + 1));
	for (u64 off : m_offsets)
		x += util::string_format("%010u 00000 n \n", unsigned(off));
	write_raw(x);
	write_raw(util::string_format("trailer\n<< /Size %u /Root 1 0 R /Info %u 0 R >>\nstartxref\n%u\n%%%%EOF\n",
			unsigned(m_offsets.size() + 1), info, unsigned(xref)));

	m_document_open = false;
	if (!m_page_objects.empty())
		osd_printf_info("%s: %d page(s) written to %s\n", tag(), int(m_page_objects.size()), filename() ? filename() : "PDF");
}

void pdf_printer_device::write_png(const printer_page_canvas &page)
{
	emu_file file(machine().options().snapshot_directory(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
	std::error_condition const filerr = machine().video().open_next(file, "png");
	if (filerr)
		return;

	bitmap_rgb32 bitmap(page.width(), page.height());
	for (int y = 0; y < page.height(); y++)
	{
		u8 const *const src = page.row(y);
		u32 *const dst = &bitmap.pix(y, 0);
		for (int x = 0; x < page.width(); x++)
		{
			if (page.channels() == 1)
				dst[x] = rgb_t(src[x], src[x], src[x]);
			else
				dst[x] = rgb_t(src[x * 3], src[x * 3 + 1], src[x * 3 + 2]);
		}
	}
	util::png_write_bitmap(file, nullptr, bitmap, 0, nullptr);
	machine().popmessage("printer page saved as %s", file.filename());
}
