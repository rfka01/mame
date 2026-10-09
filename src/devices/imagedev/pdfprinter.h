// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    pdfprinter.h

    Printer output to PDF documents

    pdf_printer_device is an image device ("printout", *.pdf) that any
    emulated printer can use as its paper output, regardless of how the
    printer is attached (Centronics, RS-232, IEEE-488, IEC, ...).

    A printer either draws into a printer_page_canvas, using dot
    primitives that simulate the marks of the print mechanism (impact
    needles hitting a ribbon, ink drops of a nozzle), or hands over a
    finished bitmap (print_bitmap(), used by bitmap_printer).

    Each finished page becomes one PDF page holding a greyscale (or RGB)
    image.  If no output file is mounted, finished pages are saved as PNG
    files to the snapshot directory instead.

    A searchable text layer would be possible in principle, either from
    character codes intercepted on the way to the printer or by running
    an OCR engine such as Tesseract over the page image.

***************************************************************************/

#ifndef MAME_IMAGEDEV_PDFPRINTER_H
#define MAME_IMAGEDEV_PDFPRINTER_H

#pragma once

#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <vector>


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> printer_dot_style

// what a single mark of the print mechanism looks like
struct printer_dot_style
{
	double diameter = 0.0110;       // in inches (0.28 mm, typical 9-pin needle)
	float density = 0.80f;          // 0..1, darkness of a single dot
	float density_jitter = 0.08f;   // random dot-to-dot variation of the density
	float softness = 0.45f;         // width of the soft edge, in pixels
	float texture = 0.10f;          // ribbon fabric / paper fibre texture strength
	double position_jitter = 0.0;   // random displacement, in inches (ink drops)
	float spread = 0.0f;            // irregular outline (ink feathering), 0..1
	u32 color = 0x000000;           // ink color (RGB)

	static printer_dot_style impact_9pin();
	static printer_dot_style impact_24pin();
	static printer_dot_style inkjet(double diameter);
	static printer_dot_style pixel(double hdpi, double vdpi);
};


// ======================> printer_page_canvas

// the sheet of paper a printer draws on
class printer_page_canvas
{
public:
	printer_page_canvas() = default;

	void reset(double width_in, double height_in, int dpi, bool color = false);

	// geometry
	int dpi() const { return m_dpi; }
	int width() const { return m_width; }
	int height() const { return m_height; }
	double width_in() const { return m_width_in; }
	double height_in() const { return m_height_in; }
	bool is_color() const { return m_channels == 3; }
	bool dirty() const { return m_dirty; }
	int channels() const { return m_channels; }

	// drawing - coordinates in inches from the top left of the sheet
	void set_dot_style(const printer_dot_style &style) { m_style = style; }
	const printer_dot_style &dot_style() const { return m_style; }
	void dot(double x, double y) { dot(x, y, m_style); }
	void dot(double x, double y, const printer_dot_style &style);

	// raw access
	u8 *row(int y) { return &m_pixels[size_t(y) * m_width * m_channels]; }
	const u8 *row(int y) const { return &m_pixels[size_t(y) * m_width * m_channels]; }

	// move the content out (the canvas is left empty)
	void take(printer_page_canvas &dest);

private:
	static u32 hash(u32 a, u32 b);
	float rnd() { m_seed = m_seed * 1664525u + 1013904223u; return float(m_seed >> 8) * (1.0f / 16777216.0f); }

	int m_dpi = 0;
	int m_width = 0, m_height = 0;
	double m_width_in = 0, m_height_in = 0;
	int m_channels = 1;
	bool m_dirty = false;
	u32 m_seed = 0x12345678;
	printer_dot_style m_style;
	std::vector<u8> m_pixels;
};


// ======================> pdf_printer_device

class pdf_printer_device : public device_t,
	public device_image_interface
{
public:
	// construction/destruction
	pdf_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	virtual ~pdf_printer_device();

	auto online_callback() { return m_online_cb.bind(); }

	// called before the document is closed, so the printer can file the
	// page it is working on (tearing off the paper)
	void set_flush_handler(std::function<void ()> &&handler) { m_flush_handler = std::move(handler); }

	// device_image_interface implementation
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual std::pair<std::error_condition, std::string> call_create(int format_type, util::option_resolution *format_options) override;
	virtual void call_unload() override;

	virtual bool is_readable()  const noexcept override { return false; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return true; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool support_command_line_image_creation() const noexcept override { return true; }
	virtual const char *file_extensions() const noexcept override { return "pdf"; }
	virtual const char *image_type_name() const noexcept override { return "printout"; }
	virtual const char *image_brief_type_name() const noexcept override { return "prin"; }

	// file a finished page; the canvas is emptied
	void eject_page(printer_page_canvas &page);

	// file a page given as bitmap, e.g. from bitmap_printer; every non-paper
	// pixel becomes one dot of the given style (pixel style if null)
	void print_bitmap(const bitmap_rgb32 &bitmap, int hdpi, int vdpi, const printer_dot_style *style = nullptr, u32 paper_color = 0xffffff);

	// resolution of the PDF page images for a printer of the given native resolution
	int output_dpi(int native_dpi) const;

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	struct page_job;

	void begin_document();
	void finish_document();
	void flush_jobs(bool wait);
	void write_job(page_job &job);
	void write_png(const printer_page_canvas &page);
	u32 write_object(std::string_view body);
	u32 write_stream_object(std::string_view dict, const std::vector<u8> &data);
	void write_raw(std::string_view data);

	static void compress_page(page_job &job);

	devcb_write_line m_online_cb;
	required_ioport m_config;
	std::function<void ()> m_flush_handler;

	bool m_document_open;
	u64 m_file_pos;
	std::vector<u64> m_offsets;         // byte offset of each object, index = object number - 1
	std::vector<u32> m_page_objects;
	std::deque<std::unique_ptr<page_job>> m_jobs;
};


// device type definition
DECLARE_DEVICE_TYPE(PDF_PRINTER, pdf_printer_device)

#endif // MAME_IMAGEDEV_PDFPRINTER_H
