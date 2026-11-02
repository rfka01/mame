// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP LaserJet 4L (C2003A), 1993

    Formatter C2004-80101:
    - Motorola 1TY4 SC414210FT: 68000 core and the printer logic in one chip
    - 2 MB mask ROM 1818-5601 (16 bit)
    - 1 MB DRAM (2x HM514260), the firmware detects 1 MB more on a memory card
    - 31.3076 MHz crystal
    - Bi-Tronics parallel port, one button, four LEDs, Canon print engine

    The formatter logic in the 1TY4 works like the separate gate arrays of the
    LaserJet IIIP (hp/hplj3p.cpp): the same merger, the same serial link to the
    engine's DC controller and the same band output.  The register meanings
    come from the firmware; timings of the engine are not measured.

    Working: self test page (hold the button for two seconds), PCL jobs from a
    file through the bitbanger ("-bitb job.pcl"), pages go to the PDF printer.
    The reverse channel of the parallel port is sent to the bitbanger, but
    IEEE 1284 negotiation is not emulated.

    Front panel: LEDs Ready, Data, Paper and Error (outputs ready_led,
    data_led, paper_led, error_led), one button.

    TODO:
    - status meanings of the 4L engine commands 1c, 3e, 40, 4f, 51, 9d, f1
      (the firmware reads two status bytes for 9d, a7 and f1)
    - IEEE 1284 negotiation, manual feed
    - the 870000 registers (written before every page, probably the laser
      timing and Resolution Enhancement)

***************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "imagedev/bitbngr.h"
#include "imagedev/pdfprinter.h"
#include "machine/ram.h"

#include <bit>

#define VERBOSE 0
#include "logmacro.h"


namespace {

class hplj4l_state : public driver_device
{
public:
	hplj4l_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_ram(*this, RAM_TAG)
		, m_config(*this, "CONFIG")
		, m_ready_led(*this, "ready_led")
		, m_data_led(*this, "data_led")
		, m_paper_led(*this, "paper_led")
		, m_error_led(*this, "error_led")
		, m_button(*this, "BUTTON")
		, m_host(*this, "host")
		, m_printout(*this, "printout")
	{ }

	void hplj4l(machine_config &config) ATTR_COLD;

private:
	required_device<m68000_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_ioport m_config;
	output_finder<> m_ready_led;
	output_finder<> m_data_led;
	output_finder<> m_paper_led;
	output_finder<> m_error_led;

	virtual void machine_start() override ATTR_COLD;

	// merger (glyph and rule blitter), the same unit as in the LaserJet IIIP:
	// 900000 long shading pattern pointer, 900004 long destination,
	// 900008 x position (bits 0-3 shift, bits 4-15 word offset), 90000a mask,
	// 90000c low byte raster operation, 900010 data word inside a row,
	// 900012 last data word of a row, 900016 low byte line stride in words
	u32 m_blt_pat;
	u32 m_blt_dest;
	u32 m_blt_row;
	u32 m_blt_ptr;
	u16 m_blt_ctrl;
	u16 m_blt_mask;
	u16 m_blt_mode;
	u16 m_blt_spill;
	u16 m_blt_cfg;
	int m_blt_patrow;
	void blt_pat_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_dest_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_ctrl_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_data_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_cfg_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_out(u16 data);

	// interrupt controller in the 1TY4: 800000 low byte disables sources
	// (bit set: masked), 800004 written acknowledges them; source 2 is the
	// system tick on level 4
	u8 m_irq_mask;
	u8 m_irq_pend;
	void update_irq();
	required_ioport m_button;
	u16 button_r(offs_t offset, u16 mem_mask);
public:
	DECLARE_INPUT_CHANGED_MEMBER(button_changed);
private:
	TIMER_CALLBACK_MEMBER(tick);
	emu_timer *m_tick_timer;
	u16 irqmask_r(offs_t offset, u16 mem_mask);
	void irqmask_w(offs_t offset, u16 data, u16 mem_mask);
	void irqack_w(offs_t offset, u16 data, u16 mem_mask);

	// print engine interface on 84001e, the same serial link as in the IIIP:
	// written bit 8 SCLK, bit 9 command data (active low), bit 12 /CBSY,
	// bits 10, 11, 13 further engine outputs; read bit 8 /STS, bit 11 /SBSY,
	// bit 12 /RDY, bit 13 /PPRDY
	u16 m_eng_ctrl;
	u8 m_eng_cmd;
	u8 m_eng_sts;
	int m_eng_cbits;
	int m_eng_bits;
	bool m_eng_sbsy;
	bool m_eng_out;
	emu_timer *m_eng_timer;
	TIMER_CALLBACK_MEMBER(eng_reply);
	TIMER_CALLBACK_MEMBER(eng_top);
	emu_timer *m_top_timer;
	u8 eng_status(u8 cmd);
	u16 eng_r(offs_t offset, u16 mem_mask);
	void eng_w(offs_t offset, u16 data, u16 mem_mask);

	// 850000: LEDs, active high: bit 0 Ready, bit 1 Data, bit 3 Paper, bit 4 Error
	u16 m_leds;
	u16 leds_r(offs_t offset, u16 mem_mask) { return m_leds; }
	void leds_w(offs_t offset, u16 data, u16 mem_mask);

	// Bi-Tronics parallel port: 820000 data (read: received byte, written:
	// byte for the reverse channel), 820002 status (bit 0 data received,
	// bit 5 reverse channel ready), 82000a interrupt enables (bit 0 data
	// received, bit 3 reverse channel ready)
	required_device<bitbanger_device> m_host;
	u8 m_pdata;
	bool m_pfull;
	u16 m_pen;
	emu_timer *m_host_timer;
	TIMER_CALLBACK_MEMBER(host_poll);
	void update_pirq();
	u16 pdata_r(offs_t offset, u16 mem_mask);
	void pdata_w(offs_t offset, u16 data, u16 mem_mask);
	u16 pstat_r(offs_t offset, u16 mem_mask);
	u16 pen_r(offs_t offset, u16 mem_mask) { return m_pen; }
	void pen_w(offs_t offset, u16 data, u16 mem_mask);

	// video output (laser data), as in the IIIP: 840000 high byte words per
	// line, 840000 bits 0-6 and 840002 band address, 840004 left margin,
	// 840006 bits 0-11 line count, bit 12 stop, bit 14 data (else blank lines)
	required_device<pdf_printer_device> m_printout;
	// the band registers are double buffered: a band written while another
	// one is output waits in the second set and is taken over when the
	// current band ends; the end of every band requests interrupt source 0
	// (level 6)
	// Left margin offset: 840004 holds the start of the image in pixels after
	// the beam detect signal of each laser line, not after the paper edge.
	// The firmware writes 0x87 for the cassette (plus the registration
	// adjustment).  With the paper edge 85 pixels after the signal a Letter
	// image (154 words, 2464 pixels) starts 50 pixels in: the PCL logical
	// page then starts 75 pixels (1/4 inch) from the edge on Letter and 71
	// pixels (0.24 inch) on A4 as specified, and the frame of the self test
	// page sits 1/4 inch from both edges.  Taken from printed rules, not
	// measured on a printer; without it the image is 135 pixels to the right
	// and the right part of the frame falls off the sheet.
	static constexpr int MARGIN_OFFSET = 85;
	struct band_t
	{
		u32 addr;
		u16 words;
		s16 x0;
		u16 lines;
		bool data;
	};
	u16 m_video[4];
	band_t m_cur;
	band_t m_next;
	bool m_cur_active;
	bool m_next_valid;
	bool m_stop_pending;
	band_t latch_band() const;
	bitmap_rgb32 m_page;
	int m_page_y;
	bool m_page_dirty;
	emu_timer *m_line_timer;
	TIMER_CALLBACK_MEMBER(video_line);
	void video_w(offs_t offset, u16 data, u16 mem_mask);
	void eject_page();

	void main_map(address_map &map) ATTR_COLD;
	u16 unk_r(offs_t offset, u16 mem_mask);
	void unk_w(offs_t offset, u16 data, u16 mem_mask);
};

u16 hplj4l_state::unk_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
		LOG("%s: read %06x & %04x\n", machine().describe_context(), offset << 1, mem_mask);
	return 0;
}

void hplj4l_state::unk_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("%s: write %06x = %04x & %04x\n", machine().describe_context(), offset << 1, data, mem_mask);
}

void hplj4l_state::machine_start()
{
	m_blt_pat = m_blt_dest = m_blt_row = m_blt_ptr = 0;
	m_blt_ctrl = m_blt_mask = m_blt_mode = m_blt_spill = m_blt_cfg = 0;
	m_blt_patrow = 0;
	save_item(NAME(m_blt_pat));
	save_item(NAME(m_blt_dest));
	save_item(NAME(m_blt_row));
	save_item(NAME(m_blt_ptr));
	save_item(NAME(m_blt_ctrl));
	save_item(NAME(m_blt_mask));
	save_item(NAME(m_blt_mode));
	save_item(NAME(m_blt_spill));
	save_item(NAME(m_blt_cfg));
	save_item(NAME(m_blt_patrow));

	m_irq_mask = 0x3f;
	m_irq_pend = 0;
	save_item(NAME(m_irq_mask));
	save_item(NAME(m_irq_pend));
	// the firmware converts times at 370 ticks per second (routines at 1178/119a)
	m_tick_timer = timer_alloc(FUNC(hplj4l_state::tick), this);
	m_tick_timer->adjust(attotime::from_hz(370), 0, attotime::from_hz(370));

	m_eng_ctrl = 0xffff;
	m_eng_cmd = m_eng_sts = 0;
	m_eng_cbits = m_eng_bits = 0;
	m_eng_sbsy = m_eng_out = false;
	m_eng_timer = timer_alloc(FUNC(hplj4l_state::eng_reply), this);
	m_top_timer = timer_alloc(FUNC(hplj4l_state::eng_top), this);
	save_item(NAME(m_eng_ctrl));
	save_item(NAME(m_eng_cmd));
	save_item(NAME(m_eng_sts));
	save_item(NAME(m_eng_cbits));
	save_item(NAME(m_eng_bits));
	save_item(NAME(m_eng_sbsy));
	save_item(NAME(m_eng_out));

	m_leds = 0x1f;
	save_item(NAME(m_leds));

	// 1 MB on the formatter at 600000, the memory card follows at 700000
	m_maincpu->space(AS_PROGRAM).install_ram(0x600000, 0x600000 + m_ram->size() - 1, m_ram->pointer());

	m_pdata = 0;
	m_pfull = false;
	m_pen = 0;
	save_item(NAME(m_pdata));
	save_item(NAME(m_pfull));
	save_item(NAME(m_pen));
	std::fill(std::begin(m_video), std::end(m_video), 0);
	m_cur = m_next = band_t{ 0, 0, 0, 0, false };
	m_cur_active = m_next_valid = m_stop_pending = false;
	m_page.allocate(2550, 3508);   // 8.5 x 11.69 inches: Letter width, A4 length
	m_page.fill(0xffffff);
	m_page_y = 0;
	m_page_dirty = false;
	m_line_timer = timer_alloc(FUNC(hplj4l_state::video_line), this);
	save_item(NAME(m_video));
	save_item(NAME(m_cur.addr));
	save_item(NAME(m_cur.words));
	save_item(NAME(m_cur.x0));
	save_item(NAME(m_cur.lines));
	save_item(NAME(m_cur.data));
	save_item(NAME(m_next.addr));
	save_item(NAME(m_next.words));
	save_item(NAME(m_next.x0));
	save_item(NAME(m_next.lines));
	save_item(NAME(m_next.data));
	save_item(NAME(m_cur_active));
	save_item(NAME(m_next_valid));
	save_item(NAME(m_stop_pending));
	save_item(NAME(m_page));
	save_item(NAME(m_page_y));
	save_item(NAME(m_page_dirty));

	m_host_timer = timer_alloc(FUNC(hplj4l_state::host_poll), this);
	m_host_timer->adjust(attotime::from_usec(20), 0, attotime::from_usec(20));
}

hplj4l_state::band_t hplj4l_state::latch_band() const
{
	band_t b;
	b.addr = (u32(m_video[0] & 0x7f) << 16) | (m_video[1] & 0xfffe);
	b.words = m_video[0] >> 8;
	// margin from the beam detect signal, converted to the paper edge
	b.x0 = (m_video[2] & 0x0fff) - MARGIN_OFFSET;
	b.lines = m_video[3] & 0x0fff;
	b.data = BIT(m_video[3], 14);
	return b;
}

void hplj4l_state::video_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_video[offset]);
	LOG("%s: video %06x = %04x at %.3f\n", machine().describe_context(), 0x840000 + offset * 2, m_video[offset], machine().time().as_double());
	if (offset != 3)
		return;
	if (BIT(m_video[3], 12))
	{
		// stop: the page ends after the bands already queued
		if (!m_cur_active && !m_next_valid)
			eject_page();
		else
			m_stop_pending = true;
	}
	else if (m_video[3] & 0x0fff)
	{
		m_stop_pending = false;
		if (!m_cur_active)
		{
			m_cur = latch_band();
			m_cur_active = true;
			m_line_timer->adjust(attotime::from_usec(1690));   // line period (not measured)
		}
		else
		{
			m_next = latch_band();
			m_next_valid = true;
		}
	}
}

TIMER_CALLBACK_MEMBER(hplj4l_state::video_line)
{
	if (!m_cur_active)
		return;
	if (m_cur.data && m_page_y < m_page.height())
	{
		address_space &space = m_maincpu->space(AS_PROGRAM);
		u32 *const line = &m_page.pix(m_page_y);
		for (int w = 0; w < m_cur.words; w++)
		{
			u16 const d = space.read_word(m_cur.addr + w * 2);
			space.write_word(m_cur.addr + w * 2, 0);   // cleared as it is read out, as in the IIIP
			for (int b = 0; b < 16; b++)
			{
				int const x = m_cur.x0 + w * 16 + b;
				if (BIT(d, 15 - b) && x >= 0 && x < m_page.width())
				{
					line[x] = 0x000000;
					m_page_dirty = true;
				}
			}
		}
		m_cur.addr += m_cur.words * 2;
	}
	m_page_y++;
	if (--m_cur.lines)
	{
		m_line_timer->adjust(attotime::from_usec(1690));
		return;
	}
	LOG("band done, page line %d at %.3f\n", m_page_y, machine().time().as_double());
	m_irq_pend |= 0x01;
	update_irq();
	if (m_next_valid)
	{
		m_cur = m_next;
		m_next_valid = false;
		m_line_timer->adjust(attotime::from_usec(1690));
	}
	else
	{
		m_cur_active = false;
		if (m_stop_pending)
		{
			m_stop_pending = false;
			eject_page();
		}
	}
}

void hplj4l_state::eject_page()
{
	if (m_page_dirty || m_page_y)
	{
		LOG("page complete, %d lines\n", m_page_y);
		m_printout->print_bitmap(m_page, 300, 300);
	}
	m_page.fill(0xffffff);
	m_page_y = 0;
	m_page_dirty = false;
}

void hplj4l_state::update_pirq()
{
	bool const irq = (m_pfull && BIT(m_pen, 0)) || BIT(m_pen, 3);
	if (irq)
		m_irq_pend |= 0x10;
	else
		m_irq_pend &= ~0x10;
	update_irq();
}

TIMER_CALLBACK_MEMBER(hplj4l_state::host_poll)
{
	// data is only offered while the formatter accepts it (receive interrupt enabled)
	if (m_pfull || !BIT(m_pen, 0))
		return;
	u8 b;
	if (m_host->input(&b, 1) == 1)
	{
		m_pdata = b;
		m_pfull = true;
		update_pirq();
	}
}

u16 hplj4l_state::pdata_r(offs_t offset, u16 mem_mask)
{
	u16 const data = m_pdata;
	if (!machine().side_effects_disabled() && m_pfull)
	{
		m_pfull = false;
		update_pirq();
	}
	return data;
}

void hplj4l_state::pdata_w(offs_t offset, u16 data, u16 mem_mask)
{
	u8 const b = data & 0xff;
	m_host->output(b);
	LOG("reverse channel %02x '%c'\n", b, (b >= 0x20 && b < 0x7f) ? b : '.');
}

u16 hplj4l_state::pstat_r(offs_t offset, u16 mem_mask)
{
	return (m_pfull ? 0x0001 : 0x0000) | 0x0022;
}

void hplj4l_state::pen_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_pen);
	update_pirq();
}

void hplj4l_state::leds_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_leds);
	m_ready_led = BIT(m_leds, 0);
	m_data_led = BIT(m_leds, 1);
	m_paper_led = BIT(m_leds, 3);
	m_error_led = BIT(m_leds, 4);
}

// Status bytes: odd parity in bit 0.  The firmware evaluates them like the
// IIIP's (bit 7 set with bit 6 clear: command not supported):
//   02  bit 2 top door open or no EP cartridge
//   2a  bit 6 paper in the cassette
u8 hplj4l_state::eng_status(u8 cmd)
{
	u8 const cfg = m_config->read();
	u8 data = 0x00;
	switch (cmd)
	{
	case 0x02:
		if (BIT(cfg, 1))
			data |= 0x04;
		break;
	case 0x2a:
		if (BIT(cfg, 0))
			data |= 0x40;
		break;
	}
	if (!(std::popcount(unsigned(data)) & 1))
		data |= 0x01;
	return data;
}

TIMER_CALLBACK_MEMBER(hplj4l_state::eng_reply)
{
	m_eng_sts = eng_status(m_eng_cmd);
	m_eng_bits = 0;
	m_eng_sbsy = true;
	LOG("engine cmd %02x -> sts %02x\n", m_eng_cmd, m_eng_sts);
}

TIMER_CALLBACK_MEMBER(hplj4l_state::eng_top)
{
	// paper at the top of page: interrupt source 1 (level 5)
	LOG("engine TOP at %.3f\n", machine().time().as_double());
	m_irq_pend |= 0x02;
	update_irq();
}

u16 hplj4l_state::eng_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_eng_ctrl & 0x00ff;
	if (!m_eng_out)
		data |= 0x0100;
	if (!m_eng_sbsy)
		data |= 0x0800;
	// /RDY and /PPRDY held active for an engine that is switched on
	return data;
}

void hplj4l_state::eng_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_eng_ctrl;
	COMBINE_DATA(&m_eng_ctrl);
	if (BIT(old, 12) && !BIT(m_eng_ctrl, 12))
	{
		m_eng_cmd = 0;
		m_eng_cbits = 0;
	}
	if (BIT(m_eng_ctrl, 8) && !BIT(old, 8))
	{
		if (!BIT(m_eng_ctrl, 12))
		{
			// command bit, MSB first
			m_eng_cmd = (m_eng_cmd << 1) | (BIT(m_eng_ctrl, 9) ? 0 : 1);
			m_eng_cbits++;
		}
		else if (m_eng_sbsy)
		{
			m_eng_out = BIT(m_eng_sts, 7 - m_eng_bits);
			if (++m_eng_bits == 8)
				m_eng_sbsy = false;
		}
	}
	if (!BIT(old, 12) && BIT(m_eng_ctrl, 12) && m_eng_cbits == 8)
		m_eng_timer->adjust(attotime::from_usec(200));
	if (BIT(old, 11) && !BIT(m_eng_ctrl, 11))
		m_top_timer->adjust(attotime::from_seconds(2));     // PRNT: initial rotation, then TOP (time not measured)
	if ((m_eng_ctrl ^ old) & 0xec00)
		LOG("%s: engine ctrl %04x\n", machine().describe_context(), m_eng_ctrl);
}

TIMER_CALLBACK_MEMBER(hplj4l_state::tick)
{
	m_irq_pend |= 0x04;
	update_irq();
}

void hplj4l_state::update_irq()
{
	// source n requests level 6 - n (the firmware masks source 2 around the
	// tick counter on level 4, source 3 around the button handler on level 3
	// and source 4 around the parallel port handler on level 2)
	u8 const act = m_irq_pend & ~m_irq_mask;
	for (int n = 0; n < 6; n++)
		m_maincpu->set_input_line(6 - n, BIT(act, n) ? ASSERT_LINE : CLEAR_LINE);
}

INPUT_CHANGED_MEMBER(hplj4l_state::button_changed)
{
	m_irq_pend |= 0x08;
	update_irq();
}

// 850002 read: front panel button, bit 0 pressed, bit 1 released (the
// firmware runs the continuous self test when bit 1 is clear at power on);
// the read acknowledges the button interrupt
u16 hplj4l_state::button_r(offs_t offset, u16 mem_mask)
{
	u16 const data = m_button->read() ? 0x0001 : 0x0002;
	if (!machine().side_effects_disabled() && BIT(m_irq_pend, 3))
	{
		m_irq_pend &= ~0x08;
		update_irq();
	}
	return data;
}

u16 hplj4l_state::irqmask_r(offs_t offset, u16 mem_mask)
{
	return m_irq_mask;
}

void hplj4l_state::irqmask_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (ACCESSING_BITS_0_7)
		m_irq_mask = data & 0xff;
	update_irq();
}

void hplj4l_state::irqack_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_irq_pend &= ~data;
	update_irq();
}

void hplj4l_state::blt_pat_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_pat = (m_blt_pat & 0xffff0000) | data;
	else
		m_blt_pat = (m_blt_pat & 0x0000ffff) | (u32(data) << 16);
	m_blt_patrow = 0;
}

void hplj4l_state::blt_dest_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_dest = (m_blt_dest & 0xffff0000) | (data & mem_mask) | (m_blt_dest & ~mem_mask & 0xffff);
	else
		m_blt_dest = (m_blt_dest & 0x0000ffff) | (u32(data & mem_mask) << 16) | (m_blt_dest & (u32(~mem_mask & 0xffff) << 16));
	m_blt_row = (m_blt_dest & 0x7ffffe) + (m_blt_ctrl >> 4) * 2;
	m_blt_ptr = m_blt_row;
	m_blt_spill = 0;
}

void hplj4l_state::blt_ctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch (offset)
	{
	case 0: COMBINE_DATA(&m_blt_ctrl); break;
	case 1: COMBINE_DATA(&m_blt_mask); break;
	case 2: COMBINE_DATA(&m_blt_mode); break;
	default: LOG("%s: merger 90000e = %04x\n", machine().describe_context(), data); break;
	}
}

void hplj4l_state::blt_cfg_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_blt_cfg;
	COMBINE_DATA(&m_blt_cfg);
	if ((old ^ m_blt_cfg) & 0xff00)
		LOG("%s: merger config %04x\n", machine().describe_context(), m_blt_cfg);
}

void hplj4l_state::blt_out(u16 data)
{
	// destination ffffffff: pipeline priming, nothing is written
	if (m_blt_dest == 0xffffffff || m_blt_ptr < 0x600000 || m_blt_ptr > 0x7ffffe)
		return;
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u16 const d = space.read_word(m_blt_ptr);
	u16 const pat = space.read_word((m_blt_pat & 0xfffffe) + (m_blt_patrow & 15) * 2);
	// raster operation indexed by data, pattern and destination bits (ROP3)
	u8 const rop = m_blt_mode & 0xff;
	u16 r = 0;
	for (int b = 0; b < 16; b++)
		if (BIT(rop, (BIT(data, b) << 2) | (BIT(pat, b) << 1) | BIT(d, b)))
			r |= 1 << b;
	space.write_word(m_blt_ptr, (d & ~m_blt_mask) | (r & m_blt_mask));
	m_blt_ptr += 2;
}

void hplj4l_state::blt_data_w(offs_t offset, u16 data, u16 mem_mask)
{
	int const sh = m_blt_ctrl & 0x0f;
	u16 const out = m_blt_spill | (data >> sh);
	m_blt_spill = sh ? u16(data << (16 - sh)) : 0;
	blt_out(out);
	if (offset)
	{
		// last word of the row: flush and go to the next row
		if (sh)
			blt_out(m_blt_spill);
		m_blt_spill = 0;
		m_blt_row += (m_blt_cfg & 0xff) * 2;
		m_blt_ptr = m_blt_row;
		m_blt_patrow++;
	}
}

void hplj4l_state::main_map(address_map &map)
{
	map(0x000000, 0xffffff).rw(FUNC(hplj4l_state::unk_r), FUNC(hplj4l_state::unk_w));
	map(0x000000, 0x1fffff).rom().region("maincpu", 0);
	map(0x800000, 0x800001).rw(FUNC(hplj4l_state::irqmask_r), FUNC(hplj4l_state::irqmask_w));
	map(0x800004, 0x800005).w(FUNC(hplj4l_state::irqack_w));
	map(0x820000, 0x820001).rw(FUNC(hplj4l_state::pdata_r), FUNC(hplj4l_state::pdata_w));
	map(0x820002, 0x820003).r(FUNC(hplj4l_state::pstat_r));
	map(0x82000a, 0x82000b).rw(FUNC(hplj4l_state::pen_r), FUNC(hplj4l_state::pen_w));
	map(0x840000, 0x840007).w(FUNC(hplj4l_state::video_w));
	map(0x84001e, 0x84001f).rw(FUNC(hplj4l_state::eng_r), FUNC(hplj4l_state::eng_w));
	map(0x850000, 0x850001).rw(FUNC(hplj4l_state::leds_r), FUNC(hplj4l_state::leds_w));
	map(0x850002, 0x850003).r(FUNC(hplj4l_state::button_r));
	map(0x900000, 0x900003).w(FUNC(hplj4l_state::blt_pat_w));
	map(0x900004, 0x900007).w(FUNC(hplj4l_state::blt_dest_w));
	map(0x900008, 0x90000f).w(FUNC(hplj4l_state::blt_ctrl_w));
	map(0x900010, 0x900013).w(FUNC(hplj4l_state::blt_data_w));
	map(0x900016, 0x900017).w(FUNC(hplj4l_state::blt_cfg_w));
}

static INPUT_PORTS_START( hplj4l )
	PORT_START("BUTTON")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("Front Panel Button") PORT_CODE(KEYCODE_ENTER) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(hplj4l_state::button_changed), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper in cassette" )
	PORT_CONFSETTING(    0x00, DEF_STR( No ) )
	PORT_CONFSETTING(    0x01, DEF_STR( Yes ) )
	PORT_CONFNAME( 0x02, 0x00, "Top door" )
	PORT_CONFSETTING(    0x00, "Closed" )
	PORT_CONFSETTING(    0x02, "Open" )
INPUT_PORTS_END

void hplj4l_state::hplj4l(machine_config &config)
{
	M68000(config, m_maincpu, 31.3076_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &hplj4l_state::main_map);

	RAM(config, m_ram).set_default_size("1M").set_extra_options("2M");

	BITBANGER(config, m_host, 0);   // host on the parallel port
	PDF_PRINTER(config, m_printout);
}

ROM_START( hplj4l )
	ROM_REGION16_BE( 0x200000, "maincpu", 0 )
	ROM_LOAD16_WORD_SWAP( "1818-5601.bin", 0x000000, 0x200000, CRC(75773ecb) SHA1(f7a14f591976a4bf74d9847f2bee7f86c119f3a2) )
ROM_END

} // anonymous namespace

//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY            FULLNAME         FLAGS
COMP( 1993, hplj4l, 0,      0,      hplj4l,  hplj4l, hplj4l_state, empty_init, "Hewlett-Packard", "LaserJet 4L",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
