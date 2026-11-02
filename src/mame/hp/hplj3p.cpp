// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP LaserJet IIIP (33481A), 1991

    Formatter board RH6-3107 / 33481-80003:
    - Toshiba TMP68HC000T-16, 16 MHz (32 MHz oscillator / 2)
    - Toshiba TC5316200 2 MB mask ROM 1818-5008 (16 bit)
    - 1 MB DRAM (8x KM44C256), expansion to 5 MB
    - 93C46 serial EEPROM (settings, page count)
    - National SD9212AL (1820-6339), HP 1TW7-0301, HP 1TU3-0201 (19.401 MHz,
      Resolution Enhancement)
    - font cartridge slot, control panel, Canon print engine (DC controller)

    Emulated: firmware, EEPROM, control panel display (HD44780), parallel host
    port (data from a file through the bitbanger device), the serial
    command/status link to the Canon engine with the MP tray and the optional
    lower cassette as configuration settings, the glyph/rule merger and the
    band video output. Printed pages go to the "printout" device (PDF, or PNG
    files in the snapshot directory).

    Not verified against hardware: the engine timing (TOP after PRNT, line
    period) and the merger's raster operations, which are modelled as ROP3
    codes from the values the firmware uses.

    Paper handling (service manual): the MP tray has no size sensor, its size
    is set in the control panel (MP SIZE); the lower cassette reports its size
    with three switches. PS2 senses paper in the MP tray, PS6 in the lower
    cassette.

***************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "imagedev/bitbngr.h"
#include "imagedev/pdfprinter.h"
#include "machine/eepromser.h"
#include "video/hd44780.h"

#include "emupal.h"
#include "screen.h"

#include <bit>

#define VERBOSE 0
#include "logmacro.h"


namespace {

class hplj3p_state : public driver_device
{
public:
	hplj3p_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_eeprom(*this, "eeprom")
		, m_lcdc(*this, "lcdc")
		, m_host(*this, "host")
		, m_config(*this, "CONFIG")
		, m_keys(*this, "KEYS")
		, m_online_led(*this, "online_led")
		, m_formfeed_led(*this, "formfeed_led")
		, m_printout(*this, "printout")
	{ }

	void hplj3p(machine_config &config) ATTR_COLD;

private:
	required_device<m68000_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<hd44780_device> m_lcdc;
	required_device<bitbanger_device> m_host;
	required_ioport m_config;
	required_ioport m_keys;
	output_finder<> m_online_led;
	output_finder<> m_formfeed_led;
	required_device<pdf_printer_device> m_printout;

	// video output: the formatter streams bands of the page from DRAM, one line
	// per engine BD pulse; 380000 high byte: words per line, 380000 low byte and
	// 380002: start address (word aligned, CPU address - 0x800000), 380004: left
	// margin, 380006: bits 0-11 line count, bit 12 stop, bits 13-15 mode
	emu_timer *m_line_timer;
	TIMER_CALLBACK_MEMBER(video_line);
	bitmap_rgb32 m_page;
	int m_page_y;
	bool m_page_dirty;
	int m_band_lines;
	// Left margin offset: 380004 holds the start of the image in pixels after
	// the beam detect signal of each laser line, not after the paper edge.
	// The firmware writes 0x8d; with the paper edge 91 pixels after the
	// signal the PCL logical page starts 71 pixels (0.24 inch) from the edge
	// of an A4 sheet as specified, and a Letter image starts 50 pixels in, as
	// on the LaserJet 4L.  Taken from printed rules, not measured on a
	// printer; without it the image is 141 pixels to the right.
	static constexpr int MARGIN_OFFSET = 91;
	u32 m_band_addr;
	void eject_page();

	// glyph and rule merger: 3a0004 destination (CPU address, bits 0-22),
	// 3e0006 low byte line stride in words,
	// 3c0000 x position in pixels (bits 0-3 shift, bits 4-15 word offset), 3c0002 mask, 3c0004 mode,
	// 3e0000 data word inside a row, 3e0002 last data word of a row
	u32 m_blt_dest;
	u32 m_blt_row;
	u32 m_blt_ptr;
	u16 m_blt_ctrl;
	u16 m_blt_mask;
	u16 m_blt_mode;
	u16 m_blt_spill;
	u32 m_blt_pat;      // 3a0000: shading pattern, one word per row
	int m_blt_patrow;
	void blt_dest_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_pat_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_ctrl_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_data_w(offs_t offset, u16 data, u16 mem_mask);
	void blt_out(u16 data);

	// parallel host interface: data latch at 200001, latch full on 3e0006 bit 10
	// and IRQ 2, 200000 bit 7 pulsed by the firmware to acknowledge the byte
	u16 m_pctrl;
	u8 m_pdata;
	bool m_pfull;
	emu_timer *m_host_timer;
	TIMER_CALLBACK_MEMBER(host_poll);
	void pctrl_w(offs_t offset, u16 data, u16 mem_mask);
	u16 pdata_r(offs_t offset, u16 mem_mask);

	u16 m_panel;
	void panel_w(offs_t offset, u16 data, u16 mem_mask);
	HD44780_PIXEL_UPDATE(lcd_pixel_update);

	void main_map(address_map &map) ATTR_COLD;

	u16 m_ctrl;
	u16 m_ctrl2;        // 3e0006 written
	u16 m_irqctl;       // 3e0004 written
	bool m_irq4;        // engine sync interrupt (acknowledged with 3e0004 bit 7)
	bool m_irq5;        // video band interrupt (acknowledged with 3e0004 bit 6)
	bool m_top;         // engine TOP input (3e0006 bit 9)
	bool m_sheet;       // a sheet is passing the drum
	emu_timer *m_print_timer;
	TIMER_CALLBACK_MEMBER(print_step);
	void irqctl_w(offs_t offset, u16 data, u16 mem_mask);
	void video_w(offs_t offset, u16 data, u16 mem_mask);
	u16 m_video[4];

	// Canon engine command/status link (bit-banged by the firmware)
	u8 m_eng_cmd;
	u8 m_eng_sts;
	int m_eng_bits;
	int m_eng_cbits;    // command bits received
	bool m_eng_sbsy;    // engine has a status byte ready (formatter input 0, 280001 bit 1 low)
	bool m_eng_out;     // STS line state (1 = bit set)
	emu_timer *m_eng_timer;
	TIMER_CALLBACK_MEMBER(eng_reply);
	u8 eng_status(u8 cmd);
	void ctrl2_w(offs_t offset, u16 data, u16 mem_mask);
	u16 ctrl2_r(offs_t offset, u16 mem_mask);

	virtual void machine_start() override ATTR_COLD;

	void ctrl_w(offs_t offset, u16 data, u16 mem_mask);
	u16 status_r(offs_t offset, u16 mem_mask);

	u16 unk_r(offs_t offset, u16 mem_mask);
	void unk_w(offs_t offset, u16 data, u16 mem_mask);
};


u16 hplj3p_state::unk_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
		LOG("%s: read %06x & %04x\n", machine().describe_context(), offset << 1, mem_mask);
	return 0;
}

void hplj3p_state::unk_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("%s: write %06x = %04x & %04x\n", machine().describe_context(), offset << 1, data, mem_mask);
}

void hplj3p_state::machine_start()
{
	m_ctrl = 0;
	m_ctrl2 = 0;
	m_irqctl = 0;
	m_irq4 = m_irq5 = m_top = m_sheet = false;
	save_item(NAME(m_sheet));
	std::fill(std::begin(m_video), std::end(m_video), 0);
	m_print_timer = timer_alloc(FUNC(hplj3p_state::print_step), this);
	m_line_timer = timer_alloc(FUNC(hplj3p_state::video_line), this);
	m_blt_dest = m_blt_row = m_blt_ptr = 0;
	m_blt_ctrl = m_blt_mask = m_blt_mode = m_blt_spill = 0;
	m_blt_pat = 0;
	m_blt_patrow = 0;
	save_item(NAME(m_blt_pat));
	save_item(NAME(m_blt_patrow));
	save_item(NAME(m_blt_dest));
	save_item(NAME(m_blt_row));
	save_item(NAME(m_blt_ptr));
	save_item(NAME(m_blt_ctrl));
	save_item(NAME(m_blt_mask));
	save_item(NAME(m_blt_mode));
	save_item(NAME(m_blt_spill));
	m_page.allocate(2550, 3508);   // 8.5 x 11.69 inches: Letter width, A4 length
	m_page.fill(0xffffff);
	m_page_y = 0;
	m_page_dirty = false;
	m_band_lines = 0;
	m_band_addr = 0;
	save_item(NAME(m_page_y));
	save_item(NAME(m_page_dirty));
	save_item(NAME(m_band_lines));
	save_item(NAME(m_band_addr));
	save_item(NAME(m_irqctl));
	save_item(NAME(m_irq4));
	save_item(NAME(m_irq5));
	save_item(NAME(m_top));
	save_item(NAME(m_video));
	m_panel = 0;
	m_pctrl = 0;
	m_pdata = 0;
	m_pfull = false;
	m_host_timer = timer_alloc(FUNC(hplj3p_state::host_poll), this);
	m_host_timer->adjust(attotime::from_usec(20), 0, attotime::from_usec(20));
	save_item(NAME(m_pctrl));
	save_item(NAME(m_pdata));
	save_item(NAME(m_pfull));
	m_eng_cmd = 0;
	m_eng_sts = 0;
	m_eng_bits = 0;
	m_eng_cbits = 0;
	m_eng_sbsy = false;
	m_eng_out = false;
	m_eng_timer = timer_alloc(FUNC(hplj3p_state::eng_reply), this);
	save_item(NAME(m_ctrl));
	save_item(NAME(m_ctrl2));
	save_item(NAME(m_panel));
	save_item(NAME(m_eng_cmd));
	save_item(NAME(m_eng_sts));
	save_item(NAME(m_eng_bits));
	save_item(NAME(m_eng_cbits));
	save_item(NAME(m_eng_sbsy));
	save_item(NAME(m_eng_out));
}

// Status bytes as the firmware evaluates them (odd parity in bit 0, bit 7 set
// with bit 6 clear: command not supported by this engine). The firmware tells
// the engine variants apart by the commands they reject; the IIP/IIIP engine
// rejects 13 and 15 and has the MP tray plus the optional lower cassette (LC).
//   01  general status: bit 3 warming up, bit 4 printing error (41 ERROR)
//   02  bit 1 engine test, bit 2 door open or no EP cartridge (12),
//       bit 3 paper jam (13), bit 4 paper out
//   04  bit 4 52 ERROR, bit 5 51 ERROR, bit 6 50 SERVICE
//   0b  lower cassette size code in bits 6-1 (from switches SW901-SW903)
//   25  bit 5 lower cassette installed
//   2a  bit 6 paper in the MP tray (PS2), bit 5 paper in the lower cassette (PS6)
u8 hplj3p_state::eng_status(u8 cmd)
{
	u8 const cfg = m_config->read();
	unsigned const lc = cfg & 0x07;     // lower cassette size, 0 = not installed
	u8 data = 0x00;
	switch (cmd)
	{
	case 0x02:
		if (BIT(cfg, 5))
			data |= 0x04;
		break;
	case 0x0b:
	{
		// engine codes for Letter, Legal, A4 and Executive as translated by the
		// firmware's table at 0x5968
		static u8 const codes[] = { 0x00, 0x04, 0x0c, 0x01, 0x0d };
		data = (lc < std::size(codes) ? codes[lc] : 0) << 1;
		break;
	}
	case 0x13:
	case 0x15:
		data = 0x80;
		break;
	case 0x25:
		if (lc)
			data |= 0x20;
		break;
	case 0x2a:
		if (BIT(cfg, 3))
			data |= 0x40;
		if (lc && BIT(cfg, 4))
			data |= 0x20;
		break;
	}
	if (!(std::popcount(unsigned(data)) & 1))
		data |= 0x01;
	return data;
}

TIMER_CALLBACK_MEMBER(hplj3p_state::eng_reply)
{
	m_eng_sts = eng_status(m_eng_cmd);
	m_eng_bits = 0;
	m_eng_sbsy = true;
	LOG("engine cmd %02x -> sts %02x\n", m_eng_cmd, m_eng_sts);
}

// 3e0006: bits 0-7 merger line stride in words, bit 8 SCLK (the formatter
// clocks commands out and status in)
void hplj3p_state::ctrl2_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_ctrl2;
	COMBINE_DATA(&m_ctrl2);
	if (BIT(m_ctrl2, 8) && !BIT(old, 8))
	{
		if (!BIT(m_ctrl, 11))
		{
			// /CBSY low: command bit on 240000 bit 10 (active low), MSB first
			m_eng_cmd = (m_eng_cmd << 1) | (BIT(m_ctrl, 10) ? 0 : 1);
			m_eng_cbits++;
		}
		else if (m_eng_sbsy)
		{
			m_eng_out = BIT(m_eng_sts, 7 - m_eng_bits);
			if (++m_eng_bits == 8)
				m_eng_sbsy = false;
		}
	}
	if ((m_ctrl2 ^ old) & ~0x0100)
		LOG("%s: 3e0006 %04x\n", machine().describe_context(), m_ctrl2);
}

// 3e0006 read: bit 8 /STS, bit 9 engine input, bit 10 parallel data latch full
u16 hplj3p_state::ctrl2_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_eng_out ? 0x0000 : 0x0100;
	if (m_top)
		data |= 0x0200;
	if (m_pfull)
		data |= 0x0400;
	return data;
}

// 240000: bit 7 EEPROM CS, bit 8 CLK, bit 9 DI; engine: bit 10 /CMD, bit 11 /CBSY,
// bits 12-14 further engine outputs
void hplj3p_state::ctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_ctrl;
	COMBINE_DATA(&m_ctrl);
	if (!BIT(old, 11) && BIT(m_ctrl, 11) && m_eng_cbits == 8)
		m_eng_timer->adjust(attotime::from_usec(200));   // CBSY released: command complete
	if (BIT(old, 11) && !BIT(m_ctrl, 11))
	{
		m_eng_cmd = 0;
		m_eng_cbits = 0;
	}
	m_eeprom->di_write(BIT(m_ctrl, 9));
	m_eeprom->cs_write(BIT(m_ctrl, 7));
	m_eeprom->clk_write(BIT(m_ctrl, 8));
	if ((m_ctrl ^ old) & 0x7000)
		LOG("%s: ctrl %04x\n", machine().describe_context(), m_ctrl);
	if (BIT(old, 13) && !BIT(m_ctrl, 13))
		m_print_timer->adjust(attotime::from_seconds(2), 0);    // initial rotation (time not measured), then TOP
}

// 280000: bit 0 EEPROM DO, bit 1 /SBSY (engine status ready), bits 2/3 engine inputs
u16 hplj3p_state::status_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_eeprom->do_read() ? 0x0001 : 0x0000;
	data |= m_keys->read() << 8;    // control panel keys, active low
	data |= m_eng_sbsy ? 0x0000 : 0x0002;
	// bit 2 /RDY and bit 3 /PPRDY (engine powered): both held active (low) for an
	// engine that is switched on and ready; the firmware waits for /PPRDY before it
	// starts talking to the engine
	return data;
}

TIMER_CALLBACK_MEMBER(hplj3p_state::host_poll)
{
	// bit 15 of 200000: the formatter is ready for the next byte (it clears the
	// bit while it reads the latch and sets it again with the acknowledge)
	if (m_pfull || !BIT(m_pctrl, 15))
		return;
	u8 b;
	if (m_host->input(&b, 1) == 1)
	{
		m_pdata = b;
		m_pfull = true;
		m_maincpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
}

u16 hplj3p_state::pdata_r(offs_t offset, u16 mem_mask)
{
	return m_pdata;
}

void hplj3p_state::pctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_pctrl;
	COMBINE_DATA(&m_pctrl);
	if (BIT(m_pctrl, 7) && !BIT(old, 7) && m_pfull)
		m_pfull = false;
	// the interrupt is only requested while the formatter accepts data
	m_maincpu->set_input_line(M68K_IRQ_2, (m_pfull && BIT(m_pctrl, 15)) ? ASSERT_LINE : CLEAR_LINE);
	if ((m_pctrl ^ old) & 0x7f7f)
		LOG("%s: parallel control %04x\n", machine().describe_context(), m_pctrl);
}

TIMER_CALLBACK_MEMBER(hplj3p_state::print_step)
{
	switch (param)
	{
	case 0:
		// TOP for the next sheet: PRNT (240000 bit 13) and bit 12 active and the
		// previous sheet finished
		if (BIT(m_ctrl, 13))
			break;
		if (BIT(m_ctrl, 12) || m_sheet)
		{
			m_print_timer->adjust(attotime::from_msec(100), 0);
			break;
		}
		LOG("engine TOP\n");
		m_sheet = true;
		m_top = true;
		m_irq4 = true;
		m_maincpu->set_input_line(M68K_IRQ_4, ASSERT_LINE);
		m_print_timer->adjust(attotime::from_msec(10), 1);
		break;
	case 1:
		m_top = false;
		break;
	}
}

void hplj3p_state::irqctl_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_irqctl;
	COMBINE_DATA(&m_irqctl);
	if (BIT(m_irqctl, 7) && !BIT(old, 7) && m_irq4)
	{
		m_irq4 = false;
		m_maincpu->set_input_line(M68K_IRQ_4, CLEAR_LINE);
	}
	if (BIT(m_irqctl, 6) && !BIT(old, 6) && m_irq5)
	{
		m_irq5 = false;
		m_maincpu->set_input_line(M68K_IRQ_5, CLEAR_LINE);
	}
	if ((m_irqctl ^ old) & 0xff1f)
		LOG("%s: 3e0004 %04x\n", machine().describe_context(), m_irqctl);
}

void hplj3p_state::video_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_video[offset];
	COMBINE_DATA(&m_video[offset]);
	LOG("%s: video %06x = %04x\n", machine().describe_context(), 0x380000 + offset * 2, m_video[offset]);
	if (offset == 3 && BIT(m_video[3], 12) && !BIT(old, 12) && !m_band_lines)
		eject_page();   // video stopped after the last band: the page is complete
	if (offset == 3 && !BIT(m_video[3], 12))
	{
		m_band_lines = m_video[3] & 0x0fff;
		m_band_addr = ((u32(m_video[0] & 0x7f) << 16) | (m_video[1] & 0xfffe)) + 0x800000;
		if (m_band_lines && !m_line_timer->enabled())
			m_line_timer->adjust(attotime::from_usec(1690));   // line period: about 50 mm/s at 300 lines per inch (not measured)
	}
}

TIMER_CALLBACK_MEMBER(hplj3p_state::video_line)
{
	if (!m_band_lines || BIT(m_video[3], 12))
		return;
	int const words = m_video[0] >> 8;
	int const x0 = (m_video[2] & 0x0fff) - MARGIN_OFFSET;
	if (BIT(m_video[3], 14) && m_page_y < m_page.height())
	{
		address_space &space = m_maincpu->space(AS_PROGRAM);
		u32 *const line = &m_page.pix(m_page_y);
		for (int w = 0; w < words; w++)
		{
			u16 const d = space.read_word(m_band_addr + w * 2);
			space.write_word(m_band_addr + w * 2, 0);   // the band memory is cleared as it is read out (the firmware never clears it)
			for (int b = 0; b < 16; b++)
			{
				int const x = x0 + w * 16 + b;
				if (BIT(d, 15 - b) && x >= 0 && x < m_page.width())
				{
					line[x] = 0x000000;
					m_page_dirty = true;
				}
			}
		}
		m_band_addr += words * 2;
	}
	m_page_y++;
	if (--m_band_lines)
		m_line_timer->adjust(attotime::from_usec(1690));
	else
	{
		LOG("band done, page line %d\n", m_page_y);
		m_irq5 = true;
		m_maincpu->set_input_line(M68K_IRQ_5, ASSERT_LINE);
	}
}

void hplj3p_state::blt_pat_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_pat = (m_blt_pat & 0xffff0000) | data;
	else
		m_blt_pat = (m_blt_pat & 0x0000ffff) | (u32(data) << 16);
	m_blt_patrow = 0;
}

void hplj3p_state::blt_dest_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_dest = (m_blt_dest & 0xffff0000) | (data & mem_mask) | (m_blt_dest & ~mem_mask & 0xffff);
	else
		m_blt_dest = (m_blt_dest & 0x0000ffff) | (u32(data & mem_mask) << 16) | (m_blt_dest & (u32(~mem_mask & 0xffff) << 16));
	m_blt_row = m_blt_dest + (m_blt_ctrl >> 4) * 2;
	m_blt_ptr = m_blt_row;
	m_blt_spill = 0;
}

void hplj3p_state::blt_ctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch (offset)
	{
	case 0: COMBINE_DATA(&m_blt_ctrl); break;
	case 1: COMBINE_DATA(&m_blt_mask); break;
	case 2:
	{
		u16 const old = m_blt_mode;
		COMBINE_DATA(&m_blt_mode);
		if ((old ^ m_blt_mode) & 0xff00)
			LOG("%s: merger mode %04x\n", machine().describe_context(), m_blt_mode);
		break;
	}
	}
}

void hplj3p_state::blt_out(u16 data)
{
	if (m_blt_dest == 0xffffffff || m_blt_ptr < 0xb00000 || m_blt_ptr > 0xfffffe)
		return;
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u16 const d = space.read_word(m_blt_ptr);
	u16 const pat = space.read_word((m_blt_pat & 0xfffffe) + (m_blt_patrow & 15) * 2);
	// raster operation from the low byte of 3c0004, indexed by data, pattern and
	// destination bits like a ROP3 code (fa: OR the data into the page, ea: OR the
	// data masked with the shading pattern)
	u8 const rop = m_blt_mode & 0xff;
	u16 r = 0;
	for (int b = 0; b < 16; b++)
		if (BIT(rop, (BIT(data, b) << 2) | (BIT(pat, b) << 1) | BIT(d, b)))
			r |= 1 << b;
	space.write_word(m_blt_ptr, (d & ~m_blt_mask) | (r & m_blt_mask));
	m_blt_ptr += 2;
}

void hplj3p_state::blt_data_w(offs_t offset, u16 data, u16 mem_mask)
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
		m_blt_row += (m_ctrl2 & 0xff) * 2;   // line stride in words: 3e0006 low byte
		m_blt_ptr = m_blt_row;
		m_blt_patrow++;
	}
}

void hplj3p_state::eject_page()
{
	if (m_page_dirty || m_page_y)
	{
		LOG("page complete, %d lines\n", m_page_y);
		m_printout->print_bitmap(m_page, 300, 300);
	}
	m_page.fill(0xffffff);
	m_page_y = 0;
	m_page_dirty = false;
	// PRNT still active: the next sheet follows
	m_sheet = false;
	if (!BIT(m_ctrl, 13))
		m_print_timer->adjust(attotime::from_seconds(1), 0);
}

// 280000 written: control panel HD44780 in 4 bit mode, bits 0-3 D4-D7,
// bit 6 RS, bit 7 E; bit 4 Form Feed and bit 5 On Line indicators (active low)
void hplj3p_state::panel_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_panel;
	COMBINE_DATA(&m_panel);
	m_lcdc->db_w((m_panel & 0x0f) << 4);
	m_lcdc->rs_w(BIT(m_panel, 6));
	m_lcdc->rw_w(0);
	m_lcdc->e_w(BIT(m_panel, 7));
	m_online_led = BIT(m_panel, 5) ? 0 : 1;      // active low
	m_formfeed_led = BIT(m_panel, 4) ? 0 : 1;
	if ((m_panel ^ old) & 0xff30)
		LOG("%s: panel %04x\n", machine().describe_context(), m_panel);
}

HD44780_PIXEL_UPDATE(hplj3p_state::lcd_pixel_update)
{
	// two rows of eight characters (user's manual)
	if (x < 5 && y < 8 && line < 2 && pos < 8)
		bitmap.pix(line * 9 + y, pos * 6 + x) = state;
}

void hplj3p_state::main_map(address_map &map)
{
	map(0x000000, 0xffffff).rw(FUNC(hplj3p_state::unk_r), FUNC(hplj3p_state::unk_w));
	map(0x000000, 0x1fffff).rom().region("maincpu", 0);
	map(0x200000, 0x200001).rw(FUNC(hplj3p_state::pdata_r), FUNC(hplj3p_state::pctrl_w));
	map(0x240000, 0x240001).w(FUNC(hplj3p_state::ctrl_w));
	map(0x280000, 0x280001).rw(FUNC(hplj3p_state::status_r), FUNC(hplj3p_state::panel_w));
	map(0x380000, 0x380007).w(FUNC(hplj3p_state::video_w));
	map(0x3a0000, 0x3a0003).w(FUNC(hplj3p_state::blt_pat_w));
	map(0x3a0004, 0x3a0007).w(FUNC(hplj3p_state::blt_dest_w));
	map(0x3c0000, 0x3c0005).w(FUNC(hplj3p_state::blt_ctrl_w));
	map(0x3e0000, 0x3e0003).w(FUNC(hplj3p_state::blt_data_w));
	map(0x3e0004, 0x3e0005).w(FUNC(hplj3p_state::irqctl_w));
	map(0x3e0006, 0x3e0007).rw(FUNC(hplj3p_state::ctrl2_r), FUNC(hplj3p_state::ctrl2_w));
	map(0xb00000, 0xbfffff).ram();
}


static INPUT_PORTS_START( hplj3p )
	// control panel: the secondary functions (Reset, Continue, Print Fonts,
	// Test, minus) are selected by holding Shift while clicking the key
	PORT_START("KEYS")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("On Line / Reset") PORT_CODE(KEYCODE_O)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("Form Feed / Continue") PORT_CODE(KEYCODE_F)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("Menu / Print Fonts") PORT_CODE(KEYCODE_M)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("Enter / Test") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("+ / -") PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x07, 0x00, "Lower cassette (LC)" )
	PORT_CONFSETTING(    0x00, "Not installed" )
	PORT_CONFSETTING(    0x01, "Letter" )
	PORT_CONFSETTING(    0x02, "Legal" )
	PORT_CONFSETTING(    0x03, "A4" )
	PORT_CONFSETTING(    0x04, "Executive" )
	PORT_CONFNAME( 0x08, 0x08, "Paper in MP tray" )
	PORT_CONFSETTING(    0x00, DEF_STR( No ) )
	PORT_CONFSETTING(    0x08, DEF_STR( Yes ) )
	PORT_CONFNAME( 0x10, 0x10, "Paper in lower cassette" )
	PORT_CONFSETTING(    0x00, DEF_STR( No ) )
	PORT_CONFSETTING(    0x10, DEF_STR( Yes ) )
	PORT_CONFNAME( 0x20, 0x00, "Paper path door" )
	PORT_CONFSETTING(    0x00, "Closed" )
	PORT_CONFSETTING(    0x20, "Open" )
INPUT_PORTS_END


void hplj3p_state::hplj3p(machine_config &config)
{
	M68000(config, m_maincpu, 32_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &hplj3p_state::main_map);
	// system tick on level 3: the firmware counts 167 ticks per second (source and acknowledge not known)
	m_maincpu->set_periodic_int(FUNC(hplj3p_state::irq3_line_hold), attotime::from_hz(32_MHz_XTAL / 192000));

	EEPROM_93C46_16BIT(config, m_eeprom);

	BITBANGER(config, m_host, 0);   // data from the host on the parallel port

	PDF_PRINTER(config, m_printout);

	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_refresh_hz(60);
	screen.set_vblank_time(ATTOSECONDS_IN_USEC(2500));
	screen.set_screen_update(m_lcdc, FUNC(hd44780_device::screen_update));
	screen.set_size(8 * 6, 2 * 9 - 1);
	screen.set_visarea_full();
	screen.set_palette("palette");
	PALETTE(config, "palette", palette_device::MONOCHROME_INVERTED);

	HD44780(config, m_lcdc, 270'000);   // clock not measured
	m_lcdc->set_lcd_size(2, 8);
	m_lcdc->set_pixel_update_cb(FUNC(hplj3p_state::lcd_pixel_update));
}


ROM_START( hplj3p )
	ROM_REGION16_BE( 0x200000, "maincpu", 0 )
	ROM_LOAD16_WORD_SWAP( "1818-5008.bin", 0x000000, 0x200000, CRC(900d4ea8) SHA1(c28333147b125bf332ce034e817df82887539cb3) )

	ROM_REGION16_BE( 0x80, "eeprom", 0 )
	ROM_LOAD16_WORD_SWAP( "93c46.bin", 0x00, 0x80, CRC(f8bf50ea) SHA1(1257d5b3912f8696f6843a13ac7e7bc7afdcef4e) )
ROM_END

} // anonymous namespace


//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY            FULLNAME          FLAGS
COMP( 1991, hplj3p, 0,      0,      hplj3p,  hplj3p, hplj3p_state, empty_init, "Hewlett-Packard", "LaserJet IIIP",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
