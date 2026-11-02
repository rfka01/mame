// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP LaserJet 4L (C2003A), 1993

    Formatter C2004-80101:
    - Motorola 1TY4 SC414210FT: 68000 core and the printer logic in one chip
    - 2 MB mask ROM 1818-5601 (16 bit)
    - 1 MB DRAM (2x HM514260), the firmware detects 1 MB more on a memory card
      (a configuration setting)
    - 31.3076 MHz crystal
    - Bi-Tronics parallel port, one button, four LEDs, Canon print engine

    The formatter logic in the 1TY4 works like the separate gate arrays of the
    LaserJet IIIP (hp_lj3p.cpp): the same merger, the same serial link to the
    engine's DC controller and the same band output.  The register meanings
    come from the firmware; timings of the engine are not measured.

    Host interface (user's manual, appendix A): the Bi-Tronics parallel port,
    a Centronics port with the IEEE 1284 reverse channel in nibble mode.  The
    device plugs into a Centronics port.  The 1TY4 negotiates 1284 itself
    and tells the firmware the extensibility byte (00 nibble mode, 04 device
    ID); the bytes the firmware writes for the reverse channel (PJL USTATUS
    and INFO answers, the device ID) go to the host in nibble mode.

    Working: self test page (hold the button for two seconds), print jobs
    from the computer, pages go to the PDF printer ("printout" device).

    Front panel: LEDs Ready, Data, Paper and Error (outputs ready_led,
    data_led, paper_led, error_led), one button.

    Registers that are written but not emulated (the PDF output is not
    affected):
    - 830000: EconoMode (0a on, 00 off)
    - 870000: 0f2e, 0faf on another engine status; probably laser timing
    - 87000a: Resolution Enhancement on (1) or off (0), 870002-870008 its
      pulse widths (30 70 f8 1fc for Medium, 30 70 fc 3fe for Dark)

    TODO:
    - status meanings of the 4L engine commands 1c, 3e, 40, 4f, 51, 7f, 9d,
      a7, f1 (7f is only sent for a print density other than the default;
      all of them are answered with 00, which the firmware accepts)
    - status 2a: only bit 6 (paper in the cassette) is emulated; with bit 7
      set the firmware reports ENGINE ERROR
    - the firmware's tables follow f1 and 9d with a parameter byte (0e, 00)
      and it expects a second status for them (routine at 16ac6); the
      handshake is not understood: answered with one status, the firmware
      skips the parameter byte, a second /SBSY right after the first one
      stops the engine communication
    - the parallel port's status lines in compatibility mode (SELECT,
      PERROR, /FAULT): the register that drives them is not known, the
      device reports a selected printer without errors
    - IEEE 1284 modes other than nibble mode (ECP, EPP) and the exact line
      timing of the negotiation
    - meaning of the parallel port events in 820002 bits 2 and 3 (the
      firmware acknowledges them with 820006 bits 2 and 1 and only counts
      them as host activity, which ends Intelligent Off)
    - manual feed: the firmware keeps status 2a bit 5 in 60089a, which the
      print start code at 17d1a checks before it asserts engine output
      bit 13 instead of bit 11; setting the bit changed nothing in tests,
      and a manual feed job (ESC &l2H) prints at once
    - engine timings are estimates: 2 s from the print request to TOP,
      1.69 ms per line; the left margin offset (MARGIN_OFFSET) comes from
      printed rules, not from a printer

***************************************************************************/

#include "emu.h"
#include "hp_lj4l.h"

#include "cpu/m68000/m68000.h"
#include "imagedev/pdfprinter.h"

#include <bit>

#define VERBOSE 0
#include "logmacro.h"


namespace {

class hp_lj4l_device : public device_t, public device_centronics_peripheral_interface
{
public:
	hp_lj4l_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	DECLARE_INPUT_CHANGED_MEMBER(button_changed);

	// Centronics interface
	virtual void input_strobe(int state) override;
	virtual void input_autofd(int state) override;
	virtual void input_select_in(int state) override;
	virtual void input_init(int state) override;
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	required_device<m68000_device> m_maincpu;
	required_ioport m_config;
	output_finder<> m_ready_led;
	output_finder<> m_data_led;
	output_finder<> m_paper_led;
	output_finder<> m_error_led;
	std::unique_ptr<u16[]> m_ram;   // 1 MB on the formatter, 1 MB on the memory card

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

	// Bi-Tronics parallel port with IEEE 1284 negotiation in hardware:
	// 820000 data (read: received byte, written: byte for the reverse channel)
	// 820002 status: bit 0 data received, bit 1 and bit 5 reverse channel
	//        ready, bit 6 1284 mode changed, bit 7 host in a 1284 mode,
	//        bit 8 negotiation done (bits 2 and 3 further events, not used here)
	// 820006 written: event acknowledge (bit 3: mode change)
	// 82000a interrupt enables: bit 0 data received, bit 3 reverse channel
	//        ready, bit 5 mode change, bit 6 negotiation (bits 1, 4: bits 3, 2)
	// 82000c read: extensibility byte of the negotiation (04: device ID);
	//        written: acknowledges the negotiation
	// The bus side follows IEEE 1284: compatibility mode (the byte is latched
	// on the falling edge of /STROBE, BUSY until the firmware reads it, then
	// an /ACK pulse), negotiation (1284 active with /SELECT IN high and
	// /AUTOFD low, the extensibility byte latched with /STROBE) and nibble
	// mode (the host requests each nibble with /AUTOFD low, the printer puts
	// it on /FAULT, SELECT, PERROR and BUSY and pulses /ACK; /FAULT low tells
	// the host that data is waiting).
	enum
	{
		P_COMPAT = 0,   // compatibility mode
		P_NEG,          // negotiation: waiting for the extensibility byte
		P_NEG_LATCHED,  // extensibility byte latched, waiting for /AUTOFD high
		P_REVERSE       // nibble mode reverse idle
	};
	u8 m_data;          // Centronics data lines
	int m_strobe;
	int m_autofd;
	int m_slctin;
	int m_pmode;
	int m_nibble;       // nibble being sent (0 low, 1 high), -1 none
	u8 m_pdata;
	bool m_pfull;
	u8 m_tx;            // byte for the reverse channel
	bool m_tx_full;
	u16 m_pen;
	u16 m_pevt;         // latched events: bit 6 1284 line change, bit 8 negotiation
	u8 m_pext;          // extensibility byte of the last negotiation
	u16 m_pcfg;         // 82000c low bits written by the firmware
	emu_timer *m_ack_timer;
	TIMER_CALLBACK_MEMBER(ack_end);
	void set_lines(int busy, int ack, int perror, int select, int fault);
	void start_negotiation();
	void update_lines();
	u16 pext_r(offs_t offset, u16 mem_mask);
	void pext_w(offs_t offset, u16 data, u16 mem_mask);
	void pack_w(offs_t offset, u16 data, u16 mem_mask);
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


ROM_START( hp_lj4l )
	ROM_REGION16_BE( 0x200000, "maincpu", 0 )
	ROM_LOAD16_WORD_SWAP( "1818-5601.bin", 0x000000, 0x200000, CRC(75773ecb) SHA1(f7a14f591976a4bf74d9847f2bee7f86c119f3a2) )
ROM_END


static INPUT_PORTS_START( hp_lj4l )
	PORT_START("BUTTON")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYPAD ) PORT_NAME("LaserJet Front Panel Button") PORT_CODE(KEYCODE_ENTER) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(hp_lj4l_device::button_changed), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper in cassette" )
	PORT_CONFSETTING(    0x00, DEF_STR( No ) )
	PORT_CONFSETTING(    0x01, DEF_STR( Yes ) )
	PORT_CONFNAME( 0x02, 0x00, "Top door" )
	PORT_CONFSETTING(    0x00, "Closed" )
	PORT_CONFSETTING(    0x02, "Open" )
	PORT_CONFNAME( 0x04, 0x00, "Memory card (1 MB)" )
	PORT_CONFSETTING(    0x00, "Not installed" )
	PORT_CONFSETTING(    0x04, "Installed" )
INPUT_PORTS_END


hp_lj4l_device::hp_lj4l_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_LJ4L, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_config(*this, "CONFIG"),
	m_ready_led(*this, "ready_led"),
	m_data_led(*this, "data_led"),
	m_paper_led(*this, "paper_led"),
	m_error_led(*this, "error_led"),
	m_button(*this, "BUTTON"),
	m_printout(*this, "printout")
{
}

const tiny_rom_entry *hp_lj4l_device::device_rom_region() const
{
	return ROM_NAME( hp_lj4l );
}

ioport_constructor hp_lj4l_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( hp_lj4l );
}

u16 hp_lj4l_device::unk_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
		LOG("%s: read %06x & %04x\n", machine().describe_context(), offset << 1, mem_mask);
	return 0;
}

void hp_lj4l_device::unk_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("%s: write %06x = %04x & %04x\n", machine().describe_context(), offset << 1, data, mem_mask);
}

void hp_lj4l_device::device_start()
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
	m_tick_timer = timer_alloc(FUNC(hp_lj4l_device::tick), this);
	m_tick_timer->adjust(attotime::from_hz(370), 0, attotime::from_hz(370));

	m_eng_ctrl = 0xffff;
	m_eng_cmd = m_eng_sts = 0;
	m_eng_cbits = m_eng_bits = 0;
	m_eng_sbsy = m_eng_out = false;
	m_eng_timer = timer_alloc(FUNC(hp_lj4l_device::eng_reply), this);
	m_top_timer = timer_alloc(FUNC(hp_lj4l_device::eng_top), this);
	save_item(NAME(m_eng_ctrl));
	save_item(NAME(m_eng_cmd));
	save_item(NAME(m_eng_sts));
	save_item(NAME(m_eng_cbits));
	save_item(NAME(m_eng_bits));
	save_item(NAME(m_eng_sbsy));
	save_item(NAME(m_eng_out));

	m_leds = 0x1f;
	save_item(NAME(m_leds));


	m_ram = std::make_unique<u16[]>(0x100000);
	save_pointer(NAME(m_ram), 0x100000);

	m_data = 0;
	m_strobe = m_autofd = m_slctin = 1;
	m_pmode = P_COMPAT;
	m_nibble = -1;
	m_pdata = 0;
	m_pfull = false;
	m_tx = 0;
	m_tx_full = false;
	m_pen = 0;
	m_pevt = 0;
	m_pext = 0;
	m_pcfg = 0;
	m_ack_timer = timer_alloc(FUNC(hp_lj4l_device::ack_end), this);
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_autofd));
	save_item(NAME(m_slctin));
	save_item(NAME(m_pmode));
	save_item(NAME(m_nibble));
	save_item(NAME(m_pdata));
	save_item(NAME(m_pfull));
	save_item(NAME(m_tx));
	save_item(NAME(m_tx_full));
	save_item(NAME(m_pen));
	save_item(NAME(m_pevt));
	save_item(NAME(m_pext));
	save_item(NAME(m_pcfg));
	std::fill(std::begin(m_video), std::end(m_video), 0);
	m_cur = m_next = band_t{ 0, 0, 0, 0, false };
	m_cur_active = m_next_valid = m_stop_pending = false;
	m_page.allocate(2550, 3508);   // 8.5 x 11.69 inches: Letter width, A4 length
	m_page.fill(0xffffff);
	m_page_y = 0;
	m_page_dirty = false;
	m_line_timer = timer_alloc(FUNC(hp_lj4l_device::video_line), this);
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

}

void hp_lj4l_device::device_reset()
{
	// 1 MB on the formatter at 600000, the memory card follows at 700000
	address_space &space = m_maincpu->space(AS_PROGRAM);
	space.install_ram(0x600000, 0x6fffff, &m_ram[0]);
	if (BIT(m_config->read(), 2))
		space.install_ram(0x700000, 0x7fffff, &m_ram[0x80000]);
	else
		space.install_readwrite_handler(0x700000, 0x7fffff, read16s_delegate(*this, FUNC(hp_lj4l_device::unk_r)), write16s_delegate(*this, FUNC(hp_lj4l_device::unk_w)));

	m_pmode = P_COMPAT;
	m_nibble = -1;
	update_lines();
}

hp_lj4l_device::band_t hp_lj4l_device::latch_band() const
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

void hp_lj4l_device::video_w(offs_t offset, u16 data, u16 mem_mask)
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

TIMER_CALLBACK_MEMBER(hp_lj4l_device::video_line)
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

void hp_lj4l_device::eject_page()
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

void hp_lj4l_device::update_pirq()
{
	bool const irq = (m_pfull && BIT(m_pen, 0)) || (!m_tx_full && BIT(m_pen, 3))
			|| (BIT(m_pevt, 6) && BIT(m_pen, 5)) || (BIT(m_pevt, 8) && BIT(m_pen, 6));
	if (irq)
		m_irq_pend |= 0x10;
	else
		m_irq_pend &= ~0x10;
	update_irq();
}

void hp_lj4l_device::set_lines(int busy, int ack, int perror, int select, int fault)
{
	output_busy(busy);
	output_ack(ack);
	output_perror(perror);
	output_select(select);
	output_fault(fault);
}

// status lines outside a nibble transfer
void hp_lj4l_device::update_lines()
{
	switch (m_pmode)
	{
	case P_COMPAT:
		// BUSY while a byte waits in the latch or the formatter does not accept
		// data (receive interrupt disabled)
		output_busy((m_pfull || !BIT(m_pen, 0)) ? 1 : 0);
		output_perror(0);
		output_select(1);
		output_fault(1);
		break;
	case P_NEG:
		// event 2: the printer answers the negotiation request
		set_lines(0, 0, 1, 1, 1);
		break;
	case P_NEG_LATCHED:
		break;
	case P_REVERSE:
		// reverse idle: /FAULT (nDataAvail) low while a byte waits for the host
		if (m_nibble < 0)
		{
			output_busy(0);
			output_perror(m_tx_full ? 0 : 1);
			output_fault(m_tx_full ? 0 : 1);
		}
		break;
	}
}

// event 1: 1284 active (/SELECT IN high) with HostBusy (/AUTOFD) low, the
// extensibility byte is on the data lines
void hp_lj4l_device::start_negotiation()
{
	LOG("1284 negotiation request at %.6f\n", machine().time().as_double());
	m_pmode = P_NEG;
	m_nibble = -1;
	m_pevt |= 0x0040;
}

void hp_lj4l_device::input_select_in(int state)
{
	if (state == m_slctin)
		return;
	m_slctin = state;
	if (state && !m_autofd && m_pmode == P_COMPAT)
		start_negotiation();
	else if (!state && m_pmode != P_COMPAT)
	{
		// termination: back to compatibility mode
		LOG("1284 termination at %.6f\n", machine().time().as_double());
		m_pmode = P_COMPAT;
		m_nibble = -1;
		m_pevt |= 0x0040;
		output_ack(1);
	}
	update_lines();
	update_pirq();
}

void hp_lj4l_device::input_autofd(int state)
{
	if (state == m_autofd)
		return;
	m_autofd = state;
	switch (m_pmode)
	{
	case P_COMPAT:
		if (!state && m_slctin)
		{
			start_negotiation();
			update_lines();
			update_pirq();
		}
		break;
	case P_NEG_LATCHED:
		if (state)
		{
			// events 5 and 6: SELECT is the XFlag (set for a supported
			// extensibility request, the device ID; nibble mode needs none),
			// PERROR and /FAULT tell whether reverse data is waiting
			output_perror(m_tx_full ? 0 : 1);
			output_select(m_pext ? 1 : 0);
			output_fault(m_tx_full ? 0 : 1);
			output_busy(0);
			output_ack(1);
			m_pmode = P_REVERSE;
			m_pevt |= 0x0100;
			update_pirq();
		}
		break;
	case P_REVERSE:
		if (!state && m_tx_full && m_nibble < 0)
			m_nibble = 0;
		if (!state && m_nibble >= 0)
		{
			// event 8: nibble on /FAULT (bit 0), SELECT, PERROR, BUSY (bit 3), then /ACK low
			u8 const n = m_nibble ? (m_tx >> 4) : (m_tx & 0x0f);
			output_fault(BIT(n, 0));
			output_select(BIT(n, 1));
			output_perror(BIT(n, 2));
			output_busy(BIT(n, 3));
			output_ack(0);
		}
		else if (state && m_nibble >= 0)
		{
			// event 10: /ACK high; after the high nibble the byte is done
			output_ack(1);
			if (m_nibble)
			{
				LOG("reverse channel %02x '%c'\n", m_tx, (m_tx >= 0x20 && m_tx < 0x7f) ? m_tx : '.');
				m_nibble = -1;
				m_tx_full = false;
				update_lines();
				update_pirq();
			}
			else
				m_nibble = 1;
		}
		break;
	}
}

void hp_lj4l_device::input_strobe(int state)
{
	int const old = m_strobe;
	m_strobe = state;
	if (!old || state)
		return;
	switch (m_pmode)
	{
	case P_COMPAT:
		// the byte is latched on the falling edge of /STROBE
		if (!m_pfull)
		{
			m_pdata = m_data;
			m_pfull = true;
			output_busy(1);
			update_pirq();
		}
		break;
	case P_NEG:
		// event 3: the extensibility byte
		m_pext = m_data;
		m_pmode = P_NEG_LATCHED;
		LOG("1284 extensibility byte %02x\n", m_pext);
		break;
	}
}

void hp_lj4l_device::input_init(int state)
{
	if (!state)
		LOG("%s: /INIT from the host (not handled)\n", machine().describe_context());
}

TIMER_CALLBACK_MEMBER(hp_lj4l_device::ack_end)
{
	if (m_pmode == P_COMPAT)
	{
		output_ack(1);
		update_lines();
	}
}

u16 hp_lj4l_device::pdata_r(offs_t offset, u16 mem_mask)
{
	u16 const data = m_pdata;
	if (!machine().side_effects_disabled() && m_pfull)
	{
		// the byte is taken: /ACK pulse, BUSY goes away with it
		m_pfull = false;
		if (m_pmode == P_COMPAT)
		{
			output_ack(0);
			m_ack_timer->adjust(attotime::from_usec(5));
		}
		update_pirq();
	}
	return data;
}

void hp_lj4l_device::pdata_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_tx = data & 0xff;
	m_tx_full = true;
	update_lines();
	update_pirq();
}

u16 hp_lj4l_device::pstat_r(offs_t offset, u16 mem_mask)
{
	return (m_pfull ? 0x0001 : 0x0000) | (m_tx_full ? 0x0000 : 0x0022) | ((m_pmode != P_COMPAT) ? 0x0080 : 0x0000) | m_pevt;
}

// 820006 written: acknowledges events (bit 3: 1284 line change)
void hp_lj4l_device::pack_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("%s: 820006 = %04x & %04x\n", machine().describe_context(), data, mem_mask);
	if (BIT(data, 3))
		m_pevt &= ~0x0040;
	update_pirq();
}

u16 hp_lj4l_device::pext_r(offs_t offset, u16 mem_mask)
{
	return m_pext | (m_pcfg & 1);
}

// 82000c written: acknowledges the negotiation, bit 0 set while the device ID is sent
void hp_lj4l_device::pext_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOG("%s: 82000c = %04x & %04x\n", machine().describe_context(), data, mem_mask);
	m_pcfg = data & 1;
	m_pevt &= ~0x0100;
	update_pirq();
}

void hp_lj4l_device::pen_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_pen);
	update_pirq();
	update_lines();
}

void hp_lj4l_device::leds_w(offs_t offset, u16 data, u16 mem_mask)
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
u8 hp_lj4l_device::eng_status(u8 cmd)
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

TIMER_CALLBACK_MEMBER(hp_lj4l_device::eng_reply)
{
	m_eng_sts = eng_status(m_eng_cmd);
	m_eng_bits = 0;
	m_eng_sbsy = true;
	LOG("engine cmd %02x -> sts %02x\n", m_eng_cmd, m_eng_sts);
}

TIMER_CALLBACK_MEMBER(hp_lj4l_device::eng_top)
{
	// paper at the top of page: interrupt source 1 (level 5)
	LOG("engine TOP at %.3f\n", machine().time().as_double());
	m_irq_pend |= 0x02;
	update_irq();
}

u16 hp_lj4l_device::eng_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_eng_ctrl & 0x00ff;
	if (!m_eng_out)
		data |= 0x0100;
	if (!m_eng_sbsy)
		data |= 0x0800;
	// /RDY and /PPRDY held active for an engine that is switched on
	return data;
}

void hp_lj4l_device::eng_w(offs_t offset, u16 data, u16 mem_mask)
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

TIMER_CALLBACK_MEMBER(hp_lj4l_device::tick)
{
	m_irq_pend |= 0x04;
	update_irq();
}

void hp_lj4l_device::update_irq()
{
	// source n requests level 6 - n (the firmware masks source 2 around the
	// tick counter on level 4, source 3 around the button handler on level 3
	// and source 4 around the parallel port handler on level 2)
	u8 const act = m_irq_pend & ~m_irq_mask;
	for (int n = 0; n < 6; n++)
		m_maincpu->set_input_line(6 - n, BIT(act, n) ? ASSERT_LINE : CLEAR_LINE);
}

INPUT_CHANGED_MEMBER(hp_lj4l_device::button_changed)
{
	m_irq_pend |= 0x08;
	update_irq();
}

// 850002 read: front panel button, bit 0 pressed, bit 1 released (the
// firmware runs the continuous self test when bit 1 is clear at power on);
// the read acknowledges the button interrupt
u16 hp_lj4l_device::button_r(offs_t offset, u16 mem_mask)
{
	u16 const data = m_button->read() ? 0x0001 : 0x0002;
	if (!machine().side_effects_disabled() && BIT(m_irq_pend, 3))
	{
		m_irq_pend &= ~0x08;
		update_irq();
	}
	return data;
}

u16 hp_lj4l_device::irqmask_r(offs_t offset, u16 mem_mask)
{
	return m_irq_mask;
}

void hp_lj4l_device::irqmask_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (ACCESSING_BITS_0_7)
		m_irq_mask = data & 0xff;
	update_irq();
}

void hp_lj4l_device::irqack_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_irq_pend &= ~data;
	update_irq();
}

void hp_lj4l_device::blt_pat_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_pat = (m_blt_pat & 0xffff0000) | data;
	else
		m_blt_pat = (m_blt_pat & 0x0000ffff) | (u32(data) << 16);
	m_blt_patrow = 0;
}

void hp_lj4l_device::blt_dest_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset)
		m_blt_dest = (m_blt_dest & 0xffff0000) | (data & mem_mask) | (m_blt_dest & ~mem_mask & 0xffff);
	else
		m_blt_dest = (m_blt_dest & 0x0000ffff) | (u32(data & mem_mask) << 16) | (m_blt_dest & (u32(~mem_mask & 0xffff) << 16));
	m_blt_row = (m_blt_dest & 0x7ffffe) + (m_blt_ctrl >> 4) * 2;
	m_blt_ptr = m_blt_row;
	m_blt_spill = 0;
}

void hp_lj4l_device::blt_ctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch (offset)
	{
	case 0: COMBINE_DATA(&m_blt_ctrl); break;
	case 1: COMBINE_DATA(&m_blt_mask); break;
	case 2: COMBINE_DATA(&m_blt_mode); break;
	default: LOG("%s: merger 90000e = %04x\n", machine().describe_context(), data); break;
	}
}

void hp_lj4l_device::blt_cfg_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const old = m_blt_cfg;
	COMBINE_DATA(&m_blt_cfg);
	if ((old ^ m_blt_cfg) & 0xff00)
		LOG("%s: merger config %04x\n", machine().describe_context(), m_blt_cfg);
}

void hp_lj4l_device::blt_out(u16 data)
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

void hp_lj4l_device::blt_data_w(offs_t offset, u16 data, u16 mem_mask)
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

void hp_lj4l_device::main_map(address_map &map)
{
	map(0x000000, 0xffffff).rw(FUNC(hp_lj4l_device::unk_r), FUNC(hp_lj4l_device::unk_w));
	map(0x000000, 0x1fffff).rom().region("maincpu", 0);
	map(0x800000, 0x800001).rw(FUNC(hp_lj4l_device::irqmask_r), FUNC(hp_lj4l_device::irqmask_w));
	map(0x800004, 0x800005).w(FUNC(hp_lj4l_device::irqack_w));
	map(0x820000, 0x820001).rw(FUNC(hp_lj4l_device::pdata_r), FUNC(hp_lj4l_device::pdata_w));
	map(0x820002, 0x820003).r(FUNC(hp_lj4l_device::pstat_r));
	map(0x820006, 0x820007).w(FUNC(hp_lj4l_device::pack_w));
	map(0x82000a, 0x82000b).rw(FUNC(hp_lj4l_device::pen_r), FUNC(hp_lj4l_device::pen_w));
	map(0x82000c, 0x82000d).rw(FUNC(hp_lj4l_device::pext_r), FUNC(hp_lj4l_device::pext_w));
	map(0x840000, 0x840007).w(FUNC(hp_lj4l_device::video_w));
	map(0x84001e, 0x84001f).rw(FUNC(hp_lj4l_device::eng_r), FUNC(hp_lj4l_device::eng_w));
	map(0x850000, 0x850001).rw(FUNC(hp_lj4l_device::leds_r), FUNC(hp_lj4l_device::leds_w));
	map(0x850002, 0x850003).r(FUNC(hp_lj4l_device::button_r));
	map(0x900000, 0x900003).w(FUNC(hp_lj4l_device::blt_pat_w));
	map(0x900004, 0x900007).w(FUNC(hp_lj4l_device::blt_dest_w));
	map(0x900008, 0x90000f).w(FUNC(hp_lj4l_device::blt_ctrl_w));
	map(0x900010, 0x900013).w(FUNC(hp_lj4l_device::blt_data_w));
	map(0x900016, 0x900017).w(FUNC(hp_lj4l_device::blt_cfg_w));
}

void hp_lj4l_device::device_add_mconfig(machine_config &config)
{
	M68000(config, m_maincpu, 31.3076_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &hp_lj4l_device::main_map);

	PDF_PRINTER(config, m_printout);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(HP_LJ4L, device_centronics_peripheral_interface, hp_lj4l_device, "hplj4l", "HP LaserJet 4L")
