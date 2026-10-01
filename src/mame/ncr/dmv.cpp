// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
/***************************************************************************

        NCR Decision Mate V

        04/01/2012 Skeleton driver.

****************************************************************************/


#include "emu.h"

#include "cpu/mcs48/mcs48.h"
#include "cpu/z80/z80.h"
#include "imagedev/floppy.h"
#include "machine/am9517a.h"
#include "dmv_keyb.h"
#include "machine/pit8253.h"
#include "machine/upd765.h"
#include "sound/spkrdev.h"
#include "video/upd7220.h"
#include "emupal.h"

// expansion slots
#include "bus/dmv/dmvbus.h"
#include "bus/dmv/k012.h"
#include "bus/dmv/k210.h"
#include "bus/dmv/k220.h"
#include "bus/dmv/k230.h"
#include "bus/dmv/k233.h"
#include "bus/dmv/k801.h"
#include "bus/dmv/k803.h"
#include "bus/dmv/k806.h"
#include "bus/dmv/ram.h"

#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#include "formats/dmv_dsk.h"
#include "imagedev/snapquik.h"

#include "dmv.lh"


namespace {

class dmv_state : public driver_device
{
public:
	dmv_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_hgdc(*this, "upd7220")
		, m_dmac(*this, "dma8237")
		, m_pit(*this, "pit8253")
		, m_fdc(*this, "i8272")
		, m_floppy0(*this, "i8272:0")
		, m_floppy1(*this, "i8272:1")
		, m_keyboard(*this, "keyboard")
		, m_speaker(*this, "speaker")
		, m_palette(*this, "palette")
		, m_ram(*this, "ram")
		, m_bootrom(*this, "boot")
		, m_chargen(*this, "chargen")
		, m_slot1(*this, "slot1")
		, m_slot2(*this, "slot2")
		, m_slot2a(*this, "slot2a")
		, m_slot3(*this, "slot3")
		, m_slot4(*this, "slot4")
		, m_slot5(*this, "slot5")
		, m_slot6(*this, "slot6")
		, m_slot7(*this, "slot7")
		, m_slot7a(*this, "slot7a")
		, m_leds(*this, "led%u", 1U)
	{ }

	void dmv(machine_config &config);

private:
	void update_halt_line();

	void leds_w(uint8_t data);
	uint8_t gdc_r(offs_t offset);
	void gdc_w(offs_t offset, uint8_t data);
	void dma_hrq_changed(int state);
	void dmac_eop(int state);
	void dmac_dack3(int state);
	void fdc_irq(int state);
	void pit_out0(int state);
	void timint_w(int state);
	void fdd_motor_w(uint8_t data);
	uint8_t sys_status_r();
	void tc_set_w(uint8_t data);
	void switch16_w(uint8_t data);
	uint8_t ramsel_r();
	uint8_t romsel_r();
	void ramsel_w(uint8_t data);
	void romsel_w(uint8_t data);
	uint8_t kb_mcu_port1_r();
	void kb_mcu_port1_w(uint8_t data);
	void kb_mcu_port2_w(uint8_t data);
	void rambank_w(offs_t offset, uint8_t data);
	uint8_t program_r(offs_t offset);
	void program_w(offs_t offset, uint8_t data);
	uint8_t exp_program_r(offs_t offset);
	void exp_program_w(offs_t offset, uint8_t data);
	void thold7_w(int state);

	void update_busint(int slot, int state);
	void busint2_w(int state)    { update_busint(0, state); }
	void busint2a_w(int state)   { update_busint(1, state); }
	void busint3_w(int state)    { update_busint(2, state); }
	void busint4_w(int state)    { update_busint(3, state); }
	void busint5_w(int state)    { update_busint(4, state); }
	void busint6_w(int state)    { update_busint(5, state); }
	void busint7_w(int state)    { update_busint(6, state); }
	void busint7a_w(int state)   { update_busint(7, state); }

	void update_irqs(int slot, int state);
	void irq2_w(int state)       { update_irqs(0, state); }
	void irq2a_w(int state)      { update_irqs(1, state); }
	void irq3_w(int state)       { update_irqs(2, state); }
	void irq4_w(int state)       { update_irqs(3, state); }
	void irq5_w(int state)       { update_irqs(4, state); }
	void irq6_w(int state)       { update_irqs(5, state); }
	void irq7_w(int state)       { update_irqs(6, state); }
	void irq7a_w(int state)      { update_irqs(7, state); }

	static void floppy_formats(format_registration &fr);
	DECLARE_QUICKLOAD_LOAD_MEMBER(quickload_cb);

	uint8_t program_read(int cas, offs_t offset);
	void program_write(int cas, offs_t offset, uint8_t data);

	void ifsel_r(int ifsel, offs_t offset, uint8_t &data);
	void ifsel_w(int ifsel, offs_t offset, uint8_t data);
	uint8_t ifsel0_r(offs_t offset)  { uint8_t data = 0xff;   ifsel_r(0, offset, data);   return data; }
	uint8_t ifsel1_r(offs_t offset)  { uint8_t data = 0xff;   ifsel_r(1, offset, data);   return data; }
	uint8_t ifsel2_r(offs_t offset)  { uint8_t data = 0xff;   ifsel_r(2, offset, data);   return data; }
	uint8_t ifsel3_r(offs_t offset)  { uint8_t data = 0xff;   ifsel_r(3, offset, data);   return data; }
	uint8_t ifsel4_r(offs_t offset)  { uint8_t data = 0xff;   ifsel_r(4, offset, data);   return data; }
	void ifsel0_w(offs_t offset, uint8_t data) { ifsel_w(0, offset, data); }
	void ifsel1_w(offs_t offset, uint8_t data) { ifsel_w(1, offset, data); }
	void ifsel2_w(offs_t offset, uint8_t data) { ifsel_w(2, offset, data); }
	void ifsel3_w(offs_t offset, uint8_t data) { ifsel_w(3, offset, data); }
	void ifsel4_w(offs_t offset, uint8_t data) { ifsel_w(4, offset, data); }

	// green monochrome monitor: off, half intensity, full intensity
	// (half intensity halves the video signal, so it is modelled as half of the
	// full level; the brightness/contrast knobs are left to the MAME sliders)
	static constexpr rgb_t MONO_OFF  = rgb_t(0x00, 0x00, 0x00);
	static constexpr rgb_t MONO_HALF = rgb_t(0x00, 0x80, 0x00);
	static constexpr rgb_t MONO_FULL = rgb_t(0x00, 0xff, 0x00);

	UPD7220_DISPLAY_PIXELS_MEMBER( hgdc_display_pixels );
	UPD7220_DRAW_TEXT_LINE_MEMBER( hgdc_draw_text );

	uint16_t vram_r(offs_t offset);
	void vram_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	void dmv_io(address_map &map) ATTR_COLD;
	void dmv_mem(address_map &map) ATTR_COLD;
	void upd7220_map(address_map &map) ATTR_COLD;

	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<upd7220_device> m_hgdc;
	required_device<am9517a_device> m_dmac;
	required_device<pit8253_device> m_pit;
	required_device<i8272a_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
	required_device<dmv_keyboard_device> m_keyboard;
	required_device<speaker_sound_device> m_speaker;
	// Video RAM behind the uPD7220 (word addressed):
	//   monochrome board: 16K words (32KB), green plane only
	//   colour board:     48K words (96KB), green, red and blue plane
	static constexpr offs_t VRAM_PLANE_WORDS = 0x4000;
	static constexpr offs_t VRAM_WORDS = 3 * VRAM_PLANE_WORDS;
	std::unique_ptr<uint16_t[]> m_video_ram;
	required_device<palette_device> m_palette;
	required_memory_region m_ram;
	required_memory_region m_bootrom;
	required_memory_region m_chargen;

	required_device<dmvcart_slot_device> m_slot1;
	required_device<dmvcart_slot_device> m_slot2;
	required_device<dmvcart_slot_device> m_slot2a;
	required_device<dmvcart_slot_device> m_slot3;
	required_device<dmvcart_slot_device> m_slot4;
	required_device<dmvcart_slot_device> m_slot5;
	required_device<dmvcart_slot_device> m_slot6;
	required_device<dmvcart_slot_device> m_slot7;
	required_device<dmvcart_slot_device> m_slot7a;
	output_finder<8> m_leds;

	bool        m_ramoutdis;
	int         m_switch16;
	int         m_thold7;
	int         m_dma_hrq;
	int         m_ram_bank;
	bool        m_color_mode;
	int         m_eop_line;
	int         m_dack3_line;
	int         m_sd_poll_state;
	int         m_floppy_motor;
	int         m_busint[8];
	int         m_irqs[8];
	uint8_t     m_zoom_latch;   // horizontal display zoom latch on the graphics board (74LS175 B6)
};

void dmv_state::tc_set_w(uint8_t data)
{
	m_fdc->tc_w(true);
}

void dmv_state::switch16_w(uint8_t data)
{
	m_switch16 = !m_switch16;
	update_halt_line();
}

void dmv_state::leds_w(uint8_t data)
{
	/*
	    LEDs    Value       Significance
	    ---------------------------------------
	    None    0xFF        Check complete
	    1+8     0x7E        Sumcheck error
	    2+8     0xBE        GDC error
	    3+8     0xDE        Disk drive error
	    4+8     0xEE        16-bit processor error
	    5+8     0xF6        Keyboard error
	    6+8     0xFA        DMA error
	    7+8     0xFC        Memory error
	    All     0x00        Processor error
	*/

	for(int i=0; i<8; i++)
		m_leds[7-i] = BIT(data, i);
}

uint8_t dmv_state::ramsel_r()
{
	m_ramoutdis = false;
	return 0;
}

uint8_t dmv_state::romsel_r()
{
	m_ramoutdis = true;
	return 0;
}

void dmv_state::ramsel_w(uint8_t data)
{
	m_ramoutdis = false;
}

void dmv_state::romsel_w(uint8_t data)
{
	m_ramoutdis = true;
}

void dmv_state::rambank_w(offs_t offset, uint8_t data)
{
	m_ram_bank = offset;
}

void dmv_state::fdd_motor_w(uint8_t data)
{
	m_pit->write_gate0(1);
	m_pit->write_gate0(0);

	m_floppy_motor = 0;
	if (m_floppy0->get_device()) m_floppy0->get_device()->mon_w(m_floppy_motor);
	if (m_floppy1->get_device()) m_floppy1->get_device()->mon_w(m_floppy_motor);
}

uint8_t dmv_state::sys_status_r()
{
	/*
	    Main system status
	    x--- ---- FDD index
	    -x--- --- IRQ 2
	    --x--- -- IRQ 3
	    ---x--- - IRQ 4
	    ---- x--- FDC interrupt
	    ---- -x-- FDD ready
	    ---- --x- 16-bit CPU available (active low)
	    ---- ---x FDD motor (active low)
	*/
	uint8_t data = 0x00;

	if (m_floppy_motor)
		data |= 0x01;

	// 16-bit CPU
	if (!(m_slot7->av16bit() || m_slot7a->av16bit()))
		data |= 0x02;

	if (m_floppy0->get_device() && !m_floppy0->get_device()->ready_r())
		data |= 0x04;

	if (m_fdc->get_irq())
		data |= 0x08;

	if (m_irqs[3])
		data |= 0x10;   // IRQ 4

	if (m_irqs[2])
		data |= 0x20;   // IRQ 3

	if (m_irqs[0])
		data |= 0x40;   // IRQ 2

	return data;
}

/*
    Graphics board I/O and display zoom latch

    The mainboard decodes BA4-BA7 into GDCSEL/ for the whole range A0h-AFh
    (74LS154 F7, schematic 017-0032012 sheet 8) and passes GDCIOW/,
    GDCIOR/, BA0 and BA1 to the graphics board (J113, sheet 7). On the
    board (017-0032489, sheet 1) BA0 drives A0 of the uPD7220, so the GDC
    shows up at every even/odd port pair of A0h-AFh.

    Sheet 5 ("ZOOM") adds a horizontal display zoom: a 74LS175 (B6) is
    clocked by GDCIOW/ gated with BA1 (74ALS00 B3) and latches D4-D7. Its
    inverted outputs preset a 74LS161 (B5) that counts PIXCLK and reloads
    on carry; carry or VSRLD1 give VSREN, the shift enable of the graphics
    plane shift registers (sheets 2 and 3). The planes therefore shift one
    pixel every (D7..D4) + 1 pixel clocks. The character shift register
    (74ALS299 B11, sheet 5) runs from PIXCLK2* and VSRLD2 and is not stretched.
    The latch has no reset.

    A write to A2h (BA1 = 1, BA0 = 0) thus goes to the GDC as a parameter
    and into the zoom latch at the same time. The mainboard firmware
    C.07.00 (33610) uses exactly that: ZOOM command 46h to A1h, parameter
    to A2h (Z80 code 0445h..049Fh). The upper nibble of the ZOOM parameter
    is the display zoom factor minus one, which matches the latch.
    Software that sends the parameter to A0h only gets the vertical zoom
    of the GDC. The NCR "System Information" calls A2h "write display zoom
    factor" and says display zooming needs change level > 35.
*/
uint8_t dmv_state::gdc_r(offs_t offset)
{
	return m_hgdc->read(offset & 1);
}

void dmv_state::gdc_w(offs_t offset, uint8_t data)
{
	m_hgdc->write(offset & 1, data);

	if (BIT(offset, 1))
		m_zoom_latch = data;
}

UPD7220_DISPLAY_PIXELS_MEMBER( dmv_state::hgdc_display_pixels )
{
	// The GDC passes the address of the word at x; in mixed mode a display line
	// is 40 words, so the line starts x/16 words earlier. With a horizontal zoom
	// factor z, screen pixel px shows memory pixel px/z of that line.
	int const hzoom = (m_zoom_latch >> 4) + 1;
	uint32_t const line = address - (x >> 4);

	for (int xi = 0; xi < 16; xi++)
	{
		if (!bitmap.cliprect().contains(x + xi, y))
			continue;

		int const src = (x + xi) / hzoom;
		uint32_t const word = (line + (src >> 4)) & (VRAM_PLANE_WORDS - 1);
		int const bit = src & 15;

		if (m_color_mode)
		{
			// the display refresh fetches all three planes in parallel
			int const g = BIT(m_video_ram[0 * VRAM_PLANE_WORDS + word], bit) ? 255 : 0;
			int const r = BIT(m_video_ram[1 * VRAM_PLANE_WORDS + word], bit) ? 255 : 0;
			int const b = BIT(m_video_ram[2 * VRAM_PLANE_WORDS + word], bit) ? 255 : 0;
			bitmap.pix(y, x + xi) = rgb_t(r, g, b);
		}
		else
		{
			// 32KB videoram, no attributes in graphics area
			bitmap.pix(y, x + xi) = BIT(m_video_ram[word], bit) ? MONO_FULL : MONO_OFF;
		}
	}
}

UPD7220_DRAW_TEXT_LINE_MEMBER( dmv_state::hgdc_draw_text )
{
	for( int x = 0; x < pitch; x++ )
	{
		uint16_t const cell = vram_r((addr + x) & 0xffff);
		uint8_t tile = cell & 0xff;
		uint8_t attr = cell >> 8;

		rgb_t bg, fg;
		if (m_color_mode)
		{
			// Color board attributes (NCR DMV System Information, Graphics Controller Board):
			//   bit 0 (VRAM bit 8)      half intensity (HI), no effect, see below
			//   bit 1 (VRAM bit 9)      blink enable
			//   bits 2-4 (VRAM 10-12)   foreground red, green, blue
			//   bits 5-7 (VRAM 13-15)   background red, green, blue, negative logic
			// The firmware default is E8h (green on black).
			//
			// Half intensity is prepared on the board but not fitted. Schematic
			// 017-0032489: the attribute latch (74LS377 B16, sheet 5) outputs bit 8
			// as HI, the character/graphics multiplexer (74LS257 A7, sheet 1)
			// passes it in character mode to three 7407 drivers (A6, A4) that would
			// pull VIDEO-RED/, VIDEO-GREEN/ and VIDEO-BLUE/ down through R8, R6 and
			// R5 (470 ohm). Assembly note 3 (sheet 5, assembly 017-0032479-A) lists
			// R1-R3 and R5-R9 as not mounted, so the HI drivers are left open and
			// every colour is shown at full intensity. The System Technical Manual
			// also only names eight colours and blinking for the colour text mode.
			bg = rgb_t(attr & 0x20 ? 0 : 255, attr & 0x40 ? 0 : 255, attr & 0x80 ? 0 : 255);
			fg = rgb_t(attr & 0x04 ? 255 : 0, attr & 0x08 ? 255 : 0, attr & 0x10 ? 255 : 0);
		}
		else
		{
			// Monochrome board attributes (NCR DMV System Information, Graphics Controller Board):
			//   bit 0 (VRAM bit 8)  inverse
			//   bit 1 (VRAM bit 9)  blink enable
			//   bit 2 (VRAM bit 10) half intensity
			//   bits 3-7            not used (firmware writes E8h, i.e. full intensity)
			// Half intensity is applied to whatever is lit in the cell, so an inverse
			// half-intensity cell shows dark characters on a dimmed background.
			rgb_t const lit = BIT(attr, 2) ? MONO_HALF : MONO_FULL;
			bg = BIT(attr, 0) ? lit : MONO_OFF;
			fg = BIT(attr, 0) ? MONO_OFF : lit;
		}

		for( int yi = 0; yi < lr; yi++)
		{
			uint8_t tile_data = m_chargen->base()[(tile*16+yi) & 0x7ff];

			// blink attribute, timed by the GDC attribute blink output (A16)
			if((attr & 2) && !m_hgdc->attr_blink_on())
				tile_data = 0;

			// Cursor: the GDC only asserts its cursor output (A17) on the raster
			// lines CTOP..CBOT set with CCHAR, blinking according to SC and BR.
			// Both mainboard firmwares program CTOP = CBOT = 14 with blinking
			// enabled (CCHAR 8F CE 72), i.e. a one line blinking underline.
			// Note that upd7220_device passes CTOP in cursor_bot and CBOT in
			// cursor_top.
			if(cursor_on && cursor_addr == addr+x && yi >= cursor_bot && yi <= cursor_top && m_hgdc->cursor_blink_on())
				tile_data^=0xff;

			for( int xi = 0; xi < 8; xi++)
			{
				int pen = (tile_data >> xi) & 1 ? 1 : 0;

				int res_x = x * 8 + xi;
				int res_y = y + yi;

				if(!m_screen->visible_area().contains(res_x, res_y))
					continue;

				if(yi >= 16) { pen = 0; }

				bitmap.pix(res_y, res_x) = pen ? fg : bg;
			}
		}
	}
}

/***********************************************************

    Quickload

    This loads a .COM file to address 0x100 then jumps
    there. Sometimes .COM has been renamed to .CPM to
    prevent windows going ballistic. These can be loaded
    as well.

************************************************************/

QUICKLOAD_LOAD_MEMBER(dmv_state::quickload_cb)
{
	// Avoid loading a program if CP/M-80 is not in memory
	if ((m_ram->base()[0] != 0xc3) || (m_ram->base()[5] != 0xc3))
		return std::make_pair(image_error::UNSUPPORTED, "CP/M must already be running");

	const int mem_avail = 256 * m_ram->base()[7] + m_ram->base()[6] - 512;
	if (mem_avail < image.length())
		return std::make_pair(image_error::UNSPECIFIED, "Insufficient memory available");

	// Load image to the TPA (Transient Program Area)
	uint16_t quickload_size = image.length();
	for (uint16_t i = 0; i < quickload_size; i++)
	{
		uint8_t data;
		if (image.fread(&data, 1) != 1)
			return std::make_pair(image_error::UNSPECIFIED, "Problem reading the image at offset " + std::to_string(i));
		m_ram->base()[i + 0x100] = data;
	}

	m_ram->base()[0x80] = m_ram->base()[0x81] = 0; // clear out command tail

	m_maincpu->set_state_int(Z80_SP, mem_avail + 384); // put the stack a bit before BDOS
	m_maincpu->set_pc(0x100); // start program

	return std::make_pair(std::error_condition(), std::string());
}

static void dmv_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
	device.option_add("525qd", FLOPPY_525_QD);
}


void dmv_state::ifsel_r(int ifsel, offs_t offset, uint8_t &data)
{
	m_slot2->io_read(ifsel, offset, data);
	m_slot2a->io_read(ifsel, offset, data);
	m_slot3->io_read(ifsel, offset, data);
	m_slot4->io_read(ifsel, offset, data);
	m_slot5->io_read(ifsel, offset, data);
	m_slot6->io_read(ifsel, offset, data);
	m_slot7->io_read(ifsel, offset, data);
	m_slot7a->io_read(ifsel, offset, data);
}

void dmv_state::ifsel_w(int ifsel, offs_t offset, uint8_t data)
{
	m_slot2->io_write(ifsel, offset, data);
	m_slot2a->io_write(ifsel, offset, data);
	m_slot3->io_write(ifsel, offset, data);
	m_slot4->io_write(ifsel, offset, data);
	m_slot5->io_write(ifsel, offset, data);
	m_slot6->io_write(ifsel, offset, data);
	m_slot7->io_write(ifsel, offset, data);
	m_slot7a->io_write(ifsel, offset, data);
}

void dmv_state::exp_program_w(offs_t offset, uint8_t data)
{
	program_write((offset >> 16) & 0x07, offset, data);
}

uint8_t dmv_state::exp_program_r(offs_t offset)
{
	return program_read((offset >> 16) & 0x07, offset);
}

void dmv_state::program_w(offs_t offset, uint8_t data)
{
	program_write(m_ram_bank, offset, data);
}

uint8_t dmv_state::program_r(offs_t offset)
{
	return program_read(m_ram_bank, offset);
}

void dmv_state::thold7_w(int state)
{
	if (m_thold7 != state)
	{
		m_thold7 = state;
		update_halt_line();
	}
}

void dmv_state::update_busint(int slot, int state)
{
	m_busint[slot] = state;

	int new_state = CLEAR_LINE;
	for (auto & elem : m_busint)
		if (elem != CLEAR_LINE)
		{
			new_state = ASSERT_LINE;
			break;
		}

	m_slot7a->busint_w(new_state);
	m_slot7->busint_w(new_state);
	m_maincpu->set_input_line(0, new_state);
}

void dmv_state::update_irqs(int slot, int state)
{
	m_irqs[slot] = state;

	switch(slot)
	{
	case 0: // slot 2
		m_slot7->irq2_w(state);
		m_slot7a->irq2_w(state);
		break;
	case 1: // slot 2a
		m_slot7->irq2a_w(state);
		m_slot7a->irq2a_w(state);
		break;
	case 2: // slot 3
		m_slot7->irq3_w(state);
		m_slot7a->irq3_w(state);
		break;
	case 3: // slot 4
		m_slot7->irq4_w(state);
		m_slot7a->irq4_w(state);
		break;
	case 4: // slot 5
		m_slot7->irq5_w(state);
		m_slot7a->irq5_w(state);
		break;
	case 5: // slot 6
		m_slot7->irq6_w(state);
		m_slot7a->irq6_w(state);
		break;
	}
}

void dmv_state::program_write(int cas, offs_t offset, uint8_t data)
{
	bool tramd = false;
	dmvcart_slot_device *slots[] = { m_slot2, m_slot2a, m_slot3, m_slot4, m_slot5, m_slot6, m_slot7, m_slot7a };
	for(int i=0; i<8 && !tramd; i++)
		tramd = slots[i]->write(offset, data);

	if (!tramd)
	{
		if (cas == 0)
			m_ram->base()[offset & 0xffff] = data;
		else
			m_slot1->ram_write(cas, offset & 0xffff, data);
	}
}

uint8_t dmv_state::program_read(int cas, offs_t offset)
{
	uint8_t data = 0xff;
	if (m_ramoutdis && offset < 0x2000)
	{
		data = m_bootrom->base()[offset];
	}
	else
	{
		bool tramd = false;
		dmvcart_slot_device *slots[] = { m_slot2, m_slot2a, m_slot3, m_slot4, m_slot5, m_slot6, m_slot7, m_slot7a };
		for(int i=0; i<8 && !tramd; i++)
			tramd = slots[i]->read(offset, data);

		if (!tramd)
		{
			if (cas == 0)
				data = m_ram->base()[offset & 0xffff];
			else
				m_slot1->ram_read(cas, offset & 0xffff, data);
		}
	}

	return data;
}

void dmv_state::dmv_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0xffff).rw(FUNC(dmv_state::program_r), FUNC(dmv_state::program_w));
}

void dmv_state::dmv_io(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x00, 0x00).w(FUNC(dmv_state::leds_w));
	map(0x10, 0x10).rw(FUNC(dmv_state::ramsel_r), FUNC(dmv_state::ramsel_w));
	map(0x11, 0x11).rw(FUNC(dmv_state::romsel_r), FUNC(dmv_state::romsel_w));
	map(0x12, 0x12).w(FUNC(dmv_state::tc_set_w));
	map(0x13, 0x13).r(FUNC(dmv_state::sys_status_r));
	map(0x14, 0x14).w(FUNC(dmv_state::fdd_motor_w));
	map(0x20, 0x2f).rw(m_dmac, FUNC(am9517a_device::read), FUNC(am9517a_device::write));
	map(0x40, 0x41).rw("kb_ctrl_mcu", FUNC(upi41_cpu_device::upi41_master_r), FUNC(upi41_cpu_device::upi41_master_w));
	map(0x50, 0x51).m(m_fdc, FUNC(i8272a_device::map));
	map(0x80, 0x83).rw(m_pit, FUNC(pit8253_device::read), FUNC(pit8253_device::write));
	map(0xa0, 0xaf).rw(FUNC(dmv_state::gdc_r), FUNC(dmv_state::gdc_w));   // GDC on BA0, zoom latch on BA1
	map(0xd0, 0xd7).w(FUNC(dmv_state::switch16_w));
	map(0xe0, 0xe7).w(FUNC(dmv_state::rambank_w));

	map(0x60, 0x6f).rw(FUNC(dmv_state::ifsel0_r), FUNC(dmv_state::ifsel0_w));
	map(0x70, 0x7f).rw(FUNC(dmv_state::ifsel1_r), FUNC(dmv_state::ifsel1_w));
	map(0x30, 0x3f).rw(FUNC(dmv_state::ifsel2_r), FUNC(dmv_state::ifsel2_w));
	map(0xb0, 0xbf).rw(FUNC(dmv_state::ifsel3_r), FUNC(dmv_state::ifsel3_w));
	map(0xc0, 0xcf).rw(FUNC(dmv_state::ifsel4_r), FUNC(dmv_state::ifsel4_w));
}

uint8_t dmv_state::kb_mcu_port1_r()
{
	return !(m_keyboard->sd_poll_r() & !m_sd_poll_state);
}

void dmv_state::kb_mcu_port1_w(uint8_t data)
{
	m_sd_poll_state = BIT(data, 1);
	m_keyboard->sd_poll_w(!m_sd_poll_state);
}

void dmv_state::kb_mcu_port2_w(uint8_t data)
{
	m_speaker->level_w(BIT(data, 0));
	m_slot7a->keyint_w(BIT(data, 4));
	m_slot7->keyint_w(BIT(data, 4));
}

/*
    Video RAM decoding

    The CPU has no direct access to the video RAM; everything goes through
    the uPD7220 (ports A0h/A1h) or DMA channel 2. The GDC sees a word
    addressed space of 64K words.

    Monochrome board: one 32KB plane (16K words). Only A0..A13 are decoded,
    so the plane repeats every 4000h words. Software that probes for the
    colour planes finds its own green data mirrored at 4000h and 8000h.

    Colour board: three 32KB planes, selected by A14/A15 when the GDC
    accesses memory for drawing or the CPU:
        0000h-3FFFh  green
        4000h-7FFFh  red
        8000h-BFFFh  blue
        C000h-FFFFh  no plane fitted, reads return FFFFh, writes are lost
    For the display refresh all three planes are fetched in parallel
    (see hgdc_display_pixels); character mode only uses the green plane.

    TODO: the mono mirroring and the open bus value at C000h-FFFFh are
    assumptions until verified against the graphics board schematics.
*/

uint16_t dmv_state::vram_r(offs_t offset)
{
	if (!m_color_mode)
		return m_video_ram[offset & (VRAM_PLANE_WORDS - 1)];

	if (offset < VRAM_WORDS)
		return m_video_ram[offset];

	return 0xffff;
}

void dmv_state::vram_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (!m_color_mode)
		COMBINE_DATA(&m_video_ram[offset & (VRAM_PLANE_WORDS - 1)]);
	else if (offset < VRAM_WORDS)
		COMBINE_DATA(&m_video_ram[offset]);
}

void dmv_state::upd7220_map(address_map &map)
{
	map.global_mask(0xffff);
	map(0x0000, 0xffff).rw(FUNC(dmv_state::vram_r), FUNC(dmv_state::vram_w));
}

/* Input ports */
INPUT_PORTS_START( dmv )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Video Board" )
	PORT_CONFSETTING( 0x00, "Monochrome" )
	PORT_CONFSETTING( 0x01, "Color" )
INPUT_PORTS_END

void dmv_state::machine_start()
{
	// the colour board's 96KB are always allocated; in monochrome mode
	// only the first plane is used (see vram_r/vram_w)
	m_video_ram = std::make_unique<uint16_t[]>(VRAM_WORDS);
	std::fill_n(m_video_ram.get(), VRAM_WORDS, 0);

	// register for state saving
	save_item(NAME(m_ramoutdis));
	save_item(NAME(m_switch16));
	save_item(NAME(m_thold7));
	save_item(NAME(m_dma_hrq));
	save_item(NAME(m_ram_bank));
	save_item(NAME(m_color_mode));
	save_item(NAME(m_eop_line));
	save_item(NAME(m_dack3_line));
	save_item(NAME(m_sd_poll_state));
	save_item(NAME(m_floppy_motor));
	save_item(NAME(m_busint));
	save_item(NAME(m_irqs));
	save_item(NAME(m_zoom_latch));
	save_pointer(NAME(m_ram->base()), m_ram->bytes());
	save_pointer(NAME(m_video_ram), VRAM_WORDS);
}

void dmv_state::machine_reset()
{
	m_color_mode = ioport("CONFIG")->read() & 0x01;

	m_ramoutdis = true;
	m_ram_bank = 0;
	m_eop_line = 0;
	m_dack3_line = 0;
	m_sd_poll_state = 0;
	m_floppy_motor = 1;
	m_switch16 = 0;
	m_thold7 = 0;
	m_dma_hrq = 0;
	memset(m_busint, 0, sizeof(m_busint));
	memset(m_irqs, 0, sizeof(m_irqs));
	m_zoom_latch = 0;   // not reset in hardware, the firmware writes 0 at boot

	update_halt_line();
}

void dmv_state::update_halt_line()
{
	m_slot7->hold_w(m_dma_hrq);
	m_slot7->switch16_w(m_switch16);
	m_slot7a->hold_w(m_dma_hrq);
	m_slot7a->switch16_w(m_switch16);

	m_maincpu->set_input_line(INPUT_LINE_HALT, (m_thold7 || m_switch16 || m_dma_hrq) ? ASSERT_LINE : CLEAR_LINE);
}

/* F4 Character Displayer */
static const gfx_layout dmv_charlayout =
{
	8, 16,                  /* 8 x 16 characters */
	128,                    /* 128 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 7, 6, 5, 4, 3, 2, 1, 0 },
	/* y offsets */
	{ STEP16(0,8) },
	8*16                    /* every char takes 16 bytes */
};

static GFXDECODE_START( gfx_dmv )
	GFXDECODE_ENTRY("chargen", 0x0000, dmv_charlayout, 0, 1)
GFXDECODE_END


//------------------------------------------------------------------------------------
//   I8237
//------------------------------------------------------------------------------------

void dmv_state::dma_hrq_changed(int state)
{
	m_dma_hrq = state;
	update_halt_line();

	// Assert HLDA
	m_dmac->hack_w(state);
}

void dmv_state::dmac_eop(int state)
{
	if (!(m_dack3_line || m_eop_line) && (m_dack3_line || state))
		m_fdc->tc_w(true);

	m_eop_line = state;
}

void dmv_state::dmac_dack3(int state)
{
	if (!(m_dack3_line || m_eop_line) && (state || m_eop_line))
		m_fdc->tc_w(true);

	m_dack3_line = state;
}

void dmv_state::pit_out0(int state)
{
	if (!state)
	{
		m_floppy_motor = 1;
		if (m_floppy0->get_device()) m_floppy0->get_device()->mon_w(m_floppy_motor);
		if (m_floppy1->get_device()) m_floppy1->get_device()->mon_w(m_floppy_motor);
	}
}

void dmv_state::timint_w(int state)
{
	m_slot7a->timint_w(state);
	m_slot7->timint_w(state);
}

void dmv_state::fdc_irq(int state)
{
	m_slot7a->flexint_w(state);
	m_slot7->flexint_w(state);

	if (state)
		m_fdc->tc_w(false);
}


void dmv_state::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_DMV_FORMAT);
}


static void dmv_slot1(device_slot_interface &device)
{
	device.option_add("k200", DMV_K200);        // K200 64K RAM expansion
	device.option_add("k202", DMV_K202);        // K202 192K RAM expansion
	device.option_add("k208", DMV_K208);        // K208 448K RAM expansion
}

static void dmv_slot2_6(device_slot_interface &device)
{
	device.option_add("k210", DMV_K210);        // K210 Centronics
	device.option_add("k211", DMV_K211);        // K211 RS-232 Communications Interface
	device.option_add("k212", DMV_K212);        // K212 RS-232 Printer Interface
	device.option_add("k213", DMV_K213);        // K213 RS-232 Plotter Interface
	device.option_add("k233", DMV_K233);        // K233 16K Shared RAM
	device.option_add("k801", DMV_K801);        // K801 RS-232 Switchable Interface
	device.option_add("k803", DMV_K803);        // K803 RTC module
	device.option_add("k806", DMV_K806);        // K806 Mouse module
	device.option_add("c3282", DMV_C3282);      // C3282 External HD Interface
}

static void dmv_slot7(device_slot_interface &device)
{
	device.option_add("k220", DMV_K220);        // K220 Diagnostic Module
	device.option_add("k231", DMV_K231);        // K231 External 8088 module without interrupt controller
	device.option_add("k234", DMV_K234);        // K234 External 68008 module
}


static void dmv_slot2a(device_slot_interface &device)
{
	device.option_add("k012", DMV_K012);        // K012 Internal HD Interface
}

static void dmv_slot7a(device_slot_interface &device)
{
	device.option_add("k230", DMV_K230);        // K230 Internal 8088 module without interrupt controller
	device.option_add("k235", DMV_K235);        // K235 Internal 8088 module with interrupt controller
}

void dmv_state::dmv(machine_config &config)
{
	/* basic machine hardware */
	Z80(config, m_maincpu, XTAL(24'000'000) / 6);
	m_maincpu->set_addrmap(AS_PROGRAM, &dmv_state::dmv_mem);
	m_maincpu->set_addrmap(AS_IO, &dmv_state::dmv_io);

	i8741a_device &kbmcu(I8741A(config, "kb_ctrl_mcu", XTAL(6'000'000)));
	kbmcu.p1_in_cb().set(FUNC(dmv_state::kb_mcu_port1_r)); // bit 0 data from kb
	kbmcu.p1_out_cb().set(FUNC(dmv_state::kb_mcu_port1_w)); // bit 1 data to kb
	kbmcu.p2_out_cb().set(FUNC(dmv_state::kb_mcu_port2_w));

	config.set_perfect_quantum(m_maincpu);

	DMV_KEYBOARD(config, m_keyboard);

	/* video hardware */
	// Video timing
	// ------------
	// The upd7220 recomputes the screen geometry and refresh rate from the RESET
	// parameters as soon as the firmware programs it as VSYNC master (VSYNC 6F),
	// so the 50 Hz below only applies until the mainboard firmware has run.
	//
	// The firmware sends RESET (00) with the 8 parameter bytes stored at 04D0h
	// in the mainboard ROM. The mono ROM 33609 (M.07.00) and the color ROM 33610
	// (C.07.00) are identical except for the version letter at 0FF9h, the
	// checksum at 1FFFh and these parameters:
	//
	//                      mono (33609)                color (33610)
	//   RESET params       14 4E 82 09 12 06 90 19     14 4E 06 15 09 0A 90 3D
	//   mode               14h: mixed mode, DRAM refresh, drawing in retrace only
	//   AW / HS/HFP/HBP    80 / 3/3/19 = 105 chars     80 / 7/6/10 = 103 chars
	//   AL / VS/VFP/VBP    400 / 12/6/6 = 424 lines    400 / 8/10/15 = 433 lines
	//   board crystal      20.000 MHz                  24.000 MHz
	//
	// Text characters are 8 pixels wide and the board crystal is taken as the
	// pixel clock. upd7220_device counts one 8 pixel character per input clock
	// in mixed mode (refresh = pixel total / (clock * 8)), so the device is
	// clocked at pixel clock / 8 here. This is an emulation convention: the
	// real 2xWCLK needs two clocks per display cycle (NEC uPD7220 GDC Design
	// Manual, 2.2). In mixed mode a display cycle shows one 8 pixel character
	// (PRAM WD = 0), and in graphics areas the address only advances every
	// other display cycle (data sheet, PRAM IM bit), so 8 pixels per display
	// cycle throughout and 2xWCLK = pixel clock / 4 on the board. Only the
	// resulting video timing is modelled, and it does not depend on this.
	// Both boards carry a D7220D-1, specified for 2xWCLK up to 5 MHz
	// (tCLK >= 200 ns).
	// Mono: 20 MHz pixel clock (2xWCLK 5 MHz), 840 x 424 pixels total,
	// 23.81 kHz horizontal, 56.15 Hz vertical.
	// Color: with the mono clock used here the color timing results in
	// 824 x 433, 24.27 kHz, 56.06 Hz. Using the 24 MHz crystal directly as
	// pixel clock would need 2xWCLK = 6 MHz, beyond the D7220D-1 rating, so
	// the color board must derive its clocks differently. This is unverified
	// (needs a measurement of HSYNC, P113 pin 8c), so the color board still
	// uses the mono clock.
	//
	// CCHAR (4B) is programmed with 8F CE 72 by both firmwares: 16 raster lines
	// per character row, cursor displayed, blinking, blink rate BR = 11, cursor
	// on line 14 only. At 56.15 Hz this gives a cursor blinking 22 frames on,
	// 22 frames off (0.78 s period) and an attribute blink of 66 frames on,
	// 22 frames off (1.57 s period). In mixed mode the GDC outputs the cursor
	// indication on A17 and the attribute blink timing on A16 during active
	// display (Design Manual 2.4.2, data sheet pin utilization). That the DMV
	// boards use A16 for the blink attribute is assumed, not verified.
	// PRAM (70) is programmed with 00 00 F0 3F: one character area (IM = 0,
	// WD = 0) starting at address 0.
	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(50);
	m_screen->set_screen_update("upd7220", FUNC(upd7220_device::screen_update));
	m_screen->set_size(640, 400);
	m_screen->set_visarea(0, 640-1, 0, 400-1);

	GFXDECODE(config, "gfxdecode", m_palette, gfx_dmv);
	PALETTE(config, m_palette, palette_device::RGB_3BIT);
	config.set_default_layout(layout_dmv);

	// devices
	UPD7220(config, m_hgdc, XTAL(20'000'000)/8); // mono board: 20 MHz pixel clock / 8 (MAME convention, see video timing above)
	m_hgdc->set_addrmap(0, &dmv_state::upd7220_map);
	m_hgdc->set_display_pixels(FUNC(dmv_state::hgdc_display_pixels));
	m_hgdc->set_draw_text(FUNC(dmv_state::hgdc_draw_text));

	AM9517A(config, m_dmac, 4_MHz_XTAL);
	m_dmac->out_hreq_callback().set(FUNC(dmv_state::dma_hrq_changed));
	m_dmac->out_eop_callback().set(FUNC(dmv_state::dmac_eop));
	m_dmac->in_memr_callback().set(FUNC(dmv_state::program_r));
	m_dmac->out_memw_callback().set(FUNC(dmv_state::program_w));
	m_dmac->in_ior_callback<0>().set([this] () { logerror("Read DMA CH1"); return u8(0); });
	m_dmac->out_iow_callback<0>().set([this] (u8 data) { logerror("Write DMA CH1 %02X", data); });
	m_dmac->in_ior_callback<1>().set([this] () { logerror("Read DMA CH2"); return u8(0); });
	m_dmac->out_iow_callback<1>().set([this] (u8 data) { logerror("Write DMA CH2 %02X", data); });
	m_dmac->in_ior_callback<2>().set(m_hgdc, FUNC(upd7220_device::dack_r));
	m_dmac->out_iow_callback<2>().set(m_hgdc, FUNC(upd7220_device::dack_w));
	m_dmac->in_ior_callback<3>().set(m_fdc, FUNC(i8272a_device::dma_r));
	m_dmac->out_iow_callback<3>().set(m_fdc, FUNC(i8272a_device::dma_w));
	m_dmac->out_dack_callback<3>().set(FUNC(dmv_state::dmac_dack3));

	I8272A(config, m_fdc, 8'000'000, true);
	m_fdc->intrq_wr_callback().set(FUNC(dmv_state::fdc_irq));
	m_fdc->drq_wr_callback().set(m_dmac, FUNC(am9517a_device::dreq3_w));
	FLOPPY_CONNECTOR(config, "i8272:0", dmv_floppies, "525dd", dmv_state::floppy_formats);
	FLOPPY_CONNECTOR(config, "i8272:1", dmv_floppies, "525dd", dmv_state::floppy_formats);

	PIT8253(config, m_pit);
	m_pit->set_clk<0>(50);
	m_pit->out_handler<0>().set(FUNC(dmv_state::pit_out0));
	m_pit->set_clk<2>(XTAL(24'000'000) / 3 / 16);
	m_pit->out_handler<2>().set(FUNC(dmv_state::timint_w));

	/* sound hardware */
	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.50);

	DMVCART_SLOT(config, m_slot1, dmv_slot1, nullptr);
	m_slot1->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot1->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot2, dmv_slot2_6, nullptr);
	m_slot2->out_int().set(FUNC(dmv_state::busint2_w));
	m_slot2->out_irq().set(FUNC(dmv_state::irq2_w));
	m_slot2->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot2->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot2a, dmv_slot2a, nullptr);
	m_slot2a->out_int().set(FUNC(dmv_state::busint2a_w));
	m_slot2a->out_irq().set(FUNC(dmv_state::irq2a_w));
	m_slot2a->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot2a->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot3, dmv_slot2_6, nullptr);
	m_slot3->out_int().set(FUNC(dmv_state::busint3_w));
	m_slot3->out_irq().set(FUNC(dmv_state::irq3_w));
	m_slot3->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot3->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot4, dmv_slot2_6, nullptr);
	m_slot4->out_int().set(FUNC(dmv_state::busint4_w));
	m_slot4->out_irq().set(FUNC(dmv_state::irq4_w));
	m_slot4->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot4->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot5, dmv_slot2_6, nullptr);
	m_slot5->out_int().set(FUNC(dmv_state::busint5_w));
	m_slot5->out_irq().set(FUNC(dmv_state::irq5_w));
	m_slot5->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot5->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot6, dmv_slot2_6, nullptr);
	m_slot6->out_int().set(FUNC(dmv_state::busint6_w));
	m_slot6->out_irq().set(FUNC(dmv_state::irq6_w));
	m_slot6->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot6->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot7, dmv_slot7, nullptr);
	m_slot7->prog_read().set(FUNC(dmv_state::exp_program_r));
	m_slot7->prog_write().set(FUNC(dmv_state::exp_program_w));
	m_slot7->out_thold().set(FUNC(dmv_state::thold7_w));
	m_slot7->out_int().set(FUNC(dmv_state::busint7_w));
	m_slot7->out_irq().set(FUNC(dmv_state::irq7_w));
	m_slot7->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot7->set_iospace(m_maincpu, AS_IO);

	DMVCART_SLOT(config, m_slot7a, dmv_slot7a, "k230");
	m_slot7a->prog_read().set(FUNC(dmv_state::exp_program_r));
	m_slot7a->prog_write().set(FUNC(dmv_state::exp_program_w));
	m_slot7a->out_thold().set(FUNC(dmv_state::thold7_w));
	m_slot7a->out_int().set(FUNC(dmv_state::busint7a_w));
	m_slot7a->out_irq().set(FUNC(dmv_state::irq7a_w));
	m_slot7a->set_memspace(m_maincpu, AS_PROGRAM);
	m_slot7a->set_iospace(m_maincpu, AS_IO);

	SOFTWARE_LIST(config, "flop_list").set_original("dmv");

	QUICKLOAD(config, "quickload", "com,cpm", attotime::from_seconds(3)).set_load_callback(FUNC(dmv_state::quickload_cb));
}

/* ROM definition */
ROM_START( dmv )
	ROM_REGION( 0x2000, "boot", 0 )
	ROM_SYSTEM_BIOS(0, "c07", "C.07.00")    // ROM bears the handwritten note "Color 7.0", this is from the machine that originally had Color, 68K and internal 8088
	ROM_SYSTEM_BIOS(1, "c06", "C.06.00")    // Color machine with older BIOS revision
	ROM_SYSTEM_BIOS(2, "m07", "M.07.00")    // Mono machine with internal 8088 and internal HD
	ROM_SYSTEM_BIOS(3, "m06", "M.06.00")    // Mono machine
	ROM_SYSTEM_BIOS(4, "m05", "M.05.00")    // Mono machine, marked "updated"

	ROMX_LOAD( "dmv_mb_rom_33610.bin", 0x00000,    0x02000,    CRC(bf25f3f0) SHA1(0c7dd37704db4799e340cc836f887cd543e5c964), ROM_BIOS(0) )
	ROMX_LOAD( "dmv_mb_rom_32838.bin", 0x00000,    0x02000,    CRC(d5ceb559) SHA1(e3a05e43aa1b09f0a857b8d54b00bcd321215bf6), ROM_BIOS(1) )
	ROMX_LOAD( "dmv_mb_rom_33609.bin", 0x00000,    0x02000,    CRC(120951b6) SHA1(57bef9cc6379dea5730bc1477e8896508e00a349), ROM_BIOS(2) )
	ROMX_LOAD( "dmv_mb_rom_32676.bin", 0x00000,    0x02000,    CRC(7796519e) SHA1(8d5dd9c1e66c96fcca271b6f673d6a0e784acb33), ROM_BIOS(3) )
	ROMX_LOAD( "dmv_mb_rom_32664.bin", 0x00000,    0x02000,    CRC(6624610e) SHA1(e9226be897d2c5f875784ab77dad8807f14c7714), ROM_BIOS(4) )

	ROM_REGION(0x400, "kb_ctrl_mcu", 0)
	ROMX_LOAD( "dmv_mb_8741_32678.bin",    0x00000,    0x00400,    CRC(50d1dc4c) SHA1(2c8251d6c8df9f507e11bf920869657f4d074db1), ROM_BIOS(0) )
	ROMX_LOAD( "dmv_mb_8741_32678.bin",    0x00000,    0x00400,    CRC(50d1dc4c) SHA1(2c8251d6c8df9f507e11bf920869657f4d074db1), ROM_BIOS(1) )
	ROMX_LOAD( "dmv_mb_8741_32678.bin",    0x00000,    0x00400,    CRC(50d1dc4c) SHA1(2c8251d6c8df9f507e11bf920869657f4d074db1), ROM_BIOS(2) )
	ROMX_LOAD( "dmv_mb_8741_32678.bin",    0x00000,    0x00400,    CRC(50d1dc4c) SHA1(2c8251d6c8df9f507e11bf920869657f4d074db1), ROM_BIOS(3) )
	ROMX_LOAD( "dmv_mb_8741_32121.bin",    0x00000,    0x00400,    CRC(a03af298) SHA1(144cba41294c46f5ca79b7ad8ced0e4408168775), ROM_BIOS(4) )

	ROM_REGION(0x800, "chargen", 0)
	ROM_LOAD( "76161.bin",    0x00000,    0x00800,  CRC(6e4df4f9) SHA1(20ff4fc48e55eaf5131f6573fff93e7f97d2f45d)) // same for both color and monochrome board

	ROM_REGION(0x10000, "ram", ROMREGION_ERASE) // 64K RAM on mainboard
ROM_END

} // anonymous namespace


/* Driver */

//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS      INIT        COMPANY  FULLNAME           FLAGS
COMP( 1984, dmv,  0,      0,      dmv,     dmv,   dmv_state, empty_init, "NCR",   "Decision Mate V", MACHINE_SUPPORTS_SAVE)
