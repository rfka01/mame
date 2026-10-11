// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Olivetti Quaderno - 640x400 reflective LCD controller ("DCGA")

    Part of the Pegasus LT7-441017 gate array.  Register compatible with
    CGA plus the Olivetti M24 / AT&T 6300 640x400 mode (mode register 2
    at 3de), driving a 7.5" reflective LCD with 8 grey levels.

    Peculiarities (from software, see the driver header for sources):
    - 32 KB of video RAM at B8000-BFFFF.  640x400x1 uses all of it.
    - In text mode there is no character ROM: the 8x16 font lives in the
      upper 8 KB of video RAM (B800:6000).  QUADERNO.COM (Polish code page
      loader, 1994) writes glyph rows 0-7 to BE00:c*8+r and rows 8-15 to
      BE00:0800+c*8+r-8, and a second (bold) font at BE00:1000/1800.  The
      tools 8.COM/80.COM patch single glyph rows at B800:61C5 ('8') and
      B800:6186 ('0'), confirming the layout.  Graphics modes overwrite
      the font, the BIOS reloads it from ROM on a text mode set.
    - CRTC index register is 8 bits wide; indices DA (grey levels) and
      DF (bit 7: inverse display) are used by the BIOS configuration
      functions (BIOS vector 0x64, functions 08 and 0C).
    - DF bit 1 is the font load mode of BIOS 1.88: while it is set,
      B800:0000-1FFF addresses the font area B800:6000-7FFF.  The BIOS
      copies the code page font from the 512K ROM this way.
    - BIOS 1.88 also programs extended registers D3 D4 D7 D8 D9 DB DD DE
      (meaning unknown, probably LCD timing).

    Assumptions (unverified, no ROM dump available yet):
    - attribute bit 3 (intensity) selects the second (bold) font when
      DFh bit 6 is set (BIOS function 64h/28h, "Fettschrift" in PA setup)
    - grey level mapping of the 16 CGA colours
    - EXT_GRAY bit 0..4 != 0 means "grey levels off" (BIOS writes 5f/40)
    - cursor start/end values <= 7 programmed for an 8 line cell are
      doubled by the hardware (the M24 does this with a PROM)
    - LCD timing; the panel is treated as fixed 640x400 at ~60 Hz

***************************************************************************/

#include "emu.h"
#include "quaderno_lcd.h"

#define LOG_REGS    (1U << 1)
#define VERBOSE     (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(QUADERNO_LCD, quaderno_lcd_device, "quaderno_lcd", "Olivetti Quaderno LCD controller")

quaderno_lcd_device::quaderno_lcd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUADERNO_LCD, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, m_palette(*this, "palette")
	, m_screen(*this, "screen")
	, m_index(0)
	, m_mode(0)
	, m_color(0)
	, m_mode2(0)
	, m_frame(0)
{
	std::fill(std::begin(m_creg), std::end(m_creg), 0);
}

void quaderno_lcd_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen).set_lcd();
	m_screen->set_raw(28.636363_MHz_XTAL, 912, 0, 640, 525, 0, 400);
	m_screen->set_screen_update(FUNC(quaderno_lcd_device::screen_update));
	m_screen->set_palette(m_palette);

	PALETTE(config, m_palette, FUNC(quaderno_lcd_device::lcd_palette), 8);
}

void quaderno_lcd_device::lcd_palette(palette_device &palette) const
{
	// reflective STN: level 0 = background (light), level 7 = fully driven (dark)
	constexpr rgb_t light(0xc4, 0xcc, 0xb0);
	constexpr rgb_t dark(0x24, 0x2c, 0x34);
	for (int i = 0; i < 8; i++)
	{
		palette.set_pen_color(i, rgb_t(
				light.r() + (dark.r() - light.r()) * i / 7,
				light.g() + (dark.g() - light.g()) * i / 7,
				light.b() + (dark.b() - light.b()) * i / 7));
	}
}

void quaderno_lcd_device::device_start()
{
	m_vram = std::make_unique<uint8_t[]>(0x8000);
	std::fill_n(m_vram.get(), 0x8000, 0);

	save_pointer(NAME(m_vram), 0x8000);
	save_item(NAME(m_index));
	save_item(NAME(m_creg));
	save_item(NAME(m_mode));
	save_item(NAME(m_color));
	save_item(NAME(m_mode2));
	save_item(NAME(m_frame));
}

void quaderno_lcd_device::device_reset()
{
	m_mode = 0;
	m_mode2 = 0;
	m_index = 0;
}

offs_t quaderno_lcd_device::vram_addr(offs_t offset) const
{
	offset &= 0x7fff;
	// font load mode (EXT_FONTLOAD bit 1): B800:0000-1FFF addresses the font area
	if (BIT(m_creg[EXT_INVERT], 1) && offset < 0x2000)
		offset |= 0x6000;
	return offset;
}

uint8_t quaderno_lcd_device::vram_r(offs_t offset)
{
	return m_vram[vram_addr(offset)];
}

void quaderno_lcd_device::vram_w(offs_t offset, uint8_t data)
{
	m_vram[vram_addr(offset)] = data;
}

uint8_t quaderno_lcd_device::io_r(offs_t offset)
{
	switch (offset & 0x0f)
	{
		case 0x00: case 0x02: case 0x04: case 0x06:
			return m_index;
		case 0x01: case 0x03: case 0x05: case 0x07:
			return m_creg[m_index];
		case 0x08:
			return m_mode;
		case 0x09:
			return m_color;
		case 0x0a:
		{
			const bool vbl = m_screen->vblank();
			const bool hbl = m_screen->hblank();
			return 0xf0 | (vbl ? 0x08 : 0x00) | ((vbl || hbl) ? 0x01 : 0x00);
		}
		case 0x0e:
			return m_mode2;
		default:
			if (!machine().side_effects_disabled())
				logerror("%s: unknown LCD read 3d%x\n", machine().describe_context(), offset & 0x0f);
			return 0xff;
	}
}

void quaderno_lcd_device::io_w(offs_t offset, uint8_t data)
{
	switch (offset & 0x0f)
	{
		case 0x00: case 0x02: case 0x04: case 0x06:
			m_index = data;
			break;
		case 0x01: case 0x03: case 0x05: case 0x07:
			if (m_index >= 0x12)
				LOGMASKED(LOG_REGS, "%s: ext CRTC reg %02x = %02x\n", machine().describe_context(), m_index, data);
			m_creg[m_index] = data;
			break;
		case 0x08:
			m_mode = data;
			break;
		case 0x09:
			m_color = data;
			break;
		case 0x0e:
			m_mode2 = data;
			break;
		default:
			logerror("%s: unknown LCD write 3d%x = %02x\n", machine().describe_context(), offset & 0x0f, data);
			break;
	}
}

uint8_t quaderno_lcd_device::cga_level(uint8_t color, bool grey) const
{
	// TODO: real mapping is done by the gate array, measure on hardware
	static constexpr uint8_t luma[16] = { 0, 1, 2, 3, 2, 3, 3, 5, 2, 4, 5, 6, 5, 6, 7, 7 };
	if (!grey)
		return color ? 7 : 0;
	return luma[color & 0x0f];
}

uint32_t quaderno_lcd_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	const bool invert = BIT(m_creg[EXT_INVERT], 7);
	uint8_t shade[8];
	for (int i = 0; i < 8; i++)
		shade[i] = invert ? (7 - i) : i;

	if (!(m_mode & MODE_ENABLE))
	{
		bitmap.fill(shade[0], cliprect);
		return 0;
	}

	if (m_mode & MODE_GRAPHICS)
	{
		if (BIT(m_mode2, 0))
			draw_gfx400(bitmap, cliprect, shade);
		else
			draw_gfx200(bitmap, cliprect, shade);
	}
	else
	{
		draw_text(bitmap, cliprect, shade);
	}

	return 0;
}

void quaderno_lcd_device::draw_text(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade)
{
	const bool grey = (m_creg[EXT_GRAY] & 0x1f) == 0;
	const bool bold = BIT(m_creg[EXT_INVERT], 6);
	const bool wide = !(m_mode & MODE_80COL);
	const int cols = wide ? 40 : 80;
	const int rows = (m_creg[6] >= 1 && m_creg[6] <= 50) ? m_creg[6] : 25;
	const int cellh = std::max(1, 400 / rows);
	const uint16_t start = ((m_creg[12] & 0x3f) << 8) | m_creg[13];
	const uint16_t cursor = ((m_creg[14] & 0x3f) << 8) | m_creg[15];
	const bool blink_phase = BIT(m_frame, 4);
	const bool cursor_phase = BIT(m_frame, 3);

	// cursor lines; 8-line CRTC programming is doubled by the hardware
	int cs = m_creg[10] & 0x1f;
	int ce = m_creg[11] & 0x1f;
	const bool cursor_off = (m_creg[10] & 0x60) == 0x20;
	if ((m_creg[9] & 0x1f) <= 7 && ce <= 7)
	{
		cs <<= 1;
		ce = (ce << 1) | 1;
	}

	for (int y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		const int row = y / cellh;
		const int line = y % cellh;
		uint16_t *const dest = &bitmap.pix(y);

		if (row >= rows)
		{
			for (int x = cliprect.min_x; x <= cliprect.max_x; x++)
				dest[x] = shade[0];
			continue;
		}

		const int fline = line * 16 / cellh;
		for (int col = 0; col < cols; col++)
		{
			const uint16_t addr = start + row * cols + col;
			const uint8_t chr = m_vram[(addr * 2) & 0x7fff];
			const uint8_t attr = m_vram[(addr * 2 + 1) & 0x7fff];

			uint8_t fg = attr & 0x0f;
			uint8_t bg = attr >> 4;
			if (m_mode & MODE_BLINK)
			{
				bg &= 0x07;
				if (BIT(attr, 7) && blink_phase)
					fg = bg;
			}

			// two 8x16 fonts in VRAM, rows 0-7 and 8-15 in separate 2K halves
			const uint16_t font = 0x6000 | ((BIT(attr, 3) && bold) ? 0x1000 : 0x0000);
			uint8_t bits = (fline < 8)
					? m_vram[font | (chr << 3) | fline]
					: m_vram[font | 0x0800 | (chr << 3) | (fline - 8)];

			if (!cursor_off && cursor_phase && addr == cursor && fline >= cs && fline <= ce)
				bits = 0xff;

			const uint8_t fgl = shade[cga_level(fg, grey)];
			const uint8_t bgl = shade[cga_level(bg, grey)];
			for (int b = 0; b < 8; b++)
			{
				const uint8_t pen = BIT(bits, 7 - b) ? fgl : bgl;
				if (wide)
				{
					const int x = col * 16 + b * 2;
					dest[x] = pen;
					dest[x + 1] = pen;
				}
				else
				{
					dest[col * 8 + b] = pen;
				}
			}
		}
	}

	if (cliprect.max_y == 399)
		m_frame++;
}

void quaderno_lcd_device::draw_gfx400(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade)
{
	// Olivetti M24 / AT&T 6300 layout: 4 banks of 8K, line n in bank n & 3
	const uint16_t start = ((((m_creg[12] & 0x3f) << 8) | m_creg[13]) << 1) & 0x1fff;
	for (int y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		uint16_t *const dest = &bitmap.pix(y);
		const uint16_t base = ((start + (y >> 2) * 80) & 0x1fff) | ((y & 3) << 13);
		for (int i = 0; i < 80; i++)
		{
			const uint8_t data = m_vram[(base + i) & 0x7fff];
			for (int b = 0; b < 8; b++)
				dest[i * 8 + b] = shade[BIT(data, 7 - b) ? 7 : 0];
		}
	}
}

void quaderno_lcd_device::draw_gfx200(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade)
{
	// CGA 320x200x2 / 640x200x1, line doubled to 400
	const bool grey = (m_creg[EXT_GRAY] & 0x1f) == 0;
	const uint16_t start = ((((m_creg[12] & 0x3f) << 8) | m_creg[13]) << 1) & 0x1fff;
	const uint8_t bgl = shade[cga_level(m_color & 0x0f, grey)];
	static constexpr uint8_t level2bpp[4] = { 0, 3, 5, 7 };

	for (int y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		uint16_t *const dest = &bitmap.pix(y);
		const int sy = y >> 1;
		const uint16_t base = ((start + (sy >> 1) * 80) & 0x1fff) | ((sy & 1) << 13);
		for (int i = 0; i < 80; i++)
		{
			const uint8_t data = m_vram[(base + i) & 0x7fff];
			if (m_mode & MODE_HIRES)
			{
				for (int b = 0; b < 8; b++)
					dest[i * 8 + b] = shade[BIT(data, 7 - b) ? 7 : 0];
			}
			else
			{
				for (int p = 0; p < 4; p++)
				{
					const uint8_t v = (data >> (6 - p * 2)) & 3;
					const uint8_t pen = v ? shade[grey ? level2bpp[v] : 7] : bgl;
					dest[i * 8 + p * 2] = pen;
					dest[i * 8 + p * 2 + 1] = pen;
				}
			}
		}
	}
}
