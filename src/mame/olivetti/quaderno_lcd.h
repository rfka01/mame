// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Olivetti Quaderno - 640x400 reflective LCD controller ("DCGA")

***************************************************************************/

#ifndef MAME_OLIVETTI_QUADERNO_LCD_H
#define MAME_OLIVETTI_QUADERNO_LCD_H

#pragma once

#include "emupal.h"
#include "screen.h"


class quaderno_lcd_device : public device_t, public device_video_interface
{
public:
	quaderno_lcd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// host interface
	uint8_t vram_r(offs_t offset);
	void vram_w(offs_t offset, uint8_t data);
	uint8_t io_r(offs_t offset);   // 3d0-3df
	void io_w(offs_t offset, uint8_t data);

	// extended CRTC registers (index >= 0x20) for the driver / debugger
	uint8_t ext_reg(uint8_t index) const { return m_creg[index]; }

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	// mode control register (3d8), CGA layout
	static constexpr uint8_t MODE_80COL    = 0x01;
	static constexpr uint8_t MODE_GRAPHICS = 0x02;
	static constexpr uint8_t MODE_ENABLE   = 0x08;
	static constexpr uint8_t MODE_HIRES    = 0x10;
	static constexpr uint8_t MODE_BLINK    = 0x20;

	// extended CRTC registers documented by A. Westfeld (BIOS Rev. 1.06)
	static constexpr uint8_t EXT_GRAY      = 0xda; // 0x40 = grey levels on, 0x5f = off
	static constexpr uint8_t EXT_INVERT    = 0xdf; // bit 7 = inverse display, bit 1 = font load mode

	void lcd_palette(palette_device &palette) const;
	offs_t vram_addr(offs_t offset) const;

	void draw_text(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade);
	void draw_gfx400(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade);
	void draw_gfx200(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t const *shade);

	uint8_t cga_level(uint8_t color, bool grey) const;

	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;

	std::unique_ptr<uint8_t[]> m_vram;
	uint8_t m_index;
	uint8_t m_creg[0x100];
	uint8_t m_mode;
	uint8_t m_color;
	uint8_t m_mode2;
	uint32_t m_frame;
};

DECLARE_DEVICE_TYPE(QUADERNO_LCD, quaderno_lcd_device)

#endif // MAME_OLIVETTI_QUADERNO_LCD_H
