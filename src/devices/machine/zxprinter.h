// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX Printer and Alphacom 32 printer mechanisms

**********************************************************************/

#ifndef MAME_MACHINE_ZXPRINTER_H
#define MAME_MACHINE_ZXPRINTER_H

#pragma once

#include "machine/bitmap_printer.h"


class zx_printer_device : public device_t
{
public:
	zx_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// port FB, selected by A2 low
	uint8_t read();
	void write(uint8_t data);

	DECLARE_INPUT_CHANGED_MEMBER(feed_w);

protected:
	zx_printer_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	static constexpr int DOTS = 256;

	int current_x() const;
	void update();
	void output_row();
	attotime pixel_period(int speed) const;
	static void aluminium_paper(printer_page_canvas &page);
	TIMER_CALLBACK_MEMBER(tick);

	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_buttons;
	emu_timer *m_timer;

	bool m_host;           // the computer runs the motor
	int m_speed;           // 0 = stopped, 1 = slow, 2 = fast
	int m_new_speed;       // speed change at the next row
	attotime m_row_start;  // time of x = -64 in the current row
	int m_pixel;           // position of the last write (encoder)
	int m_filled;          // dots of the row already burnt or left blank
	int m_stylus;
	bool m_row_done;
	uint8_t m_row[DOTS];
};


class alphacom32_device : public zx_printer_device
{
public:
	alphacom32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
};


DECLARE_DEVICE_TYPE(ZX_PRINTER, zx_printer_device)
DECLARE_DEVICE_TYPE(ALPHACOM32, alphacom32_device)

#endif // MAME_MACHINE_ZXPRINTER_H
