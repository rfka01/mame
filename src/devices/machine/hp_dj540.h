// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP DeskJet 540 inkjet printer

***************************************************************************/

#ifndef MAME_MACHINE_HP_DJ540_H
#define MAME_MACHINE_HP_DJ540_H

#pragma once

#include "cpu/m68000/m68000.h"
#include "machine/bitmap_printer.h"
#include "machine/i2cmem.h"


class hp_dj540_device : public device_t
{
public:
	hp_dj540_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// parallel interface
	void data_w(u8 data) { m_data = data; }
	void strobe_w(int state);
	void init_w(int state);
	auto busy_cb() { return m_busy_cb.bind(); }
	auto ack_cb() { return m_ack_cb.bind(); }
	auto perror_cb() { return m_perror_cb.bind(); }
	auto fault_cb() { return m_fault_cb.bind(); }
	auto select_cb() { return m_select_cb.bind(); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	static constexpr int IRQ_LEVEL = 4;     // all chip interrupts share one level (assumed)
	static constexpr XTAL TIMER_CLOCK = XTAL(2'000'000);   // gives the specified 167 cps (full line passes at 16.7 inches per second)

	// carriage: DC motor and encoder, 600 counts per inch
	static constexpr double CR_VMAX = 20000.0;      // counts per second at full PWM (placeholder)
	static constexpr double CR_TAU = 0.02;          // motor time constant in seconds (placeholder)
	static constexpr double CR_TRAVEL = 7650.0;     // counts between the end stops (from the firmware)
	static constexpr double CR_PARK = 7000.0;       // carriage position at power on
	static constexpr double PICK_CARRIAGE = 7000.0; // the carriage at the right end engages the pick

	// paper path, in paper motor half steps (1/300 inch)
	static constexpr int PICK_DIST = 450;           // tray to paper sensor (placeholder)
	static constexpr int SENSOR_TO_HEAD = 240;      // paper sensor to the top nozzle (fitted: 0.5 inch top margin for text)
	static constexpr int HEAD_ROWS = 48;

	// paper view and nozzle positions
	static constexpr int HDPI = 600;
	static constexpr int VDPI = 300;
	static constexpr double PAPER_WIDTH = 9.0;      // a sheet starts 0.25 inch in from the left of the paper view
	static constexpr double PAPER_HEIGHT = 297.0 / 25.4;
	static constexpr int SHEET_LEFT = HDPI / 4;
	static constexpr int PAPER_LEFT = 1769;         // encoder count of the left edge of a sheet (fitted to the 0.25 inch margin)
	static constexpr int REVERSE_OFFSET = -21;      // passes towards lower counts land this much further on (fitted)
	// colour cartridge, groups yellow, magenta, cyan from the top: rows below the
	// straight line and encoder counts across (fitted to a CMY line test page)
	static constexpr int COLOUR_DY[3] = { 0, 4, 8 };
	static constexpr int COLOUR_DX[3] = { 0, -36, 0 };

	enum { PEN_NONE = 0, PEN_BLACK, PEN_COLOUR };

	required_device<m68000_device> m_maincpu;
	required_device<i2cmem_device> m_eeprom;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_keys;
	required_ioport m_config;
	output_finder<2> m_led;

	devcb_write_line m_busy_cb;
	devcb_write_line m_ack_cb;
	devcb_write_line m_perror_cb;
	devcb_write_line m_fault_cb;
	devcb_write_line m_select_cb;

	emu_timer *m_tick_timer;
	emu_timer *m_step_timer;
	emu_timer *m_servo_timer;
	emu_timer *m_encoder_timer;
	emu_timer *m_ack_timer;

	u8 m_reg[0xc00];
	u8 m_irq_pending;
	bool m_host_irq;

	// host interface
	u8 m_data;
	int m_strobe;
	u8 m_host_status;
	bool m_byte_waiting;

	// pen identification
	bool m_pen_sense;

	// steppers: 0 paper feed, 1 service station
	s8 m_phase[2];

	// carriage
	double m_cr_pos;
	double m_cr_vel;
	s32 m_cr_offset;
	s32 m_cr_last;
	attotime m_cr_time;
	bool m_cmp_armed;
	bool m_printing;
	bool m_odd_count;

	// paper
	bool m_sheet;
	s32 m_sheet_lead;
	s32 m_sheet_len;
	attotime m_paper_step_time;

	// glyph engine
	u32 m_widen_last;

	void mem_map(address_map &map) ATTR_COLD;
	void cpu_space_map(address_map &map) ATTR_COLD;

	u8 reg_r(offs_t offset);
	void reg_w(offs_t offset, u8 data);
	u32 reg32(offs_t a) const { a -= 0xfff400; return (m_reg[a] << 24) | (m_reg[a + 1] << 16) | (m_reg[a + 2] << 8) | m_reg[a + 3]; }
	void set_reg32(offs_t a, u32 v) { a -= 0xfff400; m_reg[a] = v >> 24; m_reg[a + 1] = v >> 16; m_reg[a + 2] = v >> 8; m_reg[a + 3] = v; }
	u16 reg16(offs_t a) const { a -= 0xfff400; return (m_reg[a] << 8) | m_reg[a + 1]; }

	void set_irq(int bit, bool state);
	void update_irq();
	u8 irq_vector_r();

	TIMER_CALLBACK_MEMBER(tick);
	TIMER_CALLBACK_MEMBER(step_tick);
	TIMER_CALLBACK_MEMBER(servo_tick);
	TIMER_CALLBACK_MEMBER(encoder_tick);
	TIMER_CALLBACK_MEMBER(ack_end);
	void update_tick();
	void update_step_timer();
	void update_servo_timer();

	u8 port_in(int port);
	void port_changed(int port);
	int pen() const;
	void fire();
	void step(int motor, u8 coils);

	void update_carriage();
	void encoder_count(u16 count);
	void print_column(u16 count);
	void plot(int x, int row, int ink);

	void paper_step(int dir);
	bool paper_sensor() const;
	int sheet_row() const { return m_sheet_lead - SENSOR_TO_HEAD; }

	void glyph_op();
	void widen_window();
	void transpose();

	void dma_waiting();
	u8 host_status() const;
	void update_host();
	void update_busy();
};

DECLARE_DEVICE_TYPE(HP_DJ540, hp_dj540_device)

#endif // MAME_MACHINE_HP_DJ540_H
