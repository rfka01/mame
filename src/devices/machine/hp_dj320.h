// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP DeskJet 320 portable inkjet printer

***************************************************************************/

#ifndef MAME_MACHINE_HP_DJ320_H
#define MAME_MACHINE_HP_DJ320_H

#pragma once

#include "cpu/z80/z80.h"
#include "bus/hp_djp_card/slot.h"
#include "machine/bitmap_printer.h"
#include "machine/eepromser.h"


class hp_dj320_device : public device_t
{
public:
	hp_dj320_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

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
	static constexpr XTAL TIMER_CLOCK = XTAL(16'000'000) * 5 / 32;   // placeholder: the overflow of the counter is the system tick, and the firmware counts 38 of them per second

	// carriage: DC motor with an encoder, 1200 counts per inch (from the move limits)
	static constexpr double CR_VMAX = 60000.0;      // counts per second at full PWM (placeholder)
	static constexpr double CR_TAU = 0.02;          // motor time constant (placeholder)
	static constexpr double CR_TRAVEL = 11500.0;    // counts between the end stops (placeholder)
	static constexpr double CR_SPRING = 600.0;      // the left end pushes against a spring this far (placeholder)
	static constexpr double CR_SPRING_HOLD = 450.0; // spring deflection where 3/4 drive stalls (placeholder)
	static constexpr double CR_PARK = CR_TRAVEL - 200.0;   // position at power on: parked near the right end stop (placeholder)

	// service station: stepper (port 4Bh high nibble, half steps) turning a cam, 640 half
	// steps per turn (from the firmware), with an index sensor on port 61h bit 0
	static constexpr int SS_STEPS = 640;
	static constexpr int SS_SENSOR = 40;            // half steps the index sensor stays active (placeholder)

	// paper path, in paper motor steps (placeholders)
	static constexpr int PAPER_INSERT = 40;         // lead edge past the sensor when the user stops pushing
	static constexpr int SHEET_LENGTH = 3300;       // 11 inch at 300 steps per inch
	static constexpr int PAPER_OUT = 600;           // the sheet leaves the printer this far past the sensor

	// output: the print engine fires a column every second encoder count (600 dpi),
	// the nozzles are 1/300 inch apart and one paper motor step moves the paper a row
	static constexpr int HDPI = 600;
	static constexpr int VDPI = 300;
	static constexpr double PAPER_WIDTH = 9.0;      // a sheet starts 0.25 inch in from the left of the paper view
	static constexpr int SHEET_LEFT = HDPI / 4;
	static constexpr double PAPER_HEIGHT = 297.0 / 25.4;
	static constexpr int PAPER_LEFT = 318;          // encoder count of the left edge of a sheet (placeholder: fitted to the 0.25 inch margin)
	static constexpr int DOWN_SHIFT = -48;          // passes towards lower counts land this many counts further right (placeholder: aligns the two directions)
	static constexpr int HEAD_OFFSET = 40;          // paper steps from the paper sensor to the top nozzle (placeholder)
	static constexpr int COLOUR_STAGGER = 4;        // columns (1/600 inch) between the odd and even nozzle columns of the colour cartridge (from the firmware data, sign not verified)

	required_device<z80_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<hp_djp_card_slot_device> m_card;
	required_region_ptr<u8> m_rom;
	required_region_ptr<u8> m_fonts;
	required_ioport m_keys;
	required_ioport m_config;
	output_finder<9> m_leds;   // 0 Menu, 1 Change Cartridge, 2 Draft, 3 Busy, 4 Landscape, 5 Compressed, 6 Online, 7 Power, 8 Charge

	devcb_write_line m_busy_cb;
	devcb_write_line m_ack_cb;
	devcb_write_line m_perror_cb;
	devcb_write_line m_fault_cb;
	devcb_write_line m_select_cb;

	emu_timer *m_timer;
	emu_timer *m_tick_timer;
	emu_timer *m_host_timer;
	emu_timer *m_ack_timer;

	std::unique_ptr<u8[]> m_psram;
	std::unique_ptr<u8[]> m_ram;
	u8 m_port[0x100];
	u8 m_irq_status;
	u16 m_compare;
	attotime m_counter_base;

	double m_cr_pos;
	double m_cr_vel;
	s32 m_cr_offset;
	attotime m_cr_time;
	emu_timer *m_cr_timer;
	emu_timer *m_nmi_timer;
	bool m_nmi_flag;
	int m_ss_phase;
	int m_ss_pos;
	bool m_pen_sense;
	int m_pf_phase;
	bool m_sheet;
	bool m_sheet_done;
	int m_sheet_lead;
	int m_sheet_len;
	bool m_sheet_inked;
	attotime m_sheet_time;
	u8 m_engine_block;
	u8 m_engine_mode;
	int m_engine_rows = 1;
	bool m_engine_600 = false;
	void paper_step(int dir);
	bool paper_sensor() const;
	int sheet_row() const { return m_sheet_lead - HEAD_OFFSET; }
	u8 engine_byte(unsigned block, u32 offset) const;
	void sense_pen(u16 ptr);
	void print_pass();
	void plot(int x, int y, u32 ink = 0x000000);

	u8 m_data;
	int m_strobe;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 window_r(offs_t offset);
	void window_w(offs_t offset, u8 data);
	u8 *window_ptr(offs_t offset, bool write);

	u8 port_r(offs_t offset);
	void port_w(offs_t offset, u8 data);

	u16 counter() const;
	void update_timer();
	void update_irq();
	TIMER_CALLBACK_MEMBER(timer_match);
	TIMER_CALLBACK_MEMBER(tick);
	TIMER_CALLBACK_MEMBER(cr_tick);
	TIMER_CALLBACK_MEMBER(servo_tick);
	void update_carriage();
	u16 encoder() { update_carriage(); return u16(s32(m_cr_pos) - m_cr_offset); }
	TIMER_CALLBACK_MEMBER(host_idle);
	TIMER_CALLBACK_MEMBER(ack_end);
	bool host_ready() const;
	void take_byte();
	bool m_pending = false;
	void update_busy();
};

DECLARE_DEVICE_TYPE(HP_DJ320, hp_dj320_device)

#endif // MAME_MACHINE_HP_DJ320_H
