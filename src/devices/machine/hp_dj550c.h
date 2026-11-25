// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HP DeskJet 550C colour inkjet printer

    The printer with its Centronics parallel and RS-232C serial interfaces;
    see bus/centronics/hp_dj550c.h and bus/rs232/hp_dj550c.h for the
    devices that plug into a computer's ports.

***************************************************************************/

#ifndef MAME_MACHINE_HP_DJ550C_H
#define MAME_MACHINE_HP_DJ550C_H

#pragma once

#include "bus/hp_dj_cart/slot.h"
#include "cpu/z80/z80.h"
#include "machine/bitmap_printer.h"
#include "machine/eepromser.h"

#include "diserial.h"

#include <memory>


class hp_dj550c_device : public device_t, public device_serial_interface
{
public:
	// the motor and timer rates are fitted to the firmware, not measured
	static constexpr feature_type imperfect_features() { return feature::TIMING; }

	hp_dj550c_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// parallel interface: data, /STROBE and /INIT (Reset/Input Clean) from
	// the computer, the status lines to it
	void data_w(u8 data) { m_data = data; }
	void strobe_w(int state);
	void init_w(int state);
	auto busy_cb() { return m_busy_cb.bind(); }
	auto ack_cb() { return m_ack_cb.bind(); }
	auto perror_cb() { return m_perror_cb.bind(); }
	auto fault_cb() { return m_fault_cb.bind(); }
	auto select_cb() { return m_select_cb.bind(); }

	// serial interface: data from the computer arrives on the printer's RxD,
	// TxD and DTR go to the computer
	void rxd_w(int state) { rx_w(state); }
	auto txd_cb() { return m_txd_cb.bind(); }
	auto dtr_cb() { return m_dtr_cb.bind(); }

	DECLARE_INPUT_CHANGED_MEMBER(power_changed);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_serial_interface implementation
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	// free running 16 bit counter, which times the steppers; its overflow is the system
	// tick (37.5 Hz, the firmware counts 38 to a second).  The rate is fitted to a video
	// of a real printer: the service station motor sings at 600 and 300 Hz and the paper
	// motor at 500 Hz, the half step rates the firmware sets at 2.4576 MHz
	static constexpr XTAL COUNTER_CLOCK = 15.9744_MHz_XTAL * 2 / 13;
	static constexpr int DPI = 300;
	static constexpr double PAPER_WIDTH = 9.0;      // Letter sheet 0.5 inch in from the left of the paper view
	static constexpr double PAPER_HEIGHT = 297.0 / 25.4;
	static constexpr int SHEET_LEFT = DPI / 2;      // left edge of a sheet in the paper view

	// carriage, in encoder counts (1/600 inch)
	static constexpr double CR_RIGHT_STOP = 7690.0;
	static constexpr double CR_PICK = 7600.0;       // pressed against the right stop: the paper motor picks a sheet
	static constexpr int PAPER_LEFT = 1080;         // left edge of Letter and A4 sheets (fitted to the 0.25" Letter margin;
	                                                // the firmware starts A4 lines 80 counts in, near its 0.125" margin)
	static constexpr int LEFTWARD_OFFSET = -24;     // leftward passes (fitted)
	static constexpr int LEFTWARD_RASTER = -8;      // and raster blocks in them, in addition (fitted)

	// paper path, in paper motor half steps (1/300 inch)
	static constexpr int PICK_DISTANCE = 1500;      // pick roller to paper sensor (placeholder)
	static constexpr int SENSOR_TO_HEAD = 240;      // paper sensor to the top nozzle (from the top of page)
	static constexpr int HEAD_ROWS = 56;            // rows below the top nozzle a pass can print on
	static constexpr int SENSOR_TRAIL = 30;         // the sensor still sees a sheet this far past its trailing edge (fitted:
	                                                // with less, the firmware takes the paper end for the end of the page
	                                                // before the 60th line, and puts that line on a new sheet)

	required_device<z80_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device_array<hp_dj_cart_slot_device, 2> m_slot;
	required_region_ptr<u8> m_prog;
	required_region_ptr<u8> m_rom_data;
	required_ioport m_keys;
	required_ioport_array<2> m_switches;
	required_ioport m_config;
	output_finder<9> m_led;

	devcb_write_line m_busy_cb;
	devcb_write_line m_ack_cb;
	devcb_write_line m_perror_cb;
	devcb_write_line m_fault_cb;
	devcb_write_line m_select_cb;
	devcb_write_line m_txd_cb;
	devcb_write_line m_dtr_cb;

	emu_timer *m_tick_timer;
	emu_timer *m_cmp_timer;
	emu_timer *m_servo_timer;
	emu_timer *m_ack_timer;

	std::unique_ptr<u8 []> m_ram;
	u8 *m_page[0x80];       // 16 KiB banks, nullptr where nothing answers
	bool m_page_writable[0x80];
	u8 m_bank[4];
	u8 m_io[0x100];         // last value written to each port
	u8 m_irq_pending;
	u16 m_cmp;
	u16 m_dma_cur;
	u16 m_dma_end;
	bool m_dma_full;

	// carriage
	double m_cpos;          // encoder counts
	double m_cvel;          // counts per servo sample
	u16 m_enc_offset;
	bool m_nmi;

	// steppers: paper feed and service station
	s8 m_phase[2];
	s32 m_motor_pos[2];

	// paper
	bool m_sheet;           // a sheet is in the paper path
	s32 m_sheet_pos;        // half steps of its leading edge after the paper sensor
	s32 m_sheet_rows;
	u8 m_paper_sensor;

	u8 m_pen_sense;

	// host interface
	u8 m_data;
	u8 m_platch;            // parallel input latch
	bool m_platch_full;
	int m_strobe;
	int m_init;
	static constexpr int SFIFO_SIZE = 16;
	u8 m_sfifo[SFIFO_SIZE]; // serial receiver
	u8 m_sfifo_head;
	u8 m_sfifo_count;
	u8 m_sstatus;
	u8 m_lines;             // last state of the parallel status lines

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);
	u8 phys_r(u32 addr) const;

	TIMER_CALLBACK_MEMBER(tick);
	TIMER_CALLBACK_MEMBER(cmp);
	TIMER_CALLBACK_MEMBER(servo);
	TIMER_CALLBACK_MEMBER(ack_end);

	u16 counter() const { return u16(machine().time().as_ticks(COUNTER_CLOCK)); }
	u16 encoder() const { return u16(s32(m_cpos)) - m_enc_offset; }
	void arm_cmp();
	void set_irq(int bit, bool state);
	void update_irq();

	bool dma_on() const { return BIT(m_io[0x4a], 6) && (m_dma_cur != m_dma_end); }
	bool parallel_dma() const { return dma_on() && BIT(m_io[0x4a], 5); }
	bool serial_dma() const { return dma_on() && BIT(m_io[0x0b], 3); }
	void dma_store(u8 data);
	void service_host();
	void update_parallel();
	u8 status_lines() const;
	void update_serial_format();

	void step(int motor, u8 coils);
	void paper_step(int dir);
	int sheet_row() const { return m_sheet ? (m_sheet_pos - SENSOR_TO_HEAD) : -DPI * 20; }

	u8 panel_r();
	void run_list();
	void plot(int x, int row, int ink);
};

DECLARE_DEVICE_TYPE(HP_DJ550C, hp_dj550c_device)

#endif // MAME_MACHINE_HP_DJ550C_H
