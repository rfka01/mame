// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Okidata Okimate 20 thermal transfer colour printer: logic board and
    mechanism, shared by the host interface modules (Plug 'n Print)

**********************************************************************/

#ifndef MAME_MACHINE_OKIMATE20_H
#define MAME_MACHINE_OKIMATE20_H

#pragma once

#include "cpu/mcs51/i8051.h"
#include "machine/bitmap_printer.h"


class okimate20_base_device : public device_t
{
protected:
	// io_base: address of the M60307 printer gate array, decoded on the module
	okimate20_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, offs_t io_base);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// host interface module
	virtual u8 port1_r() { return 0xff; }
	virtual void port1_w(u8 data) { }
	virtual u8 port3_r() { return 0xff; }
	virtual void port3_w(u8 data) { }
	virtual u8 module_r(offs_t offset) { return 0xff; }        // M60306 registers beyond the timer
	virtual void module_w(offs_t offset, u8 data) { }
	virtual u8 module_id() const { return 0x08; }              // M60307 offset 1 bit 3

	void reset_cpu() { m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero); }

	required_device<i8031_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_panel;
	required_ioport m_sensors;
	required_ioport m_conf;

private:
	void prog_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;

	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);
	u8 q4_r(offs_t offset);
	void q4_w(offs_t offset, u8 data);

	TIMER_CALLBACK_MEMBER(step_tick);
	TIMER_CALLBACK_MEMBER(q4_tick);

	void fire();
	void paper_step(u8 data);
	double head_pos() const;
	bool marker() const;
	int ink() const;
	static u32 ink_mix(int inks);
	static u32 add_ink(u32 paper, int ink);

	output_finder<> m_ready_led;

	offs_t const m_io_base;
	emu_timer *m_step_timer;
	emu_timer *m_q4_timer;

	u8 m_io[8];             // M60307 registers as written
	u32 m_head;             // 24 head elements, shifted in through offset 0
	int m_pf_phase;         // paper motor phase (Gray code)
	int m_cr_pos;           // carriage position, steps from the home sensor edge
	int m_cr_dir;
	int m_cr_count;         // steps since the last interrupt
	attotime m_cr_start;
	attotime m_cr_period;
	int m_rib_pos;          // ribbon taken from the supply spool, carriage steps
	u8 m_q4_status;
	u16 m_q4_count;
	u8 m_q4_ctrl;           // offset 3 bit 0: head down
};

INPUT_PORTS_EXTERN( okimate20 );

#endif // MAME_MACHINE_OKIMATE20_H
