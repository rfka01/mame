// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Nixdorf MD 16 portable ink-jet printer

***************************************************************************/

#ifndef MAME_BUS_CENTRONICS_NIXDORF_MD16_H
#define MAME_BUS_CENTRONICS_NIXDORF_MD16_H

#pragma once

#include "ctronics.h"
#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"


class nixdorf_md16_device : public device_t, public device_centronics_peripheral_interface
{
public:
	nixdorf_md16_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// Centronics
	virtual void input_strobe(int state) override;
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

	DECLARE_INPUT_CHANGED_MEMBER(power_switch);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 ga_r();
	void ga_w(u8 data);
	void pa_w(u8 data);
	u8 pb_r();
	void pb_w(u8 data);
	u8 pc_r();
	void pc_w(u8 data);
	u8 an0_r();
	u8 an1_r();
	u8 an2_r();
	u8 an3_r();
	template <int N> u8 dip_r();

	void motor_phase(u8 phases);
	void update_status();
	void fire(int port, u8 data);
	double head_x() const;
	bool keys_held(u8 mask) const;

	required_device<upd78c10_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_ioport m_keys;
	required_ioport m_dip;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_paper_led;
	output_finder<> m_error_led;

	// Centronics interface (gate array)
	u8 m_data;
	u8 m_latch;
	bool m_strobe;
	bool m_busy;

	// outputs
	u8 m_ga;
	u8 m_pa;
	u8 m_pc;

	// mechanics
	int m_cr_pos;           // carriage position in 1/40" steps, 0 = left stop
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pf_phase;
	attotime m_power_on;
	bool m_power_cycled;    // switched on with the power switch, not by a machine reset
};


DECLARE_DEVICE_TYPE(NIXDORF_MD16, nixdorf_md16_device)

#endif // MAME_BUS_CENTRONICS_NIXDORF_MD16_H
