// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Star NL-10 9-pin dot matrix printer

***************************************************************************/

#ifndef MAME_BUS_CENTRONICS_STAR_NL10_H
#define MAME_BUS_CENTRONICS_STAR_NL10_H

#pragma once

#include "ctronics.h"
#include "cpu/m6800/m6801.h"
#include "machine/bitmap_printer.h"
#include "sound/spkrdev.h"


class star_nl10_device : public device_t, public device_centronics_peripheral_interface
{
public:
	star_nl10_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

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

	u8 buzzer_r();
	u8 dip_r();
	u8 panel_r();
	void needles_w(u8 data);
	u8 cent_data_r();
	void cent_clear_w(u8 data);

	u8 port2_r();
	void port2_w(u8 data);
	u8 port5_r();
	void port5_w(u8 data);
	void port6_w(u8 data);

	void update_busy();
	void fire(u8 pins);
	double head_x() const;

	required_device<hd6303y_cpu_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<speaker_sound_device> m_speaker;
	required_ioport m_dip;
	required_ioport m_panel;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_bold_led;
	output_finder<4> m_mode_led;

	// Centronics interface (on the cartridge)
	u8 m_data;
	u8 m_latch;
	bool m_strobe;
	bool m_data_ready;

	// outputs
	u8 m_needles;
	u8 m_port2;
	u8 m_port5;
	u8 m_port6;
	bool m_speaker_level;

	// mechanics
	int m_cr_pos;           // carriage position in 1/60" steps, 0 = left stop
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;
	int m_pf_phase;         // paper motor half-step phase, -1 = unknown
	attotime m_power_on;    // time the printer was switched on
};


DECLARE_DEVICE_TYPE(STAR_NL10, star_nl10_device)

#endif // MAME_BUS_CENTRONICS_STAR_NL10_H
