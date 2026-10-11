// license:BSD-3-Clause
// copyright-holders:Ramiro Polla, Felipe Sanches, rfka01
/*
 * Epson LX-810L dot matrix printer emulation
 *
 */
#ifndef MAME_BUS_CENTRONICS_EPSON_LX810L_H
#define MAME_BUS_CENTRONICS_EPSON_LX810L_H

#pragma once

#include "ctronics.h"
#include "cpu/upd7810/upd7810.h"
#include "machine/e05a30.h"
#include "machine/eepromser.h"
#include "machine/bitmap_printer.h"
#include "sound/dac.h"
#include "screen.h"

#include <cstdlib>


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> epson_lx810l_device

class epson_lx810l_device : public device_t, public device_centronics_peripheral_interface
{
public:
	// construction/destruction
	epson_lx810l_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	/* Centronics stuff */
	virtual void input_strobe(int state) override { m_e05a30->centronics_input_strobe(state); }
	virtual void input_data0(int state) override { m_e05a30->centronics_input_data0(state); }
	virtual void input_data1(int state) override { m_e05a30->centronics_input_data1(state); }
	virtual void input_data2(int state) override { m_e05a30->centronics_input_data2(state); }
	virtual void input_data3(int state) override { m_e05a30->centronics_input_data3(state); }
	virtual void input_data4(int state) override { m_e05a30->centronics_input_data4(state); }
	virtual void input_data5(int state) override { m_e05a30->centronics_input_data5(state); }
	virtual void input_data6(int state) override { m_e05a30->centronics_input_data6(state); }
	virtual void input_data7(int state) override { m_e05a30->centronics_input_data7(state); }
	virtual void input_init(int state) override { m_e05a30->centronics_input_init(state); }

	/* Panel buttons */
	DECLARE_INPUT_CHANGED_MEMBER(online_sw);

	/* Power switch: turning the printer off and on again restarts only the printer */
	DECLARE_INPUT_CHANGED_MEMBER(power_switch);

protected:
	epson_lx810l_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

private:
	uint8_t porta_r(offs_t offset);
	void porta_w(offs_t offset, uint8_t data);
	uint8_t portb_r(offs_t offset);
	void portb_w(offs_t offset, uint8_t data);
	uint8_t portc_r(offs_t offset);
	void portc_w(offs_t offset, uint8_t data);

	/* fake memory I/O to get past memory reset check */
	uint8_t fakemem_r();
	void fakemem_w(uint8_t data);

	/* Extended Timer Output */
	void co0_w(int state);

	/* ADC */
	uint8_t an0_r();
	uint8_t an1_r();
	uint8_t an2_r();
	uint8_t an3_r();
	uint8_t an4_r();
	uint8_t an5_r();
	uint8_t an6_r();
	uint8_t an7_r();


	/* GATE ARRAY */
	void printhead(uint16_t data);
	void pf_stepper(uint8_t data);
	void cr_stepper(uint8_t data);

	/* carriage, paper and printhead */
	void carriage_step(int dir, int old_half);
	double head_x() const;
	void fire();
	void restart();
	uint8_t keys_held() const;
	bool paper_sensor() const;

	void e05a30_ready(int state);

	void e05a30_centronics_ack(int state) { output_ack(state); }
	void e05a30_centronics_busy(int state) { output_busy(state); }
	void e05a30_centronics_perror(int state) { output_perror(state); }
	void e05a30_centronics_fault(int state) { output_fault(state); }
	void e05a30_centronics_select(int state)
	{
		// the ON LINE lamp follows SLCT
		output_select(state);
		m_online_led = state;
		m_bitmap_printer->set_led_state(bitmap_printer_device::LED_ONLINE, state);
	}

	void e05a30_cpu_reset(int state) { if (!state) m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero); } // reset cpu

	void e05a30_ready_led(int state)
	{
		m_ready_led = state;
		m_bitmap_printer->set_led_state(bitmap_printer_device::LED_READY, m_ready_led);
	}

	void lx810l_mem(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<e05a30_device> m_e05a30;

	output_finder<> m_online_led;
	output_finder<> m_ready_led;
	output_finder<> m_paperout_led;    // follows the paper end sensor

	required_ioport m_online_ioport;
	required_ioport m_formfeed_ioport;
	required_ioport m_linefeed_ioport;
	required_ioport m_loadeject_ioport;
	required_ioport m_dipsw1_ioport;
	required_ioport m_dipsw2_ioport;
	required_ioport m_config_ioport;

	int m_93c06_clk;
	int m_93c06_cs;
	uint16_t m_printhead;
	uint8_t m_fakemem;
	int m_co0;

	// carriage: 4-phase stepper, 1/60" per full step
	int m_cr_pos;           // full steps right of the home sensor edge
	int m_cr_half;          // position in half steps
	int m_cr_angle;         // electrical angle of the energised phases, in half steps
	int m_cr_dir;
	int m_cr_step_start;    // position in half steps before the current step
	attotime m_cr_last_step;
	attotime m_cr_last_change;
	attotime m_cr_period;

	// paper: 4-phase stepper, 1/216" per full step
	int m_pf_angle;
	int m_pf_half;

	attotime m_power_on;
	bool m_power_cycled;
};

// ======================> epson_ap2000_t

class epson_ap2000_device : public epson_lx810l_device
{
public:
	// construction/destruction
	epson_ap2000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
};


// device type definition
DECLARE_DEVICE_TYPE(EPSON_LX810L, epson_lx810l_device)
DECLARE_DEVICE_TYPE(EPSON_AP2000, epson_ap2000_device)

#endif // MAME_BUS_CENTRONICS_EPSON_LX810L_H
