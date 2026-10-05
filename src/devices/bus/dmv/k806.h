// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
#ifndef MAME_BUS_DMV_K806_H
#define MAME_BUS_DMV_K806_H

#pragma once

#include "dmvbus.h"
#include "cpu/mcs48/mcs48.h"
#include "machine/timer.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> dmv_k806_device

class dmv_k806_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k806_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void io_read(int ifsel, offs_t offset, uint8_t &data) override;
	virtual void io_write(int ifsel, offs_t offset, uint8_t data) override;

private:
	int portt1_r();
	uint8_t port1_r();
	void port2_w(uint8_t data);

	TIMER_DEVICE_CALLBACK_MEMBER(mouse_timer);

	required_device<upi41_cpu_device> m_mcu;
	required_ioport m_jumpers;
	required_ioport m_config;
	required_ioport m_mouse_buttons;
	required_ioport m_mouse_x;
	required_ioport m_mouse_y;

	void step_axis(int32_t &pending, uint8_t &phase);

	// host mouse position at the last timer tick (12-bit counters)
	uint16_t m_last_x;
	uint16_t m_last_y;

	// encoder edges still to be generated (signed, in firmware count direction)
	int32_t m_pending_x;
	int32_t m_pending_y;

	// quadrature phase of each encoder (0..3, Gray sequence)
	uint8_t m_phase_x;
	uint8_t m_phase_y;

	// axis that gets the next edge when both are moving
	uint8_t m_next_axis;

	// number of P1 reads by the 8741 since the last encoder edge
	uint8_t m_p1_reads;
};


// device type definition
DECLARE_DEVICE_TYPE(DMV_K806, dmv_k806_device)

#endif // MAME_BUS_DMV_K806_H
