// license:BSD-3-Clause
// copyright-holders:rfka01
#ifndef MAME_BUS_DMV_K0815_H
#define MAME_BUS_DMV_K0815_H

#pragma once

#include "dmvbus.h"


// ======================> dmv_k0815_device

class dmv_k0815_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k0815_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// dmvcart_interface overrides
	virtual bool mem_read(offs_t offset, offs_t ba, uint8_t &data) override;
	virtual bool mem_write(offs_t offset, offs_t ba, uint8_t data) override;

private:
	bool dataen(offs_t ba) const;
	void ramsel_tap(offs_t offset);

	required_ioport             m_config;
	std::unique_ptr<uint8_t[]>  m_ram;
	memory_passthrough_handler  m_io_tap;
	int                         m_ramdis;
};

// device type definition
DECLARE_DEVICE_TYPE(DMV_K0815, dmv_k0815_device)

#endif // MAME_BUS_DMV_K0815_H
