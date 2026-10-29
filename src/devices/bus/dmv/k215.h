// license:BSD-3-Clause
// copyright-holders:rfka01
#ifndef MAME_BUS_DMV_K215_H
#define MAME_BUS_DMV_K215_H

#pragma once

#include "dmvbus.h"
#include "cpu/mcs48/mcs48.h"
#include "machine/scn_pci.h"
#include "bus/rs232/rs232.h"

#include <array>


// ======================> dmv_k215_device

class dmv_k215_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k215_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// dmvcart_interface overrides
	virtual void io_read(int ifsel, offs_t offset, uint8_t &data) override;
	virtual void io_write(int ifsel, offs_t offset, uint8_t data) override;
	virtual bool drives_irq() const override { return true; }

private:
	bool selected(int ifsel, offs_t offset) const;
	uint8_t pci_offset() const;

	uint8_t p1_r();
	void p1_w(uint8_t data);
	void p2_w(uint8_t data);
	int t1_r();

	required_device<upi41_cpu_device> m_mcu;
	required_device<scn_pci_device> m_pci;
	required_device<rs232_port_device> m_rs232;
	required_ioport m_dsw;

	std::array<uint8_t, 0x800> m_ram;   // HM6116 receive buffer
	uint16_t m_ram_addr;                // LS377 (A0-A7) and LS175 (A8-A10)
	uint8_t m_p1;
	uint8_t m_p2;
};

// device type definition
DECLARE_DEVICE_TYPE(DMV_K215, dmv_k215_device)

#endif // MAME_BUS_DMV_K215_H
