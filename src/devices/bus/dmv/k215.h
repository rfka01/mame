// license:BSD-3-Clause
// copyright-holders:rfka01
#ifndef MAME_BUS_DMV_K215_H
#define MAME_BUS_DMV_K215_H

#pragma once

#include "dmvbus.h"
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
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// dmvcart_interface overrides
	virtual void io_read(int ifsel, offs_t offset, uint8_t &data) override;
	virtual void io_write(int ifsel, offs_t offset, uint8_t data) override;

private:
	bool selected(int ifsel, offs_t offset) const;
	void command_w(uint8_t data);
	void data_w(uint8_t data);
	void result(uint8_t data);
	void rxrdy_w(int state);
	TIMER_CALLBACK_MEMBER(rx_fetch);
	TIMER_CALLBACK_MEMBER(host_byte);
	void txrdy_w(int state);
	void tx_pump();

	required_device<scn_pci_device> m_pci;
	required_device<rs232_port_device> m_rs232;
	required_ioport m_dsw;
	emu_timer *m_rx_timer;
	emu_timer *m_cmd_timer;

	std::array<uint8_t, 0x800> m_rxbuf;
	std::array<uint8_t, 0x10> m_txbuf;
	uint16_t m_rx_head, m_rx_count;
	uint8_t m_tx_head, m_tx_count;
	uint8_t m_status;       // host status: bit 0 output buffer full, bit 1 input buffer full, bit 3 F1
	uint8_t m_out;          // output buffer
	uint8_t m_cmd;          // command waiting for its data byte (0xff = none)
	uint8_t m_last_cmd;     // last command, for the 2651 register pointers
	bool m_overrun;
	uint8_t m_in;           // byte from the host, not yet taken
	bool m_in_cmd;
};

// device type definition
DECLARE_DEVICE_TYPE(DMV_K215, dmv_k215_device)

#endif // MAME_BUS_DMV_K215_H
