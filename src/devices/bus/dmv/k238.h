// license:BSD-3-Clause
// copyright-holders:rfka01
// thanks-to:Sandro Ronco
/***************************************************************************

    K238 External Z8001 processor module (homebrew / not an NCR product)

    A Z8001 bus-master coprocessor card for the NCR Decision Mate V,
    structured after the K234 68008 module (see k230.cpp / k230.h).

    Like the K234 it is a PARASITE on the DMV bus:
      - it has NO local boot EPROM
      - the host (Z80, or the 8088 of a K235) puts the Z8001 image and its
        reset context into shared DMV RAM, then rings this card's SNR
        doorbell; every ring from the host starts the Z8001 with a reset
      - the Z8001 rings the same doorbell to hand the bus back
    The Z80 is held via THOLD while the Z8001 runs.

***************************************************************************/

#ifndef MAME_BUS_DMV_K238_H
#define MAME_BUS_DMV_K238_H

#pragma once

#include "dmvbus.h"
#include "cpu/z8000/z8000.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> dmv_k238_device

class dmv_k238_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k238_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	dmv_k238_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// no device_rom_region(): this card has no local ROM (see K234)

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// dmvcart_interface overrides
	virtual void hold_w(int state) override;
	virtual void switch16_w(int state) override;
	virtual bool av16bit() override;

private:
	// address maps for the Z8001. Only AS_PROGRAM and AS_IO are installed;
	// the core aliases data/stack cycles onto AS_PROGRAM (see k238.cpp).
	void k238_prog(address_map &map) ATTR_COLD;   // AS_PROGRAM -> shared DMV RAM
	void k238_io(address_map &map) ATTR_COLD;     // AS_IO      -> DMV motherboard I/O

	required_device<z8001_device> m_maincpu;

	int  m_switch16;     // SWITCH16/ latch, as in K234
	int  m_hold;         // bus hold request from host DMA / Z80
	int  m_snr;          // 1 while the Z8001 owns the bus
	emu_timer *m_snr_reset_timer = nullptr;

	// shared-bus program/data access (byte lanes as the DMV sees them)
	uint8_t  program_r(offs_t offset);
	void     program_w(offs_t offset, uint8_t data);

	// DMV motherboard I/O forwarded from the Z8001 I/O space
	uint8_t  io_r(offs_t offset);
	void     io_w(offs_t offset, uint8_t data);

	// SNR doorbell (0xD8-0xDF): a ring from another bus master starts the
	// Z8001, a ring from the Z8001 itself hands the bus back
	TIMER_CALLBACK_MEMBER(snr_reset);
	void     snr_start();
	void     snr_hand_back();
	uint8_t  snr_r();
	void     snr_w(uint8_t data);
};


// device type definition
DECLARE_DEVICE_TYPE(DMV_K238, dmv_k238_device)

#endif // MAME_BUS_DMV_K238_H
