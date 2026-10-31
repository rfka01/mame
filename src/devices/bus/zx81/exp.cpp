// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX81 expansion port

**********************************************************************/

#include "emu.h"
#include "exp.h"


DEFINE_DEVICE_TYPE(ZX81_EXPANSION_SLOT, zx81_expansion_slot_device, "zx81_expansion_slot", "ZX81 expansion port")


device_zx81_expansion_interface::device_zx81_expansion_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "zx81exp")
{
}


zx81_expansion_slot_device::zx81_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ZX81_EXPANSION_SLOT, tag, owner, clock),
	device_single_card_slot_interface<device_zx81_expansion_interface>(mconfig, *this),
	m_card(nullptr)
{
}

void zx81_expansion_slot_device::device_start()
{
	m_card = get_card_device();
}

uint8_t zx81_expansion_slot_device::iorq_r(offs_t offset)
{
	return m_card ? m_card->iorq_r(offset) : 0xff;
}

void zx81_expansion_slot_device::iorq_w(offs_t offset, uint8_t data)
{
	if (m_card)
		m_card->iorq_w(offset, data);
}


#include "zxprinter.h"

void zx81_expansion_devices(device_slot_interface &device)
{
	device.option_add("alphacom32", ZX81_ALPHACOM32);
	device.option_add("zxprinter", ZX81_ZXPRINTER);
}
