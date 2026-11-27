// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet Portable font card slot

**********************************************************************/

#include "emu.h"
#include "slot.h"

#include "cards.h"


DEFINE_DEVICE_TYPE(HP_DJP_CARD_SLOT, hp_djp_card_slot_device, "hp_djp_card_slot", "HP DeskJet Portable font card slot")


device_hp_djp_card_interface::device_hp_djp_card_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "hpdjpcard")
{
}


hp_djp_card_slot_device::hp_djp_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_DJP_CARD_SLOT, tag, owner, clock),
	device_single_card_slot_interface<device_hp_djp_card_interface>(mconfig, *this),
	m_card(nullptr)
{
}

void hp_djp_card_slot_device::device_start()
{
	m_card = get_card_device();
}


void hp_djp_cards(device_slot_interface &device)
{
	device.option_add("x552", HP_DJP_CARD_X552);
}
