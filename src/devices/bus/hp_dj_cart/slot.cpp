// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 500/550C cartridge slots

**********************************************************************/

#include "emu.h"
#include "slot.h"

#include "cards.h"


DEFINE_DEVICE_TYPE(HP_DJ_CART_SLOT, hp_dj_cart_slot_device, "hp_dj_cart_slot", "HP DeskJet cartridge slot")


device_hp_dj_cart_interface::device_hp_dj_cart_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "hpdjcart")
{
}


hp_dj_cart_slot_device::hp_dj_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_DJ_CART_SLOT, tag, owner, clock),
	device_single_card_slot_interface<device_hp_dj_cart_interface>(mconfig, *this),
	m_cart(nullptr)
{
}

void hp_dj_cart_slot_device::device_start()
{
	m_cart = get_card_device();
}


void hp_dj_cartridges(device_slot_interface &device)
{
	device.option_add("rom", HP_DJ_CART_ROM);
	device.option_add("odscyr", HP_DJ_CART_ODSCYR);
	device.option_add("22707l", HP_DJ_CART_22707L);
	device.option_add("ram128", HP_DJ_CART_RAM128);
	device.option_add("ram256", HP_DJ_CART_RAM256);
}
