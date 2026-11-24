// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 500/550C cartridge slots

    Each slot is a 256 KiB window of the printer's memory, in 16 KiB
    pages.  Font cartridges hold ROMs, the 22707A and 22707B cartridges
    128 KiB or 256 KiB of RAM (input buffer or downloaded soft fonts,
    configuration switch B2 of the DeskJet 550C).

**********************************************************************/

#ifndef MAME_BUS_HP_DJ_CART_SLOT_H
#define MAME_BUS_HP_DJ_CART_SLOT_H

#pragma once


class device_hp_dj_cart_interface : public device_interface
{
public:
	// 16 KiB page n (0-15) of the slot's window, nullptr where nothing answers
	virtual u8 *page(int n) = 0;
	virtual bool writable() const { return false; }

protected:
	device_hp_dj_cart_interface(const machine_config &mconfig, device_t &device);
};


class hp_dj_cart_slot_device : public device_t, public device_single_card_slot_interface<device_hp_dj_cart_interface>
{
public:
	template <typename T>
	hp_dj_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: hp_dj_cart_slot_device(mconfig, tag, owner, 0U)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}

	hp_dj_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	u8 *page(int n) { return m_cart ? m_cart->page(n) : nullptr; }
	bool writable() const { return m_cart && m_cart->writable(); }

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;

private:
	device_hp_dj_cart_interface *m_cart;
};

void hp_dj_cartridges(device_slot_interface &device);

DECLARE_DEVICE_TYPE(HP_DJ_CART_SLOT, hp_dj_cart_slot_device)

#endif // MAME_BUS_HP_DJ_CART_SLOT_H
