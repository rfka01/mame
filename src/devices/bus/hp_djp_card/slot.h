// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet Portable font card slot (DeskJet Portable, 310, 320)

    A thin half length card, PCMCIA-like but not PCMCIA compatible.  The
    slot is a 512 KiB window of the printer's memory (two flash or ROM
    chips on the card), handed out in 16 KiB pages.

**********************************************************************/

#ifndef MAME_BUS_HP_DJP_CARD_SLOT_H
#define MAME_BUS_HP_DJP_CARD_SLOT_H

#pragma once


class device_hp_djp_card_interface : public device_interface
{
public:
	// 16 KiB page n (0-31) of the slot's window, nullptr where nothing answers
	virtual u8 const *page(int n) = 0;

protected:
	device_hp_djp_card_interface(const machine_config &mconfig, device_t &device);
};


class hp_djp_card_slot_device : public device_t, public device_single_card_slot_interface<device_hp_djp_card_interface>
{
public:
	template <typename T>
	hp_djp_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: hp_djp_card_slot_device(mconfig, tag, owner, 0U)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}

	hp_djp_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	u8 const *page(int n) { return m_card ? m_card->page(n) : nullptr; }

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;

private:
	device_hp_djp_card_interface *m_card;
};


void hp_djp_cards(device_slot_interface &device);

DECLARE_DEVICE_TYPE(HP_DJP_CARD_SLOT, hp_djp_card_slot_device)

#endif // MAME_BUS_HP_DJP_CARD_SLOT_H
