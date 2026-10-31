// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX81 expansion port

    Only the I/O cycles are passed on so far, enough for the printers.

**********************************************************************/

#ifndef MAME_BUS_ZX81_EXP_H
#define MAME_BUS_ZX81_EXP_H

#pragma once


class device_zx81_expansion_interface;

class zx81_expansion_slot_device : public device_t, public device_single_card_slot_interface<device_zx81_expansion_interface>
{
public:
	template <typename T>
	zx81_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt) :
		zx81_expansion_slot_device(mconfig, tag, owner, 0)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	zx81_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	uint8_t iorq_r(offs_t offset);
	void iorq_w(offs_t offset, uint8_t data);

protected:
	virtual void device_start() override ATTR_COLD;

private:
	device_zx81_expansion_interface *m_card;
};


class device_zx81_expansion_interface : public device_interface
{
public:
	virtual uint8_t iorq_r(offs_t offset) { return 0xff; }
	virtual void iorq_w(offs_t offset, uint8_t data) { }

protected:
	device_zx81_expansion_interface(const machine_config &mconfig, device_t &device);
};


DECLARE_DEVICE_TYPE(ZX81_EXPANSION_SLOT, zx81_expansion_slot_device)

void zx81_expansion_devices(device_slot_interface &device);

#endif // MAME_BUS_ZX81_EXP_H
