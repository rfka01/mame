// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 500/550C cartridges

    Font cartridge: a ROM image (cartridge1/cartridge2 media), from the
    start of the slot's window.  Each 64 KiB block of a font ROM starts
    with a header (62 00 20 ..., the cartridge number and name); the
    printer's firmware finds the fonts by itself.  The edge connector
    carries A0-A15, the data lines, four LED lines and three lines that
    a 74LS00 decodes to the ROM's chip enable (M. Hepperle's notes on the
    22707L); how A16 and A17 of a larger ROM are wired is not known.

    22707A, 22707B: 128 KiB and 256 KiB of RAM, used by the printer as
    input buffer or for downloaded soft fonts.  Not battery backed.

    The four LEDs (which of the cartridge's fonts is selected) are not
    emulated.

**********************************************************************/

#include "emu.h"
#include "cards.h"

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"


namespace {

class hp_dj_cart_rom_device : public device_t, public device_hp_dj_cart_interface
{
public:
	hp_dj_cart_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, HP_DJ_CART_ROM, tag, owner, clock),
		device_hp_dj_cart_interface(mconfig, *this),
		m_rom(*this, "cart")
	{
	}

	virtual u8 *page(int n) override
	{
		if (!m_rom->exists() || ((u32(n + 1) << 14) > m_rom->get_rom_size()))
			return nullptr;
		return m_rom->get_rom_base() + (n << 14);
	}

protected:
	virtual void device_start() override ATTR_COLD { }

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD
	{
		GENERIC_CARTSLOT(config, m_rom, generic_plain_slot, "hpdj_cart", "bin,rom");
	}

private:
	required_device<generic_slot_device> m_rom;
};


class hp_dj_cart_ram_device : public device_t, public device_hp_dj_cart_interface
{
public:
	virtual u8 *page(int n) override
	{
		return ((n << 14) < m_size) ? &m_ram[n << 14] : nullptr;
	}

	virtual bool writable() const override { return true; }

protected:
	hp_dj_cart_ram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, u32 size) :
		device_t(mconfig, type, tag, owner, clock),
		device_hp_dj_cart_interface(mconfig, *this),
		m_size(size)
	{
	}

	virtual void device_start() override ATTR_COLD
	{
		m_ram = make_unique_clear<u8 []>(m_size);
		save_pointer(NAME(m_ram), m_size);
	}

private:
	u32 const m_size;
	std::unique_ptr<u8 []> m_ram;
};

class hp_dj_cart_ram128_device : public hp_dj_cart_ram_device
{
public:
	hp_dj_cart_ram128_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		hp_dj_cart_ram_device(mconfig, HP_DJ_CART_RAM128, tag, owner, clock, 0x20000)
	{
	}
};

class hp_dj_cart_ram256_device : public hp_dj_cart_ram_device
{
public:
	hp_dj_cart_ram256_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		hp_dj_cart_ram_device(mconfig, HP_DJ_CART_RAM256, tag, owner, clock, 0x40000)
	{
	}
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_ROM, device_hp_dj_cart_interface, hp_dj_cart_rom_device, "hpdj_cart_rom", "HP DeskJet font cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM128, device_hp_dj_cart_interface, hp_dj_cart_ram128_device, "hpdj_cart_22707a", "HP 22707A 128K RAM cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM256, device_hp_dj_cart_interface, hp_dj_cart_ram256_device, "hpdj_cart_22707b", "HP 22707B 256K RAM cartridge")
