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

    The dumped font cartridges are options of their own, with their ROMs:

    odscyr  ODS Business Cartridge (Cyrillic), B2543A Opt. ACB, PCB
            C2114-60064 (DJ100 MK5-AFU REV B): Courier, Courier Italic,
            Courier Landscape, CG Times, CG Times Italic, 12 point, symbol
            sets 3R (CP866), 10N (ISO 8859-5), 9R (Windows 1251).  The
            firmware finds the italic Courier only in the window's second
            block, so the ROM's 64 KiB blocks are loaded in the order
            0, 3, 2, 1; the board selects the ROM's upper address lines
            with a 74LS139 (Guru), the wiring is not traced.  Board from
            Artiom, ROM dumped by Guru.
    22707l  Letter Gothic landscape fonts, HP 22707L (MB831000): Letter
            Gothic 12 pitch upright and italic, 16.67 pitch, and Courier
            10 pitch, all landscape.  ROM files by M. Hepperle.

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


class hp_dj_cart_font_device : public device_t, public device_hp_dj_cart_interface
{
public:
	virtual u8 *page(int n) override
	{
		return ((u32(n + 1) << 14) <= m_rom.bytes()) ? &m_rom[n << 14] : nullptr;
	}

protected:
	hp_dj_cart_font_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, type, tag, owner, clock),
		device_hp_dj_cart_interface(mconfig, *this),
		m_rom(*this, "rom")
	{
	}

	virtual void device_start() override ATTR_COLD { }

private:
	required_region_ptr<u8> m_rom;
};


ROM_START( odscyr )
	ROM_REGION(0x40000, "rom", 0)
	ROM_LOAD("3233cyr.u2", 0x00000, 0x10000, CRC(3ba7023c) SHA1(b268b1c17b525a478d56ede90fdfca6a89634219))
	ROM_CONTINUE(          0x30000, 0x10000)
	ROM_CONTINUE(          0x20000, 0x10000)
	ROM_CONTINUE(          0x10000, 0x10000)
ROM_END

class hp_dj_cart_odscyr_device : public hp_dj_cart_font_device
{
public:
	hp_dj_cart_odscyr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		hp_dj_cart_font_device(mconfig, HP_DJ_CART_ODSCYR, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD { return ROM_NAME(odscyr); }
};


ROM_START( 22707l )
	ROM_REGION(0x20000, "rom", 0)
	ROM_LOAD("hp22707l.bin", 0x00000, 0x20000, CRC(a6526fbe) SHA1(3cadca569e6e1f4be117f385706a0a5ffc7c42f1))
ROM_END

class hp_dj_cart_22707l_device : public hp_dj_cart_font_device
{
public:
	hp_dj_cart_22707l_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		hp_dj_cart_font_device(mconfig, HP_DJ_CART_22707L, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD { return ROM_NAME(22707l); }
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
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_ODSCYR, device_hp_dj_cart_interface, hp_dj_cart_odscyr_device, "hpdj_cart_odscyr", "HP DeskJet ODS Business Cartridge (Cyrillic)")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_22707L, device_hp_dj_cart_interface, hp_dj_cart_22707l_device, "hpdj_cart_22707l", "HP 22707L Letter Gothic landscape font cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM128, device_hp_dj_cart_interface, hp_dj_cart_ram128_device, "hpdj_cart_22707a", "HP 22707A 128K RAM cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM256, device_hp_dj_cart_interface, hp_dj_cart_ram256_device, "hpdj_cart_22707b", "HP 22707B 256K RAM cartridge")
