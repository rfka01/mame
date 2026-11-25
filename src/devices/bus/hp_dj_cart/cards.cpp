// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 500/550C cartridges

    Font cartridge: a ROM, from the start of the slot's window.  Each
    64 KiB block of a font ROM starts with a header (62 00 20 ..., the
    cartridge number and name); the printer's firmware finds the fonts by
    itself.  The edge connector carries A0-A15, the data lines, four LED
    lines and three lines that a 74LS00 decodes to the ROM's chip enable
    (M. Hepperle's notes on the 22707L); how A16 and A17 of a larger ROM
    are wired is not known.

    Slot options for the dumped font cartridges:

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

    ram128, ram256: the 22707A and 22707B RAM cartridges, 128 KiB and
    256 KiB, used by the printer as input buffer or for downloaded soft
    fonts.  Not battery backed.

    Not dumped (from https://www.nefec.org/upm/printers/chpdj.htm and the
    DeskJet 550C user's guide):
      22706A  Line Draw, Courier Italic
      22706B  Prestige Elite
      22706C  Letter Gothic
      22706D  Times Roman 10, 8, 5 and 4 point
      22706E  Times Roman 12 and 6 point
      22706F  Times Roman 14 and 7 point
      22706G  Helvetica 10, 8, 5 and 4 point
      22706H  Helvetica 12 and 6 point
      22706J  Helvetica 14 and 7 point
      22707K  Courier Landscape
      22706M  Presentations
      22706P  Times Roman 14, 12, 10, 8, 7, 6, 5 and 4 point
      22706Q  Helvetica 14, 12, 10, 8, 7, 6, 5 and 4 point
      22706R  Times Roman Collection 8, 10, 12 and 14 point
      22706T  Helvetica Collection 8, 10, 12 and 14 point
      22706U  Times Roman Headlines 15 and 30 point
      22706V  Helvetica Headlines 15 and 30 point
      22706W  WordPerfect Cartridge (CG Times 6-24 point, Univers 14-24
              point)
      C2109B  Global Text (CG Century Schoolbook 8 and 10 point, CG
              Triumvirate 10 and 14 point)
      C2109C  Garamond Collection 8, 10, 12 and 14 point
      C2109D  Dom Casual 12, 14, 18 and 24 point
      C2109E  Brush 12, 18 and 24 point
    The 22708C Times Roman/Helvetica soft fonts come on disk and need a
    RAM cartridge.

    The four LEDs (which of the cartridge's fonts is selected) are not
    emulated.

**********************************************************************/

#include "emu.h"
#include "cards.h"


namespace {

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


DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_ODSCYR, device_hp_dj_cart_interface, hp_dj_cart_odscyr_device, "hpdj_cart_odscyr", "HP DeskJet ODS Business Cartridge (Cyrillic)")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_22707L, device_hp_dj_cart_interface, hp_dj_cart_22707l_device, "hpdj_cart_22707l", "HP 22707L Letter Gothic landscape font cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM128, device_hp_dj_cart_interface, hp_dj_cart_ram128_device, "hpdj_cart_22707a", "HP 22707A 128K RAM cartridge")
DEFINE_DEVICE_TYPE_PRIVATE(HP_DJ_CART_RAM256, device_hp_dj_cart_interface, hp_dj_cart_ram256_device, "hpdj_cart_22707b", "HP 22707B 256K RAM cartridge")
