// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet Portable font cards

    x552  X552 Font Cartridge For Deskjet Portable, 5181-7100, B2589A
          Opt. ACB, (C) Hewlett Packard 1993: Courier 12 and CG Times 12
          in the PC Cyrillic (3R), ISO 8859/5 (10N) and Windows Latin Cyr
          (9R) symbol sets.  PCB 90TH512KP1:0, SMART (C) 1995, one Intel
          E28F020-150 flash at U1, a second TSOP32 position at U2 not
          populated.  Dumped by Guru.  The fonts are those of the DeskJet
          500 ODS Business Cartridge (Cyrillic) (bus/hp_dj_cart, odscyr,
          font ID 3233CYR), with the 128 KiB halves in the other order.

    The firmware finds the fonts by itself: each 64 KiB block starts with
    a header (62 00 20 ..., the font ID and the names).

**********************************************************************/

#include "emu.h"
#include "cards.h"


namespace {

class hp_djp_card_rom_device : public device_t, public device_hp_djp_card_interface
{
public:
	virtual u8 const *page(int n) override
	{
		offs_t const offset = offs_t(n) << 14;
		return (offset < m_rom.bytes()) ? &m_rom[offset] : nullptr;
	}

protected:
	hp_djp_card_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, type, tag, owner, clock),
		device_hp_djp_card_interface(mconfig, *this),
		m_rom(*this, "rom")
	{
	}

	virtual void device_start() override ATTR_COLD { }

private:
	required_region_ptr<u8> m_rom;
};


ROM_START( x552 )
	ROM_REGION(0x40000, "rom", 0)
	ROM_LOAD("e28f020.u1", 0x00000, 0x40000, CRC(26c76a22) SHA1(6c9161c82f3f88e170bc3f699fc90d786906f1ce))
ROM_END

class hp_djp_card_x552_device : public hp_djp_card_rom_device
{
public:
	hp_djp_card_x552_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		hp_djp_card_rom_device(mconfig, HP_DJP_CARD_X552, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD { return ROM_NAME(x552); }
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(HP_DJP_CARD_X552, device_hp_djp_card_interface, hp_djp_card_x552_device, "hpdjp_card_x552", "HP X552 Cyrillic font card for the DeskJet Portable")
