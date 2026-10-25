// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Okidata Okimate 20 thermal transfer colour printer, Commodore module
    (logic board and mechanism: machine/okimate20.cpp)

    The Commodore firmware talks to the serial bus directly through
    port 3 and uses the M60306 only for its interval timer and the print
    head solenoid.  Other Plug 'n Print modules carry a PROM at f000
    (P1.7 low) which takes INT0; not emulated.
    The Commodore MCS 810 is the same printer with firmware CO 02.5
    (CO2.5G1 differs in a few bytes).

      data    c000-c007  M60307

      P1.0-5    function switches (low = on), switch 6 selects device 4 or 5
      P3.0      serial bus DATA in (inverted)
      P3.1      serial bus DATA out (high pulls DATA low)
      P3.4      serial bus ATN in (inverted)
      P3.5      serial bus CLK in (inverted)
      INT1      carriage step (M60307)

    TODO:
    - meanings of function switches 1-5 for the Commodore firmware

**********************************************************************/

#include "emu.h"
#include "okimate20.h"
#include "mcs810.h"

#include "machine/okimate20.h"

#define LOG_PORT (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

class okimate20_device : public okimate20_base_device, public device_cbm_iec_interface
{
public:
	okimate20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		okimate20_device(mconfig, OKIMATE20, tag, owner, clock)
	{
	}

	static constexpr flags_type emulation_flags() { return flags::NOT_WORKING; }

protected:
	okimate20_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
		okimate20_base_device(mconfig, type, tag, owner, clock, 0xc000),
		device_cbm_iec_interface(mconfig, *this),
		m_sw(*this, "SW"),
		m_p1(0xff),
		m_p3(0xff),
		m_p3in(0)
	{
	}

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void cbm_iec_reset(int state) override { if (!state) reset_cpu(); }

	virtual u8 port1_r() override;
	virtual void port1_w(u8 data) override;
	virtual u8 port3_r() override;
	virtual void port3_w(u8 data) override;

private:
	required_ioport m_sw;
	u8 m_p1, m_p3, m_p3in;
};


// Commodore MCS 810: the Okimate 20 with Commodore's firmware revision CO 02.5
class mcs810_device : public okimate20_device
{
public:
	mcs810_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		okimate20_device(mconfig, MCS810, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};


ROM_START( okimat20 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("co25g1.bin", 0x0000, 0x4000, CRC(050b4590) SHA1(a7558326cb0c7879424f9a5d861ed4e6da5d189f))
ROM_END

ROM_START( mcs810 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("mcs810-65-1115.bin", 0x0000, 0x4000, CRC(7d88d6f5) SHA1(b62396eba1910bc809979dd5444a16c7ffd431c5))
ROM_END

const tiny_rom_entry *okimate20_device::device_rom_region() const
{
	return ROM_NAME( okimat20 );
}

const tiny_rom_entry *mcs810_device::device_rom_region() const
{
	return ROM_NAME( mcs810 );
}


INPUT_PORTS_START( okimat20 )
	PORT_INCLUDE( okimate20 )

	PORT_START("SW")    // function switches, read through P1 (1 = ON); meanings for the Commodore firmware to be confirmed
	PORT_DIPNAME( 0x01, 0x00, "SW1" )             PORT_DIPLOCATION("SW:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "SW2" )             PORT_DIPLOCATION("SW:2")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "SW3" )             PORT_DIPLOCATION("SW:3")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "SW4" )             PORT_DIPLOCATION("SW:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "SW5" )             PORT_DIPLOCATION("SW:5")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, "Device number" )   PORT_DIPLOCATION("SW:6")
	PORT_DIPSETTING(    0x00, "4" )
	PORT_DIPSETTING(    0x20, "5" )
INPUT_PORTS_END

ioport_constructor okimate20_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( okimat20 );
}


void okimate20_device::device_start()
{
	okimate20_base_device::device_start();

	save_item(NAME(m_p1));
	save_item(NAME(m_p3));
	save_item(NAME(m_p3in));
}

void okimate20_device::device_reset()
{
	okimate20_base_device::device_reset();
	m_p3 = 0xff;
}

u8 okimate20_device::port1_r()
{
	// function switches (low = ON, mapping to be verified), P1.7 high: no module ROM
	return u8(~m_sw->read() | 0x80);
}

void okimate20_device::port1_w(u8 data)
{
	if (data != m_p1)
		LOGMASKED(LOG_PORT, "%s P1 %02x\n", machine().describe_context(), data);
	m_p1 = data;
}

u8 okimate20_device::port3_r()
{
	// serial bus lines through inverting buffers
	u8 const v = u8(0xce | (m_bus->data_r() ? 0x00 : 0x01) | (m_bus->atn_r() ? 0x00 : 0x10) | (m_bus->clk_r() ? 0x00 : 0x20));
	if (v != m_p3in && !machine().side_effects_disabled())
	{
		LOGMASKED(LOG_PORT, "%s P3 in %02x  t %s\n", machine().describe_context(), v, machine().time().as_string(6));
		m_p3in = v;
	}
	return v;
}

void okimate20_device::port3_w(u8 data)
{
	if (data != m_p3)
		LOGMASKED(LOG_PORT, "%s P3 %02x atn %d clk %d data %d  t %s\n", machine().describe_context(), data, m_bus->atn_r(), m_bus->clk_r(), m_bus->data_r(), machine().time().as_string(6));
	m_p3 = data;
	m_bus->data_w(this, !BIT(data, 1));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(OKIMATE20, device_cbm_iec_interface, okimate20_device, "okimat20", "Okidata Okimate 20 (Commodore)")
DEFINE_DEVICE_TYPE_PRIVATE(MCS810,    device_cbm_iec_interface, mcs810_device,    "mcs810",   "Commodore MCS 810 Color Printer")
