// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
/***************************************************************************

    K801 RS-232 Switchable Interface
    K211 RS-232 Communications Interface
    K212 RS-232 Printer Interface
    K213 RS-232 Plotter Interface

    K211, K212 and K213 have same board, but different cables.
    K801 uses a 2661 instead of the 2651 and has 4 switches for
    select the IFSEL.

    Addressing, from the schematics:

    - K211/K212/K213 (017-0031865): one jumper picks the interface
      select, J1 for IFSEL1/ (a28) and J2 for IFSEL0/ (c28); the plan
      lists J1 for the K211 modem cable, J2 for the K212 printer cable
      and the K213 plotter cable.  The 2651 gets A0 (c26), A1 (a26)
      and A2 (c25) - the latter on its R/W pin, which is why the lower
      four ports of a half read the chip and the upper four write it.
      BA3 (a25) is not connected, so the board answers on BOTH halves
      of its group, mirrored four times over sixteen ports.

    - K801 (017-0032712): BD3 (a25) runs through switch S1 into a
      74LS136, and the five ISEL lines go through a 74LS251 onto CE/.
      The card therefore COMPARES address bit 3 with the switch and
      occupies a single half.  Its four switches are labelled 4, 2, 1
      and B in the manual: the first three pick IFSEL 0..4, the last
      one picks the half.

    NCR's "Interface Configuration" table (Doc 017-0033059) shows the
    same thing: the bars for PRINTER 1 (K210/K212) and COMMUNICATION
    (K211) run from the A column into the B column, with the legend
    "Software and Hardware" for the A half and "Hardware" for the B
    half, while the K801 gets a separate single bar in each of the ten
    halves.

***************************************************************************/

#include "emu.h"
#include "k801.h"

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/


static INPUT_PORTS_START( dmv_k801 )
	PORT_START("DSW")
	// NCR's interface configuration assigns K210/K212/K213 to 0A (and
	// 0B through the mirroring above), K211 to 1A (and 1B), K215 to
	// 1A, K806 to 2A, K804 to 2B, K600 to 3B, the fixed disk to 4A and
	// K803 to 4B; 3A is the K801 itself when it drives a p-System
	// plotter.  Default 2B, which is free in any machine without the
	// IEEE-488 card - 3A and 3B are equally good.  0B and 1B are not,
	// because the K210 and the K211 family answer there as well.
	PORT_DIPNAME( 0x0f, 0x05, "K801 IFSEL" )  PORT_DIPLOCATION("S:!4,S:!3,S:!2,S:!1")
	PORT_DIPSETTING( 0x00, "0A" )
	PORT_DIPSETTING( 0x01, "0B" )
	PORT_DIPSETTING( 0x02, "1A" )
	PORT_DIPSETTING( 0x03, "1B" )
	PORT_DIPSETTING( 0x04, "2A" )
	PORT_DIPSETTING( 0x05, "2B" )   // default
	PORT_DIPSETTING( 0x06, "3A" )
	PORT_DIPSETTING( 0x07, "3B" )
	PORT_DIPSETTING( 0x08, "4A" )
	PORT_DIPSETTING( 0x09, "4B" )
INPUT_PORTS_END

static INPUT_PORTS_START( dmv_k211 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x03, 0x02, "K211 Jumpers" )  PORT_DIPLOCATION("J:2,J:1")
	PORT_DIPSETTING( 0x01, "IFSEL 0" )
	PORT_DIPSETTING( 0x02, "IFSEL 1" )
INPUT_PORTS_END

static INPUT_PORTS_START( dmv_k212 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x03, 0x01, "K212 Jumpers" )  PORT_DIPLOCATION("J:2,J:1")
	PORT_DIPSETTING( 0x01, "IFSEL 0" )
	PORT_DIPSETTING( 0x02, "IFSEL 1" )
INPUT_PORTS_END

static INPUT_PORTS_START( dmv_k213 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x03, 0x01, "K213 Jumpers" )  PORT_DIPLOCATION("J:2,J:1")
	PORT_DIPSETTING( 0x01, "IFSEL 0" )
	PORT_DIPSETTING( 0x02, "IFSEL 1" )
INPUT_PORTS_END


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DMV_K801, dmv_k801_device, "dmv_k801", "K801 RS-232 Switchable Interface")
DEFINE_DEVICE_TYPE(DMV_K211, dmv_k211_device, "dmv_k211", "K211 RS-232 Communications Interface")
DEFINE_DEVICE_TYPE(DMV_K212, dmv_k212_device, "dmv_k212", "K212 RS-232 Printer Interface")
DEFINE_DEVICE_TYPE(DMV_K213, dmv_k213_device, "dmv_k213", "K213 RS-232 Plotter Interface")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dmv_k801_device - constructor
//-------------------------------------------------

dmv_k801_device::dmv_k801_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k801_device(mconfig, DMV_K801, tag, owner, clock)
{
}

dmv_k801_device::dmv_k801_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock),
	device_dmvslot_interface( mconfig, *this ),
	m_pci(*this, "pci"),
	m_rs232(*this, "rs232"),
	m_dsw(*this, "DSW")
{
}

//-------------------------------------------------
//  dmv_k211_device - constructor
//-------------------------------------------------

dmv_k211_device::dmv_k211_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k211_device(mconfig, DMV_K211, tag, owner, clock)
{
}


dmv_k211_device::dmv_k211_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k801_device(mconfig, type, tag, owner, clock)
{
}

//-------------------------------------------------
//  dmv_k212_device - constructor
//-------------------------------------------------

dmv_k212_device::dmv_k212_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k211_device(mconfig, DMV_K212, tag, owner, clock)
{
}

//-------------------------------------------------
//  dmv_k213_device - constructor
//-------------------------------------------------

dmv_k213_device::dmv_k213_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k211_device(mconfig, DMV_K213, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dmv_k801_device::device_start()
{
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dmv_k801_device::device_reset()
{
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void dmv_k801_device::pci_mconfig(machine_config &config, bool epci, const char *default_option)
{
	if (epci)
		SCN2661C(config, m_pci, XTAL(5'068'800));
	else
		SCN2651(config, m_pci, XTAL(5'068'800));
	m_pci->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_pci->rts_handler().set("rs232", FUNC(rs232_port_device::write_rts));
	m_pci->dtr_handler().set("rs232", FUNC(rs232_port_device::write_dtr));
	m_pci->rxrdy_handler().set(FUNC(dmv_k801_device::pci_irq_w));
	m_pci->txrdy_handler().set(FUNC(dmv_k801_device::pci_irq_w));

	RS232_PORT(config, m_rs232, default_rs232_devices, default_option);
	m_rs232->rxd_handler().set(m_pci, FUNC(scn_pci_device::rxd_w));
	m_rs232->dcd_handler().set(m_pci, FUNC(scn_pci_device::dcd_w));
	m_rs232->dsr_handler().set(m_pci, FUNC(scn_pci_device::dsr_w));
	m_rs232->cts_handler().set(m_pci, FUNC(scn_pci_device::cts_w));
}

void dmv_k801_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, true, "printer");
}

void dmv_k211_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, "null_modem");
}

void dmv_k212_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, "printer");
}

void dmv_k213_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, nullptr);
}

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor dmv_k801_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k801 );
}

ioport_constructor dmv_k211_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k211 );
}

ioport_constructor dmv_k212_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k212 );
}

ioport_constructor dmv_k213_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k213 );
}

void dmv_k801_device::pci_irq_w(int state)
{
	out_irq(state);
}

void dmv_k801_device::io_read(int ifsel, offs_t offset, uint8_t &data)
{
	uint8_t dsw = m_dsw->read() & 0x0f;
	if ((dsw >> 1) == ifsel && BIT(offset, 3) == BIT(dsw, 0))
	{
		if (offset & 0x04)
			m_pci->write(offset & 0x03, data);
		else
			data = m_pci->read(offset & 0x03);
	}
}

void dmv_k801_device::io_write(int ifsel, offs_t offset, uint8_t data)
{
	uint8_t dsw = m_dsw->read() & 0x0f;
	if ((dsw >> 1) == ifsel && BIT(offset, 3) == BIT(dsw, 0))
	{
		if (offset & 0x04)
			m_pci->write(offset & 0x03, data);
		else
			data = m_pci->read(offset & 0x03);
	}
}

void dmv_k211_device::io_read(int ifsel, offs_t offset, uint8_t &data)
{
	uint8_t jumpers = m_dsw->read() & 0x03;
	if ((BIT(jumpers, 0) && ifsel == 0) || (BIT(jumpers, 1) && ifsel == 1))
	{
		if (offset & 0x04)
			m_pci->write(offset & 0x03, data);
		else
			data = m_pci->read(offset & 0x03);
	}
}

void dmv_k211_device::io_write(int ifsel, offs_t offset, uint8_t data)
{
	uint8_t jumpers = m_dsw->read() & 0x03;
	if ((BIT(jumpers, 0) && ifsel == 0) || (BIT(jumpers, 1) && ifsel == 1))
	{
		if (offset & 0x04)
			m_pci->write(offset & 0x03, data);
		else
			data = m_pci->read(offset & 0x03);
	}
}
