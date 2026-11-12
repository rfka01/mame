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

#include "bus/rs232/hlemouse.h"

/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

// K801 cable converters
//
// The K801 takes its connector wiring from a small plug-in board, the
// cable converter, that sits in a housing at the back of the card
// (Doc 017-0033070).  NCR shipped converters for a printer, a plotter
// and a modem ("CC01 COMMUNIC. MODEM"); the blank side of the modem
// converter is meant for a wiring of the user's own choice.  In MAME
// the three NCR converters pass every line straight through.
//
// The "user-wired" converter wires the blank side for a Microsoft or
// Logitech serial mouse: DTR is bridged to DCD and DSR.  Without that
// bridge a mouse cannot be read at all, because DCD/ of the 2661 comes
// from the cable through an MC1489 (A5 on 017-0032712), an open input
// reads as inactive, and the 2661 keeps its receiver off while DCD is
// inactive.  The mice drive neither line.  With the bridge, setting
// DTR (and RTS, which together power the mouse) in the command
// register also enables the receiver; the mouse then answers with "M"
// (Microsoft) or "M3" (Logitech), 7N1 at 1,200 baud.
//
// The mice are offered on the K801 only.  With an NCR converter a
// selected mouse stays silent, as it would on the real card.
static void dmv_k801_rs232_devices(device_slot_interface &device)
{
	default_rs232_devices(device);
	device.option_add("microsoft_mouse", MSFT_HLE_SERIAL_MOUSE);
	device.option_add("logitech_mouse",  LOGITECH_HLE_SERIAL_MOUSE);
}


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

	PORT_START("CABLE")
	PORT_CONFNAME( 0x03, 0x00, "K801 Cable Converter" )  PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dmv_k801_device::cable_changed), 0)
	PORT_CONFSETTING( 0x00, "Printer" )
	PORT_CONFSETTING( 0x01, "Plotter" )
	PORT_CONFSETTING( 0x02, "Modem (CC01)" )
	PORT_CONFSETTING( 0x03, "User-wired (serial mouse)" )
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
	m_dsw(*this, "DSW"),
	m_cable(*this, "CABLE"),
	m_pci_dtr(1),
	m_rs232_dcd(1),
	m_rs232_dsr(1)
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
	save_item(NAME(m_pci_dtr));
	save_item(NAME(m_rs232_dcd));
	save_item(NAME(m_rs232_dsr));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dmv_k801_device::device_reset()
{
	update_modem_inputs();
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void dmv_k801_device::pci_mconfig(machine_config &config, bool epci, const char *default_option, void (*slot_options)(device_slot_interface &))
{
	if (epci)
		SCN2661C(config, m_pci, XTAL(5'068'800));
	else
		SCN2651(config, m_pci, XTAL(5'068'800));
	m_pci->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_pci->rts_handler().set("rs232", FUNC(rs232_port_device::write_rts));
	m_pci->dtr_handler().set(FUNC(dmv_k801_device::pci_dtr_w));
	m_pci->rxrdy_handler().set(FUNC(dmv_k801_device::pci_irq_w));
	if (epci)
		m_pci->txrdy_handler().set(FUNC(dmv_k801_device::pci_irq_w));

	RS232_PORT(config, m_rs232, slot_options, default_option);
	m_rs232->rxd_handler().set(m_pci, FUNC(scn_pci_device::rxd_w));
	m_rs232->dcd_handler().set(FUNC(dmv_k801_device::rs232_dcd_w));
	m_rs232->dsr_handler().set(FUNC(dmv_k801_device::rs232_dsr_w));
	m_rs232->cts_handler().set(m_pci, FUNC(scn_pci_device::cts_w));
}

void dmv_k801_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, true, "printer", dmv_k801_rs232_devices);
}

// Measured on two machines with DMVFRAGE: the K211 interrupt follows
// RxRDY only (2651 local loopback raises it, enabling the transmitter
// does not), while the K801 follows TxRDY.  K212 and K213 share the
// 2651 setup and are assumed to be wired the same way.
void dmv_k211_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, "null_modem", default_rs232_devices);
}

void dmv_k212_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, "printer", default_rs232_devices);
}

void dmv_k213_device::device_add_mconfig(machine_config &config)
{
	pci_mconfig(config, false, nullptr, default_rs232_devices);
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

// Only the K801 has a CABLE port; the K211 family always passes the
// modem lines straight through.
bool dmv_k801_device::user_wired_cable() const
{
	return m_cable && (m_cable->read() & 0x03) == 0x03;
}

void dmv_k801_device::update_modem_inputs()
{
	if (user_wired_cable())
	{
		m_pci->dcd_w(m_pci_dtr);
		m_pci->dsr_w(m_pci_dtr);
	}
	else
	{
		m_pci->dcd_w(m_rs232_dcd);
		m_pci->dsr_w(m_rs232_dsr);
	}
}

void dmv_k801_device::pci_dtr_w(int state)
{
	m_pci_dtr = state;
	m_rs232->write_dtr(state);
	if (user_wired_cable())
		update_modem_inputs();
}

void dmv_k801_device::rs232_dcd_w(int state)
{
	m_rs232_dcd = state;
	if (!user_wired_cable())
		m_pci->dcd_w(state);
}

void dmv_k801_device::rs232_dsr_w(int state)
{
	m_rs232_dsr = state;
	if (!user_wired_cable())
		m_pci->dsr_w(state);
}

INPUT_CHANGED_MEMBER(dmv_k801_device::cable_changed)
{
	update_modem_inputs();
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
