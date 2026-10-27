// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    K215 Buffered sync/async RS-232C adapter

    HIGH LEVEL EMULATION until the 8741 is dumped.

    The board (assembly 017-0032782 D, schematic 017-0032792 B, FCC ID
    CTD7YSNCR3273K215): Intel D8741A with NCR firmware 32905, SCN26x1
    USART (marking partly hidden; V1K215's register values fit a 2651),
    HM6116 2 KB static RAM, 5.0688 MHz crystal oscillator,
    74LS244/LS05/LS32/LS175/LS377/LS02, SN75188/75189 line driver and
    receiver, ICL7660 for the negative supply.  The bus connector
    (Hardware Reference A-1-90) carries BD0-BD7, IOR/, IOW/, RESET/,
    BA0, BA3, IFSEL0/-IFSEL4/, PERC/, AUTO/ and IDSEL/, but no INT/:
    the card does not interrupt.  Jumper block J1 picks the IFSEL and
    the half (A-1-91), so the card occupies eight ports, x0-x7 or
    x8-xF; with only BA0 decoded the 8741's data port is mirrored on
    the even and its command/status port on the odd addresses.

    The protocol comes from NCR's diagnostic program V1K215
    ("3273-K215 RS232 IF DIAGNOSTICS PROGRAM, VERSION 001") and is only
    as complete as that program's use of the card.  Software that uses
    further commands will not work.

    - Data at x0, command (write) and status (read) at x1.  Default
      IFSEL 1A (70h); the program offers 70, 78, 30, 38, B0, B8 and C8.
    - Status as in any UPI-41: bit 0 = output buffer full (a result is
      waiting at x0), bit 1 = input buffer full (wait before writing),
      bit 3 = F1, address bit 0 of the last write.  An idle card reads
      00h: the program uses that to find it at the default port.
    - Results are read at x0 after waiting for status bit 0.  Commands
      that take a byte get it at x0 after the command.

      00h + byte  write the USART command register
      01h + byte  write the USART mode register (MR1, then MR2)
      03h         read the USART status; bit 1 (RxRDY) means "receive
                  buffer not empty"
      07h + byte  write the USART SYN/DLE registers (SYN1, SYN2, DLE)
      08h         read one character from the receive buffer
      09h + byte  transmit one character
      0Bh + byte  echo the byte ("DMV-BUS TEST", 55h and AAh)
      10h         self test: 0 = OK, 1 = USART error, 2 = 2K buffer
                  error, others = internal processor error

    - The register values (command 27h/32h/A3h/A7h, mode 6Eh/35h and
      6Eh/3Fh, synchronous 0Ch/3Ah with SYN 32h) are those of a 2651,
      so the HLE uses MAME's SCN2651 with the usual 5.0688 MHz crystal.
    - The receive buffer holds 2 KB: the "2K-BUFFER TEST" sends 0800h
      bytes through the turnaround adapter (use the "loopback" RS-232
      option) and reads them back afterwards.

    Not known: further commands, the timing of the host interface.  Each byte from the host is taken
    50 microseconds after it was written (a guess; V1K215 passes with
    1 microsecond as well).

***************************************************************************/

#include "emu.h"
#include "k215.h"


static INPUT_PORTS_START( dmv_k215 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x0f, 0x02, "K215 IFSEL" )
	PORT_DIPSETTING( 0x00, "0A" )
	PORT_DIPSETTING( 0x01, "0B" )
	PORT_DIPSETTING( 0x02, "1A" )   // default (V1K215)
	PORT_DIPSETTING( 0x03, "1B" )
	PORT_DIPSETTING( 0x04, "2A" )
	PORT_DIPSETTING( 0x05, "2B" )
	PORT_DIPSETTING( 0x06, "3A" )
	PORT_DIPSETTING( 0x07, "3B" )
	PORT_DIPSETTING( 0x08, "4A" )
	PORT_DIPSETTING( 0x09, "4B" )
INPUT_PORTS_END


DEFINE_DEVICE_TYPE(DMV_K215, dmv_k215_device, "dmv_k215", "K215 Buffered RS-232C Interface (HLE)")


dmv_k215_device::dmv_k215_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DMV_K215, tag, owner, clock)
	, device_dmvslot_interface(mconfig, *this)
	, m_pci(*this, "pci")
	, m_rs232(*this, "rs232")
	, m_dsw(*this, "DSW")
	, m_rx_head(0), m_rx_count(0), m_tx_head(0), m_tx_count(0)
	, m_status(0), m_out(0), m_cmd(0xff), m_last_cmd(0xff), m_overrun(false)
	, m_in(0), m_in_cmd(false)
{
}

void dmv_k215_device::device_start()
{
	m_rx_timer = timer_alloc(FUNC(dmv_k215_device::rx_fetch), this);
	m_cmd_timer = timer_alloc(FUNC(dmv_k215_device::host_byte), this);

	save_item(NAME(m_rxbuf));
	save_item(NAME(m_txbuf));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_tx_head));
	save_item(NAME(m_tx_count));
	save_item(NAME(m_status));
	save_item(NAME(m_out));
	save_item(NAME(m_cmd));
	save_item(NAME(m_last_cmd));
	save_item(NAME(m_overrun));
	save_item(NAME(m_in));
	save_item(NAME(m_in_cmd));
}

void dmv_k215_device::device_reset()
{
	m_rx_head = m_rx_count = 0;
	m_tx_head = m_tx_count = 0;
	m_status = 0;
	m_out = 0;
	m_cmd = 0xff;
	m_last_cmd = 0xff;
	m_overrun = false;
	m_cmd_timer->adjust(attotime::never);
}

void dmv_k215_device::device_add_mconfig(machine_config &config)
{
	SCN2651(config, m_pci, XTAL(5'068'800));
	m_pci->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_pci->rts_handler().set("rs232", FUNC(rs232_port_device::write_rts));
	m_pci->dtr_handler().set("rs232", FUNC(rs232_port_device::write_dtr));
	m_pci->rxrdy_handler().set(FUNC(dmv_k215_device::rxrdy_w));
	m_pci->txrdy_handler().set(FUNC(dmv_k215_device::txrdy_w));

	RS232_PORT(config, m_rs232, default_rs232_devices, "null_modem");
	m_rs232->rxd_handler().set(m_pci, FUNC(scn_pci_device::rxd_w));
	m_rs232->dcd_handler().set(m_pci, FUNC(scn_pci_device::dcd_w));
	m_rs232->dsr_handler().set(m_pci, FUNC(scn_pci_device::dsr_w));
	m_rs232->cts_handler().set(m_pci, FUNC(scn_pci_device::cts_w));
}

ioport_constructor dmv_k215_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k215 );
}

bool dmv_k215_device::selected(int ifsel, offs_t offset) const
{
	uint8_t const dsw = m_dsw->read() & 0x0f;
	return (dsw >> 1) == ifsel && BIT(offset, 3) == BIT(dsw, 0);
}

void dmv_k215_device::io_read(int ifsel, offs_t offset, uint8_t &data)
{
	if (!selected(ifsel, offset))
		return;

	if (BIT(offset, 0))
	{
		data = m_status;
	}
	else
	{
		data = m_out;
		if (!machine().side_effects_disabled())
			m_status &= ~0x01;
	}
}

void dmv_k215_device::io_write(int ifsel, offs_t offset, uint8_t data)
{
	if (!selected(ifsel, offset))
		return;

	// the card's processor takes a while for each byte: status bit 1
	// stays set until it has taken it
	m_in = data;
	m_in_cmd = BIT(offset, 0);
	m_status = (m_status & ~0x08) | 0x02 | (BIT(offset, 0) ? 0x08 : 0x00);
	m_cmd_timer->adjust(attotime::from_usec(50));
}

TIMER_CALLBACK_MEMBER(dmv_k215_device::host_byte)
{
	m_status &= ~0x02;
	if (m_in_cmd)
		command_w(m_in);
	else
		data_w(m_in);
}

void dmv_k215_device::result(uint8_t data)
{
	m_out = data;
	m_status |= 0x01;
}

void dmv_k215_device::command_w(uint8_t data)
{
	m_cmd = 0xff;

	switch (data)
	{
	case 0x01: case 0x07:
		// MR1/MR2 and SYN1/SYN2/DLE are written in sequence; reading the
		// command register resets the 2651's pointers
		if (m_last_cmd != data)
			m_pci->read(3);
		m_cmd = data;
		break;

	case 0x00: case 0x09: case 0x0b:
		m_cmd = data;           // wait for the data byte
		break;

	case 0x03:
	{
		uint8_t status = m_pci->read(1);
		status = (status & ~0x02) | (m_rx_count ? 0x02 : 0x00);
		if (m_overrun)
			status |= 0x10;
		result(status);
		break;
	}

	case 0x08:
		if (m_rx_count)
		{
			result(m_rxbuf[m_rx_head]);
			m_rx_head = (m_rx_head + 1) % m_rxbuf.size();
			m_rx_count--;
		}
		else
		{
			result(0x00);
		}
		break;

	case 0x10:
		result(0x00);           // self test passed
		break;

	default:
		logerror("unknown command %02X\n", data);
		break;
	}

	m_last_cmd = data;
}

void dmv_k215_device::data_w(uint8_t data)
{
	uint8_t const cmd = m_cmd;
	m_cmd = 0xff;

	switch (cmd)
	{
	case 0x00:
		m_pci->write(3, data);
		if (BIT(data, 4))
			m_overrun = false;
		break;

	case 0x01:
		m_pci->write(2, data);
		break;

	case 0x07:
		m_pci->write(1, data);
		break;

	case 0x09:
		if (m_tx_count < m_txbuf.size())
		{
			m_txbuf[(m_tx_head + m_tx_count) % m_txbuf.size()] = data;
			m_tx_count++;
		}
		tx_pump();
		break;

	case 0x0b:
		result(data);
		break;

	default:
		logerror("data %02X without command\n", data);
		break;
	}
}

void dmv_k215_device::rxrdy_w(int state)
{
	if (state)              // fetch the byte outside the 2651's callback
		m_rx_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(dmv_k215_device::rx_fetch)
{
	if (m_pci->rxrdy_r())
		return;

	uint8_t const data = m_pci->read(0);
	if (m_rx_count < m_rxbuf.size())
	{
		m_rxbuf[(m_rx_head + m_rx_count) % m_rxbuf.size()] = data;
		m_rx_count++;
	}
	else
	{
		m_overrun = true;
	}
}

void dmv_k215_device::txrdy_w(int state)
{
	if (state)
		tx_pump();
}

void dmv_k215_device::tx_pump()
{
	if (m_tx_count && !m_pci->txrdy_r())
	{
		m_pci->write(0, m_txbuf[m_tx_head]);
		m_tx_head = (m_tx_head + 1) % m_txbuf.size();
		m_tx_count--;
	}
}
