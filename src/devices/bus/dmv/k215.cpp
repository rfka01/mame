// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    K215 Buffered sync/async RS-232C adapter

    Labelled "Buffered Async/Bisync Adapter + Cable, KIT 3273-K215-V001"
    (603-6091341).  The board (assembly 017-0032782 D, schematic
    017-0032792 B, FCC ID CTD7YSNCR3273K215): Intel D8741A with NCR
    firmware 32905, Signetics SCN2651 USART, HM6116P-3 2 KB static RAM,
    a 5.0688 MHz crystal oscillator can, 74LS244/LS05/LS32/LS175/LS377/
    LS02, SN75188 line driver and a 75189 type receiver, ICL7660 for the
    negative supply.  The cable is the standard RS-232 modem cable
    (Hardware Reference A-1-91, built after schematic 017-0031865).

    The bus connector (Hardware Reference A-1-90) carries +5V, +12V,
    BD0-BD7, IOR/, IOW/, RESET/, BA0 (c26), BA3 (a25), IFSEL0/-IFSEL4/,
    PERC/ (a2), AUTO/ (a13), IDSEL/ (c11) and LGRD; there is no INT/
    (c31).  Jumper block J1 picks the IFSEL and the half (A-1-91):
    11-12 = A (x0-x7), 13-14 = B (x8-xF), and 9-10, 7-8, 5-6, 3-4, 1-2 =
    IFSEL 0 to 4.  With only BA0 decoded the 8741's data port is
    mirrored on the even and its command/status port on the odd
    addresses.

    8741 wiring, as the firmware drives it (disassembly of 32905):

    - P1 is the data bus to the 2651 and the 6116; the firmware writes
      FFh to it after every access.
    - P2.0  2651 CE/
      P2.1  2651 A0
      P2.2  2651 A1
      P2.3  6116 CS/
      P2.4  clock of the LS377, RAM address A0-A7 from P1
      P2.5  clock of the LS175, RAM address A8-A10 from P1.0-P1.2
      P2.6  R/W for both, low = write: the 2651 takes the data on the
            rising edge of CE/, the 6116 when CS/ and P2.6 go high again
      P2.7  receive request: low while the receiver is enabled (bit 2 of
            the last 2651 command written by the host) and the buffer
            holds characters.  PERC/ on a2 is the slot interrupt line
            that the K235 pulls up, and P2.7 is the only port line the
            firmware has left for it, so it is taken to drive PERC/
            through one of the LS05's open collector inverters.  The
            exact gate is not traced.
    - T1 is the 2651's RxRDY/.  The firmware runs the timer as an event
      counter preloaded with FFh, so the first falling edge overflows
      it and the timer interrupt (007h) flags the received character.
    - The oscillator can is the only clock source on the board, so the
      8741 runs from the same 5.0688 MHz as the 2651.

    Firmware, from the disassembly:

    - Self test after reset: ROM sum over all 1024 bytes (must be 00h),
      internal RAM 04h-3Fh with FFh and 00h, the 6116 with 55h/ABh,
      AAh/56h and an address pattern, then two 10h writes to the 2651
      command register and a read back that must give 10h.  On an
      error it waits for command 10h, answers with the error code and
      restarts: 1 = USART, 2 = 6116, 3 = ROM sum, 4 = internal RAM.
      The test takes about 670000 machine cycles, two seconds at
      5.0688 MHz; a byte the host writes in that time is thrown away
      when it ends (015h).
    - Host commands (written to x1, a data byte follows at x0 where
      noted, results are read at x0 after status bit 0):

      00h + byte  2651 command register; bit 2 (RxEN) also enables the
                  receive request on P2.7
      01h + byte  2651 mode register (MR1, then MR2); 02h does the same
      03h         2651 status, bit 1 replaced by "buffer not empty"
      04h         2651 command register, read back
      05h, 06h    2651 mode register, read back
      07h + byte  2651 SYN1/SYN2/DLE register
      08h         next character from the buffer
      09h + byte  2651 transmit holding register, written at once:
                  the host has to test TxRDY (status bit 0) first
      0Bh + byte  echo
      10h         self test result, 00h when it passed
      0Ah, 0Ch-0Fh jump into the ROM sum loop and from there into the
                  self test, which overwrites the buffer

    - The receive buffer is a ring of 2048 bytes with 11-bit pointers
      at internal RAM 20h/21h (write) and 22h/23h (read).  There is no
      overflow check: the 2049th unread character overwrites the oldest
      one and the buffer then looks empty.  Command 08h does not check
      either; reading an empty buffer returns stale data and moves the
      read pointer past the write pointer.
    - The status byte at x1 is the plain UPI-41 one (OBF, IBF, F0, F1);
      the firmware never writes the upper bits.
    - A routine at 27Bh/2A1h sends XOFF (13h) and XON (11h) when the
      buffer reaches a fill mark, but nothing calls it.

    The protocol matches NCR's diagnostic program V1K215 ("3273-K215
    RS232 IF DIAGNOSTICS PROGRAM, VERSION 001").  NCR's factory setting
    is IFSEL 1A (70h), but the K211 that most machines carry takes all
    of IFSEL 1, so the emulation defaults to 3A (B0h), the free half
    that the program also offers (70, 78, 30, 38, B0, B8 and C8).

***************************************************************************/

#include "emu.h"
#include "k215.h"


ROM_START( dmv_k215 )
	ROM_REGION( 0x0400, "mcu", 0 )
	ROM_LOAD( "dmv_k215_32905.bin", 0x0000, 0x0400, CRC(26c2d99d) SHA1(2549d0fd9d9e8132c2c4a93615c71a7d667f5344) )
ROM_END


static INPUT_PORTS_START( dmv_k215 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x0f, 0x06, "K215 IFSEL (J1)" )
	PORT_DIPSETTING( 0x00, "0A: 60-67 (11-12, 9-10)" )
	PORT_DIPSETTING( 0x01, "0B: 68-6F (13-14, 9-10)" )
	PORT_DIPSETTING( 0x02, "1A: 70-77 (11-12, 7-8)" )    // NCR factory setting, V1K215 default
	PORT_DIPSETTING( 0x03, "1B: 78-7F (13-14, 7-8)" )
	PORT_DIPSETTING( 0x04, "2A: 30-37 (11-12, 5-6)" )
	PORT_DIPSETTING( 0x05, "2B: 38-3F (13-14, 5-6)" )
	PORT_DIPSETTING( 0x06, "3A: B0-B7 (11-12, 3-4)" )    // default here: free next to a K211
	PORT_DIPSETTING( 0x07, "3B: B8-BF (13-14, 3-4)" )
	PORT_DIPSETTING( 0x08, "4A: C0-C7 (11-12, 1-2)" )
	PORT_DIPSETTING( 0x09, "4B: C8-CF (13-14, 1-2)" )
INPUT_PORTS_END


DEFINE_DEVICE_TYPE(DMV_K215, dmv_k215_device, "dmv_k215", "K215 Buffered RS-232C Interface")


dmv_k215_device::dmv_k215_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DMV_K215, tag, owner, clock)
	, device_dmvslot_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_pci(*this, "pci")
	, m_rs232(*this, "rs232")
	, m_dsw(*this, "DSW")
	, m_ram_addr(0)
	, m_p1(0xff)
	, m_p2(0xff)
{
}

void dmv_k215_device::device_start()
{
	std::fill(m_ram.begin(), m_ram.end(), 0);

	save_item(NAME(m_ram));
	save_item(NAME(m_ram_addr));
	save_item(NAME(m_p1));
	save_item(NAME(m_p2));
}

void dmv_k215_device::device_reset()
{
	// RESET/ also resets the 8741, whose ports come up high
	m_p1 = 0xff;
	m_p2 = 0xff;
	out_irq(CLEAR_LINE);
}

const tiny_rom_entry *dmv_k215_device::device_rom_region() const
{
	return ROM_NAME( dmv_k215 );
}

void dmv_k215_device::device_add_mconfig(machine_config &config)
{
	I8741A(config, m_mcu, XTAL(5'068'800));
	m_mcu->p1_in_cb().set(FUNC(dmv_k215_device::p1_r));
	m_mcu->p1_out_cb().set(FUNC(dmv_k215_device::p1_w));
	m_mcu->p2_out_cb().set(FUNC(dmv_k215_device::p2_w));
	m_mcu->t1_in_cb().set(FUNC(dmv_k215_device::t1_r));

	SCN2651(config, m_pci, XTAL(5'068'800));
	m_pci->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_pci->rts_handler().set("rs232", FUNC(rs232_port_device::write_rts));
	m_pci->dtr_handler().set("rs232", FUNC(rs232_port_device::write_dtr));

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
	if (selected(ifsel, offset))
		data = m_mcu->upi41_master_r(offset & 1);
}

void dmv_k215_device::io_write(int ifsel, offs_t offset, uint8_t data)
{
	if (selected(ifsel, offset))
		m_mcu->upi41_master_w(offset & 1, data);
}

uint8_t dmv_k215_device::pci_offset() const
{
	// P2.1 = A0, P2.2 = A1
	return (m_p2 >> 1) & 0x03;
}

uint8_t dmv_k215_device::p1_r()
{
	uint8_t data = 0xff;

	if (BIT(m_p2, 6))
	{
		if (!BIT(m_p2, 3))
			data &= m_ram[m_ram_addr];
		if (!BIT(m_p2, 0))
			data &= m_pci->read(pci_offset());
	}

	return data;
}

void dmv_k215_device::p1_w(uint8_t data)
{
	m_p1 = data;
}

void dmv_k215_device::p2_w(uint8_t data)
{
	uint8_t const old = m_p2;
	uint8_t const rising = ~old & data;
	m_p2 = data;

	// address latches clock on the rising edge
	if (BIT(rising, 4))
		m_ram_addr = (m_ram_addr & 0x700) | m_p1;
	if (BIT(rising, 5))
		m_ram_addr = (m_ram_addr & 0x0ff) | ((m_p1 & 0x07) << 8);

	// 6116 write ends when CS/ or R/W goes high
	if (!BIT(old, 3) && !BIT(old, 6) && (BIT(data, 3) || BIT(data, 6)))
		m_ram[m_ram_addr] = m_p1;

	// 2651 write on the rising edge of CE/ with R/W low; A0/A1 are
	// stable through the whole cycle
	if (BIT(rising, 0) && !BIT(old, 6))
		m_pci->write((old >> 1) & 0x03, m_p1);

	// receive request, open collector to PERC/
	if (BIT(old ^ data, 7))
		out_irq(BIT(data, 7) ? CLEAR_LINE : ASSERT_LINE);
}

int dmv_k215_device::t1_r()
{
	// RxRDY/ is active low
	return m_pci->rxrdy_r();
}
