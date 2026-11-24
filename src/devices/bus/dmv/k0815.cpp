// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    K0815 1 MB SRAM expansion (homebrew, VzEkC e.V., F. Stassen 2025)

    Not an NCR product.  A peripheral slot card (J102-J106) with two
    512Kx8 SRAMs (IC3 = BA19 low, IC4 = BA19 high), a 74LS245 data
    transceiver (IC5), two PALCE16V8 (IC1, IC2) and, from PCB v1.1 on, a
    74LS08 (IC6) for a future common area.  Schematic "1MB SRAM (K0815)"
    rev 0.01, GAL equations K0815 rev 2.2 (2025-12-19).

    The card replaces ALL memory of the machine: IC2 holds TRAMD/ low at
    all times, which keeps the mainboard RAM (and a K2xx in J1) off the
    data bus on reads, and it decodes the full bus address BA0-BA19.
    During Z80 and DMA cycles the mainboard puts the bank latch (E0-E7)
    on BA16-BA18, so the Z80 sees the lower 512 KB as its eight banks;
    a 16-bit processor that drives BA19 (K235+) reaches the full 1 MB.

    IC1 rebuilds the boot ROM switch of the mainboard (RAMDIS):
      - set by RESET and by an I/O access to 11h (ROMSEL/), cleared by an
        I/O access to 10h (RAMSEL/); BA3 is not decoded (18h/19h mirror)
      - I/O cycles of either processor count, DMA cycles do not: the
        access is taken while HOLDA (DMA for the Z80) is inactive, or while
        THOLD is active (16-bit processor running) and HOLDA16 (DMA for the
        16-bit processor) is not.  The 16-bit firmware relies on this, the
        K235 ROM 33473 itself writes 11h and 10h.  HOLDA16 comes from pin
        b13 of the 16-bit module, wired to AUTO/ (a13) of the K0815 slot;
        PCB v1.1 routes AUTO/, THOLD/ (a14) and HOLDA (c14) to IC1.
    While RAMDIS is set, the card stays off the bus in x0000-x1FFF of
    every bank (reads and writes), so the mainboard boot ROM shows.

    IC2 also leaves the EPROM of the 16-bit module alone; the jumper J1
    ("8088", pulled to GND) enables this hole.  Two GAL images exist:
      K0815-2.K235+4    FF000-FFFFF  for the K235+ with a 4 KB EPROM window
      K0815-2.K235-512  80000-FFFFF  for K230/K235/K231, whose EPROM is
                                     decoded redundantly in the upper 512 KB
    Inside the hole the card neither reads nor writes, and since TRAMD/ is
    still low, a bus read there would see a floating bus; the EPROM sits
    on the processor module and never comes from the bus.

    Not emulated: the common area output of IC2 (fixed inactive in GAL
    rev 2.2) and the 74LS08 behind it.

***************************************************************************/

#include "emu.h"
#include "k0815.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

static INPUT_PORTS_START( dmv_k0815 )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "J1 (8088 EPROM hole)" )
	PORT_CONFSETTING( 0x00, "Open (no hole, 1 MB RAM)" )
	PORT_CONFSETTING( 0x01, "Fitted" )
	PORT_CONFNAME( 0x02, 0x00, "IC2 GAL" )
	PORT_CONFSETTING( 0x00, "K0815-2.K235+4 (hole FF000-FFFFF)" )
	PORT_CONFSETTING( 0x02, "K0815-2.K235-512 (hole 80000-FFFFF)" )
INPUT_PORTS_END


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DMV_K0815, dmv_k0815_device, "dmv_k0815", "K0815 1MB SRAM (homebrew)")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

dmv_k0815_device::dmv_k0815_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DMV_K0815, tag, owner, clock)
	, device_dmvslot_interface(mconfig, *this)
	, m_config(*this, "CONFIG")
	, m_ramdis(1)
{
}

ioport_constructor dmv_k0815_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k0815 );
}

void dmv_k0815_device::device_start()
{
	// battery-less SRAM: contents are random at power-on, zero is close enough
	m_ram = make_unique_clear<uint8_t[]>(0x100000);

	// IC1 listens to 10h/11h (and the 18h/19h mirrors) on the system bus
	m_io_tap = iospace().install_readwrite_tap(0x10, 0x19, "k0815_ramdis",
			[this] (offs_t offset, uint8_t &data, uint8_t mem_mask) { ramsel_tap(offset); },
			[this] (offs_t offset, uint8_t &data, uint8_t mem_mask) { ramsel_tap(offset); },
			&m_io_tap);

	save_pointer(NAME(m_ram), 0x100000);
	save_item(NAME(m_ramdis));
}

void dmv_k0815_device::device_reset()
{
	// RESET sets the RS flip-flop in IC1: boot ROM visible
	m_ramdis = 1;
}

void dmv_k0815_device::ramsel_tap(offs_t offset)
{
	// DMA cycles (the other half of the IC1 terms) never come through here
	if (machine().side_effects_disabled())
		return;

	// BA7-BA5 = 0, BA4 = 1, BA2 = BA1 = 0, BA3 don't care
	if ((offset & 0xf6) != 0x10)
		return;

	if (offset & 1)
		m_ramdis = 1;   // 11h/19h ROMSEL/
	else
		m_ramdis = 0;   // 10h/18h RAMSEL/
}

// IC2: DATAEN/ for a memory cycle at bus address ba
bool dmv_k0815_device::dataen(offs_t ba) const
{
	// mainboard boot ROM, x0000-x1FFF in every bank while RAMDIS is set
	if (m_ramdis && !(ba & 0xe000))
		return false;

	// EPROM of the 16-bit module, enabled by J1
	ioport_value const config = m_config->read();
	if (config & 0x01)
	{
		if (config & 0x02)
		{
			if (ba & 0x80000)                       // K235-512: BA19
				return false;
		}
		else if ((ba & 0xff000) == 0xff000)         // K235+4: BA19-BA12 all high
			return false;
	}

	return true;
}

bool dmv_k0815_device::mem_read(offs_t offset, offs_t ba, uint8_t &data)
{
	ba &= 0xfffff;
	if (dataen(ba))
		data = m_ram[ba];

	// TRAMD/ is held low on every cycle: the mainboard RAM never drives
	// the bus, even where this card does not answer
	return true;
}

bool dmv_k0815_device::mem_write(offs_t offset, offs_t ba, uint8_t data)
{
	ba &= 0xfffff;

	if (dataen(ba))
		m_ram[ba] = data;

	return true;
}
