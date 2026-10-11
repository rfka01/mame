// license:BSD-3-Clause
// copyright-holders:Dirk Best, rfka01
/***************************************************************************

    E05A03 Gate Array (used in the Epson LX-800)

    Registers (from the LX-800 technical manual):

    read  0  Centronics data latched at the leading edge of /STROBE;
             resets the BUSY caused by /STROBE
    read  1  bit 7 = trailing edge of /STROBE seen, bit 6 = HOME input
    read  2  Centronics data, without touching BUSY
    read  3  shift register MSB (bit 7), then shift left
    write 0-2  24-bit shift register (high, middle, low byte)
    write 3  bit 7 = BUSY from the leading (1) or trailing (0) edge of /STROBE
             bit 6 = 1 resets, 0 sets the software BUSY
             bit 4 = NLQ lamp, bit 3 = condensed lamp (both unused in the LX-800)
             bit 2 = 1: PE high, PE lamp on
    write 4  printhead latches HD1..HD8 from bits 7..0, inverted
    write 5  printhead latch HD9 from bit 7, inverted
    write 6  paper feed motor phases PFA..PFD (bits 7..4)
    write 7  carriage motor phases CRA..CRD (bits 3..0)

    The needles fire while FIRE (from the CPU's CO0 output) is low.

***************************************************************************/

#include "emu.h"
#include "e05a03.h"


/*****************************************************************************
    DEVICE INTERFACE
*****************************************************************************/

DEFINE_DEVICE_TYPE(E05A03, e05a03_device, "e05a03", "Epson E05A03 Gate Array")

e05a03_device::e05a03_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, E05A03, tag, owner, clock),
	m_write_nlq_lp(*this),
	m_write_pe_lp(*this),
	m_write_reso(*this),
	m_write_pe(*this),
	m_write_busy(*this),
	m_write_pf_motor(*this),
	m_write_cr_motor(*this),
	m_read_data(*this, 0),
	m_shift(0),
	m_busy_leading(0),
	m_busy_software(0),
	m_nlqlp(0),
	m_cndlp(0),
	m_data(0),
	m_strobe(1),
	m_strobe_flag(0),
	m_strobe_busy(0),
	m_busy(1),
	m_home(0),
	m_printhead(0),
	m_pf_motor(0),
	m_cr_motor(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void e05a03_device::device_start()
{
	/* register for state saving */
	save_item(NAME(m_shift));
	save_item(NAME(m_busy_leading));
	save_item(NAME(m_busy_software));
	save_item(NAME(m_nlqlp));
	save_item(NAME(m_cndlp));
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_strobe_flag));
	save_item(NAME(m_strobe_busy));
	save_item(NAME(m_busy));
	save_item(NAME(m_home));
	save_item(NAME(m_printhead));
	save_item(NAME(m_pf_motor));
	save_item(NAME(m_cr_motor));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void e05a03_device::device_reset()
{
	// printhead latches cleared, motors in their hold states, BUSY until the firmware is ready
	m_printhead = 0x000;
	m_pf_motor = 0x00;
	m_cr_motor = 0x0f;

	m_write_pe(0);
	m_write_pe_lp(1);

	m_busy_software = 1;
	m_strobe_flag = 0;
	m_strobe_busy = 0;
	m_nlqlp = 1;
	m_cndlp = 1;
	update_busy();
}



/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

void e05a03_device::update_busy()
{
	int const busy = m_busy_software || m_strobe_busy;
	if (busy != m_busy)
	{
		m_busy = busy;
		m_write_busy(busy);
	}
}

void e05a03_device::write(offs_t offset, uint8_t data)
{
	switch (offset)
	{
	/* shift register */
	case 0x00: m_shift = (m_shift & 0x00ffff) | (data << 16); break;
	case 0x01: m_shift = (m_shift & 0xff00ff) | (data << 8); break;
	case 0x02: m_shift = (m_shift & 0xffff00) | (data << 0); break;

	case 0x03:
		m_busy_leading = BIT(data, 7);
		m_busy_software = !BIT(data, 6);
		m_nlqlp = BIT(data, 4);
		m_cndlp = BIT(data, 3);

		m_write_nlq_lp(!m_nlqlp);
		m_write_pe(BIT(data, 2));
		m_write_pe_lp(BIT(data, 2));
		update_busy();
		break;

	/* printhead: needle n is bit n-1, the latches are written inverted */
	case 0x04: m_printhead = (m_printhead & 0x100) | bitswap<8>(~data, 0, 1, 2, 3, 4, 5, 6, 7); break;
	case 0x05: m_printhead = (m_printhead & 0x0ff) | (BIT(data, 7) ? 0 : 0x100); break;

	/* paper feed and carriage motor phase data*/
	case 0x06:
		m_pf_motor = (data & 0xf0) >> 4;
		m_write_pf_motor(m_pf_motor);
		break;
	case 0x07:
		m_cr_motor = (data & 0x0f) >> 0;
		m_write_cr_motor(m_cr_motor);
		break;
	}
}

uint8_t e05a03_device::read(offs_t offset)
{
	uint8_t result = 0;

	switch (offset)
	{
	case 0x00:
		result = m_data;
		if (!machine().side_effects_disabled())
		{
			m_strobe_flag = 0;
			m_strobe_busy = 0;
			update_busy();
		}
		break;

	case 0x01:
		result = (m_strobe_flag << 7) | (m_home << 6);
		break;

	case 0x02:
		result = m_read_data(0);
		break;

	case 0x03:
		result |= BIT(m_shift, 23) << 7;
		if (!machine().side_effects_disabled())
			m_shift <<= 1;
		break;
	}

	return result;
}

/* home position signal */
void e05a03_device::home_w(int state)
{
	m_home = state ? 1 : 0;
}

void e05a03_device::strobe_w(int state)
{
	if (m_strobe && !state)
	{
		// leading edge: latch the data
		m_data = m_read_data(0);
		if (m_busy_leading)
			m_strobe_busy = 1;
	}
	else if (!m_strobe && state)
	{
		// trailing edge
		m_strobe_flag = 1;
		m_strobe_busy = 1;
	}
	m_strobe = state;
	update_busy();
}

void e05a03_device::resi_w(int state)
{
	if (!state)
	{
		device_reset();
		m_write_reso(1);
	}
}

void e05a03_device::init_w(int state)
{
	resi_w(state);
}
