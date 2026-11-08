// license:BSD-3-Clause
// copyright-holders:rfka01
// LOCAL TEST HARNESS - NOT FOR COMMIT
// Sends a file from the bitbanger to a printer on the Centronics port
// (compatibility mode, then IEEE 1284 nibble mode reverse reads) or on the
// RS-232 port (9600 8N1).

#include "emu.h"
#include "bus/centronics/ctronics.h"
#include "bus/rs232/rs232.h"
#include "imagedev/bitbngr.h"

#include <cstdio>

namespace {

class ljhost_state : public driver_device
{
public:
	ljhost_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_cent(*this, "centronics")
		, m_serial(*this, "serial")
		, m_host(*this, "host")
	{ }

	void ljhost(machine_config &config);

private:
	required_device<centronics_device> m_cent;
	required_device<rs232_port_device> m_serial;
	required_device<bitbanger_device> m_host;

	virtual void machine_start() override;
	TIMER_CALLBACK_MEMBER(step);
	emu_timer *m_timer;
	TIMER_CALLBACK_MEMBER(stx);
	TIMER_CALLBACK_MEMBER(srx);
	emu_timer *m_stx_timer;
	emu_timer *m_srx_timer;
	void rxd_w(int s);

	int m_busy = 1, m_ack = 1, m_perror = 0, m_select = 0, m_fault = 1;
	int m_state = 0;
	int m_wait = 0;
	int m_idle = 0;
	bool m_sent = false;
	u8 m_byte = 0;
	int m_nib = 0;
	u8 m_rx = 0;
	int m_rxd = 1;
	// serial
	int m_bit = -1;
	u16 m_shift = 0;
	int m_srx_bit = -1;
	u16 m_srx = 0;
	int m_srx_cnt = 0;
	bool m_xoff = false;
	bool use_serial() { return getenv("LJSERIAL") != nullptr; }
	int ext() { const char *e = getenv("LJEXT"); return e ? strtoul(e, nullptr, 16) : -1; }
	void data(u8 d) { m_cent->write_data0(BIT(d,0)); m_cent->write_data1(BIT(d,1)); m_cent->write_data2(BIT(d,2)); m_cent->write_data3(BIT(d,3)); m_cent->write_data4(BIT(d,4)); m_cent->write_data5(BIT(d,5)); m_cent->write_data6(BIT(d,6)); m_cent->write_data7(BIT(d,7)); }
};

void ljhost_state::machine_start()
{
	m_timer = timer_alloc(FUNC(ljhost_state::step), this);
	m_timer->adjust(attotime::from_usec(10), 0, attotime::from_usec(10));
	m_stx_timer = timer_alloc(FUNC(ljhost_state::stx), this);
	m_srx_timer = timer_alloc(FUNC(ljhost_state::srx), this);
	if (use_serial())
		m_stx_timer->adjust(attotime::from_hz(9600), 0, attotime::from_hz(9600));
	m_cent->write_strobe(1);
	m_cent->write_autofd(1);
	m_cent->write_select_in(0);
	m_cent->write_init(1);
	m_serial->write_txd(1);
	m_serial->write_rts(0);
	m_serial->write_dtr(0);
}

TIMER_CALLBACK_MEMBER(ljhost_state::step)
{
	if (use_serial())
		return;

	switch (m_state)
	{
	case 0: // compatibility: next byte
	{
		if (m_busy)
			return;
		if (getenv("LJDELAY") && machine().time().as_double() < atof(getenv("LJDELAY")))
			return;
		u8 b;
		if (m_host->input(&b, 1) == 1)
		{
			data(b);
			m_sent = true;
			m_idle = 0;
			m_state = 1;
		}
		else if (m_sent && ext() >= 0 && ++m_idle > 20000)
		{
			// 200 ms idle: negotiate
			printf("HOST negotiate %02x\n", ext());
			data(ext());
			m_cent->write_select_in(1);
			m_cent->write_autofd(0);
			m_state = 10;
			m_sent = false;
		}
		break;
	}
	case 1: m_cent->write_strobe(0); m_state = 2; break;
	case 2: m_cent->write_strobe(1); m_state = 3; break;
	case 3: m_state = 0; break; // BUSY rose with the strobe; wait for it to drop
	case 10: // event 2
		if (!m_ack && m_perror && m_select && m_fault) { m_cent->write_strobe(0); m_state = 11; }
		break;
	case 11: m_cent->write_strobe(1); m_cent->write_autofd(1); m_state = 12; break;
	case 12: // event 6
		if (m_ack) { printf("HOST negotiated: select(xflag)=%d fault=%d\n", m_select, m_fault); m_state = 20; m_nib = 0; m_rx = 0; }
		break;
	case 20: // reverse idle
		if (!m_fault)
		{
			m_cent->write_autofd(0);
			m_state = 21;
		}
		else if (++m_idle > 50000)
		{
			printf("HOST terminate\n");
			m_cent->write_select_in(0);
			m_cent->write_autofd(1);
			m_state = 0;
			m_idle = 0;
		}
		break;
	case 21: // wait ack low
		if (!m_ack)
		{
			u8 n = m_fault | (m_select << 1) | (m_perror << 2) | (m_busy << 3);
			m_rx |= n << (m_nib * 4);
			m_cent->write_autofd(1);
			m_state = 22;
		}
		break;
	case 22:
		if (m_ack)
		{
			if (m_nib)
			{
				printf("REV %02x %c\n", m_rx, (m_rx >= 0x20 && m_rx < 0x7f) ? m_rx : '.');
				m_nib = 0; m_rx = 0;
			}
			else
				m_nib = 1;
			m_idle = 0;
			m_state = (m_nib) ? 23 : 20;
		}
		break;
	case 23: // second nibble
		m_cent->write_autofd(0);
		m_state = 21;
		break;
	}
}

void ljhost_state::rxd_w(int s)
{
	if (m_rxd && !s && m_srx_bit < 0)
	{
		// start bit: sample in the middle of each bit
		m_srx_bit = 0;
		m_srx = 0;
		m_srx_timer->adjust(attotime::from_hz(9600) * 3 / 2);
	}
	m_rxd = s;
}

TIMER_CALLBACK_MEMBER(ljhost_state::srx)
{
	if (m_srx_bit < 8)
	{
		m_srx |= m_rxd << m_srx_bit;
		m_srx_bit++;
		m_srx_timer->adjust(attotime::from_hz(9600));
	}
	else
	{
		printf("REV %02x %c\n", m_srx & 0xff, (m_srx >= 0x20 && m_srx < 0x7f) ? m_srx : '.');
		if ((m_srx & 0xff) == 0x13) m_xoff = true;
		if ((m_srx & 0xff) == 0x11) m_xoff = false;
		m_srx_bit = -1;
	}
}

TIMER_CALLBACK_MEMBER(ljhost_state::stx)
{
	if (m_bit < 0)
	{
		if (m_xoff || machine().time().as_double() < 35.0)
			return;
		u8 b;
		if (m_host->input(&b, 1) != 1)
			return;
		m_shift = (b << 1) | 0x600;
		m_bit = 0;
	}
	m_serial->write_txd(BIT(m_shift, m_bit));
	if (++m_bit == 11)
		m_bit = -1;
}

static INPUT_PORTS_START( ljhost )
INPUT_PORTS_END

void ljhost_state::ljhost(machine_config &config)
{
	CENTRONICS(config, m_cent, centronics_devices, nullptr);
	m_cent->busy_handler().set([this] (int s) { if (getenv("LJDBG") && s != m_busy) printf("BUSY %d at %.4f\n", s, machine().time().as_double()); m_busy = s; });
	m_cent->ack_handler().set([this] (int s) { m_ack = s; });
	m_cent->perror_handler().set([this] (int s) { m_perror = s; });
	m_cent->select_handler().set([this] (int s) { m_select = s; });
	m_cent->fault_handler().set([this] (int s) { m_fault = s; });
	RS232_PORT(config, m_serial, default_rs232_devices, nullptr);
	m_serial->rxd_handler().set(FUNC(ljhost_state::rxd_w));
	BITBANGER(config, m_host, 0);
}

ROM_START( ljhost )
ROM_END

} // anonymous namespace

COMP( 2026, ljhost, 0, 0, ljhost, ljhost, ljhost_state, empty_init, "test", "LaserJet test host", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
