// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HOZN Auto & Soft PB-64-2 Printer Buffer - standalone test system

    The buffer itself is a Centronics peripheral (bus/centronics/pb642.cpp)
    and can be attached to the printer port of any machine with a
    Centronics slot, e.g.  mame <system> -<printer port> pb642

    This system is a minimal "computer" for using the buffer on its own:
    it reads bytes from a bitbanger image (file or socket) and sends them
    over its Centronics port with the usual BUSY/STROBE handshake.

      mame pb642 -bitb hostdata.prn -printout printed.prn
      mame pb642 -bitb socket.127.0.0.1:2345 -printout printed.prn

    The front panel layout comes from the buffer device.

***************************************************************************/

#include "emu.h"

#include "bus/centronics/ctronics.h"
#include "imagedev/bitbngr.h"
#include "machine/output_latch.h"


namespace {

class pb642_host_state : public driver_device
{
public:
	pb642_host_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_centronics(*this, "buffer")
		, m_cent_data(*this, "cent_data")
		, m_host(*this, "host")
	{ }

	void pb642(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	TIMER_CALLBACK_MEMBER(host_poll);
	void busy_w(int state) { m_busy = state; }

	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_cent_data;
	required_device<bitbanger_device> m_host;

	emu_timer *m_host_timer = nullptr;
	int m_busy = 1;
};


TIMER_CALLBACK_MEMBER(pb642_host_state::host_poll)
{
	// send the next byte as soon as the peripheral is not BUSY
	if (!m_busy)
	{
		u8 b;
		if (m_host->input(&b, 1) == 1)
		{
			m_cent_data->write(b);
			m_centronics->write_strobe(0);
			m_centronics->write_strobe(1);
		}
	}
}

void pb642_host_state::machine_start()
{
	m_host_timer = timer_alloc(FUNC(pb642_host_state::host_poll), this);
	m_host_timer->adjust(attotime::from_usec(20), 0, attotime::from_usec(20));

	save_item(NAME(m_busy));
}

void pb642_host_state::machine_reset()
{
	m_centronics->write_strobe(1);
}


void pb642_host_state::pb642(machine_config &config)
{
	CENTRONICS(config, m_centronics, centronics_devices, "pb642");
	m_centronics->busy_handler().set(FUNC(pb642_host_state::busy_w));
	OUTPUT_LATCH(config, m_cent_data);
	m_centronics->set_output_latch(*m_cent_data);

	BITBANGER(config, m_host, 0);
	m_host->set_readonly(true);
}


ROM_START(pb642)
ROM_END

} // anonymous namespace


//    YEAR  NAME   PARENT  COMPAT  MACHINE  INPUT  CLASS             INIT        COMPANY             FULLNAME                         FLAGS
SYST( 1988, pb642, 0,      0,      pb642,   0,     pb642_host_state, empty_init, "HOZN Auto & Soft", "PB-64-2 Printer Buffer (V1.2)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
