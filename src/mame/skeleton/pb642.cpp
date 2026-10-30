// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HOZN Auto & Soft PB-64-2 Printer Buffer (Taipei, Taiwan, 1988)

    64 KB Centronics printer buffer with two computer and two printer
    connectors, PORT SELECT, and a COPY function (mark a job with
    START/END, add copies with PLUS).

    Hardware (reconstructed from firmware V1.2, U20 2716):
      Z80 @ 4 MHz, no interrupts (NMI vector only toggles R.7, unused)
      0000-07FF  2 KB EPROM
      0800-FFFF  RAM (64 KB fitted, low 2 KB shadowed by the ROM)
      Buffer ring 0800-FFD8 (63449 bytes), variables FFD9-FFFF

    I/O (A0-A2 decoded, base 0x40):
      40 R   host data latch (byte strobed in by the computer)
      41 W   clear "host data ready" flip-flop
      42 R   status   b0 host data ready       b1 printer BUSY
                      b2 SELECT key (0=down)   b3 START/END key (0=down)
                      b4 PLUS key (0=down)     b5 printer PE / not ready
                      b6 n/c                   b7 jumper: 1 = no auto CR/FF
      43 W   printer data latch
      44 W   printer STROBE pulse
      45 W   host ACK start          46 W  host ACK end / release BUSY
      47 W   LED/control latch (LEDs active low)
              b0 PRINTER ERROR  b1 16K  b2 port select (1 = port 1)
              b3 32K  b4 COPY  b5 48K  b6 ready/enable  b7 64K

    Uncertain (no schematic): exact meaning of 41/45/46, of latch b6,
    and how latch b2 routes the two computer/printer connectors.
    Here: port 1 = buffered path, port 2 = firmware pass-through.

    Usage:
      mame pb642 -bitb hostdata.prn -printout printed.prn
    The bitbanger feeds bytes as if strobed in by the computer (a file
    or a socket, e.g. -bitb socket.127.0.0.1:2345).

    Power-on key combos: START/END = endless banner self-test print,
    SELECT = jumper/LED test, PLUS = RAM burn-in loop.

***************************************************************************/

#include "emu.h"

#include "bus/centronics/ctronics.h"
#include "cpu/z80/z80.h"
#include "imagedev/bitbngr.h"

#include "pb642.lh"

#define LOG_HOST (1U << 1)
#define VERBOSE 0
#include "logmacro.h"

namespace {

class pb642_state : public driver_device
{
public:
	pb642_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_centronics(*this, "printer")
		, m_cent_data(*this, "cent_data")
		, m_host(*this, "host")
		, m_keys(*this, "KEYS")
		, m_config(*this, "CONFIG")
		, m_leds(*this, "led%u", 0U)
		, m_port_led(*this, "port%u", 1U)
		, m_ready_led(*this, "ready")
	{ }

	void pb642(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(reset_button);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 host_data_r();
	void host_clear_w(u8 data);
	u8 status_r();
	void strobe_w(u8 data);
	void ack_w(offs_t offset, u8 data);
	void leds_w(u8 data);

	TIMER_CALLBACK_MEMBER(host_poll);

	void busy_w(int state) { m_busy = state; }
	void perror_w(int state) { m_perror = state; }

	required_device<z80_device> m_maincpu;
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_cent_data;
	required_device<bitbanger_device> m_host;
	required_ioport m_keys;
	required_ioport m_config;
	output_finder<8> m_leds;
	output_finder<2> m_port_led;
	output_finder<> m_ready_led;

	emu_timer *m_host_timer = nullptr;
	u8 m_host_latch = 0;
	bool m_host_ready = false;
	int m_busy = 0;
	int m_perror = 0;
};


void pb642_state::mem_map(address_map &map)
{
	map(0x0000, 0xffff).ram();
	map(0x0000, 0x07ff).rom().region("maincpu", 0);
}

void pb642_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x40, 0x40).r(FUNC(pb642_state::host_data_r));
	map(0x41, 0x41).w(FUNC(pb642_state::host_clear_w));
	map(0x42, 0x42).r(FUNC(pb642_state::status_r));
	map(0x43, 0x43).w(m_cent_data, FUNC(output_latch_device::write));
	map(0x44, 0x44).w(FUNC(pb642_state::strobe_w));
	map(0x45, 0x46).w(FUNC(pb642_state::ack_w));
	map(0x47, 0x47).w(FUNC(pb642_state::leds_w));
}


u8 pb642_state::host_data_r()
{
	return m_host_latch;
}

void pb642_state::host_clear_w(u8 data)
{
	m_host_ready = false;
}

u8 pb642_state::status_r()
{
	u8 data = 0x40;                              // b6 not connected
	if (m_host_ready)
		data |= 0x01;
	if (m_busy)
		data |= 0x02;
	data |= m_keys->read() & 0x1c;               // active-low keys
	if (m_perror)
		data |= 0x20;
	data |= m_config->read() & 0x80;             // auto CR/FF jumper
	return data;
}

void pb642_state::strobe_w(u8 data)
{
	m_centronics->write_strobe(0);
	m_centronics->write_strobe(1);
}

void pb642_state::ack_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_HOST, "host ACK %s\n", offset ? "end" : "start");
}

void pb642_state::leds_w(u8 data)
{
	// LEDs are active low
	for (int i = 0; i < 8; i++)
		m_leds[i] = BIT(~data, i);
	m_port_led[0] = BIT(data, 2);
	m_port_led[1] = BIT(~data, 2);
	m_ready_led = BIT(data, 6);
}

TIMER_CALLBACK_MEMBER(pb642_state::host_poll)
{
	// The computer strobes the next byte as soon as the latch is free
	if (!m_host_ready)
	{
		u8 b;
		if (m_host->input(&b, 1) == 1)
		{
			m_host_latch = b;
			m_host_ready = true;
			LOGMASKED(LOG_HOST, "host byte %02x\n", b);
		}
	}
}

INPUT_CHANGED_MEMBER(pb642_state::reset_button)
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, newval ? CLEAR_LINE : ASSERT_LINE);
}


void pb642_state::machine_start()
{
	m_host_timer = timer_alloc(FUNC(pb642_state::host_poll), this);
	m_host_timer->adjust(attotime::from_usec(50), 0, attotime::from_usec(50));

	save_item(NAME(m_host_latch));
	save_item(NAME(m_host_ready));
	save_item(NAME(m_busy));
	save_item(NAME(m_perror));
}

void pb642_state::machine_reset()
{
	m_host_ready = false;
	m_centronics->write_strobe(1);   // STROBE idles high
}


static INPUT_PORTS_START(pb642)
	PORT_START("KEYS")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Port Select") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Copy Start/End") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Copy Plus") PORT_CODE(KEYCODE_P)
	PORT_BIT(0xe3, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("RESET")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Reset") PORT_CODE(KEYCODE_R) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pb642_state::reset_button), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME(0x80, 0x00, "Form feed between copies")
	PORT_CONFSETTING(0x00, "Auto CR/FF")
	PORT_CONFSETTING(0x80, "Off")
INPUT_PORTS_END


void pb642_state::pb642(machine_config &config)
{
	Z80(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &pb642_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &pb642_state::io_map);

	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->busy_handler().set(FUNC(pb642_state::busy_w));
	m_centronics->perror_handler().set(FUNC(pb642_state::perror_w));
	OUTPUT_LATCH(config, m_cent_data);
	m_centronics->set_output_latch(*m_cent_data);

	BITBANGER(config, m_host, 0);
	m_host->set_readonly(true);

	config.set_default_layout(layout_pb642);
}


ROM_START(pb642)
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD("pb642_v1.2_2716_u20.bin", 0x0000, 0x0800, CRC(8b2bccbb) SHA1(40007e2680b487977bb5fd6cb95d28b5f25749bf))
ROM_END

} // anonymous namespace


//    YEAR  NAME   PARENT  COMPAT  MACHINE  INPUT  CLASS        INIT        COMPANY                  FULLNAME                         FLAGS
SYST( 1988, pb642, 0,      0,      pb642,   pb642, pb642_state, empty_init, "HOZN Auto & Soft",      "PB-64-2 Printer Buffer (V1.2)", MACHINE_SUPPORTS_SAVE | MACHINE_NO_SOUND_HW )
