// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    HOZN Auto & Soft PB-64-2 Printer Buffer (Taipei, Taiwan, 1988)

    64 KB Centronics printer buffer with PORT SELECT and a COPY function
    (mark a job with START/END, add copies with PLUS). As a Centronics
    peripheral it sits on a computer's printer port; it drives its own
    two printer ports (port 1 defaults to the standard printer).

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
              b3 32K  b4 COPY  b5 48K  b6 buffer ready  b7 64K

    Assumptions (no schematic available):
      - latch b2 routes data/strobe/status to printer port 1 or 2
        (port 2 is the firmware's unbuffered pass-through mode)
      - latch b6 drives the BUFFER READY LED
      - host BUSY is a flip-flop set by RESET and by STROBE and cleared
        by the OUT 46 ACK end; the dummy host read + ACK at the end of
        the self test (ROM 0198) is what first releases the computer
      - the COMPUTER 2 connector is not emulated

    Power-on key combos: START/END = endless banner self-test print,
    SELECT = jumper/LED test, PLUS = RAM burn-in loop.

***************************************************************************/

#include "emu.h"
#include "pb642.h"

#include "cpu/z80/z80.h"
#include "machine/output_latch.h"

#include "pb642.lh"


namespace {

class centronics_pb642_device : public device_t, public device_centronics_peripheral_interface
{
public:
	centronics_pb642_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	DECLARE_INPUT_CHANGED_MEMBER(reset_button);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_centronics_peripheral_interface implementation
	virtual void input_strobe(int state) override;
	virtual void input_data0(int state) override { set_data_bit(0, state); }
	virtual void input_data1(int state) override { set_data_bit(1, state); }
	virtual void input_data2(int state) override { set_data_bit(2, state); }
	virtual void input_data3(int state) override { set_data_bit(3, state); }
	virtual void input_data4(int state) override { set_data_bit(4, state); }
	virtual void input_data5(int state) override { set_data_bit(5, state); }
	virtual void input_data6(int state) override { set_data_bit(6, state); }
	virtual void input_data7(int state) override { set_data_bit(7, state); }

private:
	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	void set_data_bit(int bit, int state) { m_host_data = (m_host_data & ~(1 << bit)) | ((state ? 1 : 0) << bit); }
	void update_host_busy();

	u8 host_data_r();
	void host_clear_w(u8 data);
	u8 status_r();
	void prn_data_w(u8 data);
	void prn_strobe_w(u8 data);
	void ack_w(offs_t offset, u8 data);
	void leds_w(u8 data);

	template <unsigned N> void prn_busy_w(int state) { m_prn_busy[N] = state; }
	template <unsigned N> void prn_perror_w(int state) { m_prn_perror[N] = state; }

	unsigned sel() const { return BIT(m_latch, 2) ? 0 : 1; }

	required_device<z80_device> m_maincpu;
	required_device_array<centronics_device, 2> m_prn;
	required_device_array<output_latch_device, 2> m_prn_data;
	required_ioport m_keys;
	required_ioport m_config;
	output_finder<8> m_leds;
	output_finder<2> m_port_led;
	output_finder<> m_ready_led;

	u8 m_host_data;
	u8 m_host_latch;
	bool m_host_ready;
	bool m_host_strobe;
	bool m_ack_busy;
	u8 m_latch;
	int m_prn_busy[2];
	int m_prn_perror[2];
};


centronics_pb642_device::centronics_pb642_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, CENTRONICS_PB642, tag, owner, clock)
	, device_centronics_peripheral_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_prn(*this, "prn%u", 1U)
	, m_prn_data(*this, "prn%u_data", 1U)
	, m_keys(*this, "KEYS")
	, m_config(*this, "CONFIG")
	, m_leds(*this, "pb642_led%u", 0U)
	, m_port_led(*this, "pb642_port%u", 1U)
	, m_ready_led(*this, "pb642_ready")
	, m_host_data(0)
	, m_host_latch(0)
	, m_host_ready(false)
	, m_host_strobe(true)
	, m_ack_busy(false)
	, m_latch(0)
	, m_prn_busy{ 1, 1 }                       // pulled up: an empty port reads BUSY/PE
	, m_prn_perror{ 1, 1 }
{
}


void centronics_pb642_device::mem_map(address_map &map)
{
	map(0x0000, 0xffff).ram();
	map(0x0000, 0x07ff).rom().region("maincpu", 0);
}

void centronics_pb642_device::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x40, 0x40).r(FUNC(centronics_pb642_device::host_data_r));
	map(0x41, 0x41).w(FUNC(centronics_pb642_device::host_clear_w));
	map(0x42, 0x42).r(FUNC(centronics_pb642_device::status_r));
	map(0x43, 0x43).w(FUNC(centronics_pb642_device::prn_data_w));
	map(0x44, 0x44).w(FUNC(centronics_pb642_device::prn_strobe_w));
	map(0x45, 0x46).w(FUNC(centronics_pb642_device::ack_w));
	map(0x47, 0x47).w(FUNC(centronics_pb642_device::leds_w));
}


// host side

void centronics_pb642_device::input_strobe(int state)
{
	// latch on the falling edge of STROBE
	if (m_host_strobe && !state)
	{
		m_host_latch = m_host_data;
		m_host_ready = true;
		m_ack_busy = true;
		update_host_busy();
	}
	m_host_strobe = state;
}

void centronics_pb642_device::update_host_busy()
{
	output_busy(m_ack_busy ? 1 : 0);
}

u8 centronics_pb642_device::host_data_r()
{
	return m_host_latch;
}

void centronics_pb642_device::host_clear_w(u8 data)
{
	m_host_ready = false;
}

void centronics_pb642_device::ack_w(offs_t offset, u8 data)
{
	if (!offset)
	{
		output_ack(0);
	}
	else
	{
		output_ack(1);
		m_ack_busy = false;
		update_host_busy();
	}
}


// printer side

u8 centronics_pb642_device::status_r()
{
	u8 data = 0x40;                              // b6 not connected
	if (m_host_ready)
		data |= 0x01;
	if (m_prn_busy[sel()])
		data |= 0x02;
	data |= m_keys->read() & 0x1c;               // active-low keys
	if (m_prn_perror[sel()])
		data |= 0x20;
	data |= m_config->read() & 0x80;             // auto CR/FF jumper
	return data;
}

void centronics_pb642_device::prn_data_w(u8 data)
{
	m_prn_data[0]->write(data);
	m_prn_data[1]->write(data);
}

void centronics_pb642_device::prn_strobe_w(u8 data)
{
	m_prn[sel()]->write_strobe(0);
	m_prn[sel()]->write_strobe(1);
}

void centronics_pb642_device::leds_w(u8 data)
{
	m_latch = data;
	for (int i = 0; i < 8; i++)
		m_leds[i] = BIT(~data, i);               // LEDs are active low
	m_port_led[0] = BIT(data, 2);
	m_port_led[1] = BIT(~data, 2);
	m_ready_led = BIT(data, 6);
}


INPUT_CHANGED_MEMBER(centronics_pb642_device::reset_button)
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, newval ? CLEAR_LINE : ASSERT_LINE);
}


void centronics_pb642_device::device_start()
{
	save_item(NAME(m_host_data));
	save_item(NAME(m_host_latch));
	save_item(NAME(m_host_ready));
	save_item(NAME(m_host_strobe));
	save_item(NAME(m_ack_busy));
	save_item(NAME(m_latch));
	save_item(NAME(m_prn_busy));
	save_item(NAME(m_prn_perror));
}

void centronics_pb642_device::device_reset()
{
	m_host_ready = false;
	m_ack_busy = true;                           // BUSY until the first ACK after POST
	m_latch = 0;
	update_host_busy();
	output_ack(1);
	output_select(1);
	output_perror(0);
	output_fault(1);
	m_prn[0]->write_strobe(1);                   // STROBE idles high
	m_prn[1]->write_strobe(1);
}


static INPUT_PORTS_START(pb642)
	PORT_START("KEYS")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("PB-64-2 Port Select") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("PB-64-2 Copy Start/End") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("PB-64-2 Copy Plus") PORT_CODE(KEYCODE_P)
	PORT_BIT(0xe3, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("RESET")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("PB-64-2 Reset") PORT_CODE(KEYCODE_R) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(centronics_pb642_device::reset_button), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME(0x80, 0x00, "PB-64-2 form feed between copies")
	PORT_CONFSETTING(0x00, "Auto CR/FF")
	PORT_CONFSETTING(0x80, "Off")
INPUT_PORTS_END

ioport_constructor centronics_pb642_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(pb642);
}


ROM_START(pb642)
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD("pb642_v1.2_2716_u20.bin", 0x0000, 0x0800, CRC(8b2bccbb) SHA1(40007e2680b487977bb5fd6cb95d28b5f25749bf))
ROM_END

const tiny_rom_entry *centronics_pb642_device::device_rom_region() const
{
	return ROM_NAME(pb642);
}


void centronics_pb642_device::device_add_mconfig(machine_config &config)
{
	Z80(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &centronics_pb642_device::mem_map);
	m_maincpu->set_addrmap(AS_IO, &centronics_pb642_device::io_map);

	CENTRONICS(config, m_prn[0], centronics_devices, "printer");
	m_prn[0]->busy_handler().set(FUNC(centronics_pb642_device::prn_busy_w<0>));
	m_prn[0]->perror_handler().set(FUNC(centronics_pb642_device::prn_perror_w<0>));
	OUTPUT_LATCH(config, m_prn_data[0]);
	m_prn[0]->set_output_latch(*m_prn_data[0]);

	CENTRONICS(config, m_prn[1], centronics_devices, nullptr);
	m_prn[1]->busy_handler().set(FUNC(centronics_pb642_device::prn_busy_w<1>));
	m_prn[1]->perror_handler().set(FUNC(centronics_pb642_device::prn_perror_w<1>));
	OUTPUT_LATCH(config, m_prn_data[1]);
	m_prn[1]->set_output_latch(*m_prn_data[1]);

	config.set_default_layout(layout_pb642);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(CENTRONICS_PB642, device_centronics_peripheral_interface, centronics_pb642_device, "pb642", "HOZN Auto & Soft PB-64-2 Printer Buffer")
