// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Heathkit SK-203 Printer Buffer (kit SK-203-H, ca. 1986)

    Parallel and serial input, parallel and serial output, 64K of RAM
    expandable to 512K, "memory available" display in K, per-input
    CLEAR / COPY / P.PRINT / RESTART / OFFLINE keys and a SWAP key
    that exchanges the outputs of the two inputs.

    Hardware (reconstructed from ROM 444-482 at U113):
      HD64180 @ 12.288 MHz crystal (phi 6.144 MHz)
        PRT0 10 Hz and PRT1 40 Hz ticks, ASCI0 = serial input,
        ASCI1 = serial output, DMA0 memory -> TDR1 (requested by the
        ASCI1 transmitter, DAR0B = 2), DMA1 memory -> port 0x84
        (parallel output, edge-triggered /DREQ1)
      MMU: CBAR = F2 (common 0 at 0000-1FFF, bank 2000-EFFF via BBR)
      Physical: the DIP-64 HD64180RP has 19 address lines, hence the
        512K maximum; 4K ROM at 00000-00FFF (only the lower half of
        the 8K EPROM is used), RAM from 00000 up, smaller fits mirror.
      The firmware manages 2K blocks 03-FF and tests each one at
      power-on; with 512K fitted it shows 253 blocks = "506".

    External I/O:
      80-82 R  DIP switches: 80 = serial input (bits 0-3 baud, 38400 ..
               150, bits 5-7 format), 81 = serial output, 82 = options
               (read inverted, polled continuously)
               SW1/SW2 bits 5-7 = data bits/parity/stop bits,
               bit 4 = odd parity
      80-82 W  display digits 1-3 (7-segment, bit 0 = a .. bit 6 = g);
               digit 1 bit 7 = SWAP LED
      83 R     parallel input data (clears the strobe latch, ACKs host)
      84 W     parallel output data (strobes the printer)
      85 R     b0 parallel input byte ready, b1 printer busy,
               b4-b7 key matrix rows
      86 W     key matrix column select (b0-b3, one-hot)

    Key matrix (key code 4 * (3 - column) + (8 - row), dispatched by
    the state machine at 0x0659; channel blocks 0x1000 = serial,
    0x1017 = parallel):
      column 3 rows 7-4: parallel CLEAR, COPY, P.PRINT, RESTART
      column 2 rows 7-4: serial CLEAR, COPY, P.PRINT, RESTART
      column 1 row 7/6: parallel / serial OFFLINE, column 0 row 7: SWAP
    The display marks the channel by blinking the upper (parallel) or
    lower (serial) segments; messages FUL, OFL, nPP, CPP, FE/OE/PE.
    Routing: SW3:5 / SW3:1 select the printer for the parallel / serial
    input, SWAP exchanges both; COPY sets a count (up to 99) that
    RESTART applies to the buffered job.

    Assumptions (no schematic): the remaining bits of port 85, the
    mirroring of partly populated RAM, and that host BUSY is held from
    reset until the firmware's dummy read of port 83 at 0x00A1.
    The power-on write of 00 to port 84 strobes one NUL to the printer.

***************************************************************************/

#include "emu.h"
#include "sk203.h"

#include "bus/rs232/rs232.h"
#include "cpu/z180/z180.h"
#include "machine/output_latch.h"

#include "sk203.lh"


namespace {

class heathkit_sk203_device : public device_t, public device_centronics_peripheral_interface
{
public:
	heathkit_sk203_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

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

	u8 dip_r(offs_t offset);
	void display_w(offs_t offset, u8 data);
	u8 pin_data_r();
	void pout_data_w(u8 data);
	u8 status_r();
	void keycol_w(u8 data);

	void prn_busy_w(int state);
	TIMER_CALLBACK_MEMBER(ack_end);
	TIMER_CALLBACK_MEMBER(strobe_end);
	void update_dreq1();

	required_device<hd64180rp_device> m_maincpu;
	required_device<centronics_device> m_prn;
	required_device<output_latch_device> m_prn_data;
	required_region_ptr<u8> m_rom;
	required_ioport_array<3> m_dips;
	required_ioport_array<4> m_keys;
	required_ioport m_config;
	output_finder<3> m_digits;
	output_finder<> m_swap_led;

	emu_timer *m_ack_timer = nullptr;
	emu_timer *m_strobe_timer = nullptr;
	std::unique_ptr<u8[]> m_ram;

	u8 m_host_data = 0;
	u8 m_host_latch = 0;
	bool m_host_ready = false;
	bool m_host_strobe = true;
	int m_prn_busy = 1;
	bool m_prn_pending = false;
	u8 m_keycol = 0;
};


heathkit_sk203_device::heathkit_sk203_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, HEATHKIT_SK203, tag, owner, clock)
	, device_centronics_peripheral_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_prn(*this, "prn")
	, m_prn_data(*this, "prn_data")
	, m_rom(*this, "maincpu")
	, m_dips(*this, "SW%u", 1U)
	, m_keys(*this, "KEYCOL%u", 0U)
	, m_config(*this, "CONFIG")
	, m_digits(*this, "sk203_digit%u", 0U)
	, m_swap_led(*this, "sk203_swap_led")
{
}


//--------------------------------------------------
//  address maps
//--------------------------------------------------

void heathkit_sk203_device::mem_map(address_map &map)
{
	// RAM and ROM are installed in device_reset according to the RAM size
}

void heathkit_sk203_device::io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x80, 0x82).rw(FUNC(heathkit_sk203_device::dip_r), FUNC(heathkit_sk203_device::display_w));
	map(0x83, 0x83).r(FUNC(heathkit_sk203_device::pin_data_r));
	map(0x84, 0x84).w(FUNC(heathkit_sk203_device::pout_data_w));
	map(0x85, 0x85).r(FUNC(heathkit_sk203_device::status_r));
	map(0x86, 0x86).w(FUNC(heathkit_sk203_device::keycol_w));
}


//--------------------------------------------------
//  front panel
//--------------------------------------------------

u8 heathkit_sk203_device::dip_r(offs_t offset)
{
	return m_dips[offset]->read();
}

void heathkit_sk203_device::display_w(offs_t offset, u8 data)
{
	m_digits[offset] = data & 0x7f;
	if (offset == 0)
		m_swap_led = BIT(data, 7);
}

void heathkit_sk203_device::keycol_w(u8 data)
{
	m_keycol = data & 0x0f;
}

u8 heathkit_sk203_device::status_r()
{
	u8 data = 0;
	if (m_host_ready)
		data |= 0x01;
	if (m_prn_busy || m_prn_pending)
		data |= 0x02;
	for (int i = 0; i < 4; i++)
		if (BIT(m_keycol, i))
			data |= m_keys[i]->read() & 0xf0;
	return data;
}


//--------------------------------------------------
//  parallel input (from the computer)
//--------------------------------------------------

void heathkit_sk203_device::input_strobe(int state)
{
	if (m_host_strobe && !state)
	{
		m_host_latch = m_host_data;
		m_host_ready = true;
		output_busy(1);
	}
	m_host_strobe = state;
}

u8 heathkit_sk203_device::pin_data_r()
{
	if (!machine().side_effects_disabled())
	{
		m_host_ready = false;
		output_ack(0);
		m_ack_timer->adjust(attotime::from_usec(1));
	}
	return m_host_latch;
}

TIMER_CALLBACK_MEMBER(heathkit_sk203_device::ack_end)
{
	output_ack(1);
	output_busy(m_host_ready ? 1 : 0);
}


//--------------------------------------------------
//  parallel output (to the printer)
//--------------------------------------------------

void heathkit_sk203_device::pout_data_w(u8 data)
{
	m_prn_data->write(data);
	m_prn->write_strobe(0);
	m_strobe_timer->adjust(attotime::from_usec(1));
	m_prn_pending = true;      // until the printer has taken the byte
	update_dreq1();
}

TIMER_CALLBACK_MEMBER(heathkit_sk203_device::strobe_end)
{
	m_prn->write_strobe(1);
	m_prn_pending = false;
	update_dreq1();
}

void heathkit_sk203_device::prn_busy_w(int state)
{
	m_prn_busy = state;
	update_dreq1();
}

void heathkit_sk203_device::update_dreq1()
{
	// /DREQ1 is edge-triggered (DCNTL DMS1 = 1): one DMA transfer per
	// "printer ready" edge
	m_maincpu->set_input_line(Z180_INPUT_LINE_DREQ1, (m_prn_busy || m_prn_pending) ? CLEAR_LINE : ASSERT_LINE);
}


//--------------------------------------------------
//  device_t implementation
//--------------------------------------------------

void heathkit_sk203_device::device_start()
{
	m_ack_timer = timer_alloc(FUNC(heathkit_sk203_device::ack_end), this);
	m_strobe_timer = timer_alloc(FUNC(heathkit_sk203_device::strobe_end), this);
	m_ram = std::make_unique<u8[]>(0x80000);

	save_pointer(NAME(m_ram), 0x80000);
	save_item(NAME(m_host_data));
	save_item(NAME(m_host_latch));
	save_item(NAME(m_host_ready));
	save_item(NAME(m_host_strobe));
	save_item(NAME(m_prn_busy));
	save_item(NAME(m_prn_pending));
	save_item(NAME(m_keycol));
}

void heathkit_sk203_device::device_reset()
{
	// RAM from physical 0 up, mirrored through the 512K space of the
	// HD64180RP (unpopulated rows not decoded); the ROM overlays 00000-00FFF
	address_space &mem = m_maincpu->space(AS_PROGRAM);
	const offs_t size = offs_t(m_config->read() & 0x0f) << 16;
	mem.unmap_readwrite(0x00000, 0x7ffff);
	mem.install_ram(0x00000, size - 1, 0x7ffff & ~(size - 1), m_ram.get());
	mem.install_rom(0x00000, 0x00fff, m_rom.target());

	// host BUSY is held until the firmware's dummy read of port 83
	m_host_ready = true;
	output_busy(1);
	output_ack(1);
	output_select(1);
	output_perror(0);
	output_fault(1);

	m_prn->write_strobe(1);
	m_prn_pending = false;
	update_dreq1();
}


//--------------------------------------------------
//  configuration
//--------------------------------------------------

static INPUT_PORTS_START(sk203)
	// key matrix: column select port 86 b0-b3, rows port 85 b4-b7;
	// key code = 4 * (3 - column) + (8 - row), see the table at 0x0659
	PORT_START("KEYCOL0")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Swap")
	PORT_BIT(0x70, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEYCOL1")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Parallel Offline")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Serial Offline")
	PORT_BIT(0x30, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEYCOL2")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Serial Clear")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Serial Copy")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Serial P.Print")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Serial Restart")
	PORT_START("KEYCOL3")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Parallel Clear")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Parallel Copy")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Parallel P.Print")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SK-203 Parallel Restart")

	PORT_START("SW1")  // serial input
	PORT_DIPNAME(0x0f, 0x02, "SK-203 serial input baud rate") PORT_DIPLOCATION("SW1:1,2,3,4")
	PORT_DIPSETTING(0x00, "38400")
	PORT_DIPSETTING(0x01, "19200")
	PORT_DIPSETTING(0x02, "9600")
	PORT_DIPSETTING(0x03, "4800")
	PORT_DIPSETTING(0x04, "2400")
	PORT_DIPSETTING(0x05, "1200")
	PORT_DIPSETTING(0x06, "600")
	PORT_DIPSETTING(0x07, "300")
	PORT_DIPSETTING(0x08, "150")
	PORT_DIPNAME(0x10, 0x00, "SK-203 serial input parity") PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(0x00, "Even")
	PORT_DIPSETTING(0x10, "Odd")
	PORT_DIPNAME(0xe0, 0xc0, "SK-203 serial input format") PORT_DIPLOCATION("SW1:6,7,8")
	PORT_DIPSETTING(0x00, "7 bits, parity, 1 stop")
	PORT_DIPSETTING(0x20, "7 bits, parity, 2 stop")
	PORT_DIPSETTING(0x40, "7 bits, no parity, 1 stop")
	PORT_DIPSETTING(0x60, "7 bits, no parity, 2 stop")
	PORT_DIPSETTING(0x80, "8 bits, parity, 1 stop")
	PORT_DIPSETTING(0xa0, "8 bits, parity, 2 stop")
	PORT_DIPSETTING(0xc0, "8 bits, no parity, 1 stop")
	PORT_DIPSETTING(0xe0, "8 bits, no parity, 2 stop")

	PORT_START("SW2")  // serial output
	PORT_DIPNAME(0x0f, 0x02, "SK-203 serial output baud rate") PORT_DIPLOCATION("SW2:1,2,3,4")
	PORT_DIPSETTING(0x00, "38400")
	PORT_DIPSETTING(0x01, "19200")
	PORT_DIPSETTING(0x02, "9600")
	PORT_DIPSETTING(0x03, "4800")
	PORT_DIPSETTING(0x04, "2400")
	PORT_DIPSETTING(0x05, "1200")
	PORT_DIPSETTING(0x06, "600")
	PORT_DIPSETTING(0x07, "300")
	PORT_DIPSETTING(0x08, "150")
	PORT_DIPNAME(0x10, 0x00, "SK-203 serial output parity") PORT_DIPLOCATION("SW2:5")
	PORT_DIPSETTING(0x00, "Even")
	PORT_DIPSETTING(0x10, "Odd")
	PORT_DIPNAME(0xe0, 0xc0, "SK-203 serial output format") PORT_DIPLOCATION("SW2:6,7,8")
	PORT_DIPSETTING(0x00, "7 bits, parity, 1 stop")
	PORT_DIPSETTING(0x20, "7 bits, parity, 2 stop")
	PORT_DIPSETTING(0x40, "7 bits, no parity, 1 stop")
	PORT_DIPSETTING(0x60, "7 bits, no parity, 2 stop")
	PORT_DIPSETTING(0x80, "8 bits, parity, 1 stop")
	PORT_DIPSETTING(0xa0, "8 bits, parity, 2 stop")
	PORT_DIPSETTING(0xc0, "8 bits, no parity, 1 stop")
	PORT_DIPSETTING(0xe0, "8 bits, no parity, 2 stop")

	PORT_START("SW3")  // options (0 = on)
	PORT_DIPNAME(0x01, 0x01, "SK-203 serial input goes to") PORT_DIPLOCATION("SW3:1")
	PORT_DIPSETTING(0x01, "Serial printer")
	PORT_DIPSETTING(0x00, "Parallel printer")
	PORT_DIPNAME(0x02, 0x02, "SK-203 serial: form feed at end of job") PORT_DIPLOCATION("SW3:2")
	PORT_DIPSETTING(0x02, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x04, "SK-203 SW3:3 (serial channel option)") PORT_DIPLOCATION("SW3:3")
	PORT_DIPSETTING(0x04, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x08, "SK-203 serial output CTS handshake") PORT_DIPLOCATION("SW3:4")
	PORT_DIPSETTING(0x08, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "SK-203 parallel input goes to") PORT_DIPLOCATION("SW3:5")
	PORT_DIPSETTING(0x10, "Serial printer")
	PORT_DIPSETTING(0x00, "Parallel printer")
	PORT_DIPNAME(0x20, 0x20, "SK-203 parallel: form feed at end of job") PORT_DIPLOCATION("SW3:6")
	PORT_DIPSETTING(0x20, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x40, "SK-203 SW3:7 (parallel channel option)") PORT_DIPLOCATION("SW3:7")
	PORT_DIPSETTING(0x40, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))
	PORT_DIPNAME(0x80, 0x80, "SK-203 SW3:8 (serial input RTS)") PORT_DIPLOCATION("SW3:8")
	PORT_DIPSETTING(0x80, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))

	PORT_START("CONFIG")
	PORT_CONFNAME(0x0f, 0x08, "SK-203 RAM size")
	PORT_CONFSETTING(0x01, "64K")
	PORT_CONFSETTING(0x02, "128K")
	PORT_CONFSETTING(0x04, "256K")
	PORT_CONFSETTING(0x08, "512K")
INPUT_PORTS_END

ioport_constructor heathkit_sk203_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(sk203);
}

ROM_START(sk203)
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("444-482.u113", 0x0000, 0x2000, CRC(c09f7251) SHA1(d6b75f219095cadafbdee1da071ec58b1c4195fc))
ROM_END

const tiny_rom_entry *heathkit_sk203_device::device_rom_region() const
{
	return ROM_NAME(sk203);
}

void heathkit_sk203_device::device_add_mconfig(machine_config &config)
{
	HD64180RP(config, m_maincpu, 12.288_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &heathkit_sk203_device::mem_map);
	m_maincpu->set_addrmap(AS_IO, &heathkit_sk203_device::io_map);

	CENTRONICS(config, m_prn, centronics_devices, "printer");
	m_prn->busy_handler().set(FUNC(heathkit_sk203_device::prn_busy_w));
	OUTPUT_LATCH(config, m_prn_data);
	m_prn->set_output_latch(*m_prn_data);

	rs232_port_device &serin(RS232_PORT(config, "serial_in", default_rs232_devices, nullptr));
	serin.rxd_handler().set(m_maincpu, FUNC(z180_device::rxa0_w));
	m_maincpu->txa0_wr_callback().set(serin, FUNC(rs232_port_device::write_txd));

	rs232_port_device &serout(RS232_PORT(config, "serial_out", default_rs232_devices, nullptr));
	serout.rxd_handler().set(m_maincpu, FUNC(z180_device::rxa1_w));
	m_maincpu->txa1_wr_callback().set(serout, FUNC(rs232_port_device::write_txd));

	config.set_default_layout(layout_sk203);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(HEATHKIT_SK203, device_centronics_peripheral_interface, heathkit_sk203_device, "sk203", "Heathkit SK-203 Printer Buffer")
