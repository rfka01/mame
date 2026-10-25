// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Commodore MCS 820 Color Printer: the Okidata Okimate 20 with the
    IBM PC Plug 'n Print module (parallel or RS-232C), firmware AM 01.0
    (logic board and mechanism: machine/okimate20.cpp)

      data    8000-8007  M60307 (decoded on the module)
      data    0000-0007  M60306

    Parallel interface, as far as derived from the firmware:
      M60306 0 r   bit 2: /INIT from the host (the firmware restarts)
      M60306 0 w   bit 0: 0 clears the strobe latch, 1 lets a latched strobe through to RXD
      M60306 1 w   bit 0: /ACK (1 = low), bit 1: clear holds BUSY; BUSY is set by the
                   strobe and released at the start of the /ACK pulse
      M60306 2 r   latched data
      RXD (P3.0)   pulled low by a latched strobe: the 8031 serial port in
                   mode 2 then receives a frame and its RI interrupt
                   fetches the byte from M60306 2
      M60307 1 r   bit 3: 0 = parallel or serial module, 1 = serial interface
                   (read on the module)

    TODO:
    - RS-232C module (8031 serial port, baud rate from M60306 2)
    - module DIP switches on P1

**********************************************************************/

#include "emu.h"
#include "mcs820.h"

#include "machine/okimate20.h"

//#define VERBOSE 1
#include "logmacro.h"


namespace {

class mcs820_device : public okimate20_base_device, public device_centronics_peripheral_interface
{
public:
	mcs820_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		okimate20_base_device(mconfig, MCS820, tag, owner, clock, 0x8000),
		device_centronics_peripheral_interface(mconfig, *this),
		m_data(0),
		m_latch(0),
		m_strobe(1),
		m_strobed(false),
		m_busy(true),
		m_init(1),
		m_ctrl(0),
		m_cmd(0)
	{
	}

	static constexpr flags_type emulation_flags() { return flags::NOT_WORKING; }

	// Centronics
	virtual void input_strobe(int state) override;
	virtual void input_init(int state) override;
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual bool supports_pin35_5v() override { return true; }

	virtual u8 port1_r() override { return 0xff; }
	virtual u8 port3_r() override { return (m_strobed && BIT(m_cmd, 0)) ? 0xfe : 0xff; }
	virtual u8 module_r(offs_t offset) override;
	virtual void module_w(offs_t offset, u8 data) override;
	virtual u8 module_id() const override { return 0x00; }

private:
	void update_busy();

	u8 m_data;
	u8 m_latch;
	int m_strobe;
	bool m_strobed;         // strobe latched, not yet taken
	bool m_busy;
	int m_init;
	u8 m_ctrl;              // M60306 1
	u8 m_cmd;               // M60306 0
};


ROM_START( mcs820 )
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("mcs820-65-1437.bin", 0x0000, 0x4000, CRC(0371981f) SHA1(31bffce523fc20ce4a000062eef48135b1915ab7))
ROM_END

const tiny_rom_entry *mcs820_device::device_rom_region() const
{
	return ROM_NAME( mcs820 );
}

ioport_constructor mcs820_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( okimate20 );
}


void mcs820_device::device_start()
{
	okimate20_base_device::device_start();

	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_strobed));
	save_item(NAME(m_busy));
	save_item(NAME(m_init));
	save_item(NAME(m_ctrl));
	save_item(NAME(m_cmd));
}

void mcs820_device::device_reset()
{
	okimate20_base_device::device_reset();
	m_strobed = false;
	m_busy = true;
	m_ctrl = 0;
	m_cmd = 0;
	update_busy();
	output_ack(1);
}

void mcs820_device::input_strobe(int state)
{
	// the falling edge latches the data
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_strobed = true;
		m_busy = true;
		LOG("strobe %02x\n", m_latch);
		update_busy();
	}
	m_strobe = state;
}

void mcs820_device::input_init(int state)
{
	m_init = state;
}

void mcs820_device::update_busy()
{
	output_busy(m_busy ? 1 : 0);
}

u8 mcs820_device::module_r(offs_t offset)
{
	switch (offset)
	{
	case 0:
		return m_init ? 0xfb : 0xff;    // bit 2: /INIT
	case 2:
		if (!machine().side_effects_disabled())
			LOG("%s read %02x\n", machine().describe_context(), m_latch);
		return m_latch;
	default:
		return 0xff;
	}
}

void mcs820_device::module_w(offs_t offset, u8 data)
{
	LOG("%s M60306 w %d %02x\n", machine().describe_context(), offset, data);
	switch (offset)
	{
	case 0:
		m_cmd = data;
		if (!BIT(data, 0))
			m_strobed = false;
		break;
	case 1:
		m_ctrl = data;
		// BUSY: held by bit 1 clear, released by the /ACK pulse (bits 1 and 0 set) unless a
		// byte is still waiting in the latch; a strobe sets it again
		if (!BIT(data, 1))
			m_busy = true;
		else if (BIT(data, 0) && !m_strobed)
			m_busy = false;
		output_ack(BIT(data, 0) ? 0 : 1);
		update_busy();
		break;
	}
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(MCS820, device_centronics_peripheral_interface, mcs820_device, "mcs820", "Commodore MCS 820 Color Printer")
