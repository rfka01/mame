// license:BSD-3-Clause
// copyright-holders:Golden Child
/***************************************************************************

    Electronics Today International Print Buffer 48K

    Designed by Nick Sawyer and published in the July and August 1985
    issues of Electronics Today International (UK Edition)

    https://worldradiohistory.com/UK/Electronics-Today-UK/80s/Electronics-Today-1985-07.pdf
        (pages 33-37 part 1: schematic, parts list, description;
         pages 54-55 pcb foil pattern)
    https://worldradiohistory.com/UK/Electronics-Today-UK/80s/Electronics-Today-1985-08.pdf
        (pages 48-50 part 2: rom listing, memory map)
    Follow-up notes: 1985-09 p52, 1985-10 p58, 1985-11 p13

    Notes about the ROM code:
    * All state is held in the CPU registers so the entire RAM can be
      used: BC = write pointer, DE = read pointer, HL = byte count,
      B' = count limit (high byte), C' = wrap point (high byte).
    * No subroutine calls; the stack is not used.
    * DRAM refresh: every main loop pass and the RAM test run through
      NOP slides located between XX80 and XXF0, so the 64K DRAMs see all
      row addresses at least once every 4 ms.
    * The ROM was entered from the scanned magazine listing (Table 1,
      August 1985), with a bug in the printed listing fixed by inserting
      a 0x00 NOP at 0x382, shifting bytes 0x382-0x3ee to 0x383-0x3ef so
      the hard-coded addresses point at the right places.
    * Cross-checked glyph by glyph against the listing scan: all bytes
      match except 0x0098, which the magazine prints as B8 (JP 0x00B8)
      and the first transcription had as BB. With BB the 16K branch
      skipped LD HL,0x4000, so the power-on clear started at a random HL
      and strobed 4096 NULs to the printer. Corrected here.
    * Power-on sizes the RAM by writing/reading 0x8000 and 0xC000 and
      clears it. With no RAM fitted at all it reports 16K; the article's
      commissioning test (TEST without RAM -> "RAM Fail at location :
      4000" after about ten seconds) is available as RAM size "0K".
    * TEST switch (INT, IM 1 -> 0x0038 -> 0x0400) is only accepted in the
      EI/NOP/DI window at 0x010E, i.e. while the buffer is empty. It
      prints "Ram test started / Ram present : nnK", writes a pattern,
      waits ~9 s for DRAM retention, verifies and prints "Ram test
      completed" or the failing address with the data written and
      returned (whole test ~18/28/37 s for 16K/32K/48K at 2 MHz). The
      buffer pointers are reset afterwards.
    * CLEAR switch (NMI -> 0x0066 -> 0x0080) restarts from scratch.
    * Capacity is 0x3FFF / 0x7FFF / 0xBFFF bytes for 16K / 32K / 48K.

    Memory map:
    0x0000-0x07ff   ROM
    0x1000-0x1fff   read: parallel data in, strobes ACK and clears BUSY
    0x2000-0x2fff   write: data to parallel out
    0x3000-0x3fff   write: strobe parallel out
    0x4000-0xffff   RAM (16K, 32K or 48K, all used for the buffer)

    I/O map:
    0x0000-0xffff   any read: d6 = strobe received, d7 = printer not busy

    On the input port only STB, D0-D7, ACK and BUSY are connected.
    Input handshake (Fig. 2, July 1985): STB drives the asynchronous
    preset of the 74LS74 IC3b, whose Q clocks the data into the 74LS374
    IC15 and whose /Q gives BUSY (via IC2e) and status d6 (via IC2d).
    The read decode of 0x1000-0x1fff clears IC3b and is also the ACK.
    RESET is not connected to IC3b, so its power-on state is undefined;
    the firmware clears it with a dummy read at 0x00C8 once the RAM is
    cleared ("buffer ready ... after about a second").

***************************************************************************/

#include "emu.h"
#include "etibuffer.h"

#include "cpu/z80/z80.h"
#include "machine/output_latch.h"

#include "screen.h"

#include "etibuffer.lh"


namespace {

class etiprintbuffer_device : public device_t, public device_centronics_peripheral_interface
{
public:
	etiprintbuffer_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	DECLARE_INPUT_CHANGED_MEMBER(test_sw);
	DECLARE_INPUT_CHANGED_MEMBER(clear_sw);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_centronics_peripheral_interface implementation
	virtual void input_strobe(int state) override;
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

private:
	static constexpr int STROBE_DELAY = 1;  // usec
	static constexpr int ACK_DELAY = 1;     // usec
	static constexpr int SCREEN_WIDTH = 16;
	static constexpr int SCREEN_HEIGHT = 384;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 status_r();
	u8 data_in_r();
	void data_out_w(u8 data);
	void strobe_out_w(u8 data);

	void busy_w(int state);

	TIMER_CALLBACK_MEMBER(ack_tick);
	TIMER_CALLBACK_MEMBER(strobe_out_tick);

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	static void drawbar(double xval1, double xval2, double x1, double x2, double y1, double y2, int width, bitmap_rgb32 &bitmap, u32 color);
	static void draw7seg(u8 digit, int x0, int y0, int w, int h, int t, bitmap_rgb32 &bitmap, u32 color);

	required_device<z80_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<centronics_device> m_ctx;
	required_device<output_latch_device> m_ctx_data_out;
	required_ioport m_config;

	output_finder<> m_printerready_led;
	output_finder<> m_bufferready_led;

	emu_timer *m_ack_timer;
	emu_timer *m_strobeout_timer;

	std::unique_ptr<u8[]> m_ram;    // up to 48K

	u8 m_data;              // data from centronics input
	u8 m_datalatch;         // latched when strobed
	bool m_strobereceived;  // input strobe flip-flop
	bool m_strobe;          // current state of input strobe line
	bool m_busy;            // current state of printer busy line

	u16 m_buffersize;       // for the graphic representation of the buffer
	u16 m_bufferhead;
	u16 m_buffertail;
};


etiprintbuffer_device::etiprintbuffer_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ETIPRINTBUFFER, tag, owner, clock)
	, device_centronics_peripheral_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_screen(*this, "screen")
	, m_ctx(*this, "prn")
	, m_ctx_data_out(*this, "ctx_data_out")
	, m_config(*this, "CONFIG")
	, m_printerready_led(*this, "eti_printerready_led")
	, m_bufferready_led(*this, "eti_bufferready_led")
	, m_ack_timer(nullptr)
	, m_strobeout_timer(nullptr)
	, m_data(0)
	, m_datalatch(0)
	, m_strobereceived(false)
	, m_strobe(true)
	, m_busy(false)
	, m_buffersize(0)
	, m_bufferhead(0x4000)
	, m_buffertail(0x4000)
{
}


//--------------------------------------------------
//  address maps
//--------------------------------------------------

void etiprintbuffer_device::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x07ff).rom().region("maincpu", 0);
	map(0x1000, 0x1fff).r(FUNC(etiprintbuffer_device::data_in_r));
	map(0x2000, 0x2fff).w(FUNC(etiprintbuffer_device::data_out_w));
	map(0x3000, 0x3fff).w(FUNC(etiprintbuffer_device::strobe_out_w));
	// RAM at 0x4000 is installed in device_reset according to the configured size
}

void etiprintbuffer_device::io_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0xffff).r(FUNC(etiprintbuffer_device::status_r));
}


//--------------------------------------------------
//  memory and I/O handlers
//--------------------------------------------------

u8 etiprintbuffer_device::status_r()
{
	return (m_strobereceived ? 0x40 : 0x00) |  // d6: strobe received
			(m_busy ? 0x00 : 0x80);           // d7: printer not busy
}

u8 etiprintbuffer_device::data_in_r()
{
	if (!machine().side_effects_disabled())
	{
		output_ack(0);
		m_strobereceived = false;              // clear the flip-flop
		m_bufferready_led = 1;
		output_busy(0);
		m_ack_timer->adjust(attotime::from_usec(ACK_DELAY));
	}
	return m_datalatch;
}

void etiprintbuffer_device::data_out_w(u8 data)
{
	m_ctx_data_out->write(data);
}

void etiprintbuffer_device::strobe_out_w(u8 data)
{
	m_ctx->write_strobe(0);
	m_strobeout_timer->adjust(attotime::from_usec(STROBE_DELAY));
}


//--------------------------------------------------
//  centronics lines
//--------------------------------------------------

void etiprintbuffer_device::busy_w(int state)
{
	m_busy = state;
	m_printerready_led = !m_busy;
}

void etiprintbuffer_device::input_strobe(int state)
{
	if (m_strobe && !state)
	{
		m_datalatch = m_data;
		m_strobereceived = true;
		output_busy(1);
		m_bufferready_led = 0;
	}
	m_strobe = state;
}

TIMER_CALLBACK_MEMBER(etiprintbuffer_device::ack_tick)
{
	output_ack(1);
	output_busy(m_strobereceived ? 1 : 0);
}

TIMER_CALLBACK_MEMBER(etiprintbuffer_device::strobe_out_tick)
{
	m_ctx->write_strobe(1);
}


//--------------------------------------------------
//  switches
//--------------------------------------------------

INPUT_CHANGED_MEMBER(etiprintbuffer_device::clear_sw)
{
	m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? CLEAR_LINE : ASSERT_LINE);
}

INPUT_CHANGED_MEMBER(etiprintbuffer_device::test_sw)
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, newval ? CLEAR_LINE : ASSERT_LINE);
}


//--------------------------------------------------
//  device_t implementation
//--------------------------------------------------

void etiprintbuffer_device::device_start()
{
	m_ack_timer = timer_alloc(FUNC(etiprintbuffer_device::ack_tick), this);
	m_strobeout_timer = timer_alloc(FUNC(etiprintbuffer_device::strobe_out_tick), this);

	m_ram = std::make_unique<u8[]>(0xc000);

	save_pointer(NAME(m_ram), 0xc000);
	save_item(NAME(m_data));
	save_item(NAME(m_datalatch));
	save_item(NAME(m_strobereceived));
	save_item(NAME(m_strobe));
	save_item(NAME(m_busy));
}

void etiprintbuffer_device::device_reset()
{
	// install 16K blocks of RAM at 0x4000 according to the configuration
	address_space &mem = m_maincpu->space(AS_PROGRAM);
	const unsigned blocks = m_config->read() & 0x03;
	mem.unmap_readwrite(0x4000, 0xffff);
	if (blocks)
		mem.install_ram(0x4000, 0x4000 + (blocks * 0x4000) - 1, m_ram.get());

	// IC3b is not reset by RESET and powers up in an undefined state; take
	// the set state (BUSY) so the computer is held off until the firmware's
	// dummy read of 0x1000 at 0x00C8 releases it.
	m_strobereceived = true;
	m_bufferready_led = 0;
	output_busy(1);
	output_ack(1);
	output_select(1);  // not connected; Apple II Grappler cards expect SELECT high
	output_fault(1);   // not connected
	output_perror(0);  // not connected

	m_ctx->write_strobe(1);
}


//--------------------------------------------------
//  buffer display
//--------------------------------------------------

void etiprintbuffer_device::drawbar(double xval1, double xval2, double x1, double x2, double y1, double y2, int width, bitmap_rgb32 &bitmap, u32 color)
{
	const double pct1 = (xval1 - x1) / (x2 - x1);
	const double pct2 = (xval2 - x1) / (x2 - x1);
	const double yval1 = pct1 * (y2 - y1) + y1;
	const double yval2 = pct2 * (y2 - y1) + y1;
	bitmap.plot_box(0, int(yval1), width, int(yval2 - yval1 + 1), color);
}

void etiprintbuffer_device::draw7seg(u8 digit, int x0, int y0, int w, int h, int t, bitmap_rgb32 &bitmap, u32 color)
{
	static const u8 pat[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };
	const u8 seg = pat[digit & 0xf];

	if (BIT(seg, 0)) bitmap.plot_box(x0,     y0,                 w, t, color);
	if (BIT(seg, 1)) bitmap.plot_box(x0 + w, y0 + t,             t, h, color);
	if (BIT(seg, 2)) bitmap.plot_box(x0 + w, y0 + t + h + t,     t, h, color);
	if (BIT(seg, 3)) bitmap.plot_box(x0,     y0 + t + h + t + h, w, t, color);
	if (BIT(seg, 4)) bitmap.plot_box(x0 - t, y0 + t + h + t,     t, h, color);
	if (BIT(seg, 5)) bitmap.plot_box(x0 - t, y0 + t,             t, h, color);
	if (BIT(seg, 6)) bitmap.plot_box(x0,     y0 + t + h,         w, t, color);
}

u32 etiprintbuffer_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	constexpr int ledsize = 8;
	constexpr u32 ledoncolor = 0xff0000;
	constexpr u32 ledoffcolor = 0x7f0000;
	constexpr u32 bufferheadcolor = 0x00ff00;
	constexpr u32 buffertailcolor = 0x00ffff;
	constexpr u32 datacolor = 0x888888;
	constexpr u32 emptycolor = 0x000000;
	constexpr u32 fullcolor = 0xbbbbdd;

	// sample the register-held state, except where the firmware has the
	// registers temporarily swapped or reinitialised (avoids flicker)
	const u16 pc = m_maincpu->state_int(Z80_PC);
	if (!(pc >= 0x100 && pc <= 0x106) &&
			!(pc >= 0x11a && pc <= 0x11c) &&
			!(pc >= 0x134 && pc <= 0x137) &&
			!(pc >= 0x159 && pc <= 0x15b))
	{
		m_buffersize = m_maincpu->state_int(Z80_HL);  // HL is the byte count
		m_bufferhead = m_maincpu->state_int(Z80_BC);  // BC is the write pointer
		m_buffertail = m_maincpu->state_int(Z80_DE);  // DE is the read pointer
	}

	drawbar(0, 1, 0, 1, ledsize, SCREEN_HEIGHT, 16, bitmap, emptycolor);

	const u32 barcolor = (m_buffersize > 0xbfc0) ? fullcolor : datacolor;
	if (m_bufferhead >= m_buffertail)
	{
		drawbar(m_buffertail, m_bufferhead, 0x4000, 0xffff, ledsize, SCREEN_HEIGHT, 16, bitmap, barcolor);
	}
	else
	{
		drawbar(0x4000, m_bufferhead, 0x4000, 0xffff, ledsize, SCREEN_HEIGHT, 16, bitmap, barcolor);
		drawbar(m_buffertail, 0xffff, 0x4000, 0xffff, ledsize, SCREEN_HEIGHT, 16, bitmap, barcolor);
	}
	drawbar(m_buffertail, m_buffertail + 256, 0x4000, 0xffff, ledsize, SCREEN_HEIGHT, 16, bitmap, buffertailcolor);
	drawbar(m_bufferhead, m_bufferhead + 256, 0x4000, 0xffff, ledsize, SCREEN_HEIGHT, 16, bitmap, bufferheadcolor);

	bitmap.plot_box(0, 0, ledsize, ledsize, !m_busy ? ledoncolor : ledoffcolor);
	bitmap.plot_box(ledsize, 0, ledsize, ledsize, !m_strobereceived ? ledoncolor : ledoffcolor);

	constexpr int seg7w = 4, seg7h = 3, seg7t = 1;
	constexpr u32 seg7color = 0xff2222;
	draw7seg((m_buffersize >> 0) & 0xf,  9, 16, seg7w, seg7h, seg7t, bitmap, seg7color);
	draw7seg((m_buffersize >> 4) & 0xf,  2, 16, seg7w, seg7h, seg7t, bitmap, seg7color);
	draw7seg((m_buffersize >> 8) & 0xf,  9, 30, seg7w, seg7h, seg7t, bitmap, seg7color);
	draw7seg((m_buffersize >> 12) & 0xf, 2, 30, seg7w, seg7h, seg7t, bitmap, seg7color);

	return 0;
}


//--------------------------------------------------
//  configuration
//--------------------------------------------------

static INPUT_PORTS_START(etiprintbuffer)
	PORT_START("TEST")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("ETI Buffer Test") PORT_CODE(KEYCODE_2_PAD)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(etiprintbuffer_device::test_sw), 0)

	PORT_START("CLEAR")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("ETI Buffer Clear") PORT_CODE(KEYCODE_5_PAD)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(etiprintbuffer_device::clear_sw), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME(0x03, 0x03, "ETI Buffer RAM Size")
	PORT_CONFSETTING(0x00, "0K (commissioning test)")
	PORT_CONFSETTING(0x01, "16K")
	PORT_CONFSETTING(0x02, "32K")
	PORT_CONFSETTING(0x03, "48K")
INPUT_PORTS_END

ioport_constructor etiprintbuffer_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(etiprintbuffer);
}

ROM_START(etiprintbuffer)
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD("etibuffer.rom", 0x0000, 0x0800, CRC(4cae68f2) SHA1(33a4eae61fb1bc7815a1a6bf18e2523651bb0054))
ROM_END

const tiny_rom_entry *etiprintbuffer_device::device_rom_region() const
{
	return ROM_NAME(etiprintbuffer);
}

void etiprintbuffer_device::device_add_mconfig(machine_config &config)
{
	Z80(config, m_maincpu, 2_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &etiprintbuffer_device::mem_map);
	m_maincpu->set_addrmap(AS_IO, &etiprintbuffer_device::io_map);

	CENTRONICS(config, m_ctx, centronics_devices, "printer");
	m_ctx->busy_handler().set(FUNC(etiprintbuffer_device::busy_w));

	OUTPUT_LATCH(config, m_ctx_data_out);
	m_ctx->set_output_latch(*m_ctx_data_out);

	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(60);
	m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(0));
	m_screen->set_size(SCREEN_WIDTH, SCREEN_HEIGHT);
	m_screen->set_visarea(0, SCREEN_WIDTH - 1, 0, SCREEN_HEIGHT - 1);
	m_screen->set_screen_update(FUNC(etiprintbuffer_device::screen_update));
	m_screen->set_physical_aspect(1, 40);

	config.set_default_layout(layout_etibuffer);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(ETIPRINTBUFFER, device_centronics_peripheral_interface, etiprintbuffer_device, "etiprintbuffer", "Electronics Today International Print Buffer 48K")
