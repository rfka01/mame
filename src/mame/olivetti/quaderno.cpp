// license:BSD-3-Clause
// copyright-holders:rfka01
/****************************************************************************

    Olivetti Quaderno (1992) - XT class sub-notebook with voice processing
    (sold in some countries as Quaderno XT-20)

    Main board (Pegasus Electronics, Japan; date codes 1992):
        NEC V30HL @ 16 MHz (32 MHz XTAL / 2), switchable 16/8/4/2 MHz
        NEC D7?20??GD large QFP (part number not readable, under a label)
        Pegasus LT7-441017 gate array: system logic, DCGA LCD controller,
            power management, shadow RAM, EMS, memory windows
        2x Hitachi HM514800LTT8 (512Kx8 DRAM) = 1 MB:
            640K base + 64K shadow + 320K LIM EMS
        Analog Devices ADSP-2111 KS-52 DSP (voice record/playback)
        Analog Devices AD28MSP02 voiceband codec (on DSP SPORT0)
        Mitsubishi M37410M4 (mask 441028-122): keyboard matrix, real time
            clock, status LCD, voice buttons, power switch
        128 KB system ROM (U4, "ABG R1.88"): MS-ROM-DOS 5.0 kernel at
            E0000, BIOS at F0000
        512 KB ROM ("FW-AAG R1.05"): ROM disk (ROMBIO.SYS, COMMAND.COM,
            INTERLNK, ...), XBIOS DSP programs, code page fonts
        32 KB battery backed SRAM: RAM disk part of D:, CMOS setup data
        2.5" 20 MB hard disk on an IBM XT compatible controller (320h)
        PCMCIA 1.0 type II slot (SRAM/ROM/flash), RS-232, parallel/FDD

    Everything below was reverse engineered from BIOS 1.88 (see the
    Quaderno notes in the MAME project folder of the author).

    I/O map:
        000-00f  8237 DMA               020-021  8259 PIC
        026/027  gate array index/data: 10 = shadow RAM mode,
                 11 bit 0 = free running clock used for delays
        040-043  8253 PIT               060      scan code latch (r/w)
        061      port B                 062      configuration switches
        066      channel control        067      keyboard channel data
        068      M37410 command channel 069      channel clock control
        06a      channel status         06c/06e/06f  EMS page registers
        080-08f  DMA page registers
        0a0      channel interrupt status, 0a1 channel interrupt mask
        178/179  gate array index/data (c2 = CPU clock, c4 = NMI mask
                 save/restore, c0 = NMI source)
        300/302  power management index/data:
                     00 bit 2 = ROM window enable, bit 3 = SRAM window
                     10 = 55h resume signature
                     13 bit 7 = AC adapter, bit 5 = new MCU firmware
                     1b bit 6 = hard disk powered down
                     60/61 = ROM window bank, 70/71 = SRAM window bank
        320-323  XT hard disk controller (DMA 3, IRQ 5)
        350-357  ADSP-2111 host interface port
        378      printer (also receives POST codes)
        3d0-3df  LCD controller (CGA + M24 3de, extended CRTC regs)
        3f0-3f7  external floppy        3f8-3ff  COM1

    Memory map:
        00000-9ffff  RAM
        b8000-bffff  32K VRAM, 8x16 font at b e000 (no character ROM)
        d8000-dbffff ROM window (16K banks of the 512K ROM)
        dc000-dffff  SRAM window (16K banks, CMOS data at bank 1 +3900h)
        e0000-effff  ROM-DOS
        f0000-fffff  BIOS, can be shadowed into RAM

    Interrupts: the keyboard/MCU channels share master IRQ1, but the gate
    array supplies vector 71h instead of 09h.  The BIOS handler at INT 71h
    reads the byte, writes it to the port 60h latch and calls INT 09h.

    TODO:
        - EMS page mapping (registers known, page frame address not yet)
        - power management NMI, suspend/resume, lid switch
        - Fn layer of the real keyboard (F11/F12, TURBO, PA hot keys,
          voice transport keys), voice buttons on the lid
        - AD28MSP02 control words, microphone input
        - PCMCIA slot

****************************************************************************/

#include "emu.h"

#include "quaderno_lcd.h"

#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "cpu/adsp2100/adsp2100.h"
#include "cpu/nec/nec.h"
#include "machine/genpc.h"
#include "machine/nvram.h"
#include "machine/pckeybrd.h"
#include "machine/ram.h"
#include "sound/dac.h"

#include "speaker.h"

#include <deque>

#include "quaderno.lh"

#define LOG_PM      (1U << 1)
#define LOG_GA      (1U << 2)
#define LOG_MCU     (1U << 3)
#define LOG_CHAN    (1U << 4)
#define LOG_UNK     (1U << 5)
#define VERBOSE     (LOG_UNK)
#include "logmacro.h"

#define LOGPM(...)   LOGMASKED(LOG_PM, __VA_ARGS__)
#define LOGGA(...)   LOGMASKED(LOG_GA, __VA_ARGS__)
#define LOGMCU(...)  LOGMASKED(LOG_MCU, __VA_ARGS__)
#define LOGCHAN(...) LOGMASKED(LOG_CHAN, __VA_ARGS__)
#define LOGUNK(...)  LOGMASKED(LOG_UNK, __VA_ARGS__)


namespace {

class quaderno_state : public driver_device
{
public:
	quaderno_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_mb(*this, "mb")
		, m_lcd(*this, "lcd")
		, m_kbd(*this, "kbd")
		, m_dsp(*this, "dsp")
		, m_codec_dac(*this, "codec")
		, m_shadow_view(*this, "shadow_view")
		, m_romwin_view(*this, "romwin_view")
		, m_sramwin_view(*this, "sramwin_view")
		, m_rom_bank(*this, "rom_bank")
		, m_sram_bank(*this, "sram_bank")
		, m_shadow(*this, "shadow", 0x10000, ENDIANNESS_LITTLE)
		, m_sram(*this, "sram", 0x20000, ENDIANNESS_LITTLE)
		, m_config(*this, "CONFIG")
		, m_lcd_digit(*this, "lcd_digit%u", 0U)
		, m_lcd_alpha(*this, "lcd_alpha%u", 0U)
		, m_lcd_colon(*this, "lcd_colon%u", 0U)
	{ }

	void quaderno(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	struct channel
	{
		uint8_t latch = 0;
		bool full = false;
	};

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	IRQ_CALLBACK_MEMBER(inta);

	// gate array / power management index registers
	uint8_t r26_r(offs_t offset);
	void r26_w(offs_t offset, uint8_t data);
	uint8_t ga_r(offs_t offset);
	void ga_w(offs_t offset, uint8_t data);
	uint8_t pm_r(offs_t offset);
	void pm_w(offs_t offset, uint8_t data);
	uint8_t ems_r(offs_t offset);
	void ems_w(offs_t offset, uint8_t data);
	void shadow_w(offs_t offset, uint8_t data);
	void update_windows();
	void update_cpu_clock();

	// keyboard / MCU channels
	uint8_t chan_r(offs_t offset);
	void chan_w(offs_t offset, uint8_t data);
	uint8_t auxint_r(offs_t offset);
	void auxint_w(offs_t offset, uint8_t data);
	void kbd_keypress(int state);
	void kbd_fetch();
	void mcu_fetch();
	void update_irq1();
	void mcu_command(uint8_t data);
	void mcu_reply(uint8_t data) { m_mcu_out.push_back(data); }
	TIMER_CALLBACK_MEMBER(rtc_tick);

	// voice subsystem
	uint8_t hip_r(offs_t offset);
	void hip_w(offs_t offset, uint8_t data);
	void dsp_flags_w(uint8_t data);
	uint32_t dsp_sport_rx_r(offs_t offset);
	void dsp_sport_tx_w(offs_t offset, uint32_t data);
	TIMER_CALLBACK_MEMBER(codec_frame);

	// status LCD
	void status_lcd_update();
	static uint8_t lcd_7seg(uint8_t ch);

	required_device<v30_device> m_maincpu;
	required_device<pc_noppi_mb_device> m_mb;
	required_device<quaderno_lcd_device> m_lcd;
	required_device<at_keyboard_device> m_kbd;
	required_device<adsp2111_device> m_dsp;
	required_device<dac_16bit_r2r_twos_complement_device> m_codec_dac;
	memory_view m_shadow_view;
	memory_view m_romwin_view;
	memory_view m_sramwin_view;
	required_memory_bank m_rom_bank;
	required_memory_bank m_sram_bank;
	memory_share_creator<uint8_t> m_shadow;
	memory_share_creator<uint8_t> m_sram;
	required_ioport m_config;
	output_finder<4> m_lcd_digit;
	output_finder<2> m_lcd_alpha;
	output_finder<2> m_lcd_colon;

	emu_timer *m_codec_timer = nullptr;
	emu_timer *m_rtc_timer = nullptr;

	uint8_t m_r26_index = 0;
	uint8_t m_r26[0x100]{};
	uint8_t m_ga_index = 0;
	uint8_t m_ga[0x100]{};
	uint8_t m_pm_index = 0;
	uint8_t m_pm[0x100]{};
	uint8_t m_r308_index = 0;
	uint8_t m_r308[0x100]{};
	uint8_t m_ems_index = 0;
	uint16_t m_ems[0x40]{};

	uint8_t m_latch60 = 0;
	uint8_t m_p66 = 0;
	uint8_t m_p69 = 0;
	uint8_t m_auxmask = 0;
	channel m_chan[2];

	// M37410 HLE
	std::deque<uint8_t> m_mcu_out;
	uint8_t m_mcu_cmd = 0;
	uint8_t m_mcu_need = 0;
	uint8_t m_mcu_got = 0;
	uint8_t m_mcu_data[4]{};
	uint8_t m_mcu_reg[0x40]{};
	uint8_t m_mcu_ram[0x20]{};      // 80-9f command space (status LCD at 90-96)
	uint8_t m_rtc[3]{};             // second, minute, hour (binary)
	uint8_t m_date[4]{};            // day, month, year, century (binary)
	uint8_t m_alarm[3]{};

	uint8_t m_dsp_flags = 0;
	uint16_t m_codec_out = 0;
	bool m_dsp_irq = false;
};


/***************************************************************************
    Memory and I/O maps
***************************************************************************/

void quaderno_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	// 00000-9ffff: base RAM, installed by the motherboard device
	map(0xb8000, 0xbffff).rw(m_lcd, FUNC(quaderno_lcd_device::vram_r), FUNC(quaderno_lcd_device::vram_w));

	map(0xd8000, 0xdbfff).view(m_romwin_view);
	m_romwin_view[0](0xd8000, 0xdbfff).nopr().nopw();
	m_romwin_view[1](0xd8000, 0xdbfff).bankr(m_rom_bank).nopw();

	map(0xdc000, 0xdffff).view(m_sramwin_view);
	m_sramwin_view[0](0xdc000, 0xdffff).nopr().nopw();
	m_sramwin_view[1](0xdc000, 0xdffff).bankrw(m_sram_bank);

	map(0xe0000, 0xeffff).rom().region("bios", 0);

	map(0xf0000, 0xfffff).view(m_shadow_view);
	m_shadow_view[0](0xf0000, 0xfffff).rom().region("bios", 0x10000);
	m_shadow_view[1](0xf0000, 0xfffff).rom().region("bios", 0x10000).w(FUNC(quaderno_state::shadow_w));
	m_shadow_view[2](0xf0000, 0xfffff).readonly().share(m_shadow);
	m_shadow_view[3](0xf0000, 0xfffff).ram().share(m_shadow);
}

void quaderno_state::io_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x00ff).m(m_mb, FUNC(pc_noppi_mb_device::map));
	map(0x0026, 0x0027).rw(FUNC(quaderno_state::r26_r), FUNC(quaderno_state::r26_w));
	map(0x0060, 0x0060).lrw8(
			NAME([this] () { return m_latch60; }),
			NAME([this] (uint8_t data) { m_latch60 = data; }));
	map(0x0062, 0x0062).lr8(NAME([this] () -> uint8_t { return BIT(m_config->read(), 1) ? 0x04 : 0x00; }));
	map(0x0066, 0x006a).rw(FUNC(quaderno_state::chan_r), FUNC(quaderno_state::chan_w));
	map(0x006c, 0x006f).rw(FUNC(quaderno_state::ems_r), FUNC(quaderno_state::ems_w));
	map(0x00a0, 0x00a1).rw(FUNC(quaderno_state::auxint_r), FUNC(quaderno_state::auxint_w));
	map(0x0178, 0x0179).rw(FUNC(quaderno_state::ga_r), FUNC(quaderno_state::ga_w));
	map(0x0300, 0x030b).rw(FUNC(quaderno_state::pm_r), FUNC(quaderno_state::pm_w));
	map(0x0350, 0x0357).rw(FUNC(quaderno_state::hip_r), FUNC(quaderno_state::hip_w));
	map(0x03d0, 0x03df).rw(m_lcd, FUNC(quaderno_lcd_device::io_r), FUNC(quaderno_lcd_device::io_w));
}

IRQ_CALLBACK_MEMBER(quaderno_state::inta)
{
	uint8_t vector = m_mb->m_pic8259->acknowledge();
	// the gate array answers the keyboard/MCU interrupt with vector 71h
	if ((vector & 7) == 1)
		vector = 0x71;
	return vector;
}


/***************************************************************************
    Gate array registers
***************************************************************************/

uint8_t quaderno_state::r26_r(offs_t offset)
{
	if (!offset)
		return m_r26_index;

	switch (m_r26_index)
	{
		case 0x11:
			// free running clock used by the BIOS for delays (assumed 32768 Hz)
			return (machine().time().as_ticks(32768 * 2) & 1) | (m_r26[0x11] & 0xfe);
		default:
			return m_r26[m_r26_index];
	}
}

void quaderno_state::r26_w(offs_t offset, uint8_t data)
{
	if (!offset)
	{
		m_r26_index = data;
		return;
	}

	LOGGA("%s: R26[%02x] <- %02x\n", machine().describe_context(), m_r26_index, data);
	m_r26[m_r26_index] = data;
	if (m_r26_index == 0x10)
	{
		// shadow RAM for F0000-FFFFF: 0 = ROM, 1 = read ROM/write RAM,
		// 2 = RAM read only, 3 = RAM read/write
		m_shadow_view.select(data & 3);
	}
}

void quaderno_state::shadow_w(offs_t offset, uint8_t data)
{
	m_shadow[offset] = data;
}

uint8_t quaderno_state::ga_r(offs_t offset)
{
	if (!offset)
		return m_ga_index;
	const uint8_t data = m_ga[m_ga_index];
	if (!machine().side_effects_disabled())
		LOGGA("%s: GA[%02x] -> %02x\n", machine().describe_context(), m_ga_index, data);
	return data;
}

void quaderno_state::ga_w(offs_t offset, uint8_t data)
{
	if (!offset)
	{
		m_ga_index = data;
		return;
	}

	LOGGA("%s: GA[%02x] <- %02x\n", machine().describe_context(), m_ga_index, data);
	m_ga[m_ga_index] = data;
	if (m_ga_index == 0xc2)
		update_cpu_clock();
}

void quaderno_state::update_cpu_clock()
{
	// GA register c2 bits 0-2: 0 = 16 MHz, 6 = 8 MHz, 2 = 4 MHz, 3 = 2 MHz
	double scale;
	switch (m_ga[0xc2] & 7)
	{
		case 0:  scale = 1.0;   break;
		case 1:
		case 6:  scale = 0.5;   break;
		case 2:  scale = 0.25;  break;
		case 3:  scale = 0.125; break;
		default: scale = 1.0;   break;
	}
	m_maincpu->set_clock_scale(scale);
}

uint8_t quaderno_state::pm_r(offs_t offset)
{
	uint8_t data = 0xff;
	switch (offset)
	{
		case 0x0: data = m_pm_index; break;
		case 0x2:
			data = m_pm[m_pm_index];
			if (m_pm_index == 0x13)
				data = (data & 0x7f) | (BIT(m_config->read(), 0) << 7);
			break;
		case 0x8: data = m_r308_index; break;
		case 0xa: data = m_r308[m_r308_index]; break;
		default:
			if (!machine().side_effects_disabled())
				LOGUNK("%s: read from port %03x\n", machine().describe_context(), 0x300 + offset);
			break;
	}
	return data;
}

void quaderno_state::pm_w(offs_t offset, uint8_t data)
{
	switch (offset)
	{
		case 0x0: m_pm_index = data; break;
		case 0x2:
			LOGPM("%s: PM[%02x] <- %02x\n", machine().describe_context(), m_pm_index, data);
			m_pm[m_pm_index] = data;
			switch (m_pm_index)
			{
				case 0x00: case 0x60: case 0x61: case 0x70: case 0x71:
					update_windows();
					break;
			}
			break;
		case 0x8: m_r308_index = data; break;
		case 0xa: m_r308[m_r308_index] = data; break;
		default:
			LOGUNK("%s: write to port %03x = %02x\n", machine().describe_context(), 0x300 + offset, data);
			break;
	}
}

void quaderno_state::update_windows()
{
	m_rom_bank->set_entry((m_pm[0x60] | (m_pm[0x61] << 8)) & 0x1f);
	const unsigned sram_banks = BIT(m_config->read(), 1) ? 8 : 2;
	m_sram_bank->set_entry((m_pm[0x70] | (m_pm[0x71] << 8)) % sram_banks);
	m_romwin_view.select(BIT(m_pm[0x00], 2));
	m_sramwin_view.select(BIT(m_pm[0x00], 3));
}

uint8_t quaderno_state::ems_r(offs_t offset)
{
	switch (offset)
	{
		case 0: return m_ems_index;
		case 2: return m_ems[(m_ems_index >> 2) & 0x3f] & 0xff;
		case 3: return m_ems[(m_ems_index >> 2) & 0x3f] >> 8;
		default: return 0xff;
	}
}

void quaderno_state::ems_w(offs_t offset, uint8_t data)
{
	// TODO: EMS mapping registers 80-9c (8 pages) and c0-cc (4)
	uint16_t &reg = m_ems[(m_ems_index >> 2) & 0x3f];
	switch (offset)
	{
		case 0: m_ems_index = data; break;
		case 2: reg = (reg & 0xff00) | data; break;
		case 3: reg = (reg & 0x00ff) | (data << 8); break;
	}
}


/***************************************************************************
    Keyboard / M37410 channels

    066  control: bit 4 rising edge = acknowledge channel 0 byte,
         bit 6 rising edge = acknowledge channel 1 byte; bit 2 read = data ok
    067  channel 0 (keyboard) data: read = received byte (not consumed),
         write = send command to the keyboard
    068  channel 1 (M37410 control) data
    069  clock inhibit bits (3 = ch0, 4 = ch1)
    06a  status: bit 7/4 = ch0/ch1 ready to send,
                 bit 6/3 = ch0/ch1 receive buffer empty
    0a0  pending interrupts: bit 2 = ch0, bit 3 = ch1 (after mask)
    0a1  interrupt mask, same bits
***************************************************************************/

uint8_t quaderno_state::chan_r(offs_t offset)
{
	switch (offset)
	{
		case 0: return m_p66 | 0x04;
		case 1: return m_chan[0].full ? m_chan[0].latch : 0xff;
		case 2: return m_chan[1].full ? m_chan[1].latch : 0xff;
		case 3: return m_p69;
		case 4:
			return 0x80 | 0x10 | (m_chan[0].full ? 0x00 : 0x40) | (m_chan[1].full ? 0x00 : 0x08);
	}
	return 0xff;
}

void quaderno_state::chan_w(offs_t offset, uint8_t data)
{
	switch (offset)
	{
		case 0:
		{
			const uint8_t rise = data & ~m_p66;
			m_p66 = data;
			if (BIT(rise, 4) && m_chan[0].full)
			{
				m_chan[0].full = false;
				kbd_fetch();
			}
			if (BIT(rise, 6) && m_chan[1].full)
			{
				m_chan[1].full = false;
				mcu_fetch();
			}
			update_irq1();
			break;
		}

		case 1:
			LOGCHAN("%s: keyboard <- %02x\n", machine().describe_context(), data);
			if (data == 0xe0)
			{
				// prefix the BIOS sends before retrying a channel 1 command
				if (!m_chan[0].full)
				{
					m_chan[0].latch = 0xfa;
					m_chan[0].full = true;
				}
			}
			else
			{
				m_kbd->write(data);
				kbd_fetch();
			}
			update_irq1();
			break;

		case 2:
			mcu_command(data);
			mcu_fetch();
			update_irq1();
			break;

		case 3:
			m_p69 = data;
			break;
	}
}

void quaderno_state::kbd_keypress(int state)
{
	if (state)
	{
		kbd_fetch();
		update_irq1();
	}
}

void quaderno_state::kbd_fetch()
{
	if (m_chan[0].full)
		return;
	const uint8_t data = m_kbd->read();
	if (data)
	{
		m_chan[0].latch = data;
		m_chan[0].full = true;
	}
}

void quaderno_state::mcu_fetch()
{
	if (m_chan[1].full || m_mcu_out.empty())
		return;
	m_chan[1].latch = m_mcu_out.front();
	m_mcu_out.pop_front();
	m_chan[1].full = true;
}

uint8_t quaderno_state::auxint_r(offs_t offset)
{
	if (offset)
		return m_auxmask;
	return ((m_chan[0].full ? 0x04 : 0) | (m_chan[1].full ? 0x08 : 0)) & ~m_auxmask;
}

void quaderno_state::auxint_w(offs_t offset, uint8_t data)
{
	if (offset)
	{
		m_auxmask = data;
		update_irq1();
	}
}

void quaderno_state::update_irq1()
{
	const uint8_t pending = ((m_chan[0].full ? 0x04 : 0) | (m_chan[1].full ? 0x08 : 0)) & ~m_auxmask;
	m_mb->m_pic8259->ir1_w(pending ? ASSERT_LINE : CLEAR_LINE);
}


/***************************************************************************
    M37410 command channel (HLE)

    Each command is acknowledged with FA, followed by the reply bytes or
    by the data bytes sent by the host (which are not acknowledged).
        00-3f  read MCU register            40-7f  write register (+1)
        80     read time (sec, min, hour)   81     read date (day, month,
        82     read alarm                          year, century)
        83     read status                  84/85/86/87  write time/date/
        88-96  write status LCD/MCU RAM (+1)         alarm/status
        97-9e  read MCU RAM                 9f     ?
        a0     write (+1)                   a1-a3  CPU speed for the LCD
        fc     read firmware version (2)    fd     standby, fe power off
    Status LCD: 90/91 = four hex digits, 92/93 = characters, 95 = colons,
    register 00 = 0 shows the clock.
***************************************************************************/

void quaderno_state::mcu_command(uint8_t data)
{
	if (m_mcu_need)
	{
		m_mcu_data[m_mcu_got++] = data;
		if (m_mcu_got < m_mcu_need)
			return;
		m_mcu_need = 0;
		LOGMCU("MCU command %02x data %02x %02x %02x %02x\n", m_mcu_cmd, m_mcu_data[0], m_mcu_data[1], m_mcu_data[2], m_mcu_data[3]);
		const uint8_t cmd = m_mcu_cmd;
		if (cmd < 0x80)
			m_mcu_reg[cmd & 0x3f] = m_mcu_data[0];
		else if (cmd == 0x84)
			std::copy_n(m_mcu_data, 3, m_rtc);
		else if (cmd == 0x85)
			std::copy_n(m_mcu_data, 4, m_date);
		else if (cmd == 0x86)
			std::copy_n(m_mcu_data, 3, m_alarm);
		else if (cmd >= 0x88 && cmd <= 0x96)
		{
			m_mcu_ram[cmd & 0x1f] = m_mcu_data[0];
			if (cmd >= 0x90 && cmd <= 0x95)
				m_mcu_reg[0] |= 0x01;
		}
		else if (cmd == 0xa0)
			m_mcu_reg[0x3f] = m_mcu_data[0];
		status_lcd_update();
		return;
	}

	LOGMCU("MCU command %02x\n", data);
	m_mcu_cmd = data;
	m_mcu_got = 0;
	mcu_reply(0xfa);

	if (data < 0x40)
		mcu_reply(m_mcu_reg[data]);
	else if (data < 0x80)
		m_mcu_need = 1;
	else
	{
		switch (data)
		{
			case 0x80: for (uint8_t b : m_rtc) mcu_reply(b); break;
			case 0x81: for (uint8_t b : m_date) mcu_reply(b); break;
			case 0x82: for (uint8_t b : m_alarm) mcu_reply(b); break;
			case 0x83: mcu_reply(0x00); break;
			case 0x84: m_mcu_need = 3; break;
			case 0x85: m_mcu_need = 4; break;
			case 0x86: m_mcu_need = 3; break;
			case 0x87: m_mcu_need = 1; break;
			case 0xa0: m_mcu_need = 1; break;
			case 0xfc: mcu_reply(0x01); mcu_reply(0x16); break;
			case 0xfd:
			case 0xfe:
				logerror("MCU: power %s requested\n", (data == 0xfd) ? "standby" : "off");
				break;
			default:
				if (data >= 0x88 && data <= 0x96)
					m_mcu_need = 1;
				else if (data >= 0x97 && data <= 0x9e)
					mcu_reply(m_mcu_ram[data & 0x1f]);
				else if (data >= 0xa1 && data <= 0xa4)
				{
					m_mcu_reg[0x3e] = data - 0xa1;     // CPU speed shown in clock mode
					status_lcd_update();
				}
				break;
		}
	}
}

TIMER_CALLBACK_MEMBER(quaderno_state::rtc_tick)
{
	static constexpr uint8_t mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	if (++m_rtc[0] < 60) { status_lcd_update(); return; }
	m_rtc[0] = 0;
	if (++m_rtc[1] < 60) { status_lcd_update(); return; }
	m_rtc[1] = 0;
	if (++m_rtc[2] < 24) { status_lcd_update(); return; }
	m_rtc[2] = 0;
	const int year = m_date[3] * 100 + m_date[2];
	const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
	const uint8_t days = (m_date[1] == 2 && leap) ? 29 : mdays[(m_date[1] - 1) % 12];
	if (++m_date[0] <= days) return;
	m_date[0] = 1;
	if (++m_date[1] <= 12) return;
	m_date[1] = 1;
	if (++m_date[2] < 100) return;
	m_date[2] = 0;
	m_date[3]++;
}


/***************************************************************************
    Voice subsystem: ADSP-2111 + AD28MSP02 codec

    The DSP has no ROM: the BIOS POST and XBIOS boot it through the HIP.
    The vocoders set FL2 when a block is ready; FL2 drives IRQ2.
***************************************************************************/

uint8_t quaderno_state::hip_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		machine().scheduler().perfect_quantum(attotime::from_usec(50));
	return m_dsp->hip_r(offset);
}

void quaderno_state::hip_w(offs_t offset, uint8_t data)
{
	m_dsp->hip_w(offset, data);
	machine().scheduler().perfect_quantum(attotime::from_usec(50));
}

void quaderno_state::dsp_flags_w(uint8_t data)
{
	m_dsp_flags = data;
	const bool irq = BIT(data, 3);   // FL2
	if (irq != m_dsp_irq)
	{
		m_dsp_irq = irq;
		m_mb->m_pic8259->ir2_w(irq ? ASSERT_LINE : CLEAR_LINE);
	}
}

uint32_t quaderno_state::dsp_sport_rx_r(offs_t offset)
{
	return 0; // TODO: microphone
}

void quaderno_state::dsp_sport_tx_w(offs_t offset, uint32_t data)
{
	if (offset == 0)
		m_codec_out = data & 0xffff;
}

TIMER_CALLBACK_MEMBER(quaderno_state::codec_frame)
{
	m_codec_dac->write(m_codec_out);
	m_dsp->pulse_input_line(ADSP2111_SPORT0_RX, attotime::zero);
	m_dsp->pulse_input_line(ADSP2111_SPORT0_TX, attotime::zero);
}


/***************************************************************************
    Status LCD
***************************************************************************/

uint8_t quaderno_state::lcd_7seg(uint8_t ch)
{
	static constexpr uint8_t hex[16] = {
			0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
			0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x79, 0x71 };
	if (ch < 0x10)
		return hex[ch];
	if (ch >= '0' && ch <= '9')
		return hex[ch - '0'];
	switch (ch)
	{
		case '-': return 0x40;
		case '+': return 0x46;
		case 'H': return 0x76; case 'h': return 0x74;
		case 'I': case 'i': return 0x06;
		case 'L': case 'l': return 0x38;
		case 'M': case 'm': return 0x37;
		case 'O': return 0x3f; case 'o': return 0x5c;
		case 'A': case 'a': return 0x77;
		case 'B': case 'b': return 0x7c;
		case 'C': return 0x39; case 'c': return 0x58;
		case 'D': case 'd': return 0x5e;
		case 'E': case 'e': return 0x79;
		case 'F': case 'f': return 0x71;
		case 'N': case 'n': return 0x54;
		case 'P': case 'p': return 0x73;
		case 'R': case 'r': return 0x50;
		case 'S': case 's': return 0x6d;
		case 'T': case 't': return 0x78;
		case 'U': return 0x3e; case 'u': return 0x1c;
		case 'Y': case 'y': return 0x6e;
		default: return 0x00;
	}
}

void quaderno_state::status_lcd_update()
{
	if (!(m_mcu_reg[0] & 0xe3))
	{
		// clock mode: HH:MM plus CPU speed
		static const char *const speed[4] = { "HI", "Md", "Lo", "Lo" };
		m_lcd_digit[0] = lcd_7seg(m_rtc[2] / 10);
		m_lcd_digit[1] = lcd_7seg(m_rtc[2] % 10);
		m_lcd_digit[2] = lcd_7seg(m_rtc[1] / 10);
		m_lcd_digit[3] = lcd_7seg(m_rtc[1] % 10);
		m_lcd_alpha[0] = lcd_7seg(speed[m_mcu_reg[0x3e] & 3][0]);
		m_lcd_alpha[1] = lcd_7seg(speed[m_mcu_reg[0x3e] & 3][1]);
		m_lcd_colon[1] = BIT(m_rtc[0], 0);
		m_lcd_colon[0] = 0;
		return;
	}

	const uint16_t value = (m_mcu_ram[0x10] << 8) | m_mcu_ram[0x11];
	for (int i = 0; i < 4; i++)
		m_lcd_digit[i] = lcd_7seg((value >> (12 - i * 4)) & 0x0f);
	m_lcd_alpha[0] = lcd_7seg(m_mcu_ram[0x12]);
	m_lcd_alpha[1] = lcd_7seg(m_mcu_ram[0x13]);
	m_lcd_colon[0] = BIT(m_mcu_ram[0x15], 0);
	m_lcd_colon[1] = BIT(m_mcu_ram[0x15], 1);
}


/***************************************************************************
    Machine
***************************************************************************/

void quaderno_state::machine_start()
{
	m_rom_bank->configure_entries(0, 32, memregion("sdisk")->base(), 0x4000);
	m_sram_bank->configure_entries(0, 8, &m_sram[0], 0x4000);
	subdevice<nvram_device>("nvram")->set_base(&m_sram[0], 0x20000);

	m_codec_timer = timer_alloc(FUNC(quaderno_state::codec_frame), this);
	m_rtc_timer = timer_alloc(FUNC(quaderno_state::rtc_tick), this);

	system_time systime;
	machine().base_datetime(systime);
	m_rtc[0] = systime.local_time.second;
	m_rtc[1] = systime.local_time.minute;
	m_rtc[2] = systime.local_time.hour;
	m_date[0] = systime.local_time.mday;
	m_date[1] = systime.local_time.month + 1;
	m_date[2] = systime.local_time.year % 100;
	m_date[3] = systime.local_time.year / 100;

	save_item(NAME(m_r26_index));
	save_item(NAME(m_r26));
	save_item(NAME(m_ga_index));
	save_item(NAME(m_ga));
	save_item(NAME(m_pm_index));
	save_item(NAME(m_pm));
	save_item(NAME(m_r308_index));
	save_item(NAME(m_r308));
	save_item(NAME(m_ems_index));
	save_item(NAME(m_ems));
	save_item(NAME(m_latch60));
	save_item(NAME(m_p66));
	save_item(NAME(m_p69));
	save_item(NAME(m_auxmask));
	save_item(STRUCT_MEMBER(m_chan, latch));
	save_item(STRUCT_MEMBER(m_chan, full));
	save_item(NAME(m_mcu_cmd));
	save_item(NAME(m_mcu_need));
	save_item(NAME(m_mcu_got));
	save_item(NAME(m_mcu_data));
	save_item(NAME(m_mcu_reg));
	save_item(NAME(m_mcu_ram));
	save_item(NAME(m_rtc));
	save_item(NAME(m_date));
	save_item(NAME(m_alarm));
	save_item(NAME(m_dsp_flags));
	save_item(NAME(m_codec_out));
	save_item(NAME(m_dsp_irq));
}

void quaderno_state::machine_reset()
{
	std::fill(std::begin(m_r26), std::end(m_r26), 0);
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	std::fill(std::begin(m_pm), std::end(m_pm), 0);
	m_shadow_view.select(0);
	update_windows();
	update_cpu_clock();

	m_latch60 = 0;
	m_p66 = 0;
	m_p69 = 0;
	m_auxmask = 0;
	m_chan[0] = m_chan[1] = channel();
	m_mcu_out.clear();
	m_mcu_need = 0;
	update_irq1();

	m_dsp_flags = 0;
	m_dsp_irq = false;
	m_codec_timer->adjust(attotime::from_hz(8000), 0, attotime::from_hz(8000));
	m_rtc_timer->adjust(attotime::from_seconds(1), 0, attotime::from_seconds(1));
	status_lcd_update();
}


static INPUT_PORTS_START( quaderno )
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Power source" )
	PORT_CONFSETTING(    0x00, "Battery" )
	PORT_CONFSETTING(    0x01, "AC adapter" )
	PORT_CONFNAME( 0x02, 0x00, "SRAM" )
	PORT_CONFSETTING(    0x00, "32 KB" )
	PORT_CONFSETTING(    0x02, "128 KB" )
INPUT_PORTS_END

static DEVICE_INPUT_DEFAULTS_START( quaderno_hdc )
	DEVICE_INPUT_DEFAULTS("ROM", 0x01, 0x00)    // the system BIOS drives the controller
DEVICE_INPUT_DEFAULTS_END


void quaderno_state::quaderno(machine_config &config)
{
	V30(config, m_maincpu, 32_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &quaderno_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &quaderno_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(quaderno_state::inta));

	PCNOPPI_MOTHERBOARD(config, m_mb);
	m_mb->set_cputag(m_maincpu);
	m_mb->int_callback().set_inputline(m_maincpu, 0);
	m_mb->nmi_callback().set_inputline(m_maincpu, INPUT_LINE_NMI);

	// internal peripherals with standard register sets
	ISA8_SLOT(config, "fdc", 0, "mb:isa", pc_isa8_cards, "fdc_xt", true);
	ISA8_SLOT(config, "com", 0, "mb:isa", pc_isa8_cards, "com", true);
	ISA8_SLOT(config, "lpt", 0, "mb:isa", pc_isa8_cards, "lpt", true);
	ISA8_SLOT(config, "hdc", 0, "mb:isa", pc_isa8_cards, "hdc", true)
			.set_option_device_input_defaults("hdc", DEVICE_INPUT_DEFAULTS_NAME(quaderno_hdc));

	// the M37410 presents the keyboard like an AT keyboard in scan code set 1
	AT_KEYB(config, m_kbd, pc_keyboard_device::KEYBOARD_TYPE::MF2, 1);
	m_kbd->keypress().set(FUNC(quaderno_state::kbd_keypress));

	RAM(config, RAM_TAG).set_default_size("640K");
	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	QUADERNO_LCD(config, m_lcd);
	config.set_default_layout(layout_quaderno);

	// voice processing
	ADSP2111(config, m_dsp, 13_MHz_XTAL);   // KS-52 speed grade, clock source not verified
	m_dsp->flag_out().set(FUNC(quaderno_state::dsp_flags_w));
	m_dsp->sport_rx().set(FUNC(quaderno_state::dsp_sport_rx_r));
	m_dsp->sport_tx().set(FUNC(quaderno_state::dsp_sport_tx_w));

	SPEAKER(config, "voice").front_center();
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_codec_dac, 0).add_route(ALL_OUTPUTS, "voice", 0.5);
}


ROM_START( quaderno )
	// U4: MS-ROM-DOS 5.0 (E0000) + BIOS "Resident Diagnostics Rev 1.88" 10/23/92 (F0000)
	ROM_REGION( 0x20000, "bios", 0 )
	ROM_LOAD( "abg_r188.u4", 0x00000, 0x20000, CRC(f64985a6) SHA1(ad1d397bf3e04c42a54b3216d2544923364fdc2c) )

	// ROM disk, XBIOS DSP code and code page fonts
	ROM_REGION( 0x80000, "sdisk", 0 )
	ROM_LOAD( "fw-aag_r105.bin", 0x00000, 0x80000, CRC(6bc196e4) SHA1(da5bc11864ec2d78578d0c8a4d6fab233fb6f964) )

	// Mitsubishi M37410M4 internal mask ROM, emulated at high level
	ROM_REGION( 0x4000, "mcu", ROMREGION_ERASEFF )
	ROM_LOAD( "m37410m4_441028-122.bin", 0x0000, 0x4000, NO_DUMP )
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     CLASS           INIT        COMPANY     FULLNAME    FLAGS
COMP( 1992, quaderno, 0,      0,      quaderno, quaderno, quaderno_state, empty_init, "Olivetti", "Quaderno", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND )
