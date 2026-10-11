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
        378-37a  parallel port (also receives POST codes)
        3d0-3df  LCD controller (CGA + M24 3de, extended CRTC regs)
        3f0-3f7  floppy controller      3f8-3ff  COM1

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
        - MF layer: TURBO and display key, verify the codes of the PA
          keys (MF+F3..F10); voice buttons on the lid; dictation switch
          ("Diktatschalter") on the side
        - AD28MSP02 control words, microphone input
        - PCMCIA slot

****************************************************************************/

#include "emu.h"

#include "quaderno_ext.h"
#include "quaderno_lcd.h"

#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "cpu/adsp2100/adsp2100.h"
#include "cpu/nec/nec.h"
#include "machine/genpc.h"
#include "machine/nvram.h"
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
		, m_ext(*this, "ext")
		, m_dsp(*this, "dsp")
		, m_codec_dac(*this, "codec")
		, m_shadow_view(*this, "shadow_view")
		, m_romwin_view(*this, "romwin_view")
		, m_sramwin_view(*this, "sramwin_view")
		, m_rom_bank(*this, "rom_bank")
		, m_sram_bank(*this, "sram_bank")
		, m_ems_view{ {*this, "ems_view0"}, {*this, "ems_view1"}, {*this, "ems_view2"}, {*this, "ems_view3"} }
		, m_ems_bank(*this, "ems_bank%u", 0U)
		, m_ram(*this, RAM_TAG)
		, m_shadow(*this, "shadow", 0x10000, ENDIANNESS_LITTLE)
		, m_sram(*this, "sram", 0x20000, ENDIANNESS_LITTLE)
		, m_config(*this, "CONFIG")
		, m_keys(*this, "KEY%u", 0U)
		, m_lcd_digit(*this, "lcd_digit%u", 0U)
		, m_lcd_alpha(*this, "lcd_alpha%u", 0U)
		, m_lcd_colon(*this, "lcd_colon%u", 0U)
		, m_led_caps(*this, "lcd_caps")
		, m_led_num(*this, "lcd_num")
		, m_led_scrl(*this, "lcd_scrl")
	{ }

	void quaderno(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(factory_reset);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void device_post_load() override;

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
	void shadow_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);
	void update_windows();
	uint8_t lpt_r(offs_t offset);
	void lpt_w(offs_t offset, uint8_t data);
	void update_cpu_clock();

	// keyboard / MCU channels
	uint8_t chan_r(offs_t offset);
	void chan_w(offs_t offset, uint8_t data);
	uint8_t auxint_r(offs_t offset);
	void auxint_w(offs_t offset, uint8_t data);
	void kbd_fetch();
	void kbd_command(uint8_t data);
	void kbd_send(uint16_t code, bool make);
	void kbd_key(int id, bool make);
	TIMER_CALLBACK_MEMBER(kbd_scan);
	void mcu_fetch();
	void update_irq1();
	void mcu_command(uint8_t data);
	void mcu_reply(uint8_t data) { m_mcu_out.push_back(data); }
	void mcu_event(uint8_t bits);
	TIMER_CALLBACK_MEMBER(nmi_check);
	void update_ems(int reg);
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
	required_device<quaderno_ext_connector_device> m_ext;
	required_device<adsp2111_device> m_dsp;
	required_device<dac_16bit_r2r_twos_complement_device> m_codec_dac;
	memory_view m_shadow_view;
	memory_view m_romwin_view;
	memory_view m_sramwin_view;
	required_memory_bank m_rom_bank;
	required_memory_bank m_sram_bank;
	memory_view m_ems_view[4];
	required_memory_bank_array<4> m_ems_bank;
	required_device<ram_device> m_ram;
	std::unique_ptr<uint8_t[]> m_ems_ram;   // DRAM above 640K: shadow + EMS pages
	memory_share_creator<uint16_t> m_shadow;
	memory_share_creator<uint8_t> m_sram;
	required_ioport m_config;
	required_ioport_array<7> m_keys;
	output_finder<4> m_lcd_digit;
	output_finder<2> m_lcd_alpha;
	output_finder<2> m_lcd_colon;
	output_finder<> m_led_caps;
	output_finder<> m_led_num;
	output_finder<> m_led_scrl;

	emu_timer *m_codec_timer = nullptr;
	emu_timer *m_rtc_timer = nullptr;
	emu_timer *m_kbd_timer = nullptr;
	emu_timer *m_nmi_timer = nullptr;

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
	uint8_t m_lpt_data = 0;
	uint8_t m_lpt_ctrl = 0;
	uint8_t m_p66 = 0;
	uint8_t m_p69 = 0;
	uint8_t m_auxmask = 0;
	channel m_chan[2];

	// M37410 keyboard HLE
	std::deque<uint8_t> m_kbd_out;
	uint16_t m_keystate[7]{};
	uint16_t m_keysent[0x70]{};     // code sent on make, so that the break matches
	uint8_t m_kbd_cmd = 0;          // command waiting for its parameter byte
	bool m_kbd_enabled = true;
	int m_repeat_key = -1;
	int m_repeat_count = 0;

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

	// EMS page frame: four 16K windows, mapping registers c0-cc
	for (int i = 0; i < 4; i++)
	{
		const offs_t start = 0xc0000 + i * 0x4000;
		map(start, start + 0x3fff).view(m_ems_view[i]);
		m_ems_view[i][0](start, start + 0x3fff).nopr().nopw();
		m_ems_view[i][1](start, start + 0x3fff).bankrw(m_ems_bank[i]);
	}

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
	map(0x0176, 0x0176).lw8(NAME([this] (uint8_t data) { m_ga[0xc0] &= data; }));   // NMI acknowledge
	map(0x0178, 0x0179).rw(FUNC(quaderno_state::ga_r), FUNC(quaderno_state::ga_w));
	map(0x0300, 0x030b).rw(FUNC(quaderno_state::pm_r), FUNC(quaderno_state::pm_w));
	map(0x0350, 0x0357).rw(FUNC(quaderno_state::hip_r), FUNC(quaderno_state::hip_w));
	map(0x0378, 0x037a).rw(FUNC(quaderno_state::lpt_r), FUNC(quaderno_state::lpt_w));
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

void quaderno_state::shadow_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_shadow[offset]);
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

/*
    EMS (EMMPT.SYS): 06c = register index, 06e/06f = 16-bit data.  There is
    one register per 16K page of the CPU address space, index = A19-A14 * 4.
    Data bit 15 = mapping enabled, bits 5-0 = 16K page of the 1 MB DRAM.
    The driver maps 80000-9ffff 1:1 (pages 20h-27h) and uses c0000-cffff
    (registers c0-cc) as page frame.
*/
void quaderno_state::ems_w(offs_t offset, uint8_t data)
{
	const int r = (m_ems_index >> 2) & 0x3f;
	uint16_t &reg = m_ems[r];
	switch (offset)
	{
		case 0: m_ems_index = data; return;
		case 2: reg = (reg & 0xff00) | data; break;
		case 3: reg = (reg & 0x00ff) | (data << 8); break;
		default: return;
	}
	update_ems(r);
}

void quaderno_state::update_ems(int reg)
{
	const uint16_t data = m_ems[reg];
	if (reg >= 0x30 && reg <= 0x33)
	{
		const int i = reg - 0x30;
		if (BIT(data, 15))
		{
			m_ems_bank[i]->set_entry(data & 0x3f);
			m_ems_view[i].select(1);
		}
		else
			m_ems_view[i].select(0);
	}
	else if (reg >= 0x20 && reg <= 0x27 && BIT(data, 15) && (data & 0x3f) != reg)
		LOGUNK("EMS: conventional page %05x mapped to DRAM page %02x (not emulated)\n", reg << 14, data & 0x3f);
}


/***************************************************************************
    Parallel port (378-37a)

    The registers are inside the Quaderno, so the BIOS finds LPT1 (and
    can write its POST codes) even with nothing connected.  The signals
    only reach a printer through the printer cable on the connector.
***************************************************************************/

uint8_t quaderno_state::lpt_r(offs_t offset)
{
	switch (offset)
	{
		case 0: return m_lpt_data;
		case 1: return m_ext->lpt_r(1);    // 0xff without printer cable
		default: return m_lpt_ctrl | 0xe0;
	}
}

void quaderno_state::lpt_w(offs_t offset, uint8_t data)
{
	if (offset == 0)
		m_lpt_data = data;
	else if (offset == 2)
		m_lpt_ctrl = data & 0x1f;
	m_ext->lpt_w(offset, data);
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
				kbd_command(data);
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

void quaderno_state::kbd_fetch()
{
	if (m_chan[0].full || m_kbd_out.empty())
		return;
	m_chan[0].latch = m_kbd_out.front();
	m_kbd_out.pop_front();
	m_chan[0].full = true;
}

/***************************************************************************
    M37410 keyboard (HLE)

    The MCU scans the key matrix and talks to the host like an AT keyboard
    in scan code set 1.  The MF key (blue dot between STRG and ALT) has no
    scan code of its own; the MCU uses it as a modifier for the legends
    printed in turquoise:
        MF+F1/F2   F11/F12 (57h/58h)
        MF+F3..F10 NOTIZEN, RECHNER, KALEND., TELEFON, DATEI, ARCHIV,
                   cassette (voice), MENU
        MF+STRG    right STRG (E0 1D), MF+ALT = ALT GR (E0 38)
        MF+ENTER   keypad ENTER (E0 1C)
        MF+DRUCK   TURBO: CPU clock HI -> Md -> Lo
        MF+ROLLEN  inverse display
    The last three are not scan codes: the MCU sets a bit in its register
    11h (80h TURBO, 40h display, 20h PA key) and raises an NMI with gate
    array register C0h bit 7.  The BIOS NMI handler reads register 11h,
    for PA keys also register 17h (bit 0 = F3 ... bit 7 = F10), and calls
    the far hook PA installed with BIOS vector 5Eh (CMOS 2Ch).

    Key ids: 01h-58h are the set 1 make codes, 60h-6Fh are the keys that
    need a prefix, MF and a few host convenience keys.
***************************************************************************/

// code to send for each key id: 0 = none, E0xxh = with E0 prefix,
// E11Dh = PAUSE sequence
constexpr uint16_t quaderno_key_code(int id)
{
	if (id >= 0x01 && id <= 0x58)
		return id;
	switch (id)
	{
		case 0x60: return 0xe049;   // BILD auf
		case 0x61: return 0xe051;   // BILD ab
		case 0x62: return 0xe04b;   // cursor left
		case 0x63: return 0xe04d;   // cursor right
		case 0x64: return 0xe048;   // cursor up
		case 0x65: return 0xe050;   // cursor down
		case 0x66: return 0xe035;   // keypad /
		case 0x67: return 0xe037;   // DRUCK
		case 0x68: return 0xe11d;   // PAUSE
		case 0x6a: return 0xe038;   // ALT GR (host convenience)
		case 0x6b: return 0xe01d;   // right STRG (host convenience)
		case 0x6c: return 0xe047;   // POS1 (host convenience)
		case 0x6d: return 0xe04f;   // ENDE (host convenience)
		case 0x6e: return 0xe052;   // EINFG (host convenience)
		case 0x6f: return 0xe053;   // ENTF (host convenience)
		default:   return 0;
	}
}

constexpr int KEY_STRG = 0x1d;
constexpr int KEY_LSHIFT = 0x2a;
constexpr int KEY_RSHIFT = 0x36;
constexpr int KEY_ALT = 0x38;
constexpr int KEY_CAPS = 0x3a;
constexpr int KEY_NUM = 0x45;
constexpr int KEY_ROLLEN = 0x46;
constexpr int KEY_DRUCK = 0x67;
constexpr int KEY_PAUSE = 0x68;
constexpr int KEY_MF = 0x69;

constexpr bool quaderno_key_repeats(int id)
{
	// like an AT keyboard, SHIFT/STRG/ALT repeat as well; ROM-DOS checks
	// for a held ALT (boot menu) only when it starts
	return id != KEY_CAPS && id != KEY_NUM && id != KEY_ROLLEN && id != KEY_PAUSE && id != KEY_MF;
}

void quaderno_state::kbd_send(uint16_t code, bool make)
{
	if (!code || !m_kbd_enabled)
		return;
	if (code == 0xe11d)
	{
		if (make)
			for (uint8_t b : { 0xe1, 0x1d, 0x45, 0xe1, 0x9d, 0xc5 })
				m_kbd_out.push_back(b);
		return;
	}
	if (code >> 8)
		m_kbd_out.push_back(code >> 8);
	m_kbd_out.push_back((code & 0x7f) | (make ? 0x00 : 0x80));
}

void quaderno_state::kbd_key(int id, bool make)
{
	auto held = [this] (int k) { return BIT(m_keystate[k >> 4], k & 15); };

	if (!make)
	{
		kbd_send(m_keysent[id], false);
		m_keysent[id] = 0;
		if (m_repeat_key == id)
			m_repeat_key = -1;
		return;
	}

	uint16_t code = quaderno_key_code(id);
	if (held(KEY_MF))
	{
		if (id == 0x3b || id == 0x3c)
			code = id + 0x1c;                     // F11, F12
		else if (id >= 0x3d && id <= 0x44)
		{
			// PA keys: bit n of MCU register 17h, BIOS calls the PA hook
			m_mcu_reg[0x17] |= 1 << (id - 0x3d);
			mcu_event(0x20);
			code = 0;
		}
		else if (id == KEY_STRG)
			code = 0xe01d;
		else if (id == KEY_ALT)
			code = 0xe038;
		else if (id == 0x1c)
			code = 0xe01c;                        // keypad ENTER
		else if (id == KEY_DRUCK)
		{
			mcu_event(0x80);                      // TURBO, the BIOS cycles HI/Md/Lo
			code = 0;
		}
		else if (id == KEY_ROLLEN)
		{
			mcu_event(0x40);                      // the BIOS toggles inverse video
			code = 0;
		}
	}
	else if (id == KEY_DRUCK && (held(KEY_ALT) || held(0x6a)))
		code = 0x54;                              // S-ABF (SysRq)
	else if (id == KEY_PAUSE && (held(KEY_STRG) || held(0x6b)))
		code = 0xe046;                            // UNTBR (Break)

	m_keysent[id] = (code == 0xe11d) ? 0 : code;
	kbd_send(code, true);
	if (quaderno_key_repeats(id) && code)
	{
		m_repeat_key = id;
		m_repeat_count = 0;
	}
}

TIMER_CALLBACK_MEMBER(quaderno_state::kbd_scan)
{
	// 100 Hz matrix scan; typematic 500 ms delay, 10 characters/s
	for (int port = 0; port < 7; port++)
	{
		const uint16_t now = m_keys[port]->read();
		const uint16_t changed = now ^ m_keystate[port];
		for (int bit = 0; changed >> bit; bit++)
		{
			if (!BIT(changed, bit))
				continue;
			m_keystate[port] ^= 1 << bit;
			kbd_key(port * 16 + bit, BIT(now, bit));
		}
	}

	if (m_repeat_key >= 0 && ++m_repeat_count >= 50 && !((m_repeat_count - 50) % 10) && m_keysent[m_repeat_key])
		kbd_send(m_keysent[m_repeat_key], true);

	if (!m_chan[0].full && !m_kbd_out.empty())
	{
		kbd_fetch();
		update_irq1();
	}
}

void quaderno_state::kbd_command(uint8_t data)
{
	LOGCHAN("keyboard command %02x\n", data);
	if (m_kbd_cmd)
	{
		const uint8_t cmd = m_kbd_cmd;
		m_kbd_cmd = 0;
		if (cmd == 0xed)
		{
			// LEDs are shown on the status LCD
			m_led_scrl = BIT(data, 0);
			m_led_num = BIT(data, 1);
			m_led_caps = BIT(data, 2);
		}
		m_kbd_out.push_front(0xfa);
		return;
	}

	switch (data)
	{
		case 0xed: case 0xf3: case 0xf0:
			m_kbd_cmd = data;
			m_kbd_out.push_front(0xfa);
			break;
		case 0xee:
			m_kbd_out.push_front(0xee);
			break;
		case 0xf2:
			m_kbd_out.clear();
			for (uint8_t b : { 0xfa, 0xab, 0x83 })
				m_kbd_out.push_back(b);
			break;
		case 0xf4:
			m_kbd_enabled = true;
			m_kbd_out.push_front(0xfa);
			break;
		case 0xf5:
			m_kbd_enabled = false;
			m_kbd_out.clear();
			m_kbd_out.push_back(0xfa);
			break;
		case 0xff:
			m_kbd_enabled = true;
			m_kbd_out.clear();
			m_kbd_out.push_back(0xfa);
			m_kbd_out.push_back(0xaa);
			// keys held during the reset (e.g. ALT for the boot menu) are
			// reported again by the next matrix scan
			std::fill(std::begin(m_keystate), std::end(m_keystate), 0);
			std::fill(std::begin(m_keysent), std::end(m_keysent), 0);
			break;
		default:
			m_kbd_out.push_front(0xfa);
			break;
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
	{
		mcu_reply(m_mcu_reg[data]);
		if (data == 0x11 || data == 0x17)
			m_mcu_reg[data] = 0;                  // event registers clear on read
	}
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

void quaderno_state::mcu_event(uint8_t bits)
{
	m_mcu_reg[0x11] |= bits;
	m_ga[0xc0] |= 0x80;
	m_nmi_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(quaderno_state::nmi_check)
{
	// don't interrupt a command the host is exchanging with the MCU
	if (m_mcu_need || !m_mcu_out.empty() || m_chan[1].full)
	{
		m_nmi_timer->adjust(attotime::from_msec(1));
		return;
	}
	if (BIT(m_ga[0xc0], 7))
		m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::zero);
}

INPUT_CHANGED_MEMBER(quaderno_state::factory_reset)
{
	if (!newval)
		return;
	// "Werkseinstellungstaste": the setup data is lost, the machine restarts
	// and asks for date and time.  CMOS lives at DF80:0100 in the SRAM window.
	const offs_t base = (BIT(m_config->read(), 1) ? 7 : 1) * 0x4000 + 0x3900;
	std::fill_n(&m_sram[base], 0x80, 0);
	machine().schedule_soft_reset();
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
	m_dsp->pulse_input_line(ADSP2111_SPORT0_RX, m_dsp->clocks_to_attotime(1));
	m_dsp->pulse_input_line(ADSP2111_SPORT0_TX, m_dsp->clocks_to_attotime(1));
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
	m_kbd_timer = timer_alloc(FUNC(quaderno_state::kbd_scan), this);
	m_nmi_timer = timer_alloc(FUNC(quaderno_state::nmi_check), this);

	// 1 MB DRAM: pages 00-27 are the 640K base RAM, 28-3f are kept here
	m_ems_ram = std::make_unique<uint8_t[]>(0x60000);
	std::fill_n(m_ems_ram.get(), 0x60000, 0);
	save_pointer(NAME(m_ems_ram), 0x60000);
	for (int i = 0; i < 4; i++)
		for (int page = 0; page < 0x40; page++)
			m_ems_bank[i]->configure_entry(page, (page < 0x28)
					? m_ram->pointer() + page * 0x4000
					: m_ems_ram.get() + (page - 0x28) * 0x4000);
	subdevice<quaderno_fdc_device>("fdc:qfdc")->set_connector(m_ext);

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
	save_item(NAME(m_lpt_data));
	save_item(NAME(m_lpt_ctrl));
	save_item(NAME(m_p66));
	save_item(NAME(m_p69));
	save_item(NAME(m_auxmask));
	save_item(STRUCT_MEMBER(m_chan, latch));
	save_item(STRUCT_MEMBER(m_chan, full));
	save_item(NAME(m_keystate));
	save_item(NAME(m_keysent));
	save_item(NAME(m_kbd_cmd));
	save_item(NAME(m_kbd_enabled));
	save_item(NAME(m_repeat_key));
	save_item(NAME(m_repeat_count));
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

void quaderno_state::device_post_load()
{
	for (int r = 0x30; r <= 0x33; r++)
		update_ems(r);
}

void quaderno_state::machine_reset()
{
	std::fill(std::begin(m_r26), std::end(m_r26), 0);
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	std::fill(std::begin(m_pm), std::end(m_pm), 0);
	m_shadow_view.select(0);
	std::fill(std::begin(m_ems), std::end(m_ems), 0);
	for (int i = 0; i < 4; i++)
		m_ems_view[i].select(0);
	update_windows();
	update_cpu_clock();

	m_latch60 = 0;
	m_p66 = 0;
	m_p69 = 0;
	m_auxmask = 0;
	m_chan[0] = m_chan[1] = channel();
	m_mcu_out.clear();
	m_mcu_need = 0;
	m_kbd_out.clear();
	m_kbd_cmd = 0;
	m_kbd_enabled = true;
	m_repeat_key = -1;
	std::fill(std::begin(m_keystate), std::end(m_keystate), 0);
	std::fill(std::begin(m_keysent), std::end(m_keysent), 0);
	m_kbd_timer->adjust(attotime::from_hz(100), 0, attotime::from_hz(100));
	update_irq1();

	m_dsp_flags = 0;
	m_dsp_irq = false;
	m_codec_timer->adjust(attotime::from_hz(8000), 0, attotime::from_hz(8000));
	m_rtc_timer->adjust(attotime::from_seconds(1), 0, attotime::from_seconds(1));
	status_lcd_update();
}


/*
    Quaderno keyboard (German layout, see photos):

    ESC F1 .. F10 DRUCK ROLLEN PAUSE                 keypad: 7 8 9 /
    1 2 3 4 5 6 7 8 9 0 ß ´ <-                               4 5 6 *
    TAB Q W E R T Z U I O P Ü + #                            1 2 3 -
    CAPS A S D F G H J K L Ö Ä ENTER                         NUM 0 , +
    SHIFT < Y X C V B N M , . - UP SHIFT
    STRG MF ALT ^ SPACE BILD-AUF BILD-AB LEFT DOWN RIGHT

    Host mapping follows the key positions of a PC keyboard.  MF is on the
    right CTRL key and the context menu key (the Windows keys are taken by
    the host); right STRG is MF+STRG as on the real machine.  F11/F12,
    ALT GR, Home/End/Insert/Delete and keypad Enter are host conveniences
    for keys the Quaderno only has on the MF layer or keypad.
*/

#define QKEY(id_, name_, ...) \
	PORT_BIT( 1U << ((id_) & 15), IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME(name_) __VA_ARGS__

static INPUT_PORTS_START( quaderno_keyboard )
	PORT_START("KEY0")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_UNUSED )
	QKEY(0x01, "ESC", PORT_CODE(KEYCODE_ESC) PORT_CHAR(UCHAR_MAMEKEY(ESC)))
	QKEY(0x02, "1  !", PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!'))
	QKEY(0x03, "2  \"", PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('"'))
	QKEY(0x04, "3  \xc2\xa7", PORT_CODE(KEYCODE_3) PORT_CHAR('3'))
	QKEY(0x05, "4  $", PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$'))
	QKEY(0x06, "5  %", PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%'))
	QKEY(0x07, "6  &", PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&'))
	QKEY(0x08, "7  /  {", PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('/'))
	QKEY(0x09, "8  (  [", PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('('))
	QKEY(0x0a, "9  )  ]", PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR(')'))
	QKEY(0x0b, "0  =  }", PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR('='))
	QKEY(0x0c, "\xc3\x9f  ?  \\", PORT_CODE(KEYCODE_MINUS) PORT_CHAR(0xdf) PORT_CHAR('?'))
	QKEY(0x0d, "\xc2\xb4  `", PORT_CODE(KEYCODE_EQUALS))
	QKEY(0x0e, "Backspace", PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8))
	QKEY(0x0f, "Tab", PORT_CODE(KEYCODE_TAB) PORT_CHAR(9))

	PORT_START("KEY1")
	QKEY(0x10, "Q  @", PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q'))
	QKEY(0x11, "W", PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W'))
	QKEY(0x12, "E", PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E'))
	QKEY(0x13, "R", PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R'))
	QKEY(0x14, "T", PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T'))
	QKEY(0x15, "Z", PORT_CODE(KEYCODE_Y) PORT_CHAR('z') PORT_CHAR('Z'))
	QKEY(0x16, "U", PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U'))
	QKEY(0x17, "I", PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I'))
	QKEY(0x18, "O", PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O'))
	QKEY(0x19, "P", PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P'))
	QKEY(0x1a, "\xc3\x9c", PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR(0xfc) PORT_CHAR(0xdc))
	QKEY(0x1b, "+  *  ~", PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('+') PORT_CHAR('*'))
	QKEY(0x1c, "Enter", PORT_CODE(KEYCODE_ENTER) PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(13))
	QKEY(0x1d, "STRG", PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_MAMEKEY(LCONTROL)))
	QKEY(0x1e, "A", PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A'))
	QKEY(0x1f, "S", PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S'))

	PORT_START("KEY2")
	QKEY(0x20, "D", PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D'))
	QKEY(0x21, "F", PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F'))
	QKEY(0x22, "G", PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G'))
	QKEY(0x23, "H", PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H'))
	QKEY(0x24, "J", PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J'))
	QKEY(0x25, "K", PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K'))
	QKEY(0x26, "L", PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L'))
	QKEY(0x27, "\xc3\x96", PORT_CODE(KEYCODE_COLON) PORT_CHAR(0xf6) PORT_CHAR(0xd6))
	QKEY(0x28, "\xc3\x84", PORT_CODE(KEYCODE_QUOTE) PORT_CHAR(0xe4) PORT_CHAR(0xc4))
	QKEY(0x29, "^  \xc2\xb0", PORT_CODE(KEYCODE_TILDE) PORT_CHAR('^'))
	QKEY(0x2a, "Shift (left)", PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1))
	QKEY(0x2b, "#  '", PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('#') PORT_CHAR('\''))
	QKEY(0x2c, "Y", PORT_CODE(KEYCODE_Z) PORT_CHAR('y') PORT_CHAR('Y'))
	QKEY(0x2d, "X", PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X'))
	QKEY(0x2e, "C", PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C'))
	QKEY(0x2f, "V", PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V'))

	PORT_START("KEY3")
	QKEY(0x30, "B", PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B'))
	QKEY(0x31, "N", PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N'))
	QKEY(0x32, "M  \xc2\xb5", PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M'))
	QKEY(0x33, ",  ;", PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR(';'))
	QKEY(0x34, ".  :", PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR(':'))
	QKEY(0x35, "-  _", PORT_CODE(KEYCODE_SLASH) PORT_CHAR('-') PORT_CHAR('_'))
	QKEY(0x36, "Shift (right)", PORT_CODE(KEYCODE_RSHIFT))
	QKEY(0x37, "Keypad *", PORT_CODE(KEYCODE_ASTERISK) PORT_CHAR(UCHAR_MAMEKEY(ASTERISK)))
	QKEY(0x38, "ALT", PORT_CODE(KEYCODE_LALT) PORT_CHAR(UCHAR_MAMEKEY(LALT)))
	QKEY(0x39, "Space", PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' '))
	QKEY(0x3a, "Caps Lock", PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK)))
	QKEY(0x3b, "F1  HILFE  (MF: F11)", PORT_CODE(KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1)))
	QKEY(0x3c, "F2  (MF: F12)", PORT_CODE(KEYCODE_F2) PORT_CHAR(UCHAR_MAMEKEY(F2)))
	QKEY(0x3d, "F3  (MF: NOTIZEN)", PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3)))
	QKEY(0x3e, "F4  (MF: RECHNER)", PORT_CODE(KEYCODE_F4) PORT_CHAR(UCHAR_MAMEKEY(F4)))
	QKEY(0x3f, "F5  (MF: KALEND.)", PORT_CODE(KEYCODE_F5) PORT_CHAR(UCHAR_MAMEKEY(F5)))

	PORT_START("KEY4")
	QKEY(0x40, "F6  (MF: TELEFON)", PORT_CODE(KEYCODE_F6) PORT_CHAR(UCHAR_MAMEKEY(F6)))
	QKEY(0x41, "F7  (MF: DATEI)", PORT_CODE(KEYCODE_F7) PORT_CHAR(UCHAR_MAMEKEY(F7)))
	QKEY(0x42, "F8  (MF: ARCHIV)", PORT_CODE(KEYCODE_F8) PORT_CHAR(UCHAR_MAMEKEY(F8)))
	QKEY(0x43, "F9  (MF: cassette)", PORT_CODE(KEYCODE_F9) PORT_CHAR(UCHAR_MAMEKEY(F9)))
	QKEY(0x44, "F10  BEFEHLE  (MF: MENU)", PORT_CODE(KEYCODE_F10) PORT_CHAR(UCHAR_MAMEKEY(F10)))
	QKEY(0x45, "NUM", PORT_CODE(KEYCODE_NUMLOCK) PORT_CHAR(UCHAR_MAMEKEY(NUMLOCK)))
	QKEY(0x46, "ROLLEN", PORT_CODE(KEYCODE_SCRLOCK) PORT_CHAR(UCHAR_MAMEKEY(SCRLOCK)))
	QKEY(0x47, "Keypad 7  POS1", PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD)))
	QKEY(0x48, "Keypad 8  Up", PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD)))
	QKEY(0x49, "Keypad 9  BILD auf", PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD)))
	QKEY(0x4a, "Keypad -", PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)))
	QKEY(0x4b, "Keypad 4  Left", PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD)))
	QKEY(0x4c, "Keypad 5", PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD)))
	QKEY(0x4d, "Keypad 6  Right", PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD)))
	QKEY(0x4e, "Keypad +", PORT_CODE(KEYCODE_PLUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD)))
	QKEY(0x4f, "Keypad 1  ENDE", PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD)))

	PORT_START("KEY5")
	QKEY(0x50, "Keypad 2  Down", PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD)))
	QKEY(0x51, "Keypad 3  BILD ab", PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD)))
	QKEY(0x52, "Keypad 0  EINFG", PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD)))
	QKEY(0x53, "Keypad ,  ENTF", PORT_CODE(KEYCODE_DEL_PAD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD)))
	PORT_BIT( 0x0030, IP_ACTIVE_HIGH, IPT_UNUSED )
	QKEY(0x56, "<  >  |", PORT_CODE(KEYCODE_BACKSLASH2) PORT_CHAR('<') PORT_CHAR('>'))
	QKEY(0x57, "F11 (host, = MF+F1)", PORT_CODE(KEYCODE_F11))
	QKEY(0x58, "F12 (host, = MF+F2)", PORT_CODE(KEYCODE_F12))
	PORT_BIT( 0xfe00, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("KEY6")
	QKEY(0x60, "BILD auf", PORT_CODE(KEYCODE_PGUP) PORT_CHAR(UCHAR_MAMEKEY(PGUP)))
	QKEY(0x61, "BILD ab", PORT_CODE(KEYCODE_PGDN) PORT_CHAR(UCHAR_MAMEKEY(PGDN)))
	QKEY(0x62, "Cursor Left", PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT)))
	QKEY(0x63, "Cursor Right", PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)))
	QKEY(0x64, "Cursor Up", PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP)))
	QKEY(0x65, "Cursor Down", PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN)))
	QKEY(0x66, "Keypad /", PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)))
	QKEY(0x67, "DRUCK  S-ABF  (MF: TURBO)", PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR)))
	QKEY(0x68, "PAUSE  UNTBR", PORT_CODE(KEYCODE_PAUSE) PORT_CHAR(UCHAR_MAMEKEY(PAUSE)))
	QKEY(0x69, "MF", PORT_CODE(KEYCODE_RCONTROL) PORT_CODE(KEYCODE_MENU))
	QKEY(0x6a, "ALT GR (host, = MF+ALT)", PORT_CODE(KEYCODE_RALT) PORT_CHAR(UCHAR_MAMEKEY(RALT)))
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_UNUSED )   // 6b: right STRG = MF+STRG
	QKEY(0x6c, "POS1 (host)", PORT_CODE(KEYCODE_HOME) PORT_CHAR(UCHAR_MAMEKEY(HOME)))
	QKEY(0x6d, "ENDE (host)", PORT_CODE(KEYCODE_END) PORT_CHAR(UCHAR_MAMEKEY(END)))
	QKEY(0x6e, "EINFG (host)", PORT_CODE(KEYCODE_INSERT) PORT_CHAR(UCHAR_MAMEKEY(INSERT)))
	QKEY(0x6f, "ENTF (host)", PORT_CODE(KEYCODE_DEL) PORT_CHAR(UCHAR_MAMEKEY(DEL)))
INPUT_PORTS_END

static INPUT_PORTS_START( quaderno )
	PORT_INCLUDE( quaderno_keyboard )

	PORT_START("BUTTONS")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("Werkseinstellung (factory reset)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(quaderno_state::factory_reset), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Power source" )
	PORT_CONFSETTING(    0x00, "Battery" )
	PORT_CONFSETTING(    0x01, "AC adapter" )
	PORT_CONFNAME( 0x02, 0x00, "SRAM" )
	PORT_CONFSETTING(    0x00, "32 KB" )
	PORT_CONFSETTING(    0x02, "128 KB" )
INPUT_PORTS_END

static void quaderno_internal_isa(device_slot_interface &device)
{
	device.option_add_internal("qfdc", QUADERNO_FDC);
}

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
	ISA8_SLOT(config, "fdc", 0, "mb:isa", quaderno_internal_isa, "qfdc", true);
	ISA8_SLOT(config, "com", 0, "mb:isa", pc_isa8_cards, "com", true);
	ISA8_SLOT(config, "hdc", 0, "mb:isa", pc_isa8_cards, "hdc", true)
			.set_option_device_input_defaults("hdc", DEVICE_INPUT_DEFAULTS_NAME(quaderno_hdc));

	// parallel/floppy connector: printer cable or EFD 1035 floppy drive
	QUADERNO_EXT(config, m_ext, quaderno_ext_devices, nullptr);

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
	ROM_REGION16_LE( 0x20000, "bios", 0 )
	ROM_LOAD( "abg_r188.u4", 0x00000, 0x20000, CRC(f64985a6) SHA1(ad1d397bf3e04c42a54b3216d2544923364fdc2c) )

	// ROM disk, XBIOS DSP code and code page fonts
	ROM_REGION16_LE( 0x80000, "sdisk", 0 )
	ROM_LOAD( "fw-aag_r105.bin", 0x00000, 0x80000, CRC(6bc196e4) SHA1(da5bc11864ec2d78578d0c8a4d6fab233fb6f964) )

	// Mitsubishi M37410M4 internal mask ROM, emulated at high level
	ROM_REGION( 0x4000, "mcu", ROMREGION_ERASEFF )
	ROM_LOAD( "m37410m4_441028-122.bin", 0x0000, 0x4000, NO_DUMP )
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     CLASS           INIT        COMPANY     FULLNAME    FLAGS
COMP( 1992, quaderno, 0,      0,      quaderno, quaderno, quaderno_state, empty_init, "Olivetti", "Quaderno", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND )
