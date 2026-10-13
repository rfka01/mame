// license:BSD-3-Clause
// copyright-holders:rfka01
/****************************************************************************

    Olivetti Quaderno 33 (PT-AT-60, 1993) - 386SX notebook

    AMD Am386SXLV @ 20 MHz
    Western Digital WD7600LP chipset (WD76C10LP system controller,
        WD76C20 floppy/RTC/bus interface, WD76C30 serial/parallel)
    8042 keyboard controller (Olivetti firmware, not dumped)
    Chips & Technologies 655xx flat panel VGA, monochrome LCD 640x480
    4 MB DRAM (two banks of 2 MB)
    2.5" IDE hard disk, external floppy drive, PCMCIA slot (3e0h)
    256 KB flash, mapped at c0000-fffff (and fc0000-ffffff):
        c0000  "Quaderno 33 Monochrome Video BIOS" 1.09B (C&T)
        cb800  APM BIOS
        d8000  setup "BIS 103" with texts in all languages
        e4000  ROM-DOS option ROM (55 aa 60): loader, ROMBIO.SYS and
               ROMDOS.SYS (localized MS-DOS 5.0 kernel); hooks INT 19h
        f0000  system BIOS 1.07 (09/14/93), "Resident Diagnostics"

    The flash images BIOS107.* come from an Olivetti BIOS update
    diskette.  Only ITA and FRE survived intact; GER, UK and USA were
    rebuilt from the damaged files (blocks moved by 2400h) and verified
    against the BIOS checksums (8-bit sums of the D, E and F segments,
    16-bit sum of c0000-effff including the language tag at cfffd-cffff).
    SPA could not be rebuilt.

    POST codes go to the parallel port data register (378h).

    TODO:
    - WD76C10 EMS (6872h, e072h/e872h), bank and DRAM configuration
      registers are only stored
    - power management (7072h-8872h, c872h, Olivetti registers at
      37bh/37fh and 178h/179h)
    - Olivetti keyboard controller commands 89h, 8fh, a3h, b8h, bch, bdh
    - real C&T chip type (65510/65520/65530?), LCD panel modes
    - PCMCIA controller

****************************************************************************/

#include "emu.h"

#include "bus/isa/isa.h"
#include "bus/isa/isa_cards.h"
#include "bus/pc_kbd/keyboards.h"
#include "bus/pc_kbd/pc_kbdc.h"
#include "cpu/i386/i386.h"
#include "machine/at_keybc.h"
#include "machine/mc146818.h"
#include "machine/ram.h"
#include "machine/wd7600.h"
#include "sound/spkrdev.h"
#include "video/pc_vga_chips.h"

#include "screen.h"
#include "speaker.h"

#define LOG_REGS    (1U << 1)
#define LOG_SHADOW  (1U << 2)
#define LOG_POST    (1U << 3)

#define VERBOSE (0)
#include "logmacro.h"

#define LOGREGS(...)    LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGSHADOW(...)  LOGMASKED(LOG_SHADOW, __VA_ARGS__)


namespace {

/***************************************************************************
    On board video: C&T flat panel VGA
***************************************************************************/

class quad33_vga_device : public device_t, public device_isa16_card_interface
{
public:
	quad33_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	virtual void device_start() override ATTR_COLD { set_isa_device(); }
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void remap(int space_id, offs_t start, offs_t end) override;

private:
	void io_isa_map(address_map &map) ATTR_COLD;

	required_device<f65535_vga_device> m_vga;
};

} // anonymous namespace

DEFINE_DEVICE_TYPE(QUAD33_VGA, quad33_vga_device, "quad33_vga", "Olivetti Quaderno 33 on board video (C&T flat panel VGA)")

namespace {

quad33_vga_device::quad33_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUAD33_VGA, tag, owner, clock)
	, device_isa16_card_interface(mconfig, *this)
	, m_vga(*this, "vga")
{
}

void quad33_vga_device::device_add_mconfig(machine_config &config)
{
	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_raw(25.175_MHz_XTAL, 800, 0, 640, 524, 0, 480);
	screen.set_screen_update(m_vga, FUNC(f65535_vga_device::screen_update));

	F65535_VGA(config, m_vga);
	m_vga->set_screen("screen");
	m_vga->set_vram_size(256 * 1024);
}

void quad33_vga_device::io_isa_map(address_map &map)
{
	map(0x03b0, 0x03df).m(m_vga, FUNC(f65535_vga_device::io_map));
}

void quad33_vga_device::device_reset()
{
	remap(AS_PROGRAM, 0, 0xfffff);
	remap(AS_IO, 0, 0xffff);
}

void quad33_vga_device::remap(int space_id, offs_t start, offs_t end)
{
	if (space_id == AS_PROGRAM)
		m_isa->install_memory(0xa0000, 0xbffff, read8sm_delegate(*m_vga, FUNC(f65535_vga_device::mem_r)), write8sm_delegate(*m_vga, FUNC(f65535_vga_device::mem_w)));
	else if (space_id == AS_IO)
		m_isa->install_device(0x0000, 0xffff, *this, &quad33_vga_device::io_isa_map);
}

void quad33_onboard(device_slot_interface &device)
{
	device.option_add("vga", QUAD33_VGA);
}


/***************************************************************************
    Machine
***************************************************************************/

class quaderno33_state : public driver_device
{
public:
	quaderno33_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_ram(*this, RAM_TAG)
		, m_wd7600(*this, "wd7600")
		, m_isabus(*this, "isabus")
		, m_speaker(*this, "speaker")
		, m_bios(*this, "bios")
	{ }

	void quaderno33(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<i386sx_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_device<wd7600_device> m_wd7600;
	required_device<isa16_device> m_isabus;
	required_device<speaker_sound_device> m_speaker;
	required_region_ptr<u8> m_bios;

	void main_map(address_map &map) ATTR_COLD;
	void main_io(address_map &map) ATTR_COLD;

	u16 wd7600_ior(offs_t offset);
	void wd7600_iow(offs_t offset, u16 data);
	void wd7600_hold(int state);
	void wd7600_tc(offs_t offset, u8 data) { m_isabus->eop_w(offset, data); }
	void wd7600_spkr(int state) { m_speaker->level_w(state); }

	// WD76C10 registers not handled by the wd7600 device
	u16 reg_r(offs_t port) { return m_regs[(port >> 10) & 0x3f]; }
	void reg_w(offs_t port, u16 data, u16 mem_mask);
	u16 lock_status_r();
	u16 shadow_r() { return m_shadow_ctl; }
	void shadow_w(offs_t offset, u16 data, u16 mem_mask);
	u16 split_r() { return m_split; }
	void split_w(offs_t offset, u16 data, u16 mem_mask);
	void update_shadow();
	void update_split();
	void postload();

	// Olivetti registers
	u8 ga_r(offs_t offset) { return offset ? m_ga[m_ga_index] : m_ga_index; }
	void ga_w(offs_t offset, u8 data);
	u8 pmi_r() { return m_pm_index; }
	void pmi_w(u8 data) { m_pm_index = data; }
	u8 pmd_r() { return m_pm[m_pm_index]; }
	void pmd_w(u8 data);
	u8 pcic_r(offs_t offset);
	void pcic_w(offs_t offset, u8 data);

	std::array<u16, 64> m_regs;
	u16 m_shadow_ctl;
	u16 m_split;
	bool m_lock_toggle;
	offs_t m_split_base;
	offs_t m_split_size;

	u8 m_ga_index;
	u8 m_ga[256];
	u8 m_pm_index;
	u8 m_pm[256];
	u8 m_pcic_index;
	u8 m_pcic[256];
};


/***************************************************************************
    Chipset glue
***************************************************************************/

u16 quaderno33_state::wd7600_ior(offs_t offset)
{
	if (offset < 4)
		return m_isabus->dack_r(offset);
	else
		return m_isabus->dack16_r(offset);
}

void quaderno33_state::wd7600_iow(offs_t offset, u16 data)
{
	if (offset < 4)
		m_isabus->dack_w(offset, data);
	else
		m_isabus->dack16_w(offset, data);
}

void quaderno33_state::wd7600_hold(int state)
{
	m_maincpu->set_input_line(INPUT_LINE_HALT, state ? ASSERT_LINE : CLEAR_LINE);
	m_wd7600->hlda_w(state);
}

void quaderno33_state::reg_w(offs_t port, u16 data, u16 mem_mask)
{
	u16 &reg = m_regs[(port >> 10) & 0x3f];
	const u16 old = reg;
	COMBINE_DATA(&reg);
	if (reg != old)
		LOGREGS("%s: WD76C10 %04X = %04X\n", machine().describe_context(), port, reg);
}

// fc72: lock status, bit 15 toggles on every read
u16 quaderno33_state::lock_status_r()
{
	if (!machine().side_effects_disabled())
		m_lock_toggle = !m_lock_toggle;
	return (m_lock_toggle ? 0x8000 : 0) | (m_regs[0x3f] & 0x7fff);
}

/*
    6072: RAM shadow and write protect
        bit 15-14  disable on board memory above 128K/256K/512K
        bit 13     high memory write protect
        bit 12     write protect shadow RAM
        bit 9-8    shadow: 0 = none, 1 = f0000-fffff,
                   2 = e0000-fffff, 3 = video BIOS + f0000-fffff
        bit 7      also shadow e8000-effff (mode 3)
        bit 5-4    video BIOS size (16K units - 1, from c0000)
        bit 3-2    ROM chip select type
    The shadow RAM is the DRAM hidden behind c0000-fffff.
*/
void quaderno33_state::shadow_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_shadow_ctl);
	LOGSHADOW("%s: shadow control %04X\n", machine().describe_context(), m_shadow_ctl);
	update_shadow();
}

void quaderno33_state::update_shadow()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u8 *const ram = m_ram->pointer();

	const bool wp = BIT(m_shadow_ctl, 12);
	const unsigned shd = (m_shadow_ctl >> 8) & 3;
	const bool xmem = BIT(m_shadow_ctl, 7);
	const unsigned vbpages = ((m_shadow_ctl >> 4) & 3) + 1;

	// 16 pages of 16K: c0000-fffff
	bool shadowed[16] = { };
	if (shd == 3)
	{
		for (unsigned i = 0; i < vbpages; i++)
			shadowed[i] = true;
		if (xmem)
			shadowed[10] = shadowed[11] = true;
	}
	if (shd == 1 || shd == 3)
		for (unsigned i = 12; i < 16; i++)
			shadowed[i] = true;
	if (shd == 2)
		for (unsigned i = 8; i < 16; i++)
			shadowed[i] = true;

	for (unsigned i = 0; i < 16; i++)
	{
		const offs_t start = 0xc0000 + i * 0x4000;
		const offs_t end = start + 0x3fff;
		if (shadowed[i])
		{
			space.install_rom(start, end, ram + start);
			if (wp)
				space.nop_write(start, end);
			else
				space.install_writeonly(start, end, ram + start);
		}
		else
		{
			space.install_rom(start, end, &m_bios[i * 0x4000]);
			space.nop_write(start, end);
		}
	}
}

/*
    5872: split start address (wd7600 device keeps a copy)
        bit 15-12  bank enables
        bit 9-8    split size: 0 = off, 1 = 256K, 2 = 320K, 3 = 384K
        bit 7-2    split start address (512K units)
    The memory hidden behind a0000-fffff that is not used for shadowing
    appears at the split address: a0000-bffff and d0000-effff for 256K.
*/
void quaderno33_state::split_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_split);
	m_wd7600->split_addr_w(m_split);
	update_split();
}

void quaderno33_state::update_split()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u8 *const ram = m_ram->pointer();

	if (m_split_size)
		space.unmap_readwrite(m_split_base, m_split_base + m_split_size - 1);

	const unsigned size = (m_split >> 8) & 3;
	m_split_base = ((m_split >> 2) & 0x3f) << 19;
	m_split_size = size ? (0x40000 + (size - 1) * 0x10000) : 0;
	if (!m_split_size)
		return;

	LOGSHADOW("split: %06X-%06X\n", m_split_base, m_split_base + m_split_size - 1);
	space.install_ram(m_split_base, m_split_base + 0x1ffff, ram + 0xa0000);
	space.install_ram(m_split_base + 0x20000, m_split_base + 0x3ffff, ram + 0xd0000);
	if (size >= 2)
		space.install_ram(m_split_base + 0x40000, m_split_base + 0x4ffff, ram + 0xc0000);
	if (size >= 3)
		space.install_ram(m_split_base + 0x50000, m_split_base + 0x5ffff, ram + 0xf0000);
}

void quaderno33_state::ga_w(offs_t offset, u8 data)
{
	if (!offset)
		m_ga_index = data;
	else
	{
		LOGREGS("%s: GA %02X = %02X\n", machine().describe_context(), m_ga_index, data);
		m_ga[m_ga_index] = data;
	}
}

void quaderno33_state::pmd_w(u8 data)
{
	LOGREGS("%s: PM %02X = %02X\n", machine().describe_context(), m_pm_index, data);
	m_pm[m_pm_index] = data;
}

// minimal 82365-style register file, enough for the BIOS to see an empty slot
u8 quaderno33_state::pcic_r(offs_t offset)
{
	if (!offset)
		return m_pcic_index;
	if ((m_pcic_index & 0x3f) == 0x00)
		return 0x83;
	return m_pcic[m_pcic_index];
}

void quaderno33_state::pcic_w(offs_t offset, u8 data)
{
	if (!offset)
		m_pcic_index = data;
	else
		m_pcic[m_pcic_index] = data;
}


/***************************************************************************
    Address maps
***************************************************************************/

void quaderno33_state::main_map(address_map &map)
{
	map.unmap_value_high();
}

void quaderno33_state::main_io(address_map &map)
{
	map.unmap_value_high();
	map(0x0178, 0x0179).rw(FUNC(quaderno33_state::ga_r), FUNC(quaderno33_state::ga_w));
	map(0x037f, 0x037f).rw(FUNC(quaderno33_state::pmd_r), FUNC(quaderno33_state::pmd_w));
	map(0x03e0, 0x03e1).rw(FUNC(quaderno33_state::pcic_r), FUNC(quaderno33_state::pcic_w));
	map(0x6072, 0x6073).rw(FUNC(quaderno33_state::shadow_r), FUNC(quaderno33_state::shadow_w));
	map(0xfc72, 0xfc73).r(FUNC(quaderno33_state::lock_status_r));

	// remaining WD76C10 registers: stored only
	static constexpr u16 ports[] = {
			0x1072, 0x1872, 0x3072, 0x4072, 0x6872, 0x7072, 0x7872, 0x8072,
			0x8872, 0x9072, 0xa072, 0xa872, 0xb072, 0xb872, 0xc072, 0xc872,
			0xd872, 0xdc72, 0xe072, 0xe472, 0xe872, 0xf072, 0xf472, 0xf872 };
	for (u16 port : ports)
		map(port, port + 1).lrw16(
				NAME(([this, port] (offs_t offset, u16 mem_mask) { return reg_r(port); })),
				NAME(([this, port] (offs_t offset, u16 data, u16 mem_mask) { reg_w(port, data, mem_mask); })));
}


/***************************************************************************
    Machine start/reset
***************************************************************************/

void quaderno33_state::machine_start()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	address_space &io = m_maincpu->space(AS_IO);

	// the wd7600 device maps a 128K BIOS at e0000 and ram at d0000-effff;
	// the Quaderno 33 has a 256K flash at c0000-fffff
	space.install_rom(0xfc0000, 0xffffff, &m_bios[0]);
	io.install_readwrite_handler(0x5872, 0x5873,
			read16smo_delegate(*this, FUNC(quaderno33_state::split_r)),
			write16s_delegate(*this, FUNC(quaderno33_state::split_w)));
	// Olivetti power management index shares 37bh with the parallel port
	io.install_readwrite_handler(0x037a, 0x037b,
			read8smo_delegate(*this, FUNC(quaderno33_state::pmi_r)),
			write8smo_delegate(*this, FUNC(quaderno33_state::pmi_w)), 0xff00);

	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	std::fill(std::begin(m_pm), std::end(m_pm), 0);
	std::fill(std::begin(m_pcic), std::end(m_pcic), 0);
	m_ga_index = m_pm_index = m_pcic_index = 0;
	m_lock_toggle = false;
	m_split_base = m_split_size = 0;

	save_item(NAME(m_regs));
	save_item(NAME(m_shadow_ctl));
	save_item(NAME(m_split));
	save_item(NAME(m_lock_toggle));
	save_item(NAME(m_ga_index));
	save_item(NAME(m_ga));
	save_item(NAME(m_pm_index));
	save_item(NAME(m_pm));
	save_item(NAME(m_pcic_index));
	save_item(NAME(m_pcic));
	machine().save().register_postload(save_prepost_delegate(FUNC(quaderno33_state::postload), this));
}

void quaderno33_state::postload()
{
	update_shadow();
	update_split();
}

void quaderno33_state::machine_reset()
{
	m_shadow_ctl = 0;
	update_shadow();
	m_split = 0;
	update_split();
}


/***************************************************************************
    Machine configuration
***************************************************************************/

void quaderno33_state::quaderno33(machine_config &config)
{
	I386SX(config, m_maincpu, 40_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &quaderno33_state::main_map);
	m_maincpu->set_addrmap(AS_IO, &quaderno33_state::main_io);
	m_maincpu->set_irq_acknowledge_callback("wd7600", FUNC(wd7600_device::intack_cb));

	WD7600(config, m_wd7600, 40_MHz_XTAL / 2);
	m_wd7600->set_cputag(m_maincpu);
	// the BIOS keeps bit 1 of RTC register B as it finds it; the setup
	// writes the time in 24 hour format
	m_wd7600->subdevice<mc146818_device>("rtc")->set_24hrs(true);
	m_wd7600->set_ramtag(m_ram);
	m_wd7600->set_biostag("bios");
	m_wd7600->set_keybctag("keybc");
	m_wd7600->hold_callback().set(FUNC(quaderno33_state::wd7600_hold));
	m_wd7600->nmi_callback().set_inputline(m_maincpu, INPUT_LINE_NMI);
	m_wd7600->intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_wd7600->cpureset_callback().set_inputline(m_maincpu, INPUT_LINE_RESET);
	m_wd7600->a20m_callback().set_inputline(m_maincpu, INPUT_LINE_A20);
	m_wd7600->ior_callback().set(FUNC(quaderno33_state::wd7600_ior));
	m_wd7600->iow_callback().set(FUNC(quaderno33_state::wd7600_iow));
	m_wd7600->tc_callback().set(FUNC(quaderno33_state::wd7600_tc));
	m_wd7600->spkr_callback().set(FUNC(quaderno33_state::wd7600_spkr));

	ISA16(config, m_isabus);
	m_isabus->set_memspace(m_maincpu, AS_PROGRAM);
	m_isabus->set_iospace(m_maincpu, AS_IO);
	m_isabus->iochck_callback().set(m_wd7600, FUNC(wd7600_device::iochck_w));
	m_isabus->irq2_callback().set(m_wd7600, FUNC(wd7600_device::irq09_w));
	m_isabus->irq3_callback().set(m_wd7600, FUNC(wd7600_device::irq03_w));
	m_isabus->irq4_callback().set(m_wd7600, FUNC(wd7600_device::irq04_w));
	m_isabus->irq5_callback().set(m_wd7600, FUNC(wd7600_device::irq05_w));
	m_isabus->irq6_callback().set(m_wd7600, FUNC(wd7600_device::irq06_w));
	m_isabus->irq7_callback().set(m_wd7600, FUNC(wd7600_device::irq07_w));
	m_isabus->irq10_callback().set(m_wd7600, FUNC(wd7600_device::irq10_w));
	m_isabus->irq11_callback().set(m_wd7600, FUNC(wd7600_device::irq11_w));
	m_isabus->irq12_callback().set(m_wd7600, FUNC(wd7600_device::irq12_w));
	m_isabus->irq14_callback().set(m_wd7600, FUNC(wd7600_device::irq14_w));
	m_isabus->irq15_callback().set(m_wd7600, FUNC(wd7600_device::irq15_w));
	m_isabus->drq0_callback().set(m_wd7600, FUNC(wd7600_device::dreq0_w));
	m_isabus->drq1_callback().set(m_wd7600, FUNC(wd7600_device::dreq1_w));
	m_isabus->drq2_callback().set(m_wd7600, FUNC(wd7600_device::dreq2_w));
	m_isabus->drq3_callback().set(m_wd7600, FUNC(wd7600_device::dreq3_w));
	m_isabus->drq5_callback().set(m_wd7600, FUNC(wd7600_device::dreq5_w));
	m_isabus->drq6_callback().set(m_wd7600, FUNC(wd7600_device::dreq6_w));
	m_isabus->drq7_callback().set(m_wd7600, FUNC(wd7600_device::dreq7_w));

	// on board devices
	ISA16_SLOT(config, "board1", 0, m_isabus, quad33_onboard, "vga", true);
	ISA16_SLOT(config, "board2", 0, m_isabus, pc_isa16_cards, "fdc_smc", true);   // WD76C20
	ISA16_SLOT(config, "board3", 0, m_isabus, pc_isa16_cards, "comat", true);     // WD76C30
	ISA16_SLOT(config, "board4", 0, m_isabus, pc_isa16_cards, "lpt", true);       // WD76C30
	ISA16_SLOT(config, "board5", 0, m_isabus, pc_isa16_cards, "ide", true);

	ps2_keyboard_controller_device &keybc(PS2_KEYBOARD_CONTROLLER(config, "keybc", 12_MHz_XTAL));
	keybc.hot_res().set(m_wd7600, FUNC(wd7600_device::kbrst_w));
	keybc.gate_a20().set(m_wd7600, FUNC(wd7600_device::gatea20_w));
	keybc.kbd_irq().set(m_wd7600, FUNC(wd7600_device::irq01_w));
	keybc.kbd_clk().set("kbd", FUNC(pc_kbdc_device::clock_write_from_mb));
	keybc.kbd_data().set("kbd", FUNC(pc_kbdc_device::data_write_from_mb));

	pc_kbdc_device &kbd(PC_KBDC(config, "kbd", pc_at_keyboards, STR_KBD_MICROSOFT_NATURAL));
	kbd.out_clock_cb().set("keybc", FUNC(ps2_keyboard_controller_device::kbd_clk_w));
	kbd.out_data_cb().set("keybc", FUNC(ps2_keyboard_controller_device::kbd_data_w));

	RAM(config, m_ram).set_default_size("4M");

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.50);
}


/***************************************************************************
    ROM definitions
***************************************************************************/

ROM_START( quaderno33 )
	ROM_REGION(0x40000, "bios", 0)
	ROM_DEFAULT_BIOS("ger")
	ROM_SYSTEM_BIOS(0, "ger", "BIOS 1.07, German (rebuilt)")
	ROMX_LOAD("bios107.ger", 0x00000, 0x40000, BAD_DUMP CRC(bc0724a9) SHA1(1f7f0cab2647d762dfdd4a9cb61f1ee246ad1e12), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "ita", "BIOS 1.07, Italian")
	ROMX_LOAD("bios107.ita", 0x00000, 0x40000, CRC(fb79bba8) SHA1(dfc75d95d003a3eee36fc26b24ec10027b87a26d), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(2, "fre", "BIOS 1.07, French")
	ROMX_LOAD("bios107.fre", 0x00000, 0x40000, CRC(81a1310c) SHA1(f45a126576940ac36486e8a6cf13d10385090410), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(3, "uk", "BIOS 1.07, UK (rebuilt)")
	ROMX_LOAD("bios107.uk", 0x00000, 0x40000, BAD_DUMP CRC(328a759b) SHA1(7a4d3b978dc944b39aa18039bbc659acb779d451), ROM_BIOS(3))
	ROM_SYSTEM_BIOS(4, "usa", "BIOS 1.07, USA (rebuilt)")
	ROMX_LOAD("bios107.usa", 0x00000, 0x40000, BAD_DUMP CRC(ed88670e) SHA1(732cc5ffbcd9149a66645fda47e59571292ea46b), ROM_BIOS(4))
ROM_END

} // anonymous namespace


//    YEAR  NAME        PARENT  COMPAT  MACHINE     INPUT  CLASS             INIT        COMPANY     FULLNAME       FLAGS
COMP( 1993, quaderno33, 0,      0,      quaderno33, 0,     quaderno33_state, empty_init, "Olivetti", "Quaderno 33", MACHINE_NOT_WORKING )
