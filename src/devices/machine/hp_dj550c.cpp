// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 550C colour inkjet printer (C2121A), 1993

    Main board: Z80 (15.975 MHz crystal, the CPU runs at half of it),
    HP ASIC 1XK3-0301, 256 KiB DRAM, two 512 KiB mask ROMs (U12 program
    and fonts, U11 data), 93C46 EEPROM, two font cartridge slots, a
    Centronics port and an RS-232C port.

    The ASIC as far as the firmware shows (port numbers, Z80 I/O space):

      00     bit 2: the print pass runs leftwards
      02/03  compare value for the 16 bit counter (IRQ bit 5)
      04/05  carriage encoder, 600 counts per inch (a write loads it)
      06/07  start position of a print pass; writing 07 starts the pass
      08     interrupt mask (bit 0 enables the servo NMI), 09 pending
      0B     bit 3: the host DMA takes the serial port
      0C/0D  free running 16 bit counter
      0E/0F  pen sense: bit 7 is set when the fired nozzle line answers,
             reading 0F clears it; writing 0E/0F acknowledges the tick
      10     host interface: bits 1-0 baud rate (9600, 19200, 2400, 1200),
             bits 3-2 parity (0 none, 3 odd, 2 even), bit 4 serial ready
             for data (DTR), bit 5 on line (SELECT),
             bit 6 error (/FAULT low), bit 7 out of paper (PERROR)
      11     serial port: transmit data; read: receive status (bit 0
             parity error, bit 1 framing error, bit 7 overrun, assumed)
      12     serial port: receive data; the receiver holds a few characters
             (assumed: the firmware stops taking data when it sends XOFF,
             the characters a computer sends after it would be lost)
      14/15  carriage motor: 14 bit 7 direction, 15 PWM duty
      1C/1D  address of the display list of a print pass
      41     host input DMA: 8 KiB block of the buffer
      42     bank of display lists above 8000
      49     bit 0: servo NMI pending, reading clears it
      4A     bit 6 host DMA on, bit 5 parallel port
      4B     stepper motors: bits 3-0 paper feed, 7-4 service station,
             half steps 1 9 8 A 2 6 4 5
      50-52  memory banks for 0000, 8000 and C000 (4000 is fixed to bank 1);
             16 KiB banks, register value = bank number * 2
      55     panel: LED latch, and key matrix columns FD FB EF DF
      56     configuration switch matrix columns FE FD FB F7 EF DF BF 7F
      57     bit 0 EEPROM DI and Busy LED, bit 2 EEPROM CS, bit 3 EEPROM
             CLK, bit 5 selects the colour print cartridge
      59     bit 1 EEPROM DO, bit 6 paper sensor (1 = paper); the other
             bits read back port 57
      5A     bits 0-1 key matrix rows (high = pressed), bits 5-6 switch
             matrix rows (low = switch up)
      5C-5F  host DMA end and write pointer (13 bits)

    Physical memory (16 KiB banks): 00-1F U12, 20-2F DRAM, 40-5F font
    cartridges, 60-7F U11.

    Interrupts (IM 1): bit 1 system tick, bit 2 serial transmitter empty,
    bit 3 serial byte received while the host DMA is off, bit 4 reset from
    the host (/INIT, not masked; the firmware waits for its end and
    restarts), bit 5 compare, bit 6 parallel byte waiting while the host
    DMA is off, or the end of the DMA block.  The carriage servo runs on
    the NMI.  The firmware takes data from whichever port sends first.

    Print passes: the firmware builds a display list for each pass of the
    carriage; the ASIC fires the nozzles as the encoder passes the
    positions.  List entries:
      C0+n      the following addresses are in the n-th 64 KiB of memory
      20 + 8    raster block: units per dot (2 = 300 dpi, 1 for the pen
                identification), width in units (16 bit), (unknown),
                source (16 bit), (unknown), rows (50 black, 48 colour);
                the source holds columns of 'rows' bytes, 8 dots each;
                leftward passes point at the last byte and read backwards
      00+h + 6  character of a built-in font: step (h = high bits), column
                count, column masks, column data.  Two cursors take turns:
                each character starts 'step' units after the end of the
                character before last.  A mask byte tells which of seven
                bytes (56 nozzles, bit 7 = top) follow for the column.
      80+n      width of the following characters: 80 and 81 every other
                column (20 characters per inch; each character comes as two
                entries, one for each half of its columns), 82 only the
                columns with bit 7 of their mask byte set (16.67 cpi)
      A0+n      landscape characters: bit 0 every other row (20 cpi), bit 1
                bold (assumed: each column printed again two units on)
      E0+n      back to full width (n not known)
      40 + 6    not known
      60        end of the list
    Positions are encoder counts (1/600 inch); nozzle rows are 1/300
    inch.  The black cartridge prints 50 rows (48 per pass when
    printing raster graphics), the colour cartridge 3 x 16 nozzles:
    yellow, magenta and cyan from the top.

    Panel (matrix columns from left to right): Reset / Load/Eject Paper,
    Clean / Font, Alignment Test / Status, Install Print Cartridge /
    Quality.  The firmware acts when a key is pressed; keys held at power
    on select the self test and other functions.  LEDs: port 55 bits 0-2
    portrait, landscape, condensed font, bit 3 alignment test, bit 4
    Ready, bit 5 black and bit 6 colour print cartridge (lit for a missing
    cartridge), bit 7 Draft; port 57 bit 0 Busy (active low).

    Host interface: the firmware sends XOFF and drops DTR when its buffer
    has room for 100 characters, XON and DTR when there is room for 150
    (user's guide); it also sends XOFF at power on.  The serial port uses
    ten bits per character: eight data bits without parity, seven with.

    Paper path: with the carriage pressed against its right stop the
    paper motor picks a sheet from the IN tray.  The paper sensor sees a
    sheet from its leading edge to its trailing edge; the firmware feeds
    a new sheet until the sensor sees it (at most 2850 half steps), then
    to the top of the page.  At power on and to eject it feeds until the
    sensor no longer sees paper.  One half step of the paper motor moves
    the paper by 1/300 inch.

    Not known, and set to values that work with the firmware:
    - the rates of the system tick (38 Hz) and the servo NMI (250 Hz)
    - the carriage: a DC motor model with the encoder, the stops at 0 and
      7690 counts
    - the distance from the pick roller to the paper sensor, and from the
      sensor to the nozzles (from the top of page the firmware feeds to)
    - the left edge of the paper in carriage counts, and the offsets of
      leftward passes (fitted to the printout)
    - the timing of the Centronics handshake: BUSY from /STROBE until the
      byte is stored by the host DMA, then a 5 us ACK pulse
    - the bits of the serial receive status

    Cartridges: two slots (bus/hp_dj_cart), each a 256 KiB window at
    banks 40-4F and 50-5F, for font ROMs (mounted as cartridge1 and
    cartridge2) or the RAM cartridges 22707A and 22707B; the firmware
    finds the fonts and the RAM by itself.
    HP offered these cartridges for the printer (user's guide, accessories):
      22706B  Prestige Elite
      22706C  Letter Gothic
      22707L  Letter Gothic Landscape Fonts
      22706M  Presentations (ASCII characters only)
      22706R  Times Roman Collection
      22706T  Helvetica Collection
      22706U  Times Roman Headlines (15 and 30 pt)
      22706V  Helvetica Headlines (15 and 30 pt)
      22706W  WordPerfect Fonts
      C2109C  Garamond Collection
      C2109B  Global Text
      C2109D  Dom Casual
      C2109E  Brush
      22707B  256K RAM Cartridge: up to two, as input buffer (switch B2
              up) or for downloaded soft fonts such as 22708C Times
              Roman/Helvetica, which needs at least one
    Dumps seen so far:
    - 22707L (MB831000, 128 KiB): only the first 64 KiB is dumped
      (landscape Letter Gothic 12 point and italic); it works.  The
      MB831000 has A16 on pin 22, where a 27C512 has /OE - read as a
      27C512, the upper half is not reached
    - ODS Business Cartridge, B2543A Opt. ACB (ROM ID 3233 CYR, 256 KiB,
      Cyrillic CG Times and Courier with the symbol sets 3R, 10N and 9R):
      works with its 64 KiB blocks in the order 0, 3, 2, 1 of the dump -
      the firmware takes the glyphs of the italic Courier (directory in
      block 2) from the second block of the window and those of the
      italic CG Times from its own directory block; the software list
      loads it that way.  The board drives the ROM's upper address lines
      from a 74LS139 (Guru); the wiring is not traced

    TODO:
    - paper sizes other than Letter and A4 (Legal, Executive, envelopes)
    - meaning of the list entry 40, of n in E0+n and of some raster fields

**********************************************************************/

#include "emu.h"
#include "hp_dj550c.h"

#include "softlist_dev.h"

#include "hpdj550c.lh"

#define LOG_LIST  (1U << 1)
#define LOG_PAPER (1U << 2)
#define LOG_HOST  (1U << 3)

//#define VERBOSE (LOG_GENERAL | LOG_LIST | LOG_PAPER | LOG_HOST)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(HP_DJ550C, hp_dj550c_device, "hpdj550c", "HP DeskJet 550C")


namespace {

INPUT_PORTS_START( hpdj550c )
	PORT_START("KEYS")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Reset")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Load/Eject Paper")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Clean")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Font")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Alignment Test")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Status")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Install Print Cartridge")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Quality")

	// configuration switches under the IN tray, a bit is set for a switch up
	PORT_START("SWA")
	PORT_DIPNAME(0xf0, 0x00, "Character set") PORT_DIPLOCATION("A:1,2,3,4")
	PORT_DIPSETTING(   0x00, "PC-8")
	PORT_DIPSETTING(   0x10, "HP Roman8")
	PORT_DIPSETTING(   0x20, "PC-8 Danish/Norwegian")
	PORT_DIPSETTING(   0x30, "United Kingdom (ISO 4)")
	PORT_DIPSETTING(   0x40, "German (ISO 21)")
	PORT_DIPSETTING(   0x50, "French (ISO 69)")
	PORT_DIPSETTING(   0x60, "Italian (ISO 15)")
	PORT_DIPSETTING(   0x70, "Norwegian v1 (ISO 60)")
	PORT_DIPSETTING(   0x80, "Swedish Names (ISO 11)")
	PORT_DIPSETTING(   0x90, "Spanish (ISO 17)")
	PORT_DIPSETTING(   0xa0, "ASCII")
	PORT_DIPSETTING(   0xb0, "Portuguese (ISO 16)")
	PORT_DIPSETTING(   0xc0, "PC-850")
	PORT_DIPSETTING(   0xd0, "ECMA-94 Latin 1")
	PORT_DIPSETTING(   0xe0, "HP Legal")
	PORT_DIPSETTING(   0xf0, "Default of the front font cartridge")
	PORT_DIPNAME(0x0c, 0x00, "Paper size") PORT_DIPLOCATION("A:5,6")
	PORT_DIPSETTING(   0x00, "Letter")
	PORT_DIPSETTING(   0x04, "A4")
	PORT_DIPSETTING(   0x08, "Legal")
	PORT_DIPSETTING(   0x0c, "Envelope (No. 10, DL)")
	PORT_DIPNAME(0x02, 0x00, "Carriage return") PORT_DIPLOCATION("A:7")
	PORT_DIPSETTING(   0x00, "CR only")
	PORT_DIPSETTING(   0x02, "CR and LF")
	PORT_DIPNAME(0x01, 0x00, "Perforation skip") PORT_DIPLOCATION("A:8")
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPSETTING(   0x01, DEF_STR(Off))

	PORT_START("SWB")
	PORT_DIPNAME(0x80, 0x00, "Text scale mode") PORT_DIPLOCATION("B:1")
	PORT_DIPSETTING(   0x00, "63 lines per page")
	PORT_DIPSETTING(   0x80, "66 lines per page")
	PORT_DIPNAME(0x40, 0x00, "RAM cartridges") PORT_DIPLOCATION("B:2")
	PORT_DIPSETTING(   0x00, "Downloading")
	PORT_DIPSETTING(   0x40, "Input buffer")
	PORT_DIPUNUSED_DIPLOC(0x20, 0x00, "B:3")
	PORT_DIPUNUSED_DIPLOC(0x10, 0x00, "B:4")
	PORT_DIPNAME(0x0c, 0x00, "RS-232C baud rate") PORT_DIPLOCATION("B:5,6")
	PORT_DIPSETTING(   0x00, "9600")
	PORT_DIPSETTING(   0x04, "19200")
	PORT_DIPSETTING(   0x08, "2400")
	PORT_DIPSETTING(   0x0c, "1200")
	PORT_DIPNAME(0x03, 0x00, "RS-232C parity") PORT_DIPLOCATION("B:7,8")
	PORT_DIPSETTING(   0x00, DEF_STR(None))
	PORT_DIPSETTING(   0x01, "Odd")
	PORT_DIPSETTING(   0x02, "Even")
	PORT_DIPSETTING(   0x03, "None (DTR)")

	PORT_START("CONFIG")
	PORT_CONFNAME(0x01, 0x01, "Paper in the IN tray")
	PORT_CONFSETTING(   0x00, DEF_STR(No))
	PORT_CONFSETTING(   0x01, DEF_STR(Yes))
	PORT_CONFNAME(0x02, 0x02, "Black print cartridge")
	PORT_CONFSETTING(   0x00, "Not installed")
	PORT_CONFSETTING(   0x02, "Installed")
	PORT_CONFNAME(0x04, 0x04, "Colour print cartridge")
	PORT_CONFSETTING(   0x00, "Not installed")
	PORT_CONFSETTING(   0x04, "Installed")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Power switch") PORT_TOGGLE PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(hp_dj550c_device::power_changed), 0)
INPUT_PORTS_END

ROM_START( hpdj550c )
	ROM_REGION(0x80000, "prog", 0)
	ROM_LOAD("1818-5691.u12", 0x00000, 0x80000, CRC(6bb6f417) SHA1(755d2e5c1e23a956c34a24628d5e20b9ea45a382))

	ROM_REGION(0x80000, "data", 0)
	ROM_LOAD("1818-5690.u11", 0x00000, 0x80000, CRC(65251273) SHA1(8794b8080fd638a15e1825bf08a23a3d231cffb4))

	ROM_REGION16_BE(0x80, "eeprom", 0)
	ROM_LOAD16_WORD_SWAP("at93c46.u7", 0x00, 0x80, CRC(124fad0a) SHA1(73dee00e9162a7fffa4f146d77c9a1f1d570b5a0))
ROM_END

} // anonymous namespace


ioport_constructor hp_dj550c_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( hpdj550c );
}

const tiny_rom_entry *hp_dj550c_device::device_rom_region() const
{
	return ROM_NAME( hpdj550c );
}


hp_dj550c_device::hp_dj550c_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, HP_DJ550C, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_eeprom(*this, "eeprom"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_slot(*this, "slot%u", 1U),
	m_prog(*this, "prog"),
	m_rom_data(*this, "data"),
	m_keys(*this, "KEYS"),
	m_switches(*this, { "SWA", "SWB" }),
	m_config(*this, "CONFIG"),
	m_led(*this, "hpdj550c_led%u", 0U),
	m_busy_cb(*this),
	m_ack_cb(*this),
	m_perror_cb(*this),
	m_fault_cb(*this),
	m_select_cb(*this),
	m_txd_cb(*this),
	m_dtr_cb(*this)
{
}

void hp_dj550c_device::device_add_mconfig(machine_config &config)
{
	Z80(config, m_maincpu, 15.9744_MHz_XTAL / 2);   // crystal marked 15.975
	m_maincpu->set_addrmap(AS_PROGRAM, &hp_dj550c_device::mem_map);
	m_maincpu->set_addrmap(AS_IO, &hp_dj550c_device::io_map);

	EEPROM_93C46_16BIT(config, m_eeprom);

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * DPI), int(PAPER_HEIGHT * DPI), DPI, DPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::inkjet(0.0033));
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0c, bitmap_printer_device::PAPER_LETTER);   // A4 and Letter sheets
	m_bitmap_printer->set_sheet_top(0.0);

	// cartridge slots: font ROMs (cartridge1/cartridge2 media) or RAM
	HP_DJ_CART_SLOT(config, m_slot[0], hp_dj_cartridges, "rom");
	HP_DJ_CART_SLOT(config, m_slot[1], hp_dj_cartridges, "rom");
	SOFTWARE_LIST(config, "cart_list").set_original("hpdj_cart");

	config.set_default_layout(layout_hpdj550c);
}

void hp_dj550c_device::device_start()
{
	m_ram = make_unique_clear<u8 []>(0x40000);

	for (int i = 0; i < 0x80; i++)
	{
		m_page_writable[i] = (i >= 0x20) && (i < 0x30);
		if (i < 0x20)
			m_page[i] = &m_prog[i << 14];
		else if (i < 0x30)
			m_page[i] = &m_ram[(i - 0x20) << 14];
		else if (i >= 0x60)
			m_page[i] = &m_rom_data[(i - 0x60) << 14];
		else
			m_page[i] = nullptr;
	}

	m_tick_timer = timer_alloc(FUNC(hp_dj550c_device::tick), this);
	m_cmp_timer = timer_alloc(FUNC(hp_dj550c_device::cmp), this);
	m_servo_timer = timer_alloc(FUNC(hp_dj550c_device::servo), this);
	m_ack_timer = timer_alloc(FUNC(hp_dj550c_device::ack_end), this);

	m_cpos = 0.0;
	m_data = 0;
	m_strobe = 1;
	m_init = 1;
	m_platch = 0;
	m_platch_full = false;
	std::fill(std::begin(m_sfifo), std::end(m_sfifo), 0);
	m_sfifo_head = 0;
	m_sfifo_count = 0;
	m_sstatus = 0;
	m_sheet = false;
	m_sheet_pos = 0;
	m_sheet_rows = 3300;
	m_paper_sensor = 0;

	// the status lines as they are before the first update
	m_lines = 0;
	m_busy_cb(0);
	m_ack_cb(1);
	m_perror_cb(0);
	m_fault_cb(0);
	m_select_cb(0);
	m_txd_cb(1);
	m_dtr_cb(1);

	save_pointer(NAME(m_ram), 0x40000);
	save_item(NAME(m_bank));
	save_item(NAME(m_io));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_cmp));
	save_item(NAME(m_dma_cur));
	save_item(NAME(m_dma_end));
	save_item(NAME(m_dma_full));
	save_item(NAME(m_cpos));
	save_item(NAME(m_cvel));
	save_item(NAME(m_enc_offset));
	save_item(NAME(m_nmi));
	save_item(NAME(m_phase));
	save_item(NAME(m_motor_pos));
	save_item(NAME(m_sheet));
	save_item(NAME(m_sheet_pos));
	save_item(NAME(m_sheet_rows));
	save_item(NAME(m_paper_sensor));
	save_item(NAME(m_pen_sense));
	save_item(NAME(m_data));
	save_item(NAME(m_platch));
	save_item(NAME(m_platch_full));
	save_item(NAME(m_strobe));
	save_item(NAME(m_init));
	save_item(NAME(m_sfifo));
	save_item(NAME(m_sfifo_head));
	save_item(NAME(m_sfifo_count));
	save_item(NAME(m_sstatus));
	save_item(NAME(m_lines));
}

void hp_dj550c_device::device_reset()
{
	// cartridge slots: 256 KiB windows at banks 40-4F and 50-5F
	for (int slot = 0; slot < 2; slot++)
		for (int i = 0; i < 0x10; i++)
		{
			m_page[0x40 + (slot << 4) + i] = m_slot[slot]->page(i);
			m_page_writable[0x40 + (slot << 4) + i] = m_slot[slot]->writable();
		}

	m_bank[0] = 0x00;
	m_bank[1] = 0x02;   // fixed
	m_bank[2] = 0x04;
	m_bank[3] = 0x50;
	std::fill(std::begin(m_io), std::end(m_io), 0);
	m_irq_pending = m_init ? 0 : 0x10;
	m_cmp = 0;
	m_dma_cur = m_dma_end = 0;
	m_dma_full = false;
	m_pen_sense = 0;

	// the carriage stays where it was, the encoder starts at 0
	m_cpos = std::clamp(m_cpos, 0.0, CR_RIGHT_STOP);
	m_cvel = 0.0;
	m_enc_offset = u16(s32(m_cpos));
	m_nmi = false;
	m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);

	m_phase[0] = m_phase[1] = -1;
	m_motor_pos[0] = m_motor_pos[1] = 0;

	m_tick_timer->adjust(attotime::from_hz(38), 0, attotime::from_hz(38));
	m_servo_timer->adjust(attotime::from_hz(250), 0, attotime::from_hz(250));
	arm_cmp();

	// host interface: the input latches are cleared, the serial port runs at the
	// rate of port 10 (9600 baud, no parity until the firmware sets it)
	m_platch_full = false;
	m_sfifo_count = 0;
	m_sstatus = 0;
	m_ack_timer->enable(false);
	m_ack_cb(1);
	update_serial_format();
	receive_register_reset();
	transmit_register_reset();
	m_txd_cb(1);
	m_lines = ~status_lines();
	update_parallel();

	for (auto &led : m_led)
		led = 0;
}

INPUT_CHANGED_MEMBER(hp_dj550c_device::power_changed)
{
	// the power switch resets the printer
	if (newval)
		reset();
}


//**************************************************************************
//  MEMORY
//**************************************************************************

u8 hp_dj550c_device::phys_r(u32 addr) const
{
	u8 const *const page = m_page[(addr >> 14) & 0x7f];
	return page ? page[addr & 0x3fff] : 0xff;
}

u8 hp_dj550c_device::mem_r(offs_t offset)
{
	return phys_r((u32(m_bank[offset >> 14] >> 1) << 14) | (offset & 0x3fff));
}

void hp_dj550c_device::mem_w(offs_t offset, u8 data)
{
	int const bank = (m_bank[offset >> 14] >> 1) & 0x7f;
	if (m_page[bank] && m_page_writable[bank])
		m_page[bank][offset & 0x3fff] = data;
	else
		LOG("%s: write to bank %02x %04x = %02x\n", machine().describe_context(), bank, offset, data);
}

void hp_dj550c_device::mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(hp_dj550c_device::mem_r), FUNC(hp_dj550c_device::mem_w));
}

void hp_dj550c_device::io_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(hp_dj550c_device::io_r), FUNC(hp_dj550c_device::io_w));
}


//**************************************************************************
//  INTERRUPTS AND TIMERS
//**************************************************************************

TIMER_CALLBACK_MEMBER(hp_dj550c_device::tick)
{
	set_irq(1, true);
}

TIMER_CALLBACK_MEMBER(hp_dj550c_device::cmp)
{
	set_irq(5, true);
	arm_cmp();
}

void hp_dj550c_device::arm_cmp()
{
	u16 const delta = m_cmp - counter();
	m_cmp_timer->adjust(attotime::from_ticks(delta ? delta : 0x10000, COUNTER_CLOCK));
}

void hp_dj550c_device::set_irq(int bit, bool state)
{
	if (state)
		m_irq_pending |= 1 << bit;
	else
		m_irq_pending &= ~(1 << bit);
	update_irq();
}

void hp_dj550c_device::update_irq()
{
	// bit 2 follows the serial transmitter; bit 4 (/INIT) is not masked
	if (is_transmit_register_empty())
		m_irq_pending |= 0x04;
	else
		m_irq_pending &= ~0x04;
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, (m_irq_pending & ((m_io[0x08] & 0x6e) | 0x10)) ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(hp_dj550c_device::servo)
{
	// carriage DC motor: PWM duty on port 15 (high nibble counts 13), direction on
	// port 14 bit 7; a first order response, with springy stops at both ends
	u8 const code = m_io[0x15];
	double duty = (code == 0xff) ? 208.0 : double((code >> 4) * 13 + (code & 0x0f));
	if (!BIT(m_io[0x14], 7))
		duty = -duty;
	double target = duty * 0.25;
	if (m_cpos < 0.0)
		target -= m_cpos * 1.3;
	if (m_cpos > CR_RIGHT_STOP)
		target -= (m_cpos - CR_RIGHT_STOP) * 1.3;
	m_cvel += (target - m_cvel) * 0.3;
	m_cpos += m_cvel;

	m_bitmap_printer->setheadpos((int(m_cpos) - PAPER_LEFT) / 2 + SHEET_LEFT, m_bitmap_printer->m_ypos);

	if (BIT(m_io[0x08], 0))
	{
		m_nmi = true;
		m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
	}
}


//**************************************************************************
//  HOST INTERFACE
//**************************************************************************

void hp_dj550c_device::strobe_w(int state)
{
	// falling edge: the ASIC latches the byte and goes busy until it is stored
	if (m_strobe && !state && !m_platch_full)
	{
		m_platch = m_data;
		m_platch_full = true;
		LOGMASKED(LOG_HOST, "parallel %02x\n", m_platch);
		update_parallel();
		service_host();
	}
	m_strobe = state;
}

void hp_dj550c_device::init_w(int state)
{
	// /INIT (Reset/Input Clean) low: the input latch is cleared and the firmware
	// is interrupted; it waits until the line goes high again and restarts
	if (!state && m_init)
	{
		LOGMASKED(LOG_HOST, "/INIT\n");
		m_platch_full = false;
		update_parallel();
	}
	m_init = state;
	set_irq(4, !state);
}

void hp_dj550c_device::dma_store(u8 data)
{
	// the input buffer is in the DRAM, or in a RAM cartridge
	u32 const addr = (u32(m_io[0x41]) << 13) | m_dma_cur;
	int const bank = (addr >> 14) & 0x7f;
	if (m_page[bank] && m_page_writable[bank])
		m_page[bank][addr & 0x3fff] = data;
	m_dma_cur = (m_dma_cur + 1) & 0x1fff;
	if (m_dma_cur == m_dma_end)
		m_dma_full = true;
}

void hp_dj550c_device::service_host()
{
	// the host DMA stores a waiting byte in the input buffer; without it the
	// firmware is asked to (re)start the DMA
	if (m_platch_full && parallel_dma())
	{
		dma_store(m_platch);
		m_platch_full = false;
		m_ack_cb(0);
		m_ack_timer->adjust(attotime::from_usec(5));
	}
	while (m_sfifo_count && serial_dma())
	{
		dma_store(m_sfifo[m_sfifo_head]);
		m_sfifo_head = (m_sfifo_head + 1) % SFIFO_SIZE;
		m_sfifo_count--;
	}
	set_irq(3, m_sfifo_count && !BIT(m_io[0x4a], 6));
	set_irq(6, m_dma_full || (m_platch_full && !BIT(m_io[0x4a], 6)));
	update_parallel();
}

TIMER_CALLBACK_MEMBER(hp_dj550c_device::ack_end)
{
	m_ack_cb(1);
	update_parallel();
}

void hp_dj550c_device::update_parallel()
{
	// port 10 drives the status lines; BUSY is high from /STROBE until the byte is
	// stored and its ACK has ended (the firmware stops the host DMA when its buffer
	// is full or the printer is off line)
	u8 const lines = status_lines();
	u8 const changed = lines ^ m_lines;
	m_lines = lines;
	if (BIT(changed, 0)) m_busy_cb(BIT(lines, 0));
	if (BIT(changed, 1)) m_perror_cb(BIT(lines, 1));
	if (BIT(changed, 2)) m_select_cb(BIT(lines, 2));
	if (BIT(changed, 3)) m_fault_cb(BIT(lines, 3));
	if (BIT(changed, 4)) m_dtr_cb(BIT(lines, 4));
}

u8 hp_dj550c_device::status_lines() const
{
	u8 const ctl = m_io[0x10];
	bool const ack = m_ack_timer->enabled();
	u8 lines = 0;
	lines |= (m_platch_full || ack) ? 0x01 : 0;                  // BUSY
	lines |= BIT(ctl, 7) ? 0x02 : 0;                             // PERROR
	lines |= BIT(ctl, 5) ? 0x04 : 0;                             // SELECT
	lines |= BIT(ctl, 6) ? 0 : 0x08;                             // /FAULT
	lines |= BIT(ctl, 4) ? 0 : 0x10;                             // DTR (RS-232 level, 0 = on)
	return lines;
}

void hp_dj550c_device::update_serial_format()
{
	static constexpr int BAUD[4] = { 9600, 19200, 2400, 1200 };
	static constexpr parity_t PARITY[4] = { PARITY_NONE, PARITY_NONE, PARITY_EVEN, PARITY_ODD };
	u8 const ctl = m_io[0x10];
	// ten bits per frame: eight data bits, or seven and the parity bit
	parity_t const parity = PARITY[(ctl >> 2) & 3];
	set_data_frame(1, (parity == PARITY_NONE) ? 8 : 7, parity, STOP_BITS_1);
	set_rate(BAUD[ctl & 3]);
}

void hp_dj550c_device::rcv_complete()
{
	receive_register_extract();
	if (m_sfifo_count == SFIFO_SIZE)
	{
		LOGMASKED(LOG_HOST, "serial: overrun, %02x lost\n", get_received_char());
		m_sstatus |= 0x80;   // overrun
	}
	else
	{
		m_sfifo[(m_sfifo_head + m_sfifo_count++) % SFIFO_SIZE] = get_received_char();
		m_sstatus |= (is_receive_parity_error() ? 0x01 : 0) | (is_receive_framing_error() ? 0x02 : 0);
		LOGMASKED(LOG_HOST, "serial %02x status %02x\n", get_received_char(), m_sstatus);
	}
	service_host();
}

void hp_dj550c_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void hp_dj550c_device::tra_complete()
{
	update_irq();
}


//**************************************************************************
//  MOTORS AND PAPER
//**************************************************************************

void hp_dj550c_device::step(int motor, u8 coils)
{
	// unipolar steppers, half steps 1 9 8 a 2 6 4 5
	static constexpr s8 PHASE[16] = { -1, 0, 4, -1, 6, 7, 5, -1, 2, 1, 3, -1, -1, -1, -1, -1 };
	int const ph = PHASE[coils & 0x0f];
	if (ph < 0)
		return;
	if (m_phase[motor] >= 0)
	{
		int const d = (ph - m_phase[motor]) & 7;
		int const dir = (d == 1) ? 1 : (d == 7) ? -1 : 0;
		m_motor_pos[motor] += dir;
		if (dir && (motor == 0))
			paper_step(dir);
	}
	m_phase[motor] = ph;
}

void hp_dj550c_device::paper_step(int dir)
{
	// the paper motor picks a sheet while the carriage is pressed against the right stop
	if (!m_sheet && (dir > 0) && (m_cpos > CR_PICK) && BIT(m_config->read(), 0))
	{
		m_sheet = true;
		m_sheet_pos = -PICK_DISTANCE;
		m_sheet_rows = ((m_bitmap_printer->ioport("PAPER")->read() & 0x07) == 3) ? 3508 : 3300;
		LOGMASKED(LOG_PAPER, "paper: pick\n");
	}

	if (m_sheet)
	{
		// the paper view follows the lowest nozzle row; the page is filed when the
		// trailing edge has passed the nozzles
		m_sheet_pos += dir;
		m_bitmap_printer->m_ypos = std::max(sheet_row() + HEAD_ROWS, 0);
		if (m_bitmap_printer->check_new_page())
		{
			LOGMASKED(LOG_PAPER, "paper: out\n");
			m_sheet = false;
		}
		m_bitmap_printer->m_ypos = std::max(sheet_row() + HEAD_ROWS, 0);
	}

	u8 const sensor = (m_sheet && (m_sheet_pos >= 0) && (m_sheet_pos < m_sheet_rows)) ? 1 : 0;
	if (sensor != m_paper_sensor)
		LOGMASKED(LOG_PAPER, "paper: sensor %d at %d\n", sensor, m_sheet_pos);
	m_paper_sensor = sensor;
}

//**************************************************************************
//  PRINTING
//**************************************************************************

void hp_dj550c_device::plot(int x, int row, int ink)
{
	// ink 0 black, 1 yellow, 2 magenta, 3 cyan; overlapping inks mix
	static constexpr u32 INK[4] = { 0x000000, 0xffff00, 0xff00ff, 0x00ffff };
	int const px = (x - PAPER_LEFT) / 2 + SHEET_LEFT;
	int const y = sheet_row() + row;
	if ((px < 0) || (px >= int(PAPER_WIDTH * DPI)) || (y < 0) || (y >= m_sheet_rows))
		return;
	u32 &p = m_bitmap_printer->pix(y, px);
	u32 const c = INK[ink];
	p = (p & 0xff000000) | std::min(p & 0xff0000, c & 0xff0000) | std::min(p & 0xff00, c & 0xff00) | std::min(p & 0xff, c & 0xff);
}

void hp_dj550c_device::run_list()
{
	u16 const ptr = (m_io[0x1c] << 8) | m_io[0x1d];
	u32 addr = (u32(BIT(ptr, 15) ? (m_io[0x42] >> 1) : 0x28) << 14) | (ptr & 0x3fff);
	u32 seg = 0;
	bool const colour = BIT(m_io[0x57], 5);
	int const dir = BIT(m_io[0x00], 2) ? -1 : 1;
	int const x0 = s16(m_io[0x06] | (m_io[0x07] << 8)) + ((dir < 0) ? LEFTWARD_OFFSET : 0);

	// a dot is printed at a position for each colour; the colour cartridge prints 800
	// counts left of the black one, magenta another 40 further left, and its nozzle
	// groups lie 0, 3 and 7 rows below where they would be in a straight line
	auto dot = [this, colour] (int x, int r)
	{
		if (!colour)
			plot(x, r, 0);
		else
		{
			static constexpr int DX[3] = { -800, -840, -800 }, DY[3] = { 0, 3, 7 };
			int const g = std::min(r / 16, 2);
			plot(x + DX[g], r + DY[g], g + 1);
		}
	};

	int cursor[2] = { x0, x0 };
	int chr = 0;
	enum { SQUEEZE_NONE, SQUEEZE_HALF, SQUEEZE_FLAGGED } squeeze = SQUEEZE_NONE;
	int bold = 0;
	bool half_rows = false;
	for (int n = 0; n < 0x1000; n++)   // a guard against a list without an end
	{
		u8 const op = phys_r(addr++);
		if (op == 0x60)
			break;
		switch (op & 0xe0)
		{
		case 0xc0:
			seg = op & 0x1f;
			break;

		case 0x20:
			{
				u8 b[8];
				for (auto &v : b)
					v = phys_r(addr++);
				u32 const src = (seg << 16) | b[4] | (b[5] << 8);
				int const count = b[0];
				int const rows = b[7];
				if (count == 1)
				{
					// pen identification: one byte per nozzle line, bit 7 fires it; the
					// black cartridge answers on line 49, the colour cartridge on line 48
					u8 const pens = m_config->read();
					for (int i = 0; i < rows; i++)
						if (BIT(phys_r(src + i), 7) &&
								((!colour && (i == 49) && BIT(pens, 1)) || (colour && (i == 48) && BIT(pens, 2))))
							m_pen_sense = 0x80;
				}
				else if (count)
				{
					// raster: columns of 'rows' bytes, eight dots across per byte (bit 7
					// leftmost); a leftward pass starts at the right edge, with the source
					// pointing at the last byte
					int const width = b[1] | (b[2] << 8);
					int const columns = width / (8 * count);
					for (int c = 0; c < columns; c++)
						for (int r = 0; r < std::min(rows, 56); r++)
						{
							u8 const v = (dir > 0) ? phys_r(src + c * rows + r) : phys_r(src - c * rows - (rows - 1 - r));
							for (int i = 0; v && (i < 8); i++)
								if (BIT(v, 7 - i))
									for (int k = 0; k < count; k += 2)
									{
										int const dot_index = (dir > 0) ? (c * 8 + i) : (c * 8 + 7 - i);
										dot(x0 + ((dir < 0) ? LEFTWARD_RASTER : 0) + dir * (dot_index * count + k), r);
									}
						}
				}
				LOGMASKED(LOG_LIST, "raster %02x %02x%02x %02x %06x %02x %d\n", count, b[2], b[1], b[3], src, b[6], rows);
			}
			break;

		case 0x00:
			{
				// character of a built-in font
				u8 b[6];
				for (auto &v : b)
					v = phys_r(addr++);
				int const step = ((op & 0x1f) << 8) | b[0];
				int const columns = b[1];
				u32 pa = (seg << 16) | b[2] | (b[3] << 8);
				u32 pb = (seg << 16) | b[4] | (b[5] << 8);
				int &x = cursor[chr++ & 1];
				x += dir * step;

				// the glyph's columns in font order; leftwards the pointers start at
				// the last byte and count down
				auto next_column = [this, dir, &pa, &pb] (u8 (&col)[7]) -> u8
				{
					u8 const mask = phys_r(pa);
					pa += dir;
					for (int k = 0; k < 7; k++)
					{
						int const kk = (dir > 0) ? k : (6 - k);
						col[kk] = BIT(mask, kk) ? phys_r(pb) : 0;
						if (BIT(mask, kk))
							pb += dir;
					}
					return mask;
				};
				auto draw_column = [&dot, bold, half_rows] (int cx, u8 const (&col)[7])
				{
					for (int r = 0; r < 56; r++)
						if (BIT(col[r >> 3], 7 - (r & 7)))
						{
							int const row = half_rows ? (r >> 1) : r;
							dot(cx, row);
							if (bold)
								dot(cx + bold, row);
						}
				};
				for (int c = 0; c < columns; )
				{
					u8 col[7];
					u8 const mask = next_column(col);
					if ((squeeze == SQUEEZE_FLAGGED) && !BIT(mask, 7))
						continue;   // not printed at 16.67 characters per inch
					draw_column(x + dir * c, col);
					c++;
					if ((squeeze == SQUEEZE_HALF) && (c < columns))
						next_column(col);   // every other column (the other half has its own entry)
				}
				x += dir * columns;
			}
			break;

		case 0x40:
			addr += 6;
			break;

		case 0x80:
		case 0xa0:
			// character width: 80/81 every other column (20 characters per inch, one entry
			// for each half), 82 the columns with bit 7 of their mask (16.67 cpi);
			// A0+n for landscape characters: bit 0 every other row (20 cpi), bit 1 bold
			// (each column printed again two units on; guess)
			if (op & 0x20)
			{
				squeeze = SQUEEZE_NONE;
				half_rows = BIT(op, 0);
				bold = BIT(op, 1) ? 2 : 0;
			}
			else
				squeeze = BIT(op, 1) ? SQUEEZE_FLAGGED : SQUEEZE_HALF;
			LOGMASKED(LOG_LIST, "op %02x\n", op);
			break;

		case 0xe0:
			squeeze = SQUEEZE_NONE;
			half_rows = false;
			bold = 0;
			LOGMASKED(LOG_LIST, "op %02x\n", op);
			break;

		default:
			LOGMASKED(LOG_LIST, "op %02x\n", op);
			break;
		}
	}
	LOGMASKED(LOG_LIST, "pass at %d dir %d row %d %s\n", x0, dir, sheet_row(), colour ? "colour" : "black");
}


//**************************************************************************
//  PORTS
//**************************************************************************

u8 hp_dj550c_device::panel_r()
{
	// keys: column strobes on port 55, rows on bits 0 and 1;
	// configuration switches: column strobes on port 56, rows on bits 5 and 6
	u8 data = 0xff;
	u8 const keys = m_keys->read();
	static constexpr u8 KEY_COLS[4] = { 0xfd, 0xfb, 0xef, 0xdf };
	for (int i = 0; i < 4; i++)
		if (m_io[0x55] == KEY_COLS[i])
			data &= ~0x03 | (BIT(keys, 7 - 2 * i) | (BIT(keys, 6 - 2 * i) << 1));
	for (int i = 0; i < 8; i++)
		if (m_io[0x56] == u8(~(1 << i)))
		{
			// columns FE..F7: switches A8/A7 .. A2/A1, EF..7F: B8/B7 .. B2/B1
			u8 const sw = ~m_switches[i >> 2]->read();
			int const bit = 2 * (i & 3);
			data &= ~0x60 | (BIT(sw, bit) << 6) | (BIT(sw, bit + 1) << 5);
		}
	return data;
}

u8 hp_dj550c_device::io_r(offs_t offset)
{
	offset &= 0xff;
	u8 data = m_io[offset];
	switch (offset)
	{
	case 0x04: data = encoder() & 0xff; break;
	case 0x05: data = encoder() >> 8; break;
	case 0x09: data = m_irq_pending & (m_io[0x08] | 0x10); break;
	case 0x11: data = m_sstatus; break;
	case 0x12:
		data = m_sfifo[m_sfifo_head];
		if (!machine().side_effects_disabled() && m_sfifo_count)
		{
			m_sfifo_head = (m_sfifo_head + 1) % SFIFO_SIZE;
			m_sfifo_count--;
			m_sstatus = 0;
			service_host();
		}
		break;
	case 0x0c: data = counter() & 0xff; break;
	case 0x0d: data = counter() >> 8; break;
	case 0x0e: data = m_pen_sense; break;
	case 0x0f:
		data = m_pen_sense;
		if (!machine().side_effects_disabled())
			m_pen_sense = 0;
		break;
	case 0x49:
		data = m_nmi ? 1 : 0;
		if (!machine().side_effects_disabled() && m_nmi)
		{
			m_nmi = false;
			m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
		}
		break;
	case 0x50: data = m_bank[0]; break;
	case 0x51: data = m_bank[2]; break;
	case 0x52: data = m_bank[3]; break;
	case 0x59: data = (m_io[0x57] & ~0x42) | (m_eeprom->do_read() << 1) | (m_paper_sensor << 6); break;
	case 0x5a: data = panel_r(); break;
	case 0x5c: data = m_dma_end & 0xff; break;
	case 0x5d: data = m_dma_end >> 8; break;
	case 0x5e: data = m_dma_cur & 0xff; break;
	case 0x5f: data = m_dma_cur >> 8; break;
	}
	return data;
}

void hp_dj550c_device::io_w(offs_t offset, u8 data)
{
	offset &= 0xff;
	u8 const old = m_io[offset];
	m_io[offset] = data;
	switch (offset)
	{
	case 0x02:
		m_cmp = (m_cmp & 0xff00) | data;
		set_irq(5, false);
		arm_cmp();
		break;
	case 0x03:
		m_cmp = (m_cmp & 0x00ff) | (data << 8);
		set_irq(5, false);
		arm_cmp();
		break;
	case 0x04:
	case 0x05:
		m_enc_offset = u16(s32(m_cpos)) - (m_io[0x04] | (m_io[0x05] << 8));
		break;
	case 0x07:
		run_list();
		break;
	case 0x08:
		update_irq();
		break;
	case 0x0e:
	case 0x0f:
		set_irq(1, false);
		break;
	case 0x0b:
	case 0x4a:
		service_host();
		break;
	case 0x10:
		LOGMASKED(LOG_HOST, "%s: host control %02x\n", machine().describe_context(), data);
		if ((data ^ old) & 0x0f)
			update_serial_format();
		update_parallel();
		break;
	case 0x11:
		LOGMASKED(LOG_HOST, "serial: send %02x\n", data);
		if (is_transmit_register_empty())
		{
			transmit_register_setup(data);
			update_irq();
		}
		else
			LOGMASKED(LOG_HOST, "serial: transmitter busy, %02x lost\n", data);
		break;
	case 0x4b:
		step(0, data & 0x0f);
		step(1, data >> 4);
		break;
	case 0x50: m_bank[0] = data; break;
	case 0x51: m_bank[2] = data; break;
	case 0x52: m_bank[3] = data; break;
	case 0x55:
		// the LED latch; the key scan pulses a column for a few microseconds
		for (int i = 0; i < 8; i++)
			m_led[i] = BIT(data, i);
		break;
	case 0x57:
		m_eeprom->di_write(BIT(data, 0));
		m_eeprom->cs_write(BIT(data, 2));
		m_eeprom->clk_write(BIT(data, 3));
		m_led[8] = BIT(~data, 0);   // Busy, active low
		break;
	case 0x5c: m_dma_end = (m_dma_end & 0x1f00) | data; break;
	case 0x5d: m_dma_end = (m_dma_end & 0x00ff) | ((data & 0x1f) << 8); break;
	case 0x5e:
		m_dma_cur = (m_dma_cur & 0x1f00) | data;
		m_dma_full = false;
		service_host();
		break;
	case 0x5f:
		m_dma_cur = (m_dma_cur & 0x00ff) | ((data & 0x1f) << 8);
		m_dma_full = false;
		service_host();
		break;
	}
}
