// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    CalcuScribe Duo (CalcuScribe, Inc. / LinkCorp, Inc., 1998-2000)

    Battery-powered "smart keyboard" word processor / calculator for schools,
    a competitor of the AlphaSmart.  It has a "Mac" (ADB) and a "PC" (PS/2)
    port; with the cable plugged in it acts as a keyboard for the host and
    can "type" stored files into whatever application is open.

    This device models the complete machine and plugs into an ADB connector,
    so it can be used as the keyboard of an emulated Macintosh:

        mame macse -adb:0 calcuscribe

    The stand-alone system "cscribed" (src/mame/misc/calcuscribe.cpp) puts
    it on an otherwise empty ADB bus.

    Main board AC-P0101-A ("TLX1"), LinkCorp, Inc., September 1998:

    U7   Mitsubishi M37702S1BFP (7700 family, ROM-less, 512 bytes internal RAM)
    Y1   19.6608 MHz
    ---  ST M27C801 (1 MiB EPROM, firmware (c) 2000 CalcuScribe, Inc.)
    U21  Hitachi HD61830B00 LCD timing controller
    U22  NEC D43256BGU-70L (32 KiB SRAM, LCD display memory)
    ---  Samsung K6T1008C2E-GB70 (128 KiB SRAM, battery backed by a CR2032)
    U19  74HCT244 (keyboard row input buffer)
    U30  74HC05 (open drain drivers for the ADB / PS/2 lines)
    U24  TLC556, U?? MAX818 supervisor, 74HC132, 74HC174, ...
    Separate keypad board AC-P0201 with a 20 key calculator pad.

    LCD: 480x64 pixels.  The HD61830 is programmed for 240x128 (two halves
    of 64 lines); the "upper" half drives the left 240 columns and the
    "lower" half the right 240 columns.  The firmware uses two modes:
      - 80x8 characters, character mode with the internal 5x7 CG
      - 40x4 characters ("zoom", default), graphics mode with a 12x16 font

    Memory map (worked out from the firmware):
      000000-0FFFFF  EPROM (internal SFRs/RAM overlay 000000-00027F)
      4x0000         write: 74x259-style latch, A16-A18 select output, A19 data
                     read : keyboard matrix rows (active low, '244)
      800000         write: keypad column select (bits 0-4, active high)
                     read : bits 0-3 keypad rows (active low)
                            bit 4 Mac cable present, bit 5 PC cable present
      FE0000-FFFFFF  128 KiB battery-backed SRAM

    Latch outputs:
      Q0 HD61830 R/W   Q1 HD61830 E   Q2 E of a character LCD (CalcuScribe
      Uno, not fitted here)   Q3 HD61830 RS   Q4/Q5 LCD power/enable (?)
      Q6 ADB data low   Q7 PS/2 line low

    Ports:
      P4.3-7, P5.3/6/7, P6.0/7, P8.1/4-7  keyboard columns (driven low by
                                          setting the direction bit)
      P6.3  power latch, P6.4 low battery (1 = low)
      P6.5  ADB data / PS/2 clock in (inverted), P6.6 PS/2 data in (inverted)
      P7    HD61830 data bus
      UART0 infrared port ("InfraLink")

    Interrupts: INT0 any key, Timer B2 keyboard scan, Timer B0 tick,
    UART0 RX/TX (IR), watchdog.

    The ADB protocol is bit-banged with cycle-counted delay loops, so the
    CPU clock and the 7700 cycle timings matter.

    TODO:
    - verify the "cable present" polarity and the left/right LCD halves
    - PS/2 keyboard emulation port, infrared port, beeper
    - proper power switch handling (the firmware powers down with STP)

***************************************************************************/

#include "emu.h"
#include "calcuscribe.h"

#include "cpu/m37710/m37710.h"
#include "machine/nvram.h"
#include "video/hd61830.h"

#include "emupal.h"
#include "screen.h"

#define LOG_LATCH   (1U << 1)
#define LOG_ADB     (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

class calcuscribe_device : public adb_device_interface, public adb_slot_card_interface
{
public:
	calcuscribe_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	DECLARE_INPUT_CHANGED_MEMBER(power_w);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void adb_w(int state) override;

private:
	required_device<m37702s1_device> m_maincpu;
	required_device<hd61830_device> m_lcdc;
	required_device<screen_device> m_screen;
	required_ioport_array<15> m_cols;
	required_ioport_array<5> m_kpcols;
	required_ioport m_config;

	bitmap_ind16 m_lcd_bitmap;
	emu_timer *m_anykey_timer;

	u8 m_latch;
	u8 m_p7;
	u8 m_lcd_in;
	u8 m_kp_select;
	int m_adb_line;

	void program_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;

	void palette_init(palette_device &palette) const;
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	u16 io4_r(offs_t offset);
	void io4_w(offs_t offset, u16 data, u16 mem_mask);
	u16 io8_r(offs_t offset);
	void io8_w(offs_t offset, u16 data, u16 mem_mask);

	u8 p6_r();
	u8 p7_r();
	void p7_w(u8 data);

	void latch_w(unsigned bit, int state);
	TIMER_CALLBACK_MEMBER(anykey_tick);
};


ROM_START(calcuscribe)
	ROM_REGION16_LE(0x100000, "maincpu", 0)
	ROM_LOAD("calcuscribe_m27c801.bin", 0x000000, 0x100000, CRC(1b5a408b) SHA1(192966acc2b2b0040bd963d5e5be45623094ef84))
ROM_END


/*
    Keyboard matrix.  Rows are bits 0-7 of each column.  The legends come
    from the firmware's key table at 0x0002F9 (11 bytes per key:
    normal/Alt/Ctrl/Shift/Caps codes + Fn keypad code).

    Columns 15-19 of the firmware's scan buffer are the calculator keypad,
    read through 0x800000 (row 0 is the bottom row of the keypad).

    The keys called "Matrix keypad" sit on otherwise unused matrix
    positions; they probably belong to a different keyboard variant and are
    left without a default host key.
*/
INPUT_PORTS_START(calcuscribe)
	PORT_START("COL.0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)          PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)          PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)          PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)          PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE)      PORT_CHAR('`') PORT_CHAR('~')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TAB)        PORT_CHAR(9)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC)        PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LWIN)       PORT_NAME("Command")

	PORT_START("COL.1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)          PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)          PORT_CHAR('w') PORT_CHAR('W')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)          PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)          PORT_CHAR('2') PORT_CHAR('@')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F1)         PORT_CHAR(UCHAR_MAMEKEY(F1))  PORT_NAME("F1 (Help)")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CAPSLOCK)   PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F5)         PORT_CHAR(UCHAR_MAMEKEY(F5))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 0")

	PORT_START("COL.2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)          PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)          PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)          PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)          PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F2)         PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F3)         PORT_CHAR(UCHAR_MAMEKEY(F3))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F4)         PORT_CHAR(UCHAR_MAMEKEY(F4))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("COL.3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 3")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 4")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL)   PORT_CHAR(UCHAR_SHIFT_2) PORT_NAME("Ctrl")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 7")

	PORT_START("COL.4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)          PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)          PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)          PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)          PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)          PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)          PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)          PORT_CHAR('g') PORT_CHAR('G')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)          PORT_CHAR('b') PORT_CHAR('B')

	PORT_START("COL.5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 8")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_NUMLOCK)    PORT_NAME("Spell")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad 9")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PAUSE)      PORT_NAME("Print")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad -")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RWIN)       PORT_NAME("CalcuScribe / Fn")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LALT)       PORT_CHAR(UCHAR_MAMEKEY(LALT)) PORT_NAME("Alt (left)")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RALT)       PORT_CHAR(UCHAR_MAMEKEY(RALT)) PORT_NAME("Alt (right)")

	PORT_START("COL.6")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT)     PORT_CHAR(UCHAR_SHIFT_1) PORT_NAME("Shift (left)")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad +")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad Enter")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad .")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad /")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RSHIFT)     PORT_NAME("Shift (right)")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Matrix keypad *")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MENU)       PORT_NAME("Local")

	PORT_START("COL.7")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER)      PORT_CHAR(13) PORT_NAME("Enter")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH)  PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F10)        PORT_CHAR(UCHAR_MAMEKEY(F10)) PORT_NAME("F10 (File info)")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F9)         PORT_CHAR(UCHAR_MAMEKEY(F9))  PORT_NAME("F9 (Zoom)")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE)  PORT_CHAR(8)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F6)         PORT_CHAR(UCHAR_MAMEKEY(F6))  PORT_NAME("F6 (Status line)")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)      PORT_CHAR(' ')

	PORT_START("COL.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)          PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)          PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)          PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)          PORT_CHAR('7') PORT_CHAR('&')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)          PORT_CHAR('6') PORT_CHAR('^')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)          PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)          PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)          PORT_CHAR('n') PORT_CHAR('N')

	PORT_START("COL.9")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)      PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)          PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)          PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8') PORT_CHAR('*')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)     PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0xc0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("COL.10")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)          PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)      PORT_CHAR(';') PORT_CHAR(':')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)          PORT_CHAR('0') PORT_CHAR(')')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-') PORT_CHAR('_')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)      PORT_CHAR('\'') PORT_CHAR('"')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)      PORT_CHAR('/') PORT_CHAR('?')

	PORT_START("COL.11")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)       PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)          PORT_CHAR('o') PORT_CHAR('O')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)          PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9') PORT_CHAR('(')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F8)         PORT_CHAR(UCHAR_MAMEKEY(F8))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F7)         PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0xc0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("COL.12")
	PORT_BIT(0x07, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F12)        PORT_CHAR(UCHAR_MAMEKEY(F12)) PORT_NAME("F12 (Recalculate)")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_INSERT)     PORT_CHAR(UCHAR_MAMEKEY(INSERT))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_END)        PORT_CHAR(UCHAR_MAMEKEY(END))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT)      PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

	PORT_START("COL.13")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PRTSCR)     PORT_NAME("Send")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGDN)       PORT_CHAR(UCHAR_MAMEKEY(PGDN))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F11)        PORT_CHAR(UCHAR_MAMEKEY(F11))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL)        PORT_CHAR(UCHAR_MAMEKEY(DEL))
	PORT_BIT(0x60, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN)       PORT_CHAR(UCHAR_MAMEKEY(DOWN))

	PORT_START("COL.14")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SCRLOCK)    PORT_NAME("Recv")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGUP)       PORT_CHAR(UCHAR_MAMEKEY(PGUP))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_HOME)       PORT_CHAR(UCHAR_MAMEKEY(HOME))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP)         PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT)       PORT_CHAR(UCHAR_MAMEKEY(LEFT))

	// calculator keypad, rows 0-3 = bottom to top
	PORT_START("KP.0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Edit")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad (-)")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Clr")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Func.")
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KP.1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0_PAD)      PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1_PAD)      PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4_PAD)      PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7_PAD)      PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KP.2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL_PAD)    PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD)) PORT_NAME("Keypad .")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2_PAD)      PORT_CHAR(UCHAR_MAMEKEY(2_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)      PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8_PAD)      PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KP.3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER_PAD)  PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) PORT_NAME("Keypad =")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3_PAD)      PORT_CHAR(UCHAR_MAMEKEY(3_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)      PORT_CHAR(UCHAR_MAMEKEY(6_PAD))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9_PAD)      PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KP.4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS_PAD)   PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS_PAD)  PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ASTERISK)   PORT_CHAR(UCHAR_MAMEKEY(ASTERISK))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH_PAD)  PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD))
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("CONFIG")
	PORT_CONFNAME(0x03, 0x01, "Cable")
	PORT_CONFSETTING(0x00, "None")
	PORT_CONFSETTING(0x01, "Mac (ADB)")
	PORT_CONFSETTING(0x02, "PC (PS/2)")
	PORT_CONFNAME(0x04, 0x00, "Batteries")
	PORT_CONFSETTING(0x00, "OK")
	PORT_CONFSETTING(0x04, "Low")

	PORT_START("POWER")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Power") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(calcuscribe_device::power_w), 0)
INPUT_PORTS_END


calcuscribe_device::calcuscribe_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: adb_device_interface(mconfig, ADB_CALCUSCRIBE, tag, owner, clock)
	, adb_slot_card_interface(mconfig, *this, DEVICE_SELF)
	, m_maincpu(*this, "maincpu")
	, m_lcdc(*this, "lcdc")
	, m_screen(*this, "screen")
	, m_cols(*this, "COL.%u", 0U)
	, m_kpcols(*this, "KP.%u", 0U)
	, m_config(*this, "CONFIG")
	, m_anykey_timer(nullptr)
	, m_latch(0)
	, m_p7(0xff)
	, m_lcd_in(0xff)
	, m_kp_select(0)
	, m_adb_line(1)
{
}

const tiny_rom_entry *calcuscribe_device::device_rom_region() const
{
	return ROM_NAME(calcuscribe);
}

ioport_constructor calcuscribe_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(calcuscribe);
}

void calcuscribe_device::program_map(address_map &map)
{
	map.unmap_value_high();
	map(0x000000, 0x0fffff).mirror(0x300000).rom().region("maincpu", 0);
	map(0x400000, 0x4fffff).rw(FUNC(calcuscribe_device::io4_r), FUNC(calcuscribe_device::io4_w));
	map(0x800000, 0x8fffff).rw(FUNC(calcuscribe_device::io8_r), FUNC(calcuscribe_device::io8_w));
	map(0xfe0000, 0xffffff).ram().share("nvram");
}

void calcuscribe_device::lcdc_map(address_map &map)
{
	map(0x0000, 0x7fff).mirror(0x8000).ram();
}

void calcuscribe_device::palette_init(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0xa8, 0xb4, 0x96));
	palette.set_pen_color(1, rgb_t(0x20, 0x28, 0x30));
}

void calcuscribe_device::device_add_mconfig(machine_config &config)
{
	M37702S1(config, m_maincpu, 19.6608_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &calcuscribe_device::program_map);
	m_maincpu->p6_in_cb().set(FUNC(calcuscribe_device::p6_r));
	m_maincpu->p7_in_cb().set(FUNC(calcuscribe_device::p7_r));
	m_maincpu->p7_out_cb().set(FUNC(calcuscribe_device::p7_w));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	SCREEN(config, m_screen);
	m_screen->set_lcd();
	m_screen->set_refresh_hz(60);
	m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(0));
	// the HD61830 draws its two 64-line halves into a 128 line bitmap
	m_screen->set_size(480, 128);
	m_screen->set_visarea(0, 480 - 1, 0, 64 - 1);
	m_screen->set_screen_update(FUNC(calcuscribe_device::screen_update));
	m_screen->set_palette("palette");

	PALETTE(config, "palette", FUNC(calcuscribe_device::palette_init), 2);

	HD61830(config, m_lcdc, 19.6608_MHz_XTAL / 8); // clock not verified
	m_lcdc->set_addrmap(0, &calcuscribe_device::lcdc_map);
	m_lcdc->set_screen(m_screen);
}

void calcuscribe_device::device_start()
{
	adb_device_interface::device_start();

	m_lcd_bitmap.allocate(480, 128);
	m_anykey_timer = timer_alloc(FUNC(calcuscribe_device::anykey_tick), this);

	save_item(NAME(m_latch));
	save_item(NAME(m_p7));
	save_item(NAME(m_lcd_in));
	save_item(NAME(m_kp_select));
	save_item(NAME(m_adb_line));
}

void calcuscribe_device::device_reset()
{
	adb_device_interface::device_reset();

	m_latch = 0;
	m_kp_select = 0;
	m_lcd_in = 0xff;
	if (adb_is_connected())
		adb_release();

	m_anykey_timer->adjust(attotime::from_hz(200), 0, attotime::from_hz(200));
}

INPUT_CHANGED_MEMBER(calcuscribe_device::power_w)
{
	// The firmware switches itself off by dropping P6.3 and executing STP.
	// Pressing the power key restarts it from reset; the file memory is
	// battery backed.
	if (newval)
		m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
}


/***************************************************************************
    Video
***************************************************************************/

u32 calcuscribe_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_lcd_bitmap.fill(0);
	m_lcdc->screen_update(screen, m_lcd_bitmap, m_lcd_bitmap.cliprect());

	// upper half of the controller's frame -> left half of the panel,
	// lower half -> right half
	for (int y = cliprect.min_y; y <= cliprect.max_y && y < 64; y++)
	{
		for (int x = cliprect.min_x; x <= cliprect.max_x; x++)
		{
			if (x < 240)
				bitmap.pix(y, x) = m_lcd_bitmap.pix(y, x);
			else
				bitmap.pix(y, x) = m_lcd_bitmap.pix(y + 64, x - 240);
		}
	}

	return 0;
}


/***************************************************************************
    I/O
***************************************************************************/

u16 calcuscribe_device::io4_r(offs_t offset)
{
	// Keyboard rows through the '244.  A column is active when its port pin
	// is switched to output (the port data registers are kept at 0).
	address_space &space = m_maincpu->space(AS_PROGRAM);
	u8 const p4d = space.read_byte(0x0c);
	u8 const p5d = space.read_byte(0x0d);
	u8 const p6d = space.read_byte(0x10);
	u8 const p8d = space.read_byte(0x14);

	u16 const active =
			(BIT(p8d, 4) << 0) | (BIT(p8d, 5) << 1) | (BIT(p8d, 6) << 2) | (BIT(p8d, 7) << 3) |
			(BIT(p8d, 1) << 4) | (BIT(p6d, 0) << 5) | (BIT(p5d, 3) << 6) | (BIT(p6d, 7) << 7) |
			(BIT(p4d, 3) << 8) | (BIT(p4d, 4) << 9) | (BIT(p4d, 5) << 10) | (BIT(p4d, 6) << 11) |
			(BIT(p4d, 7) << 12) | (BIT(p5d, 6) << 13) | (BIT(p5d, 7) << 14);

	u8 rows = 0;
	for (int col = 0; col < 15; col++)
		if (BIT(active, col))
			rows |= m_cols[col]->read();

	u8 const data = ~rows;
	return data | (data << 8);
}

void calcuscribe_device::io4_w(offs_t offset, u16 data, u16 mem_mask)
{
	// A16-A18 select the latch output, A19 is the data bit
	unsigned const a = (offset >> 15) & 0x0f;
	latch_w(a & 7, BIT(a, 3));
}

u16 calcuscribe_device::io8_r(offs_t offset)
{
	u8 rows = 0;
	for (int col = 0; col < 5; col++)
		if (BIT(m_kp_select, col))
			rows |= m_kpcols[col]->read();

	u8 data = ~rows & 0x0f;

	// cable sense, polarity not verified
	u8 const cable = m_config->read() & 0x03;
	if (cable == 1)
		data |= 0x10;
	else if (cable == 2)
		data |= 0x20;

	data |= 0xc0;
	return data | (data << 8);
}

void calcuscribe_device::io8_w(offs_t offset, u16 data, u16 mem_mask)
{
	u8 const d = ACCESSING_BITS_0_7 ? (data & 0xff) : (data >> 8);
	m_kp_select = d & 0x1f;
}

void calcuscribe_device::latch_w(unsigned bit, int state)
{
	u8 const old = m_latch;
	if (state)
		m_latch |= 1 << bit;
	else
		m_latch &= ~(1 << bit);

	if (old == m_latch)
		return;

	LOGMASKED(LOG_LATCH, "%s: latch Q%u = %d\n", machine().describe_context(), bit, state);

	switch (bit)
	{
	case 1: // HD61830 E
		{
			bool const rw = BIT(m_latch, 0);
			bool const rs = BIT(m_latch, 3);
			if (state && rw)
			{
				m_lcd_in = rs ? m_lcdc->status_r() : m_lcdc->data_r();
			}
			else if (!state && !rw)
			{
				if (rs)
					m_lcdc->control_w(m_p7);
				else
					m_lcdc->data_w(m_p7);
			}
		}
		break;

	case 6: // ADB data: 1 pulls the line low through the '05
		if (adb_is_connected())
		{
			LOGMASKED(LOG_ADB, "%s: ADB drive %s\n", machine().describe_context(), state ? "low" : "released");
			adb_drive_w(state ? CLEAR_LINE : ASSERT_LINE);
			m_maincpu->yield();
		}
		break;

	default:
		break;
	}
}

u8 calcuscribe_device::p6_r()
{
	u8 data = 0x0c;

	// low battery detector
	if (BIT(m_config->read(), 2))
		data |= 0x10;

	// ADB data line, inverted
	if ((m_config->read() & 0x03) == 1 && !m_adb_line)
		data |= 0x20;

	return data;
}

u8 calcuscribe_device::p7_r()
{
	// HD61830 output while E and R/W are high
	if (BIT(m_latch, 1) && BIT(m_latch, 0))
		return m_lcd_in;

	return 0xff;
}

void calcuscribe_device::p7_w(u8 data)
{
	m_p7 = data;
}

void calcuscribe_device::adb_w(int state)
{
	m_adb_line = state ? 1 : 0;
}

TIMER_CALLBACK_MEMBER(calcuscribe_device::anykey_tick)
{
	// INT0 is a level-sensitive "any key down" input.  Between scans the
	// firmware leaves all columns enabled, so any pressed key pulls it.
	bool any = false;
	for (auto &col : m_cols)
		any = any || col->read();
	for (auto &col : m_kpcols)
		any = any || (col->read() & 0x0f);

	m_maincpu->set_input_line(M37710_LINE_IRQ0, any ? ASSERT_LINE : CLEAR_LINE);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(ADB_CALCUSCRIBE, adb_slot_card_interface, calcuscribe_device, "calcuscribe", "CalcuScribe Duo")
