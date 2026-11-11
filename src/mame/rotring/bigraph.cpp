// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Rotring bigraph script 1R90 and bigraph scriber 1B90

    Electronic lettering machines for technical drawings. A pen carriage
    moves along a ruler (X), the pen arm moves across it (Y), both driven
    by micro-stepped stepper motors. A two-stage solenoid lowers the pen.
    Characters come from font ROMs ("Rapidoscript Cassette") that hold the
    glyphs as plain text in the machine's command language.

    Reverse engineering notes and ROM dumps: github.com/dl6lr/bigraph_script

    bigraph script 1R90
    -------------------
    MC68008 @ 8 MHz, 27C512 firmware (RS 1.5), 6264 RAM, 38-pin cassette
    slot, 8x8 key matrix, 4-bit DIP switch, beeper, two hall sensors for
    homing, PBL3771 stepper drivers fed by PBM3960 DACs.

    00000-0FFFF  firmware ROM
    20000-21FFF  RAM
    30000-3FFFF  font cassette
    50000        W  X coils (word: high byte coil A, low byte coil B;
                    bit 7 = polarity, bits 6-0 = current)
    60000        W  Y coils, same format
    70000        W  key matrix column (one bit per column)
                 R  key matrix rows, active high
    74000        W  step timer reload (8-bit up counter, IRQ2 on overflow)
    78000        W  IRQ2 acknowledge
    7C000        R  bits 0-2 DIP switches (inverted by the firmware,
                    selects the keyboard table of the cassette),
                    bit 6 X home sensor, bit 7 Y home sensor (0 = at home)
                 W  pen solenoid, pull-in stage
    7C001        W  pen solenoid, hold stage
    7C002        W  beeper
    7C004-7C007  W  misc outputs, 7C005 CODE LED, 7C006 LOCK LED

    bigraph scriber 1B90
    --------------------
    TMS9995 @ 12 MHz, TMS9901 PSI, TI gate array (CWS9037), two 27C512
    (CS 50 2.1 firmware, CS L1.2 font), battery backed static RAM,
    one line LCD, 72 keys, two TLC7528 dual DACs for the stepper drivers,
    28-pin memory/font cassette slot.

    The 64K firmware is one program in two 32K code banks. The cooperative
    task scheduler (shared code at 0000-15FF in both banks) switches banks
    through TMS9901 P5/P6.

    0000-7FFF  firmware ROM, A15 = 9901 P6 (P5 would be A16)
    7000-7FFF  LCD character RAM, visible while 9901 P8 is low
    8000-BFFF  16K window: P9=1 font ROM (A15=P14, A14=P15)
                           P9=0 cassette (A15=P12, A14=P13)
    C000-DFFF  battery backed RAM
    F000-F0FB  TMS9995 internal RAM
    FFD0       W  X DACs (word: high byte coil A, low byte coil B;
                  bits 7-1 current, bit 0 polarity)
    FFD2       W  Y DACs
    FFE0-FFE7  LCD controller (A1 = R/W, A2 = RS), HD44780 like commands,
               character patterns from the 4K RAM at 7000-7FFF
    FFF0       R  key code from the gate array
    FFFA-FFFF  TMS9995 internal decrementer / NMI vector

    TMS9901 (CRU base >FF00): P0/P1 pen pull/hold, P4 input "key held"
    (key repeat), P5/P6 code bank, P7 unknown (set together with the +A0
    character offset of the font preview), P8 low = character RAM visible,
    P9/P12-P15 window banking, P10 unknown (cleared before IDLE and on NMI).
    9901 INT4 = key strobe from the gate array, 9901 INTREQ -> CPU INT4
    (task 0x308 reads FFF0). The START/CHECK key also reaches CPU INT1
    (task 0x302 checks for code 0F). NMI = power fail.
    The scriber has no home sensors: it homes by driving into the end stops
    (Y to the top, X to the left).

    Usage
    - 1R90: insert a font cassette (-cart), without one every key press only
      beeps (error F00B). The scriber font CS L1.2 works as a cassette;
      press LOCK once, then type. Each character is drawn immediately.
    - 1B90: press START/CHECK after power on, type a line (it appears after
      "T/" on the LCD), then ENTER and START to draw it.
    - F5 puts a new sheet of paper in. 0.01 mm per micro step is assumed,
      the grid has 5 mm squares.

    TODO
    - 1R90: real key layout, function of 7C004/7C007, IRQ6/7 source
    - 1R90: no 1R90 cassette dumped yet; the scriber font has an identity
      keyboard table, so typed characters don't match the key caps
    - 1B90: verify keyboard/interrupt wiring, beeper, memory cassette
      (RAM), font preview (P7), position of the keys in row 5
    - both: real mechanical travel and step size, timer clock of the 1R90

***************************************************************************/

#include "emu.h"

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "cpu/m68000/m68008.h"
#include "cpu/tms9900/tms9995.h"
#include "machine/nvram.h"
#include "machine/tms9901.h"
#include "sound/spkrdev.h"

#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"

#include <cmath>

#define LOG_HW      (1U << 1)
#define LOG_UNKNOWN (1U << 2)
#define LOG_POS     (1U << 3)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

/***************************************************************************
    Common pen plotter mechanics
***************************************************************************/

class bigraph_base_state : public driver_device
{
public:
	bigraph_base_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_screen(*this, "screen")
	{ }

	DECLARE_INPUT_CHANGED_MEMBER(new_sheet);

protected:
	// one electrical cycle of the stepper = 4 full steps = 40 micro steps
	static constexpr double MICROSTEPS_PER_CYCLE = 40.0;
	static constexpr double PI = 3.14159265358979323846; // M_PI is missing on MinGW

	struct axis_t
	{
		double pos = 0.0;       // micro steps from home
		double angle = 0.0;     // last electrical angle
		bool valid = false;
		double min = 0.0;
		double max = 0.0;
	};

	virtual void machine_start() override ATTR_COLD;

	void configure_paper(double xmin, double xmax, double ymin, double ymax, int scale, double xstart, double ystart);
	void set_coils(int axis, int a, int b);
	void set_pen(bool down);
	bool at_home(int axis) const { return m_axis[axis].pos <= 0.5; }

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	required_device<screen_device> m_screen;

	axis_t m_axis[2];
	bool m_pen_down = false;
	u32 m_coil_writes = 0;
	int m_scale = 8;
	double m_xstart = 0.0, m_ystart = 0.0;
	bitmap_rgb32 m_paper;

private:
	void paper_point(double x, double y, int &px, int &py) const;
	void draw_line(double x0, double y0, double x1, double y1);
	void clear_paper();
};

void bigraph_base_state::configure_paper(double xmin, double xmax, double ymin, double ymax, int scale, double xstart, double ystart)
{
	m_axis[0].min = xmin;
	m_axis[0].max = xmax;
	m_axis[1].min = ymin;
	m_axis[1].max = ymax;
	m_scale = scale;
	m_xstart = xstart;
	m_ystart = ystart;
}

void bigraph_base_state::machine_start()
{
	const int w = int((m_axis[0].max - m_axis[0].min) / m_scale) + 1;
	const int h = int((m_axis[1].max - m_axis[1].min) / m_scale) + 1;
	m_paper.allocate(w, h);
	clear_paper();

	// the carriage is somewhere in the middle at power on
	m_axis[0].pos = m_xstart;
	m_axis[1].pos = m_ystart;

	save_item(NAME(m_axis[0].pos));
	save_item(NAME(m_axis[0].angle));
	save_item(NAME(m_axis[0].valid));
	save_item(NAME(m_axis[1].pos));
	save_item(NAME(m_axis[1].angle));
	save_item(NAME(m_axis[1].valid));
	save_item(NAME(m_pen_down));
	save_item(NAME(m_paper));
}

void bigraph_base_state::clear_paper()
{
	m_paper.fill(rgb_t(0xff, 0xfc, 0xf4));
	// faint millimetre-ish grid to judge sizes
	for (int x = 0; x < m_paper.width(); x += 50)
		for (int y = 0; y < m_paper.height(); y++)
			m_paper.pix(y, x) = rgb_t(0xe8, 0xe8, 0xf4);
	for (int y = m_paper.height() - 1; y >= 0; y -= 50)
		for (int x = 0; x < m_paper.width(); x++)
			m_paper.pix(y, x) = rgb_t(0xe8, 0xe8, 0xf4);
}

INPUT_CHANGED_MEMBER(bigraph_base_state::new_sheet)
{
	if (newval)
		clear_paper();
}

void bigraph_base_state::paper_point(double x, double y, int &px, int &py) const
{
	px = int(std::lround((x - m_axis[0].min) / m_scale));
	py = m_paper.height() - 1 - int(std::lround((y - m_axis[1].min) / m_scale));
}

void bigraph_base_state::draw_line(double x0, double y0, double x1, double y1)
{
	int px0, py0, px1, py1;
	paper_point(x0, y0, px0, py0);
	paper_point(x1, y1, px1, py1);

	const int dx = std::abs(px1 - px0), sx = (px0 < px1) ? 1 : -1;
	const int dy = -std::abs(py1 - py0), sy = (py0 < py1) ? 1 : -1;
	int err = dx + dy;
	const rgb_t ink(0x10, 0x10, 0x40);
	for (;;)
	{
		for (int oy = 0; oy < 2; oy++)
			for (int ox = 0; ox < 2; ox++)
			{
				const int x = px0 + ox, y = py0 + oy;
				if (x >= 0 && y >= 0 && x < m_paper.width() && y < m_paper.height())
					m_paper.pix(y, x) = ink;
			}
		if (px0 == px1 && py0 == py1)
			break;
		const int e2 = 2 * err;
		if (e2 >= dy) { err += dy; px0 += sx; }
		if (e2 <= dx) { err += dx; py0 += sy; }
	}
}

void bigraph_base_state::set_coils(int axis, int a, int b)
{
	axis_t &ax = m_axis[axis];
	if (!a && !b)
		return; // motor without current, keeps its position

	const double angle = std::atan2(double(b), double(a));
	if (ax.valid)
	{
		double d = angle - ax.angle;
		while (d > PI) d -= 2.0 * PI;
		while (d < -PI) d += 2.0 * PI;

		const double oldx = m_axis[0].pos, oldy = m_axis[1].pos;
		ax.pos = std::clamp(ax.pos + d * MICROSTEPS_PER_CYCLE / (2.0 * PI), ax.min, ax.max);
		if (m_pen_down)
			draw_line(oldx, oldy, m_axis[0].pos, m_axis[1].pos);
	}
	ax.angle = angle;
	ax.valid = true;
	if (!(++m_coil_writes % 500))
		LOGMASKED(LOG_POS, "position %.1f %.1f (pen %d)\n", m_axis[0].pos, m_axis[1].pos, m_pen_down);
}

void bigraph_base_state::set_pen(bool down)
{
	if (down != m_pen_down)
		LOGMASKED(LOG_POS, "pen %s at %.1f %.1f\n", down ? "down" : "up", m_axis[0].pos, m_axis[1].pos);
	if (down && !m_pen_down)
		draw_line(m_axis[0].pos, m_axis[1].pos, m_axis[0].pos, m_axis[1].pos);
	m_pen_down = down;
}

u32 bigraph_base_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	copybitmap(bitmap, m_paper, 0, 0, 0, 0, cliprect);

	// pen position: red when down, blue when up
	int px, py;
	paper_point(m_axis[0].pos, m_axis[1].pos, px, py);
	const rgb_t col = m_pen_down ? rgb_t(0xe0, 0x00, 0x00) : rgb_t(0x00, 0x40, 0xe0);
	for (int i = -8; i <= 8; i++)
	{
		if (cliprect.contains(px + i, py))
			bitmap.pix(py, px + i) = col;
		if (cliprect.contains(px, py + i))
			bitmap.pix(py + i, px) = col;
	}
	return 0;
}


/***************************************************************************
    bigraph script 1R90
***************************************************************************/

class bgscript_state : public bigraph_base_state
{
public:
	bgscript_state(const machine_config &mconfig, device_type type, const char *tag) :
		bigraph_base_state(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_cart(*this, "cartslot"),
		m_speaker(*this, "speaker"),
		m_keys(*this, "COL%u", 0U),
		m_dsw(*this, "DSW"),
		m_leds(*this, "led%u", 0U)
	{ }

	void bgscript(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	TIMER_CALLBACK_MEMBER(step_timer);

	template <int Axis> void motor_w(offs_t offset, u8 data);
	u8 keyboard_r();
	void keyboard_w(u8 data);
	void timer_reload_w(u8 data);
	void irq_ack_w(u8 data);
	u8 status_r();
	void outputs_w(offs_t offset, u8 data);

	static int coil_1r90(u8 data) { return (data & 0x80) ? (data & 0x7f) : -(data & 0x7f); }

	required_device<m68008_device> m_maincpu;
	required_device<generic_slot_device> m_cart;
	required_device<speaker_sound_device> m_speaker;
	required_ioport_array<8> m_keys;
	required_ioport m_dsw;
	output_finder<2> m_leds;

	emu_timer *m_step_timer = nullptr;
	u8 m_coils[4] = { };
	u8 m_column = 0;
	u8 m_reload = 0xe0;
	u8 m_outputs[8] = { };
};

static constexpr XTAL BGSCRIPT_CLOCK = 8_MHz_XTAL;
// clock of the step timer counter, a guess giving ~4 kHz idle tick rate
static constexpr u32 BGSCRIPT_TIMER_CLOCK = 8'000'000 / 64;

void bgscript_state::machine_start()
{
	configure_paper(-400.0, 20480.0, -400.0, 5760.0, 10, 4000.0, 2000.0);
	bigraph_base_state::machine_start();

	m_step_timer = timer_alloc(FUNC(bgscript_state::step_timer), this);

	save_item(NAME(m_coils));
	save_item(NAME(m_column));
	save_item(NAME(m_reload));
	save_item(NAME(m_outputs));
}

void bgscript_state::machine_reset()
{
	m_reload = 0xe0;
	std::fill(std::begin(m_outputs), std::end(m_outputs), 0);
	m_step_timer->adjust(attotime::from_ticks(256 - m_reload, BGSCRIPT_TIMER_CLOCK));
	m_maincpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
	set_pen(false);
}

TIMER_CALLBACK_MEMBER(bgscript_state::step_timer)
{
	m_maincpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	m_step_timer->adjust(attotime::from_ticks(256 - m_reload, BGSCRIPT_TIMER_CLOCK));
}

void bgscript_state::timer_reload_w(u8 data)
{
	m_reload = data;
}

void bgscript_state::irq_ack_w(u8 data)
{
	m_maincpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
}

template <int Axis>
void bgscript_state::motor_w(offs_t offset, u8 data)
{
	// 8-bit bus: word writes arrive as high byte (coil A) then low byte (coil B)
	if (offset >= 2)
	{
		LOGMASKED(LOG_HW, "%s: motor %d register %d = %02x\n", machine().describe_context(), Axis, offset, data);
		return;
	}
	m_coils[Axis * 2 + offset] = data;
	if (offset == 1)
		set_coils(Axis, coil_1r90(m_coils[Axis * 2]), coil_1r90(m_coils[Axis * 2 + 1]));
}

void bgscript_state::keyboard_w(u8 data)
{
	m_column = data;
}

u8 bgscript_state::keyboard_r()
{
	u8 data = 0;
	for (int i = 0; i < 8; i++)
		if (BIT(m_column, 7 - i))
			data |= m_keys[i]->read();
	if (data && !machine().side_effects_disabled())
		LOGMASKED(LOG_HW, "keyboard col %02x -> %02x\n", m_column, data);
	return data;
}

u8 bgscript_state::status_r()
{
	u8 data = m_dsw->read() & 0x3f;
	if (!at_home(0))
		data |= 0x40;
	if (!at_home(1))
		data |= 0x80;
	return data;
}

void bgscript_state::outputs_w(offs_t offset, u8 data)
{
	const bool state = BIT(data, 0);
	if (m_outputs[offset] != state)
		LOGMASKED(LOG_HW, "%s: output 7C00%d = %d\n", machine().describe_context(), offset, state);
	m_outputs[offset] = state;

	switch (offset)
	{
	case 0: // pen pull-in
	case 1: // pen hold
		set_pen(m_outputs[0] || m_outputs[1]);
		break;
	case 2:
		m_speaker->level_w(state);
		break;
	case 5:
	case 6:
		m_leds[offset - 5] = state;
		break;
	default:
		break;
	}
}

void bgscript_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x00000, 0x0ffff).rom().region("maincpu", 0);
	map(0x20000, 0x21fff).mirror(0x0e000).ram();
	map(0x30000, 0x3ffff).r(m_cart, FUNC(generic_slot_device::read_rom));
	map(0x50000, 0x50003).mirror(0x0fffc).w(FUNC(bgscript_state::motor_w<0>));
	map(0x60000, 0x60003).mirror(0x0fffc).w(FUNC(bgscript_state::motor_w<1>));
	map(0x70000, 0x70000).mirror(0x03fff).rw(FUNC(bgscript_state::keyboard_r), FUNC(bgscript_state::keyboard_w));
	map(0x74000, 0x74000).mirror(0x03fff).w(FUNC(bgscript_state::timer_reload_w));
	map(0x78000, 0x78000).mirror(0x03fff).w(FUNC(bgscript_state::irq_ack_w));
	map(0x7c000, 0x7c000).mirror(0x03ff8).r(FUNC(bgscript_state::status_r));
	map(0x7c000, 0x7c007).mirror(0x03ff8).w(FUNC(bgscript_state::outputs_w));
}

/*
    Key matrix: code = column * 8 + row (column bit 7 = column 0, row bit 7 = row 0).
    The firmware translates the code through a table and then through the
    keyboard table (IK) of the cassette, so the printed character depends on
    the cassette. The labels below follow the firmware's own index order
    (which is ASCII-like) and are not verified against a real keyboard.
    Codes 60-62 are the modifiers: 60 LOCK (toggle, table +0x40), 61 SHIFT
    (held, table +0x40), 62 CODE (toggle, table +0x80). With the scriber
    font (identity IK table) only the +0x40 layer gives valid glyphs, so
    press LOCK once before typing.
*/
#define BGKEY(bit, name, key) \
	PORT_BIT(1U << (7 - (bit)), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) PORT_CODE(key)
#define BGKEY_NONE(bit) \
	PORT_BIT(1U << (7 - (bit)), IP_ACTIVE_HIGH, IPT_UNUSED)

static INPUT_PORTS_START( bgscript )
	PORT_START("COL0") // codes 0-7
	BGKEY_NONE(0)
	BGKEY(1, "Q", KEYCODE_Q)
	BGKEY(2, "T", KEYCODE_T)
	BGKEY_NONE(3)
	BGKEY(4, "N", KEYCODE_N)
	BGKEY(5, "M", KEYCODE_M)
	BGKEY(6, "L", KEYCODE_L)
	BGKEY(7, "K", KEYCODE_K)

	PORT_START("COL1") // codes 8-15
	BGKEY_NONE(0)
	BGKEY(1, "E", KEYCODE_E)
	BGKEY(2, "G", KEYCODE_G)
	BGKEY_NONE(3)
	BGKEY(4, "R", KEYCODE_R)
	BGKEY(5, "C", KEYCODE_C)
	BGKEY(6, "A", KEYCODE_A)
	BGKEY(7, "?", KEYCODE_SLASH)

	PORT_START("COL2") // codes 16-23
	BGKEY_NONE(0)
	BGKEY(1, "D", KEYCODE_D)
	BGKEY(2, "F", KEYCODE_F)
	BGKEY(3, "H", KEYCODE_H)
	BGKEY(4, "S", KEYCODE_S)
	BGKEY(5, "B", KEYCODE_B)
	BGKEY(6, "@", KEYCODE_OPENBRACE)
	BGKEY(7, ">", KEYCODE_CLOSEBRACE)

	PORT_START("COL3") // codes 24-31
	BGKEY_NONE(0)
	BGKEY(1, "7", KEYCODE_7)
	BGKEY(2, "9", KEYCODE_9)
	BGKEY(3, ";", KEYCODE_COLON)
	BGKEY(4, "P", KEYCODE_P)
	BGKEY(5, "5", KEYCODE_5)
	BGKEY(6, "3", KEYCODE_3)
	BGKEY(7, "1", KEYCODE_1)

	PORT_START("COL4") // codes 32-39
	BGKEY_NONE(0)
	BGKEY(1, "6", KEYCODE_6)
	BGKEY(2, "8", KEYCODE_8)
	BGKEY(3, ":", KEYCODE_QUOTE)
	BGKEY(4, "O", KEYCODE_O)
	BGKEY(5, "4", KEYCODE_4)
	BGKEY(6, "2", KEYCODE_2)
	BGKEY(7, "0", KEYCODE_0)

	PORT_START("COL5") // codes 40-47
	BGKEY(0, "-", KEYCODE_MINUS)
	BGKEY(1, "'", KEYCODE_BACKSLASH)
	BGKEY(2, ")", KEYCODE_I)
	BGKEY(3, "+", KEYCODE_EQUALS)
	BGKEY_NONE(4)
	BGKEY(5, "%", KEYCODE_U)
	BGKEY(6, "#", KEYCODE_Y)
	BGKEY(7, "!", KEYCODE_Z)

	PORT_START("COL6") // codes 48-55
	BGKEY(0, ",", KEYCODE_COMMA)
	BGKEY(1, "&", KEYCODE_V)
	BGKEY(2, "(", KEYCODE_J)
	BGKEY(3, "*", KEYCODE_X)
	BGKEY_NONE(4)
	BGKEY(5, "$", KEYCODE_TILDE)
	BGKEY(6, "\"", KEYCODE_W)
	BGKEY(7, "Space", KEYCODE_SPACE)

	PORT_START("COL7") // codes 56-63
	BGKEY(0, ".", KEYCODE_STOP)
	BGKEY(1, "Code 57", KEYCODE_BACKSLASH2)
	BGKEY(2, "Enter", KEYCODE_ENTER)
	BGKEY(3, "Backspace", KEYCODE_BACKSPACE)
	BGKEY(4, "Lock", KEYCODE_CAPSLOCK)
	BGKEY(5, "Shift", KEYCODE_LSHIFT)
	BGKEY(6, "Code", KEYCODE_LCONTROL)
	BGKEY(7, "Tab", KEYCODE_TAB)

	PORT_START("DSW")
	PORT_DIPNAME( 0x07, 0x07, "Keyboard table" ) PORT_DIPLOCATION("SW1:1,2,3")
	PORT_DIPSETTING(    0x07, "0" )
	PORT_DIPSETTING(    0x06, "1" )
	PORT_DIPSETTING(    0x05, "2" )
	PORT_DIPSETTING(    0x04, "3" )
	PORT_DIPSETTING(    0x03, "4" )
	PORT_DIPSETTING(    0x02, "5" )
	PORT_DIPSETTING(    0x01, "6" )
	PORT_DIPSETTING(    0x00, "7" )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) ) PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT( 0xf0, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("PAPER")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("New Sheet") PORT_CODE(KEYCODE_F5) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bigraph_base_state::new_sheet), 0)
INPUT_PORTS_END

void bgscript_state::bgscript(machine_config &config)
{
	M68008(config, m_maincpu, BGSCRIPT_CLOCK);
	m_maincpu->set_addrmap(AS_PROGRAM, &bgscript_state::mem_map);

	GENERIC_CARTSLOT(config, m_cart, generic_plain_slot, "bigraph_cass", "bin,rom");

	SCREEN(config, m_screen);
	m_screen->set_raw(2090 * 618 * 25, 2090, 0, 2090, 618, 0, 618); // 25 Hz, no real video timing
	m_screen->set_screen_update(FUNC(bgscript_state::screen_update));

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_speaker).add_route(ALL_OUTPUTS, "mono", 0.50);
}


/***************************************************************************
    bigraph scriber 1B90
***************************************************************************/

class bgscribe_state : public bigraph_base_state
{
public:
	bgscribe_state(const machine_config &mconfig, device_type type, const char *tag) :
		bigraph_base_state(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_psi(*this, "psi"),
		m_cart(*this, "cartslot"),
		m_firmware(*this, "firmware"),
		m_font(*this, "font"),
		m_keys(*this, "KEY%u", 0U)
	{ }

	void bgscribe(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(key_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;
	void cru_map(address_map &map) ATTR_COLD;

	u8 rom_r(offs_t offset);
	void rom_w(offs_t offset, u8 data);
	u8 window_r(offs_t offset);
	void window_w(offs_t offset, u8 data);
	void dac_w(offs_t offset, u8 data);
	u8 lcd_r(offs_t offset);
	void lcd_w(offs_t offset, u8 data);
	u8 key_r();
	u8 io_unk_r(offs_t offset);
	void io_unk_w(offs_t offset, u8 data);

	u8 cru_r(offs_t offset);
	void cru_w(offs_t offset, u8 data);
	u8 psi_input_r(offs_t offset);
	bool any_key_down();
	void psi_int_w(int state);
	template <int P> void psi_p_w(int state);

	u32 lcd_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	static int coil_1b90(u8 data) { return (data & 0x01) ? (data >> 1) : -(data >> 1); }

	required_device<tms9995_device> m_maincpu;
	required_device<tms9901_device> m_psi;
	required_device<generic_slot_device> m_cart;
	required_region_ptr<u8> m_firmware;
	required_region_ptr<u8> m_font;
	required_ioport_array<9> m_keys;

	std::unique_ptr<u8[]> m_cgram;  // 7000-7FFF, character patterns of the LCD
	u16 m_p = 0xffff;          // TMS9901 P outputs
	u8 m_dac[4] = { };
	u8 m_keycode = 0xff;

	// LCD controller state
	static constexpr int LCD_CHARS = 40;
	u8 m_ddram[0x80] = { };
	u8 m_lcd_ac = 0;
	bool m_lcd_cursor = false;
};

void bgscribe_state::machine_start()
{
	configure_paper(-400.0, 20480.0, -400.0, 5760.0, 10, 4000.0, 2000.0);
	bigraph_base_state::machine_start();

	m_cgram = std::make_unique<u8[]>(0x1000);

	// The character RAM is part of the battery backed RAM, so on a real
	// machine it keeps the patterns loaded earlier. Start with the DC block
	// of the internal font, as the firmware would load it.
	for (offs_t i = 0; i + 6 < 0x400; i++)
	{
		if (!memcmp(&m_font[i], "DC00FF", 6))
		{
			for (int c = 0; c < 0xfd; c++)
				for (int row = 0; row < 12; row++)
					m_cgram[(c << 4) | row] = m_font[i + 6 + c * 12 + row];
			break;
		}
	}

	save_pointer(NAME(m_cgram), 0x1000);
	save_item(NAME(m_ddram));
	save_item(NAME(m_lcd_ac));
	save_item(NAME(m_lcd_cursor));
	save_item(NAME(m_p));
	save_item(NAME(m_dac));
	save_item(NAME(m_keycode));
}

void bgscribe_state::machine_reset()
{
	// 9901 pins are inputs after reset, assume pull-downs on the bank lines
	m_p = 0x0000;
	m_keycode = 0xff;
	set_pen(false);
	m_maincpu->reset_line(ASSERT_LINE);
}

// ---------------- memory ----------------

u8 bgscribe_state::rom_r(offs_t offset)
{
	if (offset >= 0x7000 && !BIT(m_p, 8))
		return m_cgram[offset & 0xfff];
	const offs_t bank = BIT(m_p, 6) ? 0x8000 : 0;
	return m_firmware[bank | offset];
}

void bgscribe_state::rom_w(offs_t offset, u8 data)
{
	if (offset >= 0x7000 && !BIT(m_p, 8))
	{
		if (offset >= 0x7fd0 || !(offset & 0xff))
			LOGMASKED(LOG_HW, "%s: CG RAM %04x = %02x\n", machine().describe_context(), offset, data);
		m_cgram[offset & 0xfff] = data;
	}
	else if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%s: write to ROM %04x = %02x (P=%04x)\n", machine().describe_context(), offset, data, m_p);
}

u8 bgscribe_state::window_r(offs_t offset)
{
	if (BIT(m_p, 9))
	{
		// font ROM on the main board: four 16K pages, one font each
		const offs_t page = (BIT(m_p, 14) << 1) | BIT(m_p, 15);
		return m_font[(page << 14) | offset];
	}
	// cassette
	const offs_t page = (BIT(m_p, 12) << 1) | BIT(m_p, 13);
	return m_cart->read_rom((page << 14) | offset);
}

void bgscribe_state::window_w(offs_t offset, u8 data)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%s: write to window %04x = %02x (P=%04x)\n", machine().describe_context(), offset + 0x8000, data, m_p);
}

void bgscribe_state::dac_w(offs_t offset, u8 data)
{
	m_dac[offset] = data;
	// the TMS9995 writes words as two byte accesses, high byte first
	if (offset & 1)
	{
		const int axis = offset >> 1;
		set_coils(axis, coil_1b90(m_dac[axis * 2]), coil_1b90(m_dac[axis * 2 + 1]));
	}
}

/*
    The LCD controller uses an HD44780 like command set, but takes its
    character patterns from the external 4K RAM at 7000-7FFF: 16 bytes per
    character, one byte per dot row, bits 4-0 = dots. The firmware fills it
    from the DC block of the font ROM (12 rows used). Characters FD-FF are
    used for annunciators; their patterns are written at 7FD0/7FE0/7FF0.
    Only the commands used by the firmware are handled.
*/
u8 bgscribe_state::lcd_r(offs_t offset)
{
	// offset 1: status read (busy flag in bit 7, never busy here)
	if (offset == 1)
		return m_lcd_ac & 0x7f;
	if (offset == 3)
		return m_ddram[m_lcd_ac & 0x7f];
	return 0xff;
}

void bgscribe_state::lcd_w(offs_t offset, u8 data)
{
	if (offset == 0)
	{
		if (data & 0x80)
			m_lcd_ac = data & 0x7f;
		else if (data == 0x01)
		{
			std::fill(std::begin(m_ddram), std::end(m_ddram), 0x20);
			m_lcd_ac = 0;
		}
		else if (data == 0x04 || data == 0x05)
			m_lcd_cursor = BIT(data, 0);
		LOGMASKED(LOG_HW, "%s: LCD command %02x\n", machine().describe_context(), data);
	}
	else if (offset == 2)
	{
		LOGMASKED(LOG_HW, "%s: LCD data %02x at %02x\n", machine().describe_context(), data, m_lcd_ac);
		m_ddram[m_lcd_ac & 0x7f] = data;
		m_lcd_ac = (m_lcd_ac + 1) & 0x7f;
	}
}

u32 bgscribe_state::lcd_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const rgb_t bg(0xa8, 0xb4, 0x98), off(0x9c, 0xa8, 0x8c), on(0x20, 0x28, 0x20);
	bitmap.fill(bg, cliprect);
	for (int pos = 0; pos < LCD_CHARS; pos++)
	{
		const u8 c = m_ddram[pos];
		for (int row = 0; row < 16; row++)
		{
			u8 dots = m_cgram[(c << 4) | row];
			if (m_lcd_cursor && pos == m_lcd_ac && row == 11)
				dots = 0x1f;
			for (int col = 0; col < 5; col++)
			{
				const int x = 1 + pos * 6 + col, y = 1 + row;
				if (cliprect.contains(x, y))
					bitmap.pix(y, x) = BIT(dots, 4 - col) ? on : off;
			}
		}
	}
	return 0;
}

u8 bgscribe_state::key_r()
{
	// reading the key code acknowledges the keyboard interrupt
	if (!machine().side_effects_disabled())
	{
		LOGMASKED(LOG_HW, "%s: key code read %02x\n", machine().describe_context(), m_keycode);
		m_maincpu->set_input_line(INT_9995_INT1, CLEAR_LINE);
		m_psi->set_int_line(4, CLEAR_LINE);
	}
	return m_keycode;
}

u8 bgscribe_state::io_unk_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%s: read from I/O %04x\n", machine().describe_context(), offset + 0xffc0);
	return 0xff;
}

void bgscribe_state::io_unk_w(offs_t offset, u8 data)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%s: write to I/O %04x = %02x\n", machine().describe_context(), offset + 0xffc0, data);
}

void bgscribe_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x7fff).rw(FUNC(bgscribe_state::rom_r), FUNC(bgscribe_state::rom_w));
	map(0x8000, 0xbfff).rw(FUNC(bgscribe_state::window_r), FUNC(bgscribe_state::window_w));
	map(0xc000, 0xdfff).ram().share("nvram");
	map(0xffc0, 0xffef).rw(FUNC(bgscribe_state::io_unk_r), FUNC(bgscribe_state::io_unk_w));
	map(0xffd0, 0xffd3).w(FUNC(bgscribe_state::dac_w));
	map(0xffe0, 0xffe7).lrw8(
			NAME([this] (offs_t offset) { return lcd_r(offset >> 1); }),
			NAME([this] (offs_t offset, u8 data) { lcd_w(offset >> 1, data); }));
	map(0xfff0, 0xfff0).r(FUNC(bgscribe_state::key_r));
}

// ---------------- CRU / TMS9901 ----------------

u8 bgscribe_state::cru_r(offs_t offset)
{
	// offset = CRU bit address (R12 / 2 + bit)
	if ((offset & 0x7fe0) == 0x7f80)
		return m_psi->read(offset & 0x1f);
	return 1;
}

void bgscribe_state::cru_w(offs_t offset, u8 data)
{
	if ((offset & 0x7fe0) == 0x7f80)
		m_psi->write(offset & 0x1f, data);
	else if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%s: CRU write %04x = %d\n", machine().describe_context(), offset << 1, data);
}

void bgscribe_state::cru_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(bgscribe_state::cru_r), FUNC(bgscribe_state::cru_w));
}

u8 bgscribe_state::psi_input_r(offs_t offset)
{
	// offset: 1-15 = INTn pin, 16-22 = P0-P6 (P7-P15 come in as 15-7)
	// INT3 and P4 are low while a key is held (P4 is polled for key repeat)
	switch (offset)
	{
	case 3:
	case 20:
		return any_key_down() ? 0 : 1;
	default:
		return 1;
	}
}

bool bgscribe_state::any_key_down()
{
	for (auto &port : m_keys)
		if (port->read())
			return true;
	return false;
}

void bgscribe_state::psi_int_w(int state)
{
	m_maincpu->set_input_line(INT_9995_INT4, state ? ASSERT_LINE : CLEAR_LINE);
}

template <int P>
void bgscribe_state::psi_p_w(int state)
{
	const u16 old = m_p;
	m_p = (m_p & ~(1U << P)) | ((state ? 1U : 0U) << P);
	if (old == m_p)
		return;

	switch (P)
	{
	case 0: // pen pull-in
	case 1: // pen hold
		set_pen(BIT(m_p, 0) || BIT(m_p, 1));
		break;
	case 5:
	case 6:
	case 8:
	case 9:
	case 12:
	case 13:
	case 14:
	case 15:
		break;
	default:
		LOGMASKED(LOG_HW, "%s: 9901 P%d = %d\n", machine().describe_context(), P, state);
		break;
	}
}

// ---------------- keyboard ----------------

INPUT_CHANGED_MEMBER(bgscribe_state::key_changed)
{
	// Guess, consistent with the firmware: the gate array latches the code
	// and pulls 9901 /INT4 (9901 INTREQ -> CPU INT4, task 0x308 reads FFF0).
	// START/CHECK additionally goes to CPU INT1 (task 0x302 checks for code
	// 0F to start or stop plotting). Reading FFF0 clears both.
	// "Any key held" is readable on 9901 INT3 and P4 (polled only).
	if (newval)
	{
		m_keycode = param;
		if (param == 0x0f)
			m_maincpu->set_input_line(INT_9995_INT1, ASSERT_LINE);
		else
			m_psi->set_int_line(4, ASSERT_LINE);
	}
}

/*
    72 keys. The code numbers are the gate array key codes as used by the
    firmware tables at 0DBA/121C; the assignment to positions follows the
    order of those tables and the key caps of the machine (rows top to
    bottom). Row 5 is partly guessed.
*/
#define SCKEY(code, name, key) \
	PORT_BIT(1U << ((code) & 7), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) PORT_CODE(key) \
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bgscribe_state::key_changed), code)

static INPUT_PORTS_START( bgscribe )
	PORT_START("KEY0")
	SCKEY( 0, "Return Carriage", KEYCODE_HOME)
	SCKEY( 1, "1 !", KEYCODE_1)
	SCKEY( 2, "2 \"", KEYCODE_2)
	SCKEY( 3, "3 ,", KEYCODE_3)
	SCKEY( 4, "4 \xc2\xb0", KEYCODE_4)
	SCKEY( 5, "5 %", KEYCODE_5)
	SCKEY( 6, "6 &", KEYCODE_6)
	SCKEY( 7, "7 /", KEYCODE_7)

	PORT_START("KEY1")
	SCKEY( 8, "8 (", KEYCODE_8)
	SCKEY( 9, "9 )", KEYCODE_9)
	SCKEY(10, "0 =", KEYCODE_0)
	SCKEY(11, "\xc3\x9f ?", KEYCODE_MINUS)
	SCKEY(12, "\xc2\xb1 \xc3\x97", KEYCODE_EQUALS)
	SCKEY(13, "Left Small", KEYCODE_PGUP)
	SCKEY(14, "Right Small", KEYCODE_PGDN)
	SCKEY(15, "Start / Check", KEYCODE_F1)

	PORT_START("KEY2")
	SCKEY(16, "Q", KEYCODE_Q)
	SCKEY(17, "W", KEYCODE_W)
	SCKEY(18, "E", KEYCODE_E)
	SCKEY(19, "R", KEYCODE_R)
	SCKEY(20, "T", KEYCODE_T)
	SCKEY(21, "Z", KEYCODE_Y)
	SCKEY(22, "U", KEYCODE_U)
	SCKEY(23, "I", KEYCODE_I)

	PORT_START("KEY3")
	SCKEY(24, "O", KEYCODE_O)
	SCKEY(25, "P", KEYCODE_P)
	SCKEY(26, "\xc3\x9c", KEYCODE_OPENBRACE)
	SCKEY(27, "- \xc3\x98", KEYCODE_CLOSEBRACE)
	SCKEY(28, "+ *", KEYCODE_BACKSLASH)
	SCKEY(29, "Funct 3", KEYCODE_LALT)
	SCKEY(30, "A", KEYCODE_A)
	SCKEY(31, "S", KEYCODE_S)

	PORT_START("KEY4")
	SCKEY(32, "D", KEYCODE_D)
	SCKEY(33, "F", KEYCODE_F)
	SCKEY(34, "G", KEYCODE_G)
	SCKEY(35, "H", KEYCODE_H)
	SCKEY(36, "J", KEYCODE_J)
	SCKEY(37, "K", KEYCODE_K)
	SCKEY(38, "L", KEYCODE_L)
	SCKEY(39, "\xc3\x96", KEYCODE_COLON)

	PORT_START("KEY5")
	SCKEY(40, "\xc3\x84", KEYCODE_QUOTE)
	SCKEY(41, "Up", KEYCODE_UP)
	SCKEY(42, "Return (yellow)", KEYCODE_INSERT)
	SCKEY(43, "Enter", KEYCODE_ENTER)
	SCKEY(44, "Funct 2 (left)", KEYCODE_LSHIFT)
	SCKEY(45, "Funct 2", KEYCODE_LCONTROL)
	SCKEY(46, "Y", KEYCODE_Z)
	SCKEY(47, "X", KEYCODE_X)

	PORT_START("KEY6")
	SCKEY(48, "C", KEYCODE_C)
	SCKEY(49, "V", KEYCODE_V)
	SCKEY(50, "B", KEYCODE_B)
	SCKEY(51, "N", KEYCODE_N)
	SCKEY(52, "M", KEYCODE_M)
	SCKEY(53, "; ,", KEYCODE_COMMA)
	SCKEY(54, ": .", KEYCODE_STOP)
	SCKEY(55, "Left", KEYCODE_LEFT)

	PORT_START("KEY7")
	SCKEY(56, "Right", KEYCODE_RIGHT)
	SCKEY(57, "Funct 2 (right)", KEYCODE_RSHIFT)
	SCKEY(58, "Cass", KEYCODE_F2)
	SCKEY(59, "Tol", KEYCODE_F3)
	SCKEY(60, "Tab", KEYCODE_TAB)
	SCKEY(61, "Stop", KEYCODE_ESC)
	SCKEY(62, "Ins / Up", KEYCODE_F4)
	SCKEY(63, "Del", KEYCODE_DEL)

	PORT_START("KEY8")
	SCKEY(64, "Param / IP", KEYCODE_F9)
	SCKEY(65, "UL", KEYCODE_F10)
	SCKEY(66, "Space", KEYCODE_SPACE)
	SCKEY(67, "BS", KEYCODE_BACKSPACE)
	SCKEY(68, "M?", KEYCODE_F6)
	SCKEY(69, "RM / M", KEYCODE_F7)
	SCKEY(70, "Run", KEYCODE_F8)
	SCKEY(71, "Down", KEYCODE_DOWN)

	PORT_START("PAPER")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("New Sheet") PORT_CODE(KEYCODE_F5) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(bigraph_base_state::new_sheet), 0)
INPUT_PORTS_END

void bgscribe_state::bgscribe(machine_config &config)
{
	TMS9995(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &bgscribe_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &bgscribe_state::cru_map);

	TMS9901(config, m_psi, 12_MHz_XTAL / 4);
	m_psi->read_cb().set(FUNC(bgscribe_state::psi_input_r));
	m_psi->intreq_cb().set(FUNC(bgscribe_state::psi_int_w));
	m_psi->p_out_cb(0).set(FUNC(bgscribe_state::psi_p_w<0>));
	m_psi->p_out_cb(1).set(FUNC(bgscribe_state::psi_p_w<1>));
	m_psi->p_out_cb(2).set(FUNC(bgscribe_state::psi_p_w<2>));
	m_psi->p_out_cb(3).set(FUNC(bgscribe_state::psi_p_w<3>));
	m_psi->p_out_cb(4).set(FUNC(bgscribe_state::psi_p_w<4>));
	m_psi->p_out_cb(5).set(FUNC(bgscribe_state::psi_p_w<5>));
	m_psi->p_out_cb(6).set(FUNC(bgscribe_state::psi_p_w<6>));
	m_psi->p_out_cb(7).set(FUNC(bgscribe_state::psi_p_w<7>));
	m_psi->p_out_cb(8).set(FUNC(bgscribe_state::psi_p_w<8>));
	m_psi->p_out_cb(9).set(FUNC(bgscribe_state::psi_p_w<9>));
	m_psi->p_out_cb(10).set(FUNC(bgscribe_state::psi_p_w<10>));
	m_psi->p_out_cb(11).set(FUNC(bgscribe_state::psi_p_w<11>));
	m_psi->p_out_cb(12).set(FUNC(bgscribe_state::psi_p_w<12>));
	m_psi->p_out_cb(13).set(FUNC(bgscribe_state::psi_p_w<13>));
	m_psi->p_out_cb(14).set(FUNC(bgscribe_state::psi_p_w<14>));
	m_psi->p_out_cb(15).set(FUNC(bgscribe_state::psi_p_w<15>));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	GENERIC_CARTSLOT(config, m_cart, generic_plain_slot, "bigraph_cass", "bin,rom");

	// plotter paper
	SCREEN(config, m_screen);
	m_screen->set_raw(2090 * 618 * 25, 2090, 0, 2090, 618, 0, 618); // 25 Hz, no real video timing
	m_screen->set_screen_update(FUNC(bgscribe_state::screen_update));

	// LCD
	screen_device &lcd(SCREEN(config, "lcd"));
	lcd.set_raw((LCD_CHARS * 6 + 1) * 18 * 50, LCD_CHARS * 6 + 1, 0, LCD_CHARS * 6 + 1, 18, 0, 18);
	lcd.set_screen_update(FUNC(bgscribe_state::lcd_update));
}


/***************************************************************************
    ROM definitions
***************************************************************************/

ROM_START( bgscript )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "rs1.5.bin", 0x00000, 0x10000, CRC(4f9112ea) SHA1(3e63f89e0e76ac2024cfa0d63079f161a9b71548) )
ROM_END

ROM_START( bgscribe )
	ROM_REGION( 0x10000, "firmware", 0 )
	ROM_LOAD( "cs_50_2.1.bin", 0x00000, 0x10000, CRC(2c632f28) SHA1(04ca55e023510c7302c3249d8d1f32c2e7b14c20) )

	ROM_REGION( 0x10000, "font", 0 )
	ROM_LOAD( "cs_l1.2.bin", 0x00000, 0x10000, CRC(4d6776b8) SHA1(e9f9448d6d43f73e62b32a6150608ee9284fd1aa) )
ROM_END

} // anonymous namespace


/*    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     CLASS           INIT        COMPANY                 FULLNAME                      FLAGS */
COMP( 1990, bgscript, 0,      0,      bgscript, bgscript, bgscript_state, empty_init, "rotring-werke Riepe", "bigraph script 1R90",        MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 1991, bgscribe, 0,      0,      bgscribe, bgscribe, bgscribe_state, empty_init, "rotring-werke Riepe", "bigraph scriber 1B90",       MACHINE_NOT_WORKING | MACHINE_NO_SOUND | MACHINE_SUPPORTS_SAVE )
