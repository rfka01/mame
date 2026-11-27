// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Siemens PT88 S ink jet printer (also sold as Siemens 9001 and
    Rohde & Schwarz PUD 3)

    Boards:
    L203-A1 "Druckersteuerung PT88S": two uPD78C10 (no markings), each with
        an 11.0592 MHz crystal; S22761-K101-H5 (2764) for the mechanism CPU
        next to the lower one, SRM2064C (8 KiB RAM), M5L8251AP-5 USART,
        74HC373 latches, LM339, two 8 way DIP switches, piezo buzzer, a
        driver hybrid for the head.
    L310-A2 "SPEAF": 24 KiB RAM (3x M5M5165P-10L), S22761-K310-H3 (27256)
        and S22761-K310-H4 (27512) for the main CPU.
    SAP-P11: passive Centronics module (74LS132).
    Head: TDK 88S, 9 nozzles, drop on demand with piezo tubes, nozzle cover.

    ROM images from mx300i.de.

    Main CPU memory, from the firmware:
      0000-7FFF  H3
      8000-BFFF  read: H4, 16 KiB bank chosen by latch A000h bits 0-1,
                 or RAM when bit 2 is set; write: RAM (assumed)
      C000-FFFF  RAM (the firmware clears MM, so the internal RAM is off)
    The firmware copies 8000-8EFF from H4 into the RAM below at start-up
    and runs it from there. Bank 0-3 each hold the same code at the start
    followed by character generators.
    With port C bit 6 set the writes and reads go to I/O instead:
      8000       latch (copy at FD7Ah, 53h at power on): bits 2-4 drive the
                 panel lamps, bits 0-1 are set from the FT0 handler
      A000       latch (copy at FD7Bh): H4 bank or RAM, see above
      E000/E001  8251 data/control, the host interface
    Main CPU ports: PA reads the interface module jumpers once (PC5 low),
    PB0-PB4 are outputs, PB5-PB7 inputs; AN0-AN3 (switches, read at power
    on) and AN4-AN7 (keys, scanned) are used with a 80h threshold.

    The two CPUs talk over their serial ports at CLK/24 (8 data bits, even
    parity, 2 stop bits); the main CPU takes the mechanism CPU's clock.
    Commands to the mechanism CPU: 01 self test and carriage homing, 02
    homing, 03/04 paper feed (each further 03/04 byte two half steps, 05
    ends), 06-09 print a line (16 bit travel in encoder steps, then column
    pairs), 05 none.

    Mechanism CPU:
      1000h      carriage motor drive (write only)
      PA         nozzles 1-8 (low fires) or, with PB3 low, the paper
                 stepper phases on PA0-PA3 (half steps)
      PB0        nozzle 9 (low fires), PB1 fire strobe (low), PB3 see PA,
                 PB4/PB5 analogue multiplexer, PB2/PB6/PB7 inputs
      CI, INT1, INT2  carriage encoder
      AN0-AN3    analogue inputs, AN4-AN7 and AN3 also DIP switches

    Emulation status: prints text through bitmap_printer. LF held at power
    on prints the self test (firmware versions, switches, character set),
    LF and FF held give the data dump (characters, control codes as
    symbols, hex codes below).
    The carriage motor, the encoder, the paper stepper scale, the DIP
    switch assignment and the Centronics to 8251 path are deduced from the
    firmware and partly assumed, see the comments.

    Command set, from probing the firmware's table driven parser (one table
    at 1867h, no other emulation): ISO 2022/6429 style, e.g.
      ESC 0 underline, ESC 3 / ESC 4 italic on/off, ESC 8 double width,
      ESC ! 1/3/5/7/8/: bold, double strike, double height, half height
      upper/lower, NLQ, ESC K / ESC L half line down/up, ESC ( F character
      set (ISO final byte), ESC c reset, CSI n w pitch (1 = 10 cpi, 4 the
      narrowest), CSI n x line spacing in 1/72", CSI n SP E extra character
      spacing, CSI n ` absolute position, CSI n a n bytes printed as they
      are, CSI n t load characters, CSI n " y/z/{ 6 bit graphics with n
      columns in three densities (bit 6 set, bits 0-5 the dots), CSI | / CSI ~
      raster ("scanner") graphics on/off: one byte gives 8 dots across at
      about 1/72", CR LF steps to the next nozzle, eight rows make a band,
      CSI n y inside sends n binary bytes (from tests, not traced in code).
    From the command set these ROMs are the ECMA firmware (not verified). Siemens sold the printer with one of
    four program modules (S22761-L312-A1 Epson MX-80, -A2 IBM PC, -A3 ECMA,
    -A4 PC-D); the other three are not dumped.

    TODO:
    - the buzzer is not found yet; which latch bit is which lamp
    - main CPU AN0 and AN7, PB6 and PB7
    - what the analogue inputs measure (they get placeholder levels)
    - INPUT PRIME, SELECT, PAPER END and FAULT
    - nozzle cover
    - the Epson, IBM PC and PC-D program modules

**********************************************************************/

#include "emu.h"
#include "pt88s.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "machine/i8251.h"

#define LOG_IO   (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

// paper bitmap
constexpr int HDPI = 360;
constexpr int VDPI = 288;                  // paper half steps: the firmware counts in 1/432", 1.5 per half step
constexpr double PAPER_WIDTH = 9.5;
constexpr double PAPER_HEIGHT = 12.0;
constexpr double LEFT_OFFSET = 0.5;        // paper edge to the home sensor edge (placeholder)

// carriage
constexpr double CR_STEPS_PER_INCH = 126.0; // a 1/10" character spans 12.6 encoder steps (from the pitches)
constexpr double CR_RATE = 160.0;          // steps per second per drive unit (placeholder)
constexpr double CR_TAU = 0.015;           // seconds (placeholder)
constexpr double CR_MIN = -300.0;          // left stop (placeholder)
constexpr double CR_MAX = 1500.0;          // right stop (placeholder)


class siemens_pt88s_device : public device_t, public device_centronics_peripheral_interface
{
public:
	siemens_pt88s_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_centronics_peripheral_interface implementation
	virtual void input_strobe(int state) override;
	virtual void input_data0(int state) override { data_bit(0, state); }
	virtual void input_data1(int state) override { data_bit(1, state); }
	virtual void input_data2(int state) override { data_bit(2, state); }
	virtual void input_data3(int state) override { data_bit(3, state); }
	virtual void input_data4(int state) override { data_bit(4, state); }
	virtual void input_data5(int state) override { data_bit(5, state); }
	virtual void input_data6(int state) override { data_bit(6, state); }
	virtual void input_data7(int state) override { data_bit(7, state); }

private:
	required_device<upd78c10_device> m_maincpu;
	required_device<upd78c10_device> m_mechcpu;
	required_device<i8251_device> m_usart;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_region_ptr<u8> m_h4;
	required_ioport m_sw1;
	required_ioport m_sw2;
	required_ioport m_keys;
	output_finder<3> m_lamps;

	std::unique_ptr<u8[]> m_ram;
	u8 m_pc;
	u8 m_latch8000;
	u8 m_latchA000;
	u8 m_data;
	int m_strobe;
	u8 m_main_pb = 0xff;
	u8 m_mech_pa = 0xff;
	u8 m_mech_pb = 0xff;
	int m_main_txd = 1;
	int m_mech_txd = 1;

	// parallel to serial converter in front of the 8251, see main_to_w()
	u8 m_usart_mode = 0;
	bool m_usart_expect_mode = true;
	bool m_usart_rxe = false;
	u16 m_conv_frame = 0;
	int m_conv_bits = 0;
	int m_conv_div = 0;
	int m_to = 0;
	bool m_conv_busy = false;
	emu_timer *m_ack_timer = nullptr;

	// carriage: DC motor with an encoder, see carriage_update()
	emu_timer *m_carriage_timer = nullptr;
	u8 m_motor = 0xff;          // register 1000h of the mechanism CPU
	double m_cr_pos = 0.0;      // in encoder steps, home sensor below 0
	double m_cr_vel = 0.0;      // in steps per second
	attotime m_cr_time;
	int m_ci = 1;
	int m_cr_dir = 0;           // direction as seen by the encoder logic

	// head and paper
	attotime m_last_fire;
	int m_pf_phase = -1;

	void data_bit(int bit, int state)
	{
		if (state)
			m_data |= 1 << bit;
		else
			m_data &= ~(1 << bit);
	}

	void main_map(address_map &map) ATTR_COLD;
	void mech_map(address_map &map) ATTR_COLD;

	u8 main_r(offs_t offset);
	void main_w(offs_t offset, u8 data);
	void latch8000_w(u8 data);

	void main_to_w(int state);
	void update_busy();
	TIMER_CALLBACK_MEMBER(ack_end);

	void motor_w(u8 data);
	void carriage_update();
	TIMER_CALLBACK_MEMBER(carriage_tick);

	u8 main_pb_r();
	u8 main_an_r(int n);
	u8 mech_pb_r();
	u8 mech_pc_r();
	u8 mech_an_r(int n);
	void mech_pa_w(u8 data);
	void mech_pb_w(u8 data);
	void fire();
};


siemens_pt88s_device::siemens_pt88s_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SIEMENS_PT88S, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_mechcpu(*this, "mechcpu"),
	m_usart(*this, "usart"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_h4(*this, "h4"),
	m_sw1(*this, "SW1"),
	m_sw2(*this, "SW2"),
	m_keys(*this, "KEYS"),
	m_lamps(*this, "pt88s_lamp%u", 0U),
	m_pc(0xff),
	m_latch8000(0),
	m_latchA000(0),
	m_data(0),
	m_strobe(1)
{
}


ROM_START( pt88s )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("s22761-k310-h3-gs2.bin", 0x0000, 0x8000, CRC(2f18fb66) SHA1(a964bef343b468f7c66a2c3796c8c87a88e2cb73))

	ROM_REGION(0x10000, "h4", 0)
	ROM_LOAD("s22761-k310-h4-gs2.bin", 0x0000, 0x10000, CRC(07d1f96b) SHA1(e1fdf5249b630bb9bce3d4582282d4bd8dbad836))

	ROM_REGION(0x2000, "mechcpu", 0)
	ROM_LOAD("s22761-k101-h5-gs4.bin", 0x0000, 0x2000, CRC(781c9977) SHA1(2edf415569230373573cd4e4c8d5142c75f813c6))
ROM_END

const tiny_rom_entry *siemens_pt88s_device::device_rom_region() const
{
	return ROM_NAME( pt88s );
}


/*
    Switches and keys: at power on the firmware collects two bytes and sets
    its configuration from them through the table at 0300h/0724h:
      FD6Dh: mechanism CPU PB7, PF6, PF7, AN3 and AN4-AN7 (above 80h)
      FD6Eh: mechanism CPU PC6, PC7, main CPU PB5, AN1, AN2, AN3 (above 80h),
             -, AN0 (below 80h)
    The self test prints them as S1 (eight switches) and S2 (six), so FD6Dh
    is taken as SW1 and FD6Eh bits 0-5 as SW2. A bit read as 0 is taken as
    off, as all zero gives the factory settings listed for the PT88 S (ASCII,
    1/10" pitch, 1/6" line spacing). Verified in the emulation:
      SW1:1-4    character set, all off ASCII: SW1:4 off picks ASCII or one
                 of the 7 national sets at 0136h, SW1:4 on one of the 8 at
                 018Ah, of which the last two are blank in the ROM
      SW1:5      on: 1/12" pitch
      SW1:6      typeface
      SW1:8      form length (FE35h/FE36h, in 1/432"): off 6", on 12"
      SW2:1      line spacing (FE33h, in 1/432"): off 1/6", on 1/8"
    From the firmware table only: SW2:2 clears the 1" skip over perforation
    (FE39h/FE3Ah). SW1:7 (FE2Dh bit 1), SW2:3 (main PB5, no table entry),
    SW2:4/5 (FE2Bh bits 0/1) and SW2:6 (FE00h bit 1) are not known; parity,
    CR with LF and the buzzer are candidates.
    Panel: lamps Power, Select/Alarm and On Line, keys On Line, LF and FF.
    The keys are on the main CPU's AN4-AN6, scanned all the time; the order
    of the three is assumed (LF held at power on prints the self test, LF
    and FF the data dump).
*/

INPUT_PORTS_START( pt88s )
	PORT_START("SW1")
	PORT_DIPNAME(0x01, 0x00, "Character set (bit 2)") PORT_DIPLOCATION("SW1:1")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x01, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "Character set (bit 1)") PORT_DIPLOCATION("SW1:2")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Character set (bit 0)") PORT_DIPLOCATION("SW1:3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "Character set (bit 3)") PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x08, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "Pitch 1/12 in") PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x10, DEF_STR(On))
	PORT_DIPNAME(0x20, 0x00, "Typeface") PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x20, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x00, "Not known (FE2Dh bit 1)") PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x40, DEF_STR(On))
	PORT_DIPNAME(0x80, 0x00, "Form length 12 in") PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x80, DEF_STR(On))

	PORT_START("SW2")
	PORT_DIPNAME(0x01, 0x00, "Line spacing 1/8 in") PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x01, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "No skip over perforation") PORT_DIPLOCATION("SW2:2")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Not known (main PB5)") PORT_DIPLOCATION("SW2:3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "Not known (FE2Bh bit 0)") PORT_DIPLOCATION("SW2:4")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x08, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "Not known (FE2Bh bit 1)") PORT_DIPLOCATION("SW2:5")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x10, DEF_STR(On))
	PORT_DIPNAME(0x20, 0x00, "Not known (FE00h bit 1)") PORT_DIPLOCATION("SW2:6")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x20, DEF_STR(On))
	PORT_DIPNAME(0x80, 0x80, "AN0 (not known)")
	PORT_DIPSETTING(   0x00, "Low")
	PORT_DIPSETTING(   0x80, "High")

	PORT_START("KEYS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("On Line") PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("LF") PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("FF") PORT_CODE(KEYCODE_3_PAD)
INPUT_PORTS_END

ioport_constructor siemens_pt88s_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( pt88s );
}


void siemens_pt88s_device::main_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0xffff).rw(FUNC(siemens_pt88s_device::main_r), FUNC(siemens_pt88s_device::main_w));
}

void siemens_pt88s_device::mech_map(address_map &map)
{
	map(0x0000, 0x0fff).rom().region("mechcpu", 0);
	map(0x1000, 0x1000).w(FUNC(siemens_pt88s_device::motor_w));
	map(0x1001, 0x1fff).rom().region("mechcpu", 0x1001);
}


u8 siemens_pt88s_device::main_r(offs_t offset)
{
	offs_t const addr = offset + 0x8000;
	if (BIT(m_pc, 6))
	{
		if (addr >= 0xe000)
			return m_usart->read(addr & 1);
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "%s: I/O read %04x\n", machine().describe_context(), addr);
		return 0xff;
	}
	if (addr < 0xc000 && !BIT(m_latchA000, 2))
		return m_h4[((m_latchA000 & 3) << 14) | (addr & 0x3fff)];
	return m_ram[addr - 0x8000];
}

void siemens_pt88s_device::main_w(offs_t offset, u8 data)
{
	offs_t const addr = offset + 0x8000;
	if (BIT(m_pc, 6))
	{
		LOGMASKED(LOG_IO, "%s: I/O write %04x = %02x\n", machine().describe_context(), addr, data);
		if (addr == 0x8000)
			latch8000_w(data);
		else if (addr == 0xa000)
			m_latchA000 = data;
		else if (addr >= 0xe000)
		{
			if (BIT(addr, 0))
			{
				// follow the 8251 programming for the converter
				if (m_usart_expect_mode)
				{
					m_usart_mode = data;
					m_usart_expect_mode = false;
				}
				else if (BIT(data, 6))
					m_usart_expect_mode = true;
				else
					m_usart_rxe = BIT(data, 2);
				update_busy();
			}
			m_usart->write(addr & 1, data);
		}
		return;
	}
	// writes always reach the RAM (assumed)
	m_ram[addr - 0x8000] = data;
}

void siemens_pt88s_device::latch8000_w(u8 data)
{
	// bits 2-4: panel lamps as set from FD63h-FD65h (bit 4 is low when lit)
	m_latch8000 = data;
	m_lamps[0] = BIT(data, 2);
	m_lamps[1] = BIT(data, 3);
	m_lamps[2] = BIT(data, 4) ^ 1;
}


/*
    Host interface: the main CPU reads host data only from the 8251, so the
    Centronics port is assumed to be turned into a serial frame on L203 and
    fed to RxD with the 8251's own clock (TO) and format; BUSY stays up
    while the frame is sent and while the 8251 receiver is off.
*/

void siemens_pt88s_device::input_strobe(int state)
{
	if (m_strobe && !state && !m_conv_busy)
	{
		// start bit, data, parity, stop bits as programmed into the 8251
		int const len = 5 + ((m_usart_mode >> 2) & 3);
		u16 frame = (m_data & ((1 << len) - 1)) << 1;
		int bits = 1 + len;
		if (BIT(m_usart_mode, 4))
		{
			int par = 0;
			for (int i = 0; i < len; i++)
				par ^= BIT(m_data, i);
			if (!BIT(m_usart_mode, 5))
				par ^= 1;   // odd parity
			frame |= par << bits;
			bits++;
		}
		frame |= 3 << bits;
		bits += 2;
		m_conv_frame = frame;
		m_conv_bits = bits;
		m_conv_div = 0;
		m_conv_busy = true;
		m_usart->write_rxd(0);
		update_busy();
	}
	m_strobe = state;
}

void siemens_pt88s_device::main_to_w(int state)
{
	m_usart->write_txc(state);
	m_usart->write_rxc(state);

	if (state && !m_to && m_conv_bits)
	{
		static const int factor[4] = { 1, 1, 16, 64 };
		if (++m_conv_div >= factor[m_usart_mode & 3])
		{
			m_conv_div = 0;
			m_conv_frame >>= 1;
			if (!--m_conv_bits)
			{
				m_usart->write_rxd(1);
				m_conv_busy = false;
				output_ack(0);
				m_ack_timer->adjust(attotime::from_usec(5));
				update_busy();
			}
			else
				m_usart->write_rxd(m_conv_frame & 1);
		}
	}
	m_to = state;
}

TIMER_CALLBACK_MEMBER(siemens_pt88s_device::ack_end)
{
	output_ack(1);
}

void siemens_pt88s_device::update_busy()
{
	// PB3, which the firmware sets after each byte, has no known part in
	// the handshake yet
	output_busy((m_conv_busy || !m_usart_rxe || m_usart_expect_mode) ? 1 : 0);
}


/*
    Carriage, from the mechanism firmware: register 1000h drives a DC motor,
    bits 0-4 are the inverted drive level, bit 6 the direction (set: towards
    the home sensor on PB6), FFh stops it. An encoder delivers pulses at CI;
    the firmware measures their period with the event counter and closes the
    speed loop itself. The same encoder is assumed on INT1 (rising edge) and
    INT2 (falling edge), with its direction on PB2; the firmware counts the
    carriage position from these. The motor model (speed proportional to
    the drive level, first order lag) and its constants are placeholders.
*/

void siemens_pt88s_device::carriage_update()
{
	attotime const now = machine().time();
	double const dt = (now - m_cr_time).as_double();
	m_cr_time = now;
	if (dt <= 0.0)
		return;

	double target = 0.0;
	if (!BIT(m_motor, 7))
	{
		target = double(0x1f - (m_motor & 0x1f)) * CR_RATE;
		if (BIT(m_motor, 6))
			target = -target;
	}
	double const e = exp(-dt / CR_TAU);
	m_cr_pos += target * dt + (m_cr_vel - target) * CR_TAU * (1.0 - e);
	m_cr_vel = target + (m_cr_vel - target) * e;
	if (m_cr_pos < CR_MIN) { m_cr_pos = CR_MIN; m_cr_vel = 0.0; }
	if (m_cr_pos > CR_MAX) { m_cr_pos = CR_MAX; m_cr_vel = 0.0; }

	// encoder: one cycle per step, the falling edge at the whole step
	int const ci = (int(floor(m_cr_pos * 2.0)) & 1) ? 1 : 0;
	if (ci != m_ci)
	{
		m_ci = ci;
		if (m_cr_vel != 0.0)
			m_cr_dir = (m_cr_vel < 0.0) ? 1 : 0;
		m_mechcpu->set_input_line(UPD7810_CI_LINE, ci ? ASSERT_LINE : CLEAR_LINE);
		m_mechcpu->set_input_line(UPD7810_INTF1, ci ? ASSERT_LINE : CLEAR_LINE);
		m_mechcpu->set_input_line(UPD7810_INTF2, ci ? CLEAR_LINE : ASSERT_LINE);
	}

	m_bitmap_printer->setheadpos(int((m_cr_pos / CR_STEPS_PER_INCH + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);

	// next look: at the next half step, or after 1 ms
	double next = 1e-3;
	if (fabs(m_cr_vel) > 1.0)
	{
		double const half = floor(m_cr_pos * 2.0);
		double const to = (m_cr_vel > 0.0) ? ((half + 1.0) / 2.0 - m_cr_pos) : (m_cr_pos - half / 2.0);
		next = std::clamp(to / fabs(m_cr_vel), 2e-6, 1e-3);
	}
	m_carriage_timer->adjust(attotime::from_double(next));
}

TIMER_CALLBACK_MEMBER(siemens_pt88s_device::carriage_tick)
{
	carriage_update();
}

void siemens_pt88s_device::motor_w(u8 data)
{
	carriage_update();
	m_motor = data;
	carriage_update();
}


u8 siemens_pt88s_device::main_pb_r()
{
	// PB5: SW2:3; PB6/PB7: not known
	return 0xdf | (BIT(m_sw2->read(), 2) << 5);
}

u8 siemens_pt88s_device::main_an_r(int n)
{
	// AN0-AN3: read once at power on with ANM = 00h; AN1-AN3 are SW2:4-6
	//          (above 80h is off), AN0 counts below 80h and is not known
	// AN4-AN7: scanned all the time with ANM = 08h by the INTE0/INTE1
	//          handler; AN4-AN6 are the keys (below 80h is pressed), AN7
	//          counts above 80h and raises an error, not known yet
	u8 const sw = m_sw2->read();
	switch (n)
	{
	case 0: return BIT(sw, 7) ? 0xff : 0x00;
	case 1: case 2: case 3: return BIT(sw, n + 2) ? 0xff : 0x00;
	case 4: case 5: case 6: return BIT(m_keys->read(), n - 4) ? 0x00 : 0xff;
	default: return 0x00;
	}
}

u8 siemens_pt88s_device::mech_pb_r()
{
	// PB2: carriage direction from the encoder (set: towards home); the
	//      INT1/INT2 handler counts the position in FF84h with it
	// PB6: carriage home sensor
	// PB7: SW1:1
	carriage_update();
	u8 data = 0x7f | (BIT(m_sw1->read(), 0) << 7);
	if (!m_cr_dir)
		data &= ~0x04;
	if (m_cr_pos >= 0.0)
		data &= ~0x40;
	return data;
}

u8 siemens_pt88s_device::mech_pc_r()
{
	// PC6/PC7: SW2:1/2
	return 0x3f | ((m_sw2->read() & 0x03) << 6);
}

u8 siemens_pt88s_device::mech_an_r(int n)
{
	// placeholder levels inside the limits the firmware checks: AN0 sets
	// the head drive time, AN1 at least 2Eh, AN2 at least 5Eh after
	// filtering (else status bits 5 and 4); AN3-AN7 are switches above 80h
	switch (n)
	{
	case 0: return 0x60;
	case 1: return 0x80;
	case 2: return 0x80;
	default: return BIT(m_sw1->read(), n) ? 0xff : 0x00;
	}
}

void siemens_pt88s_device::mech_pa_w(u8 data)
{
	m_mech_pa = data;

	// paper stepper on PA0-PA3 while PB3 is low: half steps in the order
	// of the table at 03B7h, one half step is one bitmap line (assumed)
	if (!BIT(m_mech_pb, 3))
	{
		static const u8 phases[8] = { 0x0a, 0x0e, 0x06, 0x07, 0x05, 0x0d, 0x09, 0x0b };
		int phase = -1;
		for (int i = 0; i < 8; i++)
			if ((data & 0x0f) == phases[i])
				phase = i;
		if (phase >= 0)
		{
			if (m_pf_phase >= 0)
			{
				int const delta = (phase - m_pf_phase) & 7;
				if (delta == 1)
					m_bitmap_printer->m_ypos++;
				else if (delta == 7)
					m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
				m_bitmap_printer->check_new_page();
			}
			m_pf_phase = phase;
		}
	}
}

void siemens_pt88s_device::mech_pb_w(u8 data)
{
	// the fire handler pulses PB1 low twice in a row; one drop per column
	if (BIT(m_mech_pb, 1) && !BIT(data, 1) && BIT(data, 3))
	{
		attotime const now = machine().time();
		if ((now - m_last_fire) > attotime::from_usec(20))
		{
			m_mech_pb = data;
			fire();
		}
		m_last_fire = now;
	}
	m_mech_pb = data;
}

void siemens_pt88s_device::fire()
{
	carriage_update();
	int const x = int(std::lround((m_cr_pos / CR_STEPS_PER_INCH + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// nozzles 1/72" apart (assumed), nozzle 1 on top
	u16 const nozzles = ~(m_mech_pa | (BIT(m_mech_pb, 0) << 8)) & 0x1ff;
	for (int n = 0; n < 9; n++)
		if (BIT(nozzles, n))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + (n * VDPI + 36) / 72, 0x000000);
}


void siemens_pt88s_device::device_add_mconfig(machine_config &config)
{
	UPD78C10(config, m_maincpu, 11.0592_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &siemens_pt88s_device::main_map);
	// interface module jumpers, read once at 26C0h: PA0-PA2 low pick the
	// fastest internal rate (TM0 = 3, 8251 x16), PA5 high no XON/XOFF; the
	// SAP-P11 Centronics module is assumed to set these
	m_maincpu->pa_in_cb().set_constant(0xf8);
	m_maincpu->pb_in_cb().set(FUNC(siemens_pt88s_device::main_pb_r));
	m_maincpu->pb_out_cb().set([this] (u8 data) { m_main_pb = data; });
	m_maincpu->pc_in_cb().set([this] () { return m_pc; });
	m_maincpu->pc_out_cb().set([this] (u8 data) { m_pc = data; });
	m_maincpu->an0_func().set([this] () { return main_an_r(0); });
	m_maincpu->an1_func().set([this] () { return main_an_r(1); });
	m_maincpu->an2_func().set([this] () { return main_an_r(2); });
	m_maincpu->an3_func().set([this] () { return main_an_r(3); });
	m_maincpu->an4_func().set([this] () { return main_an_r(4); });
	m_maincpu->an5_func().set([this] () { return main_an_r(5); });
	m_maincpu->an6_func().set([this] () { return main_an_r(6); });
	m_maincpu->an7_func().set([this] () { return main_an_r(7); });

	UPD78C10(config, m_mechcpu, 11.0592_MHz_XTAL);
	m_mechcpu->set_addrmap(AS_PROGRAM, &siemens_pt88s_device::mech_map);
	m_mechcpu->pa_out_cb().set(FUNC(siemens_pt88s_device::mech_pa_w));
	m_mechcpu->pb_in_cb().set(FUNC(siemens_pt88s_device::mech_pb_r));
	m_mechcpu->pb_out_cb().set(FUNC(siemens_pt88s_device::mech_pb_w));
	m_mechcpu->pc_in_cb().set(FUNC(siemens_pt88s_device::mech_pc_r));
	m_mechcpu->pf_in_cb().set([this] () { return u8(0x3f | ((m_sw1->read() & 0x06) << 5)); });   // PF6/PF7: SW1:2/3
	m_mechcpu->an0_func().set([this] () { return mech_an_r(0); });
	m_mechcpu->an1_func().set([this] () { return mech_an_r(1); });
	m_mechcpu->an2_func().set([this] () { return mech_an_r(2); });
	m_mechcpu->an3_func().set([this] () { return mech_an_r(3); });
	m_mechcpu->an4_func().set([this] () { return mech_an_r(4); });
	m_mechcpu->an5_func().set([this] () { return mech_an_r(5); });
	m_mechcpu->an6_func().set([this] () { return mech_an_r(6); });
	m_mechcpu->an7_func().set([this] () { return mech_an_r(7); });

	// the CPUs talk over their serial ports; the main CPU takes its serial
	// clock from outside (SMH = 0Fh), the mechanism CPU runs on its internal
	// clock (SMH = 0Eh, CLK/24) and is assumed to feed its SCK across
	m_mechcpu->txd_func().set([this] (int state) { m_mech_txd = state; });
	m_maincpu->txd_func().set([this] (int state) { m_main_txd = state; });
	m_maincpu->rxd_func().set([this] () { return m_mech_txd; });
	m_mechcpu->rxd_func().set([this] () { return m_main_txd; });
	m_mechcpu->sck_func().set(m_maincpu, FUNC(upd7810_device::sck_w));

	// 8251 for the host interface: INT2 takes RxRDY, INT1 TxRDY, the clocks
	// come from TO (assumed)
	I8251(config, m_usart, 11.0592_MHz_XTAL / 3);
	m_usart->rxrdy_handler().set_inputline(m_maincpu, UPD7810_INTF2);
	m_usart->txrdy_handler().set_inputline(m_maincpu, UPD7810_INTF1);
	m_maincpu->to_func().set(FUNC(siemens_pt88s_device::main_to_w));

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::inkjet(0.0125));
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_12);
}


void siemens_pt88s_device::device_start()
{
	m_ram = std::make_unique<u8[]>(0x8000);
	save_pointer(NAME(m_ram), 0x8000);
	save_item(NAME(m_pc));
	save_item(NAME(m_latch8000));
	save_item(NAME(m_latchA000));
	save_item(NAME(m_data));
	save_item(NAME(m_strobe));
	save_item(NAME(m_main_pb));
	save_item(NAME(m_mech_pa));
	save_item(NAME(m_mech_pb));
	save_item(NAME(m_main_txd));
	save_item(NAME(m_mech_txd));
	save_item(NAME(m_usart_mode));
	save_item(NAME(m_usart_expect_mode));
	save_item(NAME(m_usart_rxe));
	save_item(NAME(m_conv_frame));
	save_item(NAME(m_conv_bits));
	save_item(NAME(m_conv_div));
	save_item(NAME(m_to));
	save_item(NAME(m_conv_busy));
	save_item(NAME(m_motor));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_vel));
	save_item(NAME(m_cr_time));
	save_item(NAME(m_ci));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_last_fire));
	save_item(NAME(m_pf_phase));

	m_ack_timer = timer_alloc(FUNC(siemens_pt88s_device::ack_end), this);
	m_carriage_timer = timer_alloc(FUNC(siemens_pt88s_device::carriage_tick), this);
	m_cr_pos = 200.0;   // somewhere right of home (assumed)
}

void siemens_pt88s_device::device_reset()
{
	latch8000_w(0x53);
	m_motor = 0xff;
	m_usart_expect_mode = true;
	m_usart_rxe = false;
	m_conv_bits = 0;
	m_conv_busy = false;
	m_usart->write_rxd(1);
	m_usart->write_cts(0);
	m_usart->write_dsr(0);
	output_ack(1);
	output_select(1);
	output_perror(0);
	output_fault(1);
	update_busy();
	m_cr_time = machine().time();
	m_carriage_timer->adjust(attotime::from_msec(1));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SIEMENS_PT88S, device_centronics_peripheral_interface, siemens_pt88s_device, "pt88s", "Siemens PT88 S")
