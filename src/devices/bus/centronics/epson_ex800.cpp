// license:BSD-3-Clause
// copyright-holders:Dirk Best, rfka01
/******************************************************************************

    Epson EX-800 Dot Matrix printer


 --

 Main CPU is a UPD7810H6 running at 12 MHz.

 --

 On startup, the ports are initialized as follows:

 Port A lines PA0-PA7 are configured as output
 Port B lines PB0-PB7 are configured as output
 Port C lines PC2, PC4 and PC5 are configured as output
        the other lines are control lines
        PC0: TxD output
        PC1: RxD input
        PC3: TI input
        PC6: CO0 output
        PC7: CO1 output

The I/O lines are connected as follows:

PA0: \
PA1:  Carriage motor
PA2:  driving pulses
PA3: /
PA4: High when carriage assembly reaches either end
PA5: High when carriage assembly reaches either end
PA6: Bank 0
PA7: Bank 1

PB0: Line feed motor driving pulses
PB1: Line feed motor driving pulses
PB2: Line feed motor current control
PB3: LED09, Online
PB4: (used for serial port)
PB5: ERR signal
PB6: ACKNLG signal, data received at printer
PB7: LED12, Paper empty

PC0: TxD output
PC1: RxD input
PC2:
PC3: TI input
PC4:
PC5:
PC6: PWD signal of 5a
PC7: Buzzer

PD0-7: Data bus D0-D7

PF0-7: Address bus A8-A15

AN0: DSW 2
AN1: DSW 1
AN2:
AN3:
AN4: Print head operating temperature sensor
AN5: Scanner Unit, Color Home sensor
AN6:
AN7: Color Home sensor adjust

INT: Online switch


Gate array 5a (parallel port, printer head)

IN0-IN7: DATA1 to DATA8 from parallel port
HD1-HD9: Head driving pulses
STROBE: STROBE signal
PWD: PC6 of CPU
ALE: ALE of CPU

Gate array 7a (inputs)

- activated when carriage assembly reaches either end?

PA0: Draft switch
PA1: NLQ Roman switch
PA2: Pica switch
PA3: NLQ sans-serif switch
PA4: Elite switch
PA5: Proportional switch
PA6: Normal switch
PA7: Condensed switch

PB0: Form feed switch
PB1: Line feed switch
PB2: Autoload switch
PB3: Low when paper empty
PB4: Low when carriage assembly is in home position
PB5: SLCTIN (connected to parallel port), disable with DSW2-1
PB6: AUTO FEEDXT (connected to parallel port), disable with DSW2-4
       "When this signal is LOW, the paper is automatically fed 1 line after
        printing."
PB7:

PC0: Dipswitch SW1-1 and SW1-2, Draft LED
PC1: Dipswitch SW1-3 and SW1-4, Roman LED
PC2: Dipswitch SW1-5 and SW1-6, Pica LED
PC3: Dipswitch SW1-7 and SW1-8, Sans-Serif LED
PC4: Dipswitch SW2-2 and SW2-3, Elite LED
PC5: Dipswitch SW2-5 and SW2-6, Proportional LED
PC6: Normal LED
PC7: Condensed LED

--

LED state on startup:

The power light comes on, then draft, pica and normal. If there is no paper,
the paper empty LED comes on.

Power LED is green, selectype LED is orange, paper out LED is red.

--

Findings from the firmware (rfka01):

 - Memory: 0000-7fff ROM, 8000-bfff RAM, c000-dfff colour option, e000 gate
   array 5a, e800 interface option, f000 gate array 7a
 - Gate array 5a: e000 r Centronics data (clears the strobe flag and BUSY),
   e001 r bit 0 = strobe flag, e002/e003 w needle latches, e005 w ?,
   e004 r settings of a serial option (used when 7a PB5 = 0)
 - Gate array 7a: f000 r SelecType keys (0 = pressed), f001 r FORM FEED,
   LINE FEED, LOAD/EJECT (bits 0-2, 0 = pressed), paper (bit 3, 0 = none),
   home (bit 4, 0 = home), 1 (bit 5), /AUTO FEED XT (bit 6), DIP SW2-4
   (bit 7, 0 = on);
   f002 w SelecType lamps (0 = on), also the strobes of the DIP switch
   matrix: a low line puts one switch on AN1 and one on AN0 (0 = ON);
   with no line low AN1/AN0 read SW2-7/SW2-8
     PC0: SW1-1/SW1-2  PC1: SW1-3/SW1-4  PC2: SW1-5/SW1-6  PC3: SW1-7/SW1-8
     PC4: SW2-2/SW2-3  PC5: SW2-5/SW2-6
 - SW2-1 (page length) is on the NMI pin, read with SKIT NMI
 - ON LINE key: INT1 (flag polled with SKIT F1)
 - PA0-PA3 carriage motor phases (A 9 5 6), PA4/PA5 carriage motor current
 - PB0/PB1 paper motor phases (Gray code 3 2 1 0 forward, 1/432" per step), PB2 paper motor current (0 = on),
   PB3 ON LINE lamp (0 = on), PB4 BUSY (also the serial DTR), PB5 /ERROR,
   PB6 /ACK, PB7 PAPER OUT lamp (0 = on)
 - CO0 (PC6): FIRE; CO1 (PC7): buzzer
 - AN4 printhead temperature (above 2Ah), AN6 supply supervision (B9h-F9h),
   AN2/AN3 printhead timing adjustment, AN5/AN7 colour option
 - Serial interface: the uPD7810 UART; interface type and baud rate on
   SW2-5 to SW2-8, XON/XOFF and BUSY (DTR)

******************************************************************************/


#include "emu.h"
#include "epson_ex800.h"

#include "cpu/upd7810/upd7810.h"
#include "machine/bitmap_printer.h"
#include "sound/beep.h"

#include "speaker.h"

#include "ex800.lh"

#define LOG_MOTOR (1U << 1)
#define LOG_FIRE  (1U << 2)
#define LOG_IO    (1U << 3)

//#define VERBOSE (LOG_GENERAL | LOG_IO | LOG_MOTOR | LOG_FIRE)
#include "logmacro.h"


namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 432;               // vertical resolution: one paper motor step
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 1.0 / 3 + 0.75 - 0.392;    // paper edge to the home sensor edge: column 1 at 3/4"

constexpr double CR_LAG = -0.00006;     // time offset of the head against the rotor (s), from bidirectional printing
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 560;           // right stop

// 2-bit Gray code phase index (00 01 11 10)
constexpr int gray_phase[4] = { 0, 1, 3, 2 };


//**************************************************************************
//  PRINTER
//**************************************************************************

class epson_ex800_base : public device_t
{
public:
	DECLARE_INPUT_CHANGED_MEMBER(online_key);
	DECLARE_INPUT_CHANGED_MEMBER(power_switch);
	DECLARE_INPUT_CHANGED_MEMBER(nmi_switch);

protected:
	epson_ex800_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// the host interface
	virtual void out_busy(int state) { }
	virtual void out_ack(int state) { }
	virtual void out_perror(int state) { }
	virtual void out_fault(int state) { }
	virtual void out_select(int state) { }
	virtual void out_txd(int state) { }

	void strobe_w(int state);
	void init_w(int state);
	void rxd_w(int state) { if (state != m_rxd) LOGMASKED(LOG_IO, "%s RXD %d\n", machine().time().as_string(), state); m_rxd = state; }

	u8 m_data;              // Centronics data lines
	int m_autofd;
	int m_slctin;

private:
	void ex800_mem(address_map &map) ATTR_COLD;

	u8 pa_r();
	void pa_w(u8 data);
	u8 pb_r();
	void pb_w(u8 data);
	u8 pc_r();
	void pc_w(u8 data);
	void co0_w(int state);

	u8 an0_r();
	u8 an1_r();

	u8 devsel_r(offs_t offset);
	void devsel_w(offs_t offset, u8 data);
	u8 gate5a_r(offs_t offset);
	void gate5a_w(offs_t offset, u8 data);
	u8 iosel_r(offs_t offset);
	void iosel_w(offs_t offset, u8 data);
	u8 gate7a_r(offs_t offset);
	void gate7a_w(offs_t offset, u8 data);

	void restart();
	void update_busy();
	bool dip(int sw) const;
	void carriage_step(int dir);
	double head_x() const;
	void fire();
	u8 keys_held() const;

	required_device<upd7810_device> m_maincpu;
	required_device<bitmap_printer_device> m_bitmap_printer;
	required_device<beep_device> m_beeper;
	required_ioport m_panel;
	required_ioport m_selectype;
	required_ioport m_dsw1;
	required_ioport m_dsw2;
	required_ioport m_config;
	output_finder<> m_online_led;
	output_finder<> m_paper_led;
	output_finder<> m_ready_led;
	output_finder<8> m_selectype_led;

	// gate array 5a
	u8 m_latch;             // Centronics data latched by /STROBE
	int m_strobe;
	bool m_strobe_flag;
	bool m_init;
	u16 m_head;             // needles 1-9 (bit 0 = needle 1), 1 = fires

	// gate array 7a
	u8 m_7a_pc;

	u8 m_pa;
	u8 m_pb;
	int m_co0;
	int m_rxd;

	// carriage
	int m_cr_pos;           // 1/60" steps right of the home sensor edge
	int m_cr_phase;
	int m_cr_dir;
	attotime m_cr_last_step;
	attotime m_cr_period;

	// paper
	int m_pf_phase;

	attotime m_power_on;
	bool m_power_cycled;
};


//-------------------------------------------------
//  ROM
//-------------------------------------------------

ROM_START( ex800 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("w8_pe9.9b", 0x0000, 0x8000, CRC(6dd41e9b) SHA1(8e30ead727b9317154742efd881206e9f9bbf95b))
ROM_END

const tiny_rom_entry *epson_ex800_base::device_rom_region() const
{
	return ROM_NAME( ex800 );
}


//-------------------------------------------------
//  INPUT PORTS
//-------------------------------------------------

#define EX800_INPUT_PORTS(_interface) \
	PORT_START("PANEL") \
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 ON LINE") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_ex800_base::online_key), 0) \
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 FORM FEED") \
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 LINE FEED") \
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 LOAD/EJECT") \
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("EX-800 Power switch (off/on)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_ex800_base::power_switch), 0) \
	\
	PORT_START("SELECTYPE") \
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Draft") \
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 NLQ Roman") \
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Pica") \
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 NLQ Sans Serif") \
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Elite") \
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Proportional") \
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Normal") \
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("EX-800 Condensed") \
	\
	PORT_START("DSW1")  /* 1 = ON */ \
	PORT_DIPNAME(0x01, 0x00, "Condensed characters")       PORT_DIPLOCATION("SW1:1") \
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) \
	PORT_DIPSETTING(   0x01, DEF_STR(On)) \
	PORT_DIPNAME(0x02, 0x00, "Slashed zero")               PORT_DIPLOCATION("SW1:2") \
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) \
	PORT_DIPSETTING(   0x02, DEF_STR(On)) \
	PORT_DIPNAME(0x04, 0x00, "Character table")            PORT_DIPLOCATION("SW1:3") \
	PORT_DIPSETTING(   0x00, "Italics") \
	PORT_DIPSETTING(   0x04, "Graphics") \
	PORT_DIPNAME(0x08, 0x00, "Printer commands")           PORT_DIPLOCATION("SW1:4") \
	PORT_DIPSETTING(   0x00, "ESC/P") \
	PORT_DIPSETTING(   0x08, "IBM printer emulation") \
	PORT_DIPNAME(0x10, 0x00, "Print quality")              PORT_DIPLOCATION("SW1:5") \
	PORT_DIPSETTING(   0x00, "Draft") \
	PORT_DIPSETTING(   0x10, "NLQ") \
	PORT_DIPNAME(0xe0, 0xe0, "International character set") PORT_DIPLOCATION("SW1:6,7,8") \
	PORT_DIPSETTING(   0xe0, "USA") \
	PORT_DIPSETTING(   0x60, "France") \
	PORT_DIPSETTING(   0xa0, "Germany") \
	PORT_DIPSETTING(   0x20, "UK") \
	PORT_DIPSETTING(   0xc0, "Denmark") \
	PORT_DIPSETTING(   0x40, "Sweden") \
	PORT_DIPSETTING(   0x80, "Italy") \
	PORT_DIPSETTING(   0x00, "Spain") \
	\
	PORT_START("DSW2")  /* 1 = ON */ \
	PORT_DIPNAME(0x01, 0x00, "Page length")                PORT_DIPLOCATION("SW2:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_ex800_base::nmi_switch), 0) \
	PORT_DIPSETTING(   0x00, "11 inch") \
	PORT_DIPSETTING(   0x01, "12 inch") \
	PORT_DIPNAME(0x02, 0x00, "Cut sheet feeder")           PORT_DIPLOCATION("SW2:2") \
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) \
	PORT_DIPSETTING(   0x02, DEF_STR(On)) \
	PORT_DIPNAME(0x04, 0x00, "Skip over perforation")      PORT_DIPLOCATION("SW2:3") \
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) \
	PORT_DIPSETTING(   0x04, "1 inch") \
	PORT_DIPNAME(0x08, 0x00, "Line feed after CR")         PORT_DIPLOCATION("SW2:4") \
	PORT_DIPSETTING(   0x00, DEF_STR(Off)) \
	PORT_DIPSETTING(   0x08, DEF_STR(On)) \
	PORT_DIPNAME(0x30, _interface, "Interface")            PORT_DIPLOCATION("SW2:5,6") \
	PORT_DIPSETTING(   0x00, "Parallel") \
	PORT_DIPSETTING(   0x10, "Serial, even parity") \
	PORT_DIPSETTING(   0x20, "Serial, odd parity") \
	PORT_DIPSETTING(   0x30, "Serial, no parity") \
	PORT_DIPNAME(0xc0, 0x00, "Baud rate")                  PORT_DIPLOCATION("SW2:7,8") \
	PORT_DIPSETTING(   0x00, "9600") \
	PORT_DIPSETTING(   0x80, "4800") \
	PORT_DIPSETTING(   0x40, "1200") \
	PORT_DIPSETTING(   0xc0, "300") \
	\
	PORT_START("CONFIG") \
	PORT_CONFNAME(0x01, 0x01, "Paper") \
	PORT_CONFSETTING(   0x00, "Empty") \
	PORT_CONFSETTING(   0x01, "Loaded") \
	PORT_CONFNAME(0x06, 0x00, "Keys held at power on") \
	PORT_CONFSETTING(   0x00, "None") \
	PORT_CONFSETTING(   0x02, "LINE FEED (self test, draft)") \
	PORT_CONFSETTING(   0x04, "FORM FEED (self test, NLQ)") \
	PORT_CONFSETTING(   0x06, "LINE FEED + FORM FEED (data dump)")

INPUT_PORTS_START( epson_ex800 )
	EX800_INPUT_PORTS(0x00)
INPUT_PORTS_END

INPUT_PORTS_START( epson_ex800_serial )
	EX800_INPUT_PORTS(0x30)
INPUT_PORTS_END


//-------------------------------------------------
//  DEVICE
//-------------------------------------------------

epson_ex800_base::epson_ex800_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_data(0),
	m_autofd(1),
	m_slctin(1),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_beeper(*this, "beeper"),
	m_panel(*this, "PANEL"),
	m_selectype(*this, "SELECTYPE"),
	m_dsw1(*this, "DSW1"),
	m_dsw2(*this, "DSW2"),
	m_config(*this, "CONFIG"),
	m_online_led(*this, "ex800_online_led"),
	m_paper_led(*this, "ex800_paper_led"),
	m_ready_led(*this, "ex800_ready_led"),
	m_selectype_led(*this, "ex800_selectype_led%u", 0U),
	m_latch(0),
	m_strobe(1),
	m_strobe_flag(false),
	m_init(true),
	m_head(0),
	m_7a_pc(0xff),
	m_pa(0xff),
	m_pb(0xff),
	m_co0(1),
	m_rxd(1),
	m_cr_pos(100),
	m_cr_phase(-1),
	m_cr_dir(1),
	m_pf_phase(-1),
	m_power_cycled(false)
{
}

void epson_ex800_base::ex800_mem(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("maincpu", 0);
	map(0x8000, 0xbfff).ram(); /* external RAM */
	map(0xc000, 0xc7ff).mirror(0x1800).rw(FUNC(epson_ex800_base::devsel_r), FUNC(epson_ex800_base::devsel_w));
	map(0xe000, 0xe7ff).rw(FUNC(epson_ex800_base::gate5a_r), FUNC(epson_ex800_base::gate5a_w));
	map(0xe800, 0xefff).rw(FUNC(epson_ex800_base::iosel_r), FUNC(epson_ex800_base::iosel_w));
	map(0xf000, 0xf7ff).rw(FUNC(epson_ex800_base::gate7a_r), FUNC(epson_ex800_base::gate7a_w));
	map(0xf800, 0xfeff).noprw(); /* not connected */
}

void epson_ex800_base::device_add_mconfig(machine_config &config)
{
	UPD7810(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &epson_ex800_base::ex800_mem);
	m_maincpu->set_ti_clock(614'400);   // baud rate clock: TM0 = 2 gives 9600 baud, the firmware's table runs from 110 to 19200
	m_maincpu->pa_in_cb().set(FUNC(epson_ex800_base::pa_r));
	m_maincpu->pa_out_cb().set(FUNC(epson_ex800_base::pa_w));
	m_maincpu->pb_in_cb().set(FUNC(epson_ex800_base::pb_r));
	m_maincpu->pb_out_cb().set(FUNC(epson_ex800_base::pb_w));
	m_maincpu->pc_in_cb().set(FUNC(epson_ex800_base::pc_r));
	m_maincpu->pc_out_cb().set(FUNC(epson_ex800_base::pc_w));
	m_maincpu->co0_func().set(FUNC(epson_ex800_base::co0_w));
	m_maincpu->co1_func().set([this] (int state) { m_beeper->set_state(!state); });
	m_maincpu->txd_func().set([this] (int state) { LOGMASKED(LOG_IO, "%s TXD %d\n", machine().time().as_string(), state); out_txd(state); });
	m_maincpu->rxd_func().set([this] () { return m_rxd; });
	m_maincpu->an0_func().set(FUNC(epson_ex800_base::an0_r));
	m_maincpu->an1_func().set(FUNC(epson_ex800_base::an1_r));
	m_maincpu->an2_func().set_constant(0x80);   // printhead timing adjustment
	m_maincpu->an3_func().set_constant(0x80);
	m_maincpu->an4_func().set_constant(0x80);   // printhead temperature, normal
	m_maincpu->an5_func().set_constant(0x00);   // colour option home sensor
	m_maincpu->an6_func().set_constant(0xd0);   // supply voltage, in range
	m_maincpu->an7_func().set_constant(0x80);   // colour option home sensor adjustment

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);

	config.set_default_layout(layout_ex800);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beeper, 4000); // measured at 4000 Hz
	m_beeper->add_route(ALL_OUTPUTS, "mono", 0.5);
}

void epson_ex800_base::device_start()
{
	save_item(NAME(m_data));
	save_item(NAME(m_autofd));
	save_item(NAME(m_slctin));
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
	save_item(NAME(m_strobe_flag));
	save_item(NAME(m_init));
	save_item(NAME(m_head));
	save_item(NAME(m_7a_pc));
	save_item(NAME(m_pa));
	save_item(NAME(m_pb));
	save_item(NAME(m_co0));
	save_item(NAME(m_rxd));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_phase));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_phase));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}

void epson_ex800_base::device_reset()
{
	m_beeper->set_state(0);
	m_strobe_flag = false;
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_power_on = machine().time();
	m_power_cycled = false;
	m_maincpu->set_input_line(UPD7810_INTF1, CLEAR_LINE);
	m_maincpu->set_input_line(INPUT_LINE_NMI, BIT(m_dsw2->read(), 0) ? ASSERT_LINE : CLEAR_LINE);
	update_busy();
	out_ack(1);
}

void epson_ex800_base::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
	m_strobe_flag = false;
	m_power_on = machine().time();
	m_power_cycled = false;
	m_maincpu->set_input_line(INPUT_LINE_NMI, BIT(m_dsw2->read(), 0) ? ASSERT_LINE : CLEAR_LINE);
	update_busy();
}

INPUT_CHANGED_MEMBER(epson_ex800_base::power_switch)
{
	// switching the printer off and on again restarts only the printer
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

INPUT_CHANGED_MEMBER(epson_ex800_base::online_key)
{
	// the ON LINE key is on INT1, rising edge
	m_maincpu->set_input_line(UPD7810_INTF1, newval ? CLEAR_LINE : ASSERT_LINE);
}

INPUT_CHANGED_MEMBER(epson_ex800_base::nmi_switch)
{
	// DIP switch 2-1 is on the NMI pin (low when on)
	m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
}

u8 epson_ex800_base::keys_held() const
{
	// keys held while the printer is switched on with its power switch (bits as in PANEL)
	static u8 const held[4] = { 0x00, 0x04, 0x02, 0x06 };
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return held[(m_config->read() >> 1) & 3];
	return 0;
}

bool epson_ex800_base::dip(int sw) const
{
	// DIP switch 1-n or 2-n, given as 10 + n or 20 + n
	return (sw < 20) ? BIT(m_dsw1->read(), sw - 11) : BIT(m_dsw2->read(), sw - 21);
}


//-------------------------------------------------
//  host interface
//-------------------------------------------------

void epson_ex800_base::strobe_w(int state)
{
	// gate array 5a latches the data at the leading edge of /STROBE
	if (m_strobe && !state)
	{
		m_latch = m_data;
		m_strobe_flag = true;
		update_busy();
	}
	m_strobe = state;
}

void epson_ex800_base::init_w(int state)
{
	// /INIT resets the printer controller
	if (m_init && !state)
		restart();
	m_init = state;
}

void epson_ex800_base::update_busy()
{
	bool const busy = BIT(m_pb, 4) || m_strobe_flag;
	out_busy(busy);
	m_ready_led = !busy;
}


//-------------------------------------------------
//  CPU ports
//-------------------------------------------------

u8 epson_ex800_base::pa_r()
{
	return m_pa;
}

void epson_ex800_base::pa_w(u8 data)
{
	u8 const old = m_pa;
	m_pa = data;

	// carriage stepper on PA3-PA0: A 9 5 6 is one direction
	if ((old ^ data) & 0x0f)
	{
		LOGMASKED(LOG_MOTOR, "%s CR %x %x pos %d\n", machine().time().as_string(), data & 0x0f, (data >> 4) & 3, m_cr_pos);
		static int const phase[16] = { -1, -1, -1, -1, -1, 2, 3, -1, -1, 1, 0, -1, -1, -1, -1, -1 };
		int const p = phase[data & 0x0f];
		if (p >= 0)
		{
			if (m_cr_phase >= 0)
			{
				int const delta = (p - m_cr_phase) & 3;
				if (delta == 1)
					carriage_step(1);
				else if (delta == 3)
					carriage_step(-1);
			}
			m_cr_phase = p;
		}
	}
	if ((old ^ data) & 0xc0)
		LOGMASKED(LOG_IO, "%s PA bank %x\n", machine().describe_context(), data >> 6);
}

u8 epson_ex800_base::pb_r()
{
	return m_pb;
}

void epson_ex800_base::pb_w(u8 data)
{
	u8 const old = m_pb;
	m_pb = data;

	// paper stepper: Gray code on PB1:PB0, energised by PB2 = 0, 1 step = 1/432"
	if ((old ^ data) & 0x03)
	{
		LOGMASKED(LOG_MOTOR, "%s PF %x %x\n", machine().time().as_string(), data & 3, BIT(data, 2));
		int const phase = gray_phase[data & 3];
		if (m_pf_phase >= 0)
		{
			int const delta = (phase - m_pf_phase) & 3;
			if (delta == 3)         // 3 2 1 0 feeds forward
				m_bitmap_printer->m_ypos++;
			else if (delta == 1)
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
		m_pf_phase = phase;
	}

	m_online_led = !BIT(data, 3);
	m_paper_led = !BIT(data, 7);
	if ((old ^ data) & 0x10)
		update_busy();
	if ((old ^ data) & 0x20)
		out_fault(BIT(data, 5));
	if ((old ^ data) & 0x40)
		out_ack(BIT(data, 6));
	out_select(BIT(data, 3));
}

u8 epson_ex800_base::pc_r()
{
	return 0xff;
}

void epson_ex800_base::pc_w(u8 data)
{
}

void epson_ex800_base::co0_w(int state)
{
	// the needles fire while FIRE is low
	if (m_co0 && !state)
		fire();
	m_co0 = state;
}

u8 epson_ex800_base::an0_r()
{
	// DIP switch matrix, even switches (0 = on)
	static int const sw[8] = { 12, 14, 16, 18, 23, 26, 0, 0 };
	for (int i = 0; i < 6; i++)
		if (!BIT(m_7a_pc, i))
			return dip(sw[i]) ? 0x00 : 0xff;
	return dip(28) ? 0x00 : 0xff;
}

u8 epson_ex800_base::an1_r()
{
	// DIP switch matrix, odd switches (0 = on)
	static int const sw[8] = { 11, 13, 15, 17, 22, 25, 0, 0 };
	for (int i = 0; i < 6; i++)
		if (!BIT(m_7a_pc, i))
			return dip(sw[i]) ? 0x00 : 0xff;
	return dip(27) ? 0x00 : 0xff;
}


//-------------------------------------------------
//  memory mapped I/O
//-------------------------------------------------

u8 epson_ex800_base::devsel_r(offs_t offset)
{
	LOGMASKED(LOG_IO, "%s DEVSEL R %x\n", machine().describe_context(), offset);
	return 0xff;
}

void epson_ex800_base::devsel_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_IO, "%s DEVSEL W %x %02x\n", machine().describe_context(), offset, data);
}

u8 epson_ex800_base::gate5a_r(offs_t offset)
{
	u8 data = 0xff;
	switch (offset & 7)
	{
	case 0:
		data = m_latch;
		if (!machine().side_effects_disabled() && m_strobe_flag)
		{
			m_strobe_flag = false;
			update_busy();
		}
		break;
	case 1:
		data = 0xfe | (m_strobe_flag ? 1 : 0);
		break;
	default:
		LOGMASKED(LOG_IO, "%s GATE5A R %x\n", machine().describe_context(), offset);
		break;
	}
	return data;
}

void epson_ex800_base::gate5a_w(offs_t offset, u8 data)
{
	switch (offset & 7)
	{
	case 2: // needles 1-8, 0 = fire
		m_head = (m_head & 0x100) | bitswap<8>(~data, 0, 1, 2, 3, 4, 5, 6, 7);
		break;
	case 3: // needle 9
		m_head = (m_head & 0x0ff) | (BIT(data, 7) ? 0 : 0x100);
		break;
	default:
		LOGMASKED(LOG_IO, "%s GATE5A W %x %02x\n", machine().describe_context(), offset, data);
		break;
	}
}

u8 epson_ex800_base::iosel_r(offs_t offset)
{
	LOGMASKED(LOG_IO, "%s IOSEL R %x\n", machine().describe_context(), offset);
	return 0xff;
}

void epson_ex800_base::iosel_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_IO, "%s IOSEL W %x %02x\n", machine().describe_context(), offset, data);
}

u8 epson_ex800_base::gate7a_r(offs_t offset)
{
	switch (offset & 3)
	{
	case 0: // SelecType keys
		return m_selectype->read();

	case 1:
		{
			u8 const keys = m_panel->read() & ~keys_held();
			bool const paper = BIT(m_config->read(), 0) && m_bitmap_printer->paper_present();
			u8 data = 0x20;
			if (BIT(keys, 1))
				data |= 0x01;   // FORM FEED
			if (BIT(keys, 2))
				data |= 0x02;   // LINE FEED
			if (BIT(keys, 3))
				data |= 0x04;   // LOAD/EJECT
			if (paper)
				data |= 0x08;
			if (m_cr_pos >= 0)
				data |= 0x10;   // home sensor, 0 in the home zone
			if (m_autofd)
				data |= 0x40;   // /AUTO FEED XT
			if (!dip(24))
				data |= 0x80;   // DIP switch 2-4, 0 = on (line feed after CR)
			return data;
		}

	default:
		LOGMASKED(LOG_IO, "%s GATE7A R %x\n", machine().describe_context(), offset);
		return 0xff;
	}
}

void epson_ex800_base::gate7a_w(offs_t offset, u8 data)
{
	switch (offset & 3)
	{
	case 2: // SelecType lamps, DIP switch matrix
		m_7a_pc = data;
		for (int i = 0; i < 8; i++)
			m_selectype_led[i] = !BIT(data, i);
		break;
	case 3: // mode (04h)
		break;
	default:
		LOGMASKED(LOG_IO, "%s GATE7A W %x %02x\n", machine().describe_context(), offset, data);
		break;
	}
}


//-------------------------------------------------
//  carriage and printhead
//-------------------------------------------------

void epson_ex800_base::carriage_step(int dir)
{
	attotime const now = machine().time();
	attotime const since = now - m_cr_last_step;

	m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
	m_cr_last_step = now;
	m_cr_dir = dir;
	m_cr_pos = std::clamp(m_cr_pos + dir, CR_LEFT, CR_RIGHT);
	m_bitmap_printer->setheadpos(int((m_cr_pos / 60.0 + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);
}

double epson_ex800_base::head_x() const
{
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return m_cr_pos;
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return m_cr_pos - m_cr_dir * (1.0 - frac + CR_LAG / m_cr_period.as_double());
}

void epson_ex800_base::fire()
{
	LOGMASKED(LOG_FIRE, "%s fire %03x x %.2f y %d\n", machine().time().as_string(), m_head, head_x(), m_bitmap_printer->m_ypos);
	if (!m_head)
		return;

	int const x = int(std::lround((head_x() / 60.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// needles are 1/72" = 3 paper steps apart
	for (int needle = 0; needle < 9; needle++)
		if (BIT(m_head, needle))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + needle * (VDPI / 72), 0x000000);
}


//**************************************************************************
//  INTERFACES
//**************************************************************************

// one device for both built-in interfaces: plugged into a parallel port it
// starts with the interface DIP switches on parallel, into a serial port on serial
class epson_ex800_device : public epson_ex800_base, public device_centronics_peripheral_interface, public device_rs232_port_interface
{
public:
	epson_ex800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		epson_ex800_base(mconfig, EPSON_EX800, tag, owner, clock),
		device_centronics_peripheral_interface(mconfig, *this),
		device_rs232_port_interface(mconfig, *this),
		m_parallel(dynamic_cast<centronics_device *>(owner) != nullptr),
		m_serial(dynamic_cast<rs232_port_device *>(owner) != nullptr)
	{
	}

	// parallel interface
	virtual void input_strobe(int state) override { strobe_w(state); }
	virtual void input_init(int state) override { init_w(state); }
	virtual void input_autofd(int state) override { m_autofd = state; }
	virtual void input_select_in(int state) override { m_slctin = state; }
	virtual void input_data0(int state) override { if (state) m_data |= 0x01; else m_data &= ~0x01; }
	virtual void input_data1(int state) override { if (state) m_data |= 0x02; else m_data &= ~0x02; }
	virtual void input_data2(int state) override { if (state) m_data |= 0x04; else m_data &= ~0x04; }
	virtual void input_data3(int state) override { if (state) m_data |= 0x08; else m_data &= ~0x08; }
	virtual void input_data4(int state) override { if (state) m_data |= 0x10; else m_data &= ~0x10; }
	virtual void input_data5(int state) override { if (state) m_data |= 0x20; else m_data &= ~0x20; }
	virtual void input_data6(int state) override { if (state) m_data |= 0x40; else m_data &= ~0x40; }
	virtual void input_data7(int state) override { if (state) m_data |= 0x80; else m_data &= ~0x80; }

	// serial interface: data from the computer arrives on the printer's RxD
	virtual void input_txd(int state) override { rxd_w(state); }

protected:
	virtual ioport_constructor device_input_ports() const override ATTR_COLD
	{
		return m_serial ? INPUT_PORTS_NAME( epson_ex800_serial ) : INPUT_PORTS_NAME( epson_ex800 );
	}
	virtual bool supports_pin35_5v() override { return true; }

	virtual void device_reset() override ATTR_COLD
	{
		epson_ex800_base::device_reset();
		if (m_serial)
		{
			output_rxd(1);
			output_dcd(0);
		}
	}

	virtual void out_busy(int state) override
	{
		if (m_parallel)
			output_busy(state);
		if (m_serial)
		{
			// the printer's BUSY goes to the computer as DSR and CTS
			output_dsr(state);
			output_cts(state);
		}
	}
	virtual void out_ack(int state) override { if (m_parallel) output_ack(state); }
	virtual void out_perror(int state) override { if (m_parallel) output_perror(state); }
	virtual void out_fault(int state) override { if (m_parallel) output_fault(state); }
	virtual void out_select(int state) override { if (m_parallel) output_select(state); }
	virtual void out_txd(int state) override { if (m_serial) output_rxd(state); }

private:
	bool const m_parallel;
	bool const m_serial;
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(EPSON_EX800, device_t, epson_ex800_device, "ex800", "Epson EX-800")
