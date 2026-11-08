// license:BSD-3-Clause
// copyright-holders:Ramiro Polla, Felipe Sanches, rfka01
/*
 * Epson LX-810L dot matrix printer emulation
 *
 * IC list:
 *   uPD7810HG (cpu)
 *   E05A30 (gate array)
 *   2064C (8k RAM)
 *   ER59256 (EEP-ROM - serial nvram)
 *   SLA7020M (step motor driver)
 *   uPC494C (pulse width modulation control)
 *
 * Carriage: 4-phase stepper behind an SLA7020M driver, 1 step = 1/60",
 * 2-2 phase drive in draft, 1-2 phase (half steps) for NLQ. Paper:
 * 4-phase stepper, 1 step = 1/216". The needles fire while FIRE (CO0)
 * is low.
 *
 * Self test: hold LINE FEED (draft) or FORM FEED (NLQ) while switching
 * the printer on, both keys for the data dump. The keys can also be
 * selected under "Keys held at power on" and the power switch toggled.
 */

#include "emu.h"
#include "epson_lx810l.h"
#include "speaker.h"

#include "epson_ap2000.lh"
#include "epson_lx810l.lh"

#define LOG_MOTOR (1U << 1)
#define LOG_FIRE  (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_MOTOR | LOG_FIRE)
#include "logmacro.h"

namespace {

constexpr int HDPI = 240;               // horizontal resolution of the paper bitmap
constexpr int VDPI = 216;               // vertical resolution (one paper step)
constexpr double PAPER_WIDTH = 9.5;     // 240 mm fanfold with tractor strips
constexpr double PAPER_HEIGHT = 11.0;
constexpr double LEFT_OFFSET = 0.115;   // paper edge to the home sensor edge: column 1 13 mm from the edge
										// of continuous paper (user's guide: left margin 13 mm)
constexpr double SHEET_TOP = 8.5 / 25.4;   // the loading feed leaves the first line 8.5 mm from the top (minimum top margin)
constexpr double SHEET_SENSOR = 1.78 - SHEET_TOP;   // single sheets: paper end sensor to print line, from the loading feed of 1.78"
constexpr double CR_LAG = 0.000167;     // the head lags the rotor by this time (s) plus CR_SHIFT steps;
constexpr double CR_SHIFT = 0.15;       // both found from bidirectional draft (full steps) and NLQ (half steps)
constexpr int CR_LEFT = -40;            // left stop, steps from the home sensor edge
constexpr int CR_RIGHT = 560;           // right stop

// electrical angle (in half steps, 0-7) of the energised windings, -1 if none or invalid
// carriage: SLA7020M outputs A (bit 0), /A (bit 1), B (bit 2), /B (bit 3)
constexpr int cr_angle_table[16] = { -1,  6,  2, -1,  0,  7,  1, -1,  4,  5,  3, -1, -1, -1, -1, -1 };
// paper: phase outputs of the gate array
constexpr int pf_angle_table[16] = { -1,  4,  0, -1,  2,  3,  1, -1,  6,  5,  7, -1, -1, -1, -1, -1 };

// change of the angle, -4..3 half steps
int angle_delta(int from, int to)
{
	return ((to - from + 4) & 7) - 4;
}

} // anonymous namespace



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(EPSON_LX810L, epson_lx810l_device, "lx810l", "Epson LX-810L")
DEFINE_DEVICE_TYPE(EPSON_AP2000, epson_ap2000_device, "ap2000", "Epson ActionPrinter 2000")


//-------------------------------------------------
//  ROM( lx810l )
//-------------------------------------------------

ROM_START( lx810l )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("lx810l.ic3c", 0x0000, 0x8000, CRC(a66454e1) SHA1(8e6f2f98abcbd8af6e34b9ba746edf0d18aef843) )
	ROM_REGION16_BE(0x20, "eeprom", 0)
	ROM_LOAD( "at93c06", 0x00, 0x20, NO_DUMP )
ROM_END


//-------------------------------------------------
//  ROM( ap2000 )
//-------------------------------------------------

ROM_START( ap2000 )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("ap2k.ic3c", 0x0000, 0x8000, CRC(ee7294b7) SHA1(219ffa6ff661ce95d5772c9fc1967093718f04e9) )
	ROM_REGION16_BE(0x20, "eeprom", 0)
	ROM_LOAD( "at93c06", 0x00, 0x20, NO_DUMP )
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *epson_lx810l_device::device_rom_region() const
{
	return ROM_NAME( lx810l );
}


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *epson_ap2000_device::device_rom_region() const
{
	return ROM_NAME( ap2000 );
}


//-------------------------------------------------
//  ADDRESS_MAP( lx810l_mem )
//-------------------------------------------------

void epson_lx810l_device::lx810l_mem(address_map &map)
{
	map(0x0000, 0x7fff).rom(); /* 32k firmware */
	map(0x8000, 0x9fff).ram(); /* 8k external RAM */
	map(0xa000, 0xbfff).rw(FUNC(epson_lx810l_device::fakemem_r), FUNC(epson_lx810l_device::fakemem_w)); /* fake memory, write one, set all */
	map(0xc000, 0xc00f).mirror(0x1ff0).rw("e05a30", FUNC(e05a30_device::read), FUNC(e05a30_device::write));
	map(0xe000, 0xfeff).noprw(); /* not used */
}


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void epson_lx810l_device::device_add_mconfig(machine_config &config)
{
	/* basic machine hardware */
	upd7810_device &upd(UPD7810(config, m_maincpu, 14.7456_MHz_XTAL));
	upd.set_addrmap(AS_PROGRAM, &epson_lx810l_device::lx810l_mem);
	upd.pa_in_cb().set(FUNC(epson_lx810l_device::porta_r));
	upd.pa_out_cb().set(FUNC(epson_lx810l_device::porta_w));
	upd.pb_in_cb().set(FUNC(epson_lx810l_device::portb_r));
	upd.pb_out_cb().set(FUNC(epson_lx810l_device::portb_w));
	upd.pc_in_cb().set(FUNC(epson_lx810l_device::portc_r));
	upd.pc_out_cb().set(FUNC(epson_lx810l_device::portc_w));
	upd.an0_func().set(FUNC(epson_lx810l_device::an0_r));
	upd.an1_func().set(FUNC(epson_lx810l_device::an1_r));
	upd.an2_func().set(FUNC(epson_lx810l_device::an2_r));
	upd.an3_func().set(FUNC(epson_lx810l_device::an3_r));
	upd.an4_func().set(FUNC(epson_lx810l_device::an4_r));
	upd.an5_func().set(FUNC(epson_lx810l_device::an5_r));
	upd.an6_func().set(FUNC(epson_lx810l_device::an6_r));
	upd.an7_func().set(FUNC(epson_lx810l_device::an7_r));
	upd.co0_func().set(FUNC(epson_lx810l_device::co0_w));
	upd.co1_func().set("dac", FUNC(dac_bit_interface::write));

	config.set_default_layout(layout_epson_lx810l);

	/* audio hardware */
	SPEAKER(config, "speaker").front_center();
	DAC_1BIT(config, "dac", 0).add_route(ALL_OUTPUTS, "speaker", 0.25);

	/* gate array */
	e05a30_device &e05a30(E05A30(config, m_e05a30));
	e05a30.printhead().set(FUNC(epson_lx810l_device::printhead));
	e05a30.pf_stepper().set(FUNC(epson_lx810l_device::pf_stepper));
	e05a30.cr_stepper().set(FUNC(epson_lx810l_device::cr_stepper));
	e05a30.ready().set(FUNC(epson_lx810l_device::e05a30_ready));
	e05a30.centronics_ack().set(FUNC(epson_lx810l_device::e05a30_centronics_ack));
	e05a30.centronics_busy().set(FUNC(epson_lx810l_device::e05a30_centronics_busy));
	e05a30.centronics_perror().set(FUNC(epson_lx810l_device::e05a30_centronics_perror));
	e05a30.centronics_fault().set(FUNC(epson_lx810l_device::e05a30_centronics_fault));
	e05a30.centronics_select().set(FUNC(epson_lx810l_device::e05a30_centronics_select));
	e05a30.cpu_reset().set(FUNC(epson_lx810l_device::e05a30_cpu_reset));
	e05a30.ready_led().set(FUNC(epson_lx810l_device::e05a30_ready_led));

	/* 256-bit eeprom */
	EEPROM_93C06_16BIT(config, "eeprom");

	BITMAP_PRINTER(config, m_bitmap_printer, int(PAPER_WIDTH * HDPI), int(PAPER_HEIGHT * VDPI), HDPI, VDPI);
	m_bitmap_printer->set_dot_style(printer_dot_style::impact_9pin());
	m_bitmap_printer->set_paper_view(3.0);
	m_bitmap_printer->set_paper_formats(0x0f, bitmap_printer_device::PAPER_FANFOLD_11);
	m_bitmap_printer->set_sheet_load_offset(SHEET_SENSOR);
	m_bitmap_printer->set_sheet_top(SHEET_TOP);
}

/***************************************************************************
    INPUT PORTS
***************************************************************************/


static INPUT_PORTS_START( epson_lx810 )
	/* Buttons on printer */
	PORT_START("ONLINE")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("On Line") PORT_CODE(KEYCODE_0_PAD) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx810l_device::online_sw), 0)
	PORT_START("FORMFEED")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Form Feed") PORT_CODE(KEYCODE_7_PAD)
	PORT_START("LINEFEED")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Line Feed") PORT_CODE(KEYCODE_9_PAD)
	PORT_START("LOADEJECT")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load/Eject") PORT_CODE(KEYCODE_1_PAD)
	PORT_START("POWER")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Power switch (off/on)") PORT_CODE(KEYCODE_2_PAD) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(epson_lx810l_device::power_switch), 0)

	PORT_START("CONFIG")
	PORT_CONFNAME( 0x01, 0x01, "Paper" )
	PORT_CONFSETTING(    0x00, "Empty" )
	PORT_CONFSETTING(    0x01, "Loaded" )
	PORT_CONFNAME( 0x06, 0x00, "Keys held at power on" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x02, "LINE FEED (self test, draft)" )
	PORT_CONFSETTING(    0x04, "FORM FEED (self test, NLQ)" )
	PORT_CONFSETTING(    0x06, "LINE FEED + FORM FEED (data dump)" )

	PORT_START("DIPSW1")
	PORT_DIPNAME(0x01, 0x00, "Character spacing")             PORT_DIPLOCATION("SW 1:!1")
	PORT_DIPSETTING(   0x00, "10 cpi")
	PORT_DIPSETTING(   0x01, "12 cpi")
	PORT_DIPNAME(0x02, 0x00, "Shape of zero")                 PORT_DIPLOCATION("SW 1:!2")
	PORT_DIPSETTING(   0x00, "Not slashed")
	PORT_DIPSETTING(   0x02, "Slashed")
	PORT_DIPNAME(0x04, 0x00, "Character table")               PORT_DIPLOCATION("SW 1:!3")
	PORT_DIPSETTING(   0x00, "Italics")
	PORT_DIPSETTING(   0x04, "Graphics")
	PORT_DIPNAME(0x08, 0x08, "Short tear-off")                PORT_DIPLOCATION("SW 1:!4")
	PORT_DIPSETTING(   0x08, DEF_STR(Off))
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "Draft printing speed")          PORT_DIPLOCATION("SW 1:!6")
	PORT_DIPSETTING(   0x10, DEF_STR(Normal))
	PORT_DIPSETTING(   0x00, "High speed")
	PORT_DIPNAME(0xe0, 0xe0, "International character set")   PORT_DIPLOCATION("SW 1:!6,!7,!8")
	PORT_DIPSETTING(   0xe0, "USA (PC 437)")
	PORT_DIPSETTING(   0x60, "France (PC 850)")
	PORT_DIPSETTING(   0xa0, "Germany (PC 860)")
	PORT_DIPSETTING(   0x20, "UK (PC 863)")
	PORT_DIPSETTING(   0xc0, "Denmark (PC 865)")
	PORT_DIPSETTING(   0x40, "Sweden (PC 437)")
	PORT_DIPSETTING(   0x80, "Italy (PC 437)")
	PORT_DIPSETTING(   0x00, "Spain (PC 437)")

	PORT_START("DIPSW2")
	PORT_DIPNAME(0x01, 0x00, "Page length")                   PORT_DIPLOCATION("SW 2:!1")
	PORT_DIPSETTING(   0x00, "11\"")
	PORT_DIPSETTING(   0x01, "12\"")
	PORT_DIPNAME(0x02, 0x00, "Cut-sheet feeder mode")         PORT_DIPLOCATION("SW 2:!2")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Skip over perforation")         PORT_DIPLOCATION("SW 2:!3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, "1\"")
	PORT_DIPNAME(0x08, 0x00, "Auto line feed")                PORT_DIPLOCATION("SW 2:!4")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x08, DEF_STR(On))
INPUT_PORTS_END

static INPUT_PORTS_START( epson_lx810l )
	PORT_INCLUDE(epson_lx810)

	PORT_MODIFY("DIPSW1")
	PORT_DIPNAME(0x0c, 0x00, "Page length")                   PORT_DIPLOCATION("SW 1:!3,!4")
	PORT_DIPSETTING(   0x08, "8.5\"")
	PORT_DIPSETTING(   0x00, "11\"")
	PORT_DIPSETTING(   0x0c, "11.7\" (A4)")
	PORT_DIPSETTING(   0x04, "12\"")
	PORT_DIPNAME(0x10, 0x00, "Character table")               PORT_DIPLOCATION("SW 1:!5")
	PORT_DIPSETTING(   0x00, "Italics")
	PORT_DIPSETTING(   0x10, "Graphics")

	PORT_MODIFY("DIPSW2")
	PORT_DIPNAME(0x01, 0x01, "Short tear-off")                PORT_DIPLOCATION("SW 2:!1")
	PORT_DIPSETTING(   0x01, "Invalid")
	PORT_DIPSETTING(   0x00, "Valid")
INPUT_PORTS_END



//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor epson_lx810l_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(epson_lx810l);
}

INPUT_CHANGED_MEMBER(epson_lx810l_device::online_sw)
{
	m_maincpu->set_input_line(UPD7810_INTF2, newval ? CLEAR_LINE : ASSERT_LINE);
}

INPUT_CHANGED_MEMBER(epson_lx810l_device::power_switch)
{
	if (newval)
	{
		restart();
		m_power_cycled = true;
	}
}

void epson_lx810l_device::restart()
{
	m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);  // reset cpu
	m_e05a30->reset();  // this will generate an NMI interrupt when the e05a30 is ready (minimum 0.9 seconds after reset)
	m_power_on = machine().time();
	m_power_cycled = false;
}

uint8_t epson_lx810l_device::keys_held() const
{
	// keys held while the printer is switched on: bit 0 LINE FEED, bit 1 FORM FEED
	if (m_power_cycled && (machine().time() - m_power_on < attotime::from_seconds(3)))
		return (m_config_ioport->read() >> 1) & 3;
	return 0;
}

bool epson_lx810l_device::paper_sensor() const
{
	return BIT(m_config_ioport->read(), 0) && m_bitmap_printer->paper_present();
}




//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  epson_lx810l_device - constructor
//-------------------------------------------------

epson_lx810l_device::epson_lx810l_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	epson_lx810l_device(mconfig, EPSON_LX810L, tag, owner, clock)
{
}

epson_lx810l_device::epson_lx810l_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_centronics_peripheral_interface(mconfig, *this),
	m_maincpu(*this, "maincpu"),
	m_bitmap_printer(*this, "bitmap_printer"),
	m_eeprom(*this, "eeprom"),
	m_e05a30(*this, "e05a30"),
	m_online_led(*this, "lx810l_online_led"),
	m_ready_led(*this, "lx810l_ready_led"),
	m_paperout_led(*this, "lx810l_paperout_led"),
	m_online_ioport(*this, "ONLINE"),
	m_formfeed_ioport(*this, "FORMFEED"),
	m_linefeed_ioport(*this, "LINEFEED"),
	m_loadeject_ioport(*this, "LOADEJECT"),
	m_dipsw1_ioport(*this, "DIPSW1"),
	m_dipsw2_ioport(*this, "DIPSW2"),
	m_config_ioport(*this, "CONFIG"),
	m_93c06_clk(0),
	m_93c06_cs(0),
	m_printhead(0),
	m_fakemem(0),
	m_co0(1),
	m_cr_pos(100),
	m_cr_half(200),
	m_cr_angle(-1),
	m_cr_dir(1),
	m_cr_step_start(200),
	m_pf_angle(-1),
	m_pf_half(0),
	m_power_cycled(false)
{
}

epson_ap2000_device::epson_ap2000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: epson_lx810l_device(mconfig, EPSON_AP2000, tag, owner, clock)
{ }

void epson_ap2000_device::device_add_mconfig(machine_config &config)
{
	epson_lx810l_device::device_add_mconfig(config);
	config.set_default_layout(layout_epson_ap2000);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void epson_lx810l_device::device_start()
{
	save_item(NAME(m_93c06_clk));
	save_item(NAME(m_93c06_cs));
	save_item(NAME(m_printhead));
	save_item(NAME(m_fakemem));
	save_item(NAME(m_co0));
	save_item(NAME(m_cr_pos));
	save_item(NAME(m_cr_half));
	save_item(NAME(m_cr_angle));
	save_item(NAME(m_cr_dir));
	save_item(NAME(m_cr_step_start));
	save_item(NAME(m_cr_last_step));
	save_item(NAME(m_cr_last_change));
	save_item(NAME(m_cr_period));
	save_item(NAME(m_pf_angle));
	save_item(NAME(m_pf_half));
	save_item(NAME(m_power_on));
	save_item(NAME(m_power_cycled));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void epson_lx810l_device::device_reset()
{
	m_cr_period = attotime::never;
	m_cr_last_step = machine().time();
	m_cr_last_change = m_cr_last_step;
	m_power_on = machine().time();
	m_power_cycled = false;
}


/***************************************************************************
    FAKEMEM READ/WRITE
***************************************************************************/

uint8_t epson_lx810l_device::fakemem_r()
{
	return m_fakemem;
}

void epson_lx810l_device::fakemem_w(uint8_t data)
{
	m_fakemem = data;
}


/***************************************************************************
    I/O PORTS
***************************************************************************/

/*
 * PA0  R   CN7 sensor (Home Position, HP, active low)
 * PA1  R   CN6 sensor (Paper-End, PE, active low)
 * PA2  R   CN4 sensor (Release, low = tractor)
 * PA3   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA4   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA5   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA6  R   Line Feed SWITCH
 * PA7  R   Form Feed SWITCH
 */
uint8_t epson_lx810l_device::porta_r(offs_t offset)
{
	uint8_t result = 0;
	uint8_t const held = keys_held();
	bool const paper = paper_sensor();

	if (m_cr_pos > 0)
		result |= 0x01;     /* home position sensor, low in the home zone */
	if (paper)
		result |= 0x02;     /* paper end sensor, low without paper */
	if (m_bitmap_printer->sheet_handling())
		result |= 0x04;     /* release lever, low = tractor */
	if (m_linefeed_ioport->read() && !BIT(held, 0))
		result |= 0x40;
	if (m_formfeed_ioport->read() && !BIT(held, 1))
		result |= 0x80;

	LOG("%s: lx810l_PA_r(%02x): result %02x\n", machine().describe_context(), offset, result);

	m_bitmap_printer->set_led_state(bitmap_printer_device::LED_ERROR, !paper);
	m_paperout_led = !paper;

	return result;
}

void epson_lx810l_device::porta_w(offs_t offset, uint8_t data)
{
	LOG("%s: lx810l_PA_w(%02x): %02x: stepper vref %d\n", machine().describe_context(), offset, data, BIT(data, 3) | (BIT(data, 4)<<1) | (BIT(data, 5)<<2));
}

/*
 * PB0  R   DIP1.0 & 93C06.DO
 * PB1  RW  DIP1.1 & 93C06.DI
 * PB2  R   DIP1.2
 * PB3  R   DIP1.3
 * PB4  R   DIP1.4
 * PB5  R   DIP1.5
 * PB6  R   DIP1.6
 * PB7  R   DIP1.7
 */
uint8_t epson_lx810l_device::portb_r(offs_t offset)
{
	uint8_t result = ~m_dipsw1_ioport->read();

	/* PB0 is DIP switch 1-1 wired-AND with the 93C06 data output, which is
	   high (pulled up) unless the EEPROM drives a 0 */
	if (m_93c06_cs && !m_eeprom->do_read())
		result &= 0xfe;

	LOG("%s: lx810l_PB_r(%02x): result %02x\n", machine().describe_context(), offset, result);

	return result;
}

void epson_lx810l_device::portb_w(offs_t offset, uint8_t data)
{
	uint8_t data_in = BIT(data, 1);

	/* if 93C06 is selected */
	if (m_93c06_cs)
		m_eeprom->di_write(data_in);

	LOG("%s: lx810l_PB_w(%02x): %02x: 93c06 data %d\n", machine().describe_context(), offset, data, data_in);
}

/*
 * PC0   W  TXD        serial i/o txd, also TAMA.25
 * PC1  R   RXD        serial i/o rxd, also E05A30.28
 * PC2   W  paper feed motor drive (1) or hold (0) current
 * PC3  R   ONLINE SW  online switch
 * PC4   W  93C06.SK
 * PC5   W  93C06.CS
 * PC6   W  FIRE       drive pulse width signal, also E05A30.57
 * PC7   W  BUZZER     buzzer signal
 */
uint8_t epson_lx810l_device::portc_r(offs_t offset)
{
	uint8_t result = 0;

	/* result |= ioport("serial")->read() << 1; */
	result |= !m_online_ioport->read() << 3;
	result |= m_93c06_clk << 4;
	result |= m_93c06_cs  << 5;

	LOG("%s: lx810l_PC_r(%02x): %02x\n", machine().describe_context(), offset, result);

	return result;
}

void epson_lx810l_device::portc_w(offs_t offset, uint8_t data)
{
	/* ioport("serial")->write(BIT(data, 0)); */

	m_93c06_clk =  BIT(data, 4);
	m_93c06_cs  = !BIT(data, 5);

	LOG("%s: PC_w(%02x): %02x 93c06 clk: %d cs: %d\n", machine().describe_context(), offset, data, m_93c06_clk, m_93c06_cs);

	m_eeprom->clk_write(m_93c06_clk ? ASSERT_LINE : CLEAR_LINE);
	m_eeprom->cs_write (m_93c06_cs  ? ASSERT_LINE : CLEAR_LINE);

}


/***************************************************************************
    GATE ARRAY
***************************************************************************/

void epson_lx810l_device::printhead(uint16_t data)
{
	m_printhead = data;
}

void epson_lx810l_device::pf_stepper(uint8_t data)
{
	LOGMASKED(LOG_MOTOR, "%s PF %x\n", machine().time().as_string(), data);

	int const angle = pf_angle_table[data & 0x0f];
	if (angle < 0)
		return;
	if (m_pf_angle >= 0)
	{
		int const old = m_pf_half >> 1;
		m_pf_half += angle_delta(m_pf_angle, angle);   // a rising angle feeds the paper forward
		int const now = m_pf_half >> 1;
		if (now != old)
		{
			if (now > old)
				m_bitmap_printer->m_ypos++;
			else
				m_bitmap_printer->m_ypos = std::max(m_bitmap_printer->min_ypos(), m_bitmap_printer->m_ypos - 1);
			m_bitmap_printer->check_new_page();
		}
	}
	m_pf_angle = angle;
}

void epson_lx810l_device::cr_stepper(uint8_t data)
{
	LOGMASKED(LOG_MOTOR, "%s CR %x pos %d\n", machine().time().as_string(), data, m_cr_pos);

	// a rising angle moves the carriage to the right
	int const angle = cr_angle_table[data & 0x0f];
	if (angle < 0)
		return;
	if (m_cr_angle >= 0)
	{
		int const delta = angle_delta(m_cr_angle, angle);
		if (delta)
		{
			int const old = m_cr_half;
			m_cr_half = std::clamp(m_cr_half + delta, CR_LEFT * 2, CR_RIGHT * 2 + 1);   // the motor slips at the stops
			carriage_step((delta > 0) ? 1 : -1, old);
		}
	}
	m_cr_angle = angle;
}

void epson_lx810l_device::carriage_step(int dir, int old_half)
{
	// a 2-2 phase step (draft) is written in two halves a few microseconds apart;
	// the second half belongs to the same step. NLQ moves in real half steps.
	attotime const now = machine().time();
	if ((now - m_cr_last_change) > attotime::from_usec(50))
	{
		attotime const since = now - m_cr_last_step;
		m_cr_period = (dir == m_cr_dir && since < attotime::from_msec(20)) ? since : attotime::never;
		m_cr_last_step = now;
		m_cr_dir = dir;
		m_cr_step_start = old_half;
	}
	m_cr_last_change = now;

	m_cr_pos = m_cr_half >> 1;
	m_bitmap_printer->setheadpos(int((m_cr_pos / 60.0 + LEFT_OFFSET) * HDPI), m_bitmap_printer->m_ypos);
}

double epson_lx810l_device::head_x() const
{
	// a step starts the move to the new rotor position, which is reached at the
	// next step when the carriage runs at constant speed
	double const pos = m_cr_half / 2.0;
	if (m_cr_period.is_never() || (m_cr_period <= attotime::zero))
		return pos;
	double const step = std::abs(m_cr_half - m_cr_step_start) / 2.0;   // 1 in draft, 1/2 in NLQ
	double const frac = std::min(1.0, (machine().time() - m_cr_last_step).as_double() / m_cr_period.as_double());
	return pos - m_cr_dir * (step * (1.0 - frac + CR_LAG / m_cr_period.as_double()) + CR_SHIFT);
}

void epson_lx810l_device::fire()
{
	LOGMASKED(LOG_FIRE, "%s fire %03x x %.2f y %d\n", machine().time().as_string(), m_printhead, head_x(), m_bitmap_printer->m_ypos);
	if (!m_printhead)
		return;

	int const x = int(std::lround((head_x() / 60.0 + LEFT_OFFSET) * HDPI));
	if (x < 0 || x >= int(PAPER_WIDTH * HDPI))
		return;

	// bit 8 is the top needle; needles are 1/72" = 3 paper steps apart
	for (int needle = 0; needle < 9; needle++)
		if (BIT(m_printhead, 8 - needle))
			m_bitmap_printer->draw_pixel(x, m_bitmap_printer->m_ypos + needle * (VDPI / 72), 0x000000);
}

void epson_lx810l_device::e05a30_ready(int state)
{
	// must be longer than attotime::zero - 0.09 is minimum to initialize properly
	m_maincpu->pulse_input_line(INPUT_LINE_NMI, attotime::from_double(0.09));
}


/***************************************************************************
    Extended Timer Output
***************************************************************************/

void epson_lx810l_device::co0_w(int state)
{
	// the needles fire while FIRE is low
	if (m_co0 && !state)
		fire();
	m_co0 = state;
}


/***************************************************************************
    ADC
***************************************************************************/

uint8_t epson_lx810l_device::an0_r()
{
	uint8_t res = !!(m_dipsw2_ioport->read() & 0x01);
	return res - 1; /* DIPSW2.1 */
}

uint8_t epson_lx810l_device::an1_r()
{
	uint8_t res = !!(m_dipsw2_ioport->read() & 0x02);
	return res - 1; /* DIPSW2.2 */
}

uint8_t epson_lx810l_device::an2_r()
{
	uint8_t res = !!(m_dipsw2_ioport->read() & 0x04);
	return res - 1; /* DIPSW2.3 */
}

uint8_t epson_lx810l_device::an3_r()
{
	uint8_t res = !!(m_dipsw2_ioport->read() & 0x08);
	return res - 1; /* DIPSW2.4 */
}

uint8_t epson_lx810l_device::an4_r()
{
	return 0xff;
}

uint8_t epson_lx810l_device::an5_r()
{
	return 0xCB; /* motor voltage, 0xcb = 24V */
}

uint8_t epson_lx810l_device::an6_r()
{
	uint8_t res = !m_loadeject_ioport->read();
	return res - 1;
}

uint8_t epson_lx810l_device::an7_r()
{
	return 0xff;
}
