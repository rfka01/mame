// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
/***************************************************************************

    K806 Mouse module

    Hardware (NCR TSC "System Information" HW1 7.2, 06.11.84):
    - Intel 8741A (ROM 33378) at 6 MHz, 74LS14 Schmitt-trigger inputs with
      10k pull-ups and 2.2nF filter capacitors on all mouse lines
    - 9-pin mouse connector P2:
        1 +5V       2 XA/Y1     3 XB/Y2     4 YA/X2     5 YB/X1
        6 S1/GND    7 S3        8 S2        9 GND/S1
      (first name: Hawley Mark II / Alps, second name: Depraz / Logitech)
    - P1.0 = S1 (left), P1.1 = S2 (right), P1.2 = S3 (middle)
      P1.3..P1.6 = connector pins 2..5, all inverted by the 74LS14
    - T1 = J8: open (OFF) for Hawley/Alps, grounded (ON) for Depraz/Logitech;
      J9/J10 swap S1 and GND between pins 6 and 9
    - P2.0 = INTR/
    - resolution (NCR diagnostic disk, 5 inches): Alps 400-600,
      Hawley Mark II and Logitech LM-P5 900-1100, Depraz Souris P4 and
      Logitech P4 1700-2000 counts; moving right and up gives positive values

    Firmware notes:
    - the main loop polls P1 and decodes every edge of both encoder
      channels (4 counts per quadrature cycle)
    - X: an edge on XA counts up when XA != XB afterwards, an edge on XB
      counts up when XA == XB afterwards; Y uses the opposite sense
    - with T1 low the firmware remaps the Depraz pin order
    - only one channel per encoder may change between two polls,
      otherwise the edge is lost or counted in the wrong direction
    - default range is X 0..640, Y 0..400

***************************************************************************/

#include "emu.h"
#include "k806.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

ROM_START( dmv_k806 )
	ROM_REGION( 0x0400, "mcu", 0 )
	ROM_LOAD( "dmv_mouse_8741a.bin", 0x0000, 0x0400, CRC(2163737a) SHA1(b82c14dba6c25cb1f60cf623989ca8c0c1ee4cc3))
ROM_END

static INPUT_PORTS_START( dmv_k806 )
	PORT_START("JUMPERS")
	PORT_DIPNAME( 0x7f, 0x24, "K806 IFSEL" )  PORT_DIPLOCATION("J:!1,J:!2,J:!3,J:!4,J:!5,J:!6,J:!7")
	PORT_DIPSETTING( 0x21, "0A" )
	PORT_DIPSETTING( 0x41, "0B" )
	PORT_DIPSETTING( 0x22, "1A" )
	PORT_DIPSETTING( 0x42, "1B" )
	PORT_DIPSETTING( 0x24, "2A" )   // default
	PORT_DIPSETTING( 0x44, "2B" )
	PORT_DIPSETTING( 0x28, "3A" )
	PORT_DIPSETTING( 0x48, "3B" )
	PORT_DIPSETTING( 0x30, "4A" )
	PORT_DIPSETTING( 0x50, "4B" )
	PORT_DIPNAME( 0x380, 0x00, "K806 Mouse" )  PORT_DIPLOCATION("J:!8,J:!9,J:!10")
	PORT_DIPSETTING( 0x000, "Hawley, Alps" )
	PORT_DIPSETTING( 0x380, "Depraz, Logitech LM-P-5" )

	// S1 = leftmost, S2 = rightmost, S3 = middle switch (K806 manual, MAUS-12)
	PORT_START("MOUSE")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Left Mouse Button (S1)")    PORT_CODE(MOUSECODE_BUTTON1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Right Mouse Button (S2)")   PORT_CODE(MOUSECODE_BUTTON2)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_NAME("Middle Mouse Button (S3)")  PORT_CODE(MOUSECODE_BUTTON3)

	// one encoder count per host mouse unit at the default sensitivity;
	// reduce it to get the resolution of the real mouse (e.g. about 10%
	// for a Hawley Mark II with a 2000 dpi host mouse)
	PORT_START("MOUSEX")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	PORT_START("MOUSEY")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	// connected mouse, selects the connector pin order; the resolutions are
	// derived from the expected values of the NCR K806 diagnostic program
	// (counts for 5 inches: Alps 400-600, Hawley/LM-P5 900-1100, Depraz/P4
	// 1700-2000), use the analog sensitivity to match the host mouse
	PORT_START("CONFIG")
	PORT_CONFNAME( 0x07, 0x00, "Mouse type" )
	PORT_CONFSETTING( 0x00, "Hawley Mark II (200 cpi)" )
	PORT_CONFSETTING( 0x01, "Alps (100 cpi)" )
	PORT_CONFSETTING( 0x02, "Logitech LM-P5 (200 cpi)" )
	PORT_CONFSETTING( 0x03, "Depraz Souris P4 (370 cpi)" )
	PORT_CONFSETTING( 0x04, "Logitech P4 (370 cpi)" )
INPUT_PORTS_END

namespace {

// maximum encoder edge rate; the manual specifies "up to 4000 positions per second"
constexpr u32 EDGE_RATE = 4000;

// limit for edges that are not yet sent to the 8741 (about 1/8 s at full speed)
constexpr s32 MAX_BACKLOG = 512;

// connector pin order of each mouse type (true = Depraz)
constexpr bool MOUSE_DEPRAZ[] =
{
	false,  // Hawley Mark II
	false,  // Alps
	true,   // Logitech LM-P5
	true,   // Depraz Souris P4
	true    // Logitech P4
};

// channel levels (bit 0 = A, bit 1 = B) as seen on P1 for each quadrature phase;
// stepping forward makes the firmware count up
constexpr u8 QUADRATURE[4] = { 0x00, 0x01, 0x03, 0x02 };

} // anonymous namespace

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DMV_K806, dmv_k806_device, "dmv_k806", "K806 mouse")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dmv_k806_device - constructor
//-------------------------------------------------

dmv_k806_device::dmv_k806_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, DMV_K806, tag, owner, clock)
	, device_dmvslot_interface( mconfig, *this )
	, m_mcu(*this, "mcu")
	, m_jumpers(*this, "JUMPERS")
	, m_config(*this, "CONFIG")
	, m_mouse_buttons(*this, "MOUSE")
	, m_mouse_x(*this, "MOUSEX")
	, m_mouse_y(*this, "MOUSEY")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dmv_k806_device::device_start()
{
	// register for state saving
	save_item(NAME(m_last_x));
	save_item(NAME(m_last_y));
	save_item(NAME(m_pending_x));
	save_item(NAME(m_pending_y));
	save_item(NAME(m_phase_x));
	save_item(NAME(m_phase_y));
	save_item(NAME(m_next_axis));
	save_item(NAME(m_p1_reads));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dmv_k806_device::device_reset()
{
	m_last_x = m_mouse_x->read();
	m_last_y = m_mouse_y->read();
	m_pending_x = m_pending_y = 0;
	m_phase_x = m_phase_y = 0;
	m_next_axis = 0;
	m_p1_reads = 0;
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void dmv_k806_device::device_add_mconfig(machine_config &config)
{
	I8741A(config, m_mcu, XTAL(6'000'000));
	m_mcu->p1_in_cb().set(FUNC(dmv_k806_device::port1_r));
	m_mcu->p2_out_cb().set(FUNC(dmv_k806_device::port2_w));
	m_mcu->t1_in_cb().set(FUNC(dmv_k806_device::portt1_r));

	TIMER(config, "mouse_timer").configure_periodic(FUNC(dmv_k806_device::mouse_timer), attotime::from_hz(EDGE_RATE));
}

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor dmv_k806_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k806 );
}

//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *dmv_k806_device::device_rom_region() const
{
	return ROM_NAME( dmv_k806 );
}

void dmv_k806_device::io_read(int ifsel, offs_t offset, uint8_t &data)
{
	uint8_t jumpers = m_jumpers->read();
	if (BIT(jumpers, ifsel) && ((!BIT(offset, 3) && BIT(jumpers, 5)) || (BIT(offset, 3) && BIT(jumpers, 6))))
		data = m_mcu->upi41_master_r(offset & 1);
}

void dmv_k806_device::io_write(int ifsel, offs_t offset, uint8_t data)
{
	uint8_t jumpers = m_jumpers->read();
	if (BIT(jumpers, ifsel) && ((!BIT(offset, 3) && BIT(jumpers, 5)) || (BIT(offset, 3) && BIT(jumpers, 6))))
	{
		m_mcu->upi41_master_w(offset & 1, data);
		out_int(CLEAR_LINE);
	}
}

uint8_t dmv_k806_device::port1_r()
{
	// ---- ---x   S1 (left button)
	// ---- --x-   S2 (right button)
	// ---- -x--   S3 (middle button)
	// ---- x---   connector pin 2  XA / Y1
	// ---x ----   connector pin 3  XB / Y2
	// --x- ----   connector pin 4  YA / X2
	// -x-- ----   connector pin 5  YB / X1
	// x--- ----   not used

	uint8_t const x = QUADRATURE[m_phase_x];
	uint8_t const y = QUADRATURE[(4 - m_phase_y) & 3];   // the firmware decodes Y with the opposite sense
	uint8_t data = m_mouse_buttons->read() & 0x07;

	// the pin order depends on the connected mouse; the firmware only
	// decodes it correctly when J8-J10 match the mouse type
	if (MOUSE_DEPRAZ[std::min<unsigned>(m_config->read() & 0x07, 4)])
	{
		// Depraz / Logitech: X1 on pin 5, X2 on pin 4, Y1 on pin 2, Y2 on pin 3
		data |= BIT(x, 0) << 6;
		data |= BIT(x, 1) << 5;
		data |= BIT(y, 0) << 3;
		data |= BIT(y, 1) << 4;
	}
	else
	{
		// Hawley / Alps: XA on pin 2, XB on pin 3, YA on pin 4, YB on pin 5
		data |= x << 3;
		data |= y << 5;
	}

	if (!machine().side_effects_disabled() && m_p1_reads < 2)
		m_p1_reads++;

	return data;
}

int dmv_k806_device::portt1_r()
{
	return BIT(m_jumpers->read(), 7) ? 0 : 1;
}

void dmv_k806_device::port2_w(uint8_t data)
{
	out_int((data & 1) ? CLEAR_LINE : ASSERT_LINE);
}

/*-------------------------------------------------------------------

    Quadrature encoder simulation

    The host mouse movement is accumulated and converted into single
    encoder edges.  Moving right increments X, moving up (away from
    the user) increments Y, as in the previous implementation and as
    the GSX coordinate system (origin bottom left) expects.  Only one
    channel changes per timer tick, and both axes take turns when the
    mouse moves diagonally, so the firmware always sees a valid Gray
    code sequence.

    The 8741 needs roughly 200 us to process an edge.  A new edge is
    only generated after the firmware has read P1 at least twice since
    the previous edge (once to detect the change and once when it is
    back in its polling loop), so no edges get lost while it is busy
    talking to the host.

-------------------------------------------------------------------*/

void dmv_k806_device::step_axis(int32_t &pending, uint8_t &phase)
{
	if (pending > 0)
	{
		phase = (phase + 1) & 3;
		pending--;
	}
	else
	{
		phase = (phase - 1) & 3;
		pending++;
	}

	m_p1_reads = 0;
}

TIMER_DEVICE_CALLBACK_MEMBER(dmv_k806_device::mouse_timer)
{
	uint16_t const x = m_mouse_x->read();
	uint16_t const y = m_mouse_y->read();

	m_pending_x = std::clamp<int32_t>(m_pending_x + util::sext(x - m_last_x, 12), -MAX_BACKLOG, MAX_BACKLOG);
	m_pending_y = std::clamp<int32_t>(m_pending_y - util::sext(y - m_last_y, 12), -MAX_BACKLOG, MAX_BACKLOG);
	m_last_x = x;
	m_last_y = y;

	if (m_p1_reads < 2)
		return;

	bool const move_x = m_pending_x != 0;
	bool const move_y = m_pending_y != 0;

	if (move_x && (!move_y || !m_next_axis))
	{
		step_axis(m_pending_x, m_phase_x);
		m_next_axis = 1;
	}
	else if (move_y)
	{
		step_axis(m_pending_y, m_phase_y);
		m_next_axis = 0;
	}
}
