// license:BSD-3-Clause
// copyright-holders:rfka01
// thanks-to:Sandro Ronco
/***************************************************************************

    K238 External Z8001 processor module (homebrew, not an NCR product)

    A fictional alternative to the K234: plugs into slot J7 like the K234,
    has no ROM of its own, and needs a K235 in J7A.  The K235 is needed for
    the same reason as with the K234: a K230 decodes D0h-DFh without BA3,
    so ringing the doorbell at D8h would also switch the K230; the K235
    separates D0h-D7h (SWITCH16) from D8h-DFh (this card) on BA3.  The
    8259A of the K235 sits on the 8088's local bus; ports 90h/91h never
    reach the system bus, so the Z8001 cannot see it.
    (Earlier drafts called the card "K240"; that number is NCR's real tilt
    mechanism kit, so the card was renamed.)

    Modelled on the K234 68008 module in k230.cpp.

    ---- STARTING AND STOPPING THE Z8001 --------------------------------

    The doorbell at D8h-DFh is decided by WHO rings it:

      * Any other bus master (the Z80 with OUT (0DFh),A, or the 8088 of a
        K235 with OUT 0D8h,AL) STARTS the Z8001: HALT is released and the
        Z8001 is reset on a zero-delay timer, so the ringing instruction
        retires first.  THOLD then holds the Z80.  Every start is a reset:
        the Z8001 re-reads its PSA (FCW at 0002h, segmented PC at
        0004h-0007h of segment 0) from shared RAM.
      * The Z8001 itself ringing D8h-DFh through its own I/O space HANDS
        THE BUS BACK: it is parked (HALT) at once and THOLD is released,
        so the Z80 resumes after its OUT.

    Deciding by origin instead of toggling a direction flag keeps both
    uses working:
      - CP/M style: Z80 starts, Z8001 hands back, Z80 starts again.
      - Job style, as DMVRACE and DMVDEMO drive the K234 from MS-DOS:
        the 8088 puts the reset context over its vectors 0/1, rings D8h
        for every job, and the coprocessor stops itself at the end of each
        job (HALT on the Z8001, STOP on the 68008) without ringing back.
        A toggle would take the second job's start for a hand-back.

    A change of SWITCH16/ (DMV I/O D0h-D7h) also parks the Z8001.

    While the Z8001 runs, every program/data/stack access is forwarded
    through prog_read/prog_write into the DMV memory space (segment n =
    RAM bank n, banks 1-7 from the expansion in slot 1), and every Z8001
    I/O access is forwarded into the DMV I/O space where the mainboard
    peripherals live (uPD765, AM9517A, 8741, uPD7220, 8253).  This card
    provides NO peripherals of its own.  No interrupts are wired yet;
    software polls, as on the K234.

    ---- OPEN ITEMS (flagged, verify on the bench) ---------------------

    * ENDIANNESS.  The Z8001 is big-endian; the DMV/Z80 world is little-
      endian and MAME's DMV peripherals expect Z80 byte lanes.  We keep
      all shared-RAM and I/O forwarding BYTE-WIDE here and let the Z8001
      core assemble words itself, which sidesteps lane-swap surprises.
      If 16-bit peripheral accesses misbehave, this is the first suspect.

    * SEGMENTATION.  On real silicon the Z8001 emits SN0-SN6 a cycle
      early and needs a segment latch (or Z8010 MMU).  In MAME the
      z8001_device presents a linear map, so we ignore that here; it
      only matters for the later physical PCB.

    * RESET CONTEXT.  A real Z8001 reset loads FCW+PC from the Program
      Status Area at the base of segment 0.  Whatever bootstrap the host
      Z80 stages in shared RAM must place a valid PSA where the Z8001
      expects it.  That is a BIOS/loader concern, not a card concern,
      but it is why "no boot EPROM" works: the reset vector lives in the
      RAM the Z80 filled.  The host must have RAM switched in over
      0000h-1FFFh (OUT 10h) before ringing: while the mainboard ROM is
      mapped there, reads below 2000h return ROM for every bus master,
      the Z8001 included.

***************************************************************************/

#include "emu.h"
#include "k238.h"


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

//  All three CPU memory spaces (program / data / stack) alias onto the
//  same shared DMV RAM via program_r/program_w.  The DMV memory space is
//  20-bit (1 MB, BA0-BA19); the Z8001 offset+segment is wider, so we
//  mask to the DMV's reach.  Adjust the top of the window once the BIOS
//  memory map is settled.

void dmv_k238_device::k238_prog(address_map &map)
{
	map.unmap_value_high();
	map(0x00000, 0xfffff).rw(FUNC(dmv_k238_device::program_r), FUNC(dmv_k238_device::program_w));
}

void dmv_k238_device::k238_io(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x00, 0xff).rw(FUNC(dmv_k238_device::io_r), FUNC(dmv_k238_device::io_w));
}


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DMV_K238, dmv_k238_device, "dmv_k238", "K238 Z8001 processor module (homebrew)")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

dmv_k238_device::dmv_k238_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k238_device(mconfig, DMV_K238, tag, owner, clock)
{
}

dmv_k238_device::dmv_k238_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_dmvslot_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_switch16(0)
	, m_hold(0)
	, m_snr(0)
{
}


//-------------------------------------------------
//  device_add_mconfig
//-------------------------------------------------

void dmv_k238_device::device_add_mconfig(machine_config &config)
{
	// Clock: placeholder. The K234 runs its 68008 at XTAL(16'000'000)/2.
	// Pick the Z8001 crystal to match whatever the physical card will use;
	// 4 MHz is a safe starting point for a period-correct part.
	Z8001(config, m_maincpu, XTAL(8'000'000) / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &dmv_k238_device::k238_prog);
	m_maincpu->set_addrmap(AS_IO,      &dmv_k238_device::k238_io);
	// NOTE: the Z8000 core makes AS_DATA / AS_STACK optional -- see
	// z8000.cpp, which does has_space(AS_STACK)?...:AS_PROGRAM. We install
	// only AS_PROGRAM, so data and stack cycles alias onto it. That matches
	// the DMV, which has a single shared RAM and no ST-line decoding.

	// Interrupts: none wired.  The 8259A of the K235 is not reachable
	// from slot J7 (it sits on the 8088's local bus).  Candidates, as on
	// the K234, are TIMINT (8253 channel 2) and the bus INT/ line, which
	// could drive the Z8001's /VI or /NVI later.  Software polls for now.
}


//-------------------------------------------------
//  device_start / device_reset
//-------------------------------------------------

void dmv_k238_device::device_start()
{
	// SNR doorbell in the DMV I/O space at 0xd8-0xdf, as on the K234.
	// Rings from the Z80 or the K235's 8088 land here and start the Z8001;
	// the Z8001's own rings are caught in io_r/io_w and hand the bus back.
	iospace().install_readwrite_handler(0xd8, 0xdf,
			read8smo_delegate(*this, FUNC(dmv_k238_device::snr_r)),
			write8smo_delegate(*this, FUNC(dmv_k238_device::snr_w)), 0);

	m_snr_reset_timer = timer_alloc(FUNC(dmv_k238_device::snr_reset), this);

	save_item(NAME(m_switch16));
	save_item(NAME(m_hold));
	save_item(NAME(m_snr));
}

void dmv_k238_device::device_reset()
{
	m_switch16 = 0;
	m_hold = 0;
	m_snr = 0;
}


//-------------------------------------------------
//  av16bit - tell the DMV a 16-bit master is present
//-------------------------------------------------

bool dmv_k238_device::av16bit()
{
	return true;
}


//-------------------------------------------------
//  hold_w - bus hold (DMA / arbitration) from host
//
//  Z8001 stays halted while a hold is asserted OR while we have not yet
//  been released (m_snr == 0). Same gating shape as K234.
//-------------------------------------------------

void dmv_k238_device::hold_w(int state)
{
	m_hold = state;
	m_maincpu->set_input_line(INPUT_LINE_HALT, (m_hold || !m_snr) ? ASSERT_LINE : CLEAR_LINE);
}


//-------------------------------------------------
//  switch16_w - SWITCH16/ toggled by DMV I/O 0xD0-0xD7
//
//  On a change we PARK the Z8001: clear the surrender flag, halt it and
//  hand the bus back to the host (THOLD low).  The next ring from the
//  host starts the Z8001 afresh.
//-------------------------------------------------

void dmv_k238_device::switch16_w(int state)
{
	if (m_switch16 != state)
	{
		m_snr = CLEAR_LINE;
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		out_thold(CLEAR_LINE);
		m_switch16 = state;
	}
}


//-------------------------------------------------
//  SNR doorbell - 0xD8-0xDF
//
//  snr_r / snr_w are reached by every OTHER bus master (Z80, K235 8088):
//  they start the Z8001.  The Z8001's own accesses to D8h-DFh never get
//  here; io_r / io_w catch them and call snr_hand_back().
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(dmv_k238_device::snr_reset)
{
	// deferred start: reset() here rather than inside the access handler,
	// so the instruction that rang the doorbell retires first
	m_snr = ASSERT_LINE;
	m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_maincpu->reset();
	out_thold(ASSERT_LINE);
}

void dmv_k238_device::snr_start()
{
	// host -> Z8001: (re)start with a reset, whatever state the Z8001 is in
	m_snr_reset_timer->adjust(attotime::zero);
}

void dmv_k238_device::snr_hand_back()
{
	// Z8001 -> host: park the Z8001 at once, release the Z80
	m_snr = CLEAR_LINE;
	m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
	out_thold(CLEAR_LINE);
}

uint8_t dmv_k238_device::snr_r()
{
	if (!machine().side_effects_disabled())   // ignore debugger reads
		snr_start();

	return 0xff;
}

void dmv_k238_device::snr_w(uint8_t data)
{
	snr_start();
}


//-------------------------------------------------
//  program_r / program_w - forward Z8001 mem cycles to shared DMV RAM
//-------------------------------------------------

uint8_t dmv_k238_device::program_r(offs_t offset)
{
	return prog_read(offset);
}

void dmv_k238_device::program_w(offs_t offset, uint8_t data)
{
	prog_write(offset, data);
}


//-------------------------------------------------
//  io_r / io_w - forward Z8001 I/O cycles to the DMV motherboard
//-------------------------------------------------

uint8_t dmv_k238_device::io_r(offs_t offset)
{
	if ((offset & 0xf8) == 0xd8)                // own doorbell: hand back
	{
		if (!machine().side_effects_disabled())
			snr_hand_back();
		return 0xff;
	}

	return iospace().read_byte(offset);
}

void dmv_k238_device::io_w(offs_t offset, uint8_t data)
{
	if ((offset & 0xf8) == 0xd8)                // own doorbell: hand back
	{
		snr_hand_back();
		return;
	}

	iospace().write_byte(offset, data);
}
