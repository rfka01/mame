// license:BSD-3-Clause
// copyright-holders:rfka01
// thanks-to:Sandro Ronco
/***************************************************************************

    K238 External Z8001 processor module (homebrew, not an NCR product)

    A fictional alternative to the K234: plugs into slot J7 like the K234,
    has no ROM of its own, and expects a K235 in J7A for the 8259A PIC.
    (Earlier drafts called it "K240"; that number is NCR's real tilt
    mechanism kit, so the card was renamed.)

    Modelled directly on the K234 68008 module in k230.cpp.  Read that
    file and k234_notes.md alongside this one; the handoff mechanism is
    identical and the comments there explain the DMV side.

    ---- HOW THE HANDOFF WORKS (same as K234) --------------------------

    The DMV host is a Z80.  It boots normally.  To use the coprocessor:

      1. Host copies the Z8001 program image (including its reset PSA:
         FCW + PC that the CPU will fetch) into shared DMV RAM.

      2. Host rings the SNR doorbell (OUT to port 0xDF; the card decodes
         0xD8-0xDF).  The Z8001 is released and reset on a zero-delay
         timer, so the Z80's OUT retires first; THOLD then freezes the
         Z80.  The Z8001 fetches its PSA from RAM and owns the bus.

      3. To hand the bus back, the Z8001 rings the SAME doorbell (OUT to
         0xD8-0xDF from Z8001 I/O space, which this card forwards to the
         DMV I/O space).  The Z8001 is parked (HALT) immediately and THOLD
         is released, so the Z80 resumes after its OUT.

    One doorbell, two directions: m_await_back records which way the next
    ring goes.  This follows NCR's own K234 formatter (FORMAT68.S rings
    SNR from the 68008 to hand back, "* must end before 911a"; FORM68.Z80
    rings it from the Z80 to start the 68008).

    Every Z80 -> Z8001 start is a RESET, exactly like the K234: the Z8001
    re-reads its PSA each time.  State that must survive a round trip to
    the Z80 lives in a shared-RAM mailbox, not in live CPU registers;
    the Z80 points the PSA at the resume entry before ringing.

    A change of SWITCH16/ (DMV I/O 0xD0-0xD7) also parks the Z8001 and
    re-arms the doorbell for a fresh Z80 -> Z8001 start.

    While the Z8001 runs, every program/data/stack access is forwarded
    through prog_read/prog_write into the DMV shared memory space, and
    every Z8001 I/O access is forwarded into the DMV I/O space where the
    motherboard peripherals live (uPD765 floppy, 8741 keyboard, uPD7220
    CRTC, 8259A PIC at 0x90, etc).  This card provides NO peripherals of
    its own -- it is a pure bus master, exactly like the K234.

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
      RAM the Z80 filled.

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
	, m_await_back(0)
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

	// Interrupts: route the DMV 8259A (motherboard) into the Z8001's
	// vectored-interrupt input once the console/floppy path works.
	// The Z8001 takes /VI and /NVI; wiring is done via the irqN_w slot
	// hooks below feeding an inputline. Left unwired in this skeleton --
	// bring it up only after polled console+disk are proven.
}


//-------------------------------------------------
//  device_start / device_reset
//-------------------------------------------------

void dmv_k238_device::device_start()
{
	// SNR doorbell in the DMV I/O space at 0xd8-0xdf, as on the K234.
	// Both CPUs reach it: the Z80 directly, the Z8001 through io_r/io_w.
	iospace().install_readwrite_handler(0xd8, 0xdf,
			read8smo_delegate(*this, FUNC(dmv_k238_device::snr_r)),
			write8smo_delegate(*this, FUNC(dmv_k238_device::snr_w)), 0);

	m_snr_reset_timer = timer_alloc(FUNC(dmv_k238_device::snr_reset), this);

	save_item(NAME(m_switch16));
	save_item(NAME(m_hold));
	save_item(NAME(m_snr));
	save_item(NAME(m_await_back));
}

void dmv_k238_device::device_reset()
{
	m_switch16 = 0;
	m_hold = 0;
	m_snr = 0;
	m_await_back = 0;
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
//  On a change we PARK the Z8001: clear the surrender flag, halt it, hand
//  the bus back to the host (THOLD low) and re-arm the doorbell so the
//  next ring is a fresh Z80 -> Z8001 start.  Identical to K234.
//-------------------------------------------------

void dmv_k238_device::switch16_w(int state)
{
	if (m_switch16 != state)
	{
		m_snr = CLEAR_LINE;
		m_await_back = 0;
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		out_thold(CLEAR_LINE);
		m_switch16 = state;
	}
}


//-------------------------------------------------
//  SNR doorbell - 0xD8-0xDF, rung by either CPU
//
//  Same model as K234::snr_toggle (see k234_notes.md):
//    Z80 -> Z8001: release and reset the Z8001 on a zero-delay timer, so
//                  the Z80's OUT retires before the reset happens.
//    Z8001 -> Z80: park the Z8001 at once and release the Z80, so the
//                  Z8001 does not run past the instruction that rang.
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(dmv_k238_device::snr_reset)
{
	// deferred release: reset() here rather than inside the access handler
	m_snr = ASSERT_LINE;
	m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_maincpu->reset();
	out_thold(ASSERT_LINE);
}

void dmv_k238_device::snr_toggle()
{
	if (!m_await_back)
	{
		// Z80 -> Z8001: start the Z8001 (deferred, see snr_reset)
		m_await_back = 1;
		m_snr_reset_timer->adjust(attotime::zero);
	}
	else
	{
		// Z8001 -> Z80: park the Z8001, give the bus back to the Z80
		m_await_back = 0;
		m_snr = CLEAR_LINE;
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		out_thold(CLEAR_LINE);
	}
}

uint8_t dmv_k238_device::snr_r()
{
	if (!machine().side_effects_disabled())   // ignore debugger reads
		snr_toggle();

	return 0xff;
}

void dmv_k238_device::snr_w(uint8_t data)
{
	snr_toggle();
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
	return iospace().read_byte(offset);
}

void dmv_k238_device::io_w(offs_t offset, uint8_t data)
{
	iospace().write_byte(offset, data);
}
