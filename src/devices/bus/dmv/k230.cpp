// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
/***************************************************************************

    K230 Internal 8088 module without interrupt controller
    K231 External 8088 module without interrupt controller
    K234 External 68008 module
    K235 Internal 8088 module with interrupt controller
    K235+ Internal 8088/V20 module with interrupt controller and 1 MB
          address space (homebrew, VzEkC e.V., F. Stassen 2025)

    ---- K235+ ---------------------------------------------------------

    Not an NCR product: a re-design of the K235 ("16-bit Processor with
    PIC Plus", schematic rev 0.01 of 28.12.2024, manual v1.0.1 of
    2025-03-05).  Same 8259A, interrupt routing (DIP-FIX S1/S2) and 8087
    socket as the K235, but:

    - the 74LS373 latch C2 puts SA16-SA19 on BA16-BA19, so all of the
      1 MB address space goes to the bus (the K235 maps the upper 512 KB
      to its EPROM).  Without a card that decodes BA19 (K0815), the
      mainboard RAM ignores BA19 and the upper 512 KB mirror the lower.
    - the EPROM only answers in a window at the top of the address space.
      RCSEL = BA19 & BA18 & BA17 & BA16 & the address lines A15-A12 whose
      switch in S3 is ON (74LS30, pull-ups RN):

         S3  1 2 3 4   window
             1 1 1 1    4 KB  FF000-FFFFF  (2732)
             1 1 1 0    8 KB  FE000-FFFFF  (2764)
             1 1 0 0   16 KB  FC000-FFFFF  (27128)
             1 0 0 0   32 KB  F8000-FFFFF  (27256)
             0 0 0 0   64 KB  F0000-FFFFF  (27512)
             (1 = ON, switch 1 = A15 ... switch 4 = A12)

      EPROM reads stay on the module (FW16CS = HOLDA16 & RCSEL), writes
      still run a bus cycle.  RDY16 is forced for EPROM cycles.
    - EPROM sockets E2 (2732, 24 pin) and a 28 pin socket for
      2764-27512; J1-J3 route pins 26, 27 and 1 to A13-A15 or +5 V:

             J1    J2    J3    EPROM
             2-3   open  open  2732
             open  2-3   2-3   2764
             1-2   2-3   2-3   27128
             1-2   1-2   2-3   27256
             1-2   1-2   1-2   27512

    - HOLDA16 is brought out on P107-b13; the K0815 needs it (wired to its
      AUTO/ pin a13).

    The VzEkC firmware (not included, it fails on this module) is the K235
    ROM 33473 with the memory test extended
    from 7 to 15 banks and a new bank name table at 0C00h.  That test also
    covers bank F000h in full and so writes into the EPROM window at
    FF000h; the readback fails there and the test stops with "FAILED (CR)"
    at "960K-999K", leaving 14 banks in 0:FE06.  Without a K0815 its
    presence test (AA55h at offset 0) also passes for banks 8-15, which
    mirror banks 0-7, and the pattern test then overwrites bank 0 (stack
    and Z80 mailbox).  The amended firmware "33473f" limits the test of
    bank F000h to F0000-FEFFD, rejects mirrored banks and reports 15
    banks with a K0815 and 7 with 512 KB; 0:FE06 cannot express the 4 KB
    hole, so software has to stop at FF000h itself.

    ---- Firmware 2.0 (homebrew, K235 and K235+) ----------------------

    New memory test and monitor on top of 33473; kept from 33473 are the
    entries FF00:0000 (far display) and FF00:0003, the command table at
    0010h, the return to the Z80 at 0018h, the 8259 test and the keyboard
    and CRT drivers.  FF00:0632 still enters the monitor.
    - memory test in 4 KB blocks (address, 55AAh, AA55h), ends at the first
      block that is no RAM or mirrors the block 512 KB below; works with
      64 KB to 512 KB on both modules and 1020 KB on the K235+ with a
      K0815.  0:FE06 counts a partial last bank (15 for 1020 KB).
    - screen, centred: "NCR" as white block letters in a red box
      (7 rows), "Decision Mate V", "RAM test (press M ...)", bar and KB
      counter; colours from the mainboard ROM id at 0FF8h (M: lit box,
      dark letters, normal text); boot text green again,
      the screen is cleared when the monitor starts.
    - errors show address, written/read word and bit mask, then wait for
      CR or about 10 s.  "M" during the test enters the monitor afterwards.
    - monitor without password: D E F M C S I O R G T Z(RAM test) L(Intel
      HEX) X ?, Q runs the self test again and then boots
    - console on screen/keyboard and, if a 2651 answers at 70h (K211 on
      IFSEL 1A), also on the serial port at 9600 8N1.

***************************************************************************/

#include "emu.h"
#include "k230.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

ROM_START( dmv_k230 )
	ROM_REGION(0x1000, "rom", 0)
	ROM_LOAD( "dmv_int_8088_32167.bin", 0x0000, 0x1000, CRC(f4a58880) SHA1(4f50ef25008851ae6f0c670f19d63f4e61249581))
ROM_END

ROM_START( dmv_k231 )
	ROM_REGION(0x1000, "rom", 0)
	ROM_LOAD( "dmv_ext_8088_32167.bin", 0x0000, 0x1000, CRC(f4a58880) SHA1(4f50ef25008851ae6f0c670f19d63f4e61249581))
ROM_END

ROM_START( dmv_k235 )
	ROM_REGION(0x1000, "rom", 0)
	ROM_DEFAULT_BIOS("33473")
	ROM_SYSTEM_BIOS(0, "33473", "33473, NCR original")
	ROMX_LOAD( "dmv_int_8088_pic_33473.bin", 0x0000, 0x1000, CRC(104195dc) SHA1(08d48ca3b84ab26c1a764792e04ec4def7dad2ad), ROM_BIOS(0) )
	ROM_SYSTEM_BIOS(1, "fw20", "firmware 2.0 (homebrew)")
	ROMX_LOAD( "k235_fw20.bin", 0x0000, 0x1000, CRC(66a7f7cf) SHA1(cec4274a5c28aacf228c55bb4daadd3fba8de52e), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS(2, "33473f", "33473, 1 MB patch, EPROM window and mirror safe")
	ROMX_LOAD( "k235p_33473f.bin", 0x0000, 0x1000, CRC(eade560f) SHA1(1b56ad4a205bda83e1c95b8dd21c8ae6c9d0b0c4), ROM_BIOS(2) )
ROM_END

ROM_START( dmv_k235p )
	ROM_REGION(0x10000, "rom", ROMREGION_ERASEFF)
	ROM_DEFAULT_BIOS("fw20")
	ROM_SYSTEM_BIOS(0, "fw20", "firmware 2.0 (homebrew)")
	ROMX_LOAD( "k235_fw20.bin", 0x0000, 0x1000, CRC(66a7f7cf) SHA1(cec4274a5c28aacf228c55bb4daadd3fba8de52e), ROM_BIOS(0) )
	// VzEkC patch amended: bank F000h tested only up to FEFFDh (4 KB window), presence test
	// rejects banks that mirror the bank 512 KB below (no BA19 decode), 10 character bank names
	ROM_SYSTEM_BIOS(1, "33473f", "33473, 1 MB patch, EPROM window and mirror safe")
	ROMX_LOAD( "k235p_33473f.bin", 0x0000, 0x1000, CRC(eade560f) SHA1(1b56ad4a205bda83e1c95b8dd21c8ae6c9d0b0c4), ROM_BIOS(1) )
	// original NCR K235 firmware, tests 7 banks (512 KB)
	ROM_SYSTEM_BIOS(2, "33473", "33473, NCR original")
	ROMX_LOAD( "dmv_int_8088_pic_33473.bin", 0x0000, 0x1000, CRC(104195dc) SHA1(08d48ca3b84ab26c1a764792e04ec4def7dad2ad), ROM_BIOS(2) )
ROM_END

void dmv_k230_device::k230_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x00000, 0x7ffff).rw(FUNC(dmv_k230_device::program_r), FUNC(dmv_k230_device::program_w));
	map(0x80000, 0xfffff).r(FUNC(dmv_k230_device::rom_r));
}

void dmv_k230_device::k230_io(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x00, 0xff).rw(FUNC(dmv_k230_device::io_r), FUNC(dmv_k230_device::io_w));
}

void dmv_k234_device::k234_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x00000, 0x7ffff).rw(FUNC(dmv_k234_device::program_r), FUNC(dmv_k234_device::program_w));
	map(0xfff00, 0xfffff).rw(FUNC(dmv_k234_device::io_r), FUNC(dmv_k234_device::io_w));
}

void dmv_k235_device::k235_io(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	map(0x00, 0xff).rw(FUNC(dmv_k235_device::io_r), FUNC(dmv_k235_device::io_w));
	map(0x90, 0x91).rw(m_pic, FUNC(pic8259_device::read), FUNC(pic8259_device::write));
}

void dmv_k235p_device::k235p_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x00000, 0xfffff).rw(FUNC(dmv_k235p_device::k235p_mem_r), FUNC(dmv_k235p_device::k235p_mem_w));
}

static INPUT_PORTS_START( dmv_k230 )
	PORT_START("DSW")
	PORT_CONFNAME( 0x80, 0x00, "8087 Coprocessor" )
	PORT_CONFSETTING( 0x00, "Not installed" )
	PORT_CONFSETTING( 0x80, "Installed" )
INPUT_PORTS_END

static INPUT_PORTS_START( dmv_k235 )
	PORT_START("DSW")
	PORT_DIPNAME( 0x01, 0x00, "K235 INT7" )  PORT_DIPLOCATION("S:1")
	PORT_DIPSETTING( 0x00, "Slot 5" )
	PORT_DIPSETTING( 0x01, "Slot 6" )
	PORT_DIPNAME( 0x02, 0x00, "K235 INT5" )  PORT_DIPLOCATION("S:2")
	PORT_DIPSETTING( 0x00, "Slot 2a" )
	PORT_DIPSETTING( 0x02, "Slot 2" )
	PORT_CONFNAME( 0x80, 0x80, "8087 Coprocessor" )
	PORT_CONFSETTING( 0x00, "Not installed" )
	PORT_CONFSETTING( 0x80, "Installed" )
INPUT_PORTS_END


static INPUT_PORTS_START( dmv_k235p )
	PORT_START("DSW")
	PORT_DIPNAME( 0x01, 0x00, "INT7 (DIP-FIX S1)" )  PORT_DIPLOCATION("S1:1")
	PORT_DIPSETTING( 0x00, "Slot 5 (IRQ5)" )
	PORT_DIPSETTING( 0x01, "Slot 6 (IRQ6)" )
	PORT_DIPNAME( 0x02, 0x00, "INT5 (DIP-FIX S2)" )  PORT_DIPLOCATION("S2:1")
	PORT_DIPSETTING( 0x00, "Slot 2a (IRQ2A)" )
	PORT_DIPSETTING( 0x02, "Slot 2 (IRQ2)" )
	PORT_CONFNAME( 0x80, 0x00, "8087 Coprocessor" )
	PORT_CONFSETTING( 0x00, "Not installed" )
	PORT_CONFSETTING( 0x80, "Installed" )

	PORT_START("EPROM")
	PORT_DIPNAME( 0x01, 0x00, "EPROM window A15" )  PORT_DIPLOCATION("S3:1")
	PORT_DIPSETTING( 0x01, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "EPROM window A14" )  PORT_DIPLOCATION("S3:2")
	PORT_DIPSETTING( 0x02, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "EPROM window A13" )  PORT_DIPLOCATION("S3:3")
	PORT_DIPSETTING( 0x04, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "EPROM window A12" )  PORT_DIPLOCATION("S3:4")
	PORT_DIPSETTING( 0x08, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x00, DEF_STR( On ) )
	PORT_CONFNAME( 0x70, 0x00, "EPROM type (J1-J3)" )
	PORT_CONFSETTING( 0x00, "2732" )
	PORT_CONFSETTING( 0x10, "2764" )
	PORT_CONFSETTING( 0x20, "27128" )
	PORT_CONFSETTING( 0x30, "27256" )
	PORT_CONFSETTING( 0x40, "27512" )
INPUT_PORTS_END


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(DMV_K230, dmv_k230_device, "dmv_k230", "K230 8088 without interrupt controller")
DEFINE_DEVICE_TYPE(DMV_K231, dmv_k231_device, "dmv_k231", "K231 8088 without interrupt controller")
DEFINE_DEVICE_TYPE(DMV_K234, dmv_k234_device, "dmv_k234", "K234 68008")
DEFINE_DEVICE_TYPE(DMV_K235, dmv_k235_device, "dmv_k235", "K235 8088 with interrupt controller")
DEFINE_DEVICE_TYPE(DMV_K235P, dmv_k235p_device, "dmv_k235p", "K235+ 8088 with interrupt controller and 1MB (homebrew)")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dmv_k230_device - constructor
//-------------------------------------------------

dmv_k230_device::dmv_k230_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k230_device(mconfig, DMV_K230, tag, owner, clock)
{
}

dmv_k230_device::dmv_k230_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_dmvslot_interface(mconfig, *this)
	, m_maincpu(*this, "maincpu")
	, m_fpu(*this, "i8087")
	, m_config(*this, "DSW")
	, m_rom(*this, "rom")
	, m_switch16(0)
	, m_hold(0)
{
}

//-------------------------------------------------
//  dmv_k231_device - constructor
//-------------------------------------------------

dmv_k231_device::dmv_k231_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k230_device(mconfig, DMV_K231, tag, owner, clock)
{
}

//-------------------------------------------------
//  dmv_k234_device - constructor
//-------------------------------------------------

dmv_k234_device::dmv_k234_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k230_device(mconfig, DMV_K234, tag, owner, clock), m_snr(0), m_await_back(0)
{
}

//-------------------------------------------------
//  dmv_k235_device - constructor
//-------------------------------------------------

dmv_k235_device::dmv_k235_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k235_device(mconfig, DMV_K235, tag, owner, clock)
{
}

dmv_k235_device::dmv_k235_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k230_device(mconfig, type, tag, owner, clock), m_pic(*this, "pic8259")
{
}

//-------------------------------------------------
//  dmv_k235p_device - constructor
//-------------------------------------------------

dmv_k235p_device::dmv_k235p_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: dmv_k235_device(mconfig, DMV_K235P, tag, owner, clock), m_eprom(*this, "EPROM")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dmv_k230_device::device_start()
{
	// register for state saving
	save_item(NAME(m_switch16));
	save_item(NAME(m_hold));
}

void dmv_k234_device::device_start()
{
	dmv_k230_device::device_start();
	iospace().install_readwrite_handler(0xd8, 0xdf, read8smo_delegate(*this, FUNC(dmv_k234_device::snr_r)), write8smo_delegate(*this, FUNC(dmv_k234_device::snr_w)), 0);

	m_snr_reset_timer = timer_alloc(FUNC(dmv_k234_device::snr_reset), this);

	// register for state saving
	save_item(NAME(m_snr));
	save_item(NAME(m_await_back));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dmv_k230_device::device_reset()
{
	m_switch16 = 0;
	m_hold = 0;
}

void dmv_k234_device::device_reset()
{
	dmv_k230_device::device_reset();
	m_snr = 0;
	m_await_back = 0;
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void dmv_k230_device::fpu_config(machine_config &config)
{
	// 8087 socket B1 next to the CPU (K235 schematic 017-0033502, sheet 1):
	// - CLK/READY/RESET shared with the CPU (8284A, 15 MHz / 3)
	// - BUSY (pin 23) -> CPU TEST/ (pin 23), used by FWAIT (V20: /POLL)
	// - INT (pin 32) -> CPU NMI (pin 17); 0 ohm jumper R5 ties NMI to GND
	//   when B1 is not populated
	// - RQ/GT0 -> CPU RQ/GT1, QS0/QS1 in parallel (not needed for emulation)
	I8087(config, m_fpu, XTAL(15'000'000) / 3);
	m_fpu->set_space_88(m_maincpu, AS_PROGRAM);
	m_fpu->busy().set_inputline(m_maincpu, INPUT_LINE_TEST);
	m_fpu->irq().set([this] (int state) { if (fpu_present()) m_maincpu->set_input_line(INPUT_LINE_NMI, state); });
}

void dmv_k230_device::device_add_mconfig(machine_config &config)
{
	i8088_cpu_device &maincpu(I8088(config, m_maincpu, XTAL(15'000'000) / 3));
	maincpu.set_addrmap(AS_PROGRAM, &dmv_k230_device::k230_mem);
	maincpu.set_addrmap(AS_IO, &dmv_k230_device::k230_io);
	maincpu.esc_opcode_handler().set(FUNC(dmv_k230_device::esc_opcode_w));
	maincpu.esc_data_handler().set(FUNC(dmv_k230_device::esc_data_w));

	fpu_config(config);
}

void dmv_k234_device::device_add_mconfig(machine_config &config)
{
	M68008(config, m_maincpu, XTAL(16'000'000) / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &dmv_k234_device::k234_mem);
}

void dmv_k235_device::device_add_mconfig(machine_config &config)
{
	v20_device &maincpu(V20(config, m_maincpu, XTAL(15'000'000) / 3));
	maincpu.set_addrmap(AS_PROGRAM, &dmv_k235_device::k230_mem);
	maincpu.set_addrmap(AS_IO, &dmv_k235_device::k235_io);
	maincpu.set_irq_acknowledge_callback("pic8259", FUNC(pic8259_device::inta_cb));
	maincpu.esc_opcode_handler().set(FUNC(dmv_k235_device::esc_opcode_w));
	maincpu.esc_data_handler().set(FUNC(dmv_k235_device::esc_data_w));

	PIC8259(config, m_pic);
	m_pic->out_int_callback().set_inputline(m_maincpu, 0);

	// C8087-1 fitted on the K235 board (V20 /POLL = NEC_INPUT_LINE_POLL = INPUT_LINE_TEST)
	fpu_config(config);
}

void dmv_k235p_device::device_add_mconfig(machine_config &config)
{
	dmv_k235_device::device_add_mconfig(config);

	// D1 is an 8088 on the schematic, the boards are built with a V20 as on the K235
	m_maincpu->set_addrmap(AS_PROGRAM, &dmv_k235p_device::k235p_mem);
}

//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *dmv_k230_device::device_rom_region() const
{
	return ROM_NAME( dmv_k230 );
}

const tiny_rom_entry *dmv_k231_device::device_rom_region() const
{
	return ROM_NAME( dmv_k231 );
}

const tiny_rom_entry *dmv_k234_device::device_rom_region() const
{
	return nullptr;
}

const tiny_rom_entry *dmv_k235_device::device_rom_region() const
{
	return ROM_NAME( dmv_k235 );
}

const tiny_rom_entry *dmv_k235p_device::device_rom_region() const
{
	return ROM_NAME( dmv_k235p );
}

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor dmv_k230_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k230 );
}

ioport_constructor dmv_k235_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k235 );
}

ioport_constructor dmv_k235p_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( dmv_k235p );
}

bool dmv_k230_device::av16bit()
{
	return true;
}

void dmv_k230_device::hold_w(int state)
{
	m_hold = state;
	m_maincpu->set_input_line(INPUT_LINE_HALT, (m_hold || !m_switch16) ? ASSERT_LINE : CLEAR_LINE);
}

void dmv_k230_device::switch16_w(int state)
{
	m_switch16 = state;
	m_maincpu->set_input_line(INPUT_LINE_HALT, (m_hold || !m_switch16) ? ASSERT_LINE : CLEAR_LINE);
}

uint8_t dmv_k230_device::rom_r(offs_t offset)
{
	return m_rom->base()[offset & 0x0fff];
}

uint8_t dmv_k230_device::io_r(offs_t offset)
{
	return iospace().read_byte(offset);
}

void dmv_k230_device::io_w(offs_t offset, uint8_t data)
{
	iospace().write_byte(offset, data);
}

uint8_t dmv_k230_device::program_r(offs_t offset)
{
	return prog_read(offset);
}

void dmv_k230_device::program_w(offs_t offset, uint8_t data)
{
	prog_write(offset, data);
}

void dmv_k234_device::hold_w(int state)
{
	m_hold = state;
	m_maincpu->set_input_line(INPUT_LINE_HALT, (m_hold || !m_snr) ? ASSERT_LINE : CLEAR_LINE);
}

void dmv_k234_device::switch16_w(int state)
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

TIMER_CALLBACK_MEMBER(dmv_k234_device::snr_reset)
{
	// deferred 68008 release: reset() here instead of in the access handler,
	// so the instruction that rang SNR retires first (a synchronous reset
	// re-entered the core and caused exception $03 after the SNR store)
	m_snr = ASSERT_LINE;
	m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_maincpu->reset();
	out_thold(ASSERT_LINE);
}

void dmv_k234_device::snr_toggle()
{
	// one SNR doorbell ($d8-$df), toggled by either CPU; m_await_back tracks
	// which way the bus is being handed over
	if (!m_await_back)
	{
		// Z80 -> 68008: start the 68008 (deferred, see snr_reset)
		m_await_back = 1;
		m_snr_reset_timer->adjust(attotime::zero);
	}
	else
	{
		// 68008 -> Z80: park the 68008, give the bus back to the Z80
		m_await_back = 0;
		m_snr = CLEAR_LINE;
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		out_thold(CLEAR_LINE);
	}
}

uint8_t dmv_k234_device::snr_r()
{
	if (!machine().side_effects_disabled())  // ignore debugger reads
		snr_toggle();

	return 0xff;
}

void dmv_k234_device::snr_w(uint8_t data)
{
	snr_toggle();
}

// RCSEL: BA19-BA16 high and every address line A15-A12 whose S3 switch is ON
bool dmv_k235p_device::rcsel(offs_t offset)
{
	if ((offset & 0xf0000) != 0xf0000)
		return false;

	offs_t const decoded = (~m_eprom->read() & 0x0f) << 12;   // S3:1 = A15 ... S3:4 = A12
	return (offset & decoded) == decoded;
}

uint8_t dmv_k235p_device::k235p_mem_r(offs_t offset)
{
	if (rcsel(offset))
	{
		// A0-A11 always, A12-A15 as far as the EPROM has them (the rest are VPP/PGM at +5 V)
		static const offs_t eprom_mask[8] = { 0x0fff, 0x1fff, 0x3fff, 0x7fff, 0xffff, 0xffff, 0xffff, 0xffff };
		offs_t const mask = eprom_mask[(m_eprom->read() >> 4) & 7];
		return m_rom->base()[offset & mask];
	}

	return program_r(offset);
}

void dmv_k235p_device::k235p_mem_w(offs_t offset, uint8_t data)
{
	// the 8288 runs a bus write cycle for the EPROM window too
	program_w(offset, data);
}
