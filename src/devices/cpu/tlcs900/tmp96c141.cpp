// license:BSD-3-Clause
// copyright-holders:AJR, Felipe Sanches
/****************************************************************************

    Toshiba TMP96C141 microcontroller

    TMP96CM40 is the same as TMP96C141 with the addition of 32K of on-chip
    ROM. TMP96PM40 is similar but with 32K OTP. TMP96C041 is RAMless as well
    as ROMless.

    Built-in peripherals, after the TMP96C141AF data book:
    - I/O ports P0-P9 with control and function registers
    - 8-bit timers 0/1 (8-bit, 16-bit, PPG and PWM modes) with TFF1
    - 8-bit PWM timers 2/3 (PWM and timer modes) with TFF2/TFF3
    - 16-bit timers 4/5 with compare, capture and TFF4/TFF5/TFF6
    - stepping motor/pattern generator PG0/PG1 on port 6
    - two serial channels (UART, whole bytes only)
    - 10-bit A/D converter with four inputs
    - watchdog timer
    - interrupt controller with six levels and four micro DMA channels

    One state is two oscillator clocks: the device clock is the crystal
    frequency fc, and the prescalers count in units of fc.

    TODO:
    - serial I/O interface mode, 9-bit UART and wake-up
    - bus request/acknowledge, standby modes

****************************************************************************/

#include "emu.h"
#include "tmp96c141.h"
#include "dasm900.h"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(TMP96C141, tmp96c141_device, "tmp96c141", "Toshiba TMP96C141")


namespace {

enum
{
	INTE0AD = 0, INTE45, INTE67, INTET10, INTEPW10, INTET54, INTET76, INTES0, INTES1
};

enum
{
	IRQ_INT0 = 0, IRQ_INT4, IRQ_INT5, IRQ_INT6, IRQ_INT7,
	IRQ_INTT0, IRQ_INTT1, IRQ_INTT2, IRQ_INTT3,
	IRQ_INTTR4, IRQ_INTTR5, IRQ_INTTR6, IRQ_INTTR7,
	IRQ_INTRX0, IRQ_INTTX0, IRQ_INTRX1, IRQ_INTTX1,
	IRQ_INTAD,
	NUM_IRQS
};

// interrupt enable register, nibble and vector of each maskable source,
// in order of default priority (lowest vector first)
const struct
{
	u8 reg;
	u8 shift;
	u16 vector;
} s_irq[NUM_IRQS] =
{
	{ INTE0AD,  0, 0x0a0 },  // INT0
	{ INTE45,   0, 0x0b0 },  // INT4
	{ INTE45,   4, 0x0c0 },  // INT5
	{ INTE67,   0, 0x0d0 },  // INT6
	{ INTE67,   4, 0x0e0 },  // INT7
	{ INTET10,  0, 0x100 },  // INTT0
	{ INTET10,  4, 0x110 },  // INTT1
	{ INTEPW10, 0, 0x120 },  // INTT2
	{ INTEPW10, 4, 0x130 },  // INTT3
	{ INTET54,  0, 0x140 },  // INTTR4
	{ INTET54,  4, 0x150 },  // INTTR5
	{ INTET76,  0, 0x160 },  // INTTR6
	{ INTET76,  4, 0x170 },  // INTTR7
	{ INTES0,   0, 0x180 },  // INTRX0
	{ INTES0,   4, 0x190 },  // INTTX0
	{ INTES1,   0, 0x1a0 },  // INTRX1
	{ INTES1,   4, 0x1b0 },  // INTTX1
	{ INTE0AD,  4, 0x1c0 }   // INTAD
};

constexpr u16 VECTOR_NMI = 0x080;
constexpr u16 VECTOR_INTWD = 0x090;

// prescaler taps, in oscillator clocks
constexpr int PRE_T1 = 3;     // phiT1 = fc/8
constexpr int PRE_T4 = 5;     // phiT4 = fc/32
constexpr int PRE_T16 = 7;    // phiT16 = fc/128
constexpr int PRE_T256 = 11;  // phiT256 = fc/2048
constexpr int PRE_P1 = 2;     // phiP1 = fc/4
constexpr int PRE_P4 = 4;     // phiP4 = fc/16
constexpr int PRE_P16 = 6;    // phiP16 = fc/64

} // anonymous namespace


//**************************************************************************
//  DEVICE CONSTRUCTION AND INITIALIZATION
//**************************************************************************

//-------------------------------------------------
//  tmp96c141_device - constructor
//-------------------------------------------------

tmp96c141_device::tmp96c141_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: tlcs900_device(mconfig, TMP96C141, tag, owner, clock)
	, m_port_read(*this, 0xff)
	, m_port_write(*this)
	, m_an_read(*this, 0)
	, m_txd_cb(*this)
{
}


//-------------------------------------------------
//  device_config_complete - device-specific startup
//-------------------------------------------------

void tmp96c141_device::device_config_complete()
{
	if (m_am8_16 == 0)
		m_program_config = address_space_config("program", ENDIANNESS_LITTLE, 16, 24, 0, address_map_constructor(FUNC(tmp96c141_device::internal_mem), this));
	else
		m_program_config = address_space_config("program", ENDIANNESS_LITTLE, 8, 24, 0, address_map_constructor(FUNC(tmp96c141_device::internal_mem), this));
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void tmp96c141_device::device_start()
{
	tlcs900_device::device_start();

	m_nmi_state = CLEAR_LINE;
	std::fill(std::begin(m_level), std::end(m_level), CLEAR_LINE);

	save_item(NAME(m_port_latch));
	save_item(NAME(m_port_cr));
	save_item(NAME(m_port_fc));
	save_item(NAME(m_trun));
	save_item(NAME(m_treg));
	save_item(NAME(m_treg_buf));
	save_item(NAME(m_tmod));
	save_item(NAME(m_tffcr));
	save_item(NAME(m_pwmod));
	save_item(NAME(m_pffcr));
	save_item(NAME(m_tff));
	save_item(NAME(m_uc16));
	save_item(NAME(m_treg16));
	save_item(NAME(m_treg16_buf));
	save_item(NAME(m_cap));
	save_item(NAME(m_t4mod));
	save_item(NAME(m_t4ffcr));
	save_item(NAME(m_t5mod));
	save_item(NAME(m_t5ffcr));
	save_item(NAME(m_t45cr));
	save_item(NAME(m_pgreg));
	save_item(NAME(m_pg01cr));
	save_item(NAME(m_scbuf_rx));
	save_item(NAME(m_sccr));
	save_item(NAME(m_scmod));
	save_item(NAME(m_brcr));
	save_item(NAME(m_ode));
	save_item(NAME(m_tx_cycles));
	save_item(NAME(m_tx_data));
	save_item(NAME(m_wdmod));
	save_item(NAME(m_wd_enabled));
	save_item(NAME(m_wd_counter));
	save_item(NAME(m_intwd));
	save_item(NAME(m_admod));
	save_item(NAME(m_adreg));
	save_item(NAME(m_ad_channel));
	save_item(NAME(m_bcs));
	save_item(NAME(m_inte));
	save_item(NAME(m_iimc));
	save_item(NAME(m_dmav));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void tmp96c141_device::device_reset()
{
	tlcs900_device::device_reset();

	static const u8 latch_reset[NUM_PORTS] = { 0x00, 0x00, 0x00, 0xff, 0x03, 0x0f, 0xff, 0x0f, 0xff, 0x3f };
	std::copy(std::begin(latch_reset), std::end(latch_reset), std::begin(m_port_latch));
	std::fill(std::begin(m_port_cr), std::end(m_port_cr), 0);
	std::fill(std::begin(m_port_fc), std::end(m_port_fc), 0);

	m_timer_pre = 0;
	std::fill(std::begin(m_timer_8), std::end(m_timer_8), 0);
	std::fill(std::begin(m_timer_change), std::end(m_timer_change), 0);
	m_trun = 0;
	std::fill(std::begin(m_treg), std::end(m_treg), 0);
	std::fill(std::begin(m_treg_buf), std::end(m_treg_buf), 0);
	m_tmod = 0;
	m_tffcr = 0;
	std::fill(std::begin(m_pwmod), std::end(m_pwmod), 0);
	m_pffcr = 0;
	m_tff = 0;

	std::fill(std::begin(m_uc16), std::end(m_uc16), 0);
	std::fill(std::begin(m_treg16), std::end(m_treg16), 0);
	std::fill(std::begin(m_treg16_buf), std::end(m_treg16_buf), 0);
	std::fill(std::begin(m_cap), std::end(m_cap), 0);
	m_t4mod = 0x20;
	m_t4ffcr = 0;
	m_t5mod = 0x20;
	m_t5ffcr = 0;
	m_t45cr = 0;

	std::fill(std::begin(m_pgreg), std::end(m_pgreg), 0);
	m_pg01cr = 0;

	std::fill(std::begin(m_scbuf_rx), std::end(m_scbuf_rx), 0);
	std::fill(std::begin(m_sccr), std::end(m_sccr), 0);
	std::fill(std::begin(m_scmod), std::end(m_scmod), 0);
	std::fill(std::begin(m_brcr), std::end(m_brcr), 0);
	m_ode = 0;
	std::fill(std::begin(m_tx_cycles), std::end(m_tx_cycles), 0);
	std::fill(std::begin(m_tx_data), std::end(m_tx_data), 0);

	m_wdmod = 0x80;
	m_wd_enabled = true;
	m_wd_counter = 0;
	m_intwd = false;

	m_admod = 0;
	std::fill(std::begin(m_adreg), std::end(m_adreg), 0);
	m_ad_channel = 0;
	m_ad_cycles_left = 0;

	m_bcs[0] = 0x00;
	m_bcs[1] = 0x00;
	m_bcs[2] = 0x80;

	std::fill(std::begin(m_inte), std::end(m_inte), 0);
	m_iimc = 0;
	std::fill(std::begin(m_dmav), std::end(m_dmav), 0);

	for (unsigned p = 0; p < NUM_PORTS; p++)
		update_port(p);
}


//**************************************************************************
//  INTERNAL REGISTERS
//**************************************************************************

//-------------------------------------------------
//  internal_mem - memory map for internal RAM and
//  I/O registers
//-------------------------------------------------

void tmp96c141_device::internal_mem(address_map &map)
{
	map(0x000000, 0x00007f).unmaprw();
	map(0x000000, 0x000000).rw(FUNC(tmp96c141_device::port_r<0>), FUNC(tmp96c141_device::port_w<0>));
	map(0x000001, 0x000001).rw(FUNC(tmp96c141_device::port_r<1>), FUNC(tmp96c141_device::port_w<1>));
	map(0x000002, 0x000002).w(FUNC(tmp96c141_device::port_cr_w<0>));
	map(0x000004, 0x000004).w(FUNC(tmp96c141_device::port_cr_w<1>));
	map(0x000005, 0x000005).w(FUNC(tmp96c141_device::port_fc_w<1>));
	map(0x000006, 0x000006).rw(FUNC(tmp96c141_device::port_r<2>), FUNC(tmp96c141_device::port_w<2>));
	map(0x000007, 0x000007).rw(FUNC(tmp96c141_device::port_r<3>), FUNC(tmp96c141_device::port_w<3>));
	map(0x000008, 0x000008).w(FUNC(tmp96c141_device::port_cr_w<2>));
	map(0x000009, 0x000009).w(FUNC(tmp96c141_device::port_fc_w<2>));
	map(0x00000a, 0x00000a).w(FUNC(tmp96c141_device::port_cr_w<3>));
	map(0x00000b, 0x00000b).w(FUNC(tmp96c141_device::port_fc_w<3>));
	map(0x00000c, 0x00000c).rw(FUNC(tmp96c141_device::port_r<4>), FUNC(tmp96c141_device::port_w<4>));
	map(0x00000d, 0x00000d).r(FUNC(tmp96c141_device::port_r<5>));
	map(0x00000e, 0x00000e).w(FUNC(tmp96c141_device::port_cr_w<4>));
	map(0x000010, 0x000010).w(FUNC(tmp96c141_device::port_fc_w<4>));
	map(0x000012, 0x000012).rw(FUNC(tmp96c141_device::port_r<6>), FUNC(tmp96c141_device::port_w<6>));
	map(0x000013, 0x000013).rw(FUNC(tmp96c141_device::port_r<7>), FUNC(tmp96c141_device::port_w<7>));
	map(0x000014, 0x000014).w(FUNC(tmp96c141_device::port_cr_w<6>));
	map(0x000015, 0x000015).w(FUNC(tmp96c141_device::port_cr_w<7>));
	map(0x000016, 0x000016).w(FUNC(tmp96c141_device::port_fc_w<6>));
	map(0x000017, 0x000017).w(FUNC(tmp96c141_device::port_fc_w<7>));
	map(0x000018, 0x000018).rw(FUNC(tmp96c141_device::port_r<8>), FUNC(tmp96c141_device::port_w<8>));
	map(0x000019, 0x000019).rw(FUNC(tmp96c141_device::port_r<9>), FUNC(tmp96c141_device::port_w<9>));
	map(0x00001a, 0x00001a).w(FUNC(tmp96c141_device::port_cr_w<8>));
	map(0x00001b, 0x00001b).w(FUNC(tmp96c141_device::port_cr_w<9>));
	map(0x00001c, 0x00001c).w(FUNC(tmp96c141_device::port_fc_w<8>));
	map(0x00001d, 0x00001d).w(FUNC(tmp96c141_device::port_fc_w<9>));
	map(0x000020, 0x000020).rw(FUNC(tmp96c141_device::trun_r), FUNC(tmp96c141_device::trun_w));
	map(0x000022, 0x000023).w(FUNC(tmp96c141_device::treg01_w));
	map(0x000024, 0x000024).w(FUNC(tmp96c141_device::tmod_w));
	map(0x000025, 0x000025).rw(FUNC(tmp96c141_device::tffcr_r), FUNC(tmp96c141_device::tffcr_w));
	map(0x000026, 0x000027).rw(FUNC(tmp96c141_device::treg23_r), FUNC(tmp96c141_device::treg23_w));
	map(0x000028, 0x000029).rw(FUNC(tmp96c141_device::pwmod_r), FUNC(tmp96c141_device::pwmod_w));
	map(0x00002a, 0x00002a).rw(FUNC(tmp96c141_device::pffcr_r), FUNC(tmp96c141_device::pffcr_w));
	map(0x000030, 0x000033).w(FUNC(tmp96c141_device::treg45_w));
	map(0x000034, 0x000037).r(FUNC(tmp96c141_device::cap12_r));
	map(0x000038, 0x000038).rw(FUNC(tmp96c141_device::t4mod_r), FUNC(tmp96c141_device::t4mod_w));
	map(0x000039, 0x000039).rw(FUNC(tmp96c141_device::t4ffcr_r), FUNC(tmp96c141_device::t4ffcr_w));
	map(0x00003a, 0x00003a).rw(FUNC(tmp96c141_device::t45cr_r), FUNC(tmp96c141_device::t45cr_w));
	map(0x000040, 0x000043).w(FUNC(tmp96c141_device::treg67_w));
	map(0x000044, 0x000047).r(FUNC(tmp96c141_device::cap34_r));
	map(0x000048, 0x000048).rw(FUNC(tmp96c141_device::t5mod_r), FUNC(tmp96c141_device::t5mod_w));
	map(0x000049, 0x000049).rw(FUNC(tmp96c141_device::t5ffcr_r), FUNC(tmp96c141_device::t5ffcr_w));
	map(0x00004c, 0x00004d).rw(FUNC(tmp96c141_device::pgreg_r), FUNC(tmp96c141_device::pgreg_w));
	map(0x00004e, 0x00004e).rw(FUNC(tmp96c141_device::pg01cr_r), FUNC(tmp96c141_device::pg01cr_w));
	map(0x000050, 0x000050).select(4).rw(FUNC(tmp96c141_device::scbuf_r), FUNC(tmp96c141_device::scbuf_w));
	map(0x000051, 0x000051).select(4).rw(FUNC(tmp96c141_device::sccr_r), FUNC(tmp96c141_device::sccr_w));
	map(0x000052, 0x000052).select(4).rw(FUNC(tmp96c141_device::scmod_r), FUNC(tmp96c141_device::scmod_w));
	map(0x000053, 0x000053).select(4).rw(FUNC(tmp96c141_device::brcr_r), FUNC(tmp96c141_device::brcr_w));
	map(0x000058, 0x000058).rw(FUNC(tmp96c141_device::ode_r), FUNC(tmp96c141_device::ode_w));
	map(0x00005c, 0x00005c).rw(FUNC(tmp96c141_device::wdmod_r), FUNC(tmp96c141_device::wdmod_w));
	map(0x00005d, 0x00005d).w(FUNC(tmp96c141_device::wdcr_w));
	map(0x00005e, 0x00005e).rw(FUNC(tmp96c141_device::admod_r), FUNC(tmp96c141_device::admod_w));
	map(0x000060, 0x000067).r(FUNC(tmp96c141_device::adreg_r));
	map(0x000068, 0x00006a).w(FUNC(tmp96c141_device::bcs_w));
	map(0x000070, 0x000078).rw(FUNC(tmp96c141_device::inte_r), FUNC(tmp96c141_device::inte_w));
	map(0x00007b, 0x00007b).w(FUNC(tmp96c141_device::iimc_w));
	map(0x00007c, 0x00007f).w(FUNC(tmp96c141_device::dmav_w));
	map(0x000080, 0x00047f).ram();
}


//**************************************************************************
//  I/O PORTS
//**************************************************************************

// value driven by the peripheral functions of a port, for the pins whose
// function register bit is set
u8 tmp96c141_device::port_function_out(unsigned port) const
{
	switch (port)
	{
	case 6:
		return (m_pgreg[0] >> 4) | (m_pgreg[1] & 0xf0);
	case 7:
		return m_tff & 0x0e;                                  // TO1, TO2, TO3
	case 8:
		return (BIT(m_tff, 4) << 2) | (BIT(m_tff, 5) << 3) | (BIT(m_tff, 6) << 6);   // TO4, TO5, TO6
	case 9:
		return 0x09;                                          // TXD0/TXD1 idle (whole-byte serial)
	default:
		return m_port_latch[port];
	}
}

void tmp96c141_device::update_port(unsigned port)
{
	u8 const fc = (port >= 6) ? m_port_fc[port] : 0;
	u8 const data = (m_port_latch[port] & ~fc) | (port_function_out(port) & fc);
	m_port_write[port](0, data, m_port_cr[port]);
}

template <unsigned N>
u8 tmp96c141_device::port_r()
{
	u8 const ext = m_port_read[N](0);
	if (N == 5)
		return ext;

	u8 const fc = (N >= 6) ? m_port_fc[N] : 0;
	u8 const out = (m_port_latch[N] & ~fc) | (port_function_out(N) & fc);
	return (out & m_port_cr[N]) | (ext & ~m_port_cr[N]);
}

template <unsigned N>
void tmp96c141_device::port_w(u8 data)
{
	m_port_latch[N] = data;
	update_port(N);
}

template <unsigned N>
void tmp96c141_device::port_cr_w(u8 data)
{
	m_port_cr[N] = data;
	update_port(N);
}

template <unsigned N>
void tmp96c141_device::port_fc_w(u8 data)
{
	m_port_fc[N] = data;
	update_port(N);
}


//**************************************************************************
//  TIMERS
//**************************************************************************

// TFF1..TFF6 change; the timer outputs follow on ports 7 and 8
void tmp96c141_device::set_tff(unsigned which, int state)
{
	u8 const old = m_tff;
	if (state < 0)
		m_tff ^= 1 << which;
	else if (state)
		m_tff |= 1 << which;
	else
		m_tff &= ~(1 << which);

	if (old == m_tff)
		return;

	if (which == 1)
	{
		// capture on TFF1 edges
		bool const rise = BIT(m_tff, 1);
		if (((m_t4mod >> 3) & 3) == 3)
			capture(rise ? 0 : 1);
		if (((m_t5mod >> 3) & 3) == 3)
			capture(rise ? 2 : 3);
	}
	update_port(which <= 3 ? 7 : 8);
}

void tmp96c141_device::set_tff1(int state)
{
	set_tff(1, state);
}

// PG shift trigger from timers 0/1 (the TFF1 inversion trigger)
void tmp96c141_device::tff1_trigger()
{
	if (!BIT(m_t45cr, 2))
		pg_shift(0);
	if (!BIT(m_t45cr, 3))
		pg_shift(1);
}

void tmp96c141_device::capture(unsigned reg)
{
	m_cap[reg] = m_uc16[reg >> 1];
	switch (reg)
	{
	case 0:
		if (BIT(m_t4ffcr, 4))
			set_tff(4, -1);
		break;
	case 1:
		if (BIT(m_t4ffcr, 5))
			set_tff(4, -1);
		if (BIT(m_t4mod, 7))
			set_tff(5, -1);
		break;
	case 2:
		if (BIT(m_t5ffcr, 4))
			set_tff(6, -1);
		break;
	case 3:
		if (BIT(m_t5ffcr, 5))
			set_tff(6, -1);
		break;
	}
}

// timers 0 and 1, in the mode TMOD selects
void tmp96c141_device::tick_timer01(unsigned timer, u32 ticks)
{
	u8 const mode = m_tmod >> 6;
	bool const tff_enable = BIT(m_tffcr, 1);

	if (timer == 0)
	{
		for (; ticks; ticks--)
		{
			switch (mode)
			{
			case 0: // two 8-bit timers
			{
				unsigned const uc = m_timer_8[0] + 1;
				if (uc == (m_treg[0] ? m_treg[0] : 0x100))
				{
					m_timer_8[0] = 0;
					set_irq(IRQ_INTT0);
					if (tff_enable && !BIT(m_tffcr, 0))
					{
						set_tff1(-1);
						tff1_trigger();
					}
					if (BIT(m_trun, 1) && ((m_tmod >> 2) & 3) == 0)
						tick_timer01(1, 1);
				}
				else
					m_timer_8[0] = uc;
				break;
			}

			case 1: // one 16-bit timer
			{
				u32 const cmp = (u32(m_treg[1]) << 8) | m_treg[0];
				u32 uc = ((u32(m_timer_8[1]) << 8) | m_timer_8[0]) + 1;
				if ((uc & 0xff) == m_treg[0])
					set_irq(IRQ_INTT0);
				if (uc == (cmp ? cmp : 0x10000))
				{
					uc = 0;
					set_irq(IRQ_INTT1);
					if (tff_enable)
					{
						set_tff1(-1);
						tff1_trigger();
					}
				}
				m_timer_8[0] = uc & 0xff;
				m_timer_8[1] = (uc >> 8) & 0xff;
				break;
			}

			case 2: // 8-bit PPG
			{
				unsigned uc = m_timer_8[0] + 1;
				if (uc == m_treg[0])
				{
					set_irq(IRQ_INTT0);
					if (tff_enable)
						set_tff1(-1);
				}
				if (uc == (m_treg[1] ? m_treg[1] : 0x100))
				{
					uc = 0;
					set_irq(IRQ_INTT1);
					if (tff_enable)
					{
						set_tff1(-1);
						tff1_trigger();
					}
					if (BIT(m_tffcr, 4))
						m_treg[0] = m_treg_buf[0];
				}
				m_timer_8[0] = uc & 0xff;
				break;
			}

			case 3: // 8-bit PWM
			{
				unsigned const period = (1U << (5 + std::max<unsigned>((m_tmod >> 4) & 3, 1))) - 1;
				unsigned uc = m_timer_8[0] + 1;
				if (uc == period)
				{
					uc = 0;
					set_irq(IRQ_INTT0);
					if (tff_enable)
						set_tff1(-1);
					if (BIT(m_tffcr, 4))
						m_treg[0] = m_treg_buf[0];
				}
				else if (uc == m_treg[0])
				{
					if (tff_enable)
						set_tff1(-1);
				}
				m_timer_8[0] = uc & 0xff;
				break;
			}
			}
		}
	}
	else
	{
		// timer 1 on its own clock: only in 8-bit timer and PWM modes
		if (mode == 1 || mode == 2)
			return;

		for (; ticks; ticks--)
		{
			unsigned const uc = m_timer_8[1] + 1;
			if (uc == (m_treg[1] ? m_treg[1] : 0x100))
			{
				m_timer_8[1] = 0;
				set_irq(IRQ_INTT1);
				if (mode == 0 && tff_enable && BIT(m_tffcr, 0))
				{
					set_tff1(-1);
					tff1_trigger();
				}
			}
			else
				m_timer_8[1] = uc;
		}
	}
}

// PWM timers 2 and 3
void tmp96c141_device::tick_timer23(unsigned timer, u32 ticks)
{
	u8 const mod = m_pwmod[timer];
	u8 const trg = (m_pffcr >> (4 * timer)) & 3;
	unsigned const tff = 2 + timer;
	unsigned const irq = IRQ_INTT2 + timer;

	for (; ticks; ticks--)
	{
		unsigned uc = m_timer_8[2 + timer] + 1;
		if (BIT(mod, 4))
		{
			// 8-bit timer mode
			if (uc == (m_treg[2 + timer] ? m_treg[2 + timer] : 0x100))
			{
				uc = 0;
				if (trg == 1)
					set_tff(tff, -1);
				if (BIT(mod, 5))
					set_irq(irq);
				if (BIT(mod, 6))
					m_treg[2 + timer] = m_treg_buf[2 + timer];
			}
		}
		else
		{
			// PWM mode: the 2^n-1 overflow takes priority over the match
			unsigned const period = (1U << (5 + std::max<unsigned>(mod & 3, 1))) - 1;
			if (uc == period)
			{
				uc = 0;
				switch (trg)
				{
				case 1: set_tff(tff, -1); break;
				case 2: set_tff(tff, 0); break;
				case 3: set_tff(tff, 1); break;
				}
				if (!BIT(mod, 5))
					set_irq(irq);
				if (BIT(mod, 6))
					m_treg[2 + timer] = m_treg_buf[2 + timer];
			}
			else if (uc == m_treg[2 + timer])
			{
				switch (trg)
				{
				case 1: set_tff(tff, -1); break;
				case 2: set_tff(tff, 1); break;
				case 3: set_tff(tff, 0); break;
				}
				if (BIT(mod, 5))
					set_irq(irq);
			}
		}
		m_timer_8[2 + timer] = uc & 0xff;
	}
}

// 16-bit timers 4 and 5
void tmp96c141_device::tick_timer45(unsigned timer, u32 ticks)
{
	for (; ticks; ticks--)
	{
		u16 const uc = ++m_uc16[timer];

		if (timer == 0)
		{
			if (uc == m_treg16[0])
			{
				set_irq(IRQ_INTTR4);
				if (BIT(m_t4ffcr, 2))
					set_tff(4, -1);
			}
			if (uc == m_treg16[1])
			{
				set_irq(IRQ_INTTR5);
				if (BIT(m_t4ffcr, 3))
					set_tff(4, -1);
				if (BIT(m_t4mod, 6))
					set_tff(5, -1);
				if (BIT(m_t45cr, 2) && (BIT(m_t4ffcr, 3) || BIT(m_t4mod, 6)))
					pg_shift(0);
				if (BIT(m_t4mod, 2))
					m_uc16[0] = 0;
				if (BIT(m_t45cr, 0))
					m_treg16[0] = m_treg16_buf[0];
			}
		}
		else
		{
			if (uc == m_treg16[2])
			{
				set_irq(IRQ_INTTR6);
				if (BIT(m_t5ffcr, 2))
					set_tff(6, -1);
			}
			if (uc == m_treg16[3])
			{
				set_irq(IRQ_INTTR7);
				if (BIT(m_t5ffcr, 3))
				{
					set_tff(6, -1);
					if (BIT(m_t45cr, 3))
						pg_shift(1);
				}
				if (BIT(m_t5mod, 2))
					m_uc16[1] = 0;
				if (BIT(m_t45cr, 1))
					m_treg16[2] = m_treg16_buf[2];
			}
		}
	}
}

void tmp96c141_device::tlcs900_handle_timers()
{
	u32 const old_pre = m_timer_pre;
	if (BIT(m_trun, 7))
		m_timer_pre += m_cycles * 2;

	auto ticks = [this, old_pre] (int shift) -> u32 { return ((m_timer_pre >> shift) - (old_pre >> shift)) & (0xffffffffU >> shift); };

	// timer 0 (timer 1 follows it in 16-bit, PPG and on match)
	if (BIT(m_trun, 0))
	{
		u32 n = 0;
		switch (m_tmod & 3)
		{
		case 0: n = m_timer_change[0]; m_timer_change[0] = 0; break;
		case 1: n = ticks(PRE_T1); break;
		case 2: n = ticks(PRE_T4); break;
		case 3: n = ticks(PRE_T16); break;
		}
		if (n)
			tick_timer01(0, n);
	}

	// timer 1 on its own clock
	if (BIT(m_trun, 1))
	{
		u32 n = 0;
		switch ((m_tmod >> 2) & 3)
		{
		case 1: n = ticks(PRE_T1); break;
		case 2: n = ticks(PRE_T16); break;
		case 3: n = ticks(PRE_T256); break;
		}
		if (n)
			tick_timer01(1, n);
	}

	// PWM timers 2 and 3
	for (unsigned t = 0; t < 2; t++)
	{
		if (BIT(m_trun, 2 + t))
		{
			u32 n = 0;
			switch ((m_pwmod[t] >> 2) & 3)
			{
			case 0: n = ticks(PRE_P1); break;
			case 1: n = ticks(PRE_P4); break;
			case 2: n = ticks(PRE_P16); break;
			}
			if (n)
				tick_timer23(t, n);
		}
	}

	// 16-bit timers 4 and 5
	for (unsigned t = 0; t < 2; t++)
	{
		if (BIT(m_trun, 4 + t))
		{
			u32 n = 0;
			switch ((t ? m_t5mod : m_t4mod) & 3)
			{
			case 0: n = m_timer_change[4 + t]; m_timer_change[4 + t] = 0; break;
			case 1: n = ticks(PRE_T1); break;
			case 2: n = ticks(PRE_T4); break;
			case 3: n = ticks(PRE_T16); break;
			}
			if (n)
				tick_timer45(t, n);
		}
	}

	// serial transmitters (whole bytes)
	for (unsigned ch = 0; ch < 2; ch++)
	{
		if (m_tx_cycles[ch] > 0)
		{
			m_tx_cycles[ch] -= m_cycles;
			if (m_tx_cycles[ch] <= 0)
			{
				m_tx_cycles[ch] = 0;
				m_txd_cb[ch](m_tx_data[ch]);
				set_irq(ch ? IRQ_INTTX1 : IRQ_INTTX0);
			}
		}
	}

	// watchdog timer, counting states
	if (m_wd_enabled)
	{
		u32 const limit = 1U << (15 + 2 * ((m_wdmod >> 5) & 3));
		m_wd_counter += m_cycles;
		if (m_wd_counter >= limit)
		{
			m_wd_counter -= limit;
			if (BIT(m_wdmod, 1))
			{
				logerror("watchdog reset\n");
				pulse_input_line(INPUT_LINE_RESET, attotime::zero);
			}
			else
			{
				m_intwd = true;
				m_check_irqs = 1;
			}
		}
	}
}

u8 tmp96c141_device::trun_r()
{
	return m_trun;
}

void tmp96c141_device::trun_w(u8 data)
{
	if (!BIT(data, 0))
	{
		m_timer_8[0] = 0;
		m_timer_change[0] = 0;
	}
	if (!BIT(data, 1))
		m_timer_8[1] = 0;
	if (!BIT(data, 2))
		m_timer_8[2] = 0;
	if (!BIT(data, 3))
		m_timer_8[3] = 0;
	if (!BIT(data, 4))
	{
		m_uc16[0] = 0;
		m_timer_change[4] = 0;
	}
	if (!BIT(data, 5))
	{
		m_uc16[1] = 0;
		m_timer_change[5] = 0;
	}
	if (!BIT(data, 7))
		m_timer_pre = 0;

	m_trun = data & 0xbf;
}

void tmp96c141_device::treg01_w(offs_t offset, u8 data)
{
	m_treg_buf[offset] = data;
	if (offset != 0 || !BIT(m_tffcr, 4))
		m_treg[offset] = data;
}

void tmp96c141_device::tmod_w(u8 data)
{
	m_tmod = data;
}

u8 tmp96c141_device::tffcr_r()
{
	return (m_tffcr & 0x13) | 0x0c;
}

void tmp96c141_device::tffcr_w(u8 data)
{
	switch ((data >> 2) & 3)
	{
	case 0: set_tff1(-1); break;
	case 1: set_tff1(1); break;
	case 2: set_tff1(0); break;
	}
	m_tffcr = data & 0x13;
}

u8 tmp96c141_device::treg23_r(offs_t offset)
{
	return m_treg_buf[2 + offset];
}

void tmp96c141_device::treg23_w(offs_t offset, u8 data)
{
	m_treg_buf[2 + offset] = data;
	if (!BIT(m_pwmod[offset], 6))
		m_treg[2 + offset] = data;
}

u8 tmp96c141_device::pwmod_r(offs_t offset)
{
	return (m_pwmod[offset] & 0x7f) | (BIT(m_tff, 2 + offset) << 7);
}

void tmp96c141_device::pwmod_w(offs_t offset, u8 data)
{
	m_pwmod[offset] = data & 0x7f;
}

u8 tmp96c141_device::pffcr_r()
{
	return (m_pffcr & 0x33) | 0xcc;
}

void tmp96c141_device::pffcr_w(u8 data)
{
	for (unsigned t = 0; t < 2; t++)
	{
		switch ((data >> (2 + 4 * t)) & 3)
		{
		case 1: set_tff(2 + t, 1); break;
		case 2: set_tff(2 + t, 0); break;
		}
	}
	m_pffcr = data & 0x33;
}

void tmp96c141_device::treg45_w(offs_t offset, u8 data)
{
	unsigned const reg = offset >> 1;
	u16 const mask = BIT(offset, 0) ? 0xff00 : 0x00ff;
	u16 const value = BIT(offset, 0) ? (u16(data) << 8) : data;
	m_treg16_buf[reg] = (m_treg16_buf[reg] & ~mask) | value;
	if (reg != 0 || !BIT(m_t45cr, 0))
		m_treg16[reg] = (m_treg16[reg] & ~mask) | value;
}

u8 tmp96c141_device::cap12_r(offs_t offset)
{
	return BIT(offset, 0) ? (m_cap[offset >> 1] >> 8) : (m_cap[offset >> 1] & 0xff);
}

u8 tmp96c141_device::t4mod_r()
{
	return m_t4mod | 0x20;
}

void tmp96c141_device::t4mod_w(u8 data)
{
	m_t4mod = data | 0x20;
	if (!BIT(data, 5) && BIT(m_trun, 7))
		capture(0);
}

u8 tmp96c141_device::t4ffcr_r()
{
	return m_t4ffcr | 0xc3;
}

void tmp96c141_device::t4ffcr_w(u8 data)
{
	switch (data & 3)
	{
	case 0: set_tff(4, -1); break;
	case 1: set_tff(4, 1); break;
	case 2: set_tff(4, 0); break;
	}
	switch ((data >> 6) & 3)
	{
	case 0: set_tff(5, -1); break;
	case 1: set_tff(5, 1); break;
	case 2: set_tff(5, 0); break;
	}
	m_t4ffcr = data & 0x3c;
}

u8 tmp96c141_device::t45cr_r()
{
	return m_t45cr;
}

void tmp96c141_device::t45cr_w(u8 data)
{
	m_t45cr = data & 0x0f;
}

void tmp96c141_device::treg67_w(offs_t offset, u8 data)
{
	unsigned const reg = 2 + (offset >> 1);
	u16 const mask = BIT(offset, 0) ? 0xff00 : 0x00ff;
	u16 const value = BIT(offset, 0) ? (u16(data) << 8) : data;
	m_treg16_buf[reg] = (m_treg16_buf[reg] & ~mask) | value;
	if (reg != 2 || !BIT(m_t45cr, 1))
		m_treg16[reg] = (m_treg16[reg] & ~mask) | value;
}

u8 tmp96c141_device::cap34_r(offs_t offset)
{
	return BIT(offset, 0) ? (m_cap[2 + (offset >> 1)] >> 8) : (m_cap[2 + (offset >> 1)] & 0xff);
}

u8 tmp96c141_device::t5mod_r()
{
	return m_t5mod | 0x20;
}

void tmp96c141_device::t5mod_w(u8 data)
{
	m_t5mod = (data & 0x3f) | 0x20;
	if (!BIT(data, 5) && BIT(m_trun, 7))
		capture(2);
}

u8 tmp96c141_device::t5ffcr_r()
{
	return m_t5ffcr | 0x03;
}

void tmp96c141_device::t5ffcr_w(u8 data)
{
	switch (data & 3)
	{
	case 0: set_tff(6, -1); break;
	case 1: set_tff(6, 1); break;
	case 2: set_tff(6, 0); break;
	}
	m_t5ffcr = data & 0x3c;
}


//**************************************************************************
//  PATTERN GENERATOR
//**************************************************************************

void tmp96c141_device::pg_shift(unsigned ch)
{
	u8 const cr = (m_pg01cr >> (4 * ch)) & 0x0f;
	if (!BIT(cr, 0))
		return;

	u8 const reg = m_pgreg[ch];
	bool const ccw = BIT(cr, 2);
	u8 result;

	if (!BIT(cr, 1))
	{
		// 4-phase 1- or 2-step excitation: the output latch rotates
		u8 const pg = reg >> 4;
		u8 const next = ccw ? ((pg >> 1) | (pg << 3)) : ((pg << 1) | (pg >> 3));
		result = ((next & 0x0f) << 4) | (reg & 0x0f);
	}
	else
	{
		// 4-phase 1-2 step excitation and pattern generation: the output
		// latch and the shift alternate register form one 8-bit ring,
		// SA0 -> PG0 -> SA1 -> PG1 -> SA2 -> PG2 -> SA3 -> PG3 -> SA0
		u8 ring = 0;
		for (int i = 0; i < 4; i++)
			ring |= (BIT(reg, i) << (2 * i)) | (BIT(reg, 4 + i) << (2 * i + 1));
		ring = ccw ? ((ring >> 1) | (ring << 7)) : ((ring << 1) | (ring >> 7));
		result = 0;
		for (int i = 0; i < 4; i++)
			result |= (BIT(ring, 2 * i) << i) | (BIT(ring, 2 * i + 1) << (4 + i));
	}

	m_pgreg[ch] = result;
	if (m_port_fc[6] & (ch ? 0xf0 : 0x0f))
		update_port(6);
}

u8 tmp96c141_device::pgreg_r(offs_t offset)
{
	return m_pgreg[offset];
}

void tmp96c141_device::pgreg_w(offs_t offset, u8 data)
{
	if (BIT(m_pg01cr, 3 + 4 * offset))
		m_pgreg[offset] = (m_pgreg[offset] & 0xf0) | (data & 0x0f);
	else
		m_pgreg[offset] = data;
	update_port(6);
}

u8 tmp96c141_device::pg01cr_r()
{
	return m_pg01cr;
}

void tmp96c141_device::pg01cr_w(u8 data)
{
	m_pg01cr = data;
}


//**************************************************************************
//  SERIAL CHANNELS
//**************************************************************************

u8 tmp96c141_device::scbuf_r(offs_t offset)
{
	return m_scbuf_rx[offset ? 1 : 0];
}

void tmp96c141_device::scbuf_w(offs_t offset, u8 data)
{
	unsigned const ch = offset ? 1 : 0;

	// UART frame time from the baud rate generator; one bit is 16 of its periods
	unsigned const div = (m_brcr[ch] & 0x0f) ? (m_brcr[ch] & 0x0f) : 16;
	unsigned const pre = 2 + 2 * ((m_brcr[ch] >> 4) & 3);
	unsigned const bits = (((m_scmod[ch] >> 2) & 3) == 3) ? 11 : 10;
	m_tx_data[ch] = data;
	m_tx_cycles[ch] = std::max<s32>((bits * 16 * div) << pre >> 1, 1);
}

u8 tmp96c141_device::sccr_r(offs_t offset)
{
	unsigned const ch = offset ? 1 : 0;
	u8 const data = m_sccr[ch];
	if (!machine().side_effects_disabled())
		m_sccr[ch] &= ~0x1c;   // error flags clear on read
	return data;
}

void tmp96c141_device::sccr_w(offs_t offset, u8 data)
{
	unsigned const ch = offset ? 1 : 0;
	m_sccr[ch] = (m_sccr[ch] & 0x9c) | (data & 0x63);
}

u8 tmp96c141_device::scmod_r(offs_t offset)
{
	return m_scmod[offset ? 1 : 0];
}

void tmp96c141_device::scmod_w(offs_t offset, u8 data)
{
	m_scmod[offset ? 1 : 0] = data;
}

u8 tmp96c141_device::brcr_r(offs_t offset)
{
	return m_brcr[offset ? 1 : 0];
}

void tmp96c141_device::brcr_w(offs_t offset, u8 data)
{
	m_brcr[offset ? 1 : 0] = data & 0x3f;
}

template <unsigned N>
void tmp96c141_device::rx_byte(u8 data)
{
	if (!BIT(m_scmod[N], 5))
		return;
	m_scbuf_rx[N] = data;
	set_irq(N ? IRQ_INTRX1 : IRQ_INTRX0);
}

template void tmp96c141_device::rx_byte<0>(u8 data);
template void tmp96c141_device::rx_byte<1>(u8 data);

u8 tmp96c141_device::ode_r()
{
	return m_ode;
}

void tmp96c141_device::ode_w(u8 data)
{
	m_ode = data & 0x03;
}


//**************************************************************************
//  WATCHDOG TIMER
//**************************************************************************

u8 tmp96c141_device::wdmod_r()
{
	return m_wdmod;
}

void tmp96c141_device::wdmod_w(u8 data)
{
	m_wdmod = data;
	if (BIT(data, 7))
		m_wd_enabled = true;
}

void tmp96c141_device::wdcr_w(u8 data)
{
	if (data == 0x4e)
		m_wd_counter = 0;
	else if (data == 0xb1 && !BIT(m_wdmod, 7))
	{
		m_wd_enabled = false;
		m_wd_counter = 0;
	}
}


//**************************************************************************
//  A/D CONVERTER
//**************************************************************************

u8 tmp96c141_device::admod_r()
{
	return m_admod;
}

void tmp96c141_device::admod_w(u8 data)
{
	m_admod = (m_admod & 0xc0) | (data & 0x3b);
	if (BIT(data, 2))
	{
		m_admod = (m_admod & ~0x80) | 0x40;
		m_ad_channel = BIT(m_admod, 4) ? 0 : (m_admod & 3);
		m_ad_cycles_left = BIT(m_admod, 3) ? 320 : 160;
	}
}

u8 tmp96c141_device::adreg_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		m_admod &= ~0x80;

	u16 const result = m_adreg[offset >> 1];
	return BIT(offset, 0) ? (result >> 2) : (((result & 3) << 6) | 0x3f);
}

void tmp96c141_device::tlcs900_handle_ad()
{
	if (m_ad_cycles_left <= 0)
		return;

	m_ad_cycles_left -= m_cycles;
	if (m_ad_cycles_left > 0)
		return;

	m_adreg[m_ad_channel] = m_an_read[m_ad_channel]() & 0x3ff;

	if (BIT(m_admod, 4) && m_ad_channel < (m_admod & 3))
	{
		// scan mode: next channel
		m_ad_channel++;
		m_ad_cycles_left = BIT(m_admod, 3) ? 320 : 160;
		return;
	}

	m_admod |= 0x80;
	set_irq(IRQ_INTAD);

	if (BIT(m_admod, 5))
	{
		// repeat mode
		m_ad_channel = BIT(m_admod, 4) ? 0 : (m_admod & 3);
		m_ad_cycles_left = BIT(m_admod, 3) ? 320 : 160;
	}
	else
		m_admod &= ~0x40;
}


//**************************************************************************
//  CHIP SELECT/WAIT CONTROLLER
//**************************************************************************

void tmp96c141_device::bcs_w(offs_t offset, u8 data)
{
	m_bcs[offset] = data;
}


//**************************************************************************
//  INTERRUPT CONTROLLER
//**************************************************************************

void tmp96c141_device::set_irq(unsigned irq)
{
	m_inte[s_irq[irq].reg] |= 0x08 << s_irq[irq].shift;
	m_check_irqs = 1;
}

u8 tmp96c141_device::inte_r(offs_t offset)
{
	return m_inte[offset];
}

void tmp96c141_device::inte_w(offs_t offset, u8 data)
{
	// writing 1 to a request flag leaves it alone, writing 0 clears it
	data = (data & 0x77) | (m_inte[offset] & data & 0x88);
	m_inte[offset] = data;
	m_check_irqs = 1;
}

void tmp96c141_device::iimc_w(u8 data)
{
	m_iimc = data & 0x07;
	m_check_irqs = 1;
}

void tmp96c141_device::dmav_w(offs_t offset, u8 data)
{
	m_dmav[offset] = data & 0x1f;
}

void tmp96c141_device::micro_dma(unsigned channel)
{
	u32 &src = m_dmas[channel].d;
	u32 &dst = m_dmad[channel].d;
	u8 const mode = m_dmam[channel].b.l & 0x0f;
	bool const word = BIT(mode, 0);
	unsigned const size = word ? 2 : 1;

	switch (mode >> 1)
	{
	case 0: // destination increment
	case 1: // destination decrement
	case 2: // source increment
	case 3: // source decrement
	case 4: // fixed addresses
		if (word)
			WRMEMW(dst, RDMEMW(src));
		else
			WRMEM(dst, RDMEM(src));
		switch (mode >> 1)
		{
		case 0: dst += size; break;
		case 1: dst -= size; break;
		case 2: src += size; break;
		case 3: src -= size; break;
		}
		m_cycles += 16;
		break;

	case 5:
		if (!word)
		{
			// read-only mode (DRAM refresh): four dummy reads
			for (int i = 0; i < 4; i++)
				RDMEM(src + i);
			src = (src & 0xffff0000) | ((src + 4) & 0xffff);
			m_cycles += 14;
		}
		else
		{
			// counter mode
			src++;
			m_cycles += 11;
		}
		break;

	default:
		logerror("micro DMA %u: undefined mode %02x\n", channel, m_dmam[channel].b.l);
		break;
	}

	m_dmac[channel].w.l--;
}

void tmp96c141_device::tlcs900_check_hdma()
{
	// micro DMA is started from the interrupt controller in tlcs900_check_irqs
}

void tmp96c141_device::tlcs900_check_irqs()
{
	u16 vector = 0;
	int level = 7;

	if (m_nmi_state == ASSERT_LINE)
	{
		m_nmi_state = CLEAR_LINE;
		vector = VECTOR_NMI;
	}
	else if (m_intwd)
	{
		m_intwd = false;
		vector = VECTOR_INTWD;
	}
	else
	{
		// highest level wins; within a level, the lowest vector
		int const iff = (m_sr.b.h >> 4) & 7;
		int best = -1;
		level = 0;
		for (unsigned i = 0; i < NUM_IRQS; i++)
		{
			u8 const bits = m_inte[s_irq[i].reg] >> s_irq[i].shift;
			int const lvl = bits & 7;
			if (BIT(bits, 3) && lvl != 0 && lvl != 7 && lvl >= iff && lvl > level)
			{
				best = i;
				level = lvl;
			}
		}

		if (best < 0)
			return;

		// accepting the request clears its flip-flop
		m_inte[s_irq[best].reg] &= ~(0x08 << s_irq[best].shift);
		vector = s_irq[best].vector;
		m_check_irqs = 1;   // look again for requests still pending

		// micro DMA: the lowest channel with a matching start vector
		for (unsigned ch = 0; ch < 4; ch++)
		{
			if (m_dmav[ch] && m_dmav[ch] == (vector >> 4))
			{
				micro_dma(ch);
				if (m_dmac[ch].w.l != 0)
					return;
				break;   // transfer count exhausted: general interrupt follows
			}
		}
	}

	tlcs900_intnest_accept();

	m_xssp.d -= 4;
	WRMEML(m_xssp.d, m_pc.d);
	m_xssp.d -= 2;
	WRMEMW(m_xssp.d, m_sr.w.l);

	m_sr.b.h = (m_sr.b.h & 0x8f) | (std::min(level + 1, 7) << 4) | 0x80;
	m_pc.d = 0x8000 + vector;
	m_cycles += 18;
	m_prefetch_clear = true;
	m_halted = 0;
}


//-------------------------------------------------
//  execute_set_input - called when a synchronized
//  input is changed
//-------------------------------------------------

void tmp96c141_device::execute_set_input(int inputnum, int state)
{
	bool const rise = m_level[inputnum < TLCS900_NUM_INPUTS ? inputnum : 0] == CLEAR_LINE && state != CLEAR_LINE;
	bool const fall = m_level[inputnum < TLCS900_NUM_INPUTS ? inputnum : 0] != CLEAR_LINE && state == CLEAR_LINE;

	switch (inputnum)
	{
	case INPUT_LINE_NMI:
	case TLCS900_NMI:
		if (rise || (fall && BIT(m_iimc, 0)))
			m_nmi_state = ASSERT_LINE;
		m_level[TLCS900_NMI] = state;
		break;

	case TLCS900_INT0:   // P87
		if (BIT(m_iimc, 2))
		{
			if (BIT(m_iimc, 1))
			{
				if (state != CLEAR_LINE)
					set_irq(IRQ_INT0);
				else
					m_inte[INTE0AD] &= ~0x08;
			}
			else if (rise)
				set_irq(IRQ_INT0);
		}
		m_level[TLCS900_INT0] = state;
		break;

	case TLCS900_INT4:   // P80/TI4
		if (!BIT(m_port_cr[8], 0))
		{
			u8 const capmode = (m_t4mod >> 3) & 3;
			if (capmode == 2 ? fall : rise)
				set_irq(IRQ_INT4);
			if (rise && (capmode == 1 || capmode == 2))
				capture(0);
			if (fall && capmode == 2)
				capture(1);
			if (rise && BIT(m_trun, 4) && (m_t4mod & 3) == 0)
				m_timer_change[4]++;
		}
		m_level[TLCS900_INT4] = state;
		break;

	case TLCS900_INT5:   // P81/TI5
		if (!BIT(m_port_cr[8], 1))
		{
			if (rise)
				set_irq(IRQ_INT5);
			if (rise && ((m_t4mod >> 3) & 3) == 1)
				capture(1);
		}
		m_level[TLCS900_INT5] = state;
		break;

	case TLCS900_INT6:   // P84/TI6
		if (!BIT(m_port_cr[8], 4))
		{
			u8 const capmode = (m_t5mod >> 3) & 3;
			if (capmode == 2 ? fall : rise)
				set_irq(IRQ_INT6);
			if (rise && (capmode == 1 || capmode == 2))
				capture(2);
			if (fall && capmode == 2)
				capture(3);
			if (rise && BIT(m_trun, 5) && (m_t5mod & 3) == 0)
				m_timer_change[5]++;
		}
		m_level[TLCS900_INT6] = state;
		break;

	case TLCS900_INT7:   // P85/TI7
		if (!BIT(m_port_cr[8], 5))
		{
			if (rise)
				set_irq(IRQ_INT7);
			if (rise && ((m_t5mod >> 3) & 3) == 1)
				capture(3);
		}
		m_level[TLCS900_INT7] = state;
		break;

	case TLCS900_TIO:    // P70/TI0
		if (rise && BIT(m_trun, 0) && (m_tmod & 3) == 0)
			m_timer_change[0]++;
		m_level[TLCS900_TIO] = state;
		break;
	}
	m_check_irqs = 1;
}

static std::pair<u16, char const *> const tmp96c141_syms[] = {
	{ 0x00, "P0" }, { 0x01, "P1" }, { 0x02, "P0CR" }, { 0x04, "P1CR" }, { 0x05, "P1FC" },
	{ 0x06, "P2" }, { 0x07, "P3" }, { 0x08, "P2CR" }, { 0x09, "P2FC" }, { 0x0a, "P3CR" }, { 0x0b, "P3FC" },
	{ 0x0c, "P4" }, { 0x0d, "P5" }, { 0x0e, "P4CR" }, { 0x10, "P4FC" },
	{ 0x12, "P6" }, { 0x13, "P7" }, { 0x14, "P6CR" }, { 0x15, "P7CR" }, { 0x16, "P6FC" }, { 0x17, "P7FC" },
	{ 0x18, "P8" }, { 0x19, "P9" }, { 0x1a, "P8CR" }, { 0x1b, "P9CR" }, { 0x1c, "P8FC" }, { 0x1d, "P9FC" },
	{ 0x20, "TRUN" },
	{ 0x22, "TREG0" }, { 0x23, "TREG1" },
	{ 0x24, "TMOD" }, { 0x25, "TFFCR" },
	{ 0x26, "TREG2" }, { 0x27, "TREG3" },
	{ 0x28, "P0MOD" }, { 0x29, "P1MOD" }, { 0x2a, "PFFCR" },
	{ 0x30, "TREG4L" }, { 0x31, "TREG4H" }, { 0x32, "TREG5L" }, { 0x33, "TREG5H" },
	{ 0x34, "CAP1L" }, { 0x35, "CAP1H" }, { 0x36, "CAP2L" }, { 0x37, "CAP2H" },
	{ 0x38, "T4MOD" }, { 0x39, "T4FFCR" }, { 0x3a, "T45CR" },
	{ 0x40, "TREG6L" }, { 0x41, "TREG6H" }, { 0x42, "TREG7L" }, { 0x43, "TREG7H" },
	{ 0x44, "CAP3L" }, { 0x45, "CAP3H" }, { 0x46, "CAP4L" }, { 0x47, "CAP4H" },
	{ 0x48, "T5MOD" }, { 0x49, "T5FFCR" },
	{ 0x4c, "PG0REG" }, { 0x4d, "PG1REG" }, { 0x4e, "PG01CR" },
	{ 0x50, "SC0BUF" }, { 0x51, "SC0CR" }, { 0x52, "SC0MOD" }, { 0x53, "BR0CR" },
	{ 0x54, "SC1BUF" }, { 0x55, "SC1CR" }, { 0x56, "SC1MOD" }, { 0x57, "BR1CR" },
	{ 0x58, "ODE" },
	{ 0x5c, "WDMOD" }, { 0x5d, "WDCR" },
	{ 0x5e, "ADMOD" },
	{ 0x60, "ADREG0L" }, { 0x61, "ADREG0H" }, { 0x62, "ADREG1L" }, { 0x63, "ADREG1H" },
	{ 0x64, "ADREG2L" }, { 0x65, "ADREG2H" }, { 0x66, "ADREG3L" }, { 0x67, "ADREG3H" },
	{ 0x68, "B0CS" }, { 0x69, "B1CS" }, { 0x6a, "B2CS" },
	{ 0x70, "INTE0AD" }, { 0x71, "INTE45" }, { 0x72, "INTE67" },
	{ 0x73, "INTET10" }, { 0x74, "INTEPW10" }, { 0x75, "INTET54" },
	{ 0x76, "INTET76" }, { 0x77, "INTES0" }, { 0x78, "INTES1" },
	{ 0x7b, "IIMC" },
	{ 0x7c, "DMA0V" }, { 0x7d, "DMA1V" }, { 0x7e, "DMA2V" }, { 0x7f, "DMA3V"}
};

static tlcs900_disassembler::cr_sym const tmp96c141_cr_syms[] = {
	{ 8,  0x22, "DMAM0" }, { 8,  0x26, "DMAM1" }, { 8,  0x2a, "DMAM2" }, { 8,  0x2e, "DMAM3" },
	{ 16, 0x20, "DMAC0" }, { 16, 0x24, "DMAC1" }, { 16, 0x28, "DMAC2" }, { 16, 0x2c, "DMAC3" },
	{ 32, 0x00, "DMAS0" }, { 32, 0x04, "DMAS1" }, { 32, 0x08, "DMAS2" }, { 32, 0x0c, "DMAS3" },
	{ 32, 0x10, "DMAD0" }, { 32, 0x14, "DMAD1" }, { 32, 0x18, "DMAD2" }, { 32, 0x1c, "DMAD3" },
};

std::unique_ptr<util::disasm_interface> tmp96c141_device::create_disassembler()
{
	return std::make_unique<tlcs900_disassembler>(tmp96c141_syms, tmp96c141_cr_syms);
}
