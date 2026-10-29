// license:BSD-3-Clause
// copyright-holders:AJR
/****************************************************************************

    Toshiba TMP96C141 microcontroller

*****************************************************************************

       (AN0)PS0  73 ----------------+ +---------------- 72  P42(_CS2/_CAS2)
       (AN1)PS1  74 --------------+ | | +-------------- 71  P41(_CS1/_CAS1)
       (AN2)PS2  75 ------------+ | | | | +------------ 70  P40(_CS0/_CAS0)
       (AN3)PS3  76 ----------+ | | | | | | +---------- 69  P37(_RAS)
            VCC  77 --------+ | | | | | | | | +-------- 68  P36(R/_W)
           VREF  78 ------+ | | | | | | | | | | +------ 67  P35(_BUSAK)
           AGND  79 ----+ | | | | | | | | | | | | +---- 66  P34(_BUSRQ)
            VSS  80 --+ | | | | | | | | | | | | | | +-- 65  P33(_WAIT)
                     _|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_
      (PG00)P60   1 |   _                             | 64  P32(_HWR)
      (PG01)P61   2 |  (_)                            | 63  P31(_WR)
      (PG02)P62   3 |                                 | 62  P30(_RD)
      (PG03)P63   4 |                                 | 61  P27(A7/A23)
      (PG10)P64   5 |                                 | 60  P26(A6/A22)
      (PG11)P65   6 |                                 | 59  P25(A5/A21)
      (PG12)P66   7 |                                 | 58  P24(A4/A20)
      (PG13)P67   8 |                                 | 57  P23(A3/A19)
       (TI0)P70   9 |                                 | 56  P22(A2/A18)
       (TO1)P71  10 |                                 | 55  P21(A1/A17)
       (TO2)P72  11 |                                 | 54  P20(A0/A16)
       (TO3)P73  12 |           TMP96C141AF           | 53  VSS
  (INT4/TI4)P80  13 |             (QFP80)             | 52  P17(AD15/A15)
  (INT5/TI5)P81  14 |                                 | 51  P16(AD14/A14)
       (TO4)P82  15 |                                 | 50  P15(AD13/A13)
       (TO5)P83  16 |                                 | 49  P14(AD12/A12)
  (INT6/TI6)P84  17 |                                 | 48  P13(AD11/A11)
  (INT7/TI7)P85  18 |                                 | 47  P12(AD10/A10)
       (TO6)P86  19 |                                 | 46  P11(AD9/A9)
      (INT0)P87  20 |                                 | 45  P10(AD8/A8)
           _NMI  21 |                                 | 44  P07(AD7)
        _WDTOUT  22 |                                 | 43  P06(AD6)
         _RESET  23 |                                 | 42  P05(AD5)
            CLK  24 |_________________________________| 41  P04(AD4)
                      | | | | | | | | | | | | | | | |
            VSS  25 --+ | | | | | | | | | | | | | | +-- 40  P03(AD3)
             X1  26 ----+ | | | | | | | | | | | | +---- 39  P02(AD2)
             X2  27 ------+ | | | | | | | | | | +------ 38  P01(AD1)
            _EA  28 --------+ | | | | | | | | +-------- 37  P00(AD0)
      (TXD0)P90  29 ----------+ | | | | | | +---------- 36  VCC
      (RXD0)P91  30 ------------+ | | | | +------------ 35  ALE
     (_CTS0)P92  31 --------------+ | | +-------------- 34  P95(SCLK1)
      (TXD1)P93  32 ----------------+ +---------------- 33  P94(RXD1)

****************************************************************************/

#ifndef MAME_CPU_TLCS900_TMP96C141_H
#define MAME_CPU_TLCS900_TMP96C141_H

#pragma once

#include "tlcs900.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> tmp96c141_device

class tmp96c141_device : public tlcs900_device
{
public:
	// device type constructor
	tmp96c141_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// configuration helpers
	// Port N reads return the pin levels; writes pass the output pins as
	// mem_mask, with the timer and pattern generator outputs merged in.
	template <unsigned N> auto port_read() { return m_port_read[N].bind(); }
	template <unsigned N> auto port_write() { return m_port_write[N].bind(); }
	template <unsigned N> auto an_read() { return m_an_read[N].bind(); }   // 10-bit result
	template <unsigned N> auto txd() { return m_txd_cb[N].bind(); }       // whole bytes
	template <unsigned N> void rx_byte(u8 data);

protected:
	// device-level overrides
	virtual void device_config_complete() override;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_execute_interface overrides: one state is two oscillator clocks
	virtual u64 execute_clocks_to_cycles(u64 clocks) const noexcept override { return (clocks + 1) / 2; }
	virtual u64 execute_cycles_to_clocks(u64 cycles) const noexcept override { return cycles * 2; }
	virtual void execute_set_input(int inputnum, int state) override;

	// tlcs900_device overrides
	virtual void tlcs900_check_hdma() override;
	virtual void tlcs900_check_irqs() override;
	virtual void tlcs900_handle_ad() override;
	virtual void tlcs900_handle_timers() override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

private:
	static constexpr unsigned NUM_PORTS = 10;

	void internal_mem(address_map &map) ATTR_COLD;

	// ports
	template <unsigned N> u8 port_r();
	template <unsigned N> void port_w(u8 data);
	template <unsigned N> void port_cr_w(u8 data);
	template <unsigned N> void port_fc_w(u8 data);
	u8 port_function_out(unsigned port) const;
	void update_port(unsigned port);

	// 8-bit timers 0-3
	u8 trun_r();
	void trun_w(u8 data);
	void treg01_w(offs_t offset, u8 data);
	void tmod_w(u8 data);
	u8 tffcr_r();
	void tffcr_w(u8 data);
	u8 treg23_r(offs_t offset);
	void treg23_w(offs_t offset, u8 data);
	u8 pwmod_r(offs_t offset);
	void pwmod_w(offs_t offset, u8 data);
	u8 pffcr_r();
	void pffcr_w(u8 data);

	// 16-bit timers 4 and 5
	void treg45_w(offs_t offset, u8 data);
	u8 cap12_r(offs_t offset);
	u8 t4mod_r();
	void t4mod_w(u8 data);
	u8 t4ffcr_r();
	void t4ffcr_w(u8 data);
	u8 t45cr_r();
	void t45cr_w(u8 data);
	void treg67_w(offs_t offset, u8 data);
	u8 cap34_r(offs_t offset);
	u8 t5mod_r();
	void t5mod_w(u8 data);
	u8 t5ffcr_r();
	void t5ffcr_w(u8 data);

	// pattern generator
	u8 pgreg_r(offs_t offset);
	void pgreg_w(offs_t offset, u8 data);
	u8 pg01cr_r();
	void pg01cr_w(u8 data);

	// serial channels
	u8 scbuf_r(offs_t offset);
	void scbuf_w(offs_t offset, u8 data);
	u8 sccr_r(offs_t offset);
	void sccr_w(offs_t offset, u8 data);
	u8 scmod_r(offs_t offset);
	void scmod_w(offs_t offset, u8 data);
	u8 brcr_r(offs_t offset);
	void brcr_w(offs_t offset, u8 data);
	u8 ode_r();
	void ode_w(u8 data);

	// watchdog timer
	u8 wdmod_r();
	void wdmod_w(u8 data);
	void wdcr_w(u8 data);

	// A/D converter
	u8 admod_r();
	void admod_w(u8 data);
	u8 adreg_r(offs_t offset);

	// chip select/wait controller
	void bcs_w(offs_t offset, u8 data);

	// interrupt controller
	u8 inte_r(offs_t offset);
	void inte_w(offs_t offset, u8 data);
	void iimc_w(u8 data);
	void dmav_w(offs_t offset, u8 data);

	// helpers
	void set_irq(unsigned irq);
	void micro_dma(unsigned channel);
	void tick_timer01(unsigned timer, u32 ticks);
	void tick_timer23(unsigned timer, u32 ticks);
	void tick_timer45(unsigned timer, u32 ticks);
	void set_tff1(int state);
	void set_tff(unsigned which, int state);
	void tff1_trigger();
	void pg_shift(unsigned ch);
	void capture(unsigned reg);

	// callbacks
	devcb_read8::array<NUM_PORTS> m_port_read;
	devcb_write8::array<NUM_PORTS> m_port_write;
	devcb_read16::array<4> m_an_read;
	devcb_write8::array<2> m_txd_cb;

	// I/O ports
	u8 m_port_latch[NUM_PORTS];
	u8 m_port_cr[NUM_PORTS];
	u8 m_port_fc[NUM_PORTS];

	// 8-bit timers
	u8 m_trun;
	u8 m_treg[4];
	u8 m_treg_buf[4];
	u8 m_tmod;
	u8 m_tffcr;
	u8 m_pwmod[2];
	u8 m_pffcr;
	u8 m_tff;      // bit 1 = TFF1, 2 = TFF2, 3 = TFF3, 4 = TFF4, 5 = TFF5, 6 = TFF6

	// 16-bit timers
	u16 m_uc16[2];
	u16 m_treg16[4];
	u16 m_treg16_buf[4];
	u16 m_cap[4];
	u8 m_t4mod;
	u8 m_t4ffcr;
	u8 m_t5mod;
	u8 m_t5ffcr;
	u8 m_t45cr;

	// pattern generator
	u8 m_pgreg[2];
	u8 m_pg01cr;

	// serial channels
	u8 m_scbuf_rx[2];
	u8 m_sccr[2];
	u8 m_scmod[2];
	u8 m_brcr[2];
	u8 m_ode;
	s32 m_tx_cycles[2];
	u8 m_tx_data[2];

	// watchdog timer
	u8 m_wdmod;
	bool m_wd_enabled;
	u32 m_wd_counter;
	bool m_intwd;

	// A/D converter
	u8 m_admod;
	u16 m_adreg[4];
	u8 m_ad_channel;

	// chip select/wait controller
	u8 m_bcs[3];

	// interrupt controller
	u8 m_inte[9];
	u8 m_iimc;
	u8 m_dmav[4];
};

// device type declaration
DECLARE_DEVICE_TYPE(TMP96C141, tmp96c141_device)

#endif // MAME_CPU_TLCS900_TMP96C141_H
