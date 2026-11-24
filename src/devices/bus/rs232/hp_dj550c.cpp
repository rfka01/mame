// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 550C on an RS-232 port

    The printer itself is machine/hp_dj550c.cpp.  Set the computer's
    baud rate and parity with the printer's configuration switches B5-B8
    (9600 baud, 8 data bits, no parity by default).  The printer answers
    with XON/XOFF and with DTR (pin 20), which goes to the computer as
    DSR and CTS.

**********************************************************************/

#include "emu.h"
#include "hp_dj550c.h"

#include "machine/hp_dj550c.h"


namespace {

class rs232_hp_dj550c_device : public device_t, public device_rs232_port_interface
{
public:
	rs232_hp_dj550c_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, RS232_HP_DJ550C, tag, owner, clock),
		device_rs232_port_interface(mconfig, *this),
		m_printer(*this, "printer")
	{
	}

protected:
	// device_rs232_port_interface implementation: data from the computer
	// arrives on the printer's RxD
	virtual void input_txd(int state) override { m_printer->rxd_w(state); }

	// device_t implementation
	virtual void device_start() override ATTR_COLD { }

	virtual void device_reset() override ATTR_COLD
	{
		output_dcd(0);
	}

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD
	{
		HP_DJ550C(config, m_printer);
		m_printer->txd_cb().set(FUNC(rs232_hp_dj550c_device::output_rxd));
		m_printer->dtr_cb().set(FUNC(rs232_hp_dj550c_device::output_dsr));
		m_printer->dtr_cb().append(FUNC(rs232_hp_dj550c_device::output_cts));
	}

private:
	required_device<hp_dj550c_device> m_printer;
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(RS232_HP_DJ550C, device_rs232_port_interface, rs232_hp_dj550c_device, "rs232_hpdj550c", "HP DeskJet 550C (RS-232)")
