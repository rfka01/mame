// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 550C on a Centronics port

    The printer itself is machine/hp_dj550c.cpp.  As in the user's guide
    (appendix, Centronics parallel interface): /STROBE latches a byte,
    BUSY is high while the printer cannot take data (a byte waiting, a
    full buffer, off line or an error), /ACK pulses when a byte has been
    taken, PERROR is high without paper, SELECT (Ready) high while on
    line, /FAULT (Error) low in an error state, and a low pulse on /INIT
    (Reset/Input Clean) resets the printer and clears its input buffer.

**********************************************************************/

#include "emu.h"
#include "hp_dj550c.h"

#include "machine/hp_dj550c.h"


namespace {

class centronics_hp_dj550c_device : public device_t, public device_centronics_peripheral_interface
{
public:
	centronics_hp_dj550c_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, CENTRONICS_HP_DJ550C, tag, owner, clock),
		device_centronics_peripheral_interface(mconfig, *this),
		m_printer(*this, "printer"),
		m_data(0)
	{
	}

protected:
	// device_centronics_peripheral_interface implementation
	virtual void input_strobe(int state) override { m_printer->strobe_w(state); }
	virtual void input_init(int state) override { m_printer->init_w(state); }
	virtual void input_data0(int state) override { data_bit(0, state); }
	virtual void input_data1(int state) override { data_bit(1, state); }
	virtual void input_data2(int state) override { data_bit(2, state); }
	virtual void input_data3(int state) override { data_bit(3, state); }
	virtual void input_data4(int state) override { data_bit(4, state); }
	virtual void input_data5(int state) override { data_bit(5, state); }
	virtual void input_data6(int state) override { data_bit(6, state); }
	virtual void input_data7(int state) override { data_bit(7, state); }
	virtual bool supports_pin35_5v() override { return true; }

	// device_t implementation
	virtual void device_start() override ATTR_COLD
	{
		save_item(NAME(m_data));
	}

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD
	{
		HP_DJ550C(config, m_printer);
		m_printer->busy_cb().set(FUNC(centronics_hp_dj550c_device::output_busy));
		m_printer->ack_cb().set(FUNC(centronics_hp_dj550c_device::output_ack));
		m_printer->perror_cb().set(FUNC(centronics_hp_dj550c_device::output_perror));
		m_printer->fault_cb().set(FUNC(centronics_hp_dj550c_device::output_fault));
		m_printer->select_cb().set(FUNC(centronics_hp_dj550c_device::output_select));
	}

private:
	required_device<hp_dj550c_device> m_printer;
	u8 m_data;

	void data_bit(int bit, int state)
	{
		if (state)
			m_data |= 1 << bit;
		else
			m_data &= ~(1 << bit);
		m_printer->data_w(m_data);
	}
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(CENTRONICS_HP_DJ550C, device_centronics_peripheral_interface, centronics_hp_dj550c_device, "centronics_hpdj550c", "HP DeskJet 550C (Centronics)")
