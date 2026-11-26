// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    HP DeskJet 540 on a Centronics port

    The printer itself is machine/hp_dj540.cpp.  The firmware model drives
    BUSY and pulses /ACK when a byte has been taken; SELECT, /FAULT and
    PERROR stay at on line and no error, and /INIT is not connected.

**********************************************************************/

#include "emu.h"
#include "hp_dj540.h"

#include "machine/hp_dj540.h"


namespace {

class centronics_hp_dj540_device : public device_t, public device_centronics_peripheral_interface
{
public:
	centronics_hp_dj540_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
		device_t(mconfig, CENTRONICS_HP_DJ540, tag, owner, clock),
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
		HP_DJ540(config, m_printer);
		m_printer->busy_cb().set(FUNC(centronics_hp_dj540_device::output_busy));
		m_printer->ack_cb().set(FUNC(centronics_hp_dj540_device::output_ack));
		m_printer->perror_cb().set(FUNC(centronics_hp_dj540_device::output_perror));
		m_printer->fault_cb().set(FUNC(centronics_hp_dj540_device::output_fault));
		m_printer->select_cb().set(FUNC(centronics_hp_dj540_device::output_select));
	}

private:
	required_device<hp_dj540_device> m_printer;
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


DEFINE_DEVICE_TYPE_PRIVATE(CENTRONICS_HP_DJ540, device_centronics_peripheral_interface, centronics_hp_dj540_device, "centronics_hpdj540", "HP DeskJet 540 (Centronics)")
