// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Olivetti Quaderno - combined parallel/floppy connector and the
    internal floppy disk controller

***************************************************************************/

#ifndef MAME_OLIVETTI_QUADERNO_EXT_H
#define MAME_OLIVETTI_QUADERNO_EXT_H

#pragma once

#include "bus/isa/isa.h"
#include "imagedev/floppy.h"
#include "machine/pc_lpt.h"
#include "machine/upd765.h"


class device_quaderno_ext_interface;

// connector on the left side: printer cable or EFD 1035 floppy drive

class quaderno_ext_connector_device : public device_t, public device_single_card_slot_interface<device_quaderno_ext_interface>
{
public:
	template <typename T>
	quaderno_ext_connector_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: quaderno_ext_connector_device(mconfig, tag, owner, 0)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	quaderno_ext_connector_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	floppy_image_device *floppy() const;
	uint8_t lpt_r(offs_t offset);
	void lpt_w(offs_t offset, uint8_t data);

protected:
	virtual void device_start() override ATTR_COLD;

private:
	device_quaderno_ext_interface *m_card;
};

class device_quaderno_ext_interface : public device_interface
{
public:
	virtual floppy_image_device *floppy() { return nullptr; }
	virtual bool has_printer_port() { return false; }
	virtual uint8_t lpt_r(offs_t offset) { return 0xff; }
	virtual void lpt_w(offs_t offset, uint8_t data) { }

protected:
	device_quaderno_ext_interface(const machine_config &mconfig, device_t &device);
};

void quaderno_ext_devices(device_slot_interface &device);


// internal floppy disk controller (AT style register set at 3f0-3f7)

class quaderno_fdc_device : public device_t, public device_isa8_card_interface
{
public:
	quaderno_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void set_connector(quaderno_ext_connector_device *ext) { m_ext = ext; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual uint8_t dack_r(int line) override;
	virtual void dack_w(int line, uint8_t data) override;
	virtual void eop_w(int state) override;

private:
	void map(address_map &map) ATTR_COLD;

	uint8_t dor_r();
	void dor_w(uint8_t data);
	void pm_mirror_w(uint8_t data);
	uint8_t dir_r();
	void ccr_w(uint8_t data);
	void fdc_irq_w(int state);
	void fdc_drq_w(int state);
	void update_lines();
	floppy_image_device *drive() const;

	required_device<upd765a_device> m_fdc;
	quaderno_ext_connector_device *m_ext;

	uint8_t m_dor;
	bool m_fdc_irq;
	bool m_fdc_drq;
	bool m_irq;
	bool m_drq;
};

DECLARE_DEVICE_TYPE(QUADERNO_EXT, quaderno_ext_connector_device)
DECLARE_DEVICE_TYPE(QUADERNO_FDC, quaderno_fdc_device)

#endif // MAME_OLIVETTI_QUADERNO_EXT_H
