// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    Olivetti Quaderno - combined parallel/floppy connector

    The Quaderno has no built-in floppy drive.  The floppy disk controller
    is inside the machine; the drive (EFD 1035, 3.5" 1.44 MB/720 KB, own
    9 V supply) plugs into the same mini connector as the printer cable
    of the "serial and parallel cable kit".  Only one of them can be
    connected at a time.

    Options:
        printer   printer cable: parallel port with a Centronics socket
        efd1035   Olivetti EFD 1035 external floppy disk drive

    The BIOS writes a copy of power management register 14h three times
    to port 3f3h before it uses the drive (bit 6 is set before the
    controller is reset); this is probably the switch between printer and
    floppy signals on the connector.  It is only logged here.

***************************************************************************/

#include "emu.h"
#include "quaderno_ext.h"

#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(QUADERNO_EXT, quaderno_ext_connector_device, "quaderno_ext", "Olivetti Quaderno parallel/floppy connector")
DEFINE_DEVICE_TYPE(QUADERNO_FDC, quaderno_fdc_device, "quaderno_fdc", "Olivetti Quaderno floppy disk controller")


/***************************************************************************
    Connector
***************************************************************************/

quaderno_ext_connector_device::quaderno_ext_connector_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUADERNO_EXT, tag, owner, clock)
	, device_single_card_slot_interface<device_quaderno_ext_interface>(mconfig, *this)
	, m_card(nullptr)
{
}

void quaderno_ext_connector_device::device_start()
{
	m_card = get_card_device();
}

floppy_image_device *quaderno_ext_connector_device::floppy() const
{
	return m_card ? m_card->floppy() : nullptr;
}

uint8_t quaderno_ext_connector_device::lpt_r(offs_t offset)
{
	return m_card ? m_card->lpt_r(offset) : 0xff;
}

void quaderno_ext_connector_device::lpt_w(offs_t offset, uint8_t data)
{
	if (m_card)
		m_card->lpt_w(offset, data);
}

device_quaderno_ext_interface::device_quaderno_ext_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "quadernoext")
{
}


namespace {

/***************************************************************************
    Printer cable
***************************************************************************/

class quaderno_printer_cable_device : public device_t, public device_quaderno_ext_interface
{
public:
	quaderno_printer_cable_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual bool has_printer_port() override { return true; }
	virtual uint8_t lpt_r(offs_t offset) override { return m_lpt->read(offset); }
	virtual void lpt_w(offs_t offset, uint8_t data) override { m_lpt->write(offset, data); }

protected:
	virtual void device_start() override { }
	virtual void device_add_mconfig(machine_config &config) override
	{
		PC_LPT(config, m_lpt);
	}

private:
	required_device<pc_lpt_device> m_lpt;
};


/***************************************************************************
    EFD 1035 floppy disk drive
***************************************************************************/

class quaderno_efd1035_device : public device_t, public device_quaderno_ext_interface
{
public:
	quaderno_efd1035_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual floppy_image_device *floppy() override { return m_drive->get_device(); }

protected:
	virtual void device_start() override { }
	virtual void device_add_mconfig(machine_config &config) override;

private:
	static void floppies(device_slot_interface &device)
	{
		device.option_add("35hd", FLOPPY_35_HD);
	}

	static void formats(format_registration &fr)
	{
		fr.add_pc_formats();
	}

	required_device<floppy_connector> m_drive;
};

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(QUADERNO_EXT_PRINTER, device_quaderno_ext_interface, quaderno_printer_cable_device, "quaderno_printer", "Olivetti Quaderno printer cable")
DEFINE_DEVICE_TYPE_PRIVATE(QUADERNO_EXT_EFD1035, device_quaderno_ext_interface, quaderno_efd1035_device, "quaderno_efd1035", "Olivetti EFD 1035 external floppy disk drive")

namespace {

quaderno_printer_cable_device::quaderno_printer_cable_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUADERNO_EXT_PRINTER, tag, owner, clock)
	, device_quaderno_ext_interface(mconfig, *this)
	, m_lpt(*this, "lpt")
{
}

quaderno_efd1035_device::quaderno_efd1035_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUADERNO_EXT_EFD1035, tag, owner, clock)
	, device_quaderno_ext_interface(mconfig, *this)
	, m_drive(*this, "0")
{
}

void quaderno_efd1035_device::device_add_mconfig(machine_config &config)
{
	FLOPPY_CONNECTOR(config, m_drive, floppies, "35hd", formats, true).enable_sound(true);
}

} // anonymous namespace

void quaderno_ext_devices(device_slot_interface &device)
{
	device.option_add("printer", QUADERNO_EXT_PRINTER);
	device.option_add("efd1035", QUADERNO_EXT_EFD1035);
}


/***************************************************************************
    Floppy disk controller

    3f2  digital output register (drive 0 only)
    3f3  power management mirror (write only, see above)
    3f4  main status register      3f5  data
    3f7  read: disk change, write: data rate
    DMA channel 2, IRQ 6
***************************************************************************/

quaderno_fdc_device::quaderno_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QUADERNO_FDC, tag, owner, clock)
	, device_isa8_card_interface(mconfig, *this)
	, m_fdc(*this, "fdc")
	, m_ext(nullptr)
	, m_dor(0)
	, m_fdc_irq(false)
	, m_fdc_drq(false)
	, m_irq(false)
	, m_drq(false)
{
}

void quaderno_fdc_device::device_add_mconfig(machine_config &config)
{
	UPD765A(config, m_fdc, 8'000'000, false, false);
	m_fdc->intrq_wr_callback().set(FUNC(quaderno_fdc_device::fdc_irq_w));
	m_fdc->drq_wr_callback().set(FUNC(quaderno_fdc_device::fdc_drq_w));
}

void quaderno_fdc_device::device_start()
{
	set_isa_device();
	m_isa->set_dma_channel(2, this, true);
	m_isa->install_device(0x03f0, 0x03f7, *this, &quaderno_fdc_device::map);

	save_item(NAME(m_dor));
	save_item(NAME(m_fdc_irq));
	save_item(NAME(m_fdc_drq));
	save_item(NAME(m_irq));
	save_item(NAME(m_drq));
}

void quaderno_fdc_device::device_reset()
{
	dor_w(0x00);
}

void quaderno_fdc_device::map(address_map &map)
{
	map(0x2, 0x2).rw(FUNC(quaderno_fdc_device::dor_r), FUNC(quaderno_fdc_device::dor_w));
	map(0x3, 0x3).w(FUNC(quaderno_fdc_device::pm_mirror_w));
	map(0x4, 0x5).m(m_fdc, FUNC(upd765a_device::map));
	map(0x7, 0x7).rw(FUNC(quaderno_fdc_device::dir_r), FUNC(quaderno_fdc_device::ccr_w));
}

floppy_image_device *quaderno_fdc_device::drive() const
{
	return m_ext ? m_ext->floppy() : nullptr;
}

uint8_t quaderno_fdc_device::dor_r()
{
	return m_dor;
}

void quaderno_fdc_device::dor_w(uint8_t data)
{
	LOG("dor = %02x\n", data);
	m_dor = data;

	floppy_image_device *const floppy = drive();
	if (floppy)
		floppy->mon_w(!BIT(data, 4));
	m_fdc->set_floppy(((data & 3) == 0 && BIT(data, 4)) ? floppy : nullptr);

	update_lines();
	m_fdc->reset_w(!BIT(data, 2));
}

void quaderno_fdc_device::pm_mirror_w(uint8_t data)
{
	LOG("3f3 <- %02x\n", data);
}

uint8_t quaderno_fdc_device::dir_r()
{
	floppy_image_device *const floppy = drive();
	if (floppy && !(m_dor & 3))
		return floppy->dskchg_r() ? 0x00 : 0x80;
	return 0x00;
}

void quaderno_fdc_device::ccr_w(uint8_t data)
{
	static constexpr int rates[4] = { 500000, 300000, 250000, 1000000 };
	m_fdc->set_rate(rates[data & 3]);
}

void quaderno_fdc_device::fdc_irq_w(int state)
{
	m_fdc_irq = state;
	update_lines();
}

void quaderno_fdc_device::fdc_drq_w(int state)
{
	m_fdc_drq = state;
	update_lines();
}

void quaderno_fdc_device::update_lines()
{
	const bool enabled = BIT(m_dor, 2) && BIT(m_dor, 3);
	const bool irq = m_fdc_irq && enabled;
	const bool drq = m_fdc_drq && enabled;
	if (irq != m_irq)
	{
		m_irq = irq;
		m_isa->irq6_w(irq ? ASSERT_LINE : CLEAR_LINE);
	}
	if (drq != m_drq)
	{
		m_drq = drq;
		m_isa->drq2_w(drq ? ASSERT_LINE : CLEAR_LINE);
	}
}

uint8_t quaderno_fdc_device::dack_r(int line)
{
	return m_fdc->dma_r();
}

void quaderno_fdc_device::dack_w(int line, uint8_t data)
{
	m_fdc->dma_w(data);
}

void quaderno_fdc_device::eop_w(int state)
{
	m_fdc->tc_w(state == ASSERT_LINE);
}
