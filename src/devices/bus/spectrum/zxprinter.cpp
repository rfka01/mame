// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX Printer and Alphacom 32 for the ZX Spectrum

    Port FB (A2 low; the Alphacom 32 also needs A7 high, see
    https://www.robertprice.co.uk/robblog/tag/alphacom-32/). The printer
    has no through connector and ends the expansion chain.

**********************************************************************/

#include "emu.h"
#include "zxprinter.h"

#include "machine/zxprinter.h"


namespace {

class spectrum_zxprinter_device : public device_t, public device_spectrum_expansion_interface
{
public:
	spectrum_zxprinter_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		spectrum_zxprinter_device(mconfig, SPECTRUM_ZXPRINTER, tag, owner, clock)
	{
	}

protected:
	spectrum_zxprinter_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
		device_t(mconfig, type, tag, owner, clock),
		device_spectrum_expansion_interface(mconfig, *this),
		m_printer(*this, "printer")
	{
	}

	virtual void device_start() override ATTR_COLD { }
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD { ZX_PRINTER(config, m_printer); }

	virtual uint8_t iorq_r(offs_t offset) override
	{
		// the printer answers port FB on its own, as in Fuse
		if (selected(offset))
			return m_printer->read();
		return device_spectrum_expansion_interface::iorq_r(offset);
	}

	virtual void iorq_w(offs_t offset, uint8_t data) override
	{
		if (selected(offset))
			m_printer->write(data);
	}

	virtual bool selected(offs_t offset) const { return !BIT(offset, 2); }

	required_device<zx_printer_device> m_printer;
};


class spectrum_alphacom32_device : public spectrum_zxprinter_device
{
public:
	spectrum_alphacom32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		spectrum_zxprinter_device(mconfig, SPECTRUM_ALPHACOM32, tag, owner, clock)
	{
	}

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD { ALPHACOM32(config, m_printer); }
	virtual bool selected(offs_t offset) const override { return !BIT(offset, 2) && BIT(offset, 7); }
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SPECTRUM_ZXPRINTER, device_spectrum_expansion_interface, spectrum_zxprinter_device, "spectrum_zxprinter", "Sinclair ZX Printer (ZX Spectrum)")
DEFINE_DEVICE_TYPE_PRIVATE(SPECTRUM_ALPHACOM32, device_spectrum_expansion_interface, spectrum_alphacom32_device, "spectrum_alphacom32", "Alphacom 32 (ZX Spectrum)")
