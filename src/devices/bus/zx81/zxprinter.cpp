// license:BSD-3-Clause
// copyright-holders:rfka01
/**********************************************************************

    Sinclair ZX Printer and Alphacom 32 for the ZX81

    Port FB (A2 low; the Alphacom 32 also needs A7 high, see
    https://www.robertprice.co.uk/robblog/tag/alphacom-32/). The printer
    passes the rest of the bus on to its through connector, which is not
    emulated.

**********************************************************************/

#include "emu.h"
#include "zxprinter.h"

#include "machine/zxprinter.h"


namespace {

class zx81_zxprinter_device : public device_t, public device_zx81_expansion_interface
{
public:
	zx81_zxprinter_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		zx81_zxprinter_device(mconfig, ZX81_ZXPRINTER, tag, owner, clock)
	{
	}

protected:
	zx81_zxprinter_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
		device_t(mconfig, type, tag, owner, clock),
		device_zx81_expansion_interface(mconfig, *this),
		m_printer(*this, "printer")
	{
	}

	virtual void device_start() override ATTR_COLD { }
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD { ZX_PRINTER(config, m_printer); }

	virtual uint8_t iorq_r(offs_t offset) override
	{
		return selected(offset) ? m_printer->read() : 0xff;
	}

	virtual void iorq_w(offs_t offset, uint8_t data) override
	{
		if (selected(offset))
			m_printer->write(data);
	}

	virtual bool selected(offs_t offset) const { return !BIT(offset, 2); }

	required_device<zx_printer_device> m_printer;
};


class zx81_alphacom32_device : public zx81_zxprinter_device
{
public:
	zx81_alphacom32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		zx81_zxprinter_device(mconfig, ZX81_ALPHACOM32, tag, owner, clock)
	{
	}

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD { ALPHACOM32(config, m_printer); }
	virtual bool selected(offs_t offset) const override { return !BIT(offset, 2) && BIT(offset, 7); }
};

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(ZX81_ZXPRINTER, device_zx81_expansion_interface, zx81_zxprinter_device, "zx81_zxprinter", "Sinclair ZX Printer (ZX81)")
DEFINE_DEVICE_TYPE_PRIVATE(ZX81_ALPHACOM32, device_zx81_expansion_interface, zx81_alphacom32_device, "zx81_alphacom32", "Alphacom 32 (ZX81)")
