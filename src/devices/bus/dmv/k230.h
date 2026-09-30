// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
// thanks-to:rfka01
#ifndef MAME_BUS_DMV_K230_H
#define MAME_BUS_DMV_K230_H

#pragma once

#include "dmvbus.h"
#include "cpu/i86/i86.h"
#include "machine/i8087.h"

// K234
#include "cpu/m68000/m68008.h"

// K235
#include "cpu/nec/nec.h"
#include "machine/pic8259.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> dmv_k230_device

class dmv_k230_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k230_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	dmv_k230_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// dmvcart_interface overrides
	virtual void hold_w(int state) override;
	virtual void switch16_w(int state) override;
	virtual bool av16bit() override;

	void k230_io(address_map &map) ATTR_COLD;
	void k230_mem(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	optional_device<i8087_device> m_fpu;
	optional_ioport             m_config;
	optional_memory_region      m_rom;
	int                         m_switch16;
	int                         m_hold;

	uint8_t io_r(offs_t offset);
	uint8_t program_r(offs_t offset);
	void io_w(offs_t offset, uint8_t data);
	void program_w(offs_t offset, uint8_t data);

	// 8087 socket (empty on K230/K231 by default, fitted on K235)
	void fpu_config(machine_config &config);
	bool fpu_present() { return m_fpu && m_config && (m_config->read() & 0x80); }
	void esc_opcode_w(uint32_t data) { if (fpu_present()) m_fpu->insn_w(data); }
	void esc_data_w(uint32_t data)   { if (fpu_present()) m_fpu->addr_w(data); }

private:
	uint8_t rom_r(offs_t offset);
};


// ======================> dmv_k231_device

class dmv_k231_device :
		public dmv_k230_device
{
public:
	// construction/destruction
	dmv_k231_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};


// ======================> dmv_k234_device

class dmv_k234_device :
		public dmv_k230_device
{
public:
	// construction/destruction
	dmv_k234_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD { return nullptr; }

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// dmvcart_interface overrides
	virtual void hold_w(int state) override;
	virtual void switch16_w(int state) override;

private:
	int                         m_snr;

	uint8_t snr_r();
	void snr_w(uint8_t data);

	void k234_mem(address_map &map) ATTR_COLD;
};


// ======================> dmv_k235_device

class dmv_k235_device :
		public dmv_k230_device
{
public:
	// construction/destruction
	dmv_k235_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	void timint_w(int state) override  { m_pic->ir0_w(state); }
	void keyint_w(int state) override  { m_pic->ir1_w(state); }
	void busint_w(int state) override  { m_pic->ir2_w(state); }
	void flexint_w(int state) override { m_pic->ir6_w(state); }
	void irq2a_w(int state) override   { if (!(m_config->read() & 0x02))  m_pic->ir5_w(state); }
	void irq2_w(int state) override    { if ( (m_config->read() & 0x02))  m_pic->ir5_w(state); }
	void irq3_w(int state) override    { m_pic->ir3_w(state); }
	void irq4_w(int state) override    { m_pic->ir4_w(state); }
	void irq5_w(int state) override    { if (!(m_config->read() & 0x01))  m_pic->ir7_w(state); }
	void irq6_w(int state) override    { if ( (m_config->read() & 0x01))  m_pic->ir7_w(state); }

private:
	required_device<pic8259_device> m_pic;

	void k235_io(address_map &map) ATTR_COLD;
};

// device type definition
DECLARE_DEVICE_TYPE(DMV_K230, dmv_k230_device)
DECLARE_DEVICE_TYPE(DMV_K231, dmv_k231_device)
DECLARE_DEVICE_TYPE(DMV_K234, dmv_k234_device)
DECLARE_DEVICE_TYPE(DMV_K235, dmv_k235_device)

#endif // MAME_BUS_DMV_K230_H
