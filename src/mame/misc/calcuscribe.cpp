// license:BSD-3-Clause
// copyright-holders:rfka01
/***************************************************************************

    CalcuScribe Duo - stand-alone system

    The hardware is emulated by the ADB keyboard device in
    src/devices/bus/adb/calcuscribe.cpp, so the same machine can be plugged
    into an emulated Macintosh ("-adb:0 calcuscribe").  This system puts it
    on an ADB bus without a host to use it as a word processor on its own.

    ROM: calcuscribe_m27c801.bin, either in calcuscribe.zip or cscribed.zip.

***************************************************************************/

#include "emu.h"

#include "bus/adb/adb.h"
#include "bus/adb/cards.h"


namespace {

class cscribe_state : public driver_device
{
public:
	cscribe_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
	{
	}

	void cscribed(machine_config &config) ATTR_COLD;
};


void cscribe_state::cscribed(machine_config &config)
{
	ADB_BUS(config, "adb");
	ADB_CONNECTOR(config, "adb:0", adb_devices, "calcuscribe", true);
}


ROM_START(cscribed)
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS          INIT        COMPANY             FULLNAME           FLAGS
COMP( 2000, cscribed, 0,      0,      cscribed, 0,     cscribe_state, empty_init, "CalcuScribe, Inc.", "CalcuScribe Duo", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
