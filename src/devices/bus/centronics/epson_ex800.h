// license:BSD-3-Clause
// copyright-holders:Dirk Best, rfka01
/**********************************************************************

    Epson EX-800 dot matrix printer emulation

    The printer has a parallel and a serial interface built in; it can
    be connected to either.

**********************************************************************/

#ifndef MAME_BUS_CENTRONICS_EPSON_EX800_H
#define MAME_BUS_CENTRONICS_EPSON_EX800_H

#pragma once

#include "ctronics.h"
#include "bus/rs232/rs232.h"

// device type declaration: one device for a parallel or a serial port
DECLARE_DEVICE_TYPE(EPSON_EX800, device_t)

#endif // MAME_BUS_CENTRONICS_EPSON_EX800_H
