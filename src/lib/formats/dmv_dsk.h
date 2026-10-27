// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/*********************************************************************

    formats/dmv_dsk.h

    NCR Decision Mate V format

*********************************************************************/
#ifndef MAME_FORMATS_DMV_DSK_H
#define MAME_FORMATS_DMV_DSK_H

#pragma once

#include "upd765_dsk.h"

class dmv_format : public upd765_format
{
public:
	dmv_format();

	virtual const char *name() const noexcept override;
	virtual const char *description() const noexcept override;
	virtual const char *extensions() const noexcept override;

	virtual bool load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const override;
	virtual bool save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const override;

private:
	static const format formats[];
};

extern const dmv_format FLOPPY_DMV_FORMAT;

#endif // MAME_FORMATS_DMV_DSK_H
