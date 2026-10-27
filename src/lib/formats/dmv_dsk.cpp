// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/*********************************************************************

    formats/dmv_dsk.cpp

    NCR Decision Mate V format

*********************************************************************/

#include "formats/dmv_dsk.h"

dmv_format::dmv_format() : upd765_format(formats)
{
}

const char *dmv_format::name() const noexcept
{
	return "dmv";
}

const char *dmv_format::description() const noexcept
{
	return "Decision Mate V disk image";
}

const char *dmv_format::extensions() const noexcept
{
	return "img";
}

/*
    40-track (48 tpi) disks in an 80-track (96 tpi) drive

    The image tracks of a floppy_image are counted in head steps of the
    drive.  upd765_format::load() puts image track n at step n, which is
    right for a 40-track drive.  In a QD drive (525qd, 84 steps) a disk
    written at 48 tpi has its track n under the head at step 2n, so a
    real DMV has to double-step it: the mainboard ROM checks this before
    booting (READ ID at physical cylinder 2, result in 0:FE08), and the
    MS-DOS boot sector and BIOS switch to double stepping.  Without this
    the emulated QD drive read a 40-track disk with single steps, which
    real hardware cannot do.

    So: 40-track formats loaded into a drive with 80 or more steps are
    spread to the even steps; the odd steps stay unformatted.  Saving
    gathers them back.
*/
static bool dmv_spread_48tpi(const floppy_image &image, int track_count)
{
	int img_tracks, img_heads;
	image.get_maximal_geometry(img_tracks, img_heads);
	return track_count <= 42 && img_tracks >= 80;
}

bool dmv_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	if (!upd765_format::load(io, form_factor, variants, image))
		return false;

	int tracks, heads;
	image.get_actual_geometry(tracks, heads);
	if (!dmv_spread_48tpi(image, tracks))
		return true;

	for (int track = tracks - 1; track > 0; track--)
		for (int head = 0; head < heads; head++)
		{
			image.get_buffer(track * 2, head).swap(image.get_buffer(track, head));
			image.get_buffer(track, head).clear();
		}
	return true;
}

bool dmv_format::save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const
{
	int img_tracks, img_heads;
	image.get_maximal_geometry(img_tracks, img_heads);

	// spread image: track 1 empty, track 2 formatted, nothing beyond step 83
	bool spread = img_tracks >= 80 && !image.track_is_formatted(1, 0) && image.track_is_formatted(2, 0);
	if (!spread)
		return upd765_format::save(io, variants, image);

	floppy_image packed(img_tracks / 2, img_heads, image.get_form_factor());
	packed.set_form_variant(image.get_form_factor(), image.get_variant());
	for (int track = 0; track < img_tracks / 2; track++)
		for (int head = 0; head < img_heads; head++)
			packed.get_buffer(track, head) = image.get_buffer(track * 2, head);
	return upd765_format::save(io, variants, packed);
}

// gap size from hardware reference manual
const dmv_format::format dmv_format::formats[] = {
	{
		floppy_image::FF_525, floppy_image::DSDD, floppy_image::MFM,
		2000,
		9, 40, 2,
		512, {},
		1, {},
		80, 50, 22, 80
	},
	{
		floppy_image::FF_525, floppy_image::DSDD, floppy_image::MFM,
		2000,
		8, 40, 2,
		512, {},
		1, {},
		80, 50, 22, 80
	},
	{
		/*  720K: 80 tracks, 9 sectors of 512 bytes (DMV with QD drives) */
		floppy_image::FF_525, floppy_image::DSQD, floppy_image::MFM,
		2000,
		9, 80, 2,
		512, {},
		1, {},
		80, 50, 22, 80
	},
	{
		/*  800K: 80 tracks, 5 sectors of 1024 bytes (DMV QD format) */
		floppy_image::FF_525, floppy_image::DSQD, floppy_image::MFM,
		2000,
		5, 80, 2,
		1024, {},
		1, {},
		80, 50, 22, 44
	},
	{}
};

const dmv_format FLOPPY_DMV_FORMAT;
