#include <stdio.h>
#include <string.h>
#ifndef TRANSPUTER
#include <sys/time.h>           /* Modified SCT 22-3-88 for new name */
#endif
#include "iff.h"		/* Modified JWD 18-12-87 for standard lib. */

void make_iff_hdr(
     struct iffhdr *hp,	/* make an iff header in *hp. */
     short it,
     short size )	/* The image type is set from 'it'.
			   The height & width are set from size.
			   Date, time, etc. are filled in.
			   Other fields set to UNKNOWN */
{

  hp->header_length = 256;
  hp->image_type    = it;
  hp->height        = size;
  hp->width         = size;
  hp->my_signed     = 0;
  hp->fov_height    = UNKNOWN;
  hp->fov_width     = UNKNOWN;
  hp->stereo        = 0;
  hp->baseline      = UNKNOWN;
  hp->vergence      = UNKNOWN;
  hp->gaze          = UNKNOWN;
  hp->source_id     = UNKNOWN; /* who allocates these? */
  hp->processed     = 0;
  hp->stop          = UNKNOWN;
  hp->focus         = UNKNOWN;
  hp->magic         = IFF_MAGIC;
  (void) strcpy(hp->title, "No title");
}
