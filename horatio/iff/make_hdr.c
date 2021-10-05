#include "iff.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>

#include <string.h>

int make_header(
struct iffhdr *hp,	/* make an iff header in *hp. */
short it,short size)	/* The image type is set from 'it'.
			   The height & width are set from size.
			   Date, time, etc. are filled in.
			   Other fields set to UNKNOWN */

{
struct timeval ttime;
struct tm *tt;
char times[10],dates[10];

    hp->header_length = 256;
    hp->image_type    = it;
    hp->height        = size;
    hp->width         = size;
    hp->my_signed     = 0; /* PFM 21-11-92 */
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
    strcpy(hp->title,"No title");
    gettimeofday(&ttime,NULL);
    tt = localtime(&ttime.tv_sec);
    sprintf(times,"%02d:%02d:%02d",tt->tm_hour,tt->tm_min,tt->tm_sec);
    sprintf(dates,"%02d/%02d/%02d",tt->tm_mday,tt->tm_mon+1,tt->tm_year);
/* Fixed to give months 1-12, not 0-11. 25 June 1985 C.R.Brown */
    strncpy(hp->time,times,8);
    strncpy(hp->date,dates,8);
}
