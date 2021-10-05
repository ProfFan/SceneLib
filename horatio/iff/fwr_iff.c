/* Based on source from ~gjb/iff/write_header.c, modified to use stream
   pointers rather than file handles.  Will Dickson, 18-12-87.          */

#include "iff.h"
#include <stdio.h>

int fwr_iff_hdr(
     FILE *outfile,		/* write iff header to outfile from *hp.  */
     struct iffhdr *hp )	/* Return -1 on error, else 0.            */
{
  short pad = 0;
  int i;

  if (fwrite((char*)hp, sizeof(struct iffhdr), (int)1, outfile) != 1)
    return -1;
				/* pad out the header block with zeros */
  for (i=0; i<hp->header_length-(sizeof(struct iffhdr)/2); i++)
    (void) fwrite((char*)&pad, sizeof(short), (int)1, outfile);
  return 0;
}
