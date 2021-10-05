#include <stdio.h>
#include "mit.h"

int fwr_mit_hdr(
     FILE *outfile,		/* write mit header to outfile from *hp.  */
     struct mithdr *hp)		/* Return -1 on error, else 0.            */
{
   unsigned char out[8], *pout;
   unsigned int i;
   unsigned short key[4], *ps;

   key[0] = hp->type;
   key[1] = hp->bits_per_pixel;
   key[2] = hp->width;
   key[3] = hp->height;

   /* byte reverse for output */

   for ( i = 0, pout = out, ps = key; i < 4; i++ ) 
   { 
       *pout++ = *ps & 0377;
       *pout++ = *ps++ >> 8;
   }

   if (fwrite((char*)out, (int)8, (int)1, outfile) != 1)
      return -1;

    return 0;
}
