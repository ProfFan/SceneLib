#ifdef TRANSPUTER
#include <iocntrl.h>
#endif
#include <stdio.h>
#include "mit.h"

int write(int,char*,int);

int write_mit_header(
int fd,			/* write mit header to file on fd from *hp.  */
struct mithdr *hp)	/* Return -1 on error, else 0.               */
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

   if (write(fd, (char *)out, 8) != 8 )
      return -1;

    return 0;
}
