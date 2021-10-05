#include <stdio.h>
#include "mit.h"

int frd_mit_hdr(
     FILE *infile,		/* read mit header from infile into *hp. */
     struct mithdr *hp)		/* Return -1 on error, else 0.           */
{
  unsigned char in[8], *pin;
  unsigned int i, store, store1;
  unsigned short key[4], *ps;

  if (fread((char*)in, (int)8, (int)1, infile) != 1)
    return -1;

   /* reverse bytes for short */
  for ( i = 0, pin = in, ps = key; i < 4; i++ ) 
    { 
      store = *pin++;
      store1 = *pin++;
      *ps++ = ( store1 << 8 ) | store;
    }

  hp->type = key[0];
  hp->bits_per_pixel = key[1];
  hp->width = key[2];
  hp->height = key[3];
  return 0;
}
