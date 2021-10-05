/* Based on source from ~gjb/iff/read_header.c, modified to use stream
   pointers rather than file handles.  Will Dickson, 17-12-87.
   Modified for HORATIO shared Sun/transputer version P McLauchlan 22/11/92 */

#include <stdio.h>
#include "iff.h"

void swab(char*,char*,int);
char *malloc(unsigned int);
int free(char*);
void bcopy(char*,char*,int);

int frd_iff_hdr(
     FILE *infile,	  /* read iff header from infile into *hp. */
     struct iffhdr *hp )  /* Return -1 on error, else 0.           */
			  /* Byte reversal problems are handled.   */
{
  int i;
  short pad;
  struct iffhdr *chp;		/* copy of header */

  if (fread((char*)hp, sizeof(struct iffhdr), (int)1, infile) != 1)
    return -1;
  if (hp->magic == IFF_AMIGC) /* header needs byte swapping */
    { chp = (struct iffhdr *) malloc(sizeof(struct iffhdr));
      bcopy((char*)hp, (char*)chp, sizeof(struct iffhdr));
      swab((char*)chp, (char*)hp, sizeof(struct iffhdr));
      free((char*)chp); }
  if (hp->magic != IFF_MAGIC)
    return -1;		/* not a valid header */
     /* soak up the padding in the header block */
  for (i=0; i<(hp->header_length - sizeof(struct iffhdr)/2); i++)
    (void) fread((char*)&pad, sizeof(short), (int)1, infile);
  return 0;
}
