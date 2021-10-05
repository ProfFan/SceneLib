#ifdef TRANSPUTER
#include <iocntrl.h>
#endif
#include "iff.h"

void swab(char*,char*,int);
int read(int,char*,int);

int read_header(
int fd,			/* read iff header from file on fd into *hp. */
struct iffhdr *hp )	/* Return -1 on error, else 0.               */
			/* Byte reversal problems are handled.       */
{
short pad;
int i;

    if (read(fd,(char *)hp,sizeof(struct iffhdr)) != sizeof(struct iffhdr))
	return -1;
#ifndef TRANSPUTER
    if (hp->magic == IFF_AMIGC) /* header needs byte swapping */
	swab((char*)hp,(char*)hp,sizeof(struct iffhdr));
#endif
    if (hp->magic != IFF_MAGIC)
	return -1;		/* not a valid header */
    /* soak up the padding in the header block */
    for (i=0;i<hp->header_length-(sizeof(struct iffhdr)/2);i++)
	read(fd,(char *)&pad,2);
    return 0;
}
