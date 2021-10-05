#ifdef TRANSPUTER
#include <iocntrl.h>
#endif
#include "iff.h"

int write(int,char*,int);

int write_header(
int fd,			/* writw iff header to file on fd from *hp.  */
struct iffhdr *hp)	/* Return -1 on error, else 0.               */
{
short pad = 0;
int i;

    if (write(fd,(char *)hp,sizeof(struct iffhdr)) != sizeof(struct iffhdr))
	return -1;
    /* pad out the header block with zeros */
    for (i=0;i<hp->header_length-(sizeof(struct iffhdr)/2);i++)
	write(fd,(char *)&pad,2);
    return 0;
}
