/* MIT image header
   Andrew Zisserman 22/9/87
   Alterations by pcl & simon, 10 June 1988
   EDGE type added az & simon, 10 October 1988
*/

struct mithdr {
  unsigned short type;
  unsigned short bits_per_pixel;
  unsigned short width;
  unsigned short height;
  };

/* Type codes */

#define MIT_UNSIGNED	1
#define MIT_RGB		2
#define MIT_HSB		3
#define MIT_CAP		4
#define MIT_SIGNED	5
#define MIT_FLOAT	6
#define MIT_EDGE        7

#define MIT_UCOMPLEX	0x101
#define MIT_SCOMPLEX	0x105
#define MIT_FCOMPLEX	0x106

#define MIT_UNSIGNED_E	0x201
#define MIT_SIGNED_E	0x205
#define MIT_FLOAT_E	0x206

#define MIT_UCOMPLEX_E	0x301
#define MIT_SCOMPLEX_E	0x305
#define MIT_FCOMPLEX_E	0x306

#define EDGE_HOR 0200		/* Edge direction codes */
#define EDGE_VER 0100

int read_mit_header(int fd,struct mithdr *hp);
int write_mit_header(int fd,struct mithdr *hp);

#ifndef TRANSPUTER
int frd_mit_hdr(FILE *infile,struct mithdr *hp);
int fwr_mit_hdr(FILE *outfile,struct mithdr *hp);
#endif
