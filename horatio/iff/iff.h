/* Image File Format for Alvey Vision Consortium 2
   From 2nd. draft 12.9.84 by Andrew Blake
   This version 15.4.85 by Chris Brown 
   EDGE type definition added 10.7.85. Chris Brown
   IFF_MAGIC forced to type "short" 10-10-88 to overcome bug AZ/CB
   non-standard EDGEL def removed 18/11/88 az -- replaced by
   EDGE_LIST 18/1/89 Simon Turner
   signed changed to my_signed to avoid compilation problems PFM 7/8/92
*/

struct iffhdr
  { short header_length;	/* length of header in words */
    short image_type;		/* BYTE, WORD, or BOOLEAN */
    short height;		/* image height in pixels */
    short width;		/* image width  in pixels */
    short my_signed;            /* 1 => signed data, 0=> unsigned */
    short fov_height;		/* field of view height (mrad) */
    short fov_width;		/* field of view width  (mrad) */
    short stereo;		/* 1 => stereo pair, 0 => mono */
    short baseline;		/* optical centres separation, (mm) */
    short vergence;		/* vergence angle (mrad) */
    short gaze;			/* gaze angle (mrad) */
    short source_id;		/* i.d. of camera etc. */
    short processed;		/* 1 => processed since generation */
    char  date[8];		/* date of generation dd/mm/yy */
    char  time[8];		/* time of generation hh:mm:ss */
    short stop;			/* lens stop (f no. x 100) */
    short focus;		/* lens focal length (mm) */
				/* next two added by Chris Brown */
    short magic;		/* magic no. to detect byte reversal */
    char title[80];		/* null-terminated text title */
  };

#define IFF_MAGIC (short)0x8516	/* magic number */
#define IFF_AMIGC (short)0x1685	/* byte swapped version */
#define UNKNOWN 32767		/* flags unknown entries */
#define BYTE 0			/* image types */
#define WORD 1
#define BOOLEAN 2
#define EDGE 3
#define EDGE_LIST 999  /* as output by canny programs -- used to be EDGEL */
#define OTHER 1000
#define EDGE_HOR 0200		/* Edge direction codes */
#define EDGE_VER 0100

/* Site identifiers    -     these go in the source_id field.
   "Thousands"  digit  identifies  the site.   Zero indicates
   universal - i.e., image generators which are common across
   sites. Bottom three digits are site-specific.    Nobody is
   expected to take any notice of these.
*/

#define SITE_SHEFF	1000	/* Sheffield AI Vision Unit */
#define SITE_EDCS	2000	/* Edinburgh Comp. Sci. (Blake) */
#define SITE_EDAI	3000	/* Edinburgh AI Dept */
#define SITE_QMC	4000	/* Queen Mary College (Buxton) */
#define WINSOM	100

int make_header(struct iffhdr *hp,short it,short size);
void make_iff_hdr ( struct iffhdr *hp, short it, short size);
int read_header(int fd,struct iffhdr *hp );
int write_header(int fd,struct iffhdr *hp);

/* Sugar */

#ifndef TRANSPUTER
#define fread_iff_hdr frd_iff_hdr
#define fwrite_iff_hdr fwr_iff_hdr

int frd_iff_hdr();
int fwr_iff_hdr();
#endif /* TRANSPUTER */
