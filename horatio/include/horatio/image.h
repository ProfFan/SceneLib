/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_IMAGE_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/image.h */

typedef enum { HOR_BIT, HOR_U_CHAR, HOR_CHAR, HOR_U_SHORT, HOR_SHORT,
	       HOR_U_INT, HOR_INT, HOR_FLOAT, HOR_RGB_UC, HOR_RGB_UI,
	       HOR_POINTER }
   Hor_Image_Type;

typedef struct
{
   u_char r, g, b;
} Hor_RGB_UC;

typedef struct
{
   u_int r, g, b;
} Hor_RGB_UI;

typedef union
{
   hor_bit    b;   /* for bitmap image */
   u_char     uc;  /* for unsigned character image */
   char       c;   /* for (signed) character image */
   u_short    us;  /* for unsigned short image */
   short      s;   /* for (signed) short image */
   u_int      ui;  /* for unsigned int image */
   int        i;   /* for (signed) int image */
   float      f;   /* for floating point image */
   Hor_RGB_UC cuc; /* for colour unsigned character image */
   Hor_RGB_UI cui; /* for colour unsigned integer image */
   void      *p;   /* for pointer image */
} Hor_Impixel;

typedef union
{
   hor_bit    **b;
   u_char     **uc;
   char       **c;
   u_short    **us;
   short      **s;
   u_int      **ui;
   int        **i;
   float      **f;
   Hor_RGB_UC **cuc;
   Hor_RGB_UI **cui;
   void      ***p;
} Hor_Imdata;

typedef struct
{
   int            width, height;            /* dimensions of image */
   Hor_Image_Type type;                     /* pixel type */
   Hor_Imdata     array;                    /* pointer to image data */
   void         (*pixel_free_func)(void *); /* function to free data associated
					       with a pixel (pointer image
					       only) */
} Hor_Image;

typedef struct
{
   int       c0, r0; /* offsets from origin (top-left corner) of main image */
   Hor_Image image;
} Hor_Sub_Image;

/*******************
*   typedef struct
*   {
*      int c0,    r0;     (offsets of image window)
*      int width, height; (dimensions of image window)
*   } @Hor_Image_Window;
*
*   Rectangular image window definition.
********************/
typedef struct
{
   int c0, r0, width, height;
} Hor_Image_Window;
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/alloc.h */

Hor_Bool       hor_alloc_image_data ( int width, int height,
				      Hor_Image_Type type,
				      Hor_Imdata *image_data );
Hor_Image     *hor_alloc_image ( int width, int height,
				 Hor_Image_Type type,
				 void (*pixel_free_func)(void *) );
Hor_Image     *hor_alloc_image_header ( int width, int height, 
				        Hor_Image_Type type, void **data );
Hor_Sub_Image *hor_alloc_sub_image ( int c0, int r0, int width, int height,
				     Hor_Image_Type type,
				     void (*pixel_free_func)(void *) );

void hor_free_image_data   ( Hor_Imdata image_data, int size,
			     Hor_Image_Type type,
			     void (*pixel_free_func)(void *) );
void hor_free_image        ( Hor_Image     *imptr );
void hor_free_image_header ( Hor_Image     *imptr );
void hor_free_images       ( Hor_Image     *imptr1, ... );
void hor_free_sub_image    ( Hor_Sub_Image *imptr );
void hor_free_sub_images   ( Hor_Sub_Image *imptr1, ... );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/misc.h */

/*******************
*   Hor_Bool @hor_type_image     ( Hor_Image     *image, Hor_Image_Type type)
*   Hor_Bool @hor_type_sub_image ( Hor_Sub_Image *image, Hor_Image_Type type)
*
*   Return HOR_TRUE if (sub-)image is of given type, HOR_FALSE otherwise.
*   Both implemented as macros.
********************/
#define hor_type_image(imptr,imtype) ((imptr)->type == (imtype))
#define hor_type_sub_image(subimptr,imtype) ((subimptr)->image.type==(imtype))

Hor_Sub_Image *hor_extract_from_image     ( Hor_Image *image,
					    int c0,    int r0,
					    int width, int height );
Hor_Sub_Image *hor_extract_from_sub_image ( Hor_Sub_Image *sub_image,
					    int ext_c0,    int ext_r0,
					    int ext_width, int ext_height );
void hor_insert_image_in_image (Hor_Image *source_image, Hor_Image *dest_image,
				int c_offset, int r_offset );
#define hor_insert_sub_image_in_image(sim,im) hor_insert_image_in_image(&(sim)->image,im,(sim)->c0,(sim)->r0)

void hor_add_constant_to_image        ( Hor_Image *, Hor_Impixel );
void hor_subtract_constant_from_image ( Hor_Image *, Hor_Impixel );
void hor_multiply_image_by_constant   ( Hor_Image *, Hor_Impixel );
void hor_divide_image_by_constant     ( Hor_Image *, Hor_Impixel );

Hor_Image     *hor_add_images ( Hor_Image *, Hor_Image * );
Hor_Sub_Image *hor_add_sub_images            (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_subtract_images           (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_subtract_sub_images       (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_subtract_signed_images    (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_subtract_signed_sub_images(Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_multiply_images           (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_multiply_sub_images       (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_multiply_double_images    (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_multiply_double_sub_images(Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_divide_images             (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_divide_sub_images         (Hor_Sub_Image *,Hor_Sub_Image *);

#define hor_add_constant_to_sub_image(sim,pix) (hor_add_constant_to_image(&((sim)->image),pix))
#define hor_subtract_constant_from_sub_image(sim,pix) (hor_subtract_constant_from_image(&((sim)->image),pix))
#define hor_multiply_sub_image_by_constant(sim,pix) (hor_multiply_image_by_constant(&((sim)->image),pix))
#define hor_divide_sub_image_by_constant(sim,pix) (hor_divide_image_by_constant(&((sim)->image),pix))

void           hor_fill_image_with_constant (Hor_Image *,     Hor_Impixel    );
Hor_Image     *hor_average_images           (Hor_Image *,     Hor_Image *    );
Hor_Sub_Image *hor_average_sub_images       (Hor_Sub_Image *, Hor_Sub_Image *);

#define hor_fill_sub_image_with_constant(sim,pix) (hor_fill_image_with_constant(&((sim)->image),pix))
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/im_comp.h */

Hor_Bool hor_same_type_images          ( Hor_Image *,     ... );
Hor_Bool hor_same_type_sub_images      ( Hor_Sub_Image *, ... );
Hor_Bool hor_same_dims_images          ( Hor_Image *,     ... );
Hor_Bool hor_same_dims_sub_images      ( Hor_Sub_Image *, ... );
Hor_Bool hor_same_type_dims_images     ( Hor_Image *,     ... );
Hor_Bool hor_same_type_dims_sub_images ( Hor_Sub_Image *, ... );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/convert.h */

void           hor_convert_image_data ( Hor_Image *image, Hor_Image *result );
Hor_Image     *hor_convert_image      ( Hor_Image     *imptr,
				        Hor_Image_Type type );
Hor_Sub_Image *hor_convert_sub_image  ( Hor_Sub_Image *sub_imptr,
				        Hor_Image_Type type );

#define hor_copy_image(imptr) (hor_convert_image(imptr,(imptr)->type))
#define hor_copy_sub_image(imptr) (hor_convert_sub_image(imptr,(imptr)->image.type))
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/subsamp.h */

Hor_Image *hor_subsample ( Hor_Image *image, int ratio );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/image_io.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

void       HorChanInImageHeader       (Channel *channel, Hor_Image     *image);
void       HorChanInSubImageHeader    (Channel *channel, Hor_Sub_Image *image);
void       HorChanInImageData         (Channel *channel, Hor_Image     *image);
Hor_Image     *HorChanInImage         (Channel *channel);
Hor_Sub_Image *HorChanInSubImage      (Channel *channel);
void       HorChanInAllocatedImage    (Channel *channel, Hor_Image     *image);
void       HorChanInAllocatedSubImage (Channel *channel, Hor_Sub_Image *image);
void       HorChanOutImageHeader      (Channel *channel, Hor_Image     *image);
void       HorChanOutSubImageHeader   (Channel *channel, Hor_Sub_Image *image);
void       HorChanOutImageData        (Channel *channel, Hor_Image     *image);
void       HorChanOutImage            (Channel *channel, Hor_Image     *image);
void       HorChanOutSubImage         (Channel *channel, Hor_Sub_Image *image);

#define HorChanInSubImageData( channel, im ) \
           HorChanInImageData ( channel, &(im)->image )
#define HorChanOutSubImageData( channel, im ) \
           HorChanOutImageData ( channel, &(im)->image )

#endif
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/im_file.h */

#ifndef HOR_REDUCED_LIB

/*******************
*   typedef enum { @HOR_IFF_FORMAT, @HOR_MIT_FORMAT, @HOR_PGM_FORMAT,
*                  @HOR_PPM_FORMAT, @HOR_JPEG_FORMAT, @HOR_GIF_FORMAT,
*                  @HOR_PS_FORMAT}
*      @Hor_Image_Format;
*
*   Allowable image file formats.
********************/
typedef enum { HOR_IFF_FORMAT, HOR_MIT_FORMAT
#ifndef HOR_TRANSPUTER
	     , HOR_PGM_FORMAT, HOR_PPM_FORMAT, HOR_JPEG_FORMAT, HOR_GIF_FORMAT,
	       HOR_PS_FORMAT
#endif
	     }
   Hor_Image_Format;

Hor_Image *hor_read_image_stream ( int fd );
#ifndef HOR_TRANSPUTER
Hor_Compress_Type hor_read_image_compress ( int fd);
Hor_Image_Format hor_read_image_format ( int fd,
					 Hor_Compress_Type read_compress_type);
#endif /* HOR_TRANSPUTER */
Hor_Image *hor_read_image ( const char *base_name );
Hor_Image *hor_read_named_image ( const char *file_name,
				  Hor_Image_Format format );

void hor_set_image_format ( Hor_Image_Format );
#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
void hor_set_image_compress ( Hor_Compress_Type new_compress_type );
Hor_Compress_Type hor_get_image_compress(void);
#endif
#endif
Hor_Bool   hor_write_image_stream ( int         fd,        Hor_Image *, ... );
Hor_Bool   hor_write_image        ( const char *base_name, Hor_Image *, ... );
Hor_Bool   hor_write_named_image  ( const char *file_name, Hor_Image *, ... );

#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/iff_file.h */

#ifndef HOR_REDUCED_LIB

Hor_Image *hor_read_iff_image_stream  ( int fd );
Hor_Image *hor_read_iff_image         ( const char *file_name );
Hor_Bool   hor_write_iff_image_stream ( int fd, Hor_Image *, ... );
Hor_Bool   hor_write_iff_image        ( const char *file_name,
				        Hor_Image *, ... );

#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/mit_file.h */

#ifndef HOR_REDUCED_LIB

Hor_Image *hor_read_mit_image_stream  ( int fd );
Hor_Image *hor_read_mit_image         ( const char *file_name );
Hor_Bool   hor_write_mit_image_stream ( int fd, Hor_Image *imptr, ...);
Hor_Bool   hor_write_mit_image        ( const char *file_name,
				        Hor_Image *imptr, ...);

#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/pgm_file.h */

#ifndef HOR_TRANSPUTER

Hor_Image *hor_read_pgm_image_stream  ( int fd );
Hor_Image *hor_read_pgm_image         ( const char *file_name );
Hor_Bool   hor_write_pgm_image_stream ( int fd, Hor_Image *imptr, ... );
Hor_Bool   hor_write_pgm_image        ( const char *file_name,
				        Hor_Image *imptr, ... );

#endif
/* Copyright 1993 Charles S Wiles (csw@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ppm_file.h */

#ifndef HOR_REDUCED_LIB
#ifdef HOR_PROVIDE_RGB

Hor_Image *hor_read_ppm_image_stream  ( int fd );
Hor_Image *hor_read_ppm_image         ( const char *file_name );
Hor_Bool   hor_write_ppm_image_stream ( int fd, Hor_Image *imptr, ... );
Hor_Bool   hor_write_ppm_image        ( const char *file_name,
				        Hor_Image *imptr, ... );

#endif /* HOR_PROVIDE_RGB */
#endif
/* Copyright 1993 Charles S Wiles (csw@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ppm_file.h */

#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER
#ifndef HOR_REDUCED_LIB
#ifdef HOR_PROVIDE_RGB

Hor_Image *hor_read_jpeg_image_stream  ( int fd );
Hor_Image *hor_read_jpeg_image         ( const char *file_name );
Hor_Bool   hor_write_jpeg_image_stream ( int fd, Hor_Image *imptr, ... );
Hor_Bool   hor_write_jpeg_image        ( const char *file_name,
				        Hor_Image *imptr, ... );
Hor_Bool   hor_set_jpeg_quality ( int loc_quality );
#endif /* HOR_PROVIDE_RGB */
#endif
#endif
#endif
/* Copyright 1993 Charles S Wiles (csw@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ppm_file.h */

#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER
#ifndef HOR_REDUCED_LIB
#ifdef HOR_PROVIDE_RGB

Hor_Image *hor_read_gif_image_stream  ( int fd );
Hor_Image *hor_read_gif_image         ( const char *file_name );
Hor_Bool   hor_write_gif_image_stream ( int fd, Hor_Image *imptr, ... );
Hor_Bool   hor_write_gif_image        ( const char *file_name,
				        Hor_Image *imptr, ... );
Hor_Bool hor_set_gif_ncolours (int loc_ncolors);

#endif /* HOR_PROVIDE_RGB */
#endif
#endif
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/sequence.h */

#ifndef HOR_REDUCED_LIB

/*******************
*   typedef struct hor_count_string
*   {
*      char string[10];
*   } @Hor_Count_String;
*
*   Definition of structure containing image sequence number string.
********************/
typedef struct hor_count_string
{
   char string[10];
} Hor_Count_String;

Hor_Count_String hor_count_string ( int count );
Hor_Image *hor_read_next_image  ( const char *root_name, int count,
				  Hor_Error_Type error_type );
void       hor_write_next_image ( const char *root_name, int count,
				  Hor_Image *image );

#ifdef _HOR_LIST_

Hor_List hor_read_image_sequence ( char *root_name, int start, int finish,
				   int *last_image );
void hor_write_image_sequence (Hor_List sequence, char *root_name, int start);
void hor_free_image_sequence (Hor_List sequence);

#endif /* _HOR_LIST_ */
#endif /* HOR_REDUCED_LIB */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/stream.h */

#ifndef HOR_REDUCED_LIB

void           hor_read_image_data_from_stream ( int fd, Hor_Image     *image);
Hor_Image     *hor_read_image_from_stream      ( int fd );
Hor_Sub_Image *hor_read_sub_image_from_stream  ( int fd );
void           hor_write_image_data_to_stream  ( int fd, Hor_Image     *image);
void           hor_write_image_to_stream       ( int fd, Hor_Image     *image);
void           hor_write_sub_image_to_stream   ( int fd,
						 Hor_Sub_Image *sub_image);

#define hor_read_sub_image_data_from_stream(fd,sim) \
           hor_read_image_data_from_stream(fd,&(sim)->image)
#define hor_write_sub_image_data_to_stream(fd,sim) \
           hor_write_image_data_to_stream(fd,&(sim)->image)

#endif
/* Copyright 1993 Ian Reid (ian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ps_image.h */

#if (!defined(Linux) && defined(FILE) || \
      defined(Linux) && defined(_STDIO_H))

#ifndef PS_IMAGE
#define PS_IMAGE

FILE *hor_ps_open(char *filename);
Hor_Bool hor_ps_close(FILE *fp);

Hor_Bool hor_ps_write_image(Hor_Image *im, FILE *fp);
Hor_Bool hor_write_ps_image_stream(int fd, Hor_Image *im, ...);
Hor_Bool hor_write_ps_image(const char *file_name, Hor_Image *im, ...);
Hor_Bool hor_ps_init(int width, int height, FILE *fp);

Hor_Bool hor_ps_setlinewidth(double linewidth, FILE *fp);
Hor_Bool hor_ps_setgray(double greyval, FILE *fp);

Hor_Bool hor_ps_line(double x1, double y1, double x2, double y2, FILE *fp);
Hor_Bool hor_ps_circle (double c, double r, double radius, FILE *fp);
Hor_Bool hor_ps_ellipse (double c, double r,
			 double axis1, double axis2, double angle, 
			 FILE *fp);

#endif
#endif
