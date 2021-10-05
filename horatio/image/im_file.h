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
