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
