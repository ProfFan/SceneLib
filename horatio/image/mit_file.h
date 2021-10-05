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
