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
