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
