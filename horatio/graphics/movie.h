/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/movie.h */

Hor_List hor_display_make_movie ( const char *root_name, ... );
Hor_List hor_make_movie_from_images ( Hor_List image_sequence );
#ifdef _XtIntrinsic_h
#ifdef _HORATIO_IMAGE_
XImage *hor_display_make_movie_image ( Hor_Image *image, ... );
#endif /* _HORATIO_IMAGE_ */
void hor_display_show_movie_image ( XImage *ximage );
#endif /* _XtIntrinsic_h */
void hor_display_destroy_movie ( Hor_List movie );

