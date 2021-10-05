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
