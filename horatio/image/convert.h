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
