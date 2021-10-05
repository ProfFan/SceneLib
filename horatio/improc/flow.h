/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/flow.h */

#ifdef _HORATIO_IMAGE_
#ifndef HOR_TRANSPUTER

/*******************
*   typedef struct
*   {
*      Hor_Sub_Image *old_image;
*      Hor_Sub_Image *flow_flag;
*      Hor_Sub_Image *flow_c;
*      Hor_Sub_Image *flow_r;
*   } @Hor_Image_Flow;
*
*   Image flow result structure definition.
********************/
typedef struct
{
   Hor_Sub_Image *old_image, *flow_flag, *flow_c, *flow_r;
} Hor_Image_Flow;

Hor_Image_Flow *hor_image_flow ( Hor_Image     *imptr,
				 Hor_Sub_Image *old_image,
				 float     *gauss_mask, int gauss_size,
				 float      rc_gradient_threshold,
				 float      t_gradient_threshold,
				 int        c1, int r1,
				 int        c2, int r2 );

void hor_free_image_flow ( Hor_Image_Flow *result );

#endif /* !HOR_TRANSPUTER */
#endif /* _HORATIO_IMAGE_ */
