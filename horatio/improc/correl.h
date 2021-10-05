/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/correl.h */

#ifdef _HORATIO_IMAGE_

/*******************
*   typedef struct
*   {
*      Hor_Sub_Image *image;
*   } @Hor_Correlation;
*
*   Correlation result structure definition.
********************/
typedef struct
{
   Hor_Sub_Image *image;
} Hor_Correlation;

Hor_Correlation *hor_correlation (Hor_Image *image, Hor_Sub_Image *old_image,
				  int    c1, int r1, int c2, int r2,
				  int    patch_size, int patch_density,
				  int    max_motion,
				  float  display_scale, u_long display_colour);

#endif /* _HORATIO_IMAGE_ */
