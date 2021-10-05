/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/segment.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   int dummy;
} Hor_Image_Segment;

Hor_Image_Segment *hor_image_segment ( Hor_Image *image,
				       int    c1, int r1, int c2, int r2,
				       int    patch_size, int patch_density,
				       float  low_threshold,
				       float  threshold_step,
				       float  high_threshold,
				       float  sd_scale,
				       u_long thres_colour );

#endif /* _HORATIO_IMAGE_ */
