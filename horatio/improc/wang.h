/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/wang.h */

#ifdef _HORATIO_IMAGE_
Hor_Corner_Map *hor_wang_corners ( Hor_Image *imptr,
				   float  edge_thresh,
				   float  corner_thresh,
				   int    patch_size,
				   int    c1, int r1,
				   int    c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
