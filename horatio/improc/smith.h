/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/smith.h */

#ifdef _HORATIO_IMAGE_
Hor_Corner_Map *hor_smith_corners ( Hor_Image *image,
				    int diff_thres, int geom_thres,
				    int patch_size,
				    int c1, int r1, int c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
