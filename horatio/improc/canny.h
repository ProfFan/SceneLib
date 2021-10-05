/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/edge.h */

#ifdef _HORATIO_IMAGE_
Hor_Edge_Map *hor_canny ( Hor_Image *imptr, float *gauss_mask, int gauss_size,
			  float low_thres, float high_thres,
			  int   string_length_thres, int c1, int r1,
			                             int c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
