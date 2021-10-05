/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/convolve.h */

#ifdef _HORATIO_IMAGE_
Hor_Bool hor_convolve_image ( Hor_Image *imptr, float *cmask, int csize,
			                        float *rmask, int rsize,
			      Hor_Sub_Image *result );
#endif /* _HORATIO_IMAGE_ */
