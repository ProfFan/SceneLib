/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/gaussian.h */

float *hor_make_gaussian_mask ( float sigma, int gauss_size );
void   hor_free_gaussian_mask ( float *mask, int gauss_size );
