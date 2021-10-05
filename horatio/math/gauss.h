/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/gauss.h */

/* Commented out by davison@etl.go.jp 98/8/14: conflicts with later
   C libraries */
/* long random(void); */

/*******************
*   double @hor_drandom(void)
*
*   Returns random number between -1 and 1.
********************/
#define hor_drandom() ((double)(random() - 1073741824)/1073741824.0)

double hor_gauss_rand ( double mean, double st_dev );
