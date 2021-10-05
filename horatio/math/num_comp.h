/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/num_comp.h */

/*******************
*   @hor_min2 ( n1, n2 )   @hor_min3 ( n1, n2, n3 )
*   @hor_max2 ( n1, n2 )   @hor_max3 ( n1, n2, n3 )
*
*   Number comparison macros for speed with simple expressions.
********************/
#define hor_min2(n1,n2) ((n2 < n1) ? n2 : n1)
#define hor_max2(n1,n2) ((n2 > n1) ? n2 : n1)
#define hor_min3(n1,n2,n3) ((n2<n1)?((n3<n2) ? n3 : n2) : ((n3<n1) ? n3 : n1))
#define hor_max3(n1,n2,n3) ((n2>n1)?((n3>n2) ? n3 : n2) : ((n3>n1) ? n3 : n1))

/* functions for complex expressions */
int    hor_imin ( int    n1, int    n2 );
int    hor_imax ( int    n1, int    n2 );
float  hor_fmin ( float  n1, float  n2 );
float  hor_fmax ( float  n1, float  n2 );
double hor_dmin ( double n1, double n2 );
double hor_dmax ( double n1, double n2 );
