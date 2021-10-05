/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/math.h */

int hor_sgn ( double );
double hor_sqr( double );

#ifdef HOR_TRANSPUTER
float hor_sqrf ( float );
double cbrt ( double );
#else
#define hor_sqrtf sqrt
#define hor_sqrf hor_sqr
#define hor_fabsf fabs
#endif

/*******************
*   double @HOR_DISTANCE_2D ( double x1, double y1, double x2, double y2 )
*   double @HOR_DISTANCE_3D ( double x1, double y1, double z1,
*                            double x2, double y2, double z2 )
*
*   float @HOR_DISTANCE_2Df ( float x1, float y1, float x2, float y2 )
*   float @HOR_DISTANCE_3Df ( float x1, float y1, float z1,
*                            float x2, float y2, float z2 )
*
*   Euclidean distance measures, implemented as macros (each argument is
*   evaluated twice). HOR_DISTANCE_2Df() and HOR_DISTANCE_3Df() are only for
*   transputers.
********************/
#define HOR_DISTANCE_2D(x1,y1,x2,y2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
#define HOR_DISTANCE_3D(x1,y1,z1,x2,y2,z2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

#ifdef HOR_TRANSPUTER
#define HOR_DISTANCE_2Df(x1,y1,x2,y2) sqrtf((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
#define HOR_DISTANCE_3Df(x1,y1,z1,x2,y2,z2) sqrtf((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))
#endif

/*******************
*   @HOR_SIGN ( x )     @HOR_SIGNF ( x )     @HOR_SIGND ( x )
*
*   Returns the sign of the argument, 1 for x>0, -1 for x<0 and 0 for x=0.
*   Implemented as a macro. HOR_SIGNF and HOR_SIGND are for float and double
*   arguments, and return float and double results.
********************/
#define HOR_SIGN(x)  ((x) > 0    ? 1    : ((x) < 0    ? -1    : 0   ))
#define HOR_SIGNF(x) ((x) > 0.0F ? 1.0F : ((x) < 0.0F ? -1.0F : 0.0F))
#define HOR_SIGND(x) ((x) > 0.0  ? 1.0  : ((x) < 0.0  ? -1.0  : 0.0 ))

#ifdef HOR_TRANSPUTER
#define M_PI  3.14159265358979323846
#define M_PIf 3.14159265358979323846F
#endif
