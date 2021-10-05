/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"

/*******************
*   int @hor_sgn ( double x )
*
*    Returns: -1 if x is negative.
*              0 if x is zero.
*             +1 if x is positive.
********************/
int hor_sgn ( double x )
{
  if (x == 0.0)
    return 0;
  else
    if (x > 0.0)
      return 1;
    else
      return -1;
}

/*******************
*   double @hor_sqr ( double x )     float hor_sqrf ( float x )
*
*   Square a floating-point number. hor_sqrf() is only for transputers.
********************/
double hor_sqr ( double x )
{
   return (x*x);
}

#ifdef HOR_TRANSPUTER
float hor_sqrf ( float x )
{
   return (x*x);
}

double cbrt ( double x )
{
   if ( x < 0 ) return ( -exp(log(-x)/3.0) );
   else         return (  exp(log(x)/3.0) );
}
#endif
