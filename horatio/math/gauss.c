/* Modified from Numerical Recipes:

   Copyright Numerical Recipes Software 1987, 1988

   HORATIO mods: Copyright 1993 Philip F. McLauchlan
                                (pm@robots.oxford.ac.uk)
                                Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/math.h"

/*******************
*   double @hor_gauss_rand ( double mean, double st_dev )
*
*   Returns Gaussian random variable with given mean and standard deviation.
********************/
double hor_gauss_rand ( double mean, double st_dev )
{
   double fac, r, v1, v2;

   do
   {
      v1 = hor_drandom();
      v2 = hor_drandom();
      r = v1*v1 + v2*v2;
   }
   while ( r > 1.0 );

   fac = sqrt ( -2.0*log(r)/r );
   return ( mean + v1*fac*st_dev );
   /* other value is mean + v2*fac*st_dev (discarded) */
}
