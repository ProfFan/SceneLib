/* Copyright 1993 Philip Torr (phst@robots.oxford.ac.uk) and
		  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/math.h"

#ifdef HOR_MSDOS
#define M_PI 3.1415926535897932384626433
#endif

/*******************
*   int @hor_solve_quadratic ( double a, double b, double c,
*                             double *x1, double *x2 )
*
*   Gives solns x1 and x2 as given by Numerical Recipes to ax^2 + bx + c = 0.
*   Returns the number of real solutions, so if both are imaginary it returns
*   zero, otherwise two.
********************/
int hor_solve_quadratic ( double a, double b, double c, double *x1, double *x2)
{
   double q, s, d;

   if (b > 0.0) s = 1.0;
   else         s = -1.0;

   d = b * b - 4 * a * c;

   if (d < 0.0) /* doesn't work for complex roots */
   {
      hor_errno = HOR_MATH_ILLEGAL_VALUE;
      return 0;
   }
   else q = -0.5 * ( b + s * sqrt(d));
 
   *x1 = q/a;
   *x2 = c/q;
   return 2;
}

#ifdef HOR_MSDOS /* define cbrt() for MSDOS */
static double cbrt ( double x )
{
   return exp(log(x)/3.0);
}
#endif /* HOR_MSDOS */

/*******************
*   int @hor_solve_cubic ( double a, double b, double c, double d,
*                         double *x1, double *x2, double *x3 )
*
*   Solves a cubic by formula given in Numerical Recipes and
*   returns the number of real solutions.
*   Thus it returns 1 if there are 2 complex roots and 1 real solution
*   2 if it is in fact a fact quadratic with 2 solns
*   3 if there are 3 real solns 
*      a x^3 + b x^2 + c x + d = 0
*   puts the solns into x1, x2, x3 respectively
*   (the last of these are unset if there aren't enough solns).
********************/
int hor_solve_cubic ( double a, double b, double c, double d,
		      double *x1, double *x2, double *x3 )
{
   double q, r, theta, e, f, len;

   /* firstly check to see if we have approximately a quadratic */
   len = a*a + b*b + c*c + d*d;
   if ( fabs(a*a/len) < 0.000001 )
      return ( hor_solve_quadratic ( b, c, d, x1, x2 ) );

   b /= a; c /= a; d /= a; a = 1.0;
   q = (b*b - 3.0*c)/9.0;
   r = (2.0*b*b*b - 9.0*b*c + 27.0*d)/54.0;

   if ( r*r >= q*q*q )
   {
      e  = cbrt(fabs(r) + sqrt(r*r - q*q*q));
      if ( e*r > 0.0) e *= -1.0; /* e has to be opposite sign of r? */

      if ( e == 0.0) f = 0.0;
      else           f = q/e;

      *x1 = e + f - b/3.0; 
      return 1;
   }

   theta = acos( r/sqrt(q*q*q) );
   *x1 = -2.0*sqrt(q)*cos(theta/3.0)              - b/3.0;
   *x2 = -2.0*sqrt(q)*cos((theta + 2.0*M_PI)/3.0) - b/3.0;
   *x3 = -2.0*sqrt(q)*cos((theta - 2.0*M_PI)/3.0) - b/3.0;
   return 3;
}

