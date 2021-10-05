/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"

#include "numrec.h"

#define VERY_SMALL_PROBABILITY 1.0e-8
#define SQRT_2 1.41421356237

/*******************
*   double @hor_normal_prob ( double mean, double st_dev, double value )
*
*   Returns the probability that a random variable with given mean and standard
*   deviation is further from the mean, in a two-sided sense.
********************/
double hor_normal_prob ( double mean, double st_dev, double value )
{
   return ( hor_erfc ( fabs(value-mean)/(SQRT_2*st_dev) ) );
}

#define LOW_START   0.0
#define HIGH_START 10.0
#define ACCURACY  1E-10

/*******************
*   double @hor_normal_thres ( double confidence_level )
*
*   Returns the threshold value of a normalised normal distribution N(0,1)
*   for a two-sided confidence test with given level, e.g. 0.05.
*   Very inefficient.
********************/
double hor_normal_thres ( double confidence_level )
{
   double low = LOW_START, high = HIGH_START, middle;

   while ( high - low > ACCURACY )
   {
      middle = 0.5*(low+high);

      if ( hor_erfc(middle/SQRT_2) > confidence_level )
	 low = middle;
      else
	 high = middle;
   }

   return middle;
}

/*******************
*   double @hor_chi_2_prob ( double chi_2, int dof )
*
*   for a specified value of chi-square and DOF returns the area under the
*   right hand section of the chi-square probability distribution, which
*   can be compared with a given confidence level (e.g. 0.05) to determine
*   whether, e.g., a least-squares residual is too large.
********************/
double hor_chi_2_prob ( double chi_2, int dof )
{
   double temp;

   if ( chi_2 < 0.0 )
   {
      hor_warning ( "illegal chi^2 value %f", chi_2 );
      return 1.0;
   }

   if ( dof < 1000 ) {
      temp = hor_gammq ( 0.5 * (double) dof, 0.5*chi_2 );
      if ( temp > VERY_SMALL_PROBABILITY )
	 return ( temp );
      else
	 return ( VERY_SMALL_PROBABILITY );
   }
   else { /* approximate with gaussian */
      double mean = (double) dof, st_dev = sqrt(2.0*mean);

      if ( chi_2 < mean )
	 return ( 1.0 - 0.5*hor_normal_prob ( mean, st_dev, chi_2 ) );
      else
	 return ( 0.5*hor_normal_prob ( mean, st_dev, chi_2 ) );
   }
}
