/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include "horatio/global.h"
#include "horatio/math.h"

/*******************
*   int    @hor_imin ( int    n1, int    n2 )
*   int    @hor_imax ( int    n1, int    n2 )
*   float  @hor_fmin ( float  n1, float  n2 )
*   float  @hor_fmax ( float  n1, float  n2 )
*   double @hor_dmin ( double n1, double n2 )
*   double @hor_dmax ( double n1, double n2 )
*
*   Number comparison functions.
********************/
int hor_imin ( int n1, int n2 )
{
   return ( (n2 < n1) ? n2 : n1 );
}

int hor_imax ( int n1, int n2 )
{
   return ( (n2 > n1) ? n2 : n1 );
}

float hor_fmin ( float n1, float n2 )
{
   return ( (n2 < n1) ? n2 : n1 );
}

float hor_fmax ( float n1, float n2 )
{
   return ( (n2 > n1) ? n2 : n1 );
}

double hor_dmin ( double n1, double n2 )
{
   return ( (n2 < n1) ? n2 : n1 );
}

double hor_dmax ( double n1, double n2 )
{
   return ( (n2 > n1) ? n2 : n1 );
}
