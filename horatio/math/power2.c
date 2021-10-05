/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include "horatio/global.h"
#include "horatio/math.h"

/*******************
*   int @hor_power_of_two             ( int power )
*   int @hor_highest_power_of_two_leq ( int num )
*   int @hor_int_log_to_base_two      ( int num )
*
*   Integer power of two module.
*
*   hor_power_of_two() returns 2^power.
*   hor_highest_power_of_two_leq() returns the highest power of two less than
*                                  or equal to num.
*   hor_int_log_to_base_two() returns the integer part of the base two
*                             logarithm.
********************/
int hor_power_of_two ( int power )
{
   if ( power < 0 || power > 30 )
      return 0;

   if ( power == 0 )
      return 1;
   else
      return ( 2*hor_power_of_two ( power-1 ) );
}

int hor_highest_power_of_two_leq ( int num )
{
   if ( num < 1 )
      return 0;

   if ( num == 1 )
      return 1;
   else
      return ( 2*hor_highest_power_of_two_leq ( num/2 ) );
}

int hor_int_log_to_base_two ( int num )
{
   if ( num < 1 )
      return -1;

   if ( num == 1 )
      return 0;
   else
      return ( 1+hor_int_log_to_base_two ( num/2 ) );
}

