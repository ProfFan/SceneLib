/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* functions for testing the validity of parameter values */

#include <stdio.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/tool.h"

/*******************
*   Hor_Bool @hor_int_null         ( int param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_int_pos          ( int param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_int_abs_pos      ( int param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_int_abs_pos_even ( int param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_int_abs_pos_odd  ( int param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_float_null       ( float param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_float_pos        ( float param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_float_abs_pos    ( float param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_float_01_inc     ( float param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_double_null      ( double param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_double_pos       ( double param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_double_abs_pos   ( double param, const char *param_name,
*                                   Hor_Error_Type error_type )
*   Hor_Bool @hor_double_01_inc    ( double param, const char *param_name,
*                                   Hor_Error_Type error_type )
*
*   Test functions to apply to int, float and double numbers. The ..._null
*   functions always return HOR_TRUE (null test). The others return HOR_TRUE
*   if the number passes the test (e.g. hor_float_pos() returns HOR_TRUE if
*   param is > 0). If the test fails hor_error() is called with a string
*   prefixed by param_name and the given error type, and HOR_FALSE is returned.
********************/
Hor_Bool hor_int_null ( int param, const char *param_name,
		        Hor_Error_Type error_type)
{
   return HOR_TRUE;
}

Hor_Bool hor_int_pos ( int param, const char *param_name,
		       Hor_Error_Type error_type)
{
   if ( param >= 0 )
      return HOR_TRUE;

   hor_error ( "%s must be >=0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_int_abs_pos ( int param, const char *param_name,
			   Hor_Error_Type error_type )
{
   if ( param > 0 )
      return HOR_TRUE;

   hor_error ( "%s must be >0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_int_abs_pos_even ( int param, const char *param_name,
			        Hor_Error_Type error_type )
{
   if ( param > 0 && (param % 2 == 0) )
      return HOR_TRUE;

   if ( param <= 0 && (param % 2 != 0) )
      hor_error ( "%s must be >0 and even", error_type, param_name );
   else if ( param <= 0 )
      hor_error ( "%s must be >0", error_type, param_name );
   else if ( (param % 2 != 0) )
      hor_error ( "%s must be even", error_type, param_name );

   return HOR_FALSE;
}

Hor_Bool hor_int_abs_pos_odd ( int param, const char *param_name,
			       Hor_Error_Type error_type )
{
   if ( param > 0 && (param % 2 == 1) )
      return HOR_TRUE;

   if ( param <= 0 && (param % 2 != 1) )
      hor_error ( "%s must be >0 and odd", error_type, param_name );
   else if ( param <= 0 )
      hor_error ( "%s must be >0", error_type, param_name );
   else if ( (param % 2 != 1) )
      hor_error ( "%s must be odd", error_type, param_name );

   return HOR_FALSE;
}

Hor_Bool hor_float_null ( float param, const char *param_name,
			  Hor_Error_Type error_type )
{
   return HOR_TRUE;
}

Hor_Bool hor_float_pos ( float param, const char *param_name,
			 Hor_Error_Type error_type )
{
   if ( param >= 0.0F )
      return HOR_TRUE;

   hor_error ( "%s must be >=0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_float_abs_pos ( float param, const char *param_name,
			     Hor_Error_Type error_type )
{
   if ( param > 0.0F )
      return HOR_TRUE;

   hor_error ( "%s must be >0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_float_01_inc ( float param, const char *param_name,
			    Hor_Error_Type error_type )
{
   if ( param >= 0.0F && param <= 1.0F )
      return HOR_TRUE;

   hor_error ( "%s must be >=0 and <=1", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_double_null ( double param, const char *param_name,
			   Hor_Error_Type error_type )
{
   return HOR_TRUE;
}

Hor_Bool hor_double_pos ( double param, const char *param_name,
			  Hor_Error_Type error_type )
{
   if ( param >= 0.0 )
      return HOR_TRUE;

   hor_error ( "%s must be >=0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_double_abs_pos ( double param, const char *param_name,
			      Hor_Error_Type error_type )
{
   if ( param > 0.0 )
      return HOR_TRUE;

   hor_error ( "%s must be >0", error_type, param_name );
   return HOR_FALSE;
}

Hor_Bool hor_double_01_inc ( double param, const char *param_name,
			     Hor_Error_Type error_type )
{
   if ( param >= 0.0 && param <= 1.0 )
      return HOR_TRUE;

   hor_error ( "%s must be >=0 and <=1", error_type, param_name );
   return HOR_FALSE;
}
