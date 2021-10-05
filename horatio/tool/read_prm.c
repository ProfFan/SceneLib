/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* read_param.c: functions to read parameter values of various types from
                 input string. Functions have form:

		 Hor_Bool read_****_param ( read_string, param_name,
		                            test_function, ptr )

		 where **** is a parameter type (int, float etc.),
		       read_string is the input string,
		       param_name is the name of the parameter to b used for
		                  printing error hor_messages,
		       test_function is a function called on the parameter
		                  value to check its validity, and
		       ptr is a pointer to the parameter.

		 If an error occurs (either the parameter could not be read
		 from the string or the value was invalid) zero is returned.
		 Otherwise one is returned. */

#include <stdlib.h>
#include <stdio.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/tool.h"

/*******************
*   Hor_Bool @hor_read_double_param(char read_string[HOR_EDIT_STRING_SIZE],
*                                  const char *param_name,
*                                  Hor_Bool (*test_func)(double, const char *,
*                                                        Hor_Error_Type),
*                                  Hor_Error_Type error_type,
*                                  double *dp)
*   Hor_Bool @hor_read_float_param(char read_string[HOR_EDIT_STRING_SIZE],
*                                 const char *param_name,
*                                 Hor_Bool (*test_func)(float, const char *,
*                                                       Hor_Error_Type),
*                                 Hor_Error_Type error_type,
*                                 float *fp)
*   Hor_Bool @hor_read_int_param ( char read_string[HOR_EDIT_STRING_SIZE],
*                                 const char *param_name,
*                                 Hor_Bool (*test_function)(int, const char *,
*                                                           Hor_Error_Type),
*                                 Hor_Error_Type error_type,
*                                 int *ip )
*
*   Reads a float/int parameter from the string read_string. The function
*   test_function() is called on the value read and if the test returns
*   HOR_FALSE, HOR_FALSE is returned, otherwise HOR_TRUE is returned.
********************/
Hor_Bool hor_read_double_param(char        read_string[HOR_EDIT_STRING_SIZE],
			       const char *param_name,
			       Hor_Bool  (*test_function)(double, const char *,
							  Hor_Error_Type),
			       Hor_Error_Type error_type,
			       double *dp )
{
   if ( sscanf ( read_string, "%lf", dp ) != 1 )
   {
      hor_error ( "%s cannot be read from string %s", error_type,
		  param_name, read_string );
      return HOR_FALSE;
   }

   if ( !test_function ( *dp, param_name, error_type ) )
      return HOR_FALSE;

   return HOR_TRUE;
}

Hor_Bool hor_read_float_param ( char        read_string[HOR_EDIT_STRING_SIZE],
			        const char *param_name,
			        Hor_Bool  (*test_function)(float, const char *,
							   Hor_Error_Type),
			        Hor_Error_Type error_type,
			        float *fp )
{
   if ( sscanf ( read_string, "%f", fp ) != 1 )
   {
      hor_error ( "%s cannot be read from string %s", error_type,
		  param_name, read_string );
      return HOR_FALSE;
   }

   if ( !test_function ( *fp, param_name, error_type ) )
      return HOR_FALSE;

   return HOR_TRUE;
}

Hor_Bool hor_read_int_param ( char        read_string[HOR_EDIT_STRING_SIZE],
			      const char *param_name,
			      Hor_Bool  (*test_function)(int, const char *,
							 Hor_Error_Type),
			      Hor_Error_Type error_type,
			      int *ip )
{
   if ( sscanf ( read_string, "%d", ip ) != 1 )
   {
      hor_error ( "%s cannot be read from string %s", error_type,
		  param_name, read_string );
      return HOR_FALSE;
   }

   if ( !test_function ( *ip, param_name, error_type ) )
      return HOR_FALSE;

   return HOR_TRUE;
}
