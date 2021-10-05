/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/read_prm.h */

/* maximum length of editable parameter string */
#define HOR_EDIT_STRING_SIZE 20

Hor_Bool hor_read_double_param(char        read_string[HOR_EDIT_STRING_SIZE],
			       const char *param_name,
			       Hor_Bool  (*test_function)(double, const char *,
							  Hor_Error_Type),
			       Hor_Error_Type error_type,
			       double *dp );
Hor_Bool hor_read_float_param ( char        read_string[HOR_EDIT_STRING_SIZE],
			        const char *param_name,
			        Hor_Bool  (*test_function)(float, const char *,
							   Hor_Error_Type),
			        Hor_Error_Type error_type,
			        float *fp );
Hor_Bool hor_read_int_param   ( char        read_string[HOR_EDIT_STRING_SIZE],
			        const char *param_name,
			        Hor_Bool  (*test_function)(int, const char *,
							   Hor_Error_Type),
			        Hor_Error_Type error_type,
			        int *ip );
