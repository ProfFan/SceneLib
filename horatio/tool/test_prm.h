/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/test_prm.h */

Hor_Bool hor_int_null         ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_pos          ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos      ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos_even ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos_odd  ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_null       ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_pos        ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_abs_pos    ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_01_inc     ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_null      ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_pos       ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_abs_pos   ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_01_inc    ( double param, const char *param_name,
			        Hor_Error_Type error_type );
