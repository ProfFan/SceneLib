/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/text_IO.h */

/*******************
*   typedef enum { @HOR_NON_FATAL, @HOR_FATAL } @Hor_Error_Type;
*
*   Definition of HORATIO hor_error types:
*      HOR_NON_FATAL: print hor_message but take no action.
*      HOR_FATAL:     print hor_message and exit.
********************/
typedef enum { HOR_NON_FATAL, HOR_FATAL } Hor_Error_Type;

void hor_set_print_func ( void (*print_func) ( const char * ) );
void hor_print          ( const char *fmt, ... );
void hor_message        ( const char *fmt, ... );
void hor_warning        ( const char *fmt, ... );
void hor_error          ( const char *fmt, Hor_Error_Type error_type, ... );

void hor_set_fatal_error_function ( void (*handler_func) (void) );

#ifndef HOR_REDUCED_LIB
char hor_wait_for_keyboard ( void );
#endif
