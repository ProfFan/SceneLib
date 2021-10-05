/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifdef HOR_TRANSPUTER
#include <misc.h>
#include <process.h>
#else
#ifdef HOR_MSDOS
#include <io.h>
#else
#include <unistd.h>
#endif
#include <signal.h>
#endif

#ifdef HOR_REDUCED_LIB
#include <stdiored.h>
#else
#include <stddef.h>
#include <stdio.h>
#endif

#include "horatio/global.h"

#define MAX_STRING_LENGTH 1024

static int    print_func_initialised = 0;
static void (*print_func) ( const char *s );

/*******************
*   void @hor_set_print_func ( void (*new_print_func) ( const char *s ) )
*
*   Sets function to be called when Horatio print functions
*   hor_message(), hor_print(), hor_warning() or hor_error() are called.
*   Typical use is to print to a text window on an X application.
*   If hor_set_print_func() is not called printing is sent to stderr.
********************/
void hor_set_print_func ( void (*new_print_func) ( const char *s ) )
{
   print_func             = new_print_func;
   print_func_initialised = 1;
}

/*******************
*   void @hor_print   ( const char *fmt, ... )
*   void @hor_message ( const char *fmt, ... )
*   void @hor_warning ( const char *fmt, ... )
*   void @hor_error   ( const char *fmt, Hor_Error_Type error_type, ... )
*
*   void @hor_set_fatal_error_function ( void (*handler_func) (void) )
*
*   hor_print() is a Horatio version of printf().
*   hor_message() is used for short simple hor_messages. hor_message()
*                 appends a '\n'.
*   hor_warning() warns of unusual events requiring no action to be taken.
*   hor_error() is the Horatio error function. HOR_FATAL error type terminates
*               the program, optionally first calling a function specified by
*               hor_set_fatal_error_function(). HOR_NON_FATAL error has no
*               effect.
********************/
void hor_print ( const char *fmt, ... )
{
   va_list ap;
   char    string[MAX_STRING_LENGTH];

   /* read string format arguments */
   va_start ( ap, fmt );
   vsprintf ( string, fmt, ap );
   va_end ( ap );

   if ( print_func_initialised )
      print_func ( string );
#ifndef HOR_REDUCED_LIB
   else
      fprintf ( stderr, string );
#endif
}

void hor_message ( const char *fmt, ... )
{
   va_list ap;
   char    string[MAX_STRING_LENGTH];

   /* read string format arguments */
   va_start ( ap, fmt );
   vsprintf ( string, fmt, ap );
   va_end ( ap );

   hor_print ( "Message: %s\n", string );
}

void hor_warning ( const char *fmt, ... )
{
   va_list ap;
   char    string[MAX_STRING_LENGTH];

   /* read string format arguments */
   va_start ( ap, fmt );
   vsprintf ( string, fmt, ap );
   va_end ( ap );

   hor_print ( "Warning: %s\n", string );
}

static Hor_Bool fatal_error_func_initialised = HOR_FALSE;
static void   (*fatal_error_func)(void);

void hor_error ( const char *fmt, Hor_Error_Type error_type, ... )
{
   va_list ap;
   char    string[MAX_STRING_LENGTH];

   /* read string format arguments */
   va_start ( ap, error_type );
   vsprintf ( string, fmt, ap );
   va_end ( ap );

   switch ( error_type )
   {
      case HOR_NON_FATAL:
      hor_print ( "Error: %s: non fatal\n", string );
      break;

      case HOR_FATAL:
      if ( print_func_initialised )
#ifdef HOR_TRANSPUTER
	 hor_print ( "Error: %s: fatal: terminating\n", string );
#else
#ifdef HOR_MSDOS
	 hor_print ( "Error: %s: fatal: terminating\n", string );
#else
      {
	 hor_print ( "Error: %s: fatal: death in five seconds\n", string );
	 fprintf ( stderr, "Error: %s: fatal: death in five seconds\n",
		   string );
	 sleep ( 5 );
      }
#endif
#endif
#ifndef HOR_REDUCED_LIB
      else fprintf ( stderr, "Error: %s: fatal\n", string );
#endif

      if ( fatal_error_func_initialised ) fatal_error_func();

#ifdef HOR_TRANSPUTER
      abort();
#else
#ifdef HOR_MSDOS
      abort();
#else
      kill ( getpid(), SIGINT );
#endif
#endif
      break;

      default:
      hor_error ( "Error: bad error type %d (hor_error)\n",
		  HOR_FATAL, error_type);
      break;
   }
}

void hor_set_fatal_error_function ( void (*handler_func) (void) )
{
   fatal_error_func = handler_func;
   fatal_error_func_initialised = HOR_TRUE;
}

#ifndef HOR_REDUCED_LIB
/*******************
*   char @hor_wait_for_keyboard ( void )
*
*   Waits for and returns character from keyboard (stdin).
********************/
char hor_wait_for_keyboard ( void )
{
   char result;

   printf ( "waiting for character\n" );
   result = (char) getchar();
   return result;
}
#endif
