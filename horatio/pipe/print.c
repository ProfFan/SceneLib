/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <process.h>
#include <channel.h>
#include <string.h>
#include <semaphor.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/pipe.h"

#define MAX_BUFFERED_PRINTS 20

static char      print_buffer[MAX_BUFFERED_PRINTS][HOR_MAX_PIPE_COMM_LENGTH];
static int       buffer_length[MAX_BUFFERED_PRINTS];
static int       print_buffer_no;
static Semaphore print_sema; /* interlocks use of print_buffer, buffer_length
			        and print_buffer_no */

/*******************
*   static void @buffer_pipe_print ( const char *string )
*
*   on pipeline processes, print requests are buffered and only sent as prefix
*   to requested data using hor_pipe_print_buffer(). For transputers only.
********************/
static void buffer_pipe_print ( const char *string )
{
   int length = strlen(string)+1;

   if ( length > HOR_MAX_PIPE_COMM_LENGTH )
   {
      hor_warning ( "hor_message too long (buffer_pipe_print)" );
      return;
   }

   SemWait ( &print_sema );
   if ( print_buffer_no < MAX_BUFFERED_PRINTS )
   {
      buffer_length[print_buffer_no] = length;
      strcpy ( print_buffer[print_buffer_no++], string );
   }

   SemSignal ( &print_sema );
}

/*******************
*   void @hor_pipe_print_buffer ( Channel *out )
*
*   output buffered print requests from pipeline process. For transputers only.
********************/
void hor_pipe_print_buffer ( Channel *out )
{
   int i;

   SemWait ( &print_sema );
   for ( i = 0; i < print_buffer_no; i++ )
   {
      ChanOutChar ( out, HOR_PIPE_PRINT );
      ChanOutInt  ( out, buffer_length[i] );
      ChanOut     ( out, print_buffer[i], buffer_length[i] );
   }

   print_buffer_no = 0;
   SemSignal ( &print_sema );
}

/*******************
*   void @hor_pipe_set_print_func(void)
*
*   sets HORATIO print function for pipeline processes to buffer print
*   requests. For transputers only.
********************/
void hor_pipe_set_print_func(void)
{
   SemInit ( &print_sema, 1 );
   print_buffer_no = 0;
   hor_set_print_func ( buffer_pipe_print );
}

