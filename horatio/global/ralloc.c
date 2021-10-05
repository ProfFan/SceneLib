/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "horatio/global.h"

static int malloc_count = 0;

int hor_test_malloc ( void )
{
   return ( ++malloc_count );
}

int hor_test_free ( void )
{
  return ( --malloc_count );
}

/*******************
*   void *@hor_malloc ( int no_bytes )  void @hor_free ( void *data )
*   void *@hor_malloc_type ( type )     void *@hor_malloc_ntype ( type, int n )
*   void @hor_free_func ( void *data )
*   void @hor_free_multiple ( void *ptr, ... )
*
*   Memory allocation/free routines. hor_malloc and hor_free will normally be
*   #define'd to malloc and free, but may be redefined for testing purposes.
*
*   hor_malloc_type() allocates and returns space for a single instance of the
*   given arbitrary type.
*
*   hor_malloc_ntype() does the same as hor_malloc_type() for an array.
*
*   hor_free_func() calls hor_free() and is guaranteed to be a function rather
*   than a macro with arguments, so can be used as an argument to a function.
*
*   hor_free_multiple() calls hor_free() on each element of the NULL-terminated
*   list of pointers.
********************/
void hor_free_func ( void *data )
{
   hor_free ( data );
}

void hor_free_multiple ( void *ptr, ... )
{
   va_list ap;

   if ( ptr == NULL ) return;

   hor_free ( ptr );
   va_start ( ap, ptr );
   for(;;)
   {
      ptr = va_arg ( ap, void * );
      if ( ptr == NULL ) break;

      hor_free ( ptr );
   }

   va_end(ap);
}
