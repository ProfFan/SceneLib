/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* ring_buffer.c: module for storing result data in a ring
                  buffer and outputting it when requested */
#include <stdlib.h>
#include <stdiored.h>
#include <channel.h>

#include "horatio/global.h"

static int    result_memory_size;
static void **ring_buffer;
static int    start = 0, end = -1;

static void    (*free_func)(void *);
static Hor_Bool  initialized = HOR_FALSE;

/*******************
*   void @hor_ring_buf_init ( int    result_memory_size,
*                            void (*free_func)(void *) )
*   void @hor_ring_buf_clear(void)
*   void @hor_ring_buf_store ( void *result )
*   void @hor_ring_buf_output ( Channel *out, int memory_size,
*                              hor_ring_buf_output_func output_func,
*                              void *params )
*
*   General ring-buffer manipulation module, for transputers only.
*
*   hor_ring_buf_init() initializes the ring buffer module with a maximum
*                       buffer size and result free function.
*   hor_ring_buf_clear() frees the existing ring buffer, if any.
*   hor_ring_buf_store() stores a new result in the ring buffer, discarding an
*                        old result if necessary.
*   hor_ring_buf_output() outputs the memory_size most recent results stored
*                         in the ring-buffer to a channel using the provided
*                         output function and parameter pointer, which is
*                         passed into output_func() when it is called as its
*                         last argument.
********************/
void hor_ring_buf_init ( int    loc_result_memory_size,
			 void (*loc_free_func)(void *) )
{
   if ( initialized )
      hor_free ( (void *) ring_buffer );

   if ( loc_result_memory_size < 1 )
      hor_error ( "illegal memory size %d (hor_ring_buf_init)", HOR_FATAL,
	      loc_result_memory_size );

   result_memory_size = loc_result_memory_size;
   ring_buffer = hor_malloc_ntype ( void *, result_memory_size );
   if ( ring_buffer == NULL )
      hor_error ( "ring buffer allocation failed (hor_ring_buf_init)",
		  HOR_FATAL );

   free_func   = loc_free_func;
   initialized = HOR_TRUE;
}

void hor_ring_buf_clear(void)
{
   int i;

   if ( !initialized )
      hor_error ( "no initialized (hor_ring_buf_clear)", HOR_FATAL );

   if ( end == -1 ) return;

   if ( start <= end )
      for ( i = start; i <= end; i++ )
         free_func ( ring_buffer[i] );
   else
   {
      for ( i = start; i < result_memory_size; i++ )
         free_func ( ring_buffer[i] );

      for ( i = 0; i <= end; i++ )
         free_func ( ring_buffer[i] );
   }

   start = 0; end = -1;
}

void hor_ring_buf_store ( void *result )
{
   if ( !initialized )
      hor_error ( "no initialized (hor_ring_buf_store)", HOR_FATAL );

   if ( end != -1 ) {
      if ( (start - end - 1) % result_memory_size == 0 )
      {
	 free_func ( ring_buffer[start] );
	 start = (start + 1) % result_memory_size;
      }
   }

   end = (end + 1) % result_memory_size;
   ring_buffer[end] = result;
}

void hor_ring_buf_output ( Channel *out, int memory_size,
			   hor_ring_buf_output_func output_func, void *params )
{
   int i, total;

   if ( !initialized )
      hor_error ( "no initialized (hor_ring_buf_output)", HOR_FATAL );

   /* output number of buffered results */
   if ( end == -1 )
   {
      ChanOutInt ( out, 0 );
      return;
   }
   else if ( start <= end )
   {
      total = end - start + 1;
      if ( total > memory_size ) total = memory_size;
      ChanOutInt ( out, total );
      for ( i = end - total + 1; i <= end; i++ )
	 output_func ( out, ring_buffer[i], params );
   }
   else /* start > end */
   {
      total = end - start + 1 + result_memory_size;
      if ( total > memory_size ) total = memory_size;
      ChanOutInt ( out, total );
      for ( i = result_memory_size - (total - end - 1);
	    i < result_memory_size; i++ )
	 output_func ( out, ring_buffer[i], params );

      if ( total > end )
	 for ( i = 0; i <= end; i++ )
	    output_func ( out, ring_buffer[i], params );
      else
	 for ( i = end - total + 1; i <= end; i++ )
	    output_func ( out, ring_buffer[i], params );
   }
}
