/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
/* talloc.c: Module for temporary stack storage. Memory must be freed
	     in reverse of the order it was allocated. Much faster than
	     malloc(). */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"

/* #undef all defined constants in case they are defined elsewhere */
#undef BIGGEST_TYPE
#undef MAX_TEMP_BLOCKS
#undef TEMP_BLOCK_SIZE
#undef TEMP_TEST_MAGIC
#undef MAGIC_NUMBER

#define BIGGEST_TYPE    double /* for byte alignment */
#define MAX_TEMP_BLOCKS 50
#define TEMP_BLOCK_SIZE 4000  /* number of BIGGEST_TYPE's in block */

#define TEMP_TEST_MAGIC 1

#if TEMP_TEST_MAGIC
#define MAGIC_NUMBER 0xe7b1 /* value checked for block overwrite */
#endif

static BIGGEST_TYPE *block_ptr[MAX_TEMP_BLOCKS+1], *last_free = NULL;
static int  temp_blocks = 0, next_start = TEMP_BLOCK_SIZE;
static int  block_end[MAX_TEMP_BLOCKS], max_blocks = 0, max_no = 0;

/*******************
*   void *@hor_talloc ( int no_bytes )  void @hor_tree ( void *data )
*   void *@hor_talloc_type ( type )     void *@hor_talloc_ntype ( type, int n )
*
*   Temporary memory allocation/free routines, faster than hor_malloc,
*   hor_free etc. Memory must be freed in reverse of the order it was
*   allocated. tfree() does not actually free any memory, but allows the
*   marked memory to be used in subsequent talloc() calls. After using the
*   temporary memory, call hor_free_temp().
********************/
void *hor_talloc ( unsigned size )
{
   BIGGEST_TYPE *ptr = NULL;

   if ( size == 0 ) return NULL;

   /* make size into number of BIGGEST_TYPE's rather than bytes */
   size = 1 + (size-1)/sizeof(BIGGEST_TYPE);
#if TEMP_TEST_MAGIC
   /* next line for overwrite test */
   size += 1;
#endif
   if ( next_start + size > TEMP_BLOCK_SIZE ) /* end of block */
   {
      if ( size > TEMP_BLOCK_SIZE )
      {
	 hor_errno = HOR_GLOBAL_TALLOC_TOO_BIG;
	 return NULL;
      }

      if ( temp_blocks == MAX_TEMP_BLOCKS )
      {
	 hor_errno = HOR_GLOBAL_TALLOC_OVERFLOW;
	 return NULL;
      }

      block_end[temp_blocks++] = next_start;
      if ( temp_blocks > max_blocks )
      {
	 block_ptr[temp_blocks] = hor_malloc_ntype ( BIGGEST_TYPE,
						     TEMP_BLOCK_SIZE );
	 if ( block_ptr[temp_blocks] == NULL )
	 {
	    hor_errno = HOR_GLOBAL_TALLOC_FAILED;
	    return NULL;
	 }

/*       printf ( "allocating temporary block at %x\n",
	 block_ptr[temp_blocks] );*/
	 max_blocks = temp_blocks;
	 max_no = size;
      }

      next_start = size;
      last_free = block_ptr[temp_blocks] + TEMP_BLOCK_SIZE;
      ptr = block_ptr[temp_blocks];
   }
   else
   {
      next_start += size;
      if ( (temp_blocks == max_blocks) && (next_start > max_no) )
	 max_no = next_start;

      last_free = block_ptr[temp_blocks] + TEMP_BLOCK_SIZE;
      ptr = block_ptr[temp_blocks] + next_start - size;
   }

#if TEMP_TEST_MAGIC
   /* next lines for overwrite test */
   *((int *) ptr) = MAGIC_NUMBER;
   ptr++;
#endif

/*   printf ( "talloc %d bytes at %x\n", size, ptr );*/
   return ( ptr );
}

void hor_tfree ( void *pointer )
{
   if ( pointer != NULL )
   {
      BIGGEST_TYPE *ptr = (BIGGEST_TYPE *) pointer;

#if TEMP_TEST_MAGIC
      /* next lines for overwrite test */
      ptr--;
      if ( *((int *)ptr) != MAGIC_NUMBER )
	 hor_error ( "talloc magic number overwritten", HOR_FATAL );
#endif
/*      hor_print ( "tfree at %x\n", ptr );*/
      if ( ptr == block_ptr[temp_blocks] )
      {
	 temp_blocks--;
	 next_start = block_end[temp_blocks];
	 if ( temp_blocks > 0 )
	    last_free = block_ptr[temp_blocks] + TEMP_BLOCK_SIZE;
      }
      else
	 if ( ptr > last_free )
	    hor_error ( "temporary memory freed in wrong order", HOR_FATAL );
	 else
	 {
	    next_start = ptr - block_ptr[temp_blocks];
	    last_free = ptr;
	 }
   }
}

/*******************
*   void @hor_free_temp(void)
*   void @hor_clean_temp(void)
*   int  @hor_get_temp_total(void)
*
*   Temporary memory routines.
*
*   hor_free_temp()      frees all temporary memory.
*   hor_clean_temp()     frees all temporary memory down to the last tfree()
*                        call.
*   hor_get_temp_total() returns the total temporary memory currently allocated
*                        in bytes.
********************/
void hor_free_temp(void)
{
   int blk;

   if ( temp_blocks > 0 )
      hor_error ( "temporary memory cannot be freed", HOR_FATAL );

   for ( blk = 1; blk <= max_blocks; blk++ )
      hor_free ( (char *) block_ptr[blk] );

   temp_blocks = 0;
   next_start = TEMP_BLOCK_SIZE;
   max_blocks = 0;
   max_no = 0;
   last_free = NULL;
}

void hor_clean_temp(void)
{
   int blk;

   for ( blk = temp_blocks + 1; blk <= max_blocks; blk++ )
      hor_free ( block_ptr[blk] );

   max_blocks = temp_blocks;
   max_no = next_start;
}

int hor_get_temp_total(void)
{
   return ( ((max_blocks - 1)*TEMP_BLOCK_SIZE + max_no)*sizeof(int) );
}
