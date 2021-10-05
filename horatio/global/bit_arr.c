/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>

#include "horatio/global.h"

/*******************
*   void @hor_fill_bit_array ( Hor_Bit_Array bit_array, int no_bits, int code )
*
*   Fills bit list with zeroes or ones, depending on the value of code.
********************/
void hor_fill_bit_array ( Hor_Bit_Array bit_array, int no_bits, int code )
{
   int i;

   if ( code == 0 )
      for ( i = hor_bit_words(no_bits) - 1; i >= 0; i-- )
	 bit_array[i] = 0;
   else
      for ( i = hor_bit_words(no_bits) - 1; i >= 0; i-- )
	 bit_array[i] = HOR_MAX_BIT_VAL;
}

/*******************
*   void @hor_copy_bit_array ( Hor_Bit_Array source, Hor_Bit_Array dest,
*                             int no_bits )
*
*   Copies source bit list into dest.
********************/
void hor_copy_bit_array ( Hor_Bit_Array source, Hor_Bit_Array dest,
			  int no_bits )
{
   int i;

   for ( i = hor_bit_words(no_bits) - 1; i >= 0; i-- ) dest[i] = source[i];
}

static Hor_Bool initialized = HOR_FALSE;
static u_int block_01[HOR_BITS_IN_WORD+1];
static u_int block_10[HOR_BITS_IN_WORD+1];

static void block_init(void)
{
   int i;

   block_01[HOR_BITS_IN_WORD] = 0;
   for ( i = HOR_BITS_IN_WORD-1; i >= 0; i-- )
      block_01[i] = block_01[i+1] | (1 << i);

   block_10[0] = 0;
   for ( i = 1; i <= HOR_BITS_IN_WORD; i++ )
      block_10[i] = block_10[i-1] | (1 << (i-1));

   initialized = HOR_TRUE;
}

/*******************
*   void @hor_insert_bit_array ( Hor_Bit_Array dest, Hor_Bit_Array source,
*                               int no_bits, int offset )
*
*   inserts source bit array (of size no_bits) in dest, where source is
*   offset from dest by offset bits.
********************/
void hor_insert_bit_array ( Hor_Bit_Array dest, Hor_Bit_Array source,
			    int no_bits, int offset )
{
   int start_word = offset/HOR_BITS_IN_WORD;
   int end_word   = (offset+no_bits-1)/HOR_BITS_IN_WORD;
   int start_bit  = offset%HOR_BITS_IN_WORD;
   int end_bit    = (offset+no_bits-1)%HOR_BITS_IN_WORD;
   int word1, word2;

   if ( !initialized ) block_init();

   if ( start_word != end_word )
   {
      /* fill region of dest filled by source with zeroes */
      dest[start_word] &= block_10[start_bit];
      for ( word1 = start_word+1; word1 < end_word; word1++ )
	 dest[word1] = 0x0;

      dest[end_word] &= block_01[end_bit+1];

      /* hor_insert source in dest */
      dest[start_word] |= (source[0] << start_bit);
      for ( word1 = start_word+1, word2 = 0; word1 < end_word;
	    word1++, word2++ )
	 dest[word1] |= ((source[word2+1] << start_bit) |
			 (source[word2] >> (HOR_BITS_IN_WORD-start_bit)));

      dest[end_word] |= (((source[word2+1] << start_bit) |
			  (source[word2] >> (HOR_BITS_IN_WORD-start_bit))) &
			 block_10[end_bit+1]);
   }
   else /* start_word == end_word */
   {
      dest[start_word] &= (block_10[start_bit] | block_01[end_bit+1]);
      dest[start_word] |= ((source[0] << start_bit) &
			   block_10[end_bit+1]);
   }
}

/*******************
*   int @hor_common_bits ( Hor_Bit_Array bit_array1, Hor_Bit_Array bit_array2,
*                         int no_bits)
*
*   Returns the number of positions in which bit lists bit_array1 and
*   bit_array2 both have ones.
********************/
int hor_common_bits ( Hor_Bit_Array bit_array1, Hor_Bit_Array bit_array2,
		      int no_bits )
{
   int i, total = 0;

   for ( i = 0; i < no_bits; i++ )
      total += hor_get_bit ( bit_array1, i ) & hor_get_bit ( bit_array2, i );

   return total;
}

/*******************
*   void @hor_bit_array_and     ( Hor_Bit_Array source1, Hor_Bit_Array source2,
*                                Hor_Bit_Array dest, int no_bits )
*   void @hor_bit_array_or      ( Hor_Bit_Array source1, Hor_Bit_Array source2,
*                                Hor_Bit_Array dest, int no_bits )
*   void @hor_bit_array_eor     ( Hor_Bit_Array source1, Hor_Bit_Array source2,
*                                Hor_Bit_Array dest, int no_bits )
*   void @hor_bit_array_and_not ( Hor_Bit_Array source1, Hor_Bit_Array source2,
*                                Hor_Bit_Array dest, int no_bits )
*
*   Performs bitwise AND, OR, Exclusive-OR, AND-NOT respectively of bit lists
*   source1 and source2 and writes result into dest.
********************/
void hor_bit_array_and ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			 Hor_Bit_Array dest, int no_bits )
{
   int no_words = hor_bit_words(no_bits), i;

   for ( i = 0; i < no_words; i++ )
      dest[i] = source1[i] & source2[i];
}

void hor_bit_array_or ( Hor_Bit_Array source1, Hor_Bit_Array source2,
		        Hor_Bit_Array dest, int no_bits )
{
   int no_words = hor_bit_words(no_bits), i;

   for ( i = 0; i < no_words; i++ )
      dest[i] = source1[i] | source2[i];
}

void hor_bit_array_eor ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			 Hor_Bit_Array dest, int no_bits )
{
   int no_words = hor_bit_words(no_bits), i;

   for ( i = 0; i < no_words; i++ )
      dest[i] = source1[i] ^ source2[i];
}

void hor_bit_array_and_not ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			     Hor_Bit_Array dest, int no_bits )
{
   int no_words = hor_bit_words(no_bits), i;

   for ( i = 0; i < no_words; i++ )
      dest[i] = source1[i] & (source2[i] ^ HOR_MAX_BIT_VAL);
}

/*******************
*   void @hor_bit_array_print ( Hor_Bit_Array bit_array, int no_bits )
*
*   Prints a bit array as a series of 0's and 1's.
********************/
void hor_bit_array_print ( Hor_Bit_Array bit_array, int no_bits )
{
   int pt;

   for ( pt = 0; pt < no_bits; pt++ )
      if ( hor_get_bit ( bit_array, pt ) ) hor_print ( "1" );
      else                                 hor_print ( "0" );

   hor_print ( "\n" );
}
