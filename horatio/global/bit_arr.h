/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/bit_arr.h */

#include <limits.h>

/*******************
*   typedef u_int    @hor_bit;
*   typedef hor_bit *@Hor_Bit_Array;
*
*   Definitions of bit and bit array types. @HOR_BITS_IN_WORD is #defined to
*   be the number of bits in a hor_bit, and @HOR_MAX_BIT_VAL is the value with
*   al bits set
********************/
typedef u_int    hor_bit;
typedef hor_bit *Hor_Bit_Array;
#define HOR_BITS_IN_WORD (8*sizeof(hor_bit))
#define HOR_MAX_BIT_VAL UINT_MAX

void    hor_set_bit   ( Hor_Bit_Array bit_array, u_int position );
void    unhor_set_bit ( Hor_Bit_Array bit_array, u_int position );
hor_bit hor_get_bit   ( Hor_Bit_Array bit_array, u_int position );

/*******************
*   Hor_Bit_Array @hor_alloc_bit_array ( int no_bits )
*   void @hor_free_bit_array ( Hor_Bit_Array bit_array )
*
*   hor_alloc_bit_array() Creates and returns bit list of given size.
*   hor_free_bit_array() frees memory associated with a bit list.
*   Both are implemented as macros.
********************/
#define hor_alloc_bit_array(no_bits) ((Hor_Bit_Array)hor_malloc((((no_bits)+HOR_BITS_IN_WORD-1)/HOR_BITS_IN_WORD)*sizeof(hor_bit)))
#define hor_free_bit_array(bit_array) (hor_free((void *)(bit_array)))

/*******************
*   void @hor_set_bit   ( Hor_Bit_Array bit_array, u_int position )
*   void @hor_unset_bit ( Hor_Bit_Array bit_array, u_int position )
*
*   Sets bit to one (hor_set_bit) or zero (unhor_set_bit) at given position in
*   bit list. Both are implemented as macros.
********************/
#define hor_set_bit(array,pos) ((array[(pos)/HOR_BITS_IN_WORD] |= (1<<(pos%HOR_BITS_IN_WORD))))
#define hor_unset_bit(array,pos) ((array[(pos)/HOR_BITS_IN_WORD] &=HOR_MAX_BIT_VAL^(1<<(pos%HOR_BITS_IN_WORD)))))

/*******************
*   hor_bit @hor_get_bit ( hor_bit *bit_array, u_int position )
*
*   Returns bit in given position in bit list. Implemented as a macro.
********************/
#define hor_get_bit(array,pos) ((array[(pos)/HOR_BITS_IN_WORD] & (1<<(pos%HOR_BITS_IN_WORD))))

void hor_fill_bit_array   ( Hor_Bit_Array bit_array, int no_bits, int code );
void hor_copy_bit_array   ( Hor_Bit_Array source, Hor_Bit_Array dest,
			    int no_bits );
void hor_insert_bit_array ( Hor_Bit_Array dest, Hor_Bit_Array source,
			    int no_bits, int offset );
int  hor_common_bits     ( Hor_Bit_Array bit_array1, Hor_Bit_Array bit_array2,
			   int no_bits );

void hor_bit_array_and     ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			     Hor_Bit_Array dest, int no_bits );
void hor_bit_array_or      ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			     Hor_Bit_Array dest, int no_bits );
void hor_bit_array_eor     ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			     Hor_Bit_Array dest, int no_bits );
void hor_bit_array_and_not ( Hor_Bit_Array source1, Hor_Bit_Array source2,
			     Hor_Bit_Array dest, int no_bits );
void hor_bit_array_print ( Hor_Bit_Array bit_array, int no_bits );

/*******************
*   int @hor_bit_words ( int no_bits )
*
*   Function returning the number of bit words needed for a bit array of
*   given size, implemented as a macro.
********************/
#define hor_bit_words(no_bits) (((no_bits) + HOR_BITS_IN_WORD-1)/HOR_BITS_IN_WORD)

