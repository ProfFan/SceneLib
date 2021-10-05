/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/talloc.h */

/*******************
*   Hor_Bit_Array @hor_talloc_bit_array ( int no_bits )
*   void @hor_tfree_bit_array ( Hor_Bit_Array bit_array )
*
*   Temporary stack versions of hor_alloc_bit_array() and hor_free_bit_array().
*   Both are implemented as macros.
********************/
#define hor_talloc_bit_array(no_bits) ((Hor_Bit_Array)hor_talloc((((no_bits)+HOR_BITS_IN_WORD-1)/HOR_BITS_IN_WORD)*sizeof(hor_bit)))
#define hor_tfree_bit_array(bit_array) (hor_tfree((void *)(bit_array)))

void *hor_talloc ( unsigned size );
void  hor_tfree ( void *pointer );
void  hor_free_temp(void);
void  hor_clean_temp(void);
int   hor_get_temp_total(void);

#define hor_talloc_type(type) ((type *) hor_talloc ( sizeof(type) ))
#define hor_talloc_ntype(type,n) ((type *) hor_talloc ( (n)*sizeof(type) ))
