/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_GLOBAL_

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk) and
                  Ian Reid (ian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/error.h */

#define HORATIO_OK                0

/* global library error codes */
#define HOR_GLOBAL_READ_FAILED     100
#define HOR_GLOBAL_TALLOC_FAILED   101
#define HOR_GLOBAL_TALLOC_TOO_BIG  102
#define HOR_GLOBAL_TALLOC_OVERFLOW 103

/* maths library error codes */
#define HOR_MATH_MATRIX_SINGULAR           200
#define HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS 201
#define HOR_MATH_MATRIX_INCOMPATIBLE       202
#define HOR_MATH_MATRIX_NOT_POS_DEFINITE   203
#define HOR_MATH_MATRIX_EXTRA_ZEROES       204
#define HOR_MATH_MATRIX_NOT_SQUARE         205
#define HOR_MATH_ILLEGAL_VALUE             206
#define HOR_MATH_NO_CONVERGENCE            207
#define HOR_MATH_NUMREC_BAD_PERMUTATION    208
#define HOR_MATH_ALLOCATION_FAILED         209
#define HOR_MATH_NULL_POINTER_ARGUMENT     210
#define HOR_MATH_LIMIT_EXCEEDED            211

/* list library error codes */
#define HOR_LIST_ASSOCIATION_NOT_FOUND 400

/* image library error codes */
#define HOR_IMAGE_CANNOT_READ_IMAGE_HEADER     500
#define HOR_IMAGE_ILLEGAL_IMAGE_TYPE_IN_HEADER 501
#define HOR_IMAGE_ILLEGAL_PIXEL_SCALE          502
#define HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL       503
#define HOR_IMAGE_INVALID_IMAGE_REGION         504
#define HOR_IMAGE_IMAGE_TOO_SMALL              505
#define HOR_IMAGE_OPEN_FAILED_FOR_READ         506
#define HOR_IMAGE_OPEN_FAILED_FOR_WRITE        507
#define HOR_IMAGE_READ_FAILED                  508
#define HOR_IMAGE_WRITE_FAILED                 509
#define HOR_IMAGE_ALLOCATION_FAILED            510
#define HOR_IMAGE_NULL_POINTER_ARGUMENT        511
#define HOR_IMAGE_WRONG_TYPE_IMAGE             512
#define HOR_IMAGE_BAD_MAGIC_NUMBER             513

/* graphics library error codes */
#define HOR_GRAPHICS_DISPLAY_WINDOW_NOT_SET      600
#define HOR_GRAPHICS_CANVAS_NOT_INITIALIZED      601
#define HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO   602
#define HOR_GRAPHICS_NON_EXISTENT_FONT_NAME      603
#define HOR_GRAPHICS_ILLEGAL_INTERNAL_DIMENSIONS 604
#define HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED   605
#define HOR_GRAPHICS_INVALID_DISPLAY_DEPTH       606
#define HOR_GRAPHICS_COLOUR_NOT_ALLOCATED        607
#define HOR_GRAPHICS_NULL_POINTER_ARGUMENT       608
#define HOR_GRAPHICS_WRONG_TYPE_IMAGE            609
#define HOR_GRAPHICS_NON_EXISTENT_IMAGE          610
#define HOR_GRAPHICS_INCOMPATIBLE_IMAGE          611

/* improc library error codes */
#define HOR_IMPROC_REGION_TOO_SMALL        700
#define HOR_IMPROC_ILLEGAL_IMAGE_TYPE      701
#define HOR_IMPROC_ILLEGAL_CORNER_POSITION 702
#define HOR_IMPROC_ILLEGAL_PARAMETERS      703
#define HOR_IMPROC_IMAGES_INCOMPATIBLE     704
#define HOR_IMPROC_NULL_POINTER_ARGUMENT   705
#define HOR_IMPROC_OPEN_FAILED_FOR_READ    706
#define HOR_IMPROC_OPEN_FAILED_FOR_WRITE   707
#define HOR_IMPROC_READ_FAILED             708
#define HOR_IMPROC_ALLOCATION_FAILED       709

/* process library error codes */
#define HOR_PROCESS_UNDEFINED_RESULT_TYPE         800
#define HOR_PROCESS_UNDEFINED_PROCESS             801
#define HOR_PROCESS_UNDEFINED_PROCESS_TYPE        802
#define HOR_PROCESS_INCOMPATIBLE_RESULT_TYPES     803
#define HOR_PROCESS_DIFFERENT_LENGTH_RESULT_LISTS 804
#define HOR_PROCESS_DUPLICATED_RESULT_TYPES       805
#define HOR_PROCESS_DUPLICATED_PROCESS_TYPES      806
#define HOR_PROCESS_ILLEGAL_PROCESS_DATA          807
#define HOR_PROCESS_ILLEGAL_OUTPUT_DATA           808
#define HOR_PROCESS_IMAGES_INCOMPATIBLE           809
#define HOR_PROCESS_NULL_POINTER_ARGUMENT         810

/* tool library error codes */
#define HOR_TOOL_POPUP_PANEL_NOT_IN_USE     900
#define HOR_TOOL_POPUP_PANEL_ALREADY_IN_USE 901
#define HOR_TOOL_POPUP_PANEL_NOT_REGISTERED 902
#define HOR_TOOL_POPUP_PANEL_NOT_INIT       903
#define HOR_TOOL_GRAPH_NEGATIVE_TIME_RANGE  904
#define HOR_TOOL_GRAPH_NEGATIVE_F_RANGE     905
#define HOR_TOOL_GRAPH_ALREADY_SET_UP       906
#define HOR_TOOL_GRAPH_NOT_SET_UP           907
#define HOR_TOOL_GRAPH_TIME_REVERSAL        908
#define HOR_TOOL_GRAPH_NO_POINTS            909
#define HOR_TOOL_OPEN_FAILED_FOR_WRITE      910
#define HOR_TOOL_3D_ITEM_NOT_FOUND          911
#define HOR_TOOL_ALLOCATION_FAILED          912

extern int hor_errno;

void hor_perror ( const char *s );
/* included from global/types.h */

/* copied from /usr/include/sys/types.h */

/*******************
*   #define @u_char  unsigned char
*   #define @u_short unsigned short
*   #define @u_int   unsigned int
*   #define @u_long  unsigned long
*
*   Abbreviations of simple C types
********************/
#ifndef __SYS_TYPES_H__
#ifndef __sys_types_h
#define u_char  unsigned char
#define u_short unsigned short
#define u_int   unsigned int
#define u_long  unsigned long
#endif
#endif

/*******************
*   typedef enum { HOR_FALSE, HOR_TRUE } @Hor_Bool;
*
*   Definition of Horatio Boolean variable type.
********************/
typedef enum { HOR_FALSE, HOR_TRUE } Hor_Bool;
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/ralloc.h */

int hor_test_malloc(void);
int hor_test_free(void);

#define hor_malloc(n) malloc(n)
#define hor_free(d)  free(d)

#if 0
/* alternative allocation routines for testing whether every malloc is
   getting freed */
static void *zxqynp;
static int   yzpwj1, yzpwj2, yzpwj3;

#define hor_malloc(nbytes) (hor_print("malloc of %7d at %8x (%4d)", nbytes, (int) (zxqynp = malloc(nbytes)), hor_test_malloc()), hor_print(": codes %8x %8x, line %4d of %s\n", *(((int *) zxqynp)-2), *(((int *) zxqynp )-1), __LINE__, __FILE__ ), zxqynp)

#define hor_free(address) (yzpwj1 = *(((int *)(address))-2), yzpwj2 = *(((int *)(address))-1), hor_print("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = free(address)), (int)address, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3)
#endif

#define hor_malloc_type(type) ((type *) hor_malloc ( sizeof(type) ))
#define hor_malloc_ntype(type,n) ((type *) hor_malloc ( (n)*sizeof(type) ))

void hor_free_func ( void * ); /* function version of hor_free */
void hor_free_multiple ( void *ptr, ... );
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

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/inmos.h */

#ifdef HOR_TRANSPUTER

#ifdef __channel_h
#define _channel_h
#endif

#ifdef __process_h
#define _process_h
#endif

/*******************
*   #define @HOR_NO_LINKS 4
*
*   Number of links per transputer.
********************/
#define HOR_NO_LINKS 4

#ifdef _process_h

void HorProcRunPar  ( Process *p1, ... );
void HorProcRunList ( Process **plist );

#endif /* _process_h */

#ifdef _channel_h

Channel **HorChanList ( Channel **old_list, int no_channels );
void HorChanListReset ( Channel **chan_dest, Channel **chan_source,
		        int no_channels );
#endif /* _channel_h */

void hor_block_move_2D ( int sstride, int dstride, int rows,
			 void *saddr, void *daddr, int width );

#ifdef _channel_h
#ifdef HOR_CHANNEL_MACRO

/* 
 * Macro definitions to replace INMOS C function call 
 * and speed up communication (similar macro exist in
 * the last release (oct 92) of the INMOS C compiler)
 *
 * Fred Chenavier 8/93
 *
 * Horatio changes: Philip McLauchlan 9/93
 *
 */



/*  Chan... are function call ; it is much more efficient for small message 
    to replace them with some assembly code */


/*******************
*   void @ChanOut     ( Channel *chan, void *ptr, int length )
*   void @ChanOutChar ( Channel *chan, char number )
*   void @ChanOutInt  ( Channel *chan, int  number )
*   void @ChanIn      ( Channel *chan, void *ptr, int length )
*   void @HorChanInChar ( Channel *chan, char *number_ptr )
*   void @HorChanInInt  ( Channel *chan, int  *number_ptr )
*
*   Macro versions of Inmos C functions ChanOut(), ChanOutChar(), ChanOutInt()
*   and ChanIn(). These should be used when you want to debug the code and then
*   switch to these macro versions by compiling with the -DHOR_CHANNEL_MACRO
*   flag. There are no macro versions of ChanInChar() and ChanInInt(), since
*   they return a value, difficult for a fragment of assembler code.
*   while there are macro versions of the Hor... versions.
*
*   HorChanInChar() and HorChanInInt() are macro versions of Horatio functions
*   of the same name that take pointers as arguments rather than returning the
*   read value directly as with ChanInChar() and ChanInInt().
********************/
#define   ChanOut(_CH_PTR,_MSG_PTR,_MSG_SIZE)  \
           { int NC_LENGTH=(_MSG_SIZE);        \
            __asm {                            \
	       ld      (_MSG_PTR);             \
	       ld      (_CH_PTR);              \
	       ld      NC_LENGTH;              \
	       out;                            \
	   } }

#define   ChanOutChar(_CH_PTR,_BYTE)           \
          { char NC_BYTE = (_BYTE);            \
            __asm {                            \
	       ld      (_CH_PTR);              \
	       ld      NC_BYTE;                \
	       outbyte;                        \
	    } }


#define   ChanOutInt(_CH_PTR,_WORD)           \
           { int NC_WORD = (_WORD);            \
            __asm {                            \
	       ld      (_CH_PTR);              \
	       ld      NC_WORD;                \
	       outword;                        \
	    } }


#define   ChanIn(_CH_PTR,_MSG_PTR,_MSG_SIZE)   \
           { int NC_LENGTH=(_MSG_SIZE);        \
            __asm {                            \
	       ld      (_MSG_PTR);             \
	       ld      (_CH_PTR);              \
	       ld      NC_LENGTH;              \
	       in;                             \
	    } }

/* The 2 following macro can not replace directly the INMOS C functions
   ChanInChar()  and  ChanInInt() because these two return respectively
   a char and an int ; => let's use similar functions (like in 3L C) */


#define   HorChanInChar(_CH_PTR,_BYTE_PTR)     \
            __asm {                            \
	       ld      (_BYTE_PTR);            \
	       ld      (_CH_PTR);              \
	       inbyte;                         \
	    }

#define   HorChanInInt(_CH_PTR,_WORD_PTR)      \
            __asm {                            \
	       ld      (_WORD_PTR);            \
	       ld      (_CH_PTR);              \
	       inword;                         \
	    }
#else
void HorChanInChar ( Channel *chan, char *number );
void HorChanInInt  ( Channel *chan, int *number  );
#endif /* HOR_CHANNEL_MACRO */
#endif /* _channel_h */
#endif /* HOR_TRANSPUTER */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/ring_buf.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

/*******************
*   typedef void (*@hor_ring_buf_free_func)(void *data);
*   typedef void (*@hor_ring_buf_output_func)(Channel *channel,void *data,
*                                            void *params);
*
*   Function types for freeing and output ring-buffer functions.
*   The arguments to the output function are a channel, a pointer to the data
*   to be output and a pointer to parameters that may modify the data is
*   outputted. For transputers only.
********************/
typedef void (*hor_ring_buf_free_func)(void *data);
typedef void (*hor_ring_buf_output_func)(Channel *channel,void *data,
					 void *params);

void hor_ring_buf_init ( int result_memory_size,
			 hor_ring_buf_free_func   free_func );
void hor_ring_buf_clear(void);
void hor_ring_buf_store ( void *result );
void hor_ring_buf_output ( Channel *out, int memory_size,
			   hor_ring_buf_output_func output_func, void *params);

#endif
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/number.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

short HorChanInShort  ( Channel *c );
float HorChanInFloat  ( Channel *c );
void  HorChanOutShort ( Channel *c, short x );
void  HorChanOutFloat ( Channel *c, float x );

#endif
#endif

#ifndef HOR_REDUCED_LIB

int hor_pipe_read ( int fd, char *buf, int n );

void hor_reverse_byte_order2 ( char *ptr, int no_bytes );
void hor_reverse_byte_order4 ( char *ptr, int no_bytes );
void hor_reverse_byte_order8 ( char *ptr, int no_bytes );

char  hor_read_char   ( int fd );
int   hor_read_int    ( int fd );
float hor_read_float  ( int fd );
void  hor_write_char  ( int fd, char  number );
void  hor_write_int   ( int fd, int   number );
void  hor_write_float ( int fd, float number );

#endif /* HOR_REDUCED_LIB */
/* included from global/types.h */

/* copied from /usr/include/sys/types.h */

#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER

/*******************
*   typedef enum { HOR_NO_COMPRESS, HOR_UNIX_COMPRESS, HOR_GNU_COMPRESS }
*      @Hor_Compress_Type;
*
*   Definition of Horatio compression types for reading/writing files.
*   HOR_UNIX_COMPRESS specifies UNIX compression (compress).
*   HOR_GNU_COMPRESS  specifies Gnu compression (gzip).
********************/
typedef enum { HOR_NO_COMPRESS, HOR_UNIX_COMPRESS, HOR_GNU_COMPRESS }
   Hor_Compress_Type;

int hor_compress ( const char *file_name );
int hor_gzip     ( const char *file_name );
int hor_uncompress ( int fdin, int *pid_ptr );
int hor_gunzip     ( int fdin, int *pid_ptr );

#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
