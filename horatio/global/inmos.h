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
