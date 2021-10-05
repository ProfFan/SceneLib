/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#ifdef HOR_TRANSPUTER
#include <iocntrl.h>
#include <channel.h>
#include <misc.h>
#else
#ifdef HOR_MSDOS
#include <io.h>
#else
#include <unistd.h>
#endif
#include <fcntl.h>
#endif

#include "horatio/global.h"

#ifdef HOR_TRANSPUTER

/*******************
*   short @HorChanInShort  ( Channel *c )
*   float @HorChanInFloat  ( Channel *c )
*   void  @HorChanOutShort ( Channel *c, short x )
*   void  @HorChanOutFloat ( Channel *c, float x )
*   
*   short and float channel I/O functions for transputers.
********************/
short HorChanInShort ( Channel *c )
{
   short x;

   ChanIn ( c, &x, sizeof(short) );
   return x;
}

float HorChanInFloat ( Channel *c )
{
   float x;

   ChanIn ( c, &x, sizeof(float) );
   return x;
}

void HorChanOutShort ( Channel *c, short x )
{
   short fx = x; /* necessary because of default type conversion short->int */

   ChanOut ( c, &fx, sizeof(short) );
}

void HorChanOutFloat ( Channel *c, float x )
{
   float fx = x; /*necessary because of default type conversion float->double*/

   ChanOut ( c, &fx, sizeof(float) );
}

#endif

#ifndef HOR_REDUCED_LIB

/*******************
*   int @hor_pipe_read ( int fd, char *buf, int n )
*
*   Like read() except it waits for all bytes to be read in case data arrives
*   in bits, which it might do from a UNIX pipe.
********************/
int hor_pipe_read ( int fd, char *buf, int n )
{
   int needed = n, nread;

   while ( needed > 0 )
      if ( (nread = read ( fd, buf, needed )) == -1 )
      {
	 hor_errno = HOR_GLOBAL_READ_FAILED;
	 return ( -1 );
      }
      else
      {
	 needed -= nread;
	 buf += nread;
      }

   return n;
}

/*******************
*   void @hor_reverse_byte_order2 ( char *ptr, int no_bytes )
*   void @hor_reverse_byte_order4 ( char *ptr, int no_bytes )
*   void @hor_reverse_byte_order8 ( char *ptr, int no_bytes )
*
*   Reverse byte order of block of data because of byte reversal between Sun
*   and transputers.
*
*   hor_reverse_byte_order2() works for short's.
*   hor_reverse_byte_order4() works for int's and float's.
*   hor_reverse_byte_order8() works for double's.
*
*   hor_reverse_byte_order4() is used a lot to transfer structures containing
*   a mixture of int's and float's.
********************/
void hor_reverse_byte_order2 ( char *ptr, int no_bytes )
{
   char temp, *p1, *p2, *pend = ptr + no_bytes;

   if ( no_bytes % 2 != 0 )
      hor_error ( "illegal number of bytes (hor_reverse_byte_order2)", HOR_FATAL );

   for ( p1 = ptr, p2 = ptr+1; p1 != pend; p1 += 2, p2 += 2 )
   {
      temp = *p1; *p1 = *p2; *p2 = temp;
   }
}

void hor_reverse_byte_order4 ( char *ptr, int no_bytes )
{
   char temp, *p1, *p2, *p3, *p4, *pend = ptr + no_bytes;

   if ( no_bytes % 4 != 0 )
      hor_error ( "illegal number of bytes (hor_reverse_byte_order4)", HOR_FATAL );

   for ( p1 = ptr, p2 = ptr+1, p3 = ptr+2, p4 = ptr+3; p1 != pend;
	 p1 += 4, p2 += 4, p3 += 4, p4 += 4 )
   {
      temp = *p1; *p1 = *p4; *p4 = temp;
      temp = *p2; *p2 = *p3; *p3 = temp;
   }
}

void hor_reverse_byte_order8 ( char *ptr,
			   int   no_bytes )
{
   char temp, *p1, *p2, *p3, *p4, *p5, *p6, *p7, *p8, *pend = ptr + no_bytes;

   if ( no_bytes % 8 != 0 )
      hor_error ( "illegal number of bytes (hor_reverse_byte_order8)", HOR_FATAL );

   for ( p1 = ptr,   p2 = ptr+1, p3 = ptr+2, p4 = ptr+3,
	 p5 = ptr+4, p6 = ptr+5, p7 = ptr+6, p8 = ptr+7; p1 != pend;
	 p1 += 8, p2 += 8, p3 += 8, p4 += 8,
	 p5 += 8, p6 += 8, p7 += 8, p8 += 8 )
   {
      temp = *p1; *p1 = *p8; *p8 = temp;
      temp = *p2; *p2 = *p7; *p7 = temp;
      temp = *p3; *p3 = *p6; *p6 = temp;
      temp = *p4; *p4 = *p5; *p5 = temp;
   }
}

/*******************
*   char  @hor_read_char   ( int fd )
*   int   @hor_read_int    ( int fd )
*   float @hor_read_float  ( int fd )
*   void  @hor_write_char  ( int fd, char  number )
*   void  @hor_write_int   ( int fd, int   number )
*   void  @hor_write_float ( int fd, float number )
*
*   I/O functions for simple C types, performing byte-reversal when
*   necessary.
********************/
char hor_read_char ( int fd )
{
   char number;

   if ( hor_pipe_read ( fd, &number, 1 ) != 1 )
      hor_error ( "read failed (hor_read_char)", HOR_FATAL );

   return number;
}

int hor_read_int ( int fd )
{
   int number;

   if ( hor_pipe_read ( fd, (char *) &number, sizeof(int) ) != sizeof(int) )
      hor_error ( "read failed (hor_read_int)", HOR_FATAL );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) &number, sizeof(int) );
#endif
   return number;
}

float hor_read_float ( int fd )
{
   float number;

   if ( hor_pipe_read ( fd, (char *) &number, sizeof(float) ) != sizeof(float))
      hor_error ( "read failed (hor_read_float)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) &number, sizeof(float) );
#endif
   return number;
}

void hor_write_char ( int fd, char number )
{
   if ( write ( fd, &number, 1 ) != 1 )
      hor_error ( "write failed (hor_write_char)", HOR_FATAL );
}

void hor_write_int ( int fd, int number )
{
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) &number, sizeof(int) );
#endif
   if ( write ( fd, (char *) &number, sizeof(int) ) != sizeof(int) )
      hor_error ( "write failed (hor_write_int)", HOR_FATAL );
}

void hor_write_float ( int fd, float number )
{
   float number_f = (float) number; /* necessary 'cos of float->double
				       argument conversion */
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) &number_f, sizeof(float) );
#endif
   if ( write ( fd, (char *) &number_f, sizeof(float) ) != sizeof(float) )
      hor_error ( "write failed (hor_write_float)", HOR_FATAL );
}

#endif /* HOR_REDUCED_LIB */
