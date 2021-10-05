/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
/* image_stream_io.c: functions for communicating images between processes.
		      In the case of transputer to Sun communication, one end
		      must reverse the byte order of 4-byte words. We choose
		      the Sparc 'cos it's quicker. */

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#ifdef HOR_TRANSPUTER
#include <misc.h>
#include <iocntrl.h>
#else
#ifdef HOR_MSDOS
#include <io.h>
#else
#include <unistd.h>
#endif
#endif

#include "horatio/global.h"
#include "horatio/image.h"

#define  FLOAT_CODE 0
#define    BIT_CODE 1
#define U_CHAR_CODE 2
#define    INT_CODE 3

static Hor_Image_Type image_type_from_code ( int type_code )
{
   switch ( type_code )
   {
      case  FLOAT_CODE: return HOR_FLOAT;  break;
      case    BIT_CODE: return HOR_BIT;    break;
      case U_CHAR_CODE: return HOR_U_CHAR; break;
      case    INT_CODE: return HOR_INT;    break;
      default: hor_error ( "illegal type character (image_type_from_code)",
			   HOR_FATAL); break;
   }

   return HOR_POINTER;
}

static int image_type_code ( Hor_Image_Type type )
{
   switch ( type )
   {
      case  HOR_FLOAT: return  FLOAT_CODE; break;
      case    HOR_BIT: return    BIT_CODE; break;
      case HOR_U_CHAR: return U_CHAR_CODE; break;
      case    HOR_INT: return    INT_CODE; break;
      default: hor_error ( "illegal image type (image_type_code)", HOR_FATAL );
	       break;
   }

   return -1;
}

/*******************
*   void @hor_read_image_data_from_stream ( int fd, Hor_Image *image )
*   void @hor_write_image_data_to_stream  ( int fd, Hor_Image *image )
*   void @hor_read_sub_image_data_from_stream ( int fd, Hor_Sub_Image *image )
*   void @hor_write_sub_image_data_to_stream  ( int fd, Hor_Sub_Image *image )
*   Hor_Image *@hor_read_image_from_stream ( int fd )
*   void       @hor_write_image_to_stream  ( int fd, Hor_Image *image )
*   Hor_Sub_Image *@hor_read_sub_image_from_stream ( int fd )
*   void           @hor_write_sub_image_to_stream  ( int fd,
*                                                   Hor_Sub_Image *sub_image )
*
*   (Sub-)image I/O functions for communicating image data between two
*   processes. Typical use is sending image data between root transputer
*   and host machine.
********************/
void hor_read_image_data_from_stream ( int fd, Hor_Image *image )
{
   int width = image->width, height = image->height;

   switch ( image->type )
   {
      int read_size, read_no;

      case HOR_FLOAT:
      read_size = width*height*sizeof(float);
      if ((read_no = hor_pipe_read (fd, (char *) image->array.f[0], read_size))
	   != read_size )
	 hor_error ( "read failed (hor_read_image_data_from_stream)",
		     HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.f[0], read_size );
#endif
      break;

      case HOR_INT:
      read_size = width*height*sizeof(int);
      if ((read_no = hor_pipe_read (fd, (char *) image->array.i[0], read_size))
	   != read_size )
	 hor_error ( "read failed (hor_read_image_data_from_stream)",
		     HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.i[0], read_size );
#endif
      break;

      case HOR_U_INT:
      read_size = width*height*sizeof(u_int);
      if((read_no = hor_pipe_read (fd, (char *) image->array.ui[0], read_size))
	   != read_size )
	 hor_error ( "read failed (hor_read_image_data_from_stream)",
		     HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.ui[0], read_size );
#endif
      break;

      case HOR_BIT:
      read_size = hor_bit_words(width)*height*sizeof(hor_bit);
      if((read_no = hor_pipe_read (fd, (char *) image->array.b[0], read_size))
	   != read_size )
	 hor_error ( "read failed (hor_read_image_data_from_stream)",
		     HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.b[0], read_size );
#endif
      break;

      case HOR_U_CHAR:
      read_size = width*height*sizeof(u_char);
      if ( hor_pipe_read ( fd, (char *) image->array.uc[0], read_size )
	   != read_size )
	 hor_error ( "read uc failed (hor_read_image_data_from_stream)",
		     HOR_FATAL );

      break;

      default:
      hor_error ( "illegal image type (hor_read_image_data_from_stream)",
		  HOR_FATAL );
      break;
   }
}

Hor_Image *hor_read_image_from_stream ( int fd )
{
   int            buf[3]; /* width, height, image type code */
   Hor_Image_Type type;
   Hor_Image     *image;
   int            read_size = 3*sizeof(int);

   if ( hor_pipe_read ( fd, (char *) buf, read_size ) != read_size )
      hor_error ( "read failed (hor_read_image_from_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) buf, 3*sizeof(int) );
#endif

   type = image_type_from_code ( buf[2] );
   image = hor_alloc_image ( buf[0], buf[1], type, NULL );
   hor_read_image_data_from_stream ( fd, image );
   return image;
}

Hor_Sub_Image *hor_read_sub_image_from_stream ( int fd )
{
   int            buf[5]; /* c0, r0, width, height, image type code */
   Hor_Image_Type type;
   Hor_Sub_Image *sub_image;
   int            read_size = 5*sizeof(int);

   if ( hor_pipe_read ( fd, (char *) buf, read_size ) != read_size )
      hor_error ( "read failed (hor_read_sub_image_from_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) buf, 5*sizeof(int) );
#endif
   type = image_type_from_code ( buf[4] );
   sub_image = hor_alloc_sub_image ( buf[0], buf[1], buf[2], buf[3], type,
				     NULL );
   hor_read_image_data_from_stream ( fd, &sub_image->image );
   return sub_image;
}

void hor_write_image_data_to_stream ( int fd, Hor_Image *image )
{
   int width = image->width, height = image->height;
   int write_size;

   switch ( image->type )
   {
      case HOR_FLOAT:
      write_size = width*height*sizeof(float);
#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.f[0], write_size );
#endif
      if ( write ( fd, (char *) image->array.f[0], write_size ) != write_size )
	 hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.f[0], write_size );
#endif
      break;

      case HOR_INT:
      write_size = width*height*sizeof(int);
#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.i[0], write_size );
#endif
      if ( write ( fd, (char *) image->array.i[0], write_size ) != write_size )
	 hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.i[0], write_size );
#endif
      break;

      case HOR_U_INT:
      write_size = width*height*sizeof(u_int);
#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.ui[0], write_size );
#endif
      if ( write ( fd, (char *) image->array.ui[0], write_size ) != write_size)
	 hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.ui[0], write_size );
#endif
      break;

      case HOR_BIT:
      write_size = hor_bit_words(width)*height*sizeof(hor_bit);
#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.f[0], write_size );
#endif
      if ( write ( fd, (char *) image->array.b[0], write_size ) != write_size)
	 hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

#ifndef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) image->array.f[0], write_size );
#endif
      break;

      case HOR_U_CHAR:
      write_size = width*height*sizeof(u_char);
      if ( write ( fd, (char *) image->array.uc[0], write_size ) != write_size)
	 hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

      break;

      default:
      hor_error ( "illegal image type (hor_write_image_to_stream)", HOR_FATAL);
      break;
   }
}

void hor_write_image_to_stream ( int fd, Hor_Image *image )
{
   int buf[3], write_size;

   buf[0] = image->width;
   buf[1] = image->height;
   buf[2] = image_type_code ( image->type );
   write_size = 3*sizeof(int);

#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) buf, 3*sizeof(int) );
#endif
   if ( write ( fd, (char *) buf, write_size ) != write_size )
      hor_error ( "write failed (hor_write_image_to_stream)", HOR_FATAL );

   hor_write_image_data_to_stream ( fd, image );
}

void hor_write_sub_image_to_stream ( int fd, Hor_Sub_Image *sub_image )
{
   int buf[2], write_size = 2*sizeof(int);

   buf[0] = sub_image->c0;
   buf[1] = sub_image->r0;
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) buf, 2*sizeof(int) );
#endif
   if ( write ( fd, (char *) buf, write_size ) != write_size )
      hor_error ( "write failed (hor_write_sub_image_to_stream)", HOR_FATAL );

   hor_write_image_to_stream ( fd, &sub_image->image );
}
