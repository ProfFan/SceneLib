/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#ifdef HOR_TRANSPUTER
#include <iocntrl.h>
#else
#include <fcntl.h>
#ifdef HOR_MSDOS
#include <sys/stat.h>
#include <io.h>
#else
#include <unistd.h>
#endif
#endif

#include "mit.h"

#include "horatio/global.h"
#include "horatio/image.h"

int read_mit_header();
int write_mit_header();

/** static functions **/

static void make_mit_header ( struct mithdr *hp,
			      u_short type,
			      u_short bits_per_pixel,
			      int     width,
			      int     height )
{
   hp->type           = type;
   hp->bits_per_pixel = bits_per_pixel;
   hp->width          = width;
   hp->height         = height;
}

/** these functions exported **/

/*******************
*   Hor_Image *@hor_read_mit_image_stream  ( int fd )
*   Hor_Bool   @hor_write_mit_image_stream ( int fd, Hor_Image *imptr, ... )
*   Hor_Image *@hor_read_mit_image  ( const char *file_name )
*   Hor_Bool   @hor_write_mit_image ( const char *file_name, Hor_Image *imptr,
*                                     ... )
*
*   MIT format image file I/O functions.
*
*   hor_read_mit_image_stream() reads and returns an MIT image read from the
*   given file stream. hor_read_mit_image() opens the given file and calls
*   hor_read_mit_image_stream().
*
*   hor_write_mit_image_stream() outputs the given image to a file stream.
*   Extra arguments are required to convert images of type HOR_FLOAT to
*   an unsigned char file. hor_write_mit_image() opens the given file and calls
*   hor_write_mit_image_stream().
********************/
Hor_Image *hor_read_mit_image_stream ( int fd )
{
   Hor_Image        *result;
   struct mithdr hdr;

   /* read MIT header from file */
   if ( read_mit_header ( fd, &hdr ) == -1 )
   {
      hor_errno = HOR_IMAGE_CANNOT_READ_IMAGE_HEADER;
      return NULL;
   }

   switch ( hdr.type )
   {
      int read_size;

      case MIT_SIGNED:
      switch ( hdr.bits_per_pixel )
      {
	 case 8:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_CHAR, NULL );
	 read_size = hdr.width*hdr.height*sizeof(char);
	 if ( hor_pipe_read ( fd, (char *) result->array.c[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
	 break;

	 case 16:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_SHORT, NULL );
	 read_size = hdr.width*hdr.height*sizeof(short);
	 if ( hor_pipe_read ( fd, (char *) result->array.s[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
#ifdef HOR_TRANSPUTER
	 hor_reverse_byte_order2 ( (char *) result->array.s[0], read_size );
#endif
	 break;

	 case 32:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_INT, NULL );
	 read_size = hdr.width*hdr.height*sizeof(int);
	 if ( hor_pipe_read ( fd, (char *) result->array.i[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
#ifdef HOR_TRANSPUTER
	 hor_reverse_byte_order4 ( (char *) result->array.i[0], read_size );
#endif
	 break;

	 default:
	 hor_errno = HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL;
#ifdef HOR_MSDOS
	 _close ( fd );
#else
	 close ( fd );
#endif
	 return NULL;
	 break;
      }
      break;

      case MIT_UNSIGNED:
      switch ( hdr.bits_per_pixel )
      {
	 case 8:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_U_CHAR, NULL );
	 read_size = hdr.width*hdr.height*sizeof(u_char);
	 if ( hor_pipe_read ( fd, (char *) result->array.uc[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
	 break;

	 case 16:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_U_SHORT, NULL );
	 read_size = hdr.width*hdr.height*sizeof(u_short);
	 if ( hor_pipe_read ( fd, (char *) result->array.us[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
#ifdef HOR_TRANSPUTER
	 hor_reverse_byte_order2 ( (char *) result->array.us[0], read_size );
#endif
	 break;

	 case 32:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_U_INT, NULL );
	 read_size = hdr.width*hdr.height*sizeof(u_int);
	 if ( hor_pipe_read ( fd, (char *) result->array.ui[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
#ifdef HOR_TRANSPUTER
	 hor_reverse_byte_order4 ( (char *) result->array.ui[0], read_size );
#endif
	 break;

	 default:
	 hor_errno = HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL;
#ifdef HOR_MSDOS
	 _close ( fd );
#else
	 close ( fd );
#endif
	 return NULL;
	 break;
      }
      break;

      case MIT_FLOAT:
      switch ( hdr.bits_per_pixel )
      {
	 case 32:
	 result = hor_alloc_image ( hdr.width, hdr.height, HOR_FLOAT, NULL );
	 read_size = hdr.width*hdr.height*sizeof(float);
	 if ( hor_pipe_read ( fd, (char *) result->array.f[0], read_size )
	      != read_size )
	 {
	    hor_free_image ( result );
	    return NULL;
	 }
#ifdef HOR_TRANSPUTER
	 hor_reverse_byte_order4 ( (char *) result->array.f[0], read_size );
#endif
	 break;

	 default:
	 hor_errno = HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL;
#ifdef HOR_MSDOS
	 _close ( fd );
#else
	 close ( fd );
#endif
	 return NULL;
	 break;
      }
      break;

      default:
      hor_errno = HOR_IMAGE_ILLEGAL_IMAGE_TYPE_IN_HEADER;
      return NULL;
      break;
   }

   return result;
}

Hor_Bool hor_write_mit_image_stream ( int fd, Hor_Image *imptr, ... )
{
   struct mithdr hdr;

   switch ( imptr->type )
   {
      int write_size;
#ifdef HOR_TRANSPUTER
      int write_no;
#endif
      case HOR_BIT:
      {
	 Hor_Image *byte_copy = hor_convert_image ( imptr, HOR_U_CHAR );

	 make_mit_header ( &hdr, MIT_UNSIGNED, 8,
			   byte_copy->width, byte_copy->height );
	 write_size = byte_copy->width*byte_copy->height*sizeof(u_char);
	 if ( write_mit_header ( fd, &hdr ) == -1 ||
	      write ( fd, (char *) byte_copy->array.uc[0], write_size )
		!= write_size )
	    goto write_error;
      }
      break;

      case HOR_CHAR:
      make_mit_header ( &hdr, MIT_SIGNED, 8, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(char);
      if ( write_mit_header ( fd, &hdr ) == -1 ||
	   write ( fd, (char *) imptr->array.c[0], write_size )
	      != write_size )
	 goto write_error;

      break;

      case HOR_U_CHAR:
      make_mit_header ( &hdr, MIT_UNSIGNED, 8, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(u_char);
      if ( write_mit_header ( fd, &hdr ) == -1 ||
	   write ( fd, (char *) imptr->array.uc[0], write_size )
	      != write_size )
	 goto write_error;

      break;

      case HOR_SHORT:
      make_mit_header ( &hdr, MIT_SIGNED, 16, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(short);
      if ( write_mit_header ( fd, &hdr ) == -1 ) goto write_error;
#ifdef HOR_TRANSPUTER
      hor_reverse_byte_order2 ( (char *) imptr->array.s[0], write_size );
      write_no = write ( fd, (char *) imptr->array.s[0], write_size );
      hor_reverse_byte_order2 ( (char *) imptr->array.s[0], write_size );
      if ( write_no != write_size ) goto write_error;
#else
      if ( write ( fd, (char *) imptr->array.s[0], write_size )
	   != write_size )
	 goto write_error;
#endif
      break;

      case HOR_U_SHORT:
      make_mit_header ( &hdr, MIT_UNSIGNED, 16, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(u_short);
      if ( write_mit_header ( fd, &hdr ) == -1 ) goto write_error;
#ifdef HOR_TRANSPUTER
      hor_reverse_byte_order2 ( (char *) imptr->array.us[0], write_size );
      write_no = write ( fd, (char *) imptr->array.us[0], write_size );
      hor_reverse_byte_order2 ( (char *) imptr->array.us[0], write_size );
      if ( write_no != write_size ) goto write_error;
#else
      if ( write ( fd, (char *) imptr->array.us[0], write_size )
	   != write_size )
	 goto write_error;
#endif
      break;

      case HOR_INT:
      make_mit_header ( &hdr, MIT_SIGNED, 32, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(int);
      if ( write_mit_header ( fd, &hdr ) == -1 ) goto write_error;
#ifdef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) imptr->array.i[0], write_size );
      write_no = write ( fd, (char *) imptr->array.i[0], write_size );
      hor_reverse_byte_order4 ( (char *) imptr->array.i[0], write_size );
      if ( write_no != write_size ) goto write_error;
#else
      if ( write ( fd, (char *) imptr->array.i[0], write_size )
	   != write_size )
	 goto write_error;
#endif
      break;

      case HOR_U_INT:
      make_mit_header ( &hdr, MIT_UNSIGNED, 32, imptr->width, imptr->height );
      write_size = imptr->width*imptr->height*sizeof(u_int);
      if ( write_mit_header ( fd, &hdr ) == -1 ) goto write_error;
#ifdef HOR_TRANSPUTER
      hor_reverse_byte_order4 ( (char *) imptr->array.ui[0], write_size );
      write_no = write ( fd, (char *) imptr->array.ui[0], write_size );
      hor_reverse_byte_order4 ( (char *) imptr->array.ui[0], write_size );
      if ( write_no != write_size ) goto write_error;
#else
      if ( write ( fd, (char *) imptr->array.ui[0], write_size )
	   != write_size )
	 goto write_error;
#endif
      break;

      case HOR_FLOAT: /* write as u_char image */
      {
	 va_list ap;
	 Hor_Image  *temp1, *temp2;
	 float   black_val, white_val;
	 Hor_Impixel number;

	 va_start ( ap, imptr );
	 black_val = (float) va_arg ( ap, double );
	 white_val = (float) va_arg ( ap, double );
	 va_end(ap);

	 if ( black_val == white_val )
	 {
	    hor_errno = HOR_IMAGE_ILLEGAL_PIXEL_SCALE;
	    return HOR_FALSE;
	    break;
	 }

	 temp1 = hor_copy_image ( imptr );
	 number.f = black_val;
	 hor_subtract_constant_from_image ( temp1, number );
	 number.f = 256.0F/(white_val-black_val);
	 hor_multiply_image_by_constant ( temp1, number );
	 temp2 = hor_convert_image ( temp1, HOR_U_CHAR );
	 make_mit_header ( &hdr, MIT_UNSIGNED, 8, temp2->width, temp2->height);
	 write_size = temp2->width*temp2->height*sizeof(u_char);
	 if ( write_mit_header ( fd, &hdr ) == -1 ||
	      write ( fd, (char *) temp2->array.uc[0], write_size )
		 != write_size )
	 {
	    hor_free_images ( temp2, temp1, NULL );
	    goto write_error;
	 }

	 hor_free_images ( temp2, temp1, NULL );
      }
      break;
	 
      write_error:
      hor_errno = HOR_IMAGE_WRITE_FAILED;
      return HOR_FALSE;
      break;

      default:
      hor_errno = HOR_IMAGE_WRONG_TYPE_IMAGE;
      return HOR_FALSE;
      break;
   }

   return HOR_TRUE;
}

Hor_Image *hor_read_mit_image ( const char *file_name )
{
   Hor_Image *result;
   int        fd;

   /* open MIT file */
#ifdef HOR_MSDOS
   if ( ( fd = _open ( (char *) file_name, _O_RDONLY ) ) == -1 )
#else
   if ( ( fd = open ( (char *) file_name, O_RDONLY ) ) == -1 )
#endif
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   result = hor_read_mit_image_stream ( fd );
#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_write_mit_image ( const char *file_name, Hor_Image *imptr, ... )
{
   int      fd = -1;
   Hor_Bool result;

#ifdef HOR_TRANSPUTER
   fd = open ( (char *) file_name, O_BINARY | O_WRONLY );
#else
#ifdef HOR_MSDOS
   fd = _open ( (char *) file_name, _O_BINARY | _O_CREAT |_O_WRONLY,
   		_S_IREAD | _S_IWRITE );
#else
   switch ( hor_get_image_compress() )
   {
      case HOR_NO_COMPRESS:
      fd = open ( (char *) file_name, O_CREAT | O_WRONLY, 0644 );
      break;

      case HOR_UNIX_COMPRESS:
      fd = hor_compress ( file_name );
      break;

      case HOR_GNU_COMPRESS:
      fd = hor_gzip ( file_name );
      break;

      default:
      hor_error ( "illegal compression mode (hor_write_mit_image)", HOR_FATAL);
   }
#endif
#endif
   if ( fd == -1 )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   if ( imptr->type == HOR_FLOAT )
   {
      va_list ap;
      float   black_val, white_val;

      va_start ( ap, imptr );
      black_val = (float) va_arg ( ap, double );
      white_val = (float) va_arg ( ap, double );
      va_end(ap);

      result = hor_write_mit_image_stream ( fd, imptr, black_val, white_val );
   }
   else
      result = hor_write_mit_image_stream ( fd, imptr );

#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}
