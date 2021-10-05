/* Copyright (C) 1992 RTCG Consortium */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("mult_width [-m/i] <scale> [<in base name> <out base name>]\n" );
hor_print("     m & i flags specify output image format: MIT (default) or IFF\n");
}

static void scan_command_line ( int          *argcp,
			        char       ***argvp,
			        Hor_Image_Format *image_format_ptr,
			        int          *scale )
{
   for ( ; *argcp > 2; (*argvp)++, (*argcp)-- )
   {
      if ( (*argvp)[1][0] != '-' ) break;

      switch ( (*argvp)[1][1] )
      {
         case 'm':
	 *image_format_ptr = HOR_MIT_FORMAT;
         break;

         case 'i':
	 *image_format_ptr = HOR_IFF_FORMAT;
         break;

	 case 'h':
         print_arguments();
         exit(0);
         break;

         default:
         print_arguments();
         hor_print("\n");
         error ( "illegal argument -%c", HOR_FATAL, (*argvp)[1][1] );
         break;
      }
   }

   sscanf ( (*argvp)[1], "%d", scale );
}

static Image *mult_width ( Image *image, int scale )
{
   int    new_width  = image->width*scale;
   Image *result;
   int    r, c;

   result = hor_alloc_image ( new_width, image->height, image->type, NULL );
   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char **imarr, *imp;
	 u_char **resarr, *resp, *resend;

	 imarr  = image->array.uc;
	 resarr = result->array.uc;
	 for ( r = 0; r < image->height; r++  )
	    for ( imp = imarr[r], resp = resarr[r],
		  resend = resp + new_width; resp != resend;
		  imp++, resp += scale )
	       for ( c = 0; c < scale; c++ ) resp[c] = *imp;
      }
      break;

      case Int:
      {
	 int **imarr, *imp;
	 int **resarr, *resp, *resend;

	 imarr  = image->array.i;
	 resarr = result->array.i;
	 for ( r = 0; r < image->height; r++  )
	    for ( imp = imarr[r], resp = resarr[r],
		  resend = resp + new_width; resp != resend;
		  imp++, resp += scale )
	       for ( c = 0; c < scale; c++ ) resp[c] = *imp;
      }
      break;

      default:
      error ( "illegal image type (mult_width)", HOR_FATAL );
      break;
   }

   return result;
}

void main ( int argc, char **argv )
{
   Image       *image;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;
   int          scale;

   scan_command_line ( &argc, &argv, &image_format, &scale );
   hor_set_image_format ( image_format );

   if ( argc != 2 && argc != 4 )
   {
      print_arguments();
      exit(0);
   }

   if ( argc == 2 )
      image = hor_read_image_stream ( 0 );
   else
      image = hor_read_image ( argv[2] );

   if ( image == NULL )
      error ( "cannot read input image", HOR_FATAL );

   if ( argc == 2 )
      hor_write_image_stream ( 1, mult_width ( image, scale ) );
   else
      hor_write_image ( argv[3], mult_width ( image, scale ) );
}
