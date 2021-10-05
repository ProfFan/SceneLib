/* Copyright (C) 1992 RTCG Consortium */

/* halve_width.c: halves width of image */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("halve_width [-m/i] [<in base name> <out base name>]\n" );
hor_print("      m & i flags specify output image format: MIT (default) or IFF\n");
}

static void scan_command_line ( int              *argcp,
			        char           ***argvp,
			        Hor_Image_Format *image_format_ptr )
{
   for ( ; *argcp > 1; (*argvp)++, (*argcp)-- )
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
         hor_error ( "illegal argument -%c", HOR_FATAL, (*argvp)[1][1] );
         break;
      }
   }
}

static Hor_Image *halve_width ( Hor_Image *image )
{
   int        half_width  = image->width/2;
   Hor_Image *result;
   int        r;

   if ( half_width*2 != image->width )
      hor_error ( "image width not even", HOR_FATAL );

   result = hor_alloc_image ( half_width, image->height, image->type, NULL );
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
		  resend = resp + half_width; resp != resend;
		  imp += 2, resp++ )
	       *resp = (u_char) (((int)imp[0] + (int)imp[1])/2);
      }
      break;

      case HOR_INT:
      {
	 int **imarr, *imp;
	 int **resarr, *resp, *resend;

	 imarr  = image->array.i;
	 resarr = result->array.i;
	 for ( r = 0; r < image->height; r++  )
	    for ( imp = imarr[r], resp = resarr[r],
		  resend = resp + half_width; resp != resend;
		  imp += 2, resp++ )
	       *resp = (imp[0] + imp[1])/2;
      }
      break;

      default:
      hor_error ( "illegal image type (halve_width)", HOR_FATAL );
      break;
   }

   return result;
}

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 1 && argc != 3 )
   {
      print_arguments();
      exit(0);
   }

   if ( argc == 1 )
      image = hor_read_image_stream ( 0 );
   else
      image = hor_read_image ( argv[1] );

   if ( image == NULL )
      hor_error ( "cannot read input image", HOR_FATAL );

   if ( argc == 1 )
      hor_write_image_stream ( 1, halve_width ( image ) );
   else
      hor_write_image ( argv[2], halve_width ( image ) );
}
