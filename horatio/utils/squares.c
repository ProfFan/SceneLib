/* Copyright (C) 1992 RTCG Consortium */

/* makes image with central white circle on black background */

#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("squares [-m/i] <c0> <r0> [<base name>]\n" );
hor_print("      m & i flags specify output image format: MIT (default) or IFF\n");
}

static void scan_command_line ( int              *argcp,
			        char           ***argvp,
			        Hor_Image_Format *image_format_ptr )
{
   for ( ; *argcp > 3; (*argvp)++, (*argcp)-- )
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

static void fill_square ( Hor_Image *image, int c0, int r0,
			  int width, int height )
{
   u_char **arr = image->array.uc;
   int i, j;

   for ( i = r0; i < r0+height; i++ )
      for ( j = c0; j < c0+width; j++ )
	 arr[i][j] = 255;
}

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;
   int c0, r0;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 3 && argc != 4 )
   {
      print_arguments();
      exit(0);
   }

   image = hor_alloc_image ( 256, 256, HOR_U_CHAR, NULL );

   sscanf ( argv[1], "%d", &c0 );
   sscanf ( argv[2], "%d", &r0 );
   fill_square ( image, c0+103, r0+102, 30, 30 );
   fill_square ( image, c0+198, r0+108, 30, 30 );
   fill_square ( image, c0+152, r0+201,  5,  5 );
   if ( argc == 4 )
      hor_write_image ( argv[3], image );
   else /* argc == 1 so send to stdout */
      hor_write_image_stream ( 1, image );
}
