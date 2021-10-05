/* Copyright (C) 1992 RTCG Consortium */

/* extracts rectangular region from image */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("extract [-m/i] <base name> <c0> <r0> <width> <height> [<output name>]\n");
hor_print("        m & i flags specify output image format: MIT (default) or IFF\n");
}

static void scan_command_line ( int *argcp, char ***argvp,
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

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   Hor_Sub_Image   *sub_image;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;
   int              c0, r0, width, height;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 6 && argc != 7 )
   {
      print_arguments();
      exit(0);
   }

   image = hor_read_image ( argv[1] );
   sscanf ( argv[2], "%d", &c0 );
   sscanf ( argv[3], "%d", &r0 );
   sscanf ( argv[4], "%d", &width );
   sscanf ( argv[5], "%d", &height );
   if ( image == NULL )
      hor_error ( "cannot read input image file", HOR_FATAL );

   sub_image = hor_extract_from_image ( image, c0, r0, width, height );
   if ( sub_image == NULL )
      hor_perror ( "extract" );
   else
      if ( argc == 6 )
	 hor_write_image_stream ( 1, &sub_image->image );
      else
	 hor_write_image ( argv[6], &sub_image->image );
}
