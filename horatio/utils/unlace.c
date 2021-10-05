/* Copyright (C) 1992 RTCG Consortium */

/* unlace.c: extracts fields of interlaced image and writes to separate files*/

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

#define VIDEO_SIZE 512

static void print_arguments(void)
{
hor_print("unlace [-m/i] <in base name> <out base 1> <out base 2>\n" );
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

static void unlace ( Hor_Image *image_in, Hor_Image *image_out1,
		                          Hor_Image *image_out2 )
{
   int      half_size = VIDEO_SIZE/2;
   int      rowi, rowo;
   u_char **arrayi  = image_in->array.uc;
   u_char **arrayo1 = image_out1->array.uc;
   u_char **arrayo2 = image_out2->array.uc;
   u_char  *pixi, *pixend, *pixo;

   for ( rowi = 0, rowo = 0; rowo < half_size; rowi += 2, rowo++ )
      for ( pixi = arrayi[rowi], pixend = arrayi[rowi] + VIDEO_SIZE,
	    pixo = arrayo1[rowo]; pixi != pixend; pixi++, pixo++ )
	 *pixo = *pixi;

   for ( rowi = 1, rowo = 0; rowo < half_size; rowi += 2, rowo++ )
      for ( pixi = arrayi[rowi], pixend = arrayi[rowi] + VIDEO_SIZE,
	    pixo = arrayo2[rowo]; pixi != pixend; pixi++, pixo++ )
	 *pixo = *pixi;
}

void main ( int argc, char **argv )
{
   Hor_Image       *image_in, *image_out1, *image_out2;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 4 )
   {
      print_arguments();
      exit(0);
   }

   image_in = hor_read_image ( argv[1] );

   if ( image_in == NULL )
      hor_error ( "could not read input image", HOR_FATAL );

   if ( image_in->type != HOR_U_CHAR )
      hor_error ( "input image must be unsigned char type", HOR_FATAL );

   if ( image_in->width != VIDEO_SIZE || image_in->height != VIDEO_SIZE )
      hor_error ( "input image wrong size", HOR_FATAL );

   image_out1 = hor_alloc_image ( VIDEO_SIZE, VIDEO_SIZE/2, HOR_U_CHAR, NULL );
   image_out2 = hor_alloc_image ( VIDEO_SIZE, VIDEO_SIZE/2, HOR_U_CHAR, NULL );
   unlace ( image_in, image_out1, image_out2 );

   hor_write_image ( argv[2], image_out1 );
   hor_write_image ( argv[3], image_out2 );
}
