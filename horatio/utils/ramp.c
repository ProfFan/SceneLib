/* Copyright (C) 1992 RTCG Consortium */

/* ramp.c makes grey-level ramp image */

#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

#define RADIUS 40.0

static void print_arguments(void)
{
hor_print("ramp [-m/i] <size> <centre pix> <c-grad> <r-grad> [<file base name>]\n");
hor_print("     m & i flags specify output image format: MIT (default) or IFF\n");
}

static void scan_command_line ( int          *argcp,
			        char       ***argvp,
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

static void ramp ( Hor_Image *image, double centre_pix,
		   double c_grad, double r_grad )
{
   int      width  = image->width,  half_width;
   int      height = image->height, half_height;
   int      row, col, temp;
   u_char **arr;

   if ( image->type != HOR_U_CHAR )
      hor_error ( "wrong type image in circle()", HOR_FATAL );

   arr = image->array.uc;
   half_width  = width/2;
   half_height = height/2;
   for ( row = 0; row < height; row++ )
      for ( col = 0; col < width; col++ )
      {
	 temp = (int) (centre_pix + (double)(col-half_width)*c_grad
		                  + (double)(row-half_height)*r_grad + 0.5);
	 arr[row][col] = (temp >= 0 && temp < 256) ? temp :
	                 (temp < 0 ? 0 : 255);
      }
}

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   int              image_size;
   double           centre_pix, c_grad, r_grad;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 5 && argc != 6 )
   {
      print_arguments();
      exit(0);
   }

   sscanf ( argv[1], "%d", &image_size );
   sscanf ( argv[2], "%lf", &centre_pix );
   sscanf ( argv[3], "%lf", &c_grad );
   sscanf ( argv[4], "%lf", &r_grad );

   image = hor_alloc_image ( image_size, image_size, HOR_U_CHAR, NULL );
   ramp ( image, centre_pix, c_grad, r_grad );
   if ( argc == 5 )
      hor_write_image_stream ( 1, image );
   else
      hor_write_image ( argv[5], image );
}
