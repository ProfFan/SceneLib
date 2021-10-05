/* Copyright (C) 1992 RTCG Consortium */

/* makes image with central white circle on black background */

#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("circle [-m/i] <size> <radius> <offset x> <offset y> [<base name>]\n" );
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

static void fill_circle ( Hor_Image *image, double R,
			  double offset_c, double offset_r )
{
   int      width  = image->width,  half_width;
   int      height = image->height, half_height;
   int      row, col;
   u_char **arr;
   double   r = 1.0/sqrt(M_PI), r2, Rp, R_Rp_r, area;
   double   lower_sqr_thres, upper_sqr_thres;
   double   row_f, col_f, Rp2;

   if ( image->type != HOR_U_CHAR )
      hor_error ( "wrong type image in circle()", HOR_FATAL );

   lower_sqr_thres = (R-r)*(R-r);
   upper_sqr_thres = (R+r)*(R+r);
   r2 = r*r;

   arr = image->array.uc;
   half_width  = width/2;
   half_height = height/2;
   for ( row = 0, row_f = -(double)half_height + offset_r;
	 row < height; row++, row_f += 1.0 )
      for ( col = 0, col_f = -(double)half_width - offset_c;
	    col < width; col++, col_f += 1.0 )
      {
	 Rp2 = col_f*col_f + row_f*row_f;
	 if ( Rp2 <= lower_sqr_thres )
	 { arr[row][col] = 255; continue; }

	 if ( Rp2 >= upper_sqr_thres )
	 { arr[row][col] = 0; continue; }

	 /* calculate area of section of circle of width r centred on
	    (c-half_width,r-half_height) that is inside circle radius R
	    centred on origin (half_width,half_height). Assumes R >> r so that
	    section is linear. */
	 Rp = sqrt ( Rp2 );
	 R_Rp_r = (R - Rp)/r;
	 area = r2*(M_PI_2 + asin(R_Rp_r) + R_Rp_r*sqrt(1.0 - R_Rp_r*R_Rp_r));
	 arr[row][col] = (u_char) (256.0*area);
      }
}

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   int              size;
   double           radius, offset_c, offset_r;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 5 && argc != 6 )
   {
      print_arguments();
      exit(0);
   }

   sscanf ( argv[1], "%d",  &size );
   sscanf ( argv[2], "%lf", &radius );
   sscanf ( argv[3], "%lf", &offset_c );
   sscanf ( argv[4], "%lf", &offset_r );

   image = hor_alloc_image ( size, size, HOR_U_CHAR, NULL );

   fill_circle ( image, radius, offset_c, offset_r );
   if ( argc == 6 )
      hor_write_image ( argv[5], image );
   else /* argc == 5 so send to stdout */
      hor_write_image_stream ( 1, image );
}
