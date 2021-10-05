/* Copyright (C) 1992 RTCG Consortium */

/* double_size.c: doubles size of image */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("double_size [-m/i] [<in base name> <out base name>]\n" );
hor_print("      m & i flags specify output image format: MIT (default) or IFF\n");
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

static Hor_Image *double_size ( Hor_Image *image )
{
   int        width  = image->width;
   int        height = image->height;
   Hor_Image *result;
   int        r;

   result = hor_alloc_image ( 2*width, 2*height, image->type, NULL );
   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char **imarr, *imp, *imend;
	 u_char **resarr, *resp1, *resp2;

	 imarr  = image->array.uc;
	 resarr = result->array.uc;
	 for ( r = 0; r < height; r++  )
	    for ( imp = imarr[r], imend = imp + width,
		  resp1 = resarr[r*2], resp2 = resarr[r*2+1]; imp != imend;
		  imp++, resp1 += 2, resp2 += 2 )
	       resp1[0] = resp1[1] = resp2[0] = resp2[1] = imp[0];
      }
      break;

      case HOR_INT:
      {
	 int **imarr, *imp, *imend;
	 int **resarr, *resp1, *resp2;

	 imarr  = image->array.i;
	 resarr = result->array.i;
	 for ( r = 0; r < height; r++  )
	    for ( imp = imarr[r], imend = imp + width,
		  resp1 = resarr[r*2], resp2 = resarr[r*2+1]; imp != imend;
		  imp++, resp1 += 2, resp2 += 2 )
	       resp1[0] = resp1[1] = resp2[0] = resp2[1] = imp[0];
      }
      break;

      case HOR_FLOAT:
      {
	 float **imarr, *imp, *imend;
	 float **resarr, *resp1, *resp2;

	 imarr  = image->array.f;
	 resarr = result->array.f;
	 for ( r = 0; r < height; r++  )
	    for ( imp = imarr[r], imend = imp + width,
		  resp1 = resarr[r*2], resp2 = resarr[r*2+1]; imp != imend;
		  imp++, resp1 += 2, resp2 += 2 )
	       resp1[0] = resp1[1] = resp2[0] = resp2[1] = imp[0];
      }
      break;

      default:
      hor_error ( "illegal image type (double_size)", HOR_FATAL );
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
      hor_write_image_stream ( 1, double_size ( image ) );
   else
      hor_write_image ( argv[2], double_size ( image ) );
}
