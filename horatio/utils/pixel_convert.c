/* Copyright (C) 1992 RTCG Consortium */

/* converts all occurences of a given pixel grey-level value to another */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("pixel_convert [-m/i] <in base name> <from val> <to val>\n" );
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


static void pixel_convert ( Hor_Image *image, int from_val, int to_val )
{
   int size = image->width*image->height;

   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char  from_pix = (u_char) from_val, to_pix = (u_char) to_val;
	 u_char *imend = image->array.uc[0] + size, *imp;

	 for ( imp = image->array.uc[0]; imp != imend; imp++ )
	    if ( *imp == from_pix )
	       *imp = to_pix;
      }
      break;

      default:
      hor_error ( "illegal image type (pixel_convert)", HOR_FATAL );
      break;
   }
}

void main ( int argc, char **argv )
{
   Hor_Image       *image;
   Hor_Image_Format image_format = HOR_MIT_FORMAT;
   int              from_val, to_val;

   scan_command_line ( &argc, &argv, &image_format );
   hor_set_image_format ( image_format );

   if ( argc != 4 )
   {
      print_arguments();
      exit(0);
   }

   sscanf ( argv[2], "%d", &from_val );
   sscanf ( argv[3], "%d", &to_val );
   image = hor_read_image ( argv[1] );
   if ( image == NULL )
      hor_error ( "cannot read input image file", HOR_FATAL );

   pixel_convert ( image, from_val, to_val );
   hor_write_image ( argv[1], image );
}
