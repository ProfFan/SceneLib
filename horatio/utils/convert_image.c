/* Copyright (C) 1992 RTCG Consortium */

/* converts image to a different pixel type */

#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"

static void print_arguments(void)
{
hor_print("convert_image [-m/i] [-c<code>] [<in base name> <out base name>]\n" );
hor_print("          <code> is one of uc c us s ui i f corresponding to\n" );
hor_print("          u_char char u_short short u_int int float\n" );
hor_print("          u_char (unsigned char) is the default output format\n" );
hor_print("          m & i flags specify output image format: MIT (default) or IFF\n" );
}

static void scan_command_line ( int              *argcp,
			        char           ***argvp,
			        Hor_Image_Format *image_format_ptr,
			        Hor_Image_Type   *image_type_ptr )
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

	 case 'c':
	 switch ( (*argvp)[1][2] )
	 {
	    case 'u':
	    switch ( (*argvp)[1][3] )
	    {
	       case 'c': *image_type_ptr = HOR_U_CHAR; break;
	       case 's': *image_type_ptr = HOR_U_SHORT; break;
	       case 'i': *image_type_ptr = HOR_U_INT; break;
	       default:
	       print_arguments();
	       hor_print("\n");
	       hor_error ( "illegal option -cu%c", HOR_FATAL, (*argvp)[1][3] );
	       break;
	    }
	    break;

	    case 'c': *image_type_ptr = HOR_CHAR; break;
	    case 's': *image_type_ptr = HOR_SHORT; break;
	    case 'i': *image_type_ptr = HOR_INT; break;
	    case 'f': *image_type_ptr = HOR_FLOAT; break;
	    default:
	    print_arguments();
	    hor_print("\n");
	    hor_error ( "illegal option -c%c", HOR_FATAL, (*argvp)[1][2] );
	    break;
	 }
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
   Hor_Image_Format image_format = HOR_MIT_FORMAT;
   Hor_Image_Type   image_type = HOR_U_CHAR;

   scan_command_line ( &argc, &argv, &image_format, &image_type );
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
      hor_write_image_stream ( 1, hor_convert_image ( image, image_type ) );
   else
      hor_write_image ( argv[2],  hor_convert_image ( image, image_type ) );
}
