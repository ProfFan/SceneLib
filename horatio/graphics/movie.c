/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

#include "ximage.h"

/*******************
*   Hor_List @hor_display_make_movie ( const char *root_name, ... )
*
*   Reads an image sequence in standard file name format root_name??.suffix
*   or root_name.???.suffix, where ??/??? is a two/three-digit number,
*   starting with zero, and suffix is either not present or .mit/.iff/etc.
*   Sets the canvas parameters to prepare to display the images and returns
*   a list of X-format images.
*
*   The variable argument list ... should contain two thresholds of type double
*   if the image to be displayed is of type HOR_INT or HOR_FLOAT. These
*   threshold are used to convert the image into X internal format, and denote
*   image values that will appear black and white on the canvas.
********************/
Hor_List hor_display_make_movie ( const char *root_name, ... )
{
   char       string[100];
   int        image_count;
   Hor_List   movie = NULL;
   Hor_Image *imptr;
   int        init_width = 0, init_height = 0; /* to be checked with every
						  image in sequence */
   Hor_Image_Type init_type = HOR_POINTER;
   va_list    ap;

   for ( image_count = 0;; image_count++ )
   {
      if ( (imptr = hor_read_next_image ( root_name, image_count,
					  HOR_NON_FATAL )) == NULL )
	 break;

      if ( image_count == 0 )
      {
	 init_width  = imptr->width;
	 init_height = imptr->height;
	 init_type   = imptr->type;

	 if ( !hor_display_set_params ( imptr->width, imptr->height ) )
	 {
	    hor_free_image ( imptr );
	    break;
	 }
      }
      else
	 if ( imptr->width != init_width || imptr->height != init_height ||
	      imptr->type  != init_type )
	 {
	    hor_warning ( "incompatible images: movie terminated" );
	    hor_free_image ( imptr );
	    break;
	 }

      va_start ( ap, root_name );
      movie = hor_insert ( movie, (void *)
		 hor_convert_image_to_X_format(imptr, display, scrnum, depth,
					       1, 1, pixel_size_factor, &ap ));
      va_end(ap);

      if ( hor_node_contents(movie) == NULL )
      {
	 hor_warning ( "image %d cannot be displayed: movie terminated",
		   image_count );
	 hor_free_image ( imptr );
	 movie = hor_delete_first ( &movie, NULL );
	 break;
      }
      
      hor_free_image ( imptr );
   }

   switch ( image_count )
   {
      case 0:  hor_message ( "zero length movie" ); return NULL; break;
      case 1:  hor_message ( "one image: not much of a movie" ); break;
      case 2:  hor_message ( "two images: pretty pathetic movie" ); break;
      default: sprintf ( string, "movie consists of %d images", image_count );
	       hor_message ( string ); break;
   }

   hor_display_clear ( image_background_colour );
   return ( hor_reverse ( movie ) );
}

/*******************
*   void @hor_display_make_movie_image ( Hor_Image *image, ... )
*
*   Makes and returns an X-format image from an Horatio format image.
*   Assumes that the canvas parameters have been set to the size of the image.
*   The variable argument list ... should contain two thresholds of type double
*   if the image to be displayed is of type HOR_INT or HOR_FLOAT. These
*   thresholds are used to convert the image into X internal format, and denote
*   image values that will appear black and white on the canvas.
********************/
XImage *hor_display_make_movie_image ( Hor_Image *image, ... )
{
   XImage *ximage;
   va_list ap;

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return NULL;
   }

   if ( image->width != user_width || image->height != user_height )
   {
      hor_errno = HOR_GRAPHICS_INCOMPATIBLE_IMAGE;
      return NULL;
   }

   va_start ( ap, image );
   ximage = hor_convert_image_to_X_format ( image, display, scrnum, depth,
				        1, 1, pixel_size_factor, &ap );
   va_end(ap);
   return ximage;
}


/*******************
*   Hor_List hor_make_movie_from_images ( Hor_List image_sequence )
*
*   Makes a movie from a list of (Hor_Image *)'s.
********************/
Hor_List hor_make_movie_from_images ( Hor_List image_sequence )
{
   Hor_List  Xsequence = NULL, list_ptr;
   Hor_Image *imptr;

   imptr = hor_node_contents (image_sequence);
   hor_display_set_params(imptr->width, imptr->height);

   for (list_ptr = image_sequence; list_ptr != NULL; list_ptr = list_ptr->next)
   { 
      imptr = (Hor_Image *) list_ptr;
      Xsequence = hor_insert (Xsequence, 
			      (void *) hor_display_make_movie_image (imptr));
   }

   return (hor_reverse (Xsequence));
}

/*******************
*   void @hor_display_show_movie_image ( void *data )
*
*   Displays a single X-format image on the current window.
*   Assumes the display parameters are still those set by
*   hor_display_make_movie().
********************/
void hor_display_show_movie_image ( XImage *ximage )
{
   hor_display_X_format_image ( ximage, display, canvas_window,
			        graphics_context,
			        canvas_top_left_c, canvas_top_left_r );
}

static void destroy_X_image ( void *data )
{
   XDestroyImage ( (XImage *) data );
}

/*******************
*   void @hor_display_destroy_movie ( Hor_List movie )
*
*   Destroys a movie created by hor_display_make_movie, and restores original
*   state of display.
********************/
void hor_display_destroy_movie ( Hor_List movie )
{
   hor_free_list ( movie, destroy_X_image );
/*   (yzpwj1 = *(((int *)(movie[image_count]->data))-2), yzpwj2 = *(((int *)(movie[image_count]->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, movie[image_count]->data, hor_test_free(), yzpwj1, yzpwj2, 

__LINE__, __FILE__ ),yzpwj3);*/
}
