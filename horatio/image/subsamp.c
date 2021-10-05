/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   Hor_Image *@hor_subsample ( Hor_Image *image, int ratio )
*
*   Subsamples image by given ratio along each direction and returns result.
********************/
Hor_Image *hor_subsample ( Hor_Image *image, int ratio )
{
   int        new_width, new_height, ratio_2 = ratio*ratio, i, j, c, r, sum;
   Hor_Image *result;

   if ( image == NULL )
   {
      hor_errno = HOR_IMAGE_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( image->type == HOR_POINTER )
      hor_error ( "cannot hor_subsample pointer type image (hor_subsample)",
		  HOR_FATAL );

   if ( ratio < 1 )
      hor_error ( "illegal subsampling ratio (hor_subsample)", HOR_FATAL );

   if ( (ratio > image->width) || (ratio > image->height) )
   {
      hor_errno = HOR_IMAGE_IMAGE_TOO_SMALL;
      return NULL;
   }

   if ( (image->width % ratio != 0) || (image->height % ratio != 0) )
      hor_error ( "image size must divide subsampling ratio (hor_subsample)",
		  HOR_FATAL );

   new_width  = image->width/ratio;
   new_height = image->height/ratio;

   hor_message ( "hor_subsample: ratio %d, new height %d, width %d\n", ratio,
	     new_height, new_width );

   result = hor_alloc_image ( new_width, new_height, image->type, NULL );
   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char  **imarr, **imp;
	 u_char  **resarr, *resp;

	 imp = hor_malloc_ntype ( u_char *, ratio );
	 imarr  = image->array.uc;
	 resarr = result->array.uc;
	 for ( r = 0; r < new_height; r++  )
	 {
	     for ( i = 0; i < ratio; i++ )
		 imp[i] = imarr[(r*ratio) + i];

	     for ( c = 0, resp = resarr[r] ; c < new_width; c++, resp++ )
	     {
		 sum = 0;
		 for ( i = 0 ; i < ratio ; i++ )
		     for ( j = 0; j < ratio ; j++ )
			 sum += (int) imp[i][j];

		 *resp = (u_char) (sum/ratio_2);

		 for ( i = 0; i < ratio; i++ )
		    imp[i] += ratio;
	     }
	 }

	 hor_free ( (void *) imp );
      }
      break;

      default:
      hor_error ( "illegal image type (hor_subsample)", HOR_FATAL );
      break;
   }

   return result;
}
