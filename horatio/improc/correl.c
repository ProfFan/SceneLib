/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   static void @correlate_patches (
*          u_char **arr1, u_char **arr2, (input image data)
*          int      c0, int r0,          (offsets of ROI from main image)
*          int      patch_size,          (size of patch)
*          int      max_motion,          (max. motion between frames in pixels)
*          float    display_scale,       (scaling factor for velocity vector)
*          int      small_size_2,        (2*small patch size)
*          int      col_patches,         (no. of large patches in x-direction)
*          int      row_patches,         (no. of large patches in y-direction)
*          int      offset )             (of top-left corner of 1st patch)
*
*   Performs correlation of individual image patches and displays "best-fit"
*   velocity vector.
********************/
static void correlate_patches ( u_char **arr1,
			        u_char **arr2,
			        int      c0, int r0,
			        int      patch_size,
			        int      max_motion,
			        float    display_scale,
			        int      small_size_2,
			        int      col_patches,
			        int      row_patches,
			        int      offset )
{
   int   pr, pc, c, r, motc, motr, c1, r1, c2, r2, total, lowest;
   int   bestc = 0, bestr = 0;
   float cf, rf;

   for ( pr = 0, r = offset + max_motion; pr < row_patches;
	 pr++, r += small_size_2 )
      for ( pc = 0, c = offset + max_motion; pc < col_patches;
	    pc++, c += small_size_2 )
      {
	 lowest = 0xfffffff;
	 for ( motr = -max_motion; motr <= max_motion; motr++ )
	    for ( motc = -max_motion; motc <= max_motion; motc++ )
	    {
	       total = 0;
	       for ( r1 = r, r2 = r + motr; r1 < r + patch_size; r1++, r2++ )
		  for ( c1 = c, c2 = c + motc; c1 < c + patch_size; c1++, c2++)
		     total += abs ( arr1[r1][c1] - arr2[r2][c2] );

	       if ( total < lowest )
	       { lowest = total; bestc = motc; bestr = motr; }
	    }

	 hor_display_fill_circle_actual_size ( c0 + c + patch_size/2,
					   r0 + r + patch_size/2, 2 );
	 cf = (float) (c0 + c + patch_size/2) + 0.5;
	 rf = (float) (r0 + r + patch_size/2) + 0.5;
	 hor_display_line ( cf, rf, cf + display_scale*((float) bestc),
			            rf + display_scale*((float) bestr) );
      }
}

/*******************
*   static void @correlate_images (
*          Hor_Image *old_image,          (previous image)
*          Hor_Image *new_image,          (current image)
*          int        c0, int r0,         (offsets of ROI from main image)
*          int        patch_size,         (size of large patch)
*          int        patch_density,      (density of large patches)
*          int        max_motion,         (max. vel. between frames in pixels)
*          float      display_scale,      (scaling factor for velocity vector)
*          u_long     display_colour,     (colour for velocity vector)
*          int        small_size,         (size of small patch)
*          int        small_size_2,       (2*small_size)
*          int        black_col_patches,
*          int        black_row_patches,  (no. of "black" and "white patches in
*          int        white_col_patches,   x & y directions)
*          int        white_row_patches )
*
*   Performs correlation first in "white" patches, then in "black" patches.
********************/
static void correlate_images ( Hor_Image *old_image,
			       Hor_Image *new_image,
			       int        c0, int r0,
			       int        patch_size,
			       int        patch_density,
			       int        max_motion,
			       float      display_scale,
			       u_long     display_colour,
			       int        small_size,
			       int        small_size_2,
			       int        black_col_patches,
			       int        black_row_patches,
			       int        white_col_patches,
			       int        white_row_patches )
{
   u_char **arr1 = old_image->array.uc;
   u_char **arr2 = new_image->array.uc;

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( display_colour );
   correlate_patches ( arr1, arr2, c0, r0, patch_size, max_motion,
		       display_scale, small_size_2,
		       black_col_patches, black_row_patches, 0 );
   correlate_patches ( arr1, arr2, c0, r0, patch_size, max_motion,
		       display_scale, small_size_2,
		       white_col_patches, white_row_patches, small_size );
}

/*******************
*   Hor_Correlation *@hor_correlation (
*          Hor_Image     *imptr,           (current image)
*          Hor_Sub_Image *old_image,       (previous image)
*          int            c1, int r1,      (top-left (1) and bottom-right
*          int            c2, int r2,       (2) corner of image region)
*          int            patch_size,      (size of large patch)
*          int            patch_density,   (density of large patches)
*          int            max_motion,      (max. vel. between frames in pixels)
*          float          display_scale,   (scaling factor for velocity vector)
*          u_long         display_colour ) (colour for velocity vector)
*
*   Performs patch-based image correlation between two images.
********************/
Hor_Correlation *hor_correlation ( Hor_Image     *imptr,
				   Hor_Sub_Image *old_image,
				   int            c1, int r1,
				   int            c2, int r2,
				   int            patch_size,
				   int            patch_density,
				   int            max_motion,
				   float          display_scale,
				   u_long         display_colour )
{
   Hor_Correlation *result;
   int              small_size = patch_size/patch_density, small_size_2;
   int              width, height;
   int              small_col_patches, small_row_patches;
   int              black_col_patches, black_row_patches;
   int              white_col_patches, white_row_patches;
   Hor_Sub_Image   *new_image;

   if ( imptr->type != HOR_U_CHAR )
      hor_error ( "illegal image type (hor_correlation)", HOR_FATAL );

   hor_adjust_region_for_border ( imptr->width, imptr->height,
			      max_motion, max_motion, max_motion, max_motion,
                              &c1, &r1, &c2, &r2 );
   result = hor_malloc_type ( Hor_Correlation );
   width  = c2 - c1;
   height = r2 - r1;
   small_size_2 = small_size*2;
   small_col_patches = 2*(width/small_size_2);
   small_row_patches = 2*(height/small_size_2);
   black_col_patches = (small_col_patches - patch_density)/2 + 1;
   black_row_patches = (small_row_patches - patch_density)/2 + 1;
   white_col_patches = (small_col_patches - patch_density)/2;
   white_row_patches = (small_row_patches - patch_density)/2;

   if ( white_col_patches <= 0 || white_row_patches <= 0 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      hor_free ( (void *) result );
      return NULL;
   }

   new_image = hor_extract_from_image ( imptr,
				        c1 - max_motion, r1 - max_motion,
				        width  + 2*max_motion,
				        height + 2*max_motion );
   result->image = new_image;
   if ( old_image == NULL )
   {
      hor_warning ( "no correlation results yet" );
      return result;
   }

   correlate_images ( &old_image->image, &new_image->image, c1, r1,
		      patch_size, patch_density, max_motion,
		      display_scale, display_colour,
		      small_size, small_size_2,
		      black_col_patches, black_row_patches,
		      white_col_patches, white_row_patches );
   return result;
}
