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

typedef struct
{
   float Sxx, Sxy, Sx, SIx;
   float      Syy, Sy, SIy;
   float           S,  SI;
   float SII;
} Sums;

typedef struct
{
   float a, b, c; /* define linear image fit */
   float rms_residual;
   float mean;
   float st_dev;
} Stats;

static Hor_Image *calc_sums ( Hor_Image *image, int c1, int r1,
			      int small_size,
			      int col_patches, int row_patches )
{
   Hor_Image *result;
   void    ***rarr;
   Sums      *sums;
   u_char   **imarr = image->array.uc;
   int        rp, cp, r, c, i, j;
   float      x, y, start_xy = 0.5 - (float)small_size*0.5, I;

   result = hor_alloc_image ( col_patches, row_patches, HOR_POINTER,
			      hor_free_func );
   rarr = result->array.p;
   for ( rp = 0, r = r1; rp < row_patches; rp++, r += small_size )
      for ( cp = 0, c = c1; cp < col_patches; cp++, c += small_size )
      {
	 sums = hor_malloc_type ( Sums );
	 rarr[rp][cp] = (void *) sums;
	 sums->Sxx = sums->Sxy = sums->Sx = sums->SIx = sums->Syy =
	 sums->Sy  = sums->SIy = sums->S  = sums->SI  = sums->SII = 0.0;
	 for ( i = 0, y = start_xy; i < small_size; i++, y += 1.0 )
	    for ( j = 0, x = start_xy; j < small_size; j++, x += 1.0 )
	    {
	       I = (float) imarr[r+i][c+j];
	       sums->Sxx += x*x; sums->Sxy += x*y; sums->Syy += y*y;
	       sums->S   += 1.0; sums->Sx  += x;   sums->Sy  += y;
	       sums->SI  += I;   sums->SIx += I*x; sums->SIy += I*y;
	       sums->SII += I*I;
	    }
      }

   return result;
}

static Hor_Image *calc_stats ( Hor_Image *sums, int patch_density,
			       int small_size,
			       int col_patches, int row_patches, int offset )
{
   void     ***rarr, ***sarr = sums->array.p;
   Hor_Image  *result;
   int         cp, rp, c, r, i, j;
   Stats      *new_stats;
   float       tot_Sxx, tot_Sxy, tot_Sx, tot_SIx;
   float                tot_Syy, tot_Sy, tot_SIy;
   float                         tot_S,  tot_SI;
   float       tot_SII;
   Sums       *sptr;
   Hor_Matrix *Mmat, *LUDmat, *xmat, *ymat, *zmat;
   double    **M, **x, **y, **LUD;
   float       start_xy = -0.5*(float)(1 - patch_density)*(float)small_size;
   float       x0, y0, step_xy = (float) small_size;
   float       A, B, C, E;

   Mmat   = hor_mat_alloc ( 3, 3 ); M   = Mmat->m;
   LUDmat = hor_mat_alloc ( 3, 3 ); LUD = LUDmat->m;
   xmat   = hor_mat_alloc ( 3, 1 ); x   = xmat->m;
   ymat   = hor_mat_alloc ( 3, 1 ); y   = ymat->m;
   zmat   = hor_mat_alloc ( 3, 1 );

   result = hor_alloc_image (col_patches, row_patches, HOR_POINTER, hor_free_func);
   rarr = result->array.p;
   for ( rp = 0, r = offset; rp < row_patches; rp++, r += 2 )
      for ( cp = 0, c = offset; cp < col_patches; cp++, c += 2 )
      {
	 new_stats = hor_malloc_type ( Stats );
	 rarr[rp][cp] = (void *) new_stats;
	 tot_Sxx = tot_Sxy = tot_Sx = tot_SIx = tot_Syy =
	 tot_Sy  = tot_SIy = tot_S  = tot_SI  = tot_SII = 0.0;
	 for ( i = 0, y0 = start_xy; i < patch_density; i++, y0 += step_xy )
	    for ( j = 0, x0 = start_xy; j < patch_density; j++, x0 += step_xy )
	    {
	       sptr = (Sums *) sarr[r+i][c+j];
	       tot_S  += sptr->S;
	       tot_Sx += sptr->Sx + x0*sptr->S;
	       tot_Sy += sptr->Sy + y0*sptr->S;
	       tot_Sxy += sptr->Sxy + x0*sptr->Sy + y0*sptr->Sx +x0*y0*sptr->S;
	       tot_Sxx += sptr->Sxx + 2.0*x0*sptr->Sx + x0*x0*sptr->S;
	       tot_Syy += sptr->Syy + 2.0*y0*sptr->Sy + y0*y0*sptr->S;
	       tot_SI  += sptr->SI;
	       tot_SIx += sptr->SIx + x0*sptr->SI;
	       tot_SIy += sptr->SIy + y0*sptr->SI;
	       tot_SII += sptr->SII;
	    }

	 /* fill in matrix and vector entries */
	 M[0][0] = tot_Sxx; M[0][1] = tot_Sxy; M[0][2] = tot_Sx;
	 M[1][0] = tot_Sxy; M[1][1] = tot_Syy; M[1][2] = tot_Sy;
	 M[2][0] = tot_Sx;  M[2][1] = tot_Sy;  M[2][2] = tot_S;

	 y[0][0] = tot_SIx; y[1][0] = tot_SIy; y[2][0] = tot_SI;

	 /* perform lower-upper decomposition */
	 hor_matq_lud ( Mmat, LUDmat );

	 /* solve for intermediate vector z */
	 hor_matq_solve_lower ( LUDmat, zmat, ymat );

	 /* solve for solution vector x */
	 hor_matq_solve_upper ( LUDmat, xmat, zmat );

	 /* read off solution vector x */
	 new_stats->a = A = (float) x[0][0];
	 new_stats->b = B = (float) x[1][0];
	 new_stats->c = C = (float) x[2][0];

	 /* calculate sum-of-squares of error distances */
	 E = A*A*tot_Sxx      + B*B*tot_Syy + C*C*tot_S +
	     2.0*(A*B*tot_Sxy + A*C*tot_Sx  + B*C*tot_Sy -
		  A*tot_SIx   - B*tot_SIy   - C*tot_SI) + tot_SII;
	 new_stats->rms_residual = sqrt(E/tot_S);

	 new_stats->mean = tot_SI/tot_S;
	 new_stats->st_dev = sqrt((tot_SII - tot_SI*tot_SI/tot_S)/tot_S);
      }

   hor_mat_free_list ( zmat, ymat, xmat, LUDmat, Mmat, NULL );
   return result;
}

static void show_variances ( int        c1,
			     int        r1,
			     int        patch_density,
			     int        small_size,
			     int        small_size_2,
			     Hor_Image *stats,
			     int        offset,
			     float      sd_scale )
{
   void ***sarr = stats->array.p;
   int     row_patches = stats->height, rp, r;
   int     col_patches = stats->width,  cp, c;
   Stats  *stats_ptr;
   int     grey_level;
   float   diamond_half_size = (float) small_size;

   hor_display_set_function ( HOR_DISPLAY_COPY );
   for ( rp = 0, r = r1 + (offset + patch_density/2)*small_size;
	 rp < row_patches; rp++, r += small_size_2 )
      for ( cp = 0, c = c1 + (offset + patch_density/2)*small_size;
	    cp < col_patches; cp++, c += small_size_2 )
      {
	 stats_ptr = (Stats *) sarr[rp][cp];
	 grey_level = (int) ((stats_ptr->st_dev/stats_ptr->mean)*sd_scale);
	 if ( grey_level < 0 )   grey_level = 0;
	 if ( grey_level > 255 ) grey_level = 255;
	 hor_display_set_colour ( Hor_Grey[grey_level] );
	 hor_display_fill_diamond ( c, r, diamond_half_size );
	 hor_display_flush();
      }
}

static void show_scaled_variance ( Hor_Image *black_stats,
				   Hor_Image *white_stats,
				   int    c1,
				   int    r1,
				   int    patch_density,
				   int    small_size,
				   int    small_size_2,
				   float  sd_scale )
{
   Hor_Assoc_Label display_label = hor_display_store_state();

   show_variances ( c1, r1, patch_density, small_size, small_size_2,
		    black_stats, 0, sd_scale );
   show_variances ( c1, r1, patch_density, small_size, small_size_2,
		    white_stats, 1, sd_scale );

   hor_wait_for_keyboard();
   hor_display_recall_state ( display_label );
   hor_display_destroy_state ( display_label );
}

static void show_thres_variances ( int    c1,
				   int    r1,
				   int    patch_density,
				   int    small_size,
				   int    small_size_2,
				   Hor_Image *stats,
				   int        offset,
				   float      threshold )
{
   void ***sarr = stats->array.p;
   int     row_patches = stats->height, rp, r;
   int     col_patches = stats->width,  cp, c;
   Stats  *stats_ptr;
   float   diamond_half_size = (float) small_size*0.5;

   for ( rp = 0, r = r1 + (offset + patch_density/2)*small_size;
	 rp < row_patches; rp++, r += small_size_2 )
      for ( cp = 0, c = c1 + (offset + patch_density/2)*small_size;
	    cp < col_patches; cp++, c += small_size_2 )
      {
	 stats_ptr = (Stats *) sarr[rp][cp];
	 if ( stats_ptr->st_dev/stats_ptr->mean < threshold )
	    hor_display_fill_diamond ( c, r, diamond_half_size );
      }
}

static void show_thresholded_variances ( Hor_Image *black_stats,
					 Hor_Image *white_stats,
					 int    c1,
					 int    r1,
					 int    patch_density,
					 int    small_size,
					 int    small_size_2,
					 float  threshold,
					 u_long thres_colour )
{
   Hor_Assoc_Label display_label = hor_display_store_state();

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( thres_colour );
   show_thres_variances ( c1, r1, patch_density, small_size, small_size_2,
			  black_stats, 0, threshold );
   show_thres_variances ( c1, r1, patch_density, small_size, small_size_2,
			  white_stats, 1, threshold );
   hor_display_flush();

   hor_wait_for_keyboard();
   hor_display_recall_state ( display_label );
   hor_display_destroy_state ( display_label );
}

static void threshold_variances ( Hor_Image *black_stats,
				  Hor_Image *white_stats,
				  float  low_threshold,
				  float  threshold_step,
				  float  high_threshold,
				  int    c1,
				  int    r1,
				  int    patch_density,
				  int    small_size,
				  int    small_size_2,
				  u_long thres_colour )
{
   float threshold;

   for ( threshold = low_threshold; threshold < high_threshold;
	 threshold *= threshold_step )
      show_thresholded_variances ( black_stats, white_stats,
				   c1, r1, patch_density,
				   small_size, small_size_2, threshold,
				   thres_colour );
}

/*******************
*   Hor_Image_Segment *@hor_image_segment (
*         Hor_Image *image,          (input image)
*         int    c1, int r1,     (top-left (1) and bottom-right (2) corners
*         int    c2, int r2,      of region-of-interest in image)
*         int    patch_size,     (size of image patch of initial labelling)
*         int    patch_density,  (density of image patches)
*         float  low_threshold,  (define values of difference threshold
*         float  threshold_step,  over which results
*         float  high_threshold,  are displayed)
*         float  sd_scale,       (scaling factor for showing std. deviations)
*         u_long thres_colour )  (colour for showing thresholded variances)
*
*   Performs region-based image segmentation.
********************/
Hor_Image_Segment *hor_image_segment ( Hor_Image *image,
				       int    c1, int r1,
				       int    c2, int r2,
				       int    patch_size,
				       int    patch_density,
				       float  low_threshold,
				       float  threshold_step,
				       float  high_threshold,
				       float  sd_scale,
				       u_long thres_colour )
{
   Hor_Image_Segment *result;
   int                small_size = patch_size/patch_density, small_size_2;
   int                width, height;
   int                small_col_patches, small_row_patches;
   int                black_col_patches, black_row_patches;
   int                white_col_patches, white_row_patches;
   Hor_Image         *sums;
   Hor_Image         *black_stats, *white_stats;

   result = hor_malloc_type ( Hor_Image_Segment );
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

   sums = calc_sums ( image, c1, r1, small_size,
		      small_col_patches, small_row_patches );
   black_stats = calc_stats ( sums, patch_density, small_size,
			      black_col_patches, black_row_patches, 0 );
   white_stats = calc_stats ( sums, patch_density, small_size,
			      white_col_patches, white_row_patches, 1 );

   show_scaled_variance ( black_stats, white_stats,
			  c1, r1, patch_density, small_size, small_size_2,
			  sd_scale );
   threshold_variances ( black_stats, white_stats,
			 low_threshold, threshold_step, high_threshold,
			 c1, r1, patch_density, small_size, small_size_2,
			 thres_colour );

   hor_free_images ( white_stats, black_stats, sums, NULL );
   return result;
}
