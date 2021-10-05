/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   static void @image_derivative ( Hor_Sub_Image *old_image,
*                                  Hor_Sub_Image *new_image,
*                                  Hor_Sub_Image *deriv_c,
*                                  Hor_Sub_Image *deriv_r,
*                                  Hor_Sub_Image *deriv_t )
*
*   Takes input sub-images and calculates x-derivative and y-derivative
*   of their average, and t-derivatives as their difference, writing the
*   results into deriv_c, deriv_r and deriv_t.
********************/
static void image_derivative ( Hor_Sub_Image *old_image,
			       Hor_Sub_Image *new_image,
			       Hor_Sub_Image *deriv_c,
			       Hor_Sub_Image *deriv_r,
			       Hor_Sub_Image *deriv_t )
{
   Hor_Image  *image;
   Hor_Image  *dc_image = &(deriv_c->image);
   Hor_Image  *dr_image = &(deriv_r->image);
   Hor_Image  *dt_image = &(deriv_t->image);
   float **imarr, **oldarr, **newarr, **dcarr, **drarr, **dtarr;
   float  *pimp, *nimp, *imcol, *imend, *oldp, *newp, *dcp, *dtp, *dtend;
   int     width, height, big_height, big_width;
   int     r, c;

   big_width  = old_image->image.width;
   big_height = old_image->image.height;
   width  = big_width  - 2;
   height = big_height - 2;

   /* average old and new images */
   image = hor_average_images ( &(old_image->image), &(new_image->image) );

   dcarr = dc_image->array.f;
   drarr = dr_image->array.f;
   dtarr = dt_image->array.f;

   imarr = hor_malloc_ntype ( float *, big_height );
   for ( r = 0; r < big_height; r++ )
      imarr[r] = image->array.f[r] + 1;

   imarr++; /* now imarr points to same positions on image as dcarr, drarr
	       and dtarr */

   /* do c-derivative */
   for ( r = 0; r < height; r++ )
      for ( imend = imarr[r] + width + 1, pimp = imarr[r]-1, nimp = imarr[r]+1,
	    dcp =  dcarr[r]; nimp != imend; pimp++, nimp++, dcp++ )
	 *dcp = 0.5F*(*nimp - *pimp);

   /* do r-derivative */
   imcol = hor_malloc_ntype ( float, big_height );
   imcol++;
   for ( c = 0; c < width; c++ )
   {
      /* set up column in image */
      for ( r = -1; r <= height; r++ )
	 imcol[r] = imarr[r][c];

      for ( r = 0, pimp = imcol-1, nimp = imcol+1; r < height;
	    r++, pimp++, nimp++ )
	 drarr[r][c] = 0.5F*(*nimp - *pimp);
   }

   /* calculate t-derivative */
   oldarr = old_image->image.array.f;
   newarr = new_image->image.array.f;
   for ( r = 0; r < height; r++ )
      for ( dtend = dtarr[r] + width, dtp = dtarr[r], oldp = oldarr[r+1] + 1,
	    newp = newarr[r+1] + 1; dtp != dtend; dtp++, oldp++, newp++ )
	 *dtp = *newp - *oldp;

   hor_free_image ( image );
   hor_free ( (void *) (imcol - 1) );
   hor_free ( (void *) (imarr - 1) );
}

/*******************
*   static Hor_Sub_Image *@threshold_derivs ( Hor_Sub_Image *deriv_c,
*                                            Hor_Sub_Image *deriv_r,
*                                            float          rc_threshold,
*                                            Hor_Sub_Image *deriv_t,
*                                            float          t_threshold )
*
*   If the sum of squares of row and column derivatives is above threshold,
*   the flag in the result bitmap is set to HOR_TRUE, otherwise to HOR_FALSE.
********************/
static Hor_Sub_Image *threshold_derivs ( Hor_Sub_Image *deriv_c,
					 Hor_Sub_Image *deriv_r,
					 float          rc_threshold,
					 Hor_Sub_Image *deriv_t,
					 float          t_threshold )
{
   Hor_Sub_Image *flow_flag;
   Hor_Impixel    zero_pixel;
   int            width, height, row, col;

   width  = deriv_r->image.width;
   height = deriv_r->image.height;
   flow_flag = hor_alloc_sub_image ( deriv_r->c0, deriv_r->r0, width, height,
				     HOR_CHAR, NULL );
   zero_pixel.c = HOR_FALSE;
   hor_fill_sub_image_with_constant ( flow_flag, zero_pixel );
   {
      float   rc_thres_2 = rc_threshold*rc_threshold;
      float **r_arr = deriv_r->image.array.f,   *r_ptr;
      float **c_arr = deriv_c->image.array.f,   *c_ptr;
      float **t_arr = deriv_t->image.array.f,   *t_ptr;
      char  **f_arr = flow_flag->image.array.c, *f_ptr;

      for ( row = 0; row < height; row++ )
	 for ( r_ptr = r_arr[row], c_ptr = c_arr[row], t_ptr = t_arr[row],
	       f_ptr = f_arr[row], col = 0; col != width;
	       r_ptr++, c_ptr++, t_ptr++, f_ptr++, col++ )
	    if ( *r_ptr**r_ptr + *c_ptr**c_ptr >= rc_thres_2 &&
		 fabs(*t_ptr) >= t_threshold )
	       *f_ptr = HOR_TRUE;
   }

   return flow_flag;
}

/*******************
*   static void @calc_normal_flow ( Hor_Sub_Image *deriv_c,
*                                  Hor_Sub_Image *deriv_r,
*                                  Hor_Sub_Image *deriv_t,
*                                  Hor_Sub_Image *flow_flag,
*                                  Hor_Sub_Image *flow_c,
*                                  Hor_Sub_Image *flow_r )
*
*   Takes spatial and temporal derivatives, calculates normal flow.
********************/
static void calc_normal_flow (Hor_Sub_Image *deriv_c, Hor_Sub_Image *deriv_r,
			      Hor_Sub_Image *deriv_t, Hor_Sub_Image *flow_flag,
			      Hor_Sub_Image *flow_c,  Hor_Sub_Image *flow_r )
{
   float *dcp, *dcend, *drp, *dtp, *fcp, *frp, deriv_rc_sqr;
   char  *ffp;

   for ( dcp = deriv_c->image.array.f[0],
	 dcend = deriv_c->image.array.f[0] +
	         deriv_c->image.width*deriv_c->image.height,
	 drp = deriv_r->image.array.f[0], dtp = deriv_t->image.array.f[0],
	 ffp = flow_flag->image.array.c[0],
	 fcp = flow_c->image.array.f[0], frp = flow_r->image.array.f[0];
	 dcp != dcend; dcp++, drp++, dtp++, ffp++, fcp++, frp++ )
      if ( *ffp )
      {
	 deriv_rc_sqr = *dcp**dcp + *drp**drp;
	 *fcp = -*dtp**dcp/deriv_rc_sqr;
	 *frp = -*dtp**drp/deriv_rc_sqr;
      }
}

/*******************
*   Hor_Image_Flow *@hor_image_flow ( Hor_Image *imptr,
*                                    Hor_Sub_Image *old_image,
*                                    float *gauss_mask, int gauss_size,
*                                    float rc_deriv_threshold,
*                                    float t_deriv_threshold,
*                                    int c1, int r1, int c2, int r2 )
*
*   Takes an image and a previously smoothed image, calculates the normal
*   image flow between them.
********************/
Hor_Image_Flow *hor_image_flow ( Hor_Image *imptr, Hor_Sub_Image *old_image,
				 float *gauss_mask, int gauss_size,
				 float  rc_deriv_threshold,
				 float  t_deriv_threshold,
				 int    c1, int r1, int c2, int r2 )
{
   Hor_Image_Flow *result;
   int             width, height;
   Hor_Sub_Image  *gauss, *deriv_c, *deriv_r, *deriv_t, *flow_flag;
   Hor_Sub_Image  *flow_c, *flow_r;

   /* adjust rectangular region to take account of convolution mask size */
   hor_adjust_region_for_border ( imptr->width, imptr->height,
				  gauss_size+1, gauss_size+1,
				  gauss_size+1, gauss_size+1,
				  &c1, &r1, &c2, &r2 );

   width  = c2 - c1;
   height = r2 - r1;

   /* can't do image flow in anything less than three by three! */
   if ( width < 3 || height < 3 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      return NULL;
   }

   /* allocate space for smoothed image */
   gauss = hor_alloc_sub_image ( c1-1, r1-1, width+2, height+2, HOR_FLOAT, NULL );

   /* convolve image with separable 2-D Gaussian */
   hor_convolve_image ( imptr, gauss_mask, gauss_size,
		        gauss_mask, gauss_size, gauss );

   result = hor_malloc_type ( Hor_Image_Flow );
   result->old_image = gauss;
   if ( old_image == NULL )
   {
      result->flow_c = result->flow_r = result->flow_flag = NULL;
      return result;
   }
   
   /* allocate space for derivative and flow arrays */
   deriv_c   = hor_alloc_sub_image ( c1, r1, width, height, HOR_FLOAT, NULL );
   deriv_r   = hor_alloc_sub_image ( c1, r1, width, height, HOR_FLOAT, NULL );
   deriv_t   = hor_alloc_sub_image ( c1, r1, width, height, HOR_FLOAT, NULL );
   flow_c    = hor_alloc_sub_image ( c1, r1, width, height, HOR_FLOAT, NULL );
   flow_r    = hor_alloc_sub_image ( c1, r1, width, height, HOR_FLOAT, NULL );

   /* take spatial and temporal derivatives of image */
   image_derivative ( old_image, gauss, deriv_c, deriv_r, deriv_t );

   /* threshold spatial and temporal derivatives */
   flow_flag = threshold_derivs ( deriv_c, deriv_r, rc_deriv_threshold,
				  deriv_t, t_deriv_threshold );

   /* calculate normal_image flow */
   calc_normal_flow ( deriv_c, deriv_r, deriv_t, flow_flag, flow_c, flow_r );

   /* free intermediate result images */
   hor_free_sub_images ( deriv_t, deriv_r, deriv_c, NULL );

   result->flow_flag = flow_flag;
   result->flow_c = flow_c;
   result->flow_r = flow_r;
   return result;
}

/*******************
*   void @hor_free_image_flow ( Hor_Image_Flow *result )
*
*   Frees the result of normal image flow computation.
********************/
void hor_free_image_flow ( Hor_Image_Flow *result )
{
   if ( result == NULL ) return;

   hor_free_sub_image ( result->flow_r );
   hor_free_sub_image ( result->flow_c );
   hor_free_sub_image ( result->flow_flag );
   hor_free_sub_image ( result->old_image );
   hor_free ( (void *) result );
}
