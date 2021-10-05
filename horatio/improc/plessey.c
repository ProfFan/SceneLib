/* Corner Detector originally written by David Murray 1990
   HORATIO additions Phil McLauchlan 11-1992

   Copyright 1993 David W. Murray (dwm@robots.oxford.ac.uk)
              and Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   static void @calc_gradients ( Hor_Image     *image,
*                                Hor_Sub_Image *gradx2,
*                                Hor_Sub_Image *grady2,
*                                Hor_Sub_Image *gradxy )
*
*   Calculate square x-derivative, square y-derivative and product of x & y
*   derivatives of input image and write them into gradx2, grady2 and gradxy.
********************/
static void calc_gradients ( Hor_Image     *image,
			     Hor_Sub_Image *gradx2,
			     Hor_Sub_Image *grady2,
			     Hor_Sub_Image *gradxy )
{
   Hor_Image  *gx2_image = &(gradx2->image);
   Hor_Image  *gy2_image = &(grady2->image);
   Hor_Image  *gxy_image = &(gradxy->image);
   float **gx2arr, **gy2arr, **gxyarr;
   float  *gx2p,    *gxyp;
   float   temp;
   int     width,     height;
   int     big_width, big_height;
   int     r0, c0, r, c;

   if ( gx2_image->type != HOR_FLOAT || gy2_image->type != HOR_FLOAT ||
        gxy_image->type != HOR_FLOAT )
      hor_error ( "attempt to take gradient into non-float images", HOR_FATAL);

   c0 = gradx2->c0;
   r0 = gradx2->r0;
   width  = gx2_image->width;
   height = gx2_image->height;
   big_width  = width  + 2;
   big_height = height + 2;

   if ( gy2_image->width != width || gy2_image->height != height ||
        gxy_image->width != width || gxy_image->height != height )
      hor_error ( "gradient images different sizes", HOR_FATAL );

   gx2arr = gx2_image->array.f;
   gy2arr = gy2_image->array.f;
   gxyarr = gxy_image->array.f;

   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char **imarr, *pimp, *nimp, *imcol, *imend;

	 imarr = hor_malloc_ntype ( u_char *, big_height );
	 for ( r = 0; r < big_height; r++ )
	    imarr[r] = image->array.uc[r0+r-1] + c0;

	 imarr++; /* now imarr points to same positions on image as gx2arr,
		     gy2arr and gxyarr */

	 /* do c-gradients: copy them into gradxy, square them into gradx2 */
	 for ( r = 0; r < height; r++ )
	    for ( imend = imarr[r] + width + 1, pimp = imarr[r]-1,
		  nimp = imarr[r]+1, gx2p = gx2arr[r], gxyp = gxyarr[r];
		  nimp != imend; pimp++, nimp++, gx2p++, gxyp++ )
	    {
	       *gxyp = (float) ((int)*nimp - (int)*pimp);
	       *gx2p = *gxyp**gxyp;
	    }

	 /* do r-gradients: multiply them into gradxy,
	    square them into grady2 */
	 imcol = hor_malloc_ntype ( u_char, big_height );
	 imcol++;
	 for ( c = 0; c < width; c++ )
	 {
	    /* set up column in image */
	    for ( r = -1; r <= height; r++ )
	       imcol[r] = imarr[r][c];

	    for ( r = 0, pimp = imcol-1, nimp = imcol+1; r < height;
		  r++, pimp++, nimp++ )
	    {
	       temp = (float) ((int)*nimp - (int)*pimp);
	       gy2arr[r][c] = temp*temp;
	       gxyarr[r][c] *= temp;
	    }
	 }

	 hor_free ( (void *) (imcol - 1) );
	 hor_free ( (void *) (imarr - 1) );
      }
      break;

      case HOR_FLOAT:
      {
	 float **imarr, *pimp, *nimp, *imcol, *imend;

	 imarr = hor_malloc_ntype ( float *, big_height );
	 for ( r = 0; r < big_height; r++ )
	    imarr[r] = image->array.f[r0+r-1] + c0;

	 imarr++; /* now imarr points to same positions on image as gx2arr,
		     gy2arr and gxyarr */

	 /* do c-gradients: copy them into gradxy, square them into gradx2 */
	 for ( r = 0; r < height; r++ )
	    for ( imend = imarr[r] + width + 1, pimp = imarr[r]-1,
		  nimp = imarr[r]+1, gx2p = gx2arr[r], gxyp = gxyarr[r];
		  nimp != imend; pimp++, nimp++, gx2p++, gxyp++ )
	    {
	       *gxyp = *nimp - *pimp;
	       *gx2p = *gxyp**gxyp;
	    }

	 /* do r-gradients: multiply them into gradxy,
	    square them into grady2 */
	 imcol = hor_malloc_ntype ( float, big_height );
	 imcol++;
	 for ( c = 0; c < width; c++ )
	 {
	    /* set up column in image */
	    for ( r = -1; r <= height; r++ )
	       imcol[r] = imarr[r][c];

	    for ( r = 0, pimp = imcol-1, nimp = imcol+1; r < height;
		  r++, pimp++, nimp++ )
	    {
	       temp = *nimp - *pimp;
	       gy2arr[r][c] = temp*temp;
	       gxyarr[r][c] *= temp;
	    }
	 }

	 hor_free ( (void *) (imcol - 1) );
	 hor_free ( (void *) (imarr - 1) );
      }
      break;

      default:
      hor_error ( "wrong type of image for taking gradients", HOR_FATAL );
      break;
   }
}

/*******************
*   static void @compute_corner_strength ( Hor_Image *gradx2,
*                                         Hor_Image *grady2,
*                                         Hor_Image *gradxy,
*                                         Hor_Image *corn_strength )
*
*   Calculate corner strengths from input gradient imags and write result
*   into corn_strength.
********************/
static void compute_corner_strength ( Hor_Image *gradx2,
				      Hor_Image *grady2,
				      Hor_Image *gradxy,
				      Hor_Image *corn_strength )
{
   int     width  = corn_strength->width;
   int     height = corn_strength->height;
   int     r;
   float **gx2arr = gradx2->array.f;
   float **gy2arr = grady2->array.f;
   float **gxyarr = gradxy->array.f;
   float **cornarr = corn_strength->array.f;
   float  *gx2p, *gy2p, *gxyp, *cornp, *cornend;
   float   num, den;

   for ( r = 0; r < height; r++ )
      for ( cornend = cornarr[r] + width, cornp = cornarr[r],
	    gx2p = gx2arr[r], gy2p = gy2arr[r], gxyp = gxyarr[r];
	    cornp != cornend; cornp++, gx2p++, gy2p++, gxyp++ )
      {
	 num = *gx2p**gy2p - *gxyp**gxyp;
	 den = *gx2p + *gy2p;
	 *cornp = (num==0) ? 0.0 : (den==0.0) ? 1000000.0 : num/den;
      }
}

/*******************
*   static Hor_Corner_Map *@compute_corners ( Hor_Sub_Image *corn_strength,
*                                            Hor_Image *image;
*                                            float thres, int patch_size)
*
*   Calculates corner positions from corner strength map and strength threshold
*   and returns corner map, storing image patches of given size around each
*   corner for use by a corner matcher.
********************/
static Hor_Corner_Map *compute_corners ( Hor_Sub_Image *corn_strength,
					 Hor_Image *image,
					 float thres, int patch_size )
{
   Hor_Corner_Map *map;
   Hor_Image      *cornimage = &(corn_strength->image);
   int             c0, r0, width, height, width_1, height_1, c, r;
   int             half_size = patch_size/2; /* patch_size assumed to be odd */
   float         **carr, *pcp, *cp, *ncp, s, c0f, r0f;

   /* variables for sub-pixel accuracy */
   float A, B, C, D, E, det;

   c0 = corn_strength->c0; width  = cornimage->width;  c0f = (float) c0;
   r0 = corn_strength->r0; height = cornimage->height; r0f = (float) r0;

   carr = cornimage->array.f;
   map = hor_alloc_corner_map ( c0+1, r0+1, width-2, height-2,
			        HOR_NO_ATTRIB, NULL, NULL );
   height_1 = height - 1;
   width_1  = width  - 1;
   for ( r = 1; r < height_1; r++ )
      for ( c = 1, pcp = carr[r-1]+1, cp = carr[r]+1, ncp = carr[r+1]+1;
	    c < width_1; c++, pcp++, cp++, ncp++ )
	 if ( (s = cp[0]) >= thres &&
	       s >= pcp[-1] && s >= pcp[0] && s >= pcp[1] &&
	       s >=  cp[-1]                && s  >  cp[1] &&
	       s  > ncp[-1] && s  > ncp[0] && s  > ncp[1] )
	 {
	    /* fit quadratic to corner strengths to find maximum point */
	    A = (pcp[-1] - 2.0F*pcp[0] + pcp[1]
	       +  cp[-1] - 2.0F*s      +  cp[1]
	       + ncp[-1] - 2.0F*ncp[0] + ncp[1])/6.0F;
	    B =    (pcp[-1] + pcp[0] +    pcp[1]
	       - 2.0*cp[-1] - 2.0F*s - 2.0*cp[1]
	          + ncp[-1] + ncp[0] +    ncp[1])/6.0F;
	    C = (pcp[-1] - pcp[1] - ncp[-1] + ncp[1])/4.0F;
	    D = (-pcp[-1] - cp[-1] - ncp[-1] + pcp[1] + cp[1] + ncp[1])/6.0;
	    E = (-pcp[-1] - pcp[0] - pcp[1] + ncp[-1] + ncp[0] + ncp[1])/6.0;
	    det = C*C - 4.0F*A*B;

	    hor_add_corner ( map, s,
			     c0f + (float) c + 0.5F, /*+ (2.0F*B*D-C*E)/det,*/
			     r0f + (float) r + 0.5F, /*+ (2.0F*A*E-C*D)/det,*/
			     image, c0+c-half_size, r0+r-half_size,
			            patch_size,     patch_size,
			     HOR_NO_ATTRIB, NULL, NULL,
			     hor_display_corner );
	 }

   return map;
}

/*******************
*   Hor_Corner_Map *@hor_plessey_corners ( Hor_Image *image,      (input image)
*                         float *gauss_mask,  (gaussian convolution mask)
*                         int    gauss_size,  (half-size of mask)
*                         float  thres,       (corner strength threshold)
*                         int    patch_size,  (size of image patch around
*                                              corner to store (must be odd))
*                         int    c1, int r1,  (top-left (1) and bottom-right
*                         int    c2, int r2 )  (2) corner of image region)
*
*   Applies Plessey(?) corner detector to region of input image, locating
*   corners to sub-pixel precision.
********************/
Hor_Corner_Map *hor_plessey_corners ( Hor_Image *image,
				      float *gauss_mask,
				      int    gauss_size,
				      float  thres,
				      int    patch_size,
				      int    c1, int r1, int c2, int r2 )
{
   Hor_Sub_Image  *gradx2;
   Hor_Sub_Image  *grady2;
   Hor_Sub_Image  *gradxy;
   Hor_Sub_Image  *sgradx2;
   Hor_Sub_Image  *sgrady2;
   Hor_Sub_Image  *sgradxy;
   Hor_Sub_Image  *corn_strength;
   int             width,     height;
   int             big_width, big_height;
   int             c0, r0, border_size = hor_imax (gauss_size+2, patch_size);
   Hor_Corner_Map *map;

   if ( patch_size % 2 != 1 )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return NULL;
   }

   hor_adjust_region_for_border ( image->width, image->height,
				  border_size, border_size,
				  border_size, border_size,
				  &c1, &r1, &c2, &r2 );
   width  = c2 - c1;
   height = r2 - r1;

   /* can't do corners in anything less than three by three! */
   if ( width < 3 || height < 3 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      return NULL;
   }

   c0 = c1 - gauss_size - 1; big_width  = width  + 2*gauss_size + 2;
   r0 = r1 - gauss_size - 1; big_height = height + 2*gauss_size + 2;

   /* set up gradient images with origin set to (c0,r0) in the coordinate frame
      of the original image (image) */
   gradx2  = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_FLOAT,
				  NULL);
   grady2  = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_FLOAT,
				  NULL);
   gradxy  = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_FLOAT,
				  NULL);

   /* calculate square x & y and xy gradients */
   calc_gradients ( image, gradx2, grady2, gradxy );

   /* set up smoothed gradient images with origin set to
      (gauss_size,gauss_size) in the coordinate frame of the gradient images */
   sgradx2 = hor_alloc_sub_image ( gauss_size,gauss_size, width+2,height+2,
				   HOR_FLOAT, NULL );
   sgrady2 = hor_alloc_sub_image ( gauss_size,gauss_size, width+2,height+2,
				   HOR_FLOAT, NULL );
   sgradxy = hor_alloc_sub_image ( gauss_size,gauss_size, width+2,height+2,
				   HOR_FLOAT, NULL );
   
   /* convolve gradient images with separable 2-D Gaussian */
   hor_convolve_image ( &(gradx2->image), gauss_mask, gauss_size,
		        gauss_mask, gauss_size, sgradx2 );
   hor_convolve_image ( &(grady2->image), gauss_mask, gauss_size,
		        gauss_mask, gauss_size, sgrady2 );
   hor_convolve_image ( &(gradxy->image), gauss_mask, gauss_size,
		        gauss_mask, gauss_size, sgradxy );

   hor_free_sub_images ( gradxy, grady2, gradx2, NULL );

   corn_strength = hor_alloc_sub_image ( c1-1, r1-1, width+2, height+2,
					 HOR_FLOAT, NULL );

   /* calculate corner strengths */
   compute_corner_strength ( &(sgradx2->image), &(sgrady2->image),
			     &(sgradxy->image), &(corn_strength->image) );

   hor_free_sub_images ( sgradxy, sgrady2, sgradx2, NULL );

   map = compute_corners ( corn_strength, image, thres, patch_size );
   hor_free_sub_image ( corn_strength );
   return map;
}
