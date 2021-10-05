/*###########################################################
* 
*  smith_corner.c
*  Copyright held by DRA
*  Written by Stephen Smith
*
*  Modified for Horatio by C Wiles, 18 Feb 1993
*
*  Changes for Horatio improc library Philip McLauchlan
*
*  This header to remain with this file
*
############################################################*/
 

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

static unsigned char *sclut = NULL;

/**********************
*   static void @smith_setup ( int diff_thres )
*
*   Setup various parameters for use in the Smith corner finder.
**********************/
static void smith_setup ( int diff_thres )
{
   static int old_diff_thres = 0;
   int i;

   if ( sclut == NULL )
      sclut = (unsigned char *) hor_malloc(511) + 255;

   if ( diff_thres != old_diff_thres )
      for ( i = -255; i < 256; i++ )
	 sclut[i] = (int)(100.0*exp(-pow((double)i/(double)diff_thres, 6.0)));

   old_diff_thres = diff_thres;
}

/******************
*   Hor_Corner_Map *@hor_smith_corners ( Hor_Image *image,
*                                       int diff_thres, int geom_thres,
*                                       int patch_size,
*                                       int c1, int r1, int c2, int r2 )
******************/
Hor_Corner_Map *hor_smith_corners ( Hor_Image *image,
				    int diff_thres, int geom_thres,
				    int patch_size,
				    int c1, int r1, int c2, int r2 )
{
   int      n, x, y, xy_2, x_2, y_2, sgn;
   float    grad, gsgn_2;
   u_char  *pm3, *pm2, *pm1, *pz, *pp1, *pp2, *pp3, *c;
   int      i, j, width, height, width_3, height_3, height_6;
   int      half_size = patch_size/2; /* patch_size assumed to be odd */
   u_char **in;
   Hor_Image         *r_image;
   int              **r;
   Hor_Corner_Map    *map;
   Hor_Impixel        pixel;
   float c0f, r0f;

   if ( image->type != HOR_U_CHAR )
      hor_error ( "illegal input image type (hor_smith_corners)", HOR_FATAL );

   if ( patch_size % 2 != 1 )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return NULL;
   }

   smith_setup ( diff_thres );

   hor_adjust_region_for_border ( image->width, image->height,
				  6, 6, 6, 6, &c1, &r1, &c2, &r2 );
   
   width  = c2 - c1; width_3  = width  + 3;
   height = r2 - r1; height_3 = height + 3; height_6 = height + 6;

   if ( width < 1 || height < 1 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      return NULL;
   }

   r_image = hor_alloc_image ( width+6, height+6, HOR_INT,    NULL );
   if ( r_image == NULL )
      hor_error ( "allocation failed (hor_smith_corners)", HOR_FATAL );

   map = hor_alloc_corner_map ( c1, r1, width, height,
			        HOR_NO_ATTRIB, NULL, NULL );
   c0f = (float) c1;
   r0f = (float) r1;
   r  = r_image->array.i;
   in = image->array.uc;

   pixel.i = 0; hor_fill_image_with_constant ( r_image, pixel );

   for ( i = 0; i < height_6; i++ ) r[i] += 3;
   r += 3;

   for ( i = -3; i < height_3; i++ )
   {
      pm3 = in[r1+i-3]+c1-3; pm2 = in[r1+i-2]+c1-3; pm1 = in[r1+i-1]+c1-3;
      pz  = in[r1+i]+c1-3;
      pp1 = in[r1+i+1]+c1-3; pp2 = in[r1+i+2]+c1-3; pp3 = in[r1+i+3]+c1-3;
      for ( j = -3; j < width_3;
	    j++, pm3++, pm2++, pm1++, pz++, pp1++, pp2++, pp3++ )
      {
	 n = 100; c = sclut - in[r1+i][c1+j];

	 n +=   c[pm3[-1]] + c[pm3[0]]  + c[pm3[1]]
	      + c[pm2[-2]] + c[pm2[-1]] + c[pm2[0]]  + c[pm2[1]] + c[pm2[2]]
	      + c[pm1[-3]] + c[pm1[-2]] + c[pm1[-1]] + c[pm1[0]] + c[pm1[1]]
	      + c[pm1[2]]  + c[pm1[3]];
	 if ( n >= geom_thres ) continue;

	 n +=   c[pz[-3]]  + c[pz[-2]]  + c[pz[-1]]  + c[pz[1]]
	      + c[pz[2]]   + c[pz[3]]
	      + c[pp1[-3]] + c[pp1[-2]] + c[pp1[-1]] + c[pp1[0]] + c[pp1[1]]
	      + c[pp1[2]]  + c[pp1[3]];
	 if ( n >= geom_thres ) continue;

	 n +=   c[pp2[-2]] + c[pp2[-1]] + c[pp2[0]] + c[pp2[1]] + c[pp2[2]]
	      + c[pp3[-1]] + c[pp3[0]]  + c[pp3[1]];
	 if ( n < geom_thres )
	 {
	    x =   3*(c[pm1[3]] + c[pz[3]] + c[pp1[3]]
		     - c[pm1[-3]] - c[pz[-3]] - c[pp1[-3]])
		+ 2*(c[pm2[2]] + c[pm1[2]] + c[pz[2]] + c[pp1[2]]
		     + c[pp2[2]] - c[pm2[-2]] - c[pm1[-2]] - c[pz[-2]]
		     - c[pp1[-2]] - c[pp2[-2]])
		+ c[pm3[1]] + c[pm2[1]] + c[pm1[1]] + c[pz[1]]
		+ c[pp1[1]] + c[pp2[1]] + c[pp3[1]]
		- c[pm3[-1]] - c[pm2[-1]] - c[pm1[-1]] - c[pz[-1]]
		- c[pp1[-1]] - c[pp2[-1]] - c[pp3[-1]];
	    y =   3*(c[pp3[-1]] + c[pp3[0]] + c[pp3[1]]
		     - c[pm3[-1]] - c[pm3[0]] - c[pm3[1]])
		+ 2*(c[pp2[-2]] + c[pp2[-1]] + c[pp2[0]] + c[pp2[1]]
		     + c[pp2[2]] - c[pm2[-2]] - c[pm2[-1]] - c[pm2[0]]
		     - c[pm2[1]] - c[pm2[2]])
		+ c[pp1[-3]] + c[pp1[-2]] + c[pp1[-1]] + c[pp1[0]]
		+ c[pp1[1]] + c[pp1[2]] + c[pp1[3]]
		- c[pm1[-3]] - c[pm1[-2]] - c[pm1[-1]] - c[pm1[0]]
		- c[pm1[1]] - c[pm1[2]] - c[pm1[3]];
		
            x_2 = x*x; y_2 = y*y; xy_2 = x_2 + y_2;
            if ( n*n < xy_2*2 )
            {
	       if ( y_2 < x_2 )
	       {
		  grad = (float)y/(float)abs(x);
		  gsgn_2 = 0.5F*HOR_SIGNF(grad);
		  sgn = HOR_SIGNF(x);
		  xy_2 = c[in[r1+i+(int)(grad+gsgn_2)][c1+j+sgn]] +
		         c[in[r1+i+(int)(2*grad+gsgn_2)][c1+j+2*sgn]] +
			 c[in[r1+i+(int)(3*grad+gsgn_2)][c1+j+3*sgn]];
	       }
	       else
	       {
		  grad = (float)x/(float)abs(y);
		  gsgn_2 = 0.5F*HOR_SIGNF(grad);
		  if ( y < 0 )
		     xy_2 = c[pm1[(int)(grad+gsgn_2)]] +
			    c[pm2[(int)(2*grad+gsgn_2)]] +
			    c[pm3[(int)(3*grad+gsgn_2)]];
		  else
		     xy_2 = c[pp1[(int)(grad+gsgn_2)]] +
			    c[pp2[(int)(2*grad+gsgn_2)]] +
			    c[pp3[(int)(3*grad+gsgn_2)]];
	       }

	       if ( xy_2 > 290 ) r[i][j] = geom_thres - n;
            }
	 }
      }
   }

   for ( i = 0; i < height; i++ )
      for ( j = 0; j < width; j++ )
	 if ( (x = r[i][j]) > 0 )
	    if ( (x>r[i-3][j-3]) && (x>r[i-3][j-2]) && (x>r[i-3][j-1]) &&
                 (x>r[i-3][j])   && (x>r[i-3][j+1]) && (x>r[i-3][j+2]) &&
                 (x>r[i-3][j+3]) &&

                 (x>r[i-2][j-3]) && (x>r[i-2][j-2]) && (x>r[i-2][j-1]) &&
                 (x>r[i-2][j])   && (x>r[i-2][j+1]) && (x>r[i-2][j+2]) &&
                 (x>r[i-2][j+3]) &&

                 (x>r[i-1][j-3]) && (x>r[i-1][j-2]) && (x>r[i-1][j-1]) &&
                 (x>r[i-1][j])   && (x>r[i-1][j+1]) && (x>r[i-1][j+2]) &&
                 (x>r[i-1][j+3]) &&

		 (x>r[i][j-3])  && (x>r[i][j-2])  && (x>r[i][j-1])  &&
		 (x>=r[i][j+1]) && (x>=r[i][j+2]) && (x>=r[i][j+3]) &&

                 (x>=r[i+1][j-3]) && (x>=r[i+1][j-2]) && (x>=r[i+1][j-1]) &&
                 (x>=r[i+1][j])   && (x>=r[i+1][j+1]) && (x>=r[i+1][j+2]) &&
                 (x>=r[i+1][j+3]) &&

                 (x>=r[i+2][j-3]) && (x>=r[i+2][j-2]) && (x>=r[i+2][j-1]) &&
                 (x>=r[i+2][j])   && (x>=r[i+2][j+1]) && (x>=r[i+2][j+2]) &&
                 (x>=r[i+2][j+3]) &&

                 (x>=r[i+3][j-3]) && (x>=r[i+3][j-2]) && (x>=r[i+3][j-1]) &&
                 (x>=r[i+3][j])   && (x>=r[i+3][j+1]) && (x>=r[i+3][j+2]) &&
                 (x>=r[i+3][j+3]) )
	       hor_add_corner ( map, r[i][j],
			        c0f + (float) j + 0.5F, r0f + (float) i + 0.5F,
			        image, c1+j-half_size, r1+i-half_size,
			               patch_size,     patch_size,
			        HOR_NO_ATTRIB, NULL, NULL,
			        hor_display_corner );

   r -= 3; for ( i = 0; i < height_6; i++ ) r[i] -= 3;
   hor_free_image ( r_image );
   return map;
}
