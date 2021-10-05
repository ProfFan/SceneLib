/* Corner Detector originally written by Han Wang 1990
   HORATIO additions Ian Reid 11-1992 and Phil McLauchlan 2-1994

   Copyright 1993 Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#define ZERO                  0.001F
#define SCALE                 0.2F
#define GRAD_OVLAP 1
#define RESP_OVLAP 2
#define NONMAX_OVLAP 1


/*************
*   static void @calc_gradients(Hor_Image *im, 
*                               Hor_Sub_Image *Dx, Hor_Sub_Image *Dy,
*                               Hor_Sub_Image *fst)
*
*   Compute the spatial derivatives Dx and Dy and the 
*   gradient magnitude squared (rp) of the byte image im.
**************/
static void calc_gradients(Hor_Image *im,
			   Hor_Sub_Image *Dx, Hor_Sub_Image *Dy,
			   Hor_Sub_Image *fst)
{
    register int i,j;
    int height = Dx->image.height,
        width  = Dy->image.width,
        c0 = Dx->c0,
        r0 = Dx->r0;
    int *p, *c, *n, **pp, **pc, **pn;
    int dx, dy, *rp, *ddx, *ddy,
        **prp, **pddx, **pddy;

    pc = im->array.i + r0;
    pp = pc-1;
    pn = pc+1;
    prp = fst->image.array.i;
    pddx = Dx->image.array.i;
    pddy = Dy->image.array.i;

    for (i=0; i<height; i++) {
	p=(*pp++)+c0; c=(*pc++)+c0; n=(*pn++)+c0; 
	ddx=(*pddx++); ddy=(*pddy++); rp=(*prp++); 
	for (j=0; j<width; j++, p++, c++, n++) {
	    dx = *(c+1) - *(c-1);
	    dx += dx + (*(n+1) + *(p+1)) - (*(n-1) + *(p-1));
	    dy = *n - *p;
	    dy += dy + (*(n+1) + *(n-1)) - (*(p-1) + *(p+1));
	    *rp++ = dx*dx+dy*dy;
	    *ddx++ = dx;
	    *ddy++ = dy;
        }
    }
}


/*************
*   static void @linear_interpolation(int j,
*                                    int dx,   int dy,
*                                    int adx,  int ady,
*                                    int **pc,
*                                    float *pg1, float *pg2,
*                                    float *pg3, float *pg4)
*
*   Find the values of the image surface along the tangent 
*   using linear interpolation (since the tangent will not, in 
*   general, be aligned with the image axes).  The mask used 
*   is one dimensional with five points.  The value at the centre 
*   pixel is known and the remaining values are returned in
*   *pg1 thru *pg4.
**************/
static void linear_interpolation(int j,
				 int dx,   int dy,
				 int adx,  int ady,
				 int **pc,
				 float *pg1, float *pg2,
				 float *pg3, float *pg4)
{
    int *p_1 = (*(pc-2)) + j,
        *p   = (*(pc-1)) + j,
        *c   = (*pc)+j,
        *n   = (*(pc+1)) + j,
        *n_1 = (*(pc+2)) + j;

#define NORTH       (*n)
#define SOUTH       (*p)
#define EAST        (*(c+1))
#define WEST        (*(c-1))
#define NE          (*(n+1))
#define SE          (*(p+1))
#define NW          (*(n-1))
#define SW          (*(p-1))

    if (dx > -ZERO && dx < ZERO) {
	*pg1 = (float)WEST;
	*pg2 = (float)EAST;
	*pg3 = (float)*(c-2);
	*pg4 = (float)*(c+2);
    }
    else if (dy == 0) {
	*pg1 = (float)NORTH;
	*pg2 = (float)SOUTH;
	*pg3 = (float)*n_1;
	*pg4 = (float)*p_1;
    }
    else {
	register float frac;

	if (dx*dy > 0) {
	    if ( adx>ady ) {
		frac = (float)ady/(float)adx;
		*pg1 = NORTH + frac*(NW-NORTH);
		*pg2 = SOUTH + frac*(SE-SOUTH);
		if (frac<0.5F) {
		    frac += frac;
		    *pg3 = *n_1 + frac*(*(n_1-1) - *n_1);
		    *pg4 = *p_1 + frac*(*(p_1+1) - *p_1);
		} else {
		    *pg3 = (1.0F-frac) * (*(n_1-1) + *(n_1-1)) +
			     (frac+frac-1.0F) * (*(n_1-2));
		    *pg4 = (1.0F-frac) * (*(p_1+1) + *(p_1+1)) +
			     (frac+frac-1.0F) * (*(p_1+2));
		}
 	    } else {
		frac = (float)adx/(float)ady;
		*pg1 = WEST + frac*(NW-WEST);
		*pg2 = EAST + frac*(SE-EAST);
		if (frac<0.5F) {
		    frac += frac;
		    *pg3 = *(c-2) + frac*(*(n-2) - *(c-2));
		    *pg4 = *(c+2) + frac*(*(p+2) - *(c+2));
		} else {
		    *pg3 = (1.0F-frac) * (*(n-2) + *(n-2)) +
			     (frac+frac-1.0F) * (*(n_1-2));
		    *pg4 = (1.0F-frac)*(*(p+2) + *(p+2)) +
			     (frac+frac-1.0F) * (*(p_1+2));
		}
	    }
	} else {
	    if ( adx<ady ) {
		frac = (float)adx/(float)ady;
		*pg1 = EAST + frac*(NE-EAST);
		*pg2 = WEST + frac*(SW-WEST);
		if (frac<0.5F) {
		    frac += frac;
		    *pg3 = *(c+2) + frac*(*(n+2) - *(c+2));
		    *pg4 = *(c-2) + frac*(*(p-2) - *(c-2));
		} else {
		    *pg3 = (1.0F-frac) * (*(n+2) + *(n+2)) +
			    (frac+frac-1.0F) * (*(n_1+2));
		    *pg4 = (1.0F-frac) * ((*p-2) + *(p-2)) +
			    (frac+frac-1.0F) * (*(p_1-2));
		}
	    } else {
		frac = (float)ady/(float)adx;
		*pg1 = NORTH + frac*(NE-NORTH);
		*pg2 = SOUTH + frac*(SW-SOUTH);
		if (frac<0.5F) {
		    frac += frac;
		    *pg3 = *n_1 + frac*(*(n_1+1) - *n_1);
		    *pg4 = *p_1 + frac*(*(p_1-1) - *p_1);
		} else {
		    *pg3 = (1.0F-frac) * (*(n_1+1) + *(n_1+1)) +
			    (frac+frac-1.0F) * (*(n_1+2));
		    *pg4 = (1.0F-frac) * (*(p_1-1) + *(p_1-1)) +
			    (frac+frac-1.0F) * (*(p_1-2));
		}
	    }
	}
    }
#undef NORTH
#undef SOUTH
#undef EAST
#undef WEST
#undef NE
#undef NW
#undef SW
#undef SE
}


/*************
*   static void @corner_response(Hor_Image *im,
*                                Hor_Sub_Image *Dx, Hor_Sub_Image *Dy,
*                                Hor_Sub_Image *fst,
*                                int edge_thresh,
*                                Hor_Sub_Image *res)
*
*   Compute the corner response function from the image, 
*   the spatial gradients and the gradient magnitude.
**************/
static void corner_response(Hor_Image *im,
			    Hor_Sub_Image *Dx, Hor_Sub_Image *Dy,
			    Hor_Sub_Image *fst,
			    int edge_thresh,
			    Hor_Sub_Image *res)
{
    register int i,j;
    int height = res->image.height,
        width  = res->image.width,
        r0     = res->r0,
        c0     = res->c0;
    int *c, **pc;
    int *dx, *dy, adx, ady, *rp, *C, 
        **prp, **pC, **pdx, **pdy;
    float g1, g2, g3, g4, dd;

    pc  = im->array.i + r0 - 1;
    pC  = fst->image.array.i + r0 - fst->r0 - 1;
    pdx = Dx->image.array.i + r0 - Dx->r0 - 1;
    pdy = Dy->image.array.i + r0 - Dy->r0 - 1;
    prp = res->image.array.i - 1;

    for (i=0; i<height; i++) {
	c=(*(++pc)) + c0;
	C=(*(++pC)) + c0 - fst->c0;
	dx=(*(++pdx)) + c0 - Dx->c0;
	dy=(*(++pdy)) + c0 - Dx->c0;
	rp=(*(++prp));  
	for (j=0; j<width; j++, C++, c++, dx++, dy++) {
	    if ( *C<edge_thresh ) {
	        *rp++ = 0; continue;
	    }
	    if ((adx=*dx) < 0) adx = -adx;
	    if ((ady=*dy) < 0) ady = -ady;

	    /*
	     * Simpler non-maximum suppression from ~hw/corner/corner.c
	     * replaces:
	     *     nonmax(j, *dx, *dy, adx, ady, pC, &mag1, &mag2);
	     *     if(*C<=mag1 || *C<mag2) {
	     *         *rp++ = 0.0F;
	     *         continue; 
	     *     }
	     *
	     */
	    if (adx>ady) {
	        if (*C <= *(C+1) || *C < *(C-1)) {
		    *rp++ = 0;
		    continue;
		}
	    }
	    else {
		if (*C <= *(*(pC-1)+j) || *C < *(*(pC+1)+j)) {
		    *rp++ = 0;
		    continue;
		}
	    }


	    linear_interpolation(c0+j, *dx, *dy, adx, ady, pc, &g1,&g2,&g3,&g4);

	    dd = (g1+g2+g3+g3+g4+g4+0.5F)-(float)(6*(*c));
	    *rp++ = (int)(dd*dd - SCALE * (float)(*C));
	}
    }
}


/*************
*   static Hor_Corner_Map *@corner_pick(Hor_Image *im,
*                                       Hor_Sub_Image *res,
*                                       int corner_thresh,
8                                       int patch_size)
*
*   Find local maxima in the corner response function
**************/
static Hor_Corner_Map *corner_pick(Hor_Image *im,
				   Hor_Sub_Image *res,
				   int corner_thresh,
				   int patch_size)
{
  /* to locate the local maxima */

    register int *p, *c, *n;
    register int i, j;
    Hor_Corner_Map *corner_map;
    int centre, half_size = patch_size/2;
    int row_size = res->image.width,
        height   = res->image.height-1,
        width    = res->image.width-1;
    int c0 = res->c0+1, r0 = res->r0+1;

    /* Check for corners column by column */
    register int *pp, *pc, *pn;

    corner_map = hor_alloc_corner_map(c0,r0,width-1,height-1,
				      HOR_NO_ATTRIB, NULL, NULL);

    pp = res->image.array.i[0] + 1;
    pc = res->image.array.i[1] + 1;
    pn = res->image.array.i[2] + 1;

    for (j=1; j<width; j++) {
	p = pp++; c = pc++; n = pn++;

	for (i=1; i<height; i++, p=c, c=n, n+=row_size) {

	    if ((centre=*c) > corner_thresh)  {
		if (   (centre >  *(p-1)) &&
		       (centre >  *p    ) &&
		       (centre >  *(p+1)) &&
		       (centre >  *(c-1)) &&
		       (centre >= *(c+1)) &&
		       (centre >= *(n-1)) &&
		       (centre >= *n    ) &&
		       (centre >= *(n+1))  )
		   hor_add_corner ( corner_map, centre,
				    (float)(c0+j) - 0.5F, (float)(r0+i) - 0.5F,
				    im, c0+j-1-half_size, r0+i-1-half_size,
				        patch_size,       patch_size,
				    HOR_NO_ATTRIB, NULL, NULL,
				    hor_display_corner );
	     }
	}
    }

    return corner_map;
}




/*******************
*   Hor_Corner_Map *@hor_wang_corners (
*                      Hor_Image *imptr,          (input image)
*                      float  edge_thresh,    (edge strenth threshold)
*                      float  corner_thresh,  (corner strenth threshold)
8                      int    patch_size,     (size of image patch around
*                                              corner to store (must be odd))
*                      int    c1, int r1,     (top-left (1) and bottom-right
*                      int    c2, int r2 )     (2) corner of image region)
*
*   Applies Wang/Brady corner detector to region of input image.
********************/
Hor_Corner_Map *hor_wang_corners ( Hor_Image *imptr,
				   float  edge_thresh,
				   float  corner_thresh,
				   int    patch_size,
				   int    c1, int r1,
				   int    c2, int r2 )
{
   Hor_Image      *im;
   Hor_Sub_Image  *Dx;
   Hor_Sub_Image  *Dy;
   Hor_Sub_Image  *fst;
   Hor_Sub_Image  *res;
   Hor_Corner_Map *corner_map;
   int             width,     height;
   int             big_width, big_height;
   int             c0, r0;

   im = hor_convert_image(imptr, HOR_INT);

   hor_adjust_region_for_border (imptr->width, imptr->height,
				 GRAD_OVLAP + RESP_OVLAP + NONMAX_OVLAP,
				 GRAD_OVLAP + RESP_OVLAP + NONMAX_OVLAP,
				 GRAD_OVLAP + RESP_OVLAP + NONMAX_OVLAP,
				 GRAD_OVLAP + RESP_OVLAP + NONMAX_OVLAP,
				 &c1, &r1, &c2, &r2 );
   width  = c2 - c1;
   height = r2 - r1;

   /* can't do corners in anything less than three by three! */
   if ( width < 3 || height < 3 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      return NULL;
   }

   /* overlap for gradients is 1 pixel */
   c0 = c1 - NONMAX_OVLAP - RESP_OVLAP; big_width  = width  + 2*(NONMAX_OVLAP+RESP_OVLAP);
   r0 = r1 - NONMAX_OVLAP - RESP_OVLAP; big_height = height + 2*(NONMAX_OVLAP+RESP_OVLAP);

   /* set up gradient images with origin set to (c0,r0) in the coordinate frame
      of the original image (imptr) */
   fst  = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_INT, NULL);
   Dx   = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_INT, NULL);
   Dy   = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_INT, NULL);

   calc_gradients(im,Dx,Dy,fst);

   /* overlap for corner response is 3 pixels */
   c0 = c1 - NONMAX_OVLAP; big_width  = width  + 2*NONMAX_OVLAP;
   r0 = r1 - NONMAX_OVLAP; big_height = height + 2*NONMAX_OVLAP;

   /* set up corner response image with origin set to (c0,r0) in the coordinate
      frame of the original image (imptr) */
   res = hor_alloc_sub_image (c0, r0, big_width, big_height, HOR_INT, NULL);

   corner_response(im, Dx, Dy, fst, edge_thresh, res);

   corner_map = corner_pick(im, res, corner_thresh, patch_size);

   hor_free_sub_image(res);
   hor_free_sub_image(fst);
   hor_free_sub_image(Dx);
   hor_free_sub_image(Dy);
   hor_free_image(im);

   return corner_map;
}
