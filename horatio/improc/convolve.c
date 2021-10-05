/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   Hor_Bool @hor_convolve_image ( Hor_Image *imptr,      (input image)
*                                 float     *cmask,      (x-convolution mask)
*                                 int        csize,      (half-size of x mask)
*                                 float     *rmask,      (y-convolution mask)
*                                 int        rsize,      (half-size of y mask)
*                                 Hor_Sub_Image *result )    (convolved image)
*
*   Convolves image with separable symmetric mask. Returns HOR_TRUE on success,
*   HOR_FALSE and sets hor_errno on failure.
********************/
Hor_Bool hor_convolve_image ( Hor_Image *imptr, float *cmask, int csize,
			                        float *rmask, int rsize,
			      Hor_Sub_Image *result )
{
   Hor_Image  *workspace;
   int         c0 = result->c0;
   int         r0 = result->r0;
   int         width = result->image.width;
   int         height = result->image.height;
   float     **work, *workp, *wp, *work_col;
   float     **result_array;
   float       sum;
   int         big_width, big_height;
   int         r, c, i;

   if ( result->image.type != HOR_FLOAT )
      hor_error ( "attempt to convolve image into non-float sub-image",
		  HOR_FATAL );

   if ( c0 < csize || r0 < rsize ||
        c0 + width  > imptr->width  - csize ||
        r0 + height > imptr->height - rsize )
      hor_error ( "incompatible images (hor_convolve_image)", HOR_FATAL );

   big_width  = width  + 2*csize;
   big_height = height + 2*rsize;
   workspace = hor_alloc_image ( width, big_height, HOR_FLOAT, NULL );
   if ( workspace == NULL )
   {
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return HOR_FALSE;
   }

   /* set relevant pointers into workspace */
   work = hor_malloc_ntype ( float *, big_height );
   if ( work == NULL )
   {
      hor_free_image ( workspace );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return HOR_FALSE;
   }

   for ( r = 0; r < big_height; r++ )
      work[r] = workspace->array.f[r];

   work_col = hor_malloc_ntype ( float, big_height );
   if ( work_col == NULL )
   {
      hor_free ( (void *) work );
      hor_free_image ( workspace );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return HOR_FALSE;
   }

   work += rsize;  /* now work[0][0] is workspace->array.f[rsize][0] */
   work_col += rsize;
   result_array = result->image.array.f;

   switch ( imptr->type )
   {
      case HOR_U_CHAR:
      {
	 u_char **image, *imagep, *imp;

	 /* set relevant pointers into imptr */
	 image = hor_malloc_ntype ( u_char *, big_height );
	 if ( image == NULL )
	 {
	    hor_free_multiple ( (void *) (work_col-rsize),
			        (void *) (work-rsize), NULL );
	    hor_free_image ( workspace );
	    hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
	    return HOR_FALSE;
	 }

	 for ( r = 0; r < big_height; r++ )
	    image[r] = imptr->array.uc[r0 + r - rsize] + c0;

	 image += rsize; /* now image[0][0] is imptr->array.uc[r0][c0] */
	 /* do c-convolution */
	 for ( r = -rsize; r < height + rsize; r++ )
	 {
	    imagep = image[r];
	    workp  = work[r];
	    for ( c = 0, imp = imagep; c < width; c++, imp++ )
	    {
	       sum = ((float) imp[0])*cmask[0];
	       for ( i = 1; i <= csize; i++ )
		  sum += (float) imp[-i]*cmask[-i] + (float) imp[i]*cmask[i];

	       workp[c] = sum;
	    }
	 }

	 /* do r-convolution */
	 for ( c = 0; c < width; c++ )
	 {
	    /* set column in workspace as 1-D array work_col */
	    for ( r = -rsize; r < height + rsize; r++ )
	       work_col[r] = work[r][c];
	   
	    for ( r = 0, wp = work_col; r < height; r++, wp++ )
	    {
	       sum = wp[0]*rmask[0];
	       for ( i = 1; i <= rsize; i++ )
		  sum += wp[-i]*rmask[-i] + wp[i]*rmask[i];

	       result_array[r][c] = sum;
	    }
	 }

	 hor_free ( (void *) (image - rsize) );
      }
      break;

      case HOR_FLOAT:
      {
	 float **image, *imagep, *imp;

	 /* set relevant pointers into imptr */
	 image = hor_malloc_ntype ( float *, big_height );
	 if ( image == NULL )
	 {
	    hor_free_multiple ( (void *) (work_col-rsize),
			        (void *) (work-rsize), NULL );
	    hor_free_image ( workspace );
	    hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
	    return HOR_FALSE;
	 }

	 for ( r = 0; r < big_height; r++ )
	    image[r] = imptr->array.f[r0 + r - rsize] + c0;

	 image += rsize; /* now image[0][0] is imptr->array.f[r0][c0] */
	 /* do c-convolution */
	 for ( r = -rsize; r < height + rsize; r++ )
	 {
	    imagep = image[r];
	    workp  = work[r];
	    for ( c = 0, imp = imagep; c < width; c++, imp++ )
	    {
	       sum = imp[0]*cmask[0];
	       for ( i = 1; i <= csize; i++ )
		  sum += imp[-i]*cmask[-i] + imp[i]*cmask[i];

	       workp[c] = sum;
	    }
	 }

	 /* do r-convolution */
	 for ( c = 0; c < width; c++ )
	 {
	    /* set column in workspace as 1-D array work_col */
	    for ( r = -rsize; r < height + rsize; r++ )
	       work_col[r] = work[r][c];
	   
	    for ( r = 0, wp = work_col; r < height; r++, wp++ )
	    {
	       sum = wp[0]*rmask[0];
	       for ( i = 1; i <= rsize; i++ )
		  sum += wp[-i]*rmask[-i] + wp[i]*rmask[i];

	       result_array[r][c] = sum;
	    }
	 }

	 hor_free ( (void *) (image - rsize) );
      }
      break;

      default:
      hor_error ( "illegal image type for convolution", HOR_FATAL );
      break;
   }

   hor_free ( (void *) (work_col - rsize) );
   hor_free ( (void *) (work  - rsize) );
   hor_free_image ( workspace );
   return HOR_TRUE;
}

