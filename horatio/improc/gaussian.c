/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#define ZERO_SIGMA_THRES 0.4  /* any standard deviation below  this treated as
				 zero */

static float gauss ( float x,
		     float sigma2 ) /* = 2*sigma^2 */
{
   return ( exp ( -x*x/sigma2 ) );
}

/*******************
*   float *@hor_make_gaussian_mask ( float sigma, int gauss_size )
*   void   @hor_free_gaussian_mask ( float *mask, int gauss_size )
*
*   Functions to create/free 1D Gaussian masks.
********************/
float *hor_make_gaussian_mask ( float sigma, int gauss_size )
{
   float *mask;
   int   i;

   mask = hor_malloc_ntype ( float, 2*gauss_size + 1 );
   mask += gauss_size;

   if ( sigma < ZERO_SIGMA_THRES ) /* treat as zero: no smoothing */
   {
      mask[0] = 1.0F;
      for ( i = 1; i <= gauss_size; i++ )
	 mask[i] = mask[-i] = 0.0F;

   }
   else
   {
      float total;
      float sigma2 = 2*sigma*sigma;

      mask[0] = total = 1.0F;
      for ( i = 1; i <= gauss_size; i++ )
	 mask[i] = gauss ( (float) i, sigma2 );

      for ( i = 1; i <= gauss_size; i++ )
	 total += 2.0F*mask[i];

      mask[0] /= total;
      for ( i = 1; i <= gauss_size; i++ )
      {
	 mask[i] /= total;
	 mask[-i] = mask[i];
      }
   }

   return mask;
}

void hor_free_gaussian_mask ( float *mask, int gauss_size )
{
   hor_free ( (void *) (mask - gauss_size) );
}
