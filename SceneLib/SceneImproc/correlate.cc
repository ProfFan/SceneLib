#include <general_headers.h>
#include "correlate.h"

/*****************************************************************************/
/*                             and_correlate                                 */
/*****************************************************************************/

/* Routine which will produce a normalised correlation value between two
   patches in images. Inputs are two pointers image[0] and image[1] to the 
   images, and six integers specifying the coordinates of the top-left and 
   bottom-right corners of the patch from image 0, and just the top-left
   coordinates for image 1 */

/* C1[0], R1[0], C2[0], R2[0], C1[1], R1[1], image[0], image[1] */


double and_correlate(int x0, int y0, int x0lim,
		     int y0lim, int x1, int y1, 
		     Hor_Image *p0, Hor_Image *p1)
{
  int x0copy = x0;
  int y0copy = y0;
  int x1copy = x1;
  int y1copy = y1;         /* make copies of these */
                                    /* IMPORTANT - only change the copies */
  int g0 = 0, g1 = 0;      /* working variables for pixel values */

  long int Sg0 = 0, Sg1 = 0, Sg0g1 = 0, Sg0sq = 0, Sg1sq = 0;

  double n = (x0lim - x0) * (y0lim - y0);     /* to hold total no of pixels */

  double covg0g1, varg0, varg1;                    /* covariance; variances */
  double Sg0doub, Sg1doub, Sg0g1doub, Sg0sqdoub, Sg1sqdoub;

  double N = 0.0;                               /* to hold the final result */

  /* at the moment the far right and bottom pixels aren't included */

  for(y0copy = y0, y1copy = y1; y0copy < y0lim ; y0copy++, y1copy++)
    for(x0copy = x0, x1copy = x1; x0copy < x0lim ; x0copy++, x1copy++)
    {
      g0 = p0->array.uc[y0copy][x0copy];
      g1 = p1->array.uc[y1copy][x1copy];
      
      Sg0 += g0;
      Sg1 += g1;
      Sg0g1 += g0 * g1;
      Sg0sq += g0 * g0;
      Sg1sq += g1 * g1;
    }

  Sg0doub = Sg0;      /* make sure these are double floats */
  Sg1doub = Sg1;
  Sg0g1doub = Sg0g1;
  Sg0sqdoub = Sg0sq;
  Sg1sqdoub = Sg1sq;

  covg0g1 = Sg0g1doub / n  -  (Sg0doub * Sg1doub) / (n*n);
  varg0   = Sg0sqdoub / n  -  (Sg0doub * Sg0doub) / (n*n);
  varg1   = Sg1sqdoub / n  -  (Sg1doub * Sg1doub) / (n*n);

  N = covg0g1 / sqrt(varg0 * varg1);

  return N;  /* returns normalised correlation coefficient */
}


/*****************************************************************************/
/*                             and_correlate2                                */
/*****************************************************************************/

// C++ified and pointers added 19/4/97

/* Alternative correlation routine which uses the "more sensible" subtraction
   type of method. Inputs are two pointers image[0] and image[1] to the 
   images, and six integers specifying the coordinates of the top-left and 
   bottom-right corners of the patch from image 0, and just the top-left
   coordinates for image 1 */

/* order to pass variables from corrtest.c:
   C1[0], R1[0], C2[0], R2[0], C1[1], R1[1], image[0], image[1] */

/* Now perfect correlation correponds to 0 returned */

double and_correlate2(int x0, int y0, int x0lim,
		     int y0lim, int x1, int y1,
		     Hor_Image *p0, Hor_Image *p1)
{
  unsigned char *p0_ptr, *p1_ptr;

  int x0copy = x0;
  int y0copy = y0;
                           /* IMPORTANT - only change the copies */
  int g0, g1;              /* working variables for pixel values */

  int patchwidth = x0lim - x0;

  long int Sg0 = 0, Sg1 = 0, Sg0g1 = 0, Sg0sq = 0, Sg1sq = 0;

  double n = (x0lim - x0) * (y0lim - y0);     /* to hold total no of pixels */

  double varg0 = 0.0, varg1 = 0.0, sigmag0 = 0.0, sigmag1 = 0.0, g0bar = 0.0, 
         g1bar = 0.0;   
                                   /* variances, standard deviations, means */
  double Sg0doub = 0.0, Sg1doub = 0.0, Sg0g1doub = 0.0, Sg0sqdoub = 0.0, 
         Sg1sqdoub = 0.0;

  double C = 0.0;                               /* to hold the final result */
  double k = 0.0;                         /* to hold an intermediate result */

  /* at the moment the far right and bottom pixels aren't included */

  p0_ptr = &(p0->array.uc[y0][x0]);
  p1_ptr = &(p1->array.uc[y1][x1]);

  for(y0copy = y0lim - 1; y0copy >= 0 ; y0copy--)
  {
    for(x0copy = x0lim - 1; x0copy >=0 ; x0copy--)
    {
      g0 = *p0_ptr;
      g1 = *p1_ptr;
      
      Sg0 += g0;
      Sg1 += g1;
      Sg0g1 += g0 * g1;
      Sg0sq += g0 * g0;
      Sg1sq += g1 * g1;

      p0_ptr++;
      p1_ptr++;
    }
    p0_ptr += (p0->width - patchwidth);
    p1_ptr += (p1->width - patchwidth);
  }

  Sg0doub = Sg0;      /* make sure these are double floats */
  Sg1doub = Sg1;
  Sg0g1doub = Sg0g1;
  Sg0sqdoub = Sg0sq;
  Sg1sqdoub = Sg1sq;

  g0bar = Sg0doub / n;
  g1bar = Sg1doub / n;

  varg0   = Sg0sqdoub / n  -  (g0bar * g0bar);
  varg1   = Sg1sqdoub / n  -  (g1bar * g1bar);

  sigmag0 = sqrt(varg0);
  sigmag1 = sqrt(varg1);

  if (sigmag0 == 0.0)      /* special checks for this algorithm */
  {                       /* to avoid division by zero */
    if (sigmag1 == 0.0)
      return 0.0;
    else
      return 1.0;
  }

  if (sigmag1 == 0.0)
    return 1.0;

  k = g0bar / sigmag0 - g1bar / sigmag1;

  C = Sg0sqdoub / varg0   +   Sg1sqdoub / varg1   +   n * (k * k)
      -   Sg0g1doub * 2.0 / (sigmag0 * sigmag1)   
      -   Sg0doub * 2.0 * k / sigmag0   +   Sg1doub * 2.0 * k / sigmag1;

  return C / n;    /* returns mean square no of s.d. from mean of pixels */ 
}


/*****************************************************************************/
/*                         and_correlate2_warning                            */
/*****************************************************************************/

/* Two extra pointer arguments return the standard deviation of the patches.
   If either of these is low, the results can be misleading. */

double and_correlate2_warning(int x0, int y0, int x0lim,
			      int y0lim, int x1, int y1,
			      Hor_Image *p0, Hor_Image *p1,
			      double *sd0ptr, double *sd1ptr)
{
  unsigned char *p0_ptr, *p1_ptr;

  int x0copy = x0;
  int y0copy = y0;
                           /* IMPORTANT - only change the copies */
  int g0, g1;              /* working variables for pixel values */

  int patchwidth = x0lim - x0;

  long int Sg0 = 0, Sg1 = 0, Sg0g1 = 0, Sg0sq = 0, Sg1sq = 0;

  double n = (x0lim - x0) * (y0lim - y0);     /* to hold total no of pixels */

  double varg0 = 0.0, varg1 = 0.0, sigmag0 = 0.0, sigmag1 = 0.0, g0bar = 0.0, 
         g1bar = 0.0;   
                                   /* variances, standard deviations, means */
  double Sg0doub = 0.0, Sg1doub = 0.0, Sg0g1doub = 0.0, Sg0sqdoub = 0.0, 
         Sg1sqdoub = 0.0;

  double C = 0.0;                               /* to hold the final result */
  double k = 0.0;                         /* to hold an intermediate result */

  /* at the moment the far right and bottom pixels aren't included */

  p0_ptr = &(p0->array.uc[y0][x0]);
  p1_ptr = &(p1->array.uc[y1][x1]);

  for(y0copy = y0lim - 1; y0copy >= 0 ; y0copy--)
  {
    for(x0copy = x0lim - 1; x0copy >=0 ; x0copy--)
    {
      g0 = *p0_ptr;
      g1 = *p1_ptr;
      
      Sg0 += g0;
      Sg1 += g1;
      Sg0g1 += g0 * g1;
      Sg0sq += g0 * g0;
      Sg1sq += g1 * g1;

      p0_ptr++;
      p1_ptr++;
    }
    p0_ptr += (p0->width - patchwidth);
    p1_ptr += (p1->width - patchwidth);
  }

  Sg0doub = Sg0;      /* make sure these are double floats */
  Sg1doub = Sg1;
  Sg0g1doub = Sg0g1;
  Sg0sqdoub = Sg0sq;
  Sg1sqdoub = Sg1sq;

  g0bar = Sg0doub / n;
  g1bar = Sg1doub / n;

  varg0   = Sg0sqdoub / n  -  (g0bar * g0bar);
  varg1   = Sg1sqdoub / n  -  (g1bar * g1bar);

  sigmag0 = sqrt(varg0);
  sigmag1 = sqrt(varg1);

  *sd0ptr = sigmag0;
  *sd1ptr = sigmag1;

  if (sigmag0 == 0.0)      /* special checks for this algorithm */
  {                       /* to avoid division by zero */
    if (sigmag1 == 0.0)
      return 0.0;
    else
      return 1.0;
  }

  if (sigmag1 == 0.0)
    return 1.0;

  k = g0bar / sigmag0 - g1bar / sigmag1;

  C = Sg0sqdoub / varg0   +   Sg1sqdoub / varg1   +   n * (k * k)
      -   Sg0g1doub * 2.0 / (sigmag0 * sigmag1)   
      -   Sg0doub * 2.0 * k / sigmag0   +   Sg1doub * 2.0 * k / sigmag1;

  return C / n;    /* returns mean square no of s.d. from mean of pixels */ 
}

/*****************************************************************************/
/*                             and_correlate3                                */
/*****************************************************************************/

/* 4000.0 is good CORR_MAXIMUM for this */


/* A third correlation method - simpler. Just does mean 
   sum of squared differences.
   Doesn't worry about means or variances.
   Inputs are two pointers image[0] and image[1] to the 
   images, and six integers specifying the coordinates of the top-left and 
   bottom-right corners of the patch from image 0, and just the top-left
   coordinates for image 1 */

/* order to pass variables from corrtest.c:
   C1[0], R1[0], C2[0], R2[0], C1[1], R1[1], image[0], image[1] */

/* Now perfect correlation correponds to 0 returned */

double and_correlate3(int x0, int y0, int x0lim,
		     int y0lim, int x1, int y1,
		     Hor_Image *p0, Hor_Image *p1)
{
  int g0, g1;               /* working variables for pixel values */

  double n = (x0lim - x0) * (y0lim - y0);     /* to hold total no of pixels */

  int x0copy = x0;
  int y0copy = y0;
  int x1copy = x1;
  int y1copy = y1;         /* make copies of these */
                                    /* IMPORTANT - only change the copies */

  double Sg0g1doub = 0.0, Sg0sqdoub = 0.0, Sg1sqdoub = 0.0;
  long int Sg0g1 = 0, Sg0sq = 0, Sg1sq = 0;

  double C = 0.0;                               /* to hold the final result */

  /* at the moment the far right and bottom pixels aren't included */

  for(y0copy = y0, y1copy = y1; y0copy < y0lim ; y0copy++, y1copy++)
    for(x0copy = x0, x1copy = x1; x0copy < x0lim ; x0copy++, x1copy++)
    {
      g0 = p0->array.uc[y0copy][x0copy];
      g1 = p1->array.uc[y1copy][x1copy];
      
      Sg0g1 += g0 * g1;   /* THESE HAVE TO BE INTS BEFORE YOU MULTIPLY */ 
      Sg0sq += g0 * g0;
      Sg1sq += g1 * g1;
    }

  Sg0g1doub = Sg0g1;
  Sg0sqdoub = Sg0sq;
  Sg1sqdoub = Sg1sq;
                                
  C = Sg0sqdoub - 2.0 * Sg0g1doub + Sg1sqdoub ;

  return C / n;
}



/*****************************************************************************/
/*                             and_correlate4                                */
/*****************************************************************************/


/* suitable CORR_MAXIMUM here is 50.0 */

/* And yet another method. Now does sum of mod[g0-g0bar - (g1-g1bar)].
   Inputs are two pointers image[0] and image[1] to the 
   images, and six integers specifying the coordinates of the top-left and 
   bottom-right corners of the patch from image 0, and just the top-left
   coordinates for image 1 */

/* order to pass variables from corrtest.c:
   C1[0], R1[0], C2[0], R2[0], C1[1], R1[1], image[0], image[1] */

/* Now perfect correlation correponds to 0 returned */

double and_correlate4(int x0, int y0, int x0lim,
		     int y0lim, int x1, int y1,
		     Hor_Image *p0, Hor_Image *p1)
{
  int x0copy = x0;
  int y0copy = y0;
  int x1copy = x1;
  int y1copy = y1;         /* make copies of these */
                                    /* IMPORTANT - only change the copies */

  double n = (x0lim - x0) * (y0lim - y0);     /* to hold total no of pixels */

  double C = 0.0, k = 0.0;             /* to hold the final result */

  /* at the moment the far right and bottom pixels aren't included */


#if 0
/* optional bit to correct for the means */
  double Sg0doub = 0, Sg1doub = 0;
  double g0bar = 0, g1bar = 0;   
                                  
  for(y0copy = y0, y1copy = y1; y0copy < y0lim ; y0copy++, y1copy++)
    for(x0copy = x0, x1copy = x1; x0copy < x0lim ; x0copy++, x1copy++)
    {
      Sg0doub += p0->array.uc[y0copy][x0copy];
      Sg1doub += p1->array.uc[y1copy][x1copy];
    }

  g0bar = Sg0doub / n;
  g1bar = Sg1doub / n;
#endif


  for(y0copy = y0, y1copy = y1; y0copy < y0lim ; y0copy++, y1copy++)
    for(x0copy = x0, x1copy = x1; x0copy < x0lim ; x0copy++, x1copy++)
    {
      k = p1->array.uc[y1copy][x1copy] - p0->array.uc[y0copy][x0copy] 
          /*        - (g1bar - g0bar)           */   ;

      C +=  (k >= 0.0) ? k : -k ;                      /* add modulus of k */
    }

  return C / n; 


}


