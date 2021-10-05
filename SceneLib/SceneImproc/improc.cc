#include <general_headers.h>
#include "correlate.h"
#include "improc.h"

/******************************Evaluate Patch*********************************/

/* Function to evaluate the suitability of a patch as a feature */

int evaluate_patch(Hor_Image *image, const int u, const int v, int BOXSIZE)
{
  /* Steps:
     1. Work out the image gradients
     2. Work out the sums
     3. Find the eigenvalues
     4. Find the eigenvectors?
     */
  
  double gx = 0.0;
  double gy = 0.0;
  double Sgxsq = 0.0, Sgysq = 0.0, Sgxgy = 0.0;

  for (int y = 0; y < BOXSIZE; y++)
    for (int x = 0; x < BOXSIZE; x++)
    {
      gx =       (  image->array.uc[y + v - (BOXSIZE - 1) / 2]
                                   [x + u - (BOXSIZE - 1) / 2 + 1]
                  - image->array.uc[y + v - (BOXSIZE - 1) / 2]
                                   [x + u - (BOXSIZE - 1) / 2 - 1] ) / 2.0;
      gy =       (  image->array.uc[y + v - (BOXSIZE - 1) / 2 + 1]
                                   [x + u - (BOXSIZE - 1) / 2]
                  - image->array.uc[y + v - (BOXSIZE - 1) / 2 - 1]
                                   [x + u - (BOXSIZE - 1) / 2] ) / 2.0;
      Sgxsq += gx * gx;
      Sgysq += gy * gy;
      Sgxgy += gx * gy;
    }

  cout << "From evaluate_patch: sums are Sgxgq = " << Sgxsq 
       << ", Sgysq = " << Sgysq << ", Sgxgy = " << Sgxgy << endl;

  // Form Z
  Hor_Matrix *Z = hor_mats_fill(2, 2, Sgxsq, Sgxgy, 
                                      Sgxgy, Sgysq);
  cout << "Matrix Z: " << Z;

  // Calculate eigenvalues and eigenvectors
  Hor_Matrix *PP = hor_mats_copy(Z);
  double eigenvalues[2];
  double spare_array[2];

  hor_matq_tred2(PP, eigenvalues, spare_array);
  hor_matq_tqli(eigenvalues, spare_array, PP);

  hor_matq_eigsrt(eigenvalues, PP);

  /* Now the eigenvalues are in the array eigenvalues, and the eigenvectors
     are rows in PP */

  for (int i = 0; i < 2; i++)
    cout << "Eigenvalue: " << eigenvalues[i] << ", Eigenvector: ("
         << matel(PP, 1, i + 1) << ", " << matel(PP, 2, i + 1) << ").\n";

  hor_mat_free_list(Z, PP, NULL);

  return 0;
}

/*****************************Find Best Patch*********************************/

/* Simple function to find the eigenvalues of the 2*2 symmetric matrix
   [A B]
   [B C] */

int find_eigenvalues(double A, double B, double C, 
		     double *eval1ptr, double *eval2ptr)
{
  double BB = sqrt( (A + C) * (A + C) - 4 * (A * C - B * B) );

  *eval1ptr = (A + C + BB) / 2.0;
  *eval2ptr = (A + C - BB) / 2.0;

  return 0;
}



/* Function to scan over image and find the best patch by the Shi and 
   Tomasi criterion (I think this is the same basically as the Harris
   corner detector).

   Method: as described in notes from 1/7/97. Form sums in an incremental 
   way to speed things up.
*/

int find_best_patch(Hor_Image *image, int *ubest, int *vbest, 
		    double *evbest, int BOXSIZE, double U0, double V0)
{
  /* Calculate the limits of the search. We will search from
     ustart <= u < ufinish, vstart <= v < vfinish . */
  int ustart = (int) (U0 - 0.5 * BESTPATCHSEARCHFRACTION * image->width);
  int ufinish = (int) (U0 + 0.5 * BESTPATCHSEARCHFRACTION * image->width);
  int vstart = (int) (V0 - 0.5 * BESTPATCHSEARCHFRACTION * image->height);
  int vfinish = (int) (V0 + 0.5 * BESTPATCHSEARCHFRACTION * image->height);

  /* Check that these limits aren't too close to the image edges.
     Note that we can't use the edge pixels because we can't form 
     gradients here. */
  if (ustart < (BOXSIZE - 1)/2 + 1) 
    ustart = (BOXSIZE - 1)/2 + 1;
  if (ufinish > image->width - (BOXSIZE - 1)/2 - 1)
    ufinish = image->width - (BOXSIZE - 1)/2 - 1;
  if (vstart < (BOXSIZE - 1)/2 + 1) 
    vstart = (BOXSIZE - 1)/2 + 1;
  if (vfinish > image->height - (BOXSIZE - 1)/2 - 1)
    vfinish = image->height - (BOXSIZE - 1)/2 - 1;

  /* Calculate the width we need to find gradients in. */
  int calc_width = ufinish - ustart + BOXSIZE - 1;

  /* Arrays which store column sums of height BOXSIZE */
  double CSgxsq[calc_width], CSgysq[calc_width], 
         CSgxgy[calc_width];

  /* For the final sums at each position (u, v) */
  double TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

  double gx, gy;
  int u = ustart, v = vstart;
  double eval1, eval2;

  /* Initial stage: fill these sums for the first horizontal position */
  int cstart = ustart - (BOXSIZE - 1)/2;
  int cfinish = ufinish + (BOXSIZE - 1)/2;
  int rstart = vstart - (BOXSIZE - 1)/2;
  int i;
  int c, r;
  for (c = cstart, i = 0; c < cfinish; c++, i++)
  {
    CSgxsq[i] = 0; CSgysq[i] = 0; CSgxgy[i] = 0;
    for (r = rstart; r < rstart + BOXSIZE; r++)
    {
      gx = (image->array.uc[r][c+1] - image->array.uc[r][c-1]) / 2.0;
      gy = (image->array.uc[r+1][c] - image->array.uc[r-1][c]) / 2.0;

      CSgxsq[i] += gx * gx;
      CSgysq[i] += gy * gy;
      CSgxgy[i] += gx * gy;      
    }
  }


  /* Now loop through u and v to form sums */
  for (v = vstart; v < vfinish; v++)
  {
    u = ustart;

    /* Form first sums for u = ustart */
    TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;
    for (i = 0; i < BOXSIZE; i++)
    {
      TSgxsq += CSgxsq[i];
      TSgysq += CSgysq[i];
      TSgxgy += CSgxgy[i];
    }

    for (u = ustart; u < ufinish; u++)
    {
      if (u != ustart)
      {
	/* Subtract old column, add new one */
	TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1]; 
	TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
	TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
      }

      /* cout << "Sums for u = " << u << ", v = " << v 
	   << ": TSgxsq = " << TSgxsq 
	   << ", TSgysq = " << TSgysq
	   << ", TSgxgy = " << TSgxgy << "." << endl; */	   

      find_eigenvalues(TSgxsq, TSgxgy, TSgysq, &eval1, &eval2);

      /* cout << "Eigenvalues found in find_best_patch: " 
           << eval1 << ", " << eval2 << endl; */

      /* eval2 will be the smaller eigenvalue. Compare it with the one
         we've already got */
      if (eval2 > *evbest)
      {
	*ubest = u;
	*vbest = v;
	*evbest = eval2;
      }

      // evaluate_patch(image, u, v);
    }

    if (v != vfinish - 1)
    {
      /* Update the column sums for the next v */
      for (c = cstart, i = 0; c < cfinish; c++, i++)
      {
	// Subtract the old top pixel
	gx = (image->array.uc[v - (BOXSIZE - 1)/2 ][c+1] 
	    - image->array.uc[v - (BOXSIZE - 1)/2 ][c-1]) / 2.0;
	gy = (image->array.uc[v - (BOXSIZE - 1)/2 + 1][c]  
            - image->array.uc[v - (BOXSIZE - 1)/2 - 1][c]) / 2.0;
	CSgxsq[i] -= gx * gx;
	CSgysq[i] -= gy * gy;
	CSgxgy[i] -= gx * gy;      
      
	// Add the new bottom pixel
	gx = (image->array.uc[v + (BOXSIZE - 1)/2 + 1][c+1] 
            - image->array.uc[v + (BOXSIZE - 1)/2 + 1][c-1]) / 2.0;
	gy = (image->array.uc[v + (BOXSIZE - 1)/2 + 1 + 1][c]  
            - image->array.uc[v + (BOXSIZE - 1)/2 + 1 - 1][c]) / 2.0;
	CSgxsq[i] += gx * gx;
	CSgysq[i] += gy * gy;
	CSgxgy[i] += gx * gy; 
      }     
    }
  }

  /* At the end: results */
  /* cout << "Best small eigenvalue of " << *evbest 
       << " found at u = " << *ubest << ", v = " << *vbest << endl; */

  return 0;
}

/* Version to find the best n patches within an image.
   Not very efficiently implemented I'm afraid.
   Now we expect the arguments ubest, vbest, evbest to be arrays of
   size n. */
int find_best_n_patches(Hor_Image *image, int n, int *ubest, int *vbest, 
                        double *evbest, int BOXSIZE, int U0, int V0)
{
  /* Calculate the limits of the search. We will search from
     ustart <= u < ufinish, vstart <= v < vfinish . */
  int ustart = (int) (U0 - 0.5 * BESTPATCHSEARCHFRACTION * image->width);
  int ufinish = (int) (U0 + 0.5 * BESTPATCHSEARCHFRACTION * image->width);
  int vstart = (int) (V0 - 0.5 * BESTPATCHSEARCHFRACTION * image->height);
  int vfinish = (int) (V0 + 0.5 * BESTPATCHSEARCHFRACTION * image->height);

  /* Check that these limits aren't too close to the image edges.
     Note that we can't use the edge pixels because we can't form 
     gradients here. */
  if (ustart < (BOXSIZE - 1)/2 + 1) 
    ustart = (BOXSIZE - 1)/2 + 1;
  if (ufinish > image->width - (BOXSIZE - 1)/2 - 1)
    ufinish = image->width - (BOXSIZE - 1)/2 - 1;
  if (vstart < (BOXSIZE - 1)/2 + 1) 
    vstart = (BOXSIZE - 1)/2 + 1;
  if (vfinish > image->height - (BOXSIZE - 1)/2 - 1)
    vfinish = image->height - (BOXSIZE - 1)/2 - 1;

  /* Calculate the width we need to find gradients in. */
  int calc_width = ufinish - ustart + BOXSIZE - 1;

  /* Arrays which store column sums of height BOXSIZE */
  double CSgxsq[calc_width], CSgysq[calc_width], 
         CSgxgy[calc_width];

  /* For the final sums at each position (u, v) */
  double TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

  double gx, gy;
  int u = ustart, v = vstart;
  double eval1, eval2;

  /* Initial stage: fill these sums for the first horizontal position */
  int cstart = ustart - (BOXSIZE - 1)/2;
  int cfinish = ufinish + (BOXSIZE - 1)/2;
  int rstart = vstart - (BOXSIZE - 1)/2;
  int i;
  int c, r;
  for (c = cstart, i = 0; c < cfinish; c++, i++)
  {
    CSgxsq[i] = 0; CSgysq[i] = 0; CSgxgy[i] = 0;
    for (r = rstart; r < rstart + BOXSIZE; r++)
    {
      gx = (image->array.uc[r][c+1] - image->array.uc[r][c-1]) / 2.0;
      gy = (image->array.uc[r+1][c] - image->array.uc[r-1][c]) / 2.0;

      CSgxsq[i] += gx * gx;
      CSgysq[i] += gy * gy;
      CSgxgy[i] += gx * gy;      
    }
  }

  double evarray[vfinish - vstart][ufinish - ustart];

  /* Now loop through u and v to form sums */
  for (v = vstart; v < vfinish; v++)
  {
    u = ustart;

    /* Form first sums for u = ustart */
    TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;
    for (i = 0; i < BOXSIZE; i++)
    {
      TSgxsq += CSgxsq[i];
      TSgysq += CSgysq[i];
      TSgxgy += CSgxgy[i];
    }

    for (u = ustart; u < ufinish; u++)
    {
      if (u != ustart)
      {
	/* Subtract old column, add new one */
	TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1]; 
	TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
	TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
      }

      /* cout << "Sums for u = " << u << ", v = " << v 
	   << ": TSgxsq = " << TSgxsq 
	   << ", TSgysq = " << TSgysq
	   << ", TSgxgy = " << TSgxgy << "." << endl; */	   

      find_eigenvalues(TSgxsq, TSgxgy, TSgysq, &eval1, &eval2);

      /* cout << "Eigenvalues found in find_best_patch: " 
           << eval1 << ", " << eval2 << endl; */

      /* eval2 will be the smaller eigenvalue. Store in the array */
      evarray[v - vstart][u - ustart] = eval2;
      
      // evaluate_patch(image, u, v);
    }

    if (v != vfinish - 1)
    {
      /* Update the column sums for the next v */
      for (c = cstart, i = 0; c < cfinish; c++, i++)
      {
	// Subtract the old top pixel
	gx = (image->array.uc[v - (BOXSIZE - 1)/2 ][c+1] 
	    - image->array.uc[v - (BOXSIZE - 1)/2 ][c-1]) / 2.0;
	gy = (image->array.uc[v - (BOXSIZE - 1)/2 + 1][c]  
            - image->array.uc[v - (BOXSIZE - 1)/2 - 1][c]) / 2.0;
	CSgxsq[i] -= gx * gx;
	CSgysq[i] -= gy * gy;
	CSgxgy[i] -= gx * gy;      
      
	// Add the new bottom pixel
	gx = (image->array.uc[v + (BOXSIZE - 1)/2 + 1][c+1] 
            - image->array.uc[v + (BOXSIZE - 1)/2 + 1][c-1]) / 2.0;
	gy = (image->array.uc[v + (BOXSIZE - 1)/2 + 1 + 1][c]  
            - image->array.uc[v + (BOXSIZE - 1)/2 + 1 - 1][c]) / 2.0;
	CSgxsq[i] += gx * gx;
	CSgysq[i] += gy * gy;
	CSgxgy[i] += gx * gy; 
      }     
    }
  }

  /* Now: work out the best n patches which don't overlap each other */
  double best_so_far;
  int xdist, ydist;
  int OKflag;
  double next_highest = 1000000000000.0;
  for (i = 0; i < n; i++)
  {
    best_so_far = 0.0;

    for (int y = 0; y < vfinish - vstart; y++)
      for (int x = 0; x < ufinish - ustart; x++)
      {

	if (evarray[y][x] > best_so_far && evarray[y][x] < next_highest)
	{
	  // We've found a high one: check it doesn't overlap with higher ones

	  // cout << "At x = " << x << ", y = " << y << ": evarray = " 
          //      << evarray[y][x] << endl;

	  OKflag = 1;
	  for (int j = 0; j < i; j++)
	  {
	    xdist = x + ustart - ubest[j];
	    ydist = y + vstart - vbest[j];
	    xdist = (xdist >= 0 ? xdist : -xdist);
	    ydist = (ydist >= 0 ? ydist : -ydist);
	    if (xdist < BOXSIZE && ydist < BOXSIZE)
	    {
	      // cout << "Overlap with patch " << j << endl;
	      OKflag = 0;
	      break;
	    }
	  }
	  if (OKflag)
	  {
	    ubest[i] = x + ustart;
	    vbest[i] = y + vstart;
	    evbest[i] = evarray[y][x];
	    best_so_far = evarray[y][x];
	    // cout << "Setting to patch " << i << endl;
	  }

	}

      }
    next_highest = evbest[i];
  }
  return 0;
}

/*****************************Epipolar Search*********************************/

/* Searches region of half-width LINESEARCHWIDTH about the line v = m u + c
   Returns 0 if correlation is good enough; -1 otherwise. */

int epipolar_search(Hor_Image *image, Hor_Image *patch, 
		    double m, double c, int *u_ptr, int *v_ptr, int BOXSIZE)
{
  int xstart = 0;
  int ystart = 0;
  int xfinish = image->width - BOXSIZE; 
  int yfinish = image->height - BOXSIZE;
  int x, y, xmax = 0, ymax = 0;

  double con1 = (m - 1) * (BOXSIZE + 1) / 2.0 + c;
  double con2 = LINESEARCHWIDTH * sqrt(m * m + 1.0);

  double corrmax = 1000000.0;
  double corr;

  // Actually do the search
  for(x = xstart; x <= xfinish; x++)
  {
    for(y = ystart; y <= yfinish; y++)
    {
      if( fabs(m * x - y + con1) < con2 )     /* distance check */
      {
	corr = and_correlate2(0, 0, BOXSIZE, BOXSIZE, x, y, 
		   patch, image);
	if (corr <= corrmax)
        {
	  corrmax = corr;
	  ymax = y;
	  xmax = x;
	}
      }
    }
  }

  *u_ptr = xmax + ( BOXSIZE - 1 )/2;
  *v_ptr = ymax + ( BOXSIZE - 1 )/2;
  
  if(corrmax > CORRTHRESH2)
  {
    cout << "Stereo Correlation not good enough." << endl;
    return -1;
  }

  return 0;
}

/*****************************Elliptical Search*******************************/

/* Do a search for patch in image within the elliptcal region 
   parameterised by the matrix PuInv */

int elliptical_search(Hor_Image *image, Hor_Image *patch,
		      Hor_Matrix *PuInv, int *u, int *v, 
		      double U0, double V0, int BOXSIZE)
{
  /* The dimensions of the bounding box of the ellipse we want to search in */
  int halfwidth = (int) (NO_SIGMA / sqrt( matel(PuInv, 1, 1) 
          - matel(PuInv, 1, 2) * matel(PuInv, 1, 2) / matel(PuInv, 2, 2) ));
  int halfheight = (int) (NO_SIGMA / sqrt( matel(PuInv, 2, 2) 
          - matel(PuInv, 1, 2) * matel(PuInv, 1, 2) / matel(PuInv, 1, 1) ));

  /* Limits of search */
  int urelstart = -halfwidth; 
  int urelfinish = halfwidth;
  int vrelstart = -halfheight;
  int vrelfinish = halfheight;

  /* Check these limits aren't outside the image */
  if ( (int) U0 + urelstart - (BOXSIZE - 1) / 2 < 0)
    urelstart = (BOXSIZE - 1) / 2 - (int) U0;
  if ( (int) U0 + urelfinish - (BOXSIZE - 1) / 2 > image->width - BOXSIZE)
    urelfinish = image->width - BOXSIZE - (int) U0 + (BOXSIZE - 1) / 2;
  if ( (int) V0 + vrelstart - (BOXSIZE - 1) / 2 < 0)
    vrelstart = (BOXSIZE - 1) / 2 - (int) V0;
  if ( (int) V0 + vrelfinish - (BOXSIZE - 1) / 2 > image->height - BOXSIZE)
    vrelfinish = image->height - BOXSIZE - (int) V0 + (BOXSIZE - 1) / 2;

  /* Search counters */
  int urel, vrel; 

  double corrmax = 1000000.0;
  double corr;

  // For passing to and_correlate2_warning
  double sdpatch, sdimage;

  /* Do the search */
  for(urel = urelstart; urel <= urelfinish; urel++)
    for(vrel = vrelstart; vrel <= vrelfinish; vrel++)
      if (       matel(PuInv, 1, 1) * urel * urel  
           + 2 * matel(PuInv, 1, 2) * urel * vrel
	       + matel(PuInv, 2, 2) * vrel * vrel  <  NO_SIGMA * NO_SIGMA )
      {
	corr = and_correlate2_warning(0, 0, BOXSIZE, BOXSIZE, 
			   (int) U0 + urel - (BOXSIZE - 1) / 2,
			   (int) V0 + vrel - (BOXSIZE - 1) / 2,
			   patch, image,
		           &sdpatch, &sdimage);		      
	if (corr <= corrmax)
	{
	  if (sdpatch < CORRELATION_SIGMA_THRESHOLD)
	    ; // cout << "Low patch sigma." << endl;
	  else if (sdimage < CORRELATION_SIGMA_THRESHOLD)
	    ; // cout << "Low image sigma." << endl;
	  else
	  {
	    corrmax = corr;
	    *u = urel + (int) U0;
	    *v = vrel + (int) V0;
	  }
	}
      }

  cerr << "Best match correlation: " << corrmax << endl;

  if(corrmax > CORRTHRESH2)
  {
    cout << "Matching correlation  not good enough." << endl;
    return -1;
  }

  return 0;
}

/*********************Display Elliptical Search Region************************/

/* XOR the elliptical search region specified by PuInv */

int xor_elliptical_search_region(Hor_Image *image, 
				 Hor_Matrix *PuInv, double U0, double V0,
				 int BOXSIZE)
{
  /* The dimensions of the bounding box of the ellipse we want to search in */
  int halfwidth = (int) (NO_SIGMA / sqrt( matel(PuInv, 1, 1) 
          - matel(PuInv, 1, 2) * matel(PuInv, 1, 2) / matel(PuInv, 2, 2) ));
  int halfheight = (int) (NO_SIGMA / sqrt( matel(PuInv, 2, 2) 
          - matel(PuInv, 1, 2) * matel(PuInv, 1, 2) / matel(PuInv, 1, 1) ));

  /* Limits of search */
  int urelstart = -halfwidth; 
  int urelfinish = halfwidth;
  int vrelstart = -halfheight;
  int vrelfinish = halfheight;

  /* Check these limits aren't outside the image */
  if ( (int) U0 + urelstart - (BOXSIZE - 1) / 2 < 0)
    urelstart = (BOXSIZE - 1) / 2 - (int) U0;
  if ( (int) U0 + urelfinish - (BOXSIZE - 1) / 2 > image->width - BOXSIZE)
    urelfinish = image->width - BOXSIZE - (int) U0 + (BOXSIZE - 1) / 2;
  if ( (int) V0 + vrelstart - (BOXSIZE - 1) / 2 < 0)
    vrelstart = (BOXSIZE - 1) / 2 - (int) V0;
  if ( (int) V0 + vrelfinish - (BOXSIZE - 1) / 2 > image->height - BOXSIZE)
    vrelfinish = image->height - BOXSIZE - (int) V0 + (BOXSIZE - 1) / 2;

  /* Counters */
  int urel, vrel; 

  /* Change image */
  for(urel = urelstart; urel <= urelfinish; urel++)
    for(vrel = vrelstart; vrel <= vrelfinish; vrel++)
      if (       matel(PuInv, 1, 1) * urel * urel  
           + 2 * matel(PuInv, 1, 2) * urel * vrel
	       + matel(PuInv, 2, 2) * vrel * vrel  <  NO_SIGMA * NO_SIGMA )
      {
	image->array.uc[vrel + (int) V0][urel + (int) U0] ^= 255;
      }

  return 0;
}

/*****************************************************************************/
