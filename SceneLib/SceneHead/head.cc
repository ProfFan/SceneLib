/*  Scene: software for sequential localisation and map-building

    head.cc
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Contributions from Nobuyuki Kita
    nkita@etl.go.jp
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/* Some stand-alone head-related functions which we can separate from 
   the robot class into a separate class */

#include <general_headers.h>
#include "head.h"

/********************************Constructor**********************************/

/* Mainly initialise all the matrices we need */

Head::Head(double ii, double hh, 
       double pp, double cc, double nn,
       double fkuL, double fkvL, double u0L, double v0L,
       double fkuR, double fkvR, double u0R, double v0R)
  : Ii(ii), Hh(hh), 
    Pp(pp), Cc(cc), Nn(nn),
    FkuL(fkuL), FkvL(fkvL), U0L(u0L), V0L(v0L),
    FkuR(fkuR), FkvR(fkvR), U0R(u0R), V0R(v0R)
{
  cout << "Initialising head functions." << endl;

  /* Left camera calibration matrix and inverse */
  CL = hor_mats_fill(3, 3,
		    -FkuL,       0.0,        (double)U0L,
		     0.0,       -FkvL,       (double)V0L,
		     0.0,        0.0,         1.0  );
  CLinv = hor_mats_fill(3, 3,
		       -1.0 / FkuL, 0.0,        (double)U0L / FkuL,
			0.0,       -1.0 / FkvL, (double)V0L / FkvL,
			0.0,        0.0,         1.0  );

  /* Right camera calibration matrix and inverse */
  CR = hor_mats_fill(3, 3,
		    -FkuR,       0.0,        (double)U0R,
		     0.0,       -FkvR,       (double)V0R,
		     0.0,        0.0,         1.0  );
  CRinv = hor_mats_fill(3, 3,
		       -1.0 / FkuR, 0.0,        (double)U0R / FkuR,
			0.0,       -1.0 / FkvR, (double)V0R / FkvR,
			0.0,        0.0,         1.0  );
  
  MC2L = hor_mat_alloc(3, 3);
  MRC2 = hor_mat_alloc(3, 3);
  pLminuspRC2 = hor_mats_fill(3, 1, Ii, 0.0, 0.0);
  nLL = hor_mats_fill(3, 1, 0.0, 0.0, Nn);
  nRR = hor_mats_fill(3, 1, 0.0, 0.0, Nn);
  imageL = hor_mat_alloc(3, 1);
  hLL = hor_mat_alloc(3, 1);
  hLC2 = hor_mat_alloc(3, 1);
  hLR = hor_mat_alloc(3, 1);
  pLminuspRR = hor_mat_alloc(3, 1);
  nLC2 = hor_mat_alloc(3, 1);
  nLR = hor_mat_alloc(3, 1);
  nLminusnRR = hor_mat_alloc(3, 1);
  V = hor_mat_alloc(3, 1);
  X = hor_mat_alloc(3, 1);
  Y = hor_mat_alloc(3, 1);

  pRminuspLC2 = hor_mats_fill(3, 1, -Ii, 0.0, 0.0);
  hRL = hor_mat_alloc(3, 1);
  hRC2 = hor_mat_alloc(3, 1);
  pRminuspLL = hor_mat_alloc(3, 1);
  nRL = hor_mat_alloc(3, 1);
  nRminusnLL = hor_mat_alloc(3, 1);

  a = hor_mat_alloc(3, 1);
  b = hor_mat_alloc(3, 1);
  c = hor_mat_alloc(3, 1);
  d = hor_mat_alloc(3, 1);
  hRR = hor_mat_alloc(3, 1);
  imageR = hor_mat_alloc(3, 1);
  MC2R = hor_mat_alloc(3, 3);
  MC01 = hor_mat_alloc(3, 3);
  MC12 = hor_mat_alloc(3, 3);
  MC02 = hor_mat_alloc(3, 3);
  MC0L = hor_mat_alloc(3, 3);
  MC0R = hor_mat_alloc(3, 3);
  nLC0 = hor_mat_alloc(3, 1);
  nRC0 = hor_mat_alloc(3, 1);
  cLC2 = hor_mats_fill(3, 1, 0.0, Cc, 0.0);
  cRC2 = hor_mats_fill(3, 1, 0.0, Cc, 0.0);
  cLC0 = hor_mat_alloc(3, 1);
  cRC0 = hor_mat_alloc(3, 1);
  pLC1 = hor_mats_fill(3, 1, Ii/2, 0.0, Pp);
  pRC1 = hor_mats_fill(3, 1, -Ii/2, 0.0, Pp);
  pLC0 = hor_mat_alloc(3, 1);
  pRC0 = hor_mat_alloc(3, 1);

  MC10 = hor_mat_alloc(3, 3);
  MC21 = hor_mat_alloc(3, 3);
  hGC1 = hor_mat_alloc(3, 1);
  hGminuspLC1 = hor_mat_alloc(3, 1);
  minuscLminusnLC2 = hor_mat_alloc(3, 1);
  hGminuspLC2 = hor_mat_alloc(3, 1);

  MLC2 = hor_mat_alloc(3, 3);
  MC20 = hor_mat_alloc(3, 3);
  grad_u_hL = hor_mat_alloc(2, 3);
  grad_u_hR = hor_mat_alloc(2, 3);
  Temp23 = hor_mat_alloc(2, 3);
  W = hor_mat_alloc(2, 3);
  WT = hor_mat_alloc(3, 2);
  Pu = hor_mat_alloc(2, 2);
  Temp22A = hor_mat_alloc(2, 2);
  Temp22B = hor_mat_alloc(2, 2);

  MlabC0 = hor_mat_alloc(3, 3);
  MlabC2 = hor_mat_alloc(3, 3);
  MC2lab = hor_mat_alloc(3, 3);
  MlabL = hor_mat_alloc(3, 3);
  MRlab = hor_mat_alloc(3, 3);
  Temp33A = hor_mat_alloc(3, 3);
  Temp33B = hor_mat_alloc(3, 3);
  nRC2 = hor_mat_alloc(3, 1);
  nLminusnRC2 = hor_mat_alloc(3, 1);
  tC2 = hor_mat_alloc(3, 1);
  tC1 = hor_mat_alloc(3, 1);
  tC0 = hor_mat_alloc(3, 1);
  tlab = hor_mat_alloc(3, 1); 
  Temp31A = hor_mat_alloc(3, 1);
  Temp31B = hor_mat_alloc(3, 1);
  HC0 = hor_mats_fill(3, 1, 0.0, Hh, 0.0);
  rv = hor_mat_alloc(3, 1);
  alab = hor_mat_alloc(3, 1);
  N = hor_mat_alloc(3, 3);

  eC0 = hor_mat_alloc(3, 1);
  MRC0 = hor_mat_alloc(3, 3);
}

/*****************************Find Epipolar Line******************************/

/* Function and_epipolar: arguments an image position uL, vL in the left image.
   Returns the epipolar line in the right image in the form vR = m uR + c
   Refer to notes from 13/3/96
   Problem: doesn't give limits on line so search won't be very clever. */

int Head::find_epipolar_lineLR(double uL, double vL, double *mp, double *cp,
			       double gammaL, double gammaR)
{
  hor_matq_fill(imageL, uL, vL, 1.0);
  hor_matq_prod2(CLinv, imageL, hLL);

  /* Have a special fast calculation for the common case where both 
     cameras are pointing straight forwards */
  if(gammaL == 0.0 && gammaR == 0.0)
  {
    hor_matq_copy(pLminuspRC2, V);
    hor_matq_copy(hLL, hLR);
  }
  else
  {
    hor_matq_fill(MC2L,
		  cos(gammaL), 0.0,  sin(gammaL),
		  0.0,         1.0,  0.0,
		 -sin(gammaL), 0.0,  cos(gammaL) );

    hor_matq_fill(MRC2,
		  cos(gammaR), 0.0, -sin(gammaR),
		  0.0,         1.0,  0.0,
		  sin(gammaR), 0.0,  cos(gammaR) );

    hor_matq_prod2(MC2L, hLL, hLC2);
    hor_matq_prod2(MRC2, hLC2, hLR);

    hor_matq_prod2(MRC2, pLminuspRC2, pLminuspRR);
  
    hor_matq_prod2(MC2L, nLL, nLC2);
    hor_matq_prod2(MRC2, nLC2, nLR);

    hor_matq_sub(nLR, nRR, nLminusnRR);
  
    hor_matq_add2(pLminuspRR, nLminusnRR, V);
  }

  hor_matq_prod2(CR, V, X);
  hor_matq_prod2(CR, hLR, Y);

  denom = matel(X, 1, 1) * matel(Y, 3, 1) - matel(X, 3, 1) * matel(Y, 1, 1);
  *mp = ( matel(X, 2, 1) * matel(Y, 3, 1) - matel(X, 3, 1) * matel(Y, 2, 1) )
                     / denom;
  *cp = ( matel(X, 1, 1) * matel(Y, 2, 1) - matel(X, 2, 1) * matel(Y, 1, 1) )
                     / denom;

  return 0;
}

int Head::find_epipolar_lineRL(double uR, double vR, double *mp, double *cp,
			       double gammaL, double gammaR)
{
  hor_matq_fill(imageR, uR, vR, 1.0);
  hor_matq_prod2(CRinv, imageR, hRR);

  /* Have a special fast calculation for the common case where both 
     cameras are pointing straight forwards */
  if(gammaL == 0.0 && gammaR == 0.0)
  {
    hor_matq_copy(pRminuspLC2, V);
    hor_matq_copy(hRR, hRL);
  }
  else
  {
    hor_matq_fill(MC2R,
		  cos(gammaR), 0.0,  sin(gammaR),
		  0.0,         1.0,  0.0,
		 -sin(gammaR), 0.0,  cos(gammaR) );

    hor_matq_fill(MLC2,
		  cos(gammaL), 0.0, -sin(gammaL),
		  0.0,         1.0,  0.0,
		  sin(gammaL), 0.0,  cos(gammaL) );

    hor_matq_prod2(MC2R, hRR, hRC2);
    hor_matq_prod2(MLC2, hRC2, hRL);

    hor_matq_prod2(MLC2, pRminuspLC2, pRminuspLL);
  
    hor_matq_prod2(MC2R, nRR, nRC2);
    hor_matq_prod2(MLC2, nRC2, nRL);

    hor_matq_sub(nRL, nLL, nRminusnLL);
  
    hor_matq_add2(pRminuspLL, nRminusnLL, V);
  }

  hor_matq_prod2(CL, V, X);
  hor_matq_prod2(CL, hRL, Y);

  denom = matel(X, 1, 1) * matel(Y, 3, 1) - matel(X, 3, 1) * matel(Y, 1, 1);
  *mp = ( matel(X, 2, 1) * matel(Y, 3, 1) - matel(X, 3, 1) * matel(Y, 2, 1) )
                     / denom;
  *cp = ( matel(X, 1, 1) * matel(Y, 2, 1) - matel(X, 2, 1) * matel(Y, 1, 1) )
                     / denom;

  return 0;
}

/************************Find 3D Position of Feature**************************/

int Head::find_3D_position(double alpha, double e, 
                           double gammaL, double gammaR,
		     double uL, double vL, double uR, double vR, Hor_Matrix *h)
{
  /* Calculate vectors from the cameras */
  hor_matq_fill(imageL, uL, vL, 1.0);
  hor_matq_fill(imageR, uR, vR, 1.0);

  hor_matq_prod2(CLinv, imageL, hLL);
  hor_matq_prod2(CRinv, imageR, hRR);

  /* Form the rotation matrices we will need */
  hor_matq_fill(MC2L,
				   cos(gammaL), 0.0,  sin(gammaL),
				   0.0,         1.0,  0.0,
				  -sin(gammaL), 0.0,  cos(gammaL) );
  hor_matq_fill(MC2R,
				   cos(gammaR), 0.0,  sin(gammaR),
				   0.0,         1.0,  0.0,
				  -sin(gammaR), 0.0,  cos(gammaR) );
  hor_matq_fill(MC12,
		       1.0,    0.0,       0.0,
		       0.0,    cos(e),    sin(e),
		       0.0,   -sin(e),    cos(e) );

  hor_matq_fill(MC01, 
		       cos(alpha),  0.0,   sin(alpha),
		       0.0,         1.0,   0.0,
		      -sin(alpha),  0.0,   cos(alpha) );
		
  hor_matq_prod2(MC01, MC12, MC02);

  hor_matq_prod2(MC02, MC2L, MC0L);
  hor_matq_prod2(MC02, MC2R, MC0R);

  /* Calculate the vectors to send to and_intersect */
  hor_matq_prod2(MC0L, hLL, b);
  hor_matq_prod2(MC0R, hRR, d);

  hor_matq_prod2(MC0L, nLL, nLC0);
  hor_matq_prod2(MC0R, nRR, nRC0);

  hor_matq_prod2(MC02, cLC2, cLC0);
  hor_matq_prod2(MC02, cRC2, cRC0);

  hor_matq_prod2(MC01, pLC1, pLC0);
  hor_matq_prod2(MC01, pRC1, pRC0);

  /* Do and hor_matq_add3 manually because it doesn't exist in Horatio */
  hor_matq_fill(a, matel(nLC0, 1, 1) + matel(cLC0, 1, 1) + matel(pLC0, 1, 1),
		   matel(nLC0, 2, 1) + matel(cLC0, 2, 1) + matel(pLC0, 2, 1),
		   matel(nLC0, 3, 1) + matel(cLC0, 3, 1) + matel(pLC0, 3, 1));
  hor_matq_fill(c, matel(nRC0, 1, 1) + matel(cRC0, 1, 1) + matel(pRC0, 1, 1),
		   matel(nRC0, 2, 1) + matel(cRC0, 2, 1) + matel(pRC0, 2, 1),
		   matel(nRC0, 3, 1) + matel(cRC0, 3, 1) + matel(pRC0, 3, 1));

  hor_matq_vector_intersect(a, b, c, d, h);

  return 0;
}

/*****************************Find Stereo Angles******************************/

/* Find the head angles necessary to fixate on a feature at position hGC0.
   Also returns hL, the length of the vector hL or hR once fixation is
   achieved. */

int Head::find_stereo_angles(Hor_Matrix *hGC0, double *hL, 
        	    double &alpha, double &e, double &gammaL, double &gammaR)
{
  double A, phi;         /* intermediate variables to use in the calculation */

  alpha = atan2( matel(hGC0, 1, 1) , matel(hGC0, 3, 1) );
  hor_matq_fill(MC10, 
		       cos(alpha), 0.0, -sin(alpha),
		       0.0,          1.0,  0.0,
		       sin(alpha), 0.0,  cos(alpha) );

  hor_matq_prod2(MC10, hGC0, hGC1);
  A = sqrt(   matel(hGC1, 2, 1) * matel(hGC1, 2, 1)   
            + pow( matel(hGC1, 3, 1) - Pp , 2.0 )     );
  phi = atan2( matel(hGC1, 2, 1) , matel(hGC1, 3, 1) - Pp );
                /* intermediate values --- but they do have physical meaning */

  e = phi - asin( Cc / A );
  hor_matq_fill(MC21,
		       1.0,    0.0,       0.0,
		       0.0,    cos(e), -sin(e),
		       0.0,    sin(e),  cos(e) );

  /* All that's left now is to calculate the vergences. This seems a bit 
     complicated but it uses the full Yorick model with all offsets. */

  hor_matq_fill(hGminuspLC1,
			      matel(hGC1, 1, 1) - Ii/2,
			      matel(hGC1, 2, 1), 
			      matel(hGC1, 3, 1) - Pp );

  hor_matq_fill(minuscLminusnLC2,
				   0.0, 
				  -Cc,
				  -Nn );
			      
  hor_matq_prod2(MC21, hGminuspLC1, hGminuspLC2);
  
  hor_matq_add2(hGminuspLC2, minuscLminusnLC2, hLC2);

  *hL = sqrt (hor_matq_dot31(hLC2, hLC2));

  gammaL = atan2( matel(hLC2, 1, 1) , matel(hLC2, 3, 1) );
  gammaR = -gammaL;

  return 0;
}

/*******************Convert Linear Measurement to Angular*********************/

int Head::convert_linear_measurement_to_angular(Hor_Matrix *hL, Hor_Matrix *h)
{
  double h_rho= sqrt(   matel(hL, 1, 1) * matel(hL, 1, 1)
                + matel(hL, 3, 1) * matel(hL, 3, 1) ); 

  double h_len = sqrt (   h_rho * h_rho 
                + matel(hL, 2, 1) * matel(hL, 2, 1) );

  hor_matq_fill(h,
			     atan2(matel(hL, 1, 1), matel(hL, 3, 1)),
			     atan2(matel(hL, 2, 1), h_rho),
			     atan2(Ii / 2.0, h_len));
  
  return 0;
}

/***************Check the consistency of the measurement result***************/
// Check the estimated feature position is within the predicted volume
// added by nkita 001024
// This is innovation gating
// Is this the same as checking S^-1 nu ?
int Head::check_miss_measurement_angular(Hor_Matrix *h, Hor_Matrix *z, 
					 Hor_Matrix *S, const double NO_SIGMA)
{
  double diff_alpha, diff_e, diff_gamma;
  double t_alpha, t_e, t_gamma;
  Hor_Matrix *diff_h = hor_mat_alloc(3, 1);

  hor_matq_sub(z, h, diff_h);
  diff_alpha = fabs(matel(diff_h, 1, 1));
  diff_e = fabs(matel(diff_h, 2, 1));
  diff_gamma = fabs(matel(diff_h, 3, 1));

  // cout << "S = " << S << endl;

  t_alpha = NO_SIGMA * sqrt(matel(S, 1, 1));
  t_e = NO_SIGMA * sqrt(matel(S, 2, 2));
  t_gamma = NO_SIGMA * sqrt(matel(S, 3, 3));

  // cout << "diff_alpha = " << diff_alpha << " diff_e = " << diff_e << 
  //  " diff_gamma = " << diff_gamma << endl;
  // cout << "t_alpha = " << t_alpha << " t_e = " << t_e << " t_gamma = " << 
  //  t_gamma << endl;

  if(diff_alpha > t_alpha || diff_e > t_e || diff_gamma > t_gamma) return 1;
  return 0;
}

/**********************Calculate Image Search Ellipses************************/

/* Take predicted measurement and covariance and work out the matrices
   describing the ellipses we need to search in. */

int Head::calculate_image_search_ellipses(Hor_Matrix *ShL, double h_len, 
					  double alpha, double e, 
					  double gammaL, double gammaR,
				      Hor_Matrix *PuInvL, Hor_Matrix *PuInvR)
{
  hor_matq_fill(MLC2,
		cos(gammaL), 0.0, -sin(gammaL),
		0.0,         1.0,  0.0,
		sin(gammaL), 0.0,  cos(gammaL) );

  hor_matq_fill(MRC2,
		cos(gammaR), 0.0, -sin(gammaR),
		0.0,         1.0,  0.0,
		sin(gammaR), 0.0,  cos(gammaR) );

  hor_matq_fill(MC21,
		1.0,    0.0,       0.0,
		0.0,    cos(e),   -sin(e),
		0.0,    sin(e),    cos(e) );
  hor_matq_fill(MC10, 
		cos(alpha),  0.0, -sin(alpha),
		0.0,         1.0,  0.0,
		sin(alpha),  0.0,  cos(alpha) );

  hor_matq_prod2(MC21, MC10, MC20);
  
  /* Left */
  hor_matq_fill(grad_u_hL, 
               -FkuL / h_len,  0.0,       0.0,
		0.0,      -FkvL / h_len,  0.0);

  hor_matq_prod3(grad_u_hL, MLC2, MC20, Temp23, W);

  hor_matq_transpose(W, WT);
  
  hor_matq_prod3(W, ShL, WT, Temp23, Pu);

  hor_matq_inv(Pu, Temp22A, Temp22B, PuInvL);

  /* Right */
  hor_matq_fill(grad_u_hR, 
               -FkuR / h_len,  0.0,       0.0,
		0.0,      -FkvR / h_len,  0.0);

  hor_matq_prod3(grad_u_hR, MRC2, MC20, Temp23, W);

  hor_matq_transpose(W, WT);
  
  hor_matq_prod3(W, ShL, WT, Temp23, Pu);

  hor_matq_inv(Pu, Temp22A, Temp22B, PuInvR);


  return 0;
}

/*************************Calculate Planar Homography*************************/

/* pi is the unit vector perpendicular to a plane, and d its distance from
   the origin. x is the current vehicle state (z, x, phi). 
   From alpha, e, gammaL, gammaR we calculate the homographies
   between left and right images for points lying on this plane.

   Note: if it's the ground plane, the actual vehicle position shouldn't 
   matter at all. */


int Head::calculate_planar_homography_world_frame(Hor_Matrix *pi, double d,
				      Hor_Matrix *x,
				      double alpha, double e, 
				      double gammaL, double gammaR,
				      Hor_Matrix *HRL, Hor_Matrix *HLR)
{
  hor_matq_fill(MlabC0,
		cos(matel(x, 3, 1)), 0.0,  sin(matel(x, 3, 1)),
		0.0,      1.0,  0.0,
	       -sin(matel(x, 3, 1)), 0.0,  cos(matel(x, 3, 1)));

  hor_matq_fill(MC01, 
		       cos(alpha),  0.0,   sin(alpha),
		       0.0,         1.0,   0.0,
		      -sin(alpha),  0.0,   cos(alpha) );
  
  hor_matq_fill(MC12,
		       1.0,    0.0,       0.0,
		       0.0,    cos(e),    sin(e),
		       0.0,   -sin(e),    cos(e) );

  hor_matq_fill(MC2L,
		cos(gammaL), 0.0,  sin(gammaL),
		0.0,         1.0,  0.0,
	       -sin(gammaL), 0.0,  cos(gammaL) );

  hor_matq_fill(MRC2,
		cos(gammaR), 0.0, -sin(gammaR),
		0.0,         1.0,  0.0,
		sin(gammaR), 0.0,  cos(gammaR) );

  hor_matq_transpose(MRC2, MC2R);

  hor_matq_prod3(MlabC0, MC01, MC12, Temp33A, MlabC2);
  hor_matq_transpose(MlabC2, MC2lab);

  hor_matq_prod2(MlabC2, MC2L, MlabL);
  hor_matq_prod2(MRC2, MC2lab, MRlab);

  // Work out tRL (t)
  hor_matq_prod2(MC2L, nLL, nLC2);
  hor_matq_prod2(MC2R, nRR, nRC2);
  
  hor_matq_sub(nLC2, nRC2, nLminusnRC2);
  hor_matq_add2(pLminuspRC2, nLminusnRC2, tC2);

  hor_matq_prod2(MC12, tC2, tC1);
  hor_matq_prod2(MC01, tC1, tC0);
  hor_matq_prod2(MlabC0, tC0, tlab);

  // Work out aL (a)
  hor_matq_add2(cLC2, nLC2, Temp31A);        // cL + nL in C2
  hor_matq_prod2(MC12, Temp31A, Temp31B);    // cL + nL in C1
  hor_matq_add2(pLC1, Temp31B, Temp31A);     // pL + cL + nL in C1
  hor_matq_prod2(MC01, Temp31A, Temp31B);    // pL + cL + nL in C0
  hor_matq_add2(HC0, Temp31B, Temp31A);      // H + pL + cL + nL in C0
  hor_matq_prod2(MlabC0, Temp31A, Temp31B);  // H + pL + cL + nL in lab
  hor_matq_fill(rv, 
		matel(x, 2, 1),
		0.0, 
		matel(x, 1, 1));
  hor_matq_add2(rv, Temp31B, alab);          // rv + H + pL + cL + nL in lab

  hor_matq_ABT(tlab, pi, Temp33A);
  double tt = d - hor_matq_dot31(alab, pi);
  hor_matq_fill(Temp33B, 
		tt, 0.0, 0.0,
		0.0, tt, 0.0,
		0.0, 0.0, tt);
		
  hor_matq_add2(Temp33A, Temp33B, N);

  hor_matq_prod3(N, MlabL, CLinv, Temp33B, Temp33A);
  hor_matq_prod3(CR, MRlab, Temp33A, Temp33B, HRL);

  hor_matq_inv(HRL, Temp33A, Temp33B, HLR);

  return 0;
}

/* Same as the function above but works in the vehicle coordinate frame.
   We don't need to pass the vehicle state to this one. */

int Head::calculate_planar_homography_robot_frame(Hor_Matrix *piC0, 
                                      double dC0,
				      double alpha, double e, 
				      double gammaL, double gammaR,
				      Hor_Matrix *HRL, Hor_Matrix *HLR)
{
  hor_matq_fill(MC01, 
		       cos(alpha),  0.0,   sin(alpha),
		       0.0,         1.0,   0.0,
		      -sin(alpha),  0.0,   cos(alpha) );
  
  hor_matq_fill(MC12,
		       1.0,    0.0,       0.0,
		       0.0,    cos(e),    sin(e),
		       0.0,   -sin(e),    cos(e) );

  hor_matq_fill(MC2L,
		cos(gammaL), 0.0,  sin(gammaL),
		0.0,         1.0,  0.0,
	       -sin(gammaL), 0.0,  cos(gammaL) );

  hor_matq_fill(MRC2,
		cos(gammaR), 0.0, -sin(gammaR),
		0.0,         1.0,  0.0,
		sin(gammaR), 0.0,  cos(gammaR) );

  hor_matq_prod2(MC01, MC12, MC02);
  hor_matq_transpose(MC02, MC20);

  hor_matq_prod2(MC02, MC2L, MC0L);
  hor_matq_prod2(MRC2, MC20, MRC0);


  // Work out tRL (t)
  hor_matq_prod2(MC2L, nLL, nLC2);
  hor_matq_prod2(MC2R, nRR, nRC2);
  
  hor_matq_sub(nLC2, nRC2, nLminusnRC2);
  hor_matq_add2(pLminuspRC2, nLminusnRC2, tC2);

  hor_matq_prod2(MC12, tC2, tC1);
  hor_matq_prod2(MC01, tC1, tC0);

  // Work out eL (e)
  hor_matq_add2(cLC2, nLC2, Temp31A);        // cL + nL in C2
  hor_matq_prod2(MC12, Temp31A, Temp31B);    // cL + nL in C1
  hor_matq_add2(pLC1, Temp31B, Temp31A);     // pL + cL + nL in C1
  hor_matq_prod2(MC01, Temp31A, eC0);    // pL + cL + nL in C0



  hor_matq_ABT(tC0, piC0, Temp33A);
  double tt = dC0 - hor_matq_dot31(eC0, piC0);
  hor_matq_fill(Temp33B, 
		tt, 0.0, 0.0,
		0.0, tt, 0.0,
		0.0, 0.0, tt);
		
  hor_matq_add2(Temp33A, Temp33B, N);

  hor_matq_prod3(N, MC0L, CLinv, Temp33B, Temp33A);
  hor_matq_prod3(CR, MRC0, Temp33A, Temp33B, HRL);

  hor_matq_inv(HRL, Temp33A, Temp33B, HLR);

  return 0;
}
