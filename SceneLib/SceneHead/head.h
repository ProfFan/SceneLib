/*  Scene: software for sequential localisation and map-building

    head.h
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

/* Functions for calculations for an active head */


class Head {
public:
  // Constructor
  Head(double Ii, double Hh, 
       double Pp, double Cc, double Nn,
       double FkuL, double FkvL, double U0L, double V0L,
       double FkuR, double FkvR, double U0R, double V0R);

  int find_epipolar_lineLR(double uL, double vL, double *m, double *c,
			 double gammaL, double gammaR);
  int find_epipolar_lineRL(double uL, double vL, double *m, double *c,
			 double gammaL, double gammaR);
  int find_3D_position(double alpha, double e, double gammaL, double gammaR,
		    double uL, double vL, double uR, double vR, Hor_Matrix *h);
  int find_stereo_angles(Hor_Matrix *hGC0, double *hL, 
        	    double &alpha, double &e, double &gammaL, double &gammaR);
  int convert_linear_measurement_to_angular(Hor_Matrix *hL, 
					    Hor_Matrix *h);
  int check_miss_measurement_angular(Hor_Matrix *h, Hor_Matrix *z, 
				     Hor_Matrix *S, const double NO_SIGMA);
  int calculate_image_search_ellipses(Hor_Matrix *ShL, double h_len,
				      double alpha, double e, 
				      double gammaL, double gammaR,
				      Hor_Matrix *PuInvL, Hor_Matrix *PuInvR);

  int calculate_planar_homography_world_frame(Hor_Matrix *pi, double d,
				  Hor_Matrix *x,
				  double alpha, double e, 
				  double gammaL, double gammaR,
				  Hor_Matrix *HRL, Hor_Matrix *HLR);
  int calculate_planar_homography_robot_frame(Hor_Matrix *piC0, 
                                      double dC0,
				      double alpha, double e, 
				      double gammaL, double gammaR,
				      Hor_Matrix *HRL, Hor_Matrix *HLR);

  // Constants: interocular separation and height of head centre
  const double Ii, Hh;

  // Head Parameters:
  const double Pp;    /* Horizontal offset between pan and elevation 
			 axes */
  const double Cc;    /* Offset between elevation axis and optic axes */
  const double Nn;    /* Offset along optic axis between vergence axes and
			 optic centres of cameras */

  // Camera Parameters for left and right
  const double FkuL;   
  const double FkvL;			
  const double U0L;
  const double V0L;      
  const double FkuR;   
  const double FkvR;			
  const double U0R;
  const double V0R;      

protected:
  // For find_epipolar_lineLR:
  double denom;
  Hor_Matrix *CL, *CLinv, *CR, *CRinv, *MC2L, *MRC2, *pLminuspRC2, *nLL, *nRR, 
                    *pLminuspRR, *nLC2, *nLR, *nLminusnRR, *imageL, 
                    *hLL, *hLC2, *hLR, *V, *X, *Y;

  // Extra for find_epipolar_lineRL:
  Hor_Matrix *pRminuspLC2, *hRL, *hRC2, *pRminuspLL, *nRL, *nRminusnLL;

  // Extra for and_3D_position
  Hor_Matrix *a, *b, *c, *d, 
                    *hRR, *imageR, 
                    *MC2R, *MC01, *MC12, *MC02, *MC0L, *MC0R, 
                    *nLC0, *nRC0,
                    *cLC2, *cLC0, *cRC2, *cRC0,
                    *pLC1, *pLC0, *pRC1, *pRC0;

  // Extra for and_find_stereo_angles
  Hor_Matrix *MC10, *MC21, *hGC1, *hGminuspLC1,
                    *minuscLminusnLC2, *hGminuspLC2;

  // Extra for calculate_image_search_ellipses
  Hor_Matrix *MLC2, *MC20, *grad_u_hL, *grad_u_hR, *Temp23, *W, *WT, *Pu, 
             *Temp22A, *Temp22B;  

  // Extra for calculate_planar_homography_world_frame
  Hor_Matrix *MlabC0, *MlabC2, *MC2lab, *MlabL, *MRlab,
             *Temp33A, *Temp33B,
             *nRC2, *nLminusnRC2, *tC2, *tC1, *tC0, *tlab,
             *Temp31A, *Temp31B,
             *HC0, 
             *rv, *alab, *N;

  // Extra for calculate_planar_homography_robot_frame
  Hor_Matrix *eC0, *MRC0;
  
};
