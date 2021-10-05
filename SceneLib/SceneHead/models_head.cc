/*  Scene: software for sequential localisation and map-building

    models_head.cc
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
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

#include "general_headers.h"
#include "models_base.h"
#include "models_head.h"


Head_Point_Feature_Measurement_Model::Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double ii, double hh,
       double pan_range_limit, double elevation_range_limit, 
       double vergence_range_limit,
       double maximum_length_ratio, double maximum_angle_difference,
       double sd_alpha, double sd_e, double sd_gamma,
       double sd_alpha_filter, double sd_e_filter, double sd_gamma_filter)
  : Point_Feature_Measurement_Model(3, 3, m_m, "HEAD_POINT"),
    Ii(ii), Hh(hh),
    PAN_RANGE_LIMIT(pan_range_limit), 
    ELEVATION_RANGE_LIMIT(elevation_range_limit), 
    VERGENCE_RANGE_LIMIT(vergence_range_limit),
    MAXIMUM_LENGTH_RATIO(maximum_length_ratio), 
    MAXIMUM_ANGLE_DIFFERENCE(maximum_angle_difference),
    SD_alpha(sd_alpha), SD_e(sd_e), SD_gamma(sd_gamma),
    SD_alpha_filter(sd_alpha_filter), SD_e_filter(sd_e_filter), 
    SD_gamma_filter(sd_gamma_filter)
{}


// Point feature for 2D motion, stereo head measurement model
TwoD_Head_Point_Feature_Measurement_Model::TwoD_Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double ii, double hh,
       double pan_range_limit, double elevation_range_limit, 
       double vergence_range_limit,
       double maximum_length_ratio, double maximum_angle_difference,
       double sd_alpha, double sd_e, double sd_gamma,
       double sd_alpha_filter, double sd_e_filter, double sd_gamma_filter)
  : Head_Point_Feature_Measurement_Model(m_m, ii, hh, 
					 pan_range_limit, 
					 elevation_range_limit,
					 vergence_range_limit,
					 maximum_length_ratio, 
					 maximum_angle_difference,
					 sd_alpha, sd_e, sd_gamma,
					 sd_alpha_filter, sd_e_filter, 
					 sd_gamma_filter)
{
  assert(strcmp(motion_model->motion_model_dimensionality_type, "TWOD") == 0);

  // Allocate temporary matrices for calculations

  // Initialise feature
  dyi_by_dhiL = hor_mat_alloc(3, 3);
  dhiL_by_dhi = hor_mat_alloc(3, 3);
  hiL = hor_mat_alloc(3, 1);

  // Measure feature
  dhi_by_dhiL = hor_mat_alloc(3, 3);

  // Visibility Test
  MlabC0 = hor_mat_alloc(3, 3);
  hiLlab = hor_mat_alloc(3, 1);
  hiL_origlab = hor_mat_alloc(3, 1);
}

TwoD_Head_Point_Feature_Measurement_Model::~TwoD_Head_Point_Feature_Measurement_Model()
{
  hor_mat_free_list(dyi_by_dhiL, dhiL_by_dhi, hiL, 
		    dhi_by_dhiL, 
		    MlabC0, hiLlab, hiL_origlab,
		    NULL);
  Feature_Measurement_Model_destructor();
}

// Defines the pose of a point for 3D graphics
// Trivial in this case
int TwoD_Head_Point_Feature_Measurement_Model::func_yipose_and_Pyiyipose(Hor_Matrix *yi, 
						Hor_Matrix *Pyiyi)
{
  assert(yi != NULL && Pyiyi != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 Pyiyi->rows == FEATURE_STATE_SIZE && 
	 Pyiyi->cols == FEATURE_STATE_SIZE);

  hor_matq_copy(yi, yiposeRES);
  hor_matq_copy(Pyiyi, PyiyiposeRES);

  return 0;
}

// For initialising a feature: note that this function does introduce
// some bias (we tend to overestimate the distance to the feature)
int TwoD_Head_Point_Feature_Measurement_Model::
               func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
							    Hor_Matrix *xp)
{
  assert(hi != NULL && xp != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  double hi_len = Ii / (2.0 * tan( matel(hi, 3, 1) ) );  // distance
  double se = sin(matel(hi, 2, 1));
  double ce = cos(matel(hi, 2, 1));
  double salpha = sin(matel(hi, 1, 1));
  double calpha = cos(matel(hi, 1, 1));
  double sgamma = sin(matel(hi, 3, 1));
  double dhi_dgamma = -Ii / (2.0 * sgamma * sgamma);
  double sphi = sin(matel(xp, 3, 1));
  double cphi = cos(matel(xp, 3, 1));


  hor_matq_fill(dyi_by_dhiL,
		cphi, 0.0, sphi,
		0.0,  1.0, 0.0,
               -sphi, 0.0, cphi);
                                     
  hor_matq_fill(dhiL_by_dhi,
	hi_len * ce * calpha, -hi_len * se * salpha, dhi_dgamma * ce * salpha,
	0.0,                   hi_len * ce,          dhi_dgamma * se,
       -hi_len * ce * salpha, -hi_len * se * calpha, dhi_dgamma * ce * calpha);

  hor_matq_prod2(dyi_by_dhiL, dhiL_by_dhi, dyi_by_dhiRES);

  hor_matq_fill(hiL,
		hi_len * ce * salpha,
		hi_len * se,
		hi_len * ce * calpha);

  hor_matq_fill(dyi_by_dxpRES,
		0.0, 1.0, -sphi * matel(hiL, 1, 1) + cphi * matel(hiL, 3, 1),
		0.0, 0.0,  0.0,
		1.0, 0.0, -cphi * matel(hiL, 1, 1) - sphi * matel(hiL, 3, 1));

  hor_matq_fill(yiRES, 
		matel(xp, 2, 1) 
              + matel(hiL, 1, 1) * cphi + matel(hiL, 3, 1) * sphi,
		Hh + matel(hiL, 2, 1),
		matel(xp, 1, 1) 
              - matel(hiL, 1, 1) * sphi + matel(hiL, 3, 1) * cphi);

  // Set the measurement noise
  func_Ri(hi);

  return 0;
}

int TwoD_Head_Point_Feature_Measurement_Model::
             func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
						   Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  // This function gives relative position of feature
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // hiL is the position of the feature relative to the head centre
  hor_matq_fill(hiL,
		matel(zeroedyiRES, 1, 1),
		matel(zeroedyiRES, 2, 1) - Hh,
		matel(zeroedyiRES, 3, 1));
  double hi_rho = sqrt(  matel(hiL, 1, 1) * matel(hiL, 1, 1)
		  + matel(hiL, 3, 1) * matel(hiL, 3, 1) );
  double hi_len = sqrt (  hi_rho * hi_rho 
		   + matel(hiL, 2, 1) * matel(hiL, 2, 1) );

  /* Now work out the actual predicted measurement */
  hor_matq_fill(hiRES,
		atan2(matel(hiL, 1, 1), matel(hiL, 3, 1)),
		atan2(matel(hiL, 2, 1), hi_rho),
		atan2(Ii / 2.0, hi_len));
    
  /* Now calculate dhi_by_dxp and dhi_by_dyi, the Jacobians we need */
  double tmp = -2.0 * Ii / ( (4.0 * hi_len * hi_len + Ii * Ii) * hi_len);
  hor_matq_fill(dhi_by_dhiL,
        matel(hiL, 3, 1) / (hi_rho * hi_rho), 0.0, 
       -matel(hiL, 1, 1) / (hi_rho * hi_rho),
       -matel(hiL, 1, 1) * matel(hiL, 2, 1) / (hi_len * hi_len * hi_rho), 
        hi_rho / (hi_len * hi_len), 
       -matel(hiL, 3, 1) * matel(hiL, 2, 1) / (hi_len * hi_len * hi_rho), 
        tmp * matel(hiL, 1, 1), tmp * matel(hiL, 2, 1), 
        tmp * matel(hiL, 3, 1));

  hor_matq_prod2(dhi_by_dhiL, dzeroedyi_by_dxpRES, dhi_by_dxpRES);
  hor_matq_prod2(dhi_by_dhiL, dzeroedyi_by_dyiRES, dhi_by_dyiRES);
  
  return 0;
}

int TwoD_Head_Point_Feature_Measurement_Model::func_nui(Hor_Matrix *hi, 
						   Hor_Matrix *zi)
{
  hor_matq_sub(zi, hi, nuiRES);
  
  for (int i = 0; i < MEASUREMENT_SIZE; i++)
    and_pi_range(nuiRES->m[i][0]);

  return 0;
}

int TwoD_Head_Point_Feature_Measurement_Model::func_Ri(Hor_Matrix *hi)
{
  assert(hi != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  // In this measurement model, Ri doesn't depend on hi
  hor_matq_diagonal(RiRES, 
		    SD_alpha_filter * SD_alpha_filter,
		    SD_e_filter * SD_e_filter,
		    SD_gamma_filter * SD_gamma_filter);

  return 0;
}

// In this particular noisy measurement function we just add
// Gaussian noise to the true measurement, but having this function
// separate provides the generality for different noise models
int TwoD_Head_Point_Feature_Measurement_Model::
             func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true)
{
  assert(yi_true != NULL && xp_true != NULL);

  assert(yi_true->rows == FEATURE_STATE_SIZE && yi_true->cols == 1 &&
	 xp_true->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_true->cols == 1);

  func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi_true, xp_true);

  hor_matq_fill(hi_noisyRES,
		hor_gauss_rand(matel(hiRES, 1, 1), SD_alpha),
		hor_gauss_rand(matel(hiRES, 2, 1), SD_e),
		hor_gauss_rand(matel(hiRES, 3, 1), SD_gamma));

  return 0;
}

int TwoD_Head_Point_Feature_Measurement_Model::
          func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		                          Hor_Matrix *yi, Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  double cphi = cos(matel(xp, 3, 1));
  double sphi = sin(matel(xp, 3, 1));

  // Position of feature relative to robot in Euclidean form
  hor_matq_fill(zeroedyiRES,
          cphi * (matel(yi, 1, 1) 
          - matel(xp, 2, 1)) 
          - sphi * (matel(yi, 3, 1) 
          - matel(xp, 1, 1)),
          matel(yi, 2, 1),
          sphi * (matel(yi, 1, 1) 
          - matel(xp, 2, 1)) 
          + cphi * (matel(yi, 3, 1) 
          - matel(xp, 1, 1)));

  // Now calculate Jacobians
  hor_matq_fill(dzeroedyi_by_dyiRES, 
		cphi, 0.0, -sphi,
		0.0,  1.0,  0.0,
		sphi, 0.0,  cphi);

  hor_matq_fill(dzeroedyi_by_dxpRES,
		  sphi, -cphi,  
                - (matel(yi, 3, 1) - matel(xp, 1, 1)) * cphi 
                - (matel(yi, 1, 1) - matel(xp, 2, 1)) * sphi,
		  0.0,   0.0,     0.0,
	         -cphi, -sphi, 
                  (matel(yi, 1, 1) - matel(xp, 2, 1)) * cphi 
                - (matel(yi, 3, 1) - matel(xp, 1, 1)) * sphi);

  return 0;
}

int TwoD_Head_Point_Feature_Measurement_Model::visibility_test(Hor_Matrix *xp, 
						     Hor_Matrix *yi,
						     Hor_Matrix *xp_orig, 
						     Hor_Matrix *hi)
{
  assert(xp != NULL && xp_orig != NULL && hi != NULL);

  assert(xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1 &&
	 yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp_orig->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_orig->cols == 1 &&
	 hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  int cant_see_flag = 0;

  /* Test axis range limit */
  if ( matel(hi, 1, 1) > PAN_RANGE_LIMIT || 
       matel(hi, 1, 1) < -PAN_RANGE_LIMIT )
    cant_see_flag |= PAN_RANGE_FAIL;

  if ( matel(hi, 2, 1) > ELEVATION_RANGE_LIMIT || 
       matel(hi, 2, 1) < -ELEVATION_RANGE_LIMIT )
    cant_see_flag |= ELEVATION_RANGE_FAIL;

  /* Do tests on length and angle of predicted view */
  double hi_len = Ii / (2.0 * tan( matel(hi, 3, 1) ) );  // distance
  double se = sin(matel(hi, 2, 1));
  double ce = cos(matel(hi, 2, 1));
  double salpha = sin(matel(hi, 1, 1));
  double calpha = cos(matel(hi, 1, 1));
  double sphi = sin(matel(xp, 3, 1));
  double cphi = cos(matel(xp, 3, 1));

  hor_matq_fill(hiL,
		hi_len * ce * salpha,
		hi_len * se,
		hi_len * ce * calpha);

  hor_matq_fill(MlabC0,
		cphi, 0.0, sphi,
		0.0,  1.0, 0.0,
	       -sphi, 0.0, cphi);   

  hor_matq_prod2(MlabC0, hiL, hiLlab);

  hor_matq_fill(hiL_origlab,
		matel(yi, 1, 1) - matel(xp_orig, 2, 1),
		matel(yi, 2, 1) - Hh,
		matel(yi, 3, 1) - matel(xp_orig, 1, 1));

  double mod_hiLlab = sqrt( hor_matq_dot31(hiLlab, hiLlab) );
  double mod_hiL_origlab = sqrt( hor_matq_dot31(hiL_origlab, hiL_origlab) );

  double length_ratio = mod_hiLlab / mod_hiL_origlab;
  if ( length_ratio > MAXIMUM_LENGTH_RATIO || 
       length_ratio < (1.0 / MAXIMUM_LENGTH_RATIO) )
    cant_see_flag |= DISTANCE_FAIL;

  double dot_prod = hor_matq_dot31(hiLlab, hiL_origlab);

  double angle = acos(dot_prod / (mod_hiLlab * mod_hiL_origlab));
  angle = (angle >= 0.0 ? angle : -angle);  // Make angle positive
  if (angle > MAXIMUM_ANGLE_DIFFERENCE)
    cant_see_flag |= ANGLE_FAIL;

  return cant_see_flag;   // 0 if OK, otherwise error code
}

// Give a score which says how important it is to measure this feature
double TwoD_Head_Point_Feature_Measurement_Model::selection_score(Hor_Matrix *Si)
{
  // Calculate the volume of uncertainty ellipsoid
  double eigenvalues[3];
  double spare_array[3];

  hor_matq_copy(Si, Temp_MM1);

  hor_matq_tred2(Temp_MM1, eigenvalues, spare_array);
  hor_matq_tqli(eigenvalues, spare_array, Temp_MM1);
    
  hor_matq_eigsrt(eigenvalues, Temp_MM1);

  return (4.0 / 3.0) * M_PI // * NO_SIGMA * NO_SIGMA * NO_SIGMA 
             * sqrt(eigenvalues[0] * eigenvalues[1] * eigenvalues[2]);
}


// Point feature for 3D motion, stereo head measurement model
ThreeD_Head_Point_Feature_Measurement_Model::
ThreeD_Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double ii, double hh,
       double pan_range_limit, double elevation_range_limit, 
       double vergence_range_limit,
       double maximum_length_ratio, double maximum_angle_difference,
       double sd_alpha, double sd_e, double sd_gamma,
       double sd_alpha_filter, double sd_e_filter, double sd_gamma_filter)
  : Head_Point_Feature_Measurement_Model(m_m, ii, hh, 
					 pan_range_limit, 
					 elevation_range_limit,
					 vergence_range_limit,
					 maximum_length_ratio, 
					 maximum_angle_difference,
					 sd_alpha, sd_e, sd_gamma,
					 sd_alpha_filter, sd_e_filter, 
					 sd_gamma_filter)
{
  assert(strcmp(motion_model->motion_model_dimensionality_type, "THREED") 
	 == 0);

  threed_motion_model = (ThreeD_Motion_Model *) motion_model;

  // Allocate temporary matrices for calculations

  // Initialise feature
  dyi_by_dhiL = hor_mat_alloc(3, 3);
  dhiL_by_dhi = hor_mat_alloc(3, 3);
  hiL = hor_mat_alloc(3, 1);

  dhi_by_dhiL = hor_mat_alloc(3, 3);
  RWR = hor_mat_alloc(3, 3);
  hiLW = hor_mat_alloc(3, 1);
  dyi_by_dx = hor_mat_alloc(3, 3);
  dyi_by_dq = hor_mat_alloc(3, 4);

  yminusxW = hor_mat_alloc(3, 1);
  qRW = hor_mat_alloc(4, 1);
  RRW = hor_mat_alloc(3, 3);
  dzeroedyi_by_dx = hor_mat_alloc(3, 3);
  dzeroedyi_by_dq = hor_mat_alloc(3, 4);
  dqRW_by_dq = hor_mat_alloc(4, 4);
  dzeroedyi_by_dqRW = hor_mat_alloc(3, 4);

  // Visibility Test
  MlabC0 = hor_mat_alloc(3, 3);
  hiLlab = hor_mat_alloc(3, 1);
  hiL_origlab = hor_mat_alloc(3, 1);
  hiL_orig = hor_mat_alloc(3, 1);
  MlabC0_orig = hor_mat_alloc(3, 3);
}

ThreeD_Head_Point_Feature_Measurement_Model::
~ThreeD_Head_Point_Feature_Measurement_Model()
{
  hor_mat_free_list(dyi_by_dhiL, dhiL_by_dhi, hiL, 
		    dhi_by_dhiL, RWR, hiLW,
		    dyi_by_dx, dyi_by_dq,
		    yminusxW, qRW, RRW, 
		    dzeroedyi_by_dx, dzeroedyi_by_dq,
		    dqRW_by_dq, dzeroedyi_by_dqRW,
		    MlabC0, hiLlab, hiL_origlab,
		    hiL_orig, MlabC0_orig,
		    NULL);
  Feature_Measurement_Model_destructor();
}

// Defines the pose of a point for 3D graphics
// Trivial in this case
int ThreeD_Head_Point_Feature_Measurement_Model
::func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi)
{
  assert(yi != NULL && Pyiyi != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 Pyiyi->rows == FEATURE_STATE_SIZE && 
	 Pyiyi->cols == FEATURE_STATE_SIZE);

  hor_matq_copy(yi, yiposeRES);
  hor_matq_copy(Pyiyi, PyiyiposeRES);

  return 0;
}

// For initialising a feature: note that this function does introduce
// some bias (we tend to overestimate the distance to the feature)
int ThreeD_Head_Point_Feature_Measurement_Model::
               func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
							    Hor_Matrix *xp)
{
  assert(hi != NULL && xp != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  double hi_len = Ii / (2.0 * tan( matel(hi, 3, 1) ) );  // distance
  double se = sin(matel(hi, 2, 1));
  double ce = cos(matel(hi, 2, 1));
  double salpha = sin(matel(hi, 1, 1));
  double calpha = cos(matel(hi, 1, 1));
  double sgamma = sin(matel(hi, 3, 1));
  double dhi_dgamma = -Ii / (2.0 * sgamma * sgamma);

  // Extract cartesian and quaternion parts from 3D position state
  threed_motion_model->func_x(xp);
  threed_motion_model->func_q(xp);

  // Calculate vector from robot to feature in world frame
  R_from_q(threed_motion_model->qRES, RWR);
  
  hor_matq_copy(RWR, dyi_by_dhiL);
                                     
  hor_matq_fill(dhiL_by_dhi,
	hi_len * ce * calpha, -hi_len * se * salpha, dhi_dgamma * ce * salpha,
	0.0,                   hi_len * ce,          dhi_dgamma * se,
       -hi_len * ce * salpha, -hi_len * se * calpha, dhi_dgamma * ce * calpha);

  hor_matq_prod2(dyi_by_dhiL, dhiL_by_dhi, dyi_by_dhiRES);

  hor_matq_fill(hiL,
		hi_len * ce * salpha,
		hi_len * se,
		hi_len * ce * calpha);


  matel(hiL, 2, 1) += Hh; // Add height of head
  hor_matq_prod2(RWR, hiL, hiLW);

  hor_matq_add2(threed_motion_model->xRES, hiLW, yiRES);

  // dyi_by_dxpRES has 2 parts
  // dyi_by_dx and dyi_by_dq

  hor_matq_identity(dyi_by_dx);
  dRq_times_a_by_dq(threed_motion_model->qRES, hiL, dyi_by_dq);

  hor_matq_insert_chunkyx(dyi_by_dx, dyi_by_dxpRES, 0, 0);
  hor_matq_insert_chunkyx(dyi_by_dq, dyi_by_dxpRES, 0, 3);

  // Set the measurement noise
  func_Ri(hi);

  return 0;
}

int ThreeD_Head_Point_Feature_Measurement_Model::
             func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
						   Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  // This function gives relative position of feature
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // hiL is the position of the feature relative to the head centre
  hor_matq_fill(hiL,
		matel(zeroedyiRES, 1, 1),
		matel(zeroedyiRES, 2, 1) - Hh,
		matel(zeroedyiRES, 3, 1));

  double hi_rho = sqrt(  matel(hiL, 1, 1) * matel(hiL, 1, 1)
		  + matel(hiL, 3, 1) * matel(hiL, 3, 1) );
  double hi_len = sqrt (  hi_rho * hi_rho 
		   + matel(hiL, 2, 1) * matel(hiL, 2, 1) );

  /* Now work out the actual predicted measurement */
  hor_matq_fill(hiRES,
		atan2(matel(hiL, 1, 1), matel(hiL, 3, 1)),
		atan2(matel(hiL, 2, 1), hi_rho),
		atan2(Ii / 2.0, hi_len));
    
  /* Now calculate dhi_by_dxp and dhi_by_dyi, the Jacobians we need */
  double tmp = -2.0 * Ii / ( (4.0 * hi_len * hi_len + Ii * Ii) * hi_len);
  hor_matq_fill(dhi_by_dhiL,
        matel(hiL, 3, 1) / (hi_rho * hi_rho), 0.0, 
       -matel(hiL, 1, 1) / (hi_rho * hi_rho),
       -matel(hiL, 1, 1) * matel(hiL, 2, 1) / (hi_len * hi_len * hi_rho), 
        hi_rho / (hi_len * hi_len), 
       -matel(hiL, 3, 1) * matel(hiL, 2, 1) / (hi_len * hi_len * hi_rho), 
        tmp * matel(hiL, 1, 1), tmp * matel(hiL, 2, 1), 
        tmp * matel(hiL, 3, 1));

  hor_matq_prod2(dhi_by_dhiL, dzeroedyi_by_dxpRES, dhi_by_dxpRES);
  hor_matq_prod2(dhi_by_dhiL, dzeroedyi_by_dyiRES, dhi_by_dyiRES);
  
  return 0;
}

int ThreeD_Head_Point_Feature_Measurement_Model::func_nui(Hor_Matrix *hi, 
							  Hor_Matrix *zi)
{
  hor_matq_sub(zi, hi, nuiRES);
  
  for (int i = 0; i < MEASUREMENT_SIZE; i++)
    and_pi_range(nuiRES->m[i][0]);

  return 0;
}

int ThreeD_Head_Point_Feature_Measurement_Model::func_Ri(Hor_Matrix *hi)
{
  assert(hi != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  // In this measurement model, Ri doesn't depend on hi
  hor_matq_diagonal(RiRES, 
		    SD_alpha_filter * SD_alpha_filter,
		    SD_e_filter * SD_e_filter,
		    SD_gamma_filter * SD_gamma_filter);

  return 0;
}

// In this particular noisy measurement function we just add
// Gaussian noise to the true measurement, but having this function
// separate provides the generality for different noise models
int ThreeD_Head_Point_Feature_Measurement_Model::
             func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true)
{
  assert(yi_true != NULL && xp_true != NULL);

  assert(yi_true->rows == FEATURE_STATE_SIZE && yi_true->cols == 1 &&
	 xp_true->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_true->cols == 1);

  func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi_true, xp_true);

  hor_matq_fill(hi_noisyRES,
		hor_gauss_rand(matel(hiRES, 1, 1), SD_alpha),
		hor_gauss_rand(matel(hiRES, 2, 1), SD_e),
		hor_gauss_rand(matel(hiRES, 3, 1), SD_gamma));

  return 0;
}

// Calculate position of feature relative to robot in robot frame
int ThreeD_Head_Point_Feature_Measurement_Model::
          func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		                          Hor_Matrix *yi, Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  // Extract cartesian and quaternion components of xp
  threed_motion_model->func_x(xp);
  threed_motion_model->func_q(xp);

  // y - x in world frame
  hor_matq_sub(yi, threed_motion_model->xRES, yminusxW);

  invert_quaternion(threed_motion_model->qRES, qRW);
  dqbar_by_dq(dqRW_by_dq);

  // Rotation RRW
  R_from_q(qRW, RRW);

  // Position of feature relative to robot in robot frame
  hor_matq_prod2(RRW, yminusxW, zeroedyiRES);

  // Now calculate Jacobians
  // dzeroedyi_by_dyi is RRW
  hor_matq_copy(RRW, dzeroedyi_by_dyiRES);

  // dzeroedyi_by_dxp has 2 partitions:
  // dzeroedyi_by_dx (3 * 3)
  // dzeroedyi_by_dq (3 * 4)
  hor_matq_copy(RRW, dzeroedyi_by_dx);
  hor_matq_scale(dzeroedyi_by_dx, -1.0);

  dRq_times_a_by_dq(qRW, yminusxW, dzeroedyi_by_dqRW);
  hor_matq_prod2(dzeroedyi_by_dqRW, dqRW_by_dq, dzeroedyi_by_dq);

  hor_matq_insert_chunkyx(dzeroedyi_by_dx, dzeroedyi_by_dxpRES, 0, 0);
  hor_matq_insert_chunkyx(dzeroedyi_by_dq, dzeroedyi_by_dxpRES, 0, 3);

  return 0;
}

// This function is not really very efficient!
int ThreeD_Head_Point_Feature_Measurement_Model::visibility_test(
                                                     Hor_Matrix *xp, 
						     Hor_Matrix *yi,
						     Hor_Matrix *xp_orig, 
						     Hor_Matrix *hi)
{
  assert(xp != NULL && xp_orig != NULL && hi != NULL);

  assert(xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1 &&
	 yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp_orig->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_orig->cols == 1 &&
	 hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  int cant_see_flag = 0;

  /* Test axis range limit */
  if ( matel(hi, 1, 1) > PAN_RANGE_LIMIT || 
       matel(hi, 1, 1) < -PAN_RANGE_LIMIT ) {
    cant_see_flag |= PAN_RANGE_FAIL;
    // cout << "Pan fail." << endl;
  }

  if ( matel(hi, 2, 1) > ELEVATION_RANGE_LIMIT || 
       matel(hi, 2, 1) < -ELEVATION_RANGE_LIMIT ) {
    cant_see_flag |= ELEVATION_RANGE_FAIL;
    // cout << "Elevation fail." << endl;
  }

  /* Do tests on length and angle of predicted view */

  // hiLlab is current predicted vector from head to feature in 
  // world frame

  // This function gives relative position of feature
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // hiL is the position of the feature relative to the head centre
  hor_matq_fill(hiL,
		matel(zeroedyiRES, 1, 1),
		matel(zeroedyiRES, 2, 1) - Hh,
		matel(zeroedyiRES, 3, 1));

  threed_motion_model->func_q(xp);
  R_from_q(threed_motion_model->qRES, MlabC0);

  hor_matq_prod2(MlabC0, hiL, hiLlab);

  // hiL_origlab is vector from head to feature in world frame
  // WHEN THAT FEATURE WAS FIRST MEASURED: i.e. when the image
  // patch was saved
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp_orig);

  hor_matq_fill(hiL_orig,
		matel(zeroedyiRES, 1, 1),
		matel(zeroedyiRES, 2, 1) - Hh,
		matel(zeroedyiRES, 3, 1));

  threed_motion_model->func_q(xp_orig);
  R_from_q(threed_motion_model->qRES, MlabC0_orig);

  hor_matq_prod2(MlabC0_orig, hiL_orig, hiL_origlab);

  // Compare hiLlab and hiL_origlab for length and orientation
  double mod_hiLlab = sqrt( hor_matq_dot31(hiLlab, hiLlab) );
  double mod_hiL_origlab = sqrt( hor_matq_dot31(hiL_origlab, hiL_origlab) );

  double length_ratio = mod_hiLlab / mod_hiL_origlab;
  if ( length_ratio > MAXIMUM_LENGTH_RATIO || 
       length_ratio < (1.0 / MAXIMUM_LENGTH_RATIO) ) {
    cant_see_flag |= DISTANCE_FAIL;
    // cout << "Distance fail." << endl;
  }

  double dot_prod = hor_matq_dot31(hiLlab, hiL_origlab);

  double angle = acos(dot_prod / (mod_hiLlab * mod_hiL_origlab));
  angle = (angle >= 0.0 ? angle : -angle);  // Make angle positive
  if (angle > MAXIMUM_ANGLE_DIFFERENCE) {
    cant_see_flag |= ANGLE_FAIL;
    // cout << "Angle fail." << endl;
  }

  return cant_see_flag;   // 0 if OK, otherwise error code
}

// Give a score which says how important it is to measure this feature
double ThreeD_Head_Point_Feature_Measurement_Model::
       selection_score(Hor_Matrix *Si)
{
  // Calculate the volume of uncertainty ellipsoid
  double eigenvalues[3];
  double spare_array[3];

  hor_matq_copy(Si, Temp_MM1);

  hor_matq_tred2(Temp_MM1, eigenvalues, spare_array);
  hor_matq_tqli(eigenvalues, spare_array, Temp_MM1);
    
  hor_matq_eigsrt(eigenvalues, Temp_MM1);

  return (4.0 / 3.0) * M_PI // * NO_SIGMA * NO_SIGMA * NO_SIGMA 
             * sqrt(eigenvalues[0] * eigenvalues[1] * eigenvalues[2]);
}

