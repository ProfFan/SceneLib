/*  SceneApp: applications for sequential localisation and map-building
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "general_headers.h"
#include "models_base.h"
#include "models_oned.h"

// Constructor where we allocate matrices, etc.
Simple_OneD_Motion_Model::Simple_OneD_Motion_Model()
  : OneD_Motion_Model(1, 1, "SIMPLE_ONED"),
                          /* In this motion model, STATE_SIZE = 1,
			     CONTROL_SIZE = 1 */
    SD_frac_v_filter(0.1), 
    SD_frac_v(0.1)
{  
  u_noisy = hor_mat_alloc(CONTROL_SIZE, 1);
}

// Destructor
Simple_OneD_Motion_Model::~Simple_OneD_Motion_Model()
{
  hor_mat_free_list(u_noisy, NULL);
  Motion_Model_destructor();
}

// Calculate new state vector and Jacobian
int Simple_OneD_Motion_Model::func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
					       Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // New state vector fv
  hor_matq_copy(xv, fvRES);         // Makes sense to do with this motion model
  matel(fvRES, 1, 1) += matel(u, 1, 1) * delta_t; // x += v * delta_t

  // Jacobian dfv_by_dxv
  hor_matq_identity(dfv_by_dxvRES);

  return 0;
}

// Calculate process noise
int Simple_OneD_Motion_Model::func_Q(Hor_Matrix *xv, Hor_Matrix *u, 
					double delta_t)
{
  assert(xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  double sigma = SD_frac_v_filter * matel(u, 1, 1) * delta_t;
  matel(QxRES, 1, 1) = sigma * sigma;

  return 0;
}

// Calculate position state from total state
int Simple_OneD_Motion_Model::func_xp(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_fill(xpRES, matel(xv, 1, 1));

  return 0;
}

int Simple_OneD_Motion_Model::func_dxp_by_dxv(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_identity(dxp_by_dxvRES);

  return 0;
}

// Noisy process equation for simulation
// For this particular model we simply perturb u with Gaussian noise
// and send it through func_fv
// Want to be able to synthesise different types of noise as well
int Simple_OneD_Motion_Model::func_fv_noisy(Hor_Matrix *xv_true, 
					 Hor_Matrix *u_true, double delta_t)
{
  assert (xv_true != NULL && u_true != NULL);

  assert (xv_true->rows == STATE_SIZE && xv_true->cols == 1 &&
	  u_true->rows == CONTROL_SIZE && u_true->cols == 1);

  double SD_v = SD_frac_v * matel(u_true, 1, 1);

  hor_matq_fill(u_noisy,
		hor_gauss_rand(matel(u_true, 1, 1), SD_v));

  func_fv_and_dfv_by_dxv(xv_true, u_noisy, delta_t);

  hor_matq_copy(fvRES, fv_noisyRES);

  return 0;
}

int Simple_OneD_Motion_Model::
    func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef)
{
  assert (xv != NULL && xpdef != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(xpRES);

  func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(local_xp, xpdef);

  // In this motion model xv is the same as xp so just copy the results
  hor_matq_copy(xpredefRES, xvredefRES);

  hor_matq_copy(dxpredef_by_dxpRES, dxvredef_by_dxvRES);

  hor_matq_copy(dxpredef_by_dxpdefRES, dxvredef_by_dxpdefRES);

  hor_mat_free_list(local_xp, NULL);

  return 0;
}

// Function to set the control vector u in order to go towards a target
// waypoint.
int Simple_OneD_Motion_Model::navigate_to_waypoint(Hor_Matrix *xv, 
	         Hor_Matrix *xv_goal, Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && xv_goal != NULL && u != NULL);
  
  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xv_goal->rows == STATE_SIZE && xv_goal->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Stop when we are this close
  const double ARRIVED_DISTANCE_THRESHOLD = 0.02;
  // Usual speed
  const double USUAL_SPEED = 0.1;

  double dist = matel(xv_goal, 1, 1) - matel(xv, 1, 1);

  if (fabs(dist) <= ARRIVED_DISTANCE_THRESHOLD) // We are there
    return 1;

  if (fabs(dist) < 2 * USUAL_SPEED * delta_t)
    matel(u, 1, 1) = 0.5 * dist / delta_t;
  else 
    matel(u, 1, 1) = dist >= 0 ? USUAL_SPEED : -USUAL_SPEED;

  return 0;
}


// Point feature for 1D motion, constant measurement noise model
OneD_Point_Feature_Measurement_Model::OneD_Point_Feature_Measurement_Model(
						    Motion_Model *m_m)
  : Point_Feature_Measurement_Model(1, 1, m_m, "SIMPLE_ONED_POINT"),
    // Initialise constants

    // Measurement errors for simulation
    SD_h(0.09),               /* distance measurement */

    // Measurement errors to use in filter
    SD_h_filter(0.09)         /* distance measurement */
{
  assert(strcmp(motion_model->motion_model_dimensionality_type, "ONED") == 0);
}

OneD_Point_Feature_Measurement_Model::~OneD_Point_Feature_Measurement_Model()
{
  Feature_Measurement_Model_destructor();
}

// Defines the pose of a point for 3D graphics
int OneD_Point_Feature_Measurement_Model::func_yipose_and_Pyiyipose(Hor_Matrix *yi, 
						Hor_Matrix *Pyiyi)
{
  assert(yi != NULL && Pyiyi != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 Pyiyi->rows == FEATURE_STATE_SIZE && 
	 Pyiyi->cols == FEATURE_STATE_SIZE);

  hor_matq_fill(yiposeRES, 0.0, 1.0, matel(yi, 1, 1));

  hor_matq_zero(PyiyiposeRES);
  matel(PyiyiposeRES, 3, 3) = matel(Pyiyi, 1, 1);

  return 0;
}

int OneD_Point_Feature_Measurement_Model::
               func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
							    Hor_Matrix *xp)
{
  assert(hi != NULL && xp != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  hor_matq_identity(dyi_by_dhiRES);
  hor_matq_identity(dyi_by_dxpRES);
  hor_matq_add2(xp, hi, yiRES);
	   
  func_Ri(hi);

  return 0;
}

int OneD_Point_Feature_Measurement_Model::
             func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
						   Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  // In this model, measurements are just the Euclidean distance
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  hor_matq_copy(zeroedyiRES, hiRES);
  hor_matq_copy(dzeroedyi_by_dxpRES, dhi_by_dxpRES);
  hor_matq_copy(dzeroedyi_by_dyiRES, dhi_by_dyiRES);

  return 0;
}


int OneD_Point_Feature_Measurement_Model::func_Ri(Hor_Matrix *hi)
{
  assert(hi != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  hor_matq_diagonal(RiRES, SD_h_filter * SD_h_filter);

  return 0;
}

int OneD_Point_Feature_Measurement_Model::func_nui(Hor_Matrix *hi, Hor_Matrix *zi)
{
  hor_matq_sub(zi, hi, nuiRES);
  
  return 0;
}


// In this particular noisy measurement function we just add
// Gaussian noise to the true measurement, but having this function
// separate provides the generality for different noise models
int OneD_Point_Feature_Measurement_Model::
             func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true)
{
  assert(yi_true != NULL && xp_true != NULL);

  assert(yi_true->rows == FEATURE_STATE_SIZE && yi_true->cols == 1 &&
	 xp_true->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_true->cols == 1);

  func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi_true, xp_true);

  hor_matq_fill(hi_noisyRES,
		hor_gauss_rand(matel(hiRES, 1, 1), SD_h));

  return 0;
}

int OneD_Point_Feature_Measurement_Model::
          func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		                          Hor_Matrix *yi, Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  /* Now work out the actual predicted measurement */
  hor_matq_sub(yi, xp, zeroedyiRES);

  hor_matq_fill(dzeroedyi_by_dxpRES, -1.0);
  hor_matq_fill(dzeroedyi_by_dyiRES, 1.0);

  return 0;
}

// These features are visible within a limited range
const double VISIBILITY_RANGE = 4.0;
int OneD_Point_Feature_Measurement_Model::visibility_test(Hor_Matrix *xp, 
						     Hor_Matrix *yi,
						     Hor_Matrix *xp_orig, 
						     Hor_Matrix *hi)
{
  assert(xp != NULL && xp_orig != NULL && hi != NULL);

  assert(xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1 &&
	 xp_orig->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_orig->cols == 1 &&
	 hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  if ( fabs(matel(hi, 1, 1)) <= VISIBILITY_RANGE )
    return 0;
  else
    return 1;
}

// Give a score which says how important it is to measure this feature
double OneD_Point_Feature_Measurement_Model::selection_score(Hor_Matrix *Si)
{
  // Simply return innovation covariance
  return matel(Si, 1, 1);
}
