/*  Scene: software for sequential localisation and map-building

    Camera/models_camera.cc
    Copyright (C) 2001 Andrew Davison
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
#include "models_camera.h"

/// Model for single camera making point measurements

Camera_Point_Feature_Measurement_Model::
Camera_Point_Feature_Measurement_Model(Motion_Model *m_m)
  : Point_Feature_Measurement_Model(2, 3, m_m, "CAMERA_POINT")
{
  assert(strcmp(motion_model->motion_model_dimensionality_type, "THREED") 
	 == 0);

  threed_motion_model = (ThreeD_Motion_Model *) motion_model;

  // Camera calibration matrix and inverse
  C = hor_mats_fill(3, 3,
		    -Fku,       0.0,        (double) U0,
		     0.0,       -Fkv,       (double) V0,
		     0.0,        0.0,         1.0  );
  Cinv = hor_mats_fill(3, 3,
		       -1.0 / Fku, 0.0,        (double) U0 / Fku,
			0.0,       -1.0 / Fkv, (double) V0 / Fkv,
			0.0,        0.0,         1.0  );

  // Calculation matrices
  du_by_dhR = hor_mat_alloc(2, 3);

  yminusxW = hor_mat_alloc(3, 1);
  qRW = hor_mat_alloc(4, 1);
  RRW = hor_mat_alloc(3, 3);
  dzeroedyi_by_dx = hor_mat_alloc(3, 3);
  dzeroedyi_by_dq = hor_mat_alloc(3, 4);
  dqRW_by_dq = hor_mat_alloc(4, 4);
  dzeroedyi_by_dqRW = hor_mat_alloc(3, 4);

  MlabC0 = hor_mat_alloc(3, 3);
  hiLlab = hor_mat_alloc(3, 1);
  hiL_origlab = hor_mat_alloc(3, 1);
  MlabC0_orig = hor_mat_alloc(3, 3);
}

Camera_Point_Feature_Measurement_Model::
~Camera_Point_Feature_Measurement_Model()
{
  hor_mat_free_list(C, Cinv, 
		    du_by_dhR,
		    yminusxW, qRW, dqRW_by_dq, RRW, dzeroedyi_by_dx, 
		    dzeroedyi_by_dqRW, dzeroedyi_by_dq,
		    hiLlab, MlabC0, hiL_origlab, MlabC0_orig,
		    NULL);
}

// Redefined virtual functions
int Camera_Point_Feature_Measurement_Model::
func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi)
{
  assert(yi != NULL && Pyiyi != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 Pyiyi->rows == FEATURE_STATE_SIZE && 
	 Pyiyi->cols == FEATURE_STATE_SIZE);

  hor_matq_copy(yi, yiposeRES);
  hor_matq_copy(Pyiyi, PyiyiposeRES);

  return 0; 
}

int Camera_Point_Feature_Measurement_Model::
func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, Hor_Matrix *xp)
{
  assert(hi != NULL && xp != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  cerr << "Initialsing feature not yet implemented." << endl;
  assert(0);

  return 0;
}

int Camera_Point_Feature_Measurement_Model::
func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, Hor_Matrix *xp)
{
  assert(yi != NULL && xp != NULL);

  assert(yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1);

  // This function gives relative position of feature: also call this hR
  // (vector from camera to feature in robot frame)
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  // Labels for clarity
  double &hRx = vecel(zeroedyiRES, 1);
  double &hRy = vecel(zeroedyiRES, 2);
  double &hRz = vecel(zeroedyiRES, 3);

  double &u = vecel(hiRES, 1);
  double &v = vecel(hiRES, 2);

  // Image measurement: note minus signs put us in normal image coordinates
  // with (0, 0) at top left
  u = U0 - Fku * (hRx / hRz);
  v = V0 - Fkv * (hRy / hRz);

  // Jacobians
  hor_matq_fill(du_by_dhR,
               -Fku / hRz,   0.0,          Fku * hRx / (hRz * hRz),
		0.0,        -Fkv / hRz,    Fkv * hRy / (hRz * hRz));

  hor_matq_prod2(du_by_dhR, dzeroedyi_by_dxpRES, dhi_by_dxpRES);
  hor_matq_prod2(du_by_dhR, dzeroedyi_by_dyiRES, dhi_by_dyiRES);

  return 0;
}

int Camera_Point_Feature_Measurement_Model::func_Ri(Hor_Matrix *hi)
{
  assert(hi != NULL);

  assert(hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  // In this measurement model, Ri doesn't depend on hi
  hor_matq_diagonal(RiRES, 
		    SD_image_filter * SD_image_filter,
		    SD_image_filter * SD_image_filter);

  return 0;
}

int Camera_Point_Feature_Measurement_Model::
func_nui(Hor_Matrix *hi, Hor_Matrix *zi)
{
  hor_matq_sub(zi, hi, nuiRES);
  
  return 0;

}

int Camera_Point_Feature_Measurement_Model::
func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true)
{
  assert(yi_true != NULL && xp_true != NULL);

  assert(yi_true->rows == FEATURE_STATE_SIZE && yi_true->cols == 1 &&
	 xp_true->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_true->cols == 1);

  func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi_true, xp_true);

  hor_matq_fill(hi_noisyRES,
		hor_gauss_rand(matel(hiRES, 1, 1), SD_image),
		hor_gauss_rand(matel(hiRES, 2, 1), SD_image));

  return 0;
}

// This function is general for any point measurement model in 3D so
// that points to something wrong in derivation hierarchy...
int Camera_Point_Feature_Measurement_Model::
func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(Hor_Matrix *yi, 
							Hor_Matrix *xp)
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

// Don't try to search this close to the edge of the image
const double IMAGE_SEARCH_BOUNDARY = 20.0;

int Camera_Point_Feature_Measurement_Model::
visibility_test(Hor_Matrix *xp, Hor_Matrix *yi, 
		Hor_Matrix *xp_orig, Hor_Matrix *hi)
{
  assert(xp != NULL && xp_orig != NULL && hi != NULL);

  assert(xp->rows == motion_model->POSITION_STATE_SIZE && xp->cols == 1 &&
	 yi->rows == FEATURE_STATE_SIZE && yi->cols == 1 &&
	 xp_orig->rows == motion_model->POSITION_STATE_SIZE && 
	 xp_orig->cols == 1 &&
	 hi->rows == MEASUREMENT_SIZE && hi->cols == 1);

  int cant_see_flag = 0;

  /* Test image boundaries */
  if ( vecel(hi, 1) < 0.0 + IMAGE_SEARCH_BOUNDARY || 
       vecel(hi, 1) >  (double) (IMAGE_WIDTH - 1 - IMAGE_SEARCH_BOUNDARY)) {
    cant_see_flag |= LEFT_RIGHT_FAIL;
    // cout << "Left / right fail." << endl;
  }
  if ( vecel(hi, 2) < 0.0 + IMAGE_SEARCH_BOUNDARY || 
       vecel(hi, 2) >  (double) (IMAGE_HEIGHT - 1 - IMAGE_SEARCH_BOUNDARY)) {
    cant_see_flag |= UP_DOWN_FAIL;
    // cout << "Up / down fail." << endl;
  }


  /* Do tests on length and angle of predicted view */

  // hiLlab is current predicted vector from head to feature in 
  // world frame

  // This function gives relative position of feature
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

  threed_motion_model->func_q(xp);
  R_from_q(threed_motion_model->qRES, MlabC0);

  hor_matq_prod2(MlabC0, zeroedyiRES, hiLlab);

  // hiL_origlab is vector from head to feature in world frame
  // WHEN THAT FEATURE WAS FIRST MEASURED: i.e. when the image
  // patch was saved
  func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp_orig);

  threed_motion_model->func_q(xp_orig);
  R_from_q(threed_motion_model->qRES, MlabC0_orig);

  hor_matq_prod2(MlabC0_orig, zeroedyiRES, hiL_origlab);

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

double Camera_Point_Feature_Measurement_Model::selection_score(Hor_Matrix *Si)
{
  // Return the trace of the innovation covariance... (???)
  return (hor_trace(Si));
}
