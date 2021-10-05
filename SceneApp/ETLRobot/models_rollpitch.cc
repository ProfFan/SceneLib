/*  SceneApp: applications for sequential localisation and map-building

    models_rollpitch.cc
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
#include "models_rollpitch.h"

Roll_Pitch_Sensor_Internal_Measurement_Model::
Roll_Pitch_Sensor_Internal_Measurement_Model(Motion_Model *m_m)
  : Internal_Measurement_Model(2, m_m, "ROLL_PITCH")
{
  assert(strcmp(motion_model->motion_model_dimensionality_type, "THREED") 
	 == 0);
  threed_motion_model = (ThreeD_Motion_Model *) m_m;
  local_xp = hor_mat_alloc(7, 1);
  qbar = hor_mat_alloc(4, 1);
  dqbarby_dq   = hor_mat_alloc(4, 4);
  Rqbar = hor_mat_alloc(3, 3);
  yWW = hor_mat_alloc(3, 1);
  yWR = hor_mat_alloc(3, 1);
  dhv_by_dxp = hor_mat_alloc(2, 7);
  dyWR_by_dqbar = hor_mat_alloc(3, 4);
  dyWR_by_dq = hor_mat_alloc(3, 4);
  dhv_by_dyWR = hor_mat_alloc(2, 3);
  dhv_by_dq = hor_mat_alloc(2, 4);
}

Roll_Pitch_Sensor_Internal_Measurement_Model::~Roll_Pitch_Sensor_Internal_Measurement_Model()
{
  hor_mat_free_list(local_xp, qbar, dqbarby_dq, Rqbar, yWW, yWR, dhv_by_dxp,
    dyWR_by_dqbar, dyWR_by_dq, dhv_by_dyWR, dhv_by_dq, NULL);
}

// Redefined virtual functions
int Roll_Pitch_Sensor_Internal_Measurement_Model::
    func_hv_and_dhv_by_dxv(Hor_Matrix *xv)
{
  assert(xv != NULL);
  
  assert(xv->rows == motion_model->STATE_SIZE && xv->cols == 1);

  // Elements of measurement vector are thetaR (roll) and thetaP (pitch)

  // Extract quaternion from state
  motion_model->func_xp(xv);
  hor_matq_copy(motion_model->xpRES, local_xp);
  threed_motion_model->func_q(local_xp);


  // Invert quaternion to give rotation RRW (to take world vector into 
  // robot frame
  invert_quaternion(threed_motion_model->qRES, qbar);
  dqbar_by_dq(dqbarby_dq);

  // Calculate what world y axis is like in robot frame (this is -gravity)
  hor_matq_fill(yWW, 0.0, 1.0, 0.0);
  R_from_q(qbar, Rqbar);
  hor_matq_prod2(Rqbar, yWW, yWR);

  // Now we can calculate roll and pitch measurements
  double thetaR = atan2(vecel(yWR, 1), vecel(yWR, 2));
  double thetaP = atan2(-vecel(yWR, 3), vecel(yWR, 2));
  hor_matq_fill(hvRES, thetaR, thetaP);

  // Finish working out Jacobian
  dRq_times_a_by_dq(qbar, yWW, dyWR_by_dqbar);
  hor_matq_prod2(dyWR_by_dqbar, dqbarby_dq, dyWR_by_dq);

  // For clarity
  double &X = vecel(yWR, 1);
  double &Y = vecel(yWR, 2);
  double &Z = vecel(yWR, 3);

  // thetaR = arctan X/Y, thetaP = arctan -Z/Y

  double A = 1.0 / (1.0 + X*X / (Y*Y));
  double B = 1.0 / (1.0 + Z*Z / (Y*Y));
  hor_matq_fill(dhv_by_dyWR,
		A / Y,   -A * X / (Y*Y),   0.0,
		0.0,     B * Z / (Y*Y),    -A / Y);

  hor_matq_prod2(dhv_by_dyWR, dyWR_by_dq, dhv_by_dq);

  hor_matq_zero(dhv_by_dxp);
  hor_matq_insert_chunkyx(dhv_by_dq, dhv_by_dxp, 0, 3);

  motion_model->func_dxp_by_dxv(xv);
  hor_matq_prod2(dhv_by_dxp, motion_model->dxp_by_dxvRES, dhv_by_dxvRES);

  return 0;
}

int Roll_Pitch_Sensor_Internal_Measurement_Model::func_Rv(Hor_Matrix *hv)
{
  assert(hv != NULL);

  assert(hv->rows == MEASUREMENT_SIZE && hv->cols == 1);

  // Constant, diagonal measurement noise
  hor_matq_fill(RvRES, 
		SD_angle_filter * SD_angle_filter, 0.0,
		0.0, SD_angle_filter * SD_angle_filter);

  return 0;
}

int Roll_Pitch_Sensor_Internal_Measurement_Model::
    func_nuv(Hor_Matrix *hv, Hor_Matrix *zv)
{
  assert(hv != NULL && zv != NULL);

  assert(hv->rows == MEASUREMENT_SIZE && hv->cols == 1 &&
	 zv->rows == MEASUREMENT_SIZE && zv->cols == 1);

  // Innovation is just measurement minus prediction
  // though they are angles so make sure they are in range -pi -> pi
  hor_matq_sub(zv, hv, nuvRES);
  and_pi_range(vecel(nuvRES, 1));
  and_pi_range(vecel(nuvRES, 2));

  return 0;
}

int Roll_Pitch_Sensor_Internal_Measurement_Model::
    func_hv_noisy(Hor_Matrix *xv_true)
{
  assert(xv_true != NULL);
  
  assert(xv_true->rows == motion_model->STATE_SIZE && xv_true->cols == 1);


  // In this particular noisy measurement function we just add
  // Gaussian noise to the true measurement
  func_hv_and_dhv_by_dxv(xv_true);

  hor_matq_fill(hv_noisyRES,
		hor_gauss_rand(vecel(hvRES, 1), SD_angle),
		hor_gauss_rand(vecel(hvRES, 2), SD_angle));

  return 0;
}

int Roll_Pitch_Sensor_Internal_Measurement_Model::
    feasibility_test(Hor_Matrix *xv, Hor_Matrix *hv)
{
  assert(xv != NULL && hv != NULL);
  
  assert(xv->rows == motion_model->STATE_SIZE && xv->cols == 1 &&
	 hv->rows == MEASUREMENT_SIZE && hv->cols == 1);

  // It is always OK to make this measurement so just return 0
  return 0;
}
