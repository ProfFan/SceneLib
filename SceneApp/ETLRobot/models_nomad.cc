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
#include "models_nomad.h"

// Constructor where we allocate matrices, etc.
Turn_Move_TwoD_Motion_Model::Turn_Move_TwoD_Motion_Model()
  : TwoD_Motion_Model(5, 3, "TURN_MOVE_TWOD"),
                          /* In this motion model, STATE_SIZE = 5,
			     CONTROL_SIZE = 3 */
    SD_frac_v_filter(0.14), 
    SD_frac_dS_filter(0.02), 
    SD_frac_dT_filter(0.02),
    SD_frac_v(0.14), 
    SD_frac_dS(0.02), 
    SD_frac_dT(0.02)
{
  Qu = hor_mat_alloc(CONTROL_SIZE, CONTROL_SIZE);
  dfv_by_du = hor_mat_alloc(STATE_SIZE, CONTROL_SIZE);
  dfv_by_duT = hor_mat_alloc(CONTROL_SIZE, STATE_SIZE);
  Temp_STATE_CONTROL = hor_mat_alloc(STATE_SIZE, CONTROL_SIZE);
  u_noisy = hor_mat_alloc(CONTROL_SIZE, 1);
}

// Destructor
Turn_Move_TwoD_Motion_Model::~Turn_Move_TwoD_Motion_Model()
{
  hor_mat_free_list(Qu, dfv_by_du, dfv_by_duT, Temp_STATE_CONTROL, u_noisy,
		    NULL);
  Motion_Model_destructor();
}

// Calculate new state vector and Jacobian
int Turn_Move_TwoD_Motion_Model::func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
					       Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // New state vector fv
  hor_matq_copy(xv, fvRES);         // Makes sense to do with this motion model
  double A = matel(u, 1, 1) * delta_t;  // v.delta_t
  double B = A * matel(xv, 5, 1);       // v.Fv.delta_t

  double SdS = matel(xv, 3, 1) + matel(u, 2, 1); // S + delta S

  matel(fvRES, 1, 1) += B * cos(SdS); // z
  matel(fvRES, 2, 1) += B * sin(SdS); // x

  matel(fvRES, 3, 1) += matel(u, 2, 1); // s
  matel(fvRES, 4, 1) += matel(u, 3, 1); // T
    
  and_pi_range(matel(fvRES, 3, 1));   // Keep these angles in -PI -> PI range
  and_pi_range(matel(fvRES, 4, 1));  

  // Jacobian dfv_by_dxv
  hor_matq_identity(dfv_by_dxvRES);  // Start from this 
  matel(dfv_by_dxvRES, 1, 3) = -B * sin(SdS);
  matel(dfv_by_dxvRES, 1, 5) = A * cos(SdS);
  matel(dfv_by_dxvRES, 2, 3) = B * cos(SdS);
  matel(dfv_by_dxvRES, 2, 5) = A * sin(SdS);

  return 0;
}

// Calculate process noise
int Turn_Move_TwoD_Motion_Model::func_Q(Hor_Matrix *xv, Hor_Matrix *u, 
					double delta_t)
{
  assert(xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  hor_matq_zero(Qu);
  matel(Qu, 1, 1) = SD_frac_v_filter * SD_frac_v_filter * matel(u, 1, 1)
                                                        * matel(u, 1, 1);
  matel(Qu, 2, 2) = SD_frac_dS_filter * SD_frac_dS_filter * matel(u, 2, 1)
                                                          * matel(u, 2, 1);
  matel(Qu, 3, 3) = SD_frac_dT_filter * SD_frac_dT_filter * matel(u, 3, 1)
                                                          * matel(u, 3, 1);

  hor_matq_zero(dfv_by_du);
  double A = matel(xv, 5, 1) * delta_t;  // Fv.delta_t
  double B = matel(u, 1, 1) * A;         // v.Fv.delta_t

  double SdS = matel(xv, 3, 1) + matel(u, 2, 1); // S + delta S

  matel(dfv_by_du, 1, 1) = A * cos(SdS);
  matel(dfv_by_du, 1, 2) = -B * sin(SdS);
  matel(dfv_by_du, 2, 1) = A * sin(SdS);
  matel(dfv_by_du, 2, 2) = B * cos(SdS);
  matel(dfv_by_du, 3, 2) = 1.0;
  matel(dfv_by_du, 4, 3) = 1.0;

  hor_matq_transpose(dfv_by_du, dfv_by_duT);

  hor_matq_prod2(dfv_by_du, Qu, Temp_STATE_CONTROL);
  hor_matq_prod2(Temp_STATE_CONTROL, dfv_by_duT, QxRES);

  return 0;
}

// Calculate position state from total state
int Turn_Move_TwoD_Motion_Model::func_xp(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_fill(xpRES, 
		matel(xv, 1, 1),
		matel(xv, 2, 1),
		matel(xv, 4, 1));

  return 0;
}

int Turn_Move_TwoD_Motion_Model::func_dxp_by_dxv(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_zero(dxp_by_dxvRES);
  matel(dxp_by_dxvRES, 1, 1) = 1.0;
  matel(dxp_by_dxvRES, 2, 2) = 1.0;
  matel(dxp_by_dxvRES, 3, 4) = 1.0;

  return 0;
}

// Noisy process equation for simulation
// For this particular model we simply perturb u with Gaussian noise
// and send it through func_fv
int Turn_Move_TwoD_Motion_Model::func_fv_noisy(Hor_Matrix *xv_true, 
					 Hor_Matrix *u_true, double delta_t)
{
  assert (xv_true != NULL && u_true != NULL);

  assert (xv_true->rows == STATE_SIZE && xv_true->cols == 1 &&
	  u_true->rows == CONTROL_SIZE && u_true->cols == 1);

  double SD_v = SD_frac_v * matel(u_true, 1, 1);
  double SD_dS = SD_frac_dS * matel(u_true, 2, 1);
  double SD_dT = SD_frac_dT * matel(u_true, 3, 1); 

  hor_matq_fill(u_noisy,
		hor_gauss_rand(matel(u_true, 1, 1), SD_v),
		hor_gauss_rand(matel(u_true, 2, 1), SD_dS),
		hor_gauss_rand(matel(u_true, 3, 1), SD_dT));

  func_fv_and_dfv_by_dxv(xv_true, u_noisy, delta_t);

  hor_matq_copy(fvRES, fv_noisyRES);

  return 0;
}

int Turn_Move_TwoD_Motion_Model::
    func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef)
{
  assert (xv != NULL && xpdef != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  // We can mainly use the stuff from the general redefinition of axes in
  // position coordinates, but need to change it a bit
  // State is z, x, S, T, Fv
  // Position state is z, x, phi (phi is the same as T)
  //    state  position
  //      1  <=>  1
  //      2  <=>  2
  //      4  <=>  3

  double &phidef = vecel(xpdef, 3);
  double &S0 = vecel(xv, 3);
  double &Fv0 = vecel(xv, 5);

  func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(xpRES);

  func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(local_xp, xpdef);

  hor_matq_fill(xvredefRES,
		vecel(xpredefRES, 1),
		vecel(xpredefRES, 2),
		S0 - phidef,
		vecel(xpredefRES, 3),
		Fv0);

  hor_matq_fill(dxvredef_by_dxvRES,
		matel(dxpredef_by_dxpRES, 1, 1), 
		matel(dxpredef_by_dxpRES, 1, 2), 0.0, 
		matel(dxpredef_by_dxpRES, 1, 3), 0.0,
 
		matel(dxpredef_by_dxpRES, 2, 1), 
		matel(dxpredef_by_dxpRES, 2, 2), 0.0, 
		matel(dxpredef_by_dxpRES, 2, 3), 0.0,
 
		0.0, 0.0, 1.0, 0.0, 0.0,

		matel(dxpredef_by_dxpRES, 3, 1), 
		matel(dxpredef_by_dxpRES, 3, 2), 0.0, 
		matel(dxpredef_by_dxpRES, 3, 3), 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0);

  hor_matq_fill(dxvredef_by_dxpdefRES,
		matel(dxpredef_by_dxpdefRES, 1, 1), 
		matel(dxpredef_by_dxpdefRES, 1, 2),
		matel(dxpredef_by_dxpdefRES, 1, 3),
 
		matel(dxpredef_by_dxpdefRES, 2, 1), 
		matel(dxpredef_by_dxpdefRES, 2, 2),
		matel(dxpredef_by_dxpdefRES, 2, 3),
 
		0.0, 0.0, -1.0,

		matel(dxpredef_by_dxpdefRES, 3, 1), 
		matel(dxpredef_by_dxpdefRES, 3, 2),
		matel(dxpredef_by_dxpdefRES, 3, 3),

		0.0, 0.0, 0.0);

  hor_mat_free_list(local_xp, NULL);

  return 0;
}

// Function to set the control vector u in order to go towards a target
// waypoint. Turning the steering and turret together, this one does a
// big stationary turn first, then moves with a speed which decreases when
// the goal is near. Returns 1 if goal is reached.
// At the moment, steering part of goal vector is ignored
// Fv part is ignored of course too because we cannot control that
int Turn_Move_TwoD_Motion_Model::navigate_to_waypoint(Hor_Matrix *xv, 
	         Hor_Matrix *xv_goal, Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && xv_goal != NULL && u != NULL);
  
  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xv_goal->rows == STATE_SIZE && xv_goal->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Stop when we are this close
  const double ARRIVED_DISTANCE_THRESHOLD = 0.02;
  const double ARRIVED_TURRET_THRESHOLD = 1.0 * M_PI / 180.0;
  // Steer on the spot if we need to turn by more than this
  const double STATIONARY_STEERING_LIMIT = 5.0 * M_PI / 180.0;
  // Only control steering, not turret, if we are within this distance
  const double CLOSE_MANOUEVRE_LIMIT = 1.00;

  // Usual speed
  const double USUAL_SPEED = 0.1;

  double z_diff = vecel(xv_goal, 1) - vecel(xv, 1);
  double x_diff = vecel(xv_goal, 2) - vecel(xv, 2);

  double dist = sqrt (z_diff * z_diff + x_diff * x_diff);
  double angle = atan2 (x_diff, z_diff); // Angle to waypoint in world frame

  if (dist <= ARRIVED_DISTANCE_THRESHOLD) // We are there: finally set turret
  {
    hor_matq_zero(u);

    double turret_diff = vecel(xv_goal, 4) - vecel(xv, 4);
    // and_pi_range (turret_diff);
    if (fabs(turret_diff) > ARRIVED_TURRET_THRESHOLD)
    {
      vecel(u, 3) = turret_diff;     
      return 0;
    }
    else
      return 1; // Arrived: zero movement
  }

  // Work out how much we need to steer to go straight to waypoint
  double S_increment = angle - matel(xv, 3, 1);
  and_pi_range (S_increment); // Puts in range +/- PI
  double T_increment = (dist > CLOSE_MANOUEVRE_LIMIT ? 
                              angle - matel(xv, 4, 1) : 0.0) ;
  // and_pi_range (T_increment);
  and_pi_range (T_increment); // revived by nkita 000630
  double v;

  if (dist < USUAL_SPEED * delta_t)
  {
    v = 0.5 * dist / delta_t;
    T_increment = 0.0; // Don't make large turrent movements when near
  }
  else v = USUAL_SPEED;

  if (fabs(S_increment) > STATIONARY_STEERING_LIMIT || 
      fabs(T_increment) > STATIONARY_STEERING_LIMIT)
    v = 0.0; // Steer on the spot

  matel(u, 1, 1) = v;
  matel(u, 2, 1) = S_increment;
  matel(u, 3, 1) = T_increment;

  return 0;
}


Turn_Move_ThreeD_Motion_Model::Turn_Move_ThreeD_Motion_Model
  (Turn_Move_TwoD_Motion_Model *t_m_t_m_m)
    : ThreeD_Motion_Model(8, 3, "TURN_MOVE_THREED"),
      turn_move_twod_motion_model(t_m_t_m_m)
{
  r = hor_mat_alloc(3, 1);
  q = hor_mat_alloc(4, 1);
  Rq = hor_mat_alloc(3, 3);
  vR = hor_mat_alloc(3, 1);
  qS = hor_mat_alloc(4, 1);
  qdeltaS = hor_mat_alloc(4, 1);
  RS = hor_mat_alloc(3, 3);
  RdeltaS = hor_mat_alloc(3, 3);
  qdeltaT = hor_mat_alloc(4, 1);
  Qu = hor_mat_alloc(CONTROL_SIZE, CONTROL_SIZE);
  dfv_by_du = hor_mat_alloc(STATE_SIZE, CONTROL_SIZE);
  dfv_by_duT = hor_mat_alloc(CONTROL_SIZE, STATE_SIZE);
  Temp_STATE_CONTROL = hor_mat_alloc(STATE_SIZE, CONTROL_SIZE);
  u_noisy = hor_mat_alloc(CONTROL_SIZE, 1);
  Pqsqs = hor_mat_alloc(4, 4);
  qx = hor_mat_alloc(4, 1);
  qs = hor_mat_alloc(4, 1);
  dqnew_by_dqx = hor_mat_alloc(4, 4);
  dqnew_by_dqs = hor_mat_alloc(4, 4);
  Temp88a = hor_mat_alloc(8, 8);
  Temp88b = hor_mat_alloc(8, 8);
  pW = hor_mat_alloc(3, 1);
  qinv = hor_mat_alloc(4, 1);
  RRW = hor_mat_alloc(3, 3);
  Temp31A = hor_mat_alloc(3, 1);
  Temp31B = hor_mat_alloc(3, 1);
  Temp31C= hor_mat_alloc(3, 1);
  dR = hor_mat_alloc(3, 1);
}
 
Turn_Move_ThreeD_Motion_Model::~Turn_Move_ThreeD_Motion_Model()
{
  hor_mat_free_list(r, q, Rq, vR, qS, qdeltaS, RS, RdeltaS, 
		    qdeltaT,
		    Qu, dfv_by_du, dfv_by_duT, Temp_STATE_CONTROL, u_noisy,
		    Pqsqs, qx, qs, dqnew_by_dqx, dqnew_by_dqs, Temp88a, 
		    Temp88b, pW, qinv, RRW, Temp31A, Temp31B, Temp31C,
		    dR,
		    NULL);
}


int Turn_Move_ThreeD_Motion_Model::func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
                			     Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // cout << "xv" << xv;
  // cout << "u" << u;

  // State vector:
  // { r } {    q    } S
  // x y z q0 qx qy qz S
  hor_matq_fill(r, vecel(xv, 1), vecel(xv, 2), vecel(xv, 3));
  hor_matq_fill(q, vecel(xv, 4), vecel(xv, 5), vecel(xv, 6), 
		   vecel(xv, 7));
  double &S = vecel(xv, 8);

  // Control vector:
  // v, deltaS, deltaT
  double &v = vecel(u, 1);
  double &deltaS = vecel(u, 2);
  double &deltaT = vecel(u, 3);


  // Model assumes turn/move on the current plane
  // Although can call with v and deltaS, deltaT all non-zero, this
  // assumes that we have infinitely fast turn before velocity
  // Better to call with either zero v or zero deltaS and deltaT

  // Velocity vector in robot frame
  hor_matq_fill(vR, 0.0, 0.0, v * delta_t * FV);

  // Form quaternions and rotation matrices
  R_from_q(q, Rq);

  q_from_thetay(S, qS);
  R_from_q(qS, RS);

  q_from_thetay(deltaS, qdeltaS);
  R_from_q(qdeltaS, RdeltaS);

  q_from_thetay(deltaT, qdeltaT);

  // Transform velocity vector into world frame
  hor_matq_prod2(RdeltaS, vR, Temp31a);
  hor_matq_prod2(RS, Temp31a, Temp31c); // Save Temp31c here for use later
  hor_matq_prod2(Rq, Temp31c, Temp31a);

  // New position r
  hor_matq_add2(r, Temp31a, Temp31b);
  hor_matq_insert_chunky1(Temp31b, fvRES, 0);

  // New quaternion q
  prodq2q1(q, qdeltaT, Tempqa);
  hor_matq_insert_chunky1(Tempqa, fvRES, 3);

  // New S
  vecel(fvRES, 8) = S + deltaS - deltaT;

  // Now form Jacobian dfv_by_dxv
  hor_matq_zero(dfv_by_dxvRES);

  // dr_by_dr
  hor_matq_identity(Temp33a);
  hor_matq_insert_chunkyx(Temp33a, dfv_by_dxvRES, 0, 0);

  // dr_by_dq
  // Temp31c is R(S+deltaS) * vR
  dRq_times_a_by_dq(q, Temp31c, Temp34a);
  hor_matq_insert_chunkyx(Temp34a, dfv_by_dxvRES, 0, 3);

  // dr_by_dS
  hor_matq_prod2(Rq, RdeltaS, Temp33a);
  dRq_times_a_by_dq(qS, vR, Temp34a);
  dq_by_dthetay_from_thetay(S, Tempqa);
  hor_matq_prod2(Temp34a, Tempqa, Temp31a);
  hor_matq_prod2(Temp33a, Temp31a, Temp31b);
  hor_matq_insert_chunkyx(Temp31b, dfv_by_dxvRES, 0, 7);

  // dq_by_dr = 0

  // dq_by_dq
  dq3_by_dq2(qdeltaT, Temp44a);
  hor_matq_insert_chunkyx(Temp44a, dfv_by_dxvRES, 3, 3);

  // dq_by_dS = 0

  // dS_by_dr = 0

  // dS_by_dq = 0

  // dS_by_dS
  matel(dfv_by_dxvRES, 8, 8) = 1.0;

  // cout << "fvRES" << fvRES;

  return 0;
}

// Noise for 3D motion model
// First normal noise like in 2D motion model (in the plane)
// Then add rotational noise to represent uncertainty in slope changes
int Turn_Move_ThreeD_Motion_Model::func_Q(Hor_Matrix *xv, Hor_Matrix *u, 
					  double delta_t)
{
  assert(xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);


  // State vector:
  // { r } {    q    } S
  // x y z q0 qx qy qz S
  hor_matq_fill(r, vecel(xv, 1), vecel(xv, 2), vecel(xv, 3));
  hor_matq_fill(q, vecel(xv, 4), vecel(xv, 5), vecel(xv, 6), 
		   vecel(xv, 7));
  double &S = vecel(xv, 8);

  // Control vector:
  // v, deltaS, deltaT
  double &v = vecel(u, 1);
  double &deltaS = vecel(u, 2);
  double &deltaT = vecel(u, 3);

  hor_matq_zero(Qu);
  matel(Qu, 1, 1) = turn_move_twod_motion_model->SD_frac_v_filter * 
                turn_move_twod_motion_model->SD_frac_v_filter * matel(u, 1, 1)
                                                        * matel(u, 1, 1);
  matel(Qu, 2, 2) = turn_move_twod_motion_model->SD_frac_dS_filter * 
                turn_move_twod_motion_model->SD_frac_dS_filter * matel(u, 2, 1)
                                                          * matel(u, 2, 1);
  matel(Qu, 3, 3) = turn_move_twod_motion_model->SD_frac_dT_filter * 
                turn_move_twod_motion_model->SD_frac_dT_filter * matel(u, 3, 1)
                                                          * matel(u, 3, 1);

  hor_matq_zero(dfv_by_du);

  // Form Jacobian
 
  // Velocity vector in robot frame
  hor_matq_fill(vR, 0.0, 0.0, v * delta_t * FV);

  // Form quaternions and rotation matrices
  R_from_q(q, Rq);

  q_from_thetay(S, qS);
  R_from_q(qS, RS);

  q_from_thetay(deltaS, qdeltaS);
  R_from_q(qdeltaS, RdeltaS);

  q_from_thetay(deltaT, qdeltaT);

  // dr_by_dv
  hor_matq_fill(Temp31a, 0.0, 0.0, delta_t * FV);
  hor_matq_prod2(RdeltaS, Temp31a, Temp31b);
  hor_matq_prod2(RS, Temp31b, Temp31a);
  hor_matq_prod2(Rq, Temp31a, Temp31b);
  hor_matq_insert_chunkyx(Temp31b, dfv_by_du, 0, 0);

  // dr_by_ddeltaS
  hor_matq_prod2(Rq, RS, Temp33a);
  dRq_times_a_by_dq(qdeltaS, vR, Temp34a);
  dq_by_dthetay_from_thetay(deltaS, Tempqa);
  hor_matq_prod2(Temp34a, Tempqa, Temp31a);
  hor_matq_prod2(Temp33a, Temp31a, Temp31b);
  hor_matq_insert_chunkyx(Temp31b, dfv_by_du, 0, 1);

  // dr_by_ddeltaT = 0

  // dq_by_dv = 0

  // dq_by_ddeltaS = 0

  // dq_by_ddeltaT
  dq3_by_dq1(q, Temp44a);
  dq_by_dthetay_from_thetay(deltaT, Tempqa);
  hor_matq_prod2(Temp44a, Tempqa, Tempqb);
  hor_matq_insert_chunkyx(Tempqb, dfv_by_du, 3, 2);

  // dS_by_dv = 0

  // dS_by_ddeltaS
  matel(dfv_by_du, 8, 2) = 1.0;

  // dS_by_ddeltaT
  matel(dfv_by_du, 8, 3) = -1.0;
  

  hor_matq_transpose(dfv_by_du, dfv_by_duT);

  hor_matq_prod2(dfv_by_du, Qu, Temp_STATE_CONTROL);
  hor_matq_prod2(Temp_STATE_CONTROL, dfv_by_duT, QxRES);

  /*******Now add more noise to represent uncertainty in slope changes*******/

  // Pqsqs is covariance of random quaternion (mean zero) which is the
  // rotation that happens 
  // The random angle of rotation is beta
  // The orientation is random in 0 -> 2pi which gives this form of covariance
  double SD_beta = SD_CURVATURE * vecel(u, 1) * delta_t * FV;
  hor_matq_fill(Pqsqs, 
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.125 * SD_beta * SD_beta, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.125 * SD_beta * SD_beta);

  //****This can be simplified!!!
  //****Change when I have time
  //****See 3D docs

  // Now this angle is incorporated with the state prediction up to now
  // by doing qnew = qx (old) * qs (random slope rotation, mean zero)
  // qs has mean zero so set to null quaternion
  hor_matq_fill(qs, 1.0, 0.0, 0.0, 0.0);
  func_xp(xv);
  func_q(xpRES);
  hor_matq_copy(qRES, qx); // Strictly speaking should this be from xv after
                           // the first step of func_fv ... but I don't think
                           // it matters
  // Jacobians for q2 * q1 product where q2 is qx, q1 is qs
  dq3_by_dq2(qs, dqnew_by_dqx);
  dq3_by_dq1(qx, dqnew_by_dqs);

  // Temp88a is Jacobian dxvnew_by_dxv
  hor_matq_zero(Temp88a);
  matel(Temp88a, 1, 1) = 1.0;
  matel(Temp88a, 2, 2) = 1.0;
  matel(Temp88a, 3, 3) = 1.0;
  hor_matq_insert_chunkyx(dqnew_by_dqx, Temp88a, 3, 3);
  matel(Temp88a, 8, 8) = 1.0;

  // Transform already calculated QxRES by this Jacobian and store in Temp88b
  hor_matq_ABAT(Temp88a, QxRES, Temp88b);

  // Now add new noise due to qs
  hor_matq_zero(Temp88a);
  hor_matq_ABAT(dqnew_by_dqs, Pqsqs, Temp44a);
  hor_matq_insert_chunkyx(Temp44a, Temp88a, 3, 3);

  // Add it all up and finally store in QxRES
  hor_matq_add2(Temp88b, Temp88a, QxRES);

  // cout << "QxRES " << QxRES << endl;

  return 0;
}

int Turn_Move_ThreeD_Motion_Model::func_xp(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  // xp is just xv without the last element S
  hor_matq_fill(xpRES,
		vecel(xv, 1),
		vecel(xv, 2),
		vecel(xv, 3),
		vecel(xv, 4),
		vecel(xv, 5),
		vecel(xv, 6),
		vecel(xv, 7));

  return 0;
}

int Turn_Move_ThreeD_Motion_Model::func_dxp_by_dxv(Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_zero(dxp_by_dxvRES);

  matel(dxp_by_dxvRES, 1, 1) = 1.0;
  matel(dxp_by_dxvRES, 2, 2) = 1.0;
  matel(dxp_by_dxvRES, 3, 3) = 1.0;
  matel(dxp_by_dxvRES, 4, 4) = 1.0;
  matel(dxp_by_dxvRES, 5, 5) = 1.0;
  matel(dxp_by_dxvRES, 6, 6) = 1.0;
  matel(dxp_by_dxvRES, 7, 7) = 1.0;

  return 0;
}

// Noisy process equation for simulation
// First we simply perturb u with Gaussian noise and send it through func_fv
// Then add random rotation representing the unknown slope of the floor
int Turn_Move_ThreeD_Motion_Model::func_fv_noisy(Hor_Matrix *xv_true, 
					  Hor_Matrix *u_true, double delta_t)
{
  assert (xv_true != NULL && u_true != NULL);

  assert (xv_true->rows == STATE_SIZE && xv_true->cols == 1 &&
	  u_true->rows == CONTROL_SIZE && u_true->cols == 1);

  double SD_v = turn_move_twod_motion_model->SD_frac_v * vecel(u_true, 1);
  double SD_dS = turn_move_twod_motion_model->SD_frac_dS * vecel(u_true, 2);
  double SD_dT = turn_move_twod_motion_model->SD_frac_dT * vecel(u_true, 3); 

  hor_matq_fill(u_noisy,
		hor_gauss_rand(vecel(u_true, 1), SD_v),
		hor_gauss_rand(vecel(u_true, 2), SD_dS),
		hor_gauss_rand(vecel(u_true, 3), SD_dT));

  func_fv_and_dfv_by_dxv(xv_true, u_noisy, delta_t);

  hor_matq_copy(fvRES, fv_noisyRES);

  // Now add random rotation representing the unknown slope of the floor:
  // like there is a single fold in the planar floor

  // Pick a random direction (0 -> 2pi) in the xz plane of the robot plane
  double phi = rand() * 2 * M_PI / RAND_MAX;
  // cout << "phi = " << phi << endl;

  // Pick random angle, gaussian, representing angle of change
  // Standard deviation of this angle is 
  // distance travelled * curvature constant
  double beta = hor_gauss_rand(0, SD_CURVATURE * vecel(u_true, 1) * delta_t 
			       * FV);
  // cout << "beta = " << beta << endl;
  // Just for safety but should never happen?
  and_pi_range(beta);

  // Form quaternion
  hor_matq_fill(Tempqa,
		cos(beta / 2.0),
		cos(phi) * sin(beta / 2.0),
		0.0,
		sin(phi) * sin(beta / 2.0));

  // Compose with state
  func_q(fv_noisyRES);
  prodq2q1(qRES, Tempqa, Tempqb);

  hor_matq_insert_chunky1(Tempqb, fv_noisyRES, 3);

  return 0;
}

int Turn_Move_ThreeD_Motion_Model::
    func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef)
{
  assert (xv != NULL && xpdef != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  // We can mainly use the stuff from the general redefinition of axes in
  // position coordinates, but need to change it a little bit
  // State is          x, y, z, q0, q1, q2, q3, S
  // Position state is x, y, z, q0, q1, q2, q3

  func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(xpRES);

  func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(local_xp, xpdef);

  // Redefined xv is just redefined xp with S added at the end
  hor_matq_insert_chunky1(xpredefRES, xvredefRES, 0);
  vecel(xvredefRES, 8) = vecel(xv, 8);

  // Jacobians
  hor_matq_zero(dxvredef_by_dxvRES);
  hor_matq_insert_chunkyx(dxpredef_by_dxpRES, dxvredef_by_dxvRES, 0, 0);
  matel(dxvredef_by_dxvRES, 8, 8) = 1.0;

  hor_matq_zero(dxvredef_by_dxpdefRES);
  hor_matq_insert_chunkyx(dxpredef_by_dxpdefRES, dxvredef_by_dxpdefRES, 0, 0);

  hor_mat_free_list(local_xp, NULL);

  return 0;
}

int Turn_Move_ThreeD_Motion_Model::navigate_to_waypoint(Hor_Matrix *xv, 
			 Hor_Matrix *xv_goal, Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && xv_goal != NULL && u != NULL);
  
  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xv_goal->rows == STATE_SIZE && xv_goal->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Stop when we are this close
  const double ARRIVED_DISTANCE_THRESHOLD = 0.02;
  //  const double ARRIVED_TURRET_THRESHOLD = 1.0 * M_PI / 180.0;
  // Steer on the spot if we need to turn by more than this
  const double STATIONARY_STEERING_LIMIT = 5.0 * M_PI / 180.0;
  // Only control steering, not turret, if we are within this distance
  // const double CLOSE_MANOUEVRE_LIMIT = 1.00;

  // Usual speed
  const double USUAL_SPEED = 0.1;

  // 3D waypoint navigation:

  // pW is world frame vector from current position to waypoint
  hor_matq_fill(pW, 
		vecel(xv_goal, 1) - vecel(xv, 1),
		vecel(xv_goal, 2) - vecel(xv, 2),
		vecel(xv_goal, 3) - vecel(xv, 3));

  // Extract Rotation matrix for current position
  func_xp(xv);
  func_q(xpRES);
  invert_quaternion(qRES, qinv);
  R_from_q(qinv, RRW);

  // Now in 3D we want to find where the robot's ground plane intersects
  // the vertical line running through the waypoint and aim for that
  // That is to say that rather than aiming for the 3D point itself
  // (because we don't know what the terrain is like) we just aim to get 
  // above or below it.

  // Parameter lambda which describes the location of this intersection
  // point rR in the equation
  // rR = RRW pW + lambda RRW jW (jW is (0 1 0)^T)
  // is found as follows:

  double lambda = - (matel(RRW, 2, 1) * vecel(pW, 1) +
		     matel(RRW, 2, 2) * vecel(pW, 2) +
		     matel(RRW, 2, 3) * vecel(pW, 3) ) / matel(RRW, 2, 2);
		    
  // So dR, vector to where we want to get to is:
  hor_matq_fill(Temp31A, 0.0, 1.0, 0.0);
  hor_matq_prod2(RRW, Temp31A, Temp31B);
  hor_matq_scale(Temp31B, lambda);
  hor_matq_prod2(RRW, pW, Temp31C);
  hor_matq_add2(Temp31B, Temp31C, dR);

  // From now on proceed more or less as in 2D case
  // What angle does that vector make with turret and steering direction?
  double z_diff = vecel(dR, 3);
  double x_diff = vecel(dR, 1);

  double dist = sqrt (z_diff * z_diff + x_diff * x_diff);
  double angle = atan2 (x_diff, z_diff); // Angle to waypoint in world frame

  if (dist <= ARRIVED_DISTANCE_THRESHOLD) // We are there: finally set turret
  {
    hor_matq_zero(u);
    return 1; // Arrived: zero movement
  }

  // Work out how much we need to steer to go straight to waypoint
  // Point turret and steering straight at it
  double S_increment = angle - matel(xv, 8, 1);
  and_pi_range (S_increment); // Puts in range +/- PI
  double T_increment = (angle) ;
  and_pi_range (T_increment);

  double v;

  if (dist < USUAL_SPEED * delta_t * FV)
  {
    v = 0.5 * dist / (delta_t * FV);
    T_increment = 0.0; // Don't make large turrent movements when near
  }
  else v = USUAL_SPEED;

  if (fabs(S_increment) > STATIONARY_STEERING_LIMIT || 
      fabs(T_increment) > STATIONARY_STEERING_LIMIT)
    v = 0.0; // Steer on the spot

  matel(u, 1, 1) = v;
  matel(u, 2, 1) = S_increment;
  matel(u, 3, 1) = T_increment;

  return 0;
}

int Turn_Move_ThreeD_Motion_Model::func_xvnorm_and_dxvnorm_by_dxv(
							   Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);


  // Normalise the state vector: since quaternion is redundant we sometimes
  // need to enforce that it stays with size 1

  // Most parts of the state vector don't change so copy as starting point
  hor_matq_copy(xv, xvnormRES);

  // Most parts of Jacobian are identity
  hor_matq_identity(dxvnorm_by_dxvRES);

  // Extract quaternion  
  func_xp(xv);
  func_q(xpRES);

  hor_matq_copy(qRES, Tempqa);

  normalise_quaternion(Tempqa, Tempqb);
  dqnorm_by_dq(Tempqa, Temp44a);

  hor_matq_insert_chunky1(Tempqb, xvnormRES, 3);
  hor_matq_insert_chunkyx(Temp44a, dxvnorm_by_dxvRES, 3, 3);

  return 0;
}
