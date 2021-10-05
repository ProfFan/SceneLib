/*  SceneApp: applications for sequential localisation and map-building

    ETLRobot/models_nomad.h
    Copyright (C) 2001 Andrew Davison
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

// Turn/move motion model: at the moment, assumes steering on the spot then
// driving. Best to use separately for steering and driving (otherwise
// we must assume the steering happens infinitely fast)
// State vector: (z, x, S, T, Fv)
// Control vector: (v, dS, dT)

class Turn_Move_TwoD_Motion_Model : public TwoD_Motion_Model
{
 public:
  Turn_Move_TwoD_Motion_Model();
  ~Turn_Move_TwoD_Motion_Model();

  // Constants
  const double SD_frac_v_filter, SD_frac_dS_filter, SD_frac_dT_filter;
  const double SD_frac_v, SD_frac_dS, SD_frac_dT;

  // Redefined virtual functions
  int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t);

  int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  int func_xp(Hor_Matrix *xv);

  int func_dxp_by_dxv(Hor_Matrix *xv);

  int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, double delta_t);

  int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef);

  int navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xv_goal, 
			   Hor_Matrix *u, double delta_t);

 protected:

  // Calculation space matrices
  Hor_Matrix *Qu, *dfv_by_du, *dfv_by_duT, *Temp_STATE_CONTROL, *u_noisy;
};


// Turn/move motion model in 3D: assumes steering on the spot then
// driving. 
// Introduces uncertainty to approximate uncertainty in floor orientation
// Best to use separately for steering and driving (otherwise
// we must assume the steering happens infinitely fast)
// State vector: (x, y, z, q0, q1, q2, q3, S)
// S is steering angle relative to turret
// Control vector: (v, dS, dT)
// Have a TwoD_Turn_Move_Motion_Model to refer to sometimes (constants etc.)

class Turn_Move_ThreeD_Motion_Model : public ThreeD_Motion_Model
{
 public:
  Turn_Move_ThreeD_Motion_Model(Turn_Move_TwoD_Motion_Model 
				*turn_move_twod_motion_model);
  ~Turn_Move_ThreeD_Motion_Model();

  // Constants
  // Standard deviation of curvature we expect for floor
  static const double SD_CURVATURE = 0.5;

  static const double FV = 1.0;

  // Redefined virtual functions
  int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t);

  int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  int func_xp(Hor_Matrix *xv);

  int func_dxp_by_dxv(Hor_Matrix *xv);

  int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, double delta_t);

  int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef);

  int navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xv_goal, 
			   Hor_Matrix *u, double delta_t);

  int func_xvnorm_and_dxvnorm_by_dxv(Hor_Matrix *xv);

 protected:
  // Uses a Turn_Move_TwoD_Motion_Model for constants only
  Turn_Move_TwoD_Motion_Model *turn_move_twod_motion_model;

  // Calculation space matrices
  Hor_Matrix *r, *q, *Rq, *vR, *qS, *qdeltaS, *RS, *RdeltaS,
    *qdeltaT,
    *Qu, *dfv_by_du, *dfv_by_duT, *Temp_STATE_CONTROL, *u_noisy,
    *Pqsqs, *qx, *qs, *dqnew_by_dqx, *dqnew_by_dqs, *Temp88a, *Temp88b,
    *pW, *qinv, *RRW, *Temp31A, *Temp31B, *Temp31C, *dR;
};

