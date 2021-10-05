/*  Scene: software for sequential localisation and map-building
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

// Special motion model for two robots, where one is a blind 
// assistant
class Double_Motion_Model : public Motion_Model
{
 public:
  Double_Motion_Model(Motion_Model *m_m1, Motion_Model *m_m2);
  ~Double_Motion_Model();

  // Redefined virtual functions
  int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t);

  int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  int func_xp(Hor_Matrix *xv);

  int func_dxp_by_dxv(Hor_Matrix *xv);

  int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, double delta_t);

  int navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xp_goal, 
			   Hor_Matrix *u, double delta_t);

  int func_xpose_and_Rpose(Hor_Matrix *xv);

  int bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
			Hor_Matrix *xptest);

  int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(Hor_Matrix *xv, 
							   Hor_Matrix *xpdef);
  int func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(Hor_Matrix *xp, 
							   Hor_Matrix *xpdef);


  // Extra functions specific to double case
  int func_xpose1_and_Rpose1(Hor_Matrix *xv);
  int func_xpose2_and_Rpose2(Hor_Matrix *xv);
  int func_xp1(Hor_Matrix *xv);
  int func_xp2(Hor_Matrix *xv);
  int func_dxp1_by_dxv1(Hor_Matrix *xv);
  int func_dxp2_by_dxv2(Hor_Matrix *xv);
  int func_xv1(Hor_Matrix *xv);
  int func_xv2(Hor_Matrix *xv);
  int func_u1(Hor_Matrix *u);
  int func_u2(Hor_Matrix *u);

  // Extra results matrices
  Hor_Matrix *xpose1RES, *xpose2RES, *Rpose1RES, *Rpose2RES, *xp1RES, *xp2RES,
    *dxp1_by_dxv1RES, *dxp2_by_dxv2RES, *xv1RES, *xv2RES, *u1RES, *u2RES;

  // Extra individual constants
  const int POSITION_STATE_SIZE1, POSITION_STATE_SIZE2;
  const int STATE_SIZE1, STATE_SIZE2;
  const int CONTROL_SIZE1, CONTROL_SIZE2;

  // Inidividual motion models
  Motion_Model *motion_model1, *motion_model2;
};

