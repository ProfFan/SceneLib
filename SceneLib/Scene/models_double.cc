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

#include "general_headers.h"
#include "models_base.h"
#include "models_double.h"

Double_Motion_Model::Double_Motion_Model(Motion_Model *m_m1, Motion_Model *m_m2)
  : Motion_Model(m_m1->POSITION_STATE_SIZE, 
		 m_m1->STATE_SIZE + m_m2->STATE_SIZE, 
		 m_m1->CONTROL_SIZE + m_m2->CONTROL_SIZE, 
		 m_m1->motion_model_dimensionality_type, 
		 "DOUBLE"),
    POSITION_STATE_SIZE1(m_m1->POSITION_STATE_SIZE), 
    POSITION_STATE_SIZE2(m_m2->POSITION_STATE_SIZE),
    STATE_SIZE1(m_m1->STATE_SIZE),
    STATE_SIZE2(m_m2->STATE_SIZE),
    CONTROL_SIZE1(m_m1->CONTROL_SIZE),
    CONTROL_SIZE2(m_m2->CONTROL_SIZE)
{
  motion_model1 = m_m1;
  motion_model2 = m_m2;

  xpose1RES = hor_mat_alloc(3, 1);
  Rpose1RES = hor_mat_alloc(3, 3);
  xpose2RES = hor_mat_alloc(3, 1);
  Rpose2RES = hor_mat_alloc(3, 3);
  xp1RES = hor_mat_alloc(POSITION_STATE_SIZE1, 1);
  xp2RES = hor_mat_alloc(POSITION_STATE_SIZE2, 1);
  dxp1_by_dxv1RES = hor_mat_alloc(POSITION_STATE_SIZE1, STATE_SIZE1);
  dxp2_by_dxv2RES = hor_mat_alloc(POSITION_STATE_SIZE2, STATE_SIZE2);
  xv1RES = hor_mat_alloc(STATE_SIZE1, 1);
  xv2RES = hor_mat_alloc(STATE_SIZE2, 1);
  u1RES = hor_mat_alloc(CONTROL_SIZE1, 1);
  u2RES = hor_mat_alloc(CONTROL_SIZE2, 1);
}

Double_Motion_Model::~Double_Motion_Model()
{
  hor_mat_free_list(xpose1RES, Rpose1RES, xpose2RES, Rpose2RES, 
		    xp1RES, xp2RES, dxp1_by_dxv1RES, dxp2_by_dxv2RES,
		    xv1RES, xv2RES, u1RES, u2RES, NULL);

  Motion_Model_destructor();
}


int Double_Motion_Model::func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t)
{
  // Call the individual routines then combine the results

  // First extract individual states and control vectors
  func_xv1(xv);
  func_xv2(xv);
  func_u1(u);
  func_u2(u);

  // Call individual routines
  motion_model1->func_fv_and_dfv_by_dxv(xv1RES, u1RES, delta_t);
  motion_model2->func_fv_and_dfv_by_dxv(xv2RES, u2RES, delta_t);

  // Fill overall result matrices
  hor_matq_insert_chunky1(motion_model1->fvRES, fvRES, 0);
  hor_matq_insert_chunky1(motion_model2->fvRES, fvRES, STATE_SIZE1);
  hor_matq_zero(dfv_by_dxvRES);
  hor_matq_insert_chunkyx(motion_model1->dfv_by_dxvRES, dfv_by_dxvRES, 0, 0);
  hor_matq_insert_chunkyx(motion_model2->dfv_by_dxvRES, dfv_by_dxvRES, 
			  STATE_SIZE1, STATE_SIZE1);

  return 0;
}

int Double_Motion_Model::func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t)
{
  // First extract individual states and control vectors
  func_xv1(xv);
  func_xv2(xv);
  func_u1(u);
  func_u2(u);

  // Call individual routines
  motion_model1->func_Q(xv1RES, u1RES, delta_t);
  motion_model2->func_Q(xv2RES, u2RES, delta_t);

  // Fill overall result matrices
  hor_matq_zero(QxRES);
  hor_matq_insert_chunkyx(motion_model1->QxRES, QxRES, 0, 0);
  hor_matq_insert_chunkyx(motion_model2->QxRES, QxRES, STATE_SIZE1, 
			  STATE_SIZE1);

  return 0;
}

int Double_Motion_Model::func_xpose1_and_Rpose1(Hor_Matrix *xv)
{
  func_xv1(xv);

  motion_model1->func_xpose_and_Rpose(xv1RES);

  hor_matq_copy(motion_model1->xposeRES, xpose1RES);
  hor_matq_copy(motion_model1->RposeRES, Rpose1RES);

  return 0;
}

int Double_Motion_Model::func_xpose2_and_Rpose2(Hor_Matrix *xv)
{
  func_xv2(xv);

  motion_model2->func_xpose_and_Rpose(xv2RES);

  hor_matq_copy(motion_model2->xposeRES, xpose2RES);
  hor_matq_copy(motion_model2->RposeRES, Rpose2RES);

  return 0;
}

int Double_Motion_Model::func_xp1(Hor_Matrix *xv)
{
  func_xv1(xv);

  motion_model1->func_xp(xv1RES);

  hor_matq_copy(motion_model1->xpRES, xp1RES);

  return 0;
}

int Double_Motion_Model::func_xp2(Hor_Matrix *xv)
{
  func_xv2(xv);

  motion_model2->func_xp(xv2RES);

  hor_matq_copy(motion_model2->xpRES, xp2RES);

  return 0;
}

int Double_Motion_Model::func_dxp1_by_dxv1(Hor_Matrix *xv)
{
  func_xv1(xv);

  motion_model1->func_dxp_by_dxv(xv1RES);
  hor_matq_copy(motion_model1->dxp_by_dxvRES, dxp1_by_dxv1RES);

  return 0;
}

int Double_Motion_Model::func_dxp2_by_dxv2(Hor_Matrix *xv)
{
  func_xv2(xv);

  motion_model2->func_dxp_by_dxv(xv2RES);
  hor_matq_copy(motion_model2->dxp_by_dxvRES, dxp2_by_dxv2RES);

  return 0;
}

int Double_Motion_Model::func_xv1(Hor_Matrix *xv)
{
  hor_matq_extract_chunky1(xv, xv1RES, 0);

  return 0;
}

int Double_Motion_Model::func_xv2(Hor_Matrix *xv)
{
  hor_matq_extract_chunky1(xv, xv2RES, STATE_SIZE1);

  return 0;
}

int Double_Motion_Model::func_u1(Hor_Matrix *u)
{
  hor_matq_extract_chunky1(u, u1RES, 0);

  return 0;
}

int Double_Motion_Model::func_u2(Hor_Matrix *u)
{
  hor_matq_extract_chunky1(u, u2RES, CONTROL_SIZE1);

  return 0;
}

// Return just the position state of the first robot
int Double_Motion_Model::func_xp(Hor_Matrix *xv)
{
  func_xp1(xv);

  hor_matq_copy(xp1RES, xpRES);

  return 0;
}

// This uses just the first robot too
int Double_Motion_Model::func_dxp_by_dxv(Hor_Matrix *xv)
{
  func_xv1(xv);
  
  motion_model1->func_dxp_by_dxv(xv1RES);

  hor_matq_zero(dxp_by_dxvRES);

  hor_matq_insert_chunkyx(motion_model1->dxp_by_dxvRES, dxp_by_dxvRES, 0, 0);

  return 0;
}

int Double_Motion_Model::func_fv_noisy(Hor_Matrix *xv_true, 
				       Hor_Matrix *u_true, double delta_t)
{
  // First extract individual states and control vectors
  func_xv1(xv_true);
  func_xv2(xv_true);
  func_u1(u_true);
  func_u2(u_true);

  motion_model1->func_fv_noisy(xv1RES, u1RES, delta_t);
  motion_model2->func_fv_noisy(xv2RES, u2RES, delta_t);

  hor_matq_insert_chunky1(motion_model1->fv_noisyRES, fv_noisyRES, 0);
  hor_matq_insert_chunky1(motion_model2->fv_noisyRES, fv_noisyRES, 
			  STATE_SIZE1);

  return 0;
}

// Navigate to waypoints functions calls those of the two motions
// models independently
int Double_Motion_Model::navigate_to_waypoint(Hor_Matrix *xv, 
	         Hor_Matrix *xv_goal, Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && xv_goal != NULL && u != NULL);
  
  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xv_goal->rows == STATE_SIZE && xv_goal->cols == 1 && 
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Form individual state, control and goal vectors
  func_xv1(xv);
  Hor_Matrix *local_xv1 = hor_mats_copy(xv1RES);
  func_xv2(xv);
  Hor_Matrix *local_xv2 = hor_mats_copy(xv2RES);

  func_xv1(xv_goal);
  Hor_Matrix *local_xv1_goal = hor_mats_copy(xv1RES);
  func_xv2(xv_goal);
  Hor_Matrix *local_xv2_goal = hor_mats_copy(xv2RES);

  func_u1(u);
  func_u2(u);

  // Call individual functions
  int arrived1 = motion_model1->navigate_to_waypoint
    (local_xv1, local_xv1_goal, u1RES, delta_t);
  int arrived2 = motion_model2->navigate_to_waypoint
    (local_xv2, local_xv2_goal, u2RES, delta_t);

  hor_mat_free_list(local_xv1, local_xv2, local_xv1_goal, local_xv2_goal,
		    NULL);

  if (arrived1 == 1 && arrived2 == 1)
    return 1;

  // Combine results into global control vector
  hor_matq_insert_chunky1(u1RES, u, 0);
  hor_matq_insert_chunky1(u2RES, u, motion_model1->CONTROL_SIZE);

  return 0;
}

int Double_Motion_Model::func_xpose_and_Rpose(Hor_Matrix *xv)
{
  func_xpose1_and_Rpose1(xv);

  hor_matq_copy(xpose1RES, xposeRES);
  hor_matq_copy(Rpose1RES, RposeRES);
  
  return 0;
}

int Double_Motion_Model::bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
			Hor_Matrix *xptest)
{
  return motion_model1->bounding_box_test(xp1, xp2, xptest);
}

int Double_Motion_Model::
  func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(Hor_Matrix *xv, 
							  Hor_Matrix *xpdef)
{
  func_xv1(xv);
  func_xv2(xv);

  motion_model1->func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef
    (xv1RES, xpdef);
  motion_model2->func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef
    (xv2RES, xpdef);

  hor_matq_insert_chunky1(motion_model1->xvredefRES, xvredefRES,
			  0);
  hor_matq_insert_chunky1(motion_model2->xvredefRES, xvredefRES,
			  motion_model1->STATE_SIZE);

  hor_matq_zero(dxvredef_by_dxvRES);
  hor_matq_insert_chunkyx(motion_model1->dxvredef_by_dxvRES, 
			  dxvredef_by_dxvRES,
			  0, 0);
  hor_matq_insert_chunkyx(motion_model2->dxvredef_by_dxvRES, 
			  dxvredef_by_dxvRES,
			  motion_model1->STATE_SIZE, 
			  motion_model1->STATE_SIZE);

  hor_matq_insert_chunkyx(motion_model1->dxvredef_by_dxpdefRES, 
			  dxvredef_by_dxpdefRES,
			  0, 0);
  hor_matq_insert_chunkyx(motion_model2->dxvredef_by_dxpdefRES, 
			  dxvredef_by_dxpdefRES,
			  motion_model1->STATE_SIZE, 0);

  return 0;  
}

int Double_Motion_Model::
  func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(Hor_Matrix *xp, 
							  Hor_Matrix *xpdef)
{
  // This function should not be called
  assert(0);

  motion_model1->func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef
    (xp, xpdef);

  hor_matq_copy(motion_model1->xpredefRES, xpredefRES);
  hor_matq_copy(motion_model1->dxpredef_by_dxpRES, dxpredef_by_dxpRES);
  hor_matq_copy(motion_model1->dxpredef_by_dxpdefRES, dxpredef_by_dxpdefRES);

  return 0;
}
