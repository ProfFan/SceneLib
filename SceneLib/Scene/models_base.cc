/*  Scene: software for sequential localisation and map-building

    models_base.cc
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

/***************************Motion Model Functions****************************/

Motion_Model::Motion_Model(int position_state_size, int state_size, 
			   int control_size, char *m_m_d_t, char *m_m_t)
  : POSITION_STATE_SIZE(position_state_size),
    STATE_SIZE(state_size),
    CONTROL_SIZE(control_size),
    motion_model_dimensionality_type(m_m_d_t),
    motion_model_type(m_m_t)
{
  fvRES = hor_mat_alloc(STATE_SIZE, 1);
  dfv_by_dxvRES = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
  QxRES = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
  xpRES = hor_mat_alloc(POSITION_STATE_SIZE, 1);
  dxp_by_dxvRES = hor_mat_alloc(POSITION_STATE_SIZE, STATE_SIZE);
  fv_noisyRES = hor_mat_alloc(STATE_SIZE, 1);
  xvredefRES = hor_mat_alloc(STATE_SIZE, 1);
  dxvredef_by_dxvRES = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
  dxvredef_by_dxpdefRES = hor_mat_alloc(STATE_SIZE, POSITION_STATE_SIZE);
  xvnormRES = hor_mat_alloc(STATE_SIZE, 1);
  dxvnorm_by_dxvRES = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
  zeroedxvRES = hor_mat_alloc(STATE_SIZE, 1);
  dzeroedxv_by_dxvRES = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
  xpredefRES = hor_mat_alloc(POSITION_STATE_SIZE, 1);
  dxpredef_by_dxpRES = hor_mat_alloc(POSITION_STATE_SIZE, POSITION_STATE_SIZE);
  dxpredef_by_dxpdefRES = hor_mat_alloc(POSITION_STATE_SIZE, 
					POSITION_STATE_SIZE);
  xposeRES = hor_mat_alloc(3, 1);
  RposeRES = hor_mat_alloc(3, 3);
  Temp_SS1 = hor_mat_alloc(STATE_SIZE, STATE_SIZE);
}

void Motion_Model::Motion_Model_destructor()
{
  hor_mat_free_list(fvRES, dfv_by_dxvRES, QxRES, 
		    xpRES, dxp_by_dxvRES, fv_noisyRES,
		    xvredefRES, dxvredef_by_dxvRES, dxvredef_by_dxpdefRES, 
		    xvnormRES, dxvnorm_by_dxvRES,
		    zeroedxvRES, dzeroedxv_by_dxvRES,
		    xpredefRES, dxpredef_by_dxpRES, dxpredef_by_dxpdefRES,     
		    xposeRES, RposeRES,
		    Temp_SS1,
		    NULL);
}

// Normalise state vector if there is a redundant representation:
// Default null version
int Motion_Model::func_xvnorm_and_dxvnorm_by_dxv(Hor_Matrix *xv)
{
  hor_matq_copy(xv, xvnormRES);
  hor_matq_identity(dxvnorm_by_dxvRES);

  return 0;
}

// Set control vector to get to a waypoint
// Default null version
int Motion_Model::navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xv_goal, 
				   Hor_Matrix *u, double delta_t)
{
  cerr << "Error: navigate_to_waypoint() not implemented.\n"; 
  return 0;
}

// Zero the axes at the current state: i.e. redefine axes such that 
// the current estimate's position state becomes the new origin
int Motion_Model::func_zeroedxv_and_dzeroedxv_by_dxv(Hor_Matrix *xv)
{
  func_xp(xv);

  func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(xv, xpRES);

  hor_matq_copy(xvredefRES, zeroedxvRES);

  // This is a bit subtle: we need to know dzeroedxv_by_dxv,
  // and we know the two Jacobians dxvredef_by_dxv and dxvredef_by_dxpdef
  // --- we must remember that xv and xpdef are not independent though
  func_dxp_by_dxv(xv);

  hor_matq_prod2(dxvredef_by_dxpdefRES, dxp_by_dxvRES, Temp_SS1);
  hor_matq_add2(Temp_SS1, dxvredef_by_dxvRES, dzeroedxv_by_dxvRES);

  return 0;
}

const double INCREMENT_SIZE = 0.00000001;
int Motion_Model::jacobians_test(Hor_Matrix *xv, Hor_Matrix *u, double delta_t)
{
  // Test dfv_by_dxv
  // Testing Jacobians
  // Here we test Jacobians against a small increment approximation
  // a = a(b)
  

  // b0 is standard value of b
  Hor_Matrix *b0 = hor_mats_copy(xv);
  // da0_by_db0 is originally calculated Jacobian
  func_fv_and_dfv_by_dxv(b0, u, delta_t);
  // a0 is standard value of a
  Hor_Matrix *a0 = hor_mats_copy(fvRES);
  Hor_Matrix *da0_by_db0 = hor_mats_copy(dfv_by_dxvRES);

  // a and b are locally adjusted versions
  Hor_Matrix *a = hor_mats_copy(a0);
  Hor_Matrix *b = hor_mats_copy(b0);
  // adiff and bdiff are vectors of differences
  Hor_Matrix *adiff = hor_mats_copy(a0);
  Hor_Matrix *bdiff = hor_mats_copy(b0);
  // da_by_db is locally calculated Jacobian
  Hor_Matrix *da_by_db = hor_mats_copy(da0_by_db0);

  for (int bpos = 1; bpos <= b0->rows; bpos++) {
    hor_matq_copy(b0, b);
    vecel(b, bpos) += INCREMENT_SIZE;
    hor_matq_sub(b, b0, bdiff);
    func_fv_and_dfv_by_dxv(b, u, delta_t);
    hor_matq_copy(fvRES, a);
    hor_matq_sub(a, a0, adiff);
    for (int apos = 1; apos <= a0->rows; apos++) {
      matel(da_by_db, apos, bpos) = vecel(adiff, apos) / vecel(bdiff, bpos);
    }
  }

  cout << "Explicit Jacobian:" << da0_by_db0 << endl;
  cout << "Approximated Jacobian:" << da_by_db << endl;

  hor_mat_free_list(a0, b0, da0_by_db0, a, b, adiff, bdiff, da_by_db, NULL);

  return 0;
}

ThreeD_Motion_Model::ThreeD_Motion_Model(int state_size, int control_size, 
					 char *m_m_t)
    : Motion_Model(7, state_size, control_size, "THREED", m_m_t) 
{
  xRES = hor_mat_alloc(3, 1);
  qRES = hor_mat_alloc(4, 1);

  x0 = hor_mat_alloc(3, 1);
  q0 = hor_mat_alloc(4, 1);

  xn = hor_mat_alloc(3, 1);
  qn = hor_mat_alloc(4, 1);
  qnbar = hor_mat_alloc(4, 1);

  Temp31a = hor_mat_alloc(3, 1);
  Temp31b = hor_mat_alloc(3, 1);
  Temp31c = hor_mat_alloc(3, 1);
  Temp33a = hor_mat_alloc(3, 3);
  Temp33b = hor_mat_alloc(3, 3);
  Tempqa = hor_mat_alloc(4, 1);
  Tempqb = hor_mat_alloc(4, 1);
  Temp44a = hor_mat_alloc(4, 4);
  Temp44b = hor_mat_alloc(4, 4);
  Temp44c = hor_mat_alloc(4, 4);
  Temp34a = hor_mat_alloc(3, 4);
  Temp34b = hor_mat_alloc(3, 4);
}

ThreeD_Motion_Model::~ThreeD_Motion_Model()
{
  hor_mat_free_list(xRES, qRES, x0, q0, xn, qn, qnbar,
		    Temp31a, Temp31b, Temp31c, Temp33a, Temp33b, Tempqa, Tempqb,
		    Temp44a, Temp44b, Temp44c, Temp34a, Temp34b, NULL);
}

int ThreeD_Motion_Model::
    func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
    (Hor_Matrix *xp, Hor_Matrix *xpdef)
{
  // Split position state into cartesian and quaternion

  // x0, q0: position in original frame
  func_x(xp);
  func_q(xp);
  hor_matq_copy(xRES, x0);
  hor_matq_copy(qRES, q0);

  // xn, qn: definition of new frame
  func_x(xpdef);
  func_q(xpdef);
  hor_matq_copy(xRES, xn);
  hor_matq_copy(qRES, qn);

  invert_quaternion(qn, qnbar);

  // Calculate new cartesian part of xp
  hor_matq_sub(x0, xn, Temp31a); // Temp31a is x0 - xn
  R_from_q(qnbar, Temp33a); // Temp33a is R(qnbar)
  hor_matq_prod2(Temp33a, Temp31a, Temp31b);

  // Copy into xpredefRES
  hor_matq_insert_chunky1(Temp31b, xpredefRES, 0);

  // Calculate new quaternion part of xp
  prodq2q1(qnbar, q0, Tempqa);

  // Copy into xpredefRES
  hor_matq_insert_chunky1(Tempqa, xpredefRES, 3);

  // Form Jacobian dxpredef_by_dxpRES
  hor_matq_zero(dxpredef_by_dxpRES);
  hor_matq_insert_chunkyx(Temp33a, dxpredef_by_dxpRES, 0, 0);
  dq3_by_dq1(qnbar, Temp44a);
  hor_matq_insert_chunkyx(Temp44a, dxpredef_by_dxpRES, 3, 3);

  // Form Jacobian dxpredef_by_dxpdefRES
  hor_matq_zero(dxpredef_by_dxpdefRES);
  hor_matq_scale(Temp33a, -1);
  hor_matq_insert_chunkyx(Temp33a, dxpredef_by_dxpdefRES, 0, 0);

  dq3_by_dq2(q0, Temp44a);
  dqbar_by_dq(Temp44b);  // Temp44b is dqbar_by_dq
  hor_matq_prod2(Temp44a, Temp44b, Temp44c);
  hor_matq_insert_chunkyx(Temp44c, dxpredef_by_dxpdefRES, 3, 3);

  // Top right corner of this Jacobian is tricky because we have to
  // differentiate a rotation matrix
  // Uses function that does this in bits.cc
  // Build this corner in Temp34a
  dRq_times_a_by_dq(qnbar, Temp31a, Temp34a);

  // So far we have dxN_by_dqnbar; want _by_dqn
  hor_matq_prod2(Temp34a, Temp44b, Temp34b);
  // Finally copy into result matrix
  hor_matq_insert_chunkyx(Temp34b, dxpredef_by_dxpdefRES, 0, 3);

  cout << "dxpredef_by_dxpdefRES" << dxpredef_by_dxpdefRES;

  return 0;
}

int ThreeD_Motion_Model::func_xpose_and_Rpose(Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  func_xp(xv);

  // Turn xp vector (x, y, z, q0, q1, q2, q3) into pose
  hor_matq_fill(xposeRES, 
		matel(xpRES, 1, 1), 
		matel(xpRES, 2, 1),
		matel(xpRES, 3, 1));

  Hor_Matrix *q = hor_mats_fill(4, 1,
				matel (xpRES, 4, 1),
				matel (xpRES, 5, 1),
				matel (xpRES, 6, 1),
				matel (xpRES, 7, 1));
	
  R_from_q(q, RposeRES);
                                        /* Rpose is MLC0 */
  hor_mat_free(q);

  return 0;
}

int ThreeD_Motion_Model::bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
					   Hor_Matrix *xptest)
{
  assert(xp1 != NULL || xp2 != NULL || xptest != NULL);
  
  assert(xp1->rows == POSITION_STATE_SIZE && xp1->cols == 1 &&
	 xp2->rows == POSITION_STATE_SIZE && xp2->cols == 1 &&
	 xptest->rows == POSITION_STATE_SIZE && xptest->cols == 1);

  //  cout << "xp1" << xp1 << "xp2" << xp2 << "xptest" << xptest;

  // ThreeD position state xp (x, y, z, q0, q1, q2, q3)
  // Only test x, y, z
  return (matel(xptest, 1, 1) >= matel(xp1, 1, 1) &&
	  matel(xptest, 1, 1) < matel(xp2, 1, 1) &&
	  matel(xptest, 2, 1) >= matel(xp1, 2, 1) &&
	  matel(xptest, 2, 1) < matel(xp2, 2, 1) &&
	  matel(xptest, 3, 1) >= matel(xp1, 3, 1) &&
	  matel(xptest, 3, 1) < matel(xp2, 3, 1));
}

int ThreeD_Motion_Model::func_x(Hor_Matrix *xp)
{
  hor_matq_fill(xRES,
		matel(xp, 1, 1),
		matel(xp, 2, 1),
		matel(xp, 3, 1));

  return 0;
}

int ThreeD_Motion_Model::func_q(Hor_Matrix *xp)
{
  hor_matq_fill(qRES,
		matel(xp, 4, 1),
		matel(xp, 5, 1),
		matel(xp, 6, 1),
		matel(xp, 7, 1));

  return 0;
}

int TwoD_Motion_Model::func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef
    (Hor_Matrix *xp, Hor_Matrix *xpdef)
{
  assert (xp != NULL && xpdef != NULL);
  
  assert (xp->rows == POSITION_STATE_SIZE && xp->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  double &zdef = matel(xpdef, 1, 1);
  double &xdef = matel(xpdef, 2, 1);
  double &phidef = matel(xpdef, 3, 1);
  double cphidef = cos(phidef);
  double sphidef = sin(phidef);
  double &z0 = matel(xp, 1, 1);
  double &x0 = matel(xp, 2, 1);
  double &phi0 = matel(xp, 3, 1);

  hor_matq_fill(xpredefRES,
		sphidef * (x0 - xdef) + cphidef * (z0 - zdef),
		cphidef * (x0 - xdef) - sphidef * (z0 - zdef),
		phi0 - phidef);
  and_pi_range(matel(xpredefRES, 3, 1));

  hor_matq_fill(dxpredef_by_dxpRES,
		cphidef, sphidef, 0.0,
		-sphidef, cphidef, 0.0,
		0.0, 0.0, 1.0);

  hor_matq_fill(dxpredef_by_dxpdefRES,
	-cphidef, -sphidef, (x0 - xdef) * cphidef - (z0 - zdef) * sphidef,
	sphidef, -cphidef, -(x0 - xdef) * sphidef - (z0 - zdef) * cphidef,
		0.0, 0.0, -1.0);

  return 0;
}

int TwoD_Motion_Model::func_xpose_and_Rpose(Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  func_xp(xv);

  // Turn xp vector (z, x, phi) into pose
  hor_matq_fill(xposeRES, 
		matel(xpRES, 2, 1), 
		0.0, 
		matel(xpRES, 1, 1));

  hor_matq_fill(RposeRES,
                cos(matel(xpRES, 3, 1)), 0.0, sin(matel(xpRES, 3, 1)),
                0.0, 1.0, 0.0,
               -sin(matel(xpRES, 3, 1)), 0.0, cos(matel(xpRES, 3, 1)));
                                        /* Rpose is MLC0 */
  return 0;
}


int TwoD_Motion_Model::bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
					 Hor_Matrix *xptest)
{
  assert(xp1 != NULL || xp2 != NULL || xptest != NULL);
  
  assert(xp1->rows == POSITION_STATE_SIZE && xp1->cols == 1 &&
	 xp2->rows == POSITION_STATE_SIZE && xp2->cols == 1 &&
	 xptest->rows == POSITION_STATE_SIZE && xptest->cols == 1);

  //  cout << "xp1" << xp1 << "xp2" << xp2 << "xptest" << xptest;

  // TwoD position state xp (z, x, phi)
  // Only test z and x
  return (matel(xptest, 1, 1) >= matel(xp1, 1, 1) &&
	  matel(xptest, 1, 1) < matel(xp2, 1, 1) &&
	  matel(xptest, 2, 1) >= matel(xp1, 2, 1) &&
	  matel(xptest, 2, 1) < matel(xp2, 2, 1));
}

int OneD_Motion_Model::func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef
    (Hor_Matrix *xp, Hor_Matrix *xpdef)
{
  assert (xp != NULL && xpdef != NULL);

  assert (xp->rows == STATE_SIZE && xp->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  double &zdef = matel(xpdef, 1, 1);
  double &z0 = matel(xp, 1, 1);

  hor_matq_fill(xpredefRES,
		z0 - zdef);

  hor_matq_fill(dxpredef_by_dxpRES,
		1.0);

  hor_matq_fill(dxpredef_by_dxpdefRES,
		-1.0);

  return 0;
}

int OneD_Motion_Model::func_xpose_and_Rpose(Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  func_xp(xv);

  // Turn xp vector (z) into pose
  hor_matq_fill(xposeRES, 
		0.0, 
		0.0, 
		matel(xpRES, 1, 1));

  hor_matq_identity(RposeRES);
                                        /* Rpose is MLC0 */
  return 0;
}

int OneD_Motion_Model::bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
					 Hor_Matrix *xptest)
{
  assert(xp1 != NULL || xp2 != NULL || xptest != NULL);
  
  assert(xp1->rows == POSITION_STATE_SIZE && xp1->cols == 1 &&
	 xp2->rows == POSITION_STATE_SIZE && xp2->cols == 1 &&
	 xptest->rows == POSITION_STATE_SIZE && xptest->cols == 1);

  return (matel(xptest, 1, 1) >= matel(xp1, 1, 1) &&
	  matel(xptest, 1, 1) < matel(xp2, 1, 1));
}

/**********************Feature Measurement Model Functions********************/

// Feature measurement model: base class
Feature_Measurement_Model::Feature_Measurement_Model(int measurement_size, 
			    int feature_state_size, Motion_Model *m_m, 
			    char *f_t, char *f_d_t, int pose_size)
  : MEASUREMENT_SIZE(measurement_size), 
    FEATURE_STATE_SIZE(feature_state_size),
    motion_model(m_m),
    feature_type(f_t),
    feature_dimensionality_type(f_d_t),
    POSE_SIZE(pose_size)
{
  // Allocate matrices for storage of results
  yiposeRES = hor_mat_alloc(POSE_SIZE, 1);
  PyiyiposeRES = hor_mat_alloc(POSE_SIZE, POSE_SIZE);
  yiRES = hor_mat_alloc(FEATURE_STATE_SIZE, 1);
  dyi_by_dxpRES = hor_mat_alloc(FEATURE_STATE_SIZE, 
				motion_model->POSITION_STATE_SIZE);
  dyi_by_dhiRES = hor_mat_alloc(FEATURE_STATE_SIZE, 
				MEASUREMENT_SIZE);
  hiRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  dhi_by_dxpRES = hor_mat_alloc(MEASUREMENT_SIZE, 
				motion_model->POSITION_STATE_SIZE);
  dhi_by_dyiRES = hor_mat_alloc(MEASUREMENT_SIZE, FEATURE_STATE_SIZE);
  RiRES = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  nuiRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  SiRES = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  hi_noisyRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  zeroedyiRES = hor_mat_alloc(FEATURE_STATE_SIZE, 1);
  dzeroedyi_by_dxpRES = hor_mat_alloc(FEATURE_STATE_SIZE, 
				      motion_model->POSITION_STATE_SIZE);
  dzeroedyi_by_dyiRES = hor_mat_alloc(FEATURE_STATE_SIZE, FEATURE_STATE_SIZE);

  // Allocate matrices for storage of temporary results
  Temp_FF1 = hor_mat_alloc(FEATURE_STATE_SIZE, FEATURE_STATE_SIZE);
  Temp_FF2 = hor_mat_alloc(FEATURE_STATE_SIZE, FEATURE_STATE_SIZE);
  Temp_FS1 = hor_mat_alloc(FEATURE_STATE_SIZE, motion_model->STATE_SIZE);
  Temp_MM1 = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  Temp_MM2 = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  Temp_MF1 = hor_mat_alloc(MEASUREMENT_SIZE, FEATURE_STATE_SIZE); 
}

void Feature_Measurement_Model::Feature_Measurement_Model_destructor()
{
  hor_mat_free_list(yiposeRES, PyiyiposeRES, yiRES, dyi_by_dxpRES, 
		    dyi_by_dhiRES,
		    hiRES, dhi_by_dxpRES, dhi_by_dyiRES, RiRES, nuiRES, SiRES,
		    hi_noisyRES, zeroedyiRES, dzeroedyi_by_dxpRES, 
		    dzeroedyi_by_dyiRES, Temp_FF1, Temp_FF2, Temp_FS1, 
		    Temp_MM1, Temp_MM2, Temp_MF1, NULL);
}

// This function for calculating innovation covariance is generic
// to all feature measurement models, but it is good to include it
// in the models hierarchy for efficient implementation
int Feature_Measurement_Model::func_Si(Hor_Matrix *Pxx, Hor_Matrix *Pxyi, 
                	      Hor_Matrix *Pyiyi, Hor_Matrix *dhi_by_dxv, 
                              Hor_Matrix *dhi_by_dyi, Hor_Matrix *Ri)
{
  assert(Pxx != NULL && Pxyi != NULL && Pyiyi != NULL && 
	 dhi_by_dxv != NULL && dhi_by_dyi != NULL && Ri != NULL);

  assert(Pxx->rows == motion_model->STATE_SIZE && 
	 Pxx->cols == motion_model->STATE_SIZE);
  assert(Pxyi->rows == motion_model->STATE_SIZE && 
	 Pxyi->cols == FEATURE_STATE_SIZE);
  assert(Pyiyi->rows == FEATURE_STATE_SIZE &&
	 Pyiyi->cols == FEATURE_STATE_SIZE);
  assert(dhi_by_dxv->rows == MEASUREMENT_SIZE &&
	 dhi_by_dxv->cols == motion_model->STATE_SIZE);
  assert(dhi_by_dyi->rows == MEASUREMENT_SIZE &&
	 dhi_by_dyi->cols == FEATURE_STATE_SIZE);
  assert(Ri->rows == MEASUREMENT_SIZE && Ri->cols == MEASUREMENT_SIZE);

  // Zero SiRES and add bits on
  hor_matq_zero(SiRES);

  hor_matq_ABAT(dhi_by_dxv, Pxx, Temp_MM1);
  hor_matq_add2(Temp_MM1, SiRES, SiRES);

  hor_matq_prod2(dhi_by_dxv, Pxyi, Temp_MF1);
  hor_matq_ABT(Temp_MF1, dhi_by_dyi, Temp_MM1);
  hor_matq_add2(Temp_MM1, SiRES, SiRES);

  hor_matq_transpose(Temp_MM1, Temp_MM2);
  hor_matq_add2(Temp_MM2, SiRES, SiRES);

  hor_matq_ABAT(dhi_by_dyi, Pyiyi, Temp_MM1);
  hor_matq_add2(Temp_MM1, SiRES, SiRES);

  hor_matq_add2(Ri, SiRES, SiRES);

  return 0;
}

Internal_Measurement_Model::Internal_Measurement_Model(int measurement_size, 
						       Motion_Model *m_m, 
						       char *i_t)
  : MEASUREMENT_SIZE(measurement_size),
    motion_model(m_m),
    internal_type(i_t)
{
  hvRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  dhv_by_dxvRES = hor_mat_alloc(MEASUREMENT_SIZE, motion_model->STATE_SIZE);
  RvRES = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  nuvRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  SvRES = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  hv_noisyRES = hor_mat_alloc(MEASUREMENT_SIZE, 1);
  Temp_MM1 = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
  Temp_MM2 = hor_mat_alloc(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
}


void Internal_Measurement_Model::Internal_Measurement_Model_destructor()
{
  hor_mat_free_list(hvRES, dhv_by_dxvRES, RvRES, nuvRES, SvRES, hv_noisyRES, 
		    Temp_MM1, Temp_MM2, NULL);
}

int Internal_Measurement_Model::func_Sv(Hor_Matrix *Pxx, 
					Hor_Matrix *dhv_by_dxv, 
					Hor_Matrix *Rv)
{
  assert(Pxx != NULL && dhv_by_dxv != NULL && Rv != NULL);

  assert(Pxx->rows == motion_model->STATE_SIZE && 
	 Pxx->cols == motion_model->STATE_SIZE);
  assert(dhv_by_dxv->rows == MEASUREMENT_SIZE && 
	 dhv_by_dxv->cols == motion_model->STATE_SIZE);
  assert(Rv->rows == MEASUREMENT_SIZE && Rv->cols == MEASUREMENT_SIZE);

  // Zero SiRES and add bits on
  hor_matq_zero(SvRES);

  hor_matq_ABAT(dhv_by_dxv, Pxx, Temp_MM1);
  hor_matq_add2(Temp_MM1, SvRES, SvRES);

  hor_matq_add2(Rv, SvRES, SvRES);

  return 0;
}
