/*  SceneApp: applications for sequential localisation and map-building

    models_interrobot.cc
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
#include "models_double.h"
#include "models_interrobot.h"


Double_Marker_Internal_Measurement_Model::
      Double_Marker_Internal_Measurement_Model(Motion_Model *m_m, 
				    Feature_Measurement_Model *f_m_m)
	: Internal_Measurement_Model(f_m_m->MEASUREMENT_SIZE, 
				     m_m, "DOUBLE_MARKER"),
	  feature_measurement_model(f_m_m)
{
  assert(strcmp(motion_model->motion_model_type, "DOUBLE") == 0);
  double_motion_model = (Double_Motion_Model *) motion_model;

  yv2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE, 1);
  dyv2_by_dxv2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			       double_motion_model->STATE_SIZE2);
  dyv2_by_dxp2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			       double_motion_model->POSITION_STATE_SIZE2);
  dhv_by_dxv1 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			      double_motion_model->STATE_SIZE1);
  dhv_by_dxv2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			      double_motion_model->STATE_SIZE2);
  m_C0 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
				1);
  m_lab = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
				1);
  ym = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
				1);
  MlabC0 = hor_mat_alloc(3, 3);
  dm_lab_by_dxp2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
				 double_motion_model->POSITION_STATE_SIZE2);
  dym_by_dxp2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			      double_motion_model->POSITION_STATE_SIZE2);
  dym_by_dxv2 = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			      double_motion_model->STATE_SIZE2);
}

Double_Marker_Internal_Measurement_Model::
      ~Double_Marker_Internal_Measurement_Model()
{
  hor_mat_free_list(yv2, dyv2_by_dxv2, dyv2_by_dxp2, dhv_by_dxv1, 
		    dhv_by_dxv2, m_C0, m_lab, ym, MlabC0, dm_lab_by_dxp2, 
		    dym_by_dxp2, dym_by_dxv2, NULL);
  Internal_Measurement_Model_destructor();
}

int Double_Marker_Internal_Measurement_Model::
                         func_hv_and_dhv_by_dxv(Hor_Matrix *xv)
{
  assert(xv != NULL);
  assert(xv->rows == double_motion_model->STATE_SIZE && xv->cols == 1);

  // Make like second robot is a feature 
  func_ym(xv);

  double_motion_model->func_xp1(xv);
  feature_measurement_model->func_hi_and_dhi_by_dxp_and_dhi_by_dyi(ym,
				   double_motion_model->xp1RES);

  hor_matq_copy(feature_measurement_model->hiRES, hvRES);

  // Jacobian
  // Vehicle 1 bit
  double_motion_model->func_dxp1_by_dxv1(xv);

  hor_matq_prod2(feature_measurement_model->dhi_by_dxpRES, 
		 double_motion_model->dxp1_by_dxv1RES,
		 dhv_by_dxv1);

  // Vehicle 2 bit
  // ym (marker position) = yv2 (vehicle 2 position in feature coordinates)
  //                        + m_lab (marker position relative to vehicle 2)
  hor_matq_fill(dyv2_by_dxp2,
		0.0, 1.0, 0.0,
		0.0, 0.0, 0.0,
		1.0, 0.0, 0.0);

  double cphi = cos(matel(double_motion_model->xp2RES, 3, 1));
  double sphi = sin(matel(double_motion_model->xp2RES, 3, 1));
  hor_matq_fill(dm_lab_by_dxp2,
		0.0, 0.0, -MARKER_X * sphi + MARKER_Z * cphi,
		0.0, 0.0, 0.0,
		0.0, 0.0, -MARKER_X * cphi - MARKER_Z * sphi);

  hor_matq_add2(dyv2_by_dxp2, dm_lab_by_dxp2, dym_by_dxp2);

  //  cout << "dym_by_dxp2" << dym_by_dxp2;

  double_motion_model->func_dxp2_by_dxv2(xv);

  hor_matq_prod2(dym_by_dxp2, double_motion_model->dxp2_by_dxv2RES, 
		 dym_by_dxv2);
  hor_matq_prod2(feature_measurement_model->dhi_by_dyiRES, dym_by_dxv2,
		 dhv_by_dxv2);

  //  cout << "dhv_by_dxv1" << dhv_by_dxv1 << "dhv_by_dxv2" << dhv_by_dxv2 ;

  hor_matq_insert_chunkyx(dhv_by_dxv1, dhv_by_dxvRES, 0, 0);
  hor_matq_insert_chunkyx(dhv_by_dxv2, dhv_by_dxvRES, 
			  0, double_motion_model->STATE_SIZE1);  

  return 0;
}


int Double_Marker_Internal_Measurement_Model::func_Rv(Hor_Matrix *hv)
{
  assert(hv != NULL);
  assert(hv->rows == MEASUREMENT_SIZE && hv->cols == 1);

  feature_measurement_model->func_Ri(hv);

  hor_matq_copy(feature_measurement_model->RiRES, RvRES);

  return 0;
}

int Double_Marker_Internal_Measurement_Model::func_nuv(Hor_Matrix *hv, 
						       Hor_Matrix *zv)
{
  assert(hv != NULL && zv != NULL);
  assert(hv->rows == MEASUREMENT_SIZE && hv->cols == 1 &&
	 zv->rows == MEASUREMENT_SIZE && zv->cols == 1);

  feature_measurement_model->func_nui(hv, zv);

  hor_matq_copy(feature_measurement_model->nuiRES, nuvRES);

  return 0;
}

int Double_Marker_Internal_Measurement_Model::
                         func_hv_noisy(Hor_Matrix *xv_true)
{
  assert(xv_true != NULL);
  assert(xv_true->rows == double_motion_model->STATE_SIZE && 
	 xv_true->cols == 1);

  func_ym(xv_true);

  double_motion_model->func_xp1(xv_true);
  feature_measurement_model->func_hi_noisy(ym, double_motion_model->xp1RES);

  hor_matq_copy(feature_measurement_model->hi_noisyRES, hv_noisyRES);

  return 0;
}

int Double_Marker_Internal_Measurement_Model::feasibility_test(Hor_Matrix *xv, 
							       Hor_Matrix *hv)
{
  assert(xv != NULL && hv != NULL);
  assert(xv->rows == double_motion_model->STATE_SIZE && xv->cols == 1 &&
	 hv->rows == MEASUREMENT_SIZE && hv->cols == 1);

  // We use the normal feature visibility test, but setting xp_orig to
  // the current xp so the only thing that is tested is the head axis
  // limits

  double_motion_model->func_xp1(xv);

  func_ym(xv);

  return feature_measurement_model->visibility_test(double_motion_model->xp1RES,
			      ym, double_motion_model->xp1RES, hv);
}


int Double_Marker_Internal_Measurement_Model::func_ym(Hor_Matrix *xv)
{
  assert(xv != NULL);
  assert(xv->rows == double_motion_model->STATE_SIZE && xv->cols == 1);

  // Make like second robot is a feature 
  double_motion_model->func_xp2(xv);

  double cphi = cos(matel(double_motion_model->xp2RES, 3, 1));
  double sphi = sin(matel(double_motion_model->xp2RES, 3, 1));

  hor_matq_fill(MlabC0,
		cphi, 0.0, sphi,
		0.0,  1.0, 0.0,
		-sphi, 0.0, cphi);   // Transforms from second robot frame
                                     // to lab
		
  hor_matq_fill(m_C0,
		MARKER_X, MARKER_Y, MARKER_Z);
					
  hor_matq_prod2(MlabC0, m_C0, m_lab);

  hor_matq_fill(yv2,
		matel(double_motion_model->xp2RES, 2, 1),
		0.0,
		matel(double_motion_model->xp2RES, 1, 1));

  hor_matq_add2(yv2, m_lab, ym);

  //  cout << "xp2" << double_motion_model->xp2RES << "yv2" << yv2
  //       << "ym" << ym;

  return 0;
}
