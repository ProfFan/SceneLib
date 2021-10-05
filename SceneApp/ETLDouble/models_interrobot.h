/*  SceneApp: applications for sequential localisation and map-building

    models_interrobot.h
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

// Internal measurement model for measurement when the vision robot observes 
// the marker on the second robot

// Marker position relative to second robot
const double MARKER_X = -0.22;   // left/right
const double MARKER_Y = 0.985;   // height
const double MARKER_Z = 0.0;     // front/back

class Double_Marker_Internal_Measurement_Model 
                          : public Internal_Measurement_Model
{
 public:
  // Constructor
  Double_Marker_Internal_Measurement_Model(Motion_Model *m_m, 
				    Feature_Measurement_Model *f_m_m);

  ~Double_Marker_Internal_Measurement_Model();

  // Redefined virtual functions
  int func_hv_and_dhv_by_dxv(Hor_Matrix *xv);
  int func_Rv(Hor_Matrix *hv);
  int func_nuv(Hor_Matrix *hv, Hor_Matrix *zv);
  int func_hv_noisy(Hor_Matrix *xv_true);
  int feasibility_test(Hor_Matrix *xv, Hor_Matrix *hv);

  // Extra label for motion_model to make clear it's always a DOUBLE
  Double_Motion_Model *double_motion_model;

  // Use a feature measurement model as the basis for measurements
  Feature_Measurement_Model *feature_measurement_model;

 protected:
  // Extra function to calculate the feature-like representation 
  // for robot 2: calculates marker position ym
  int func_ym(Hor_Matrix *xv);

  // For the feature-like representation of robot 2
  Hor_Matrix *yv2, *dyv2_by_dxv2, *dyv2_by_dxp2;
  Hor_Matrix *dhv_by_dxv1, *dhv_by_dxv2;
  Hor_Matrix *m_C0, *m_lab, *ym, *MlabC0, *dm_lab_by_dxp2;
  Hor_Matrix *dym_by_dxp2, *dym_by_dxv2;
};
