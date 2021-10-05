/*  SceneApp: applications for sequential localisation and map-building

    models_rollpitch.h
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

// Internal Measurement Model for measurements from roll/pitch sensor
// which works with any 3D robot motion model
class Roll_Pitch_Sensor_Internal_Measurement_Model 
                          : public Internal_Measurement_Model
{
 public:
  // Constructor
  Roll_Pitch_Sensor_Internal_Measurement_Model(Motion_Model *m_m);

  ~Roll_Pitch_Sensor_Internal_Measurement_Model();

  // Constants

  // Standard deviation for angular measurements
  // Used in filter
  static const double SD_angle_filter = 0.4 * M_PI / 180.0;
  // Used in simulator (i.e. can potentially be different) 
  static const double SD_angle = 0.4 * M_PI / 180.0; 


  // Redefined virtual functions
  int func_hv_and_dhv_by_dxv(Hor_Matrix *xv);
  int func_Rv(Hor_Matrix *hv);
  int func_nuv(Hor_Matrix *hv, Hor_Matrix *zv);
  int func_hv_noisy(Hor_Matrix *xv_true);
  int feasibility_test(Hor_Matrix *xv, Hor_Matrix *hv);

 protected:
  // Matrices for calculation
  Hor_Matrix *local_xp, *qbar, *dqbarby_dq, *Rqbar, *yWW, *yWR, *dhv_by_dxp,
    *dyWR_by_dqbar, *dyWR_by_dq, *dhv_by_dyWR, *dhv_by_dq;

  // We know motion model is a three D one so have this pointer to it too
  ThreeD_Motion_Model *threed_motion_model;
};
