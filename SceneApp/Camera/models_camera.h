/*  Scene: software for sequential localisation and map-building

    Camera/models_camera.h
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

/// Model for single camera making point measurements

class Camera_Point_Feature_Measurement_Model 
                                      : public Point_Feature_Measurement_Model 
{
 public:

  // Constants

  // Image size
  static const int IMAGE_WIDTH = 320;
  static const int IMAGE_HEIGHT = 240;

  // Camera calibration parameters
  static const double Fku = 372.0;   
  static const double Fkv = 363.0;			
  static const double U0 = 160.0;
  static const double V0 = 134.0;      

  // Image measurement uncertainty (in pixels) for filter and simulation
  static const double SD_image_filter = 1.0;
  static const double SD_image = 1.0;

  // For visibility tests
  static const int LEFT_RIGHT_FAIL = 1;
  static const int UP_DOWN_FAIL = 2;
  static const int DISTANCE_FAIL = 4;
  static const int ANGLE_FAIL = 8;
  static const double MAXIMUM_LENGTH_RATIO = 1.4;
  static const double MAXIMUM_ANGLE_DIFFERENCE = M_PI * 35.0 / 180.0;

  // Constructor
  Camera_Point_Feature_Measurement_Model(Motion_Model *motion_model);
  ~Camera_Point_Feature_Measurement_Model();

  // Redefined virtual functions
  int func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi);

  int func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
					    Hor_Matrix *xp);

  int func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
					   Hor_Matrix *xp);

  int func_Ri(Hor_Matrix *hi);

  int func_nui(Hor_Matrix *hi, Hor_Matrix *zi);

  int func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true);

  int func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		    Hor_Matrix *yi, Hor_Matrix *xp);
  int visibility_test(Hor_Matrix *xp, Hor_Matrix *yi, 
			      Hor_Matrix *xp_orig, 
			      Hor_Matrix *hi);

  double selection_score(Hor_Matrix *Si);

 protected:
  // Another label for the motion model of the base class
  ThreeD_Motion_Model *threed_motion_model;

  // Camera calibration matrix and inverse
  Hor_Matrix *C, *Cinv;

  // Matrices for calculation
  Hor_Matrix *du_by_dhR;
  Hor_Matrix *yminusxW, *qRW, *dqRW_by_dq, *RRW, *dzeroedyi_by_dx, 
    *dzeroedyi_by_dqRW, *dzeroedyi_by_dq;
  Hor_Matrix *hiLlab, *MlabC0, *hiL_origlab, *MlabC0_orig;
};

