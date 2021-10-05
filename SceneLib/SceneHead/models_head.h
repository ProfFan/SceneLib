/*  Scene: software for sequential localisation and map-building

    models_head.h
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

// Model for pan-tilt active heads

// For visibility tests
const int PAN_RANGE_FAIL = 1;
const int ELEVATION_RANGE_FAIL = 2;
const int DISTANCE_FAIL = 4;
const int ANGLE_FAIL = 8; 


// Currently 2D and 3D head models separate
// Eventually combine into 1 class
class Head_Point_Feature_Measurement_Model 
                                      : public Point_Feature_Measurement_Model 
{
 public:
  // Constructor
  Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double Ii, double Hh,
       double PAN_RANGE_LIMIT, double ELEVATION_RANGE_LIMIT, 
       double VERGENCE_RANGE_LIMIT,
       double MAXIMUM_ANGLE_RATIO, double MAXIMUM_ANGLE_DIFFERENCE,
       double SD_alpha, double SD_e, double SD_gamma,
       double SD_alpha_filter, double SD_e_filter, double SD_gamma_filter);
  virtual ~Head_Point_Feature_Measurement_Model() {};

  // Redefined virtual functions
  virtual int func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi) = 0;

  virtual int func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
					    Hor_Matrix *xp) = 0;

  virtual int func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
					   Hor_Matrix *xp) = 0;

  virtual int func_Ri(Hor_Matrix *hi) = 0;

  virtual int func_nui(Hor_Matrix *hi, Hor_Matrix *zi) = 0;

  virtual int func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true) = 0;

  virtual int func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		    Hor_Matrix *yi, Hor_Matrix *xp) = 0;
  virtual int visibility_test(Hor_Matrix *xp, Hor_Matrix *yi, 
			      Hor_Matrix *xp_true, 
			      Hor_Matrix *hi) = 0;

  virtual double selection_score(Hor_Matrix *Si) = 0;

  // Constants defining this head
  // Inter-ocular separation
  const double Ii;   
  // Height of head centre above the ground
  const double Hh;   

  // Safe Escher range limits
  const double PAN_RANGE_LIMIT;
  const double ELEVATION_RANGE_LIMIT;
  const double VERGENCE_RANGE_LIMIT;

  // Values for visibility tests
  const double MAXIMUM_LENGTH_RATIO;
  const double MAXIMUM_ANGLE_DIFFERENCE;

  // Measurement errors for simulation
  // Pan measurement
  const double SD_alpha;
  // Elevation measurement
  const double SD_e;
  // Vergence measurement
  const double SD_gamma;

  // Measurement errors to use in filter
  // Pan measurement
  const double SD_alpha_filter;
  // Elevation measurement
  const double SD_e_filter;
  // Vergence measurement
  const double SD_gamma_filter;

};



class TwoD_Head_Point_Feature_Measurement_Model 
                               : public Head_Point_Feature_Measurement_Model 
{
 public:
  // Constructor
  TwoD_Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double Ii, double Hh,
       double PAN_RANGE_LIMIT, double ELEVATION_RANGE_LIMIT, 
       double VERGENCE_RANGE_LIMIT,
       double MAXIMUM_ANGLE_RATIO, double MAXIMUM_ANGLE_DIFFERENCE,
       double SD_alpha, double SD_e, double SD_gamma,
       double SD_alpha_filter, double SD_e_filter, double SD_gamma_filter);
  ~TwoD_Head_Point_Feature_Measurement_Model();

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
			      Hor_Matrix *xp_true, 
			      Hor_Matrix *hi);

  double selection_score(Hor_Matrix *Si);

 protected:
  // Matrices for use in calculations
  Hor_Matrix *dyi_by_dhiL, *dhiL_by_dhi, *hiL,
             *dhi_by_dhiL;
  // For Visibility Test:
  Hor_Matrix *MlabC0, *hiLlab, *hiL_origlab;
};


// Experimental class for use with 3D motion models
// Should eventually replace 2D one and be able to do both
class ThreeD_Head_Point_Feature_Measurement_Model 
                                : public Head_Point_Feature_Measurement_Model 
{
 public:
  // Constructor
  ThreeD_Head_Point_Feature_Measurement_Model(
       Motion_Model *m_m,
       double Ii, double Hh,
       double PAN_RANGE_LIMIT, double ELEVATION_RANGE_LIMIT, 
       double VERGENCE_RANGE_LIMIT,
       double MAXIMUM_ANGLE_RATIO, double MAXIMUM_ANGLE_DIFFERENCE,
       double SD_alpha, double SD_e, double SD_gamma,
       double SD_alpha_filter, double SD_e_filter, double SD_gamma_filter);
  ~ThreeD_Head_Point_Feature_Measurement_Model();

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
			      Hor_Matrix *xp_true, 
			      Hor_Matrix *hi);

  double selection_score(Hor_Matrix *Si);

 protected:
  // Specific 3D pointer to motion model
  ThreeD_Motion_Model *threed_motion_model;

  // Matrices for use in calculations
  Hor_Matrix *dyi_by_dhiL, *dhiL_by_dhi, *hiL,
             *dhi_by_dhiL, *RWR, *hiLW,
             *dyi_by_dx, *dyi_by_dq;

  Hor_Matrix *yminusxW, *qRW, *RRW,
             *dzeroedyi_by_dx, *dzeroedyi_by_dq,
             *dqRW_by_dq, *dzeroedyi_by_dqRW;


  // For Visibility Test:
  Hor_Matrix *MlabC0, *hiLlab, *hiL_origlab, 
    *hiL_orig, *MlabC0_orig;
};

