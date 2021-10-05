/*  Scene: software for sequential localisation and map-building

    models_base.h
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

// Models for vehicle motion, feature measurement

/********************************Motion Models********************************/
/* We have a derivation hierarchy here:
   Motion_Model should encompass everything

   ThreeD_Motion_Model is specific to 3D movement
   TwoD_Motion_Model is specific to 2D movement on the plane
   OneD_Motion_Model is for 1D movement along a line
*/

class Motion_Model
{
 public:
  Motion_Model(int position_state_size, int state_size, int control_size,
	       char *m_m_d_t, char *m_m_t);

  virtual ~Motion_Model() {}     
  void Motion_Model_destructor();    
                                 /* Standard practice to have a virtual
				    destructor for an abstract class,
				    but we have an explicit function to 
				    clear up matrices defined in base
				    class */

  const int POSITION_STATE_SIZE; /* The number of parameters to minimally
                                    represent position in this dimension */
  const int STATE_SIZE;          /* The number of parameters we will actually
				    use in the state vector (allows for more
				    generality) */
  const int CONTROL_SIZE;        /* Number of parameters in control vector */
  char *motion_model_dimensionality_type; /* String identifier */
  char *motion_model_type;       /* String identifier */

  /****The following virtual functions must be supplied by a derived class****/

  // Process equation and Jacobian
  virtual int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, Hor_Matrix *u, 
				     double delta_t) = 0;

  // Process Noise
  virtual int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t) = 0;

  // Position State: minimalistic representation of position in a certain
  // dimension
  virtual int func_xp(Hor_Matrix *xv) = 0;

  // Jacobian for position state
  virtual int func_dxp_by_dxv(Hor_Matrix *xv) = 0;

  // Noisy process equation for simulation
  virtual int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, 
			    double delta_t) = 0;

  // How to adjust the state if the coordinate frame is redefined so that
  // xpdef (in position coordinates) becomes the new origin
  virtual int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef) = 0;

  /******These virtual functions are optional; not pure virtual functions*****/
  /**********and they have default null instances. Define if necessary.*******/

  // Normalise state vector if there is a redundant representation
  virtual int func_xvnorm_and_dxvnorm_by_dxv(Hor_Matrix *xv);

  // Set control vector to get to a waypoint
  virtual int navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xv_goal, 
				   Hor_Matrix *u, double delta_t);

  /******This function is defined in models base.cc, and should only be*******/
  /******************redefined in special cases (DOUBLE)**********************/

  // How to "zero" the axes at the current robot position
  virtual int func_zeroedxv_and_dzeroedxv_by_dxv(Hor_Matrix *xv);

  /******These functions are defined in the dimension model types below*******/
  /****and therefore do not need to be individually defined for each model****/

  // How to adjust the position state if the coordinate frame is redefined so 
  // that xpdef (in position coordinates) becomes the new origin
  virtual int func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
    (Hor_Matrix *xp, Hor_Matrix *xpdef) = 0;

  // Pose parameters: 3D position and rotation matrix for graphics, etc.
  // Defined for each dimensionality
  virtual int func_xpose_and_Rpose(Hor_Matrix *xv) = 0;

  // Test whether a position state lies withing a bounding box
  virtual int bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
				Hor_Matrix *xptest) = 0;

  /*************This function is defined in models base.cc, and***************/
  /**********************does not neet to be redefined************************/
  // For testing purposes only:
  // Check if Jacobians are making sense
  int jacobians_test(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  // Where results will be stored after function calls
  Hor_Matrix *fvRES, *dfv_by_dxvRES, *QxRES, 
    *xpRES, *dxp_by_dxvRES, *fv_noisyRES,
    *xvredefRES, *dxvredef_by_dxvRES, *dxvredef_by_dxpdefRES, 
    *xvnormRES, *dxvnorm_by_dxvRES,
    *zeroedxvRES, *dzeroedxv_by_dxvRES, 
    *xpredefRES, *dxpredef_by_dxpRES, *dxpredef_by_dxpdefRES,     
    *xposeRES, *RposeRES;

  // Matrices available for calculations
  Hor_Matrix *Temp_SS1;
};

// 3D general model
// Position state (standard representation): (x, y, z, q0, q1, q2, q3)
class ThreeD_Motion_Model : public Motion_Model
{
 public:
  ThreeD_Motion_Model(int state_size, int control_size, char *m_m_t);
  ~ThreeD_Motion_Model();

  // General motion model functions
  int func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
    (Hor_Matrix *xp, Hor_Matrix *xpdef);
  int func_xpose_and_Rpose(Hor_Matrix *xv);
  int bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
			Hor_Matrix *xptest);

  // Functions specific to 3D case

  // Extract cartesian part of position state vector
  int func_x(Hor_Matrix *xp);

  // Extract quaternion part of position state vector
  int func_q(Hor_Matrix *xp);

  // Results matrices
  Hor_Matrix *xRES, *qRES;

 protected:
  // Temporary matrices for calculations
  Hor_Matrix *x0, *q0, *xn, *qn, *qnbar, 
    *Temp31a, *Temp31b, *Temp31c, *Temp33a, *Temp33b, *Tempqa, *Tempqb, 
    *Temp44a, *Temp44b, *Temp44c, *Temp34a, *Temp34b;
};


// 2D planar model
// Position state (minimal representation): (z, x, phi)
class TwoD_Motion_Model : public Motion_Model
{
 public:
  TwoD_Motion_Model(int state_size, int control_size, char *m_m_t)
    : Motion_Model(3, state_size, control_size, "TWOD", m_m_t) 
    {}

  int func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
    (Hor_Matrix *xp, Hor_Matrix *xpdef);
  int func_xpose_and_Rpose(Hor_Matrix *xv);
  int bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
			Hor_Matrix *xptest);
};


// 1D model
// Position state (minimal representation): (z)
class OneD_Motion_Model : public Motion_Model
{
public:
  OneD_Motion_Model(int state_size, int control_size, char *m_m_t)
    : Motion_Model(1, state_size, control_size, "ONED", m_m_t) 
    {}

  int func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
    (Hor_Matrix *xp, Hor_Matrix *xpdef);
  int func_xpose_and_Rpose(Hor_Matrix *xv);
  int bounding_box_test(Hor_Matrix *xp1, Hor_Matrix *xp2, 
			Hor_Matrix *xptest);
};

/*************************Feature Measurement Models**************************/
/* Derivation hierarchy:
   
   Feature_Measurement_Model

   Point_Feature_Measurement_Model for points
   Line_Feature_Measurement_Model for lines (not yet implemented)

   Then specific types for hardware
*/


// feature_type possibilities: "POINT", "LINE" for 2D
// 

class Feature_Measurement_Model
{
 public:
  // Constructor
  Feature_Measurement_Model(int measurement_size, int feature_state_size, 
			    Motion_Model *m_m, char *f_t, char *f_d_t,
			    int pose_size);

  virtual ~Feature_Measurement_Model() {}
  void Feature_Measurement_Model_destructor();
                                 /* Standard practice to have a virtual
				    destructor for an abstract class,
				    but we have an explicit function to 
				    clear up matrices defined in base
				    class */

  // Constants: set in initialisation list for individual models
  const int MEASUREMENT_SIZE;
  const int FEATURE_STATE_SIZE;
  Motion_Model *motion_model;
  char *feature_type;
  char *feature_dimensionality_type;
  const int POSE_SIZE;

  /* Calculation functions which define the model.  Some of these functions 
     produce several results: this is for efficiency reaons when these 
     results often need to be used together */

  // Get 3D pose (and its covariance) from state: for graphics, etc. 
  // Sometimes actual state representation
  // will have extra parameters, etc.
  virtual int func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi) = 0;

  // For initialising a feature
  virtual int func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
						   Hor_Matrix *xp) = 0;

  // Measurement function
  virtual int func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
						    Hor_Matrix *xp) = 0;

  // Measurement noise
  virtual int func_Ri(Hor_Matrix *hi) = 0;

  // Calculate innovation
  virtual int func_nui(Hor_Matrix *hi, Hor_Matrix *zi) = 0;

  // Noisy measurement for use in simulation
  virtual int func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true) = 0;

  // Function which calculates feature state as it would be if the 
  // robot state was defined at position xp
  // (This function replaces hiEuc)
  virtual int func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		    Hor_Matrix *yi, Hor_Matrix *xp) = 0;

  // Test for visibility
  virtual int visibility_test(Hor_Matrix *xp, Hor_Matrix *yi, 
			      Hor_Matrix *xp_orig, 
			      Hor_Matrix *hi) = 0;

  // Score for selecting features
  virtual double selection_score(Hor_Matrix *Si) = 0;

  // Innovation Covariance: this is not a virtual function as it is generic
  int func_Si(Hor_Matrix *Pxx, Hor_Matrix *Pxyi, Hor_Matrix *Pyiyi, 
	      Hor_Matrix *dhi_by_dxv, Hor_Matrix *dhi_by_dyi, 
	      Hor_Matrix *Ri);

  // Where results will be stored after calls to functions
  Hor_Matrix *yiposeRES, *PyiyiposeRES, *yiRES, *dyi_by_dxpRES, *dyi_by_dhiRES,
    *hiRES, *dhi_by_dxpRES, *dhi_by_dyiRES, *RiRES, *nuiRES, *SiRES,
    *hi_noisyRES, *zeroedyiRES, *dzeroedyi_by_dxpRES, *dzeroedyi_by_dyiRES;

  // Model also supplies some storage-space matrices for use in
  // calculations
  Hor_Matrix *Temp_FF1, *Temp_FF2, *Temp_FS1, *Temp_MM1, *Temp_MM2, *Temp_MF1;
};


// Feature Measurement Model specific to points
// Specifies feature_dimensionality_type "POINT" and POSE_SIZE 3
class Point_Feature_Measurement_Model : public Feature_Measurement_Model
{
 public:
  Point_Feature_Measurement_Model(int measurement_size, 
				  int feature_state_size, 
				  Motion_Model *m_m, char *f_t)
    : Feature_Measurement_Model(measurement_size, feature_state_size, 
			    m_m, f_t, "POINT", 3)
    {}
};

// Feature Measurement Model specific to lines
// Specifies feature_dimensionality_type "LINE" and POSE_SIZE 6
class Line_Feature_Measurement_Model : public Feature_Measurement_Model
{
 public:
  Line_Feature_Measurement_Model(int measurement_size, 
				  int feature_state_size, 
				  Motion_Model *m_m, char *f_t)
    : Feature_Measurement_Model(measurement_size, feature_state_size, 
			    m_m, f_t, "LINE", 6)
    {}
};

/**************************Internal Measurement Model*************************/

// For making measurements internal to the robot state

class Internal_Measurement_Model
{
 public:
  // Constructor
  Internal_Measurement_Model(int measurement_size, Motion_Model *m_m, 
			     char *i_t);

  virtual ~Internal_Measurement_Model() {}
  void Internal_Measurement_Model_destructor();
                                 /* Standard practice to have a virtual
				    destructor for an abstract class,
				    but we have an explicit function to 
				    clear up matrices defined in base
				    class */

  // Constants: set in initialisation list for individual models
  const int MEASUREMENT_SIZE;
  Motion_Model *motion_model;
  char *internal_type;

  /* Calculation functions which define the model.  Some of these functions 
     produce several results: this is for efficiency reaons when these 
     results often need to be used together */

  // Measurement function
  virtual int func_hv_and_dhv_by_dxv(Hor_Matrix *xv) = 0;

  // Measurement noise
  virtual int func_Rv(Hor_Matrix *hv) = 0;

  // Innovation calculation
  virtual int func_nuv(Hor_Matrix *hv, Hor_Matrix *zv) = 0;

  // Noisy measurement for use in simulation
  virtual int func_hv_noisy(Hor_Matrix *xv_true) = 0;

  // Test for feasibility of measurement
  virtual int feasibility_test(Hor_Matrix *xv, Hor_Matrix *hv) = 0;

  // Innovation Covariance: this is not a virtual function as it is generic
  int func_Sv(Hor_Matrix *Pxx, Hor_Matrix *dhv_by_dxv, Hor_Matrix *Rv);

  // Where results will be stored after calls to functions
  Hor_Matrix *hvRES, *dhv_by_dxvRES, *RvRES, *nuvRES, *SvRES, *hv_noisyRES;

  // Calculation space matrices
  Hor_Matrix *Temp_MM1, *Temp_MM2;
};
