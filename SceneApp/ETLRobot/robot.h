/*  SceneApp: applications for sequential localisation and map-building
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

// Robot class to interface with hardware
/* Horatio colour identifiers */
extern u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, 
  Cyan;

/**********************************Constants**********************************/

const int LOCK_ON_ITERATIONS = 3;            /* How many iterations we will try
						when locking onto a feature */
const int SATISFIED = 2;                     /* How close we have to get in 
						pixels for a lock-on to be 
						acceptable */
const double LOCKSEARCHSIGMA = 8.0;          /* Standard deviation of circle 
						we will search for features 
						when we are supposedly 
						fixated */
const int PATCHES_TO_AUTO_FIND = 3;          /* The number of patches to 
						auto-find in an image. */
const int NEW_FEATURES = 3;                  /* The number of new features to 
						find automatically in 
						autonomous mode. */
const int BOXSIZE = 15;              /* Side length of square image patches */


// Camera parameters: in head.h

// How much to move the turret axis by so that ESCHeR faces forwards
const double TURRET_OFFSET = -90.0; // In degrees

/*****************************************************************************/

class Robot : public Sim_Or_Rob{
  friend class Head;
 public:
  Robot(FD_formrob *fd_fr, int zero_axes_flag, int reduced_image_flag,
	Head_Point_Feature_Measurement_Model 
	*head_point_feature_measurement_model,
	const double Ii, const double Hh, 
	int argc, char **argv);
  ~Robot();

  // Redefined virtual functions
  int measure_feature(void *id, Hor_Matrix *z, 
		      Hor_Matrix *h, Hor_Matrix *S);
  int set_control(Hor_Matrix *u, double delta_t);
  int continue_control(Hor_Matrix *u, double delta_t);
  double wait_for_end_of_motion(Hor_Matrix *u);
  int stop_vehicle();
  void *initialise_known_feature(Feature_Measurement_Model *f_m_m,
				 Hor_Matrix *yi,
				 int known_feature_label);
  int make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
					  Feature_Measurement_Model *f_m_m);

#ifdef _THREED_
  // Make measurement from roll/pitch sensor
  int make_internal_measurement(Internal_Measurement_Model 
             *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
	     Hor_Matrix *Sv);
#endif

  // Head control
  int move_head();
  int print_head_angles();
  int get_head_angles(double *alpha_ptr, double *e_ptr, 
                      double *gammaL_ptr, double *gammaR_ptr);
  int read_head_odometry();
  int wait_for_head();
  int set_pan(double new_alpha);
  int set_elevation(double new_e);

  int note_current_head_position();
  int return_to_noted_head_position();

  int move_head_to_new_point_feature_search_position(int i);

  // Grab Images
  int grab_and_display_images();

  // Display Images
  int display_images();

  // Write Images to disk
  int write_images();

  // Write image patch to disk
  int write_patch();

  // Read Images from disk
  int read_images();

  // Set the selected image point from externally
  int set_left_image_selection(int uL_, int vL_);
  int set_right_image_selection(int uR_, int vR_);

  // Initialise Point_Feature
  Hor_Image *Robot::initialise_point_feature(Hor_Matrix *z);

  // Lock onto a point_feature
  int lock_on_to_point(int *uLp, int *vLp, int *uRp, int *vRp);

  int print_robot_odometry();

  int rob_find_best_n_patches(int n, int *ubest, int *vbest, double *evbest);

  // Speak
  int say(char *speech_string);

protected:
  // So we can access XForms
  FD_formrob *fd_formrob;

  // Class containing head functions
  Head *head;

  // Model class for active head contains some parameters we need
  Head_Point_Feature_Measurement_Model 
    *head_point_feature_measurement_model;

  // Head angles in Radians   
  double alpha, e, gammaL, gammaR;   

  // For when we need to note a position to come back to later
  double alpha_note, e_note, gammaL_note, gammaR_note;

  // For noting positions in image
  int uL, vL, uR, vR;

  // For controlling the vehicle
  float v_control, dS_control, dT_control;

  // Timing using odometry
  int current_count, required_count, distance_left;
  double current_distance, required_distance;
  // Alternative with timer
  struct timeval current_tod, required_tod;
  double current_time, required_time;

  /* Counter increments every time we do a step where the timing doesn't
     overrun. When we make timing_OK_threshold, it's OK to decrease 
     delta_t by a small amount delta_t_change (in defines.h) */
  int timing_OK_counter;

  // Flag which is set when the robot is operating remotely over the 
  // radio ethernet: don't want to show images all the time because it's 
  // too slow
  int reduced_image_flag;

  // To point to left and right images
  Hor_Image *image[2];

  // For initialising features
  Hor_Image *left_patch, *right_patch;  
  Hor_Matrix *hL;

  // For lock_on
  Hor_Matrix *PuInvLock;

  // For measure_feature
  Hor_Matrix *PuInvL, *PuInvR;

  // For calculations
  Hor_Matrix *Temp31, *MC0G, *MC01, *MC1G, *unit;
};


