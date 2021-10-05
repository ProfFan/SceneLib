/*  SceneApp: applications for sequential localisation and map-building

    control.cc
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

// Main file where everything is controlled from

#include <general_headers.h>

#include "models_base.h"
#include "models_nomad.h"
#include "models_head.h"
#include "models_head_escher.h"
#ifdef _THREED_
#include "models_rollpitch.h"
#endif

#include "3d.h"
#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"

#include "kalman.h"
#include "sim_or_rob.h"
#include "simul.h"
#include "waypoints.h"
#include "forms.h"
#include "formcom.h"
#include "waypoint_browser.h"

#include "postscript.h"

#ifdef _INSPECT_
// At the moment, kalman_form and scene_inspector are tangled and must be used
// together
#include "formkalman.h"
#include "forminspect.h"
#include "scene_inspect.h"
#include "kalman_form.h"
#endif


#ifdef _ROBOT_
// View simulator Related
#if 0
#include <OB/CORBA.h>
#include <OB/Util.h>
#include <OB/CosNaming.h>
#include <OB/CosEventChannelAdmin.h>
#include <OB/CosEventComm_skel.h>
#include <OB/Reactor.h>
#include "VisionSimulator.h"
#include "FishEye.h"
#include "modsim.h"
#include "common.h"
#endif

#include "formrobmode.h"
#include "formrob.h"
#include "robot.h"

#else
#include "formsim.h"
#endif


#include "control_general.h"
#include "control.h"



/**********************Main class pointers defined here***********************/

// Define one generic type motion model, but potentially several 
// specific feature models
Turn_Move_TwoD_Motion_Model *turn_move_twod_motion_model;

#ifdef _THREED_
// For 3D mode where the robot runs on a non-flat surface
// We use measurements from a roll-pitch sensor to help
Turn_Move_ThreeD_Motion_Model *turn_move_threed_motion_model;
Roll_Pitch_Sensor_Internal_Measurement_Model 
       *roll_pitch_sensor_internal_measurement_model;
Internal_Measurement_Model *internal_measurement_model;
#endif

Motion_Model *motion_model;

#ifdef _THREED_
ThreeD_Head_Point_Feature_Measurement_Model 
#else
TwoD_Head_Point_Feature_Measurement_Model 
#endif
  *head_point_feature_measurement_model;

int number_of_feature_measurement_models = 1;
Feature_Measurement_Model **feature_measurement_model_array;

// For dual robot/simulation modes
// sim_or_rob is set to either robot or simulation
// sim_or_rob used for common things; simulation or robot for specific
Sim_Or_Rob *sim_or_rob;
#ifdef _ROBOT_
Robot *robot;
#else
Simulation *simulation;
#endif

Three_D_Display *three_d_disp;
Scene_Single *scene;
Kalman *kalman;

Postscript *postscript;

#ifdef _INSPECT_
Scene_Inspect *inspector;
#endif

#ifdef _THREED_
char *initial_state_file = "initial_state3D";
#ifndef _ROBOT_
char *true_data_file = "scene_features3D";
#endif
char *waypoints_file = "waypoints3D";
char *known_features_file = "known_features3D";

#else

char *initial_state_file = "initial_state2D";
#ifndef _ROBOT_
char *true_data_file = "scene_features2D";
#endif
char *waypoints_file = "waypoints2D";
char *known_features_file = "known_features2D";

#endif

/*****************************Display Parameters******************************/

// Global display pointer
Display *display;

/* Horatio colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;

/*****************************Control Parameters******************************/

// Initial values for these are read from xform in set_up_control()

// Elements of u: (v_control, S_control, T_control)
Hor_Matrix *u;

int no_of_steps;
double delta_t;

// Flag set if steering and turret sliders are locked together
int steering_turret_lock_flag;

/****************************Navigation Waypoints*****************************/

Waypoints *waypoints;

/************************Initialise stuff in this file************************/

static FD_formcom *fd_formcom;
#ifdef _ROBOT_
static FD_formrob *fd_formrob;
#else
static FD_formsim *fd_formsim;
#endif

#ifdef _INSPECT_
static FD_formkalman *fd_formkalman;
static FD_forminspect *fd_forminspect;
#endif

void pass_fd_pointer_com(FD_formcom *fd_fc)
{
  fd_formcom = fd_fc;
}
#ifdef _ROBOT_
void pass_fd_pointer_rob(FD_formrob *fd_fr)
{
  fd_formrob = fd_fr;
}
#else
void pass_fd_pointer_sim(FD_formsim *fd_fs)
{
  fd_formsim = fd_fs;
}
#endif

#ifdef _ROBOT_
int initialise_robot(int argc, char **argv)
{
  FD_formrobmode *fd_formrobmode;

  fd_formrobmode = create_form_formrobmode();

  fl_show_form(fd_formrobmode->formrobmode,FL_PLACE_CENTERFREE,FL_FULLBORDER,
	       "formrobmode");
  while (fl_do_forms() != fd_formrobmode->button_done)
    ;

  int zero_axes_flag = fl_get_button(fd_formrobmode->button_zero_axes);
  int reduced_image_flag = fl_get_button(fd_formrobmode->button_reduced_image);

  fl_hide_form(fd_formrobmode->formrobmode);

  robot = new Robot(fd_formrob, zero_axes_flag, reduced_image_flag,
		    head_point_feature_measurement_model, 
		    head_point_feature_measurement_model->Ii, 
		    head_point_feature_measurement_model->Hh, 
		    argc, argv);

  return 0;
}
#endif

XtAppContext app_con;

int set_up_control(int argc, char **argv)
{
  display = fl_get_display();

  turn_move_twod_motion_model = new Turn_Move_TwoD_Motion_Model;
#ifdef _THREED_
  turn_move_threed_motion_model = 
         new Turn_Move_ThreeD_Motion_Model(turn_move_twod_motion_model);
  motion_model = turn_move_threed_motion_model;
  roll_pitch_sensor_internal_measurement_model = 
    new Roll_Pitch_Sensor_Internal_Measurement_Model(motion_model);
  internal_measurement_model =
   (Internal_Measurement_Model *) roll_pitch_sensor_internal_measurement_model;
#else
  motion_model = turn_move_twod_motion_model;
#endif

  Escher_Head_Point_Feature_Measurement_Model 
          *escher_head_point_feature_measurement_model = 
               new Escher_Head_Point_Feature_Measurement_Model (motion_model);
  
  head_point_feature_measurement_model = 
                          escher_head_point_feature_measurement_model;


  // Make array of different available feature measurement models
  number_of_feature_measurement_models = 1;
  feature_measurement_model_array = new Feature_Measurement_Model*[number_of_feature_measurement_models];
  feature_measurement_model_array[0] = head_point_feature_measurement_model;

  Hor_Matrix *initial_xv = hor_mat_alloc(motion_model->STATE_SIZE, 1);
  Hor_Matrix *initial_Pxx = hor_mat_alloc(motion_model->STATE_SIZE,
					  motion_model->STATE_SIZE);   

  read_in_initial_state(initial_xv, initial_Pxx, initial_state_file, 
			motion_model);

#ifdef _THREED_
  scene = new Scene_Single(initial_xv, initial_Pxx, motion_model,
			   internal_measurement_model);
#else
  scene = new Scene_Single(initial_xv, initial_Pxx, motion_model);
#endif

  hor_mat_free_list(initial_xv, initial_Pxx, NULL);

#ifdef _INSPECT_
  fd_forminspect = create_form_forminspect ();
  inspector = new Scene_Inspect (fd_forminspect, scene);
#endif

  kalman = new Kalman;

#ifdef _INSPECT_
  // Create kalman parameter form
  fd_formkalman = create_form_formkalman ();
#endif

  postscript = new Postscript;

  // Read values of control parameters, delta_t and no_of_steps from form
  u = hor_mat_alloc(motion_model->CONTROL_SIZE, 1);

  set_all_vehicle_parameters_from_sliders();

  steering_turret_lock_flag = 
                    fl_get_button(fd_formcom->button_lock_steering_turret);

  // Initialise horatio display for 3D tool
  hor_colourmap_setup ( display, 2, 6,
		      "Red",       &Red,       "Green",         &Green,
		      "Blue",      &Blue,      "Yellow",        &Yellow,
		      "SteelBlue", &SteelBlue, "LightSeaGreen", &LightSeaGreen,
		      "thistle",   &thistle,   "Cyan",          &Cyan,
			NULL );

  String fallback_resources[] = {
    "*input: True",
    "*text*editType:          append",
    "*text*scrollVertical:    Always",
    "*text*height:            20",
    NULL,
  };
  Widget dummy;
  //  char *argv = "nnn";
  //  int argc = 0;
  dummy = XtAppInitialize(&app_con, "horatio", NULL, ZERO,
			  &argc, argv, fallback_resources, NULL, ZERO);
  GC gc = fl_get_gc();
  hor_display_initialise ( display, gc,
			   SteelBlue, Green, Red, LightSeaGreen, thistle );

  three_d_disp = new Three_D_Display(dummy, display);

#ifdef _ROBOT_
  initialise_robot(argc, argv);
  sim_or_rob = robot;
#else
  initialise_simulation(simulation, true_data_file, motion_model, 
			number_of_feature_measurement_models, 
			feature_measurement_model_array);
  sim_or_rob = simulation;
#endif

  initialise_known_features(known_features_file,
			    number_of_feature_measurement_models, 
			    feature_measurement_model_array, 
			    sim_or_rob, scene);

  // Initialise waypoints class
  waypoints = new Waypoints(waypoints_file, motion_model);

  initialise_waypoint_browser(waypoints);


#ifndef _ROBOT_
  //  simulation->print_true_features();
  display_true_features(simulation, three_d_disp);
  display_true_vehicle(simulation, three_d_disp);  
#endif

  display_estimated_vehicle(scene, three_d_disp);
  display_estimated_features(scene, three_d_disp);

#ifdef _INSPECT_
  // Now everything's declared, pass pointers to kalman form
  pass_all_ptrs (kalman, scene, three_d_disp, sim_or_rob,
		 inspector, fd_formkalman);

  // Display initial state on inspect form
  inspector->update_form ();
#endif

  return 0;
}

/********* Callbacks for when buttons to open other forms are clicked ********/

#ifdef _INSPECT_
void show_inspect_form (FL_OBJECT *obj, long data)
{
  if (!fd_forminspect->forminspect->visible) {
    fl_show_form (fd_forminspect->forminspect, FL_PLACE_FREE,
		  FL_FULLBORDER, "Scene Inspector Form");
  }
  else fl_hide_form (fd_forminspect->forminspect);
}


void show_filter_form (FL_OBJECT *obj, long data)
{
  if (!fd_formkalman->formkalman->visible) {
    fl_show_form (fd_formkalman->formkalman, FL_PLACE_FREE,
		  FL_FULLBORDER, "Kalman Filter Parameters");
  }
  else fl_hide_form (fd_formkalman->formkalman);
}
#else
void show_inspect_form (FL_OBJECT *obj, long data)
{
  cerr << "Sorry: scene inspector not linked with." << endl;
}

void show_filter_form (FL_OBJECT *obj, long data)
{
  cerr << "Sorry: kalman form not linked with." << endl;
}
#endif

/**********Called from 3d.cc when feature is selected with the mouse**********/

int please_redisplay_features_flag = 0;

#ifndef _ROBOT_
int true_feature_selected(int i)
{
  // Get the identifier for this feature (always a pointer)
  void *id = (void *) simulation->find_identifier(i);

  // See if it is a feature we already know about
  if (scene->find_feature(id))
  {
    if (!scene->toggle_feature(id))
      cerr << "Trouble toggling feature with identifier" << id << endl;

    scene->mark_feature_by_lab(scene->find_feature(id)->get_label());
  }
  else // New feature
  {
    Feature_Measurement_Model *f_m_m = 
                             simulation->find_feature_measurement_model(i);

    initialise_feature(scene, sim_or_rob, three_d_disp, f_m_m->feature_type,
		       id, number_of_feature_measurement_models, 
		       feature_measurement_model_array);
  }

  scene->print_selected_features();

  // This is a bit of a hack because we can't just call 
  // display_estimated_features from here (gives trouble in 3D tool code)
  // Instead called from main_control_loop()
  please_redisplay_features_flag = 1;

#ifdef _INSPECT_
  inspector->update_form ();
#endif

  return 0;
}
#else
int true_feature_selected(int i)
{
  cerr << "Bug: true feature selected in robot mode." << endl;
  exit(0);
}
#endif

int estimated_feature_selected(int i)
{
  if (!scene->toggle_feature_lab(i))
    cerr << "Trouble toggling feature " << i << endl;

  scene->print_selected_features();

  // This is a bit of a hack because we can't just call 
  // display_estimated_features from here (gives trouble in 3D tool code)
  // Instead called from main_control_loop()
  please_redisplay_features_flag = 1;

  scene->mark_feature_by_lab(i);

#ifdef _INSPECT_
  inspector->update_form ();
#endif

  return 0;
}

/**********************Set vehicle control parameters*************************/

void set_v(FL_OBJECT *ob, long data)
{
  matel(u, 1, 1) = fl_get_slider_value(ob);
}

void zero_v(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_v, 0.0);
  matel(u, 1, 1) = 0.0;
}

void lock_steering_turret(FL_OBJECT *ob, long data)
{
  steering_turret_lock_flag = fl_get_button(ob);
}


void set_S(FL_OBJECT *ob, long data)
{
  matel(u, 2, 1) = fl_get_slider_value(ob) * M_PI / 180.0;
  if (steering_turret_lock_flag)
  {
    matel(u, 3, 1) = matel(u, 2, 1);
    fl_set_slider_value(fd_formcom->slider_set_T, 
			matel(u, 2, 1) * 180.0 / M_PI);
  }
}

void zero_S(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_S, 0.0);
  matel(u, 2, 1) = 0.0;
  if (steering_turret_lock_flag)
  {
    matel(u, 3, 1) = matel(u, 2, 1);
    fl_set_slider_value(fd_formcom->slider_set_T, 
			matel(u, 2, 1) * 180.0 / M_PI);
  }
}

void set_T(FL_OBJECT *ob, long data)
{
  matel(u, 3, 1) = fl_get_slider_value(ob) * M_PI / 180.0;
  if (steering_turret_lock_flag)
  {
    matel(u, 2, 1) = matel(u, 3, 1);
    fl_set_slider_value(fd_formcom->slider_set_S, 
			matel(u, 3, 1) * 180.0 / M_PI);
  }
}

void zero_T(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_T, 0.0);
  matel(u, 3, 1) = 0.0;
  if (steering_turret_lock_flag)
  {
    matel(u, 2, 1) = matel(u, 3, 1);
    fl_set_slider_value(fd_formcom->slider_set_S, 
			matel(u, 3, 1) * 180.0 / M_PI);
  }
}

void set_delta_t(FL_OBJECT *ob, long data)
{
  delta_t = fl_get_slider_value(ob);
}

void one_second(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_delta_t, 1.0);
  delta_t = 1.0;
}

void set_steps(FL_OBJECT *ob, long data)
{
  no_of_steps = (int) fl_get_slider_value(ob);
}

void one_steps(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_steps, 1.0);
  no_of_steps = 1;
}

int set_all_vehicle_sliders_from_parameters()
{
  fl_set_slider_value(fd_formcom->slider_set_v, matel(u, 1, 1));
  fl_set_slider_value(fd_formcom->slider_set_S, matel(u, 2, 1));
  fl_set_slider_value(fd_formcom->slider_set_T, matel(u, 3, 3));
  fl_set_slider_value(fd_formcom->slider_set_delta_t, delta_t);
  fl_set_slider_value(fd_formcom->slider_set_steps, no_of_steps);

  return 0;
}

int set_all_vehicle_parameters_from_sliders()
{
  matel(u, 1, 1) = fl_get_slider_value(fd_formcom->slider_set_v);
  matel(u, 2, 1) = fl_get_slider_value(fd_formcom->slider_set_S) 
                                 * M_PI / 180.0;
  matel(u, 3, 1) = fl_get_slider_value(fd_formcom->slider_set_T) 
                                 * M_PI / 180.0;
  delta_t = fl_get_slider_value(fd_formcom->slider_set_delta_t);
  no_of_steps = (int) fl_get_slider_value(fd_formcom->slider_set_steps);

  return 0;
}

#ifdef _THREED_
// How often we are going to make roll/pitch measurements
const double ROLLPITCH_MEASUREMENT_TIME_INTERVAL = 0.5;

// Local version of go_vehicle making frequent measurements from
// the roll/pitch sensor during motion
// Split up the demanded time interval into steps of 
// ROLLPITCH_MEASUREMENT_TIME_INTERVAL and make measurements that often
int go_vehicle_rollpitch(int auto_select_flag)
{
  Hor_Matrix *local_u = hor_mats_copy(u);

  // First if there is a non-zero turn component do that separately
  if (vecel(u, 2) != 0.0 || vecel(u, 3) != 0.0) {
    vecel(local_u, 1) = 0.0;

    sim_or_rob->set_control(local_u, delta_t);
  
    kalman->predict_filter(scene, local_u, delta_t);

    sim_or_rob->wait_for_end_of_motion(local_u);
    sim_or_rob->stop_vehicle();

    // Then zero the turning bits of the control vector so we can just go
    // straight in the rest of the motion
    hor_matq_copy(u, local_u);
    vecel(local_u, 2) = 0.0;
    vecel(local_u, 3) = 0.0;
  }
    

  double remaining_time = delta_t;
  int first_time_flag = 1;

  // Small movement steps where we measure roll/pitch each time
  while (remaining_time > 0) {
    double current_timestep = ROLLPITCH_MEASUREMENT_TIME_INTERVAL;

    if (current_timestep > remaining_time)
      current_timestep = remaining_time;

    if (first_time_flag) {
      sim_or_rob->set_control(local_u, current_timestep);
      first_time_flag = 0;
    }
    else
      sim_or_rob->continue_control(local_u, current_timestep);
      

    kalman->predict_filter(scene, local_u, current_timestep);

    sim_or_rob->wait_for_end_of_motion(local_u);

    remaining_time -= current_timestep;

    if (remaining_time <= 0.0)
      sim_or_rob->stop_vehicle();

    // Make pan/tilt measurement
    scene->predict_internal_measurement();
    if (sim_or_rob->make_internal_measurement(
                                roll_pitch_sensor_internal_measurement_model, 
					    scene->get_zv(), scene->get_hv(),
					    scene->get_Sv()) == 0)
      scene->successful_internal_measurement();
    else
      scene->failed_internal_measurement();

    if (scene->get_successful_internal_measurement_flag()) {
      kalman->update_filter_internal_measurement(scene, 
		    roll_pitch_sensor_internal_measurement_model);
      scene->normalise_state();
    }

    if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
    {
      display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
      display_true_features((Simulation *) sim_or_rob, three_d_disp);
    }

    display_estimated_features(scene, three_d_disp);
    display_estimated_vehicle(scene, three_d_disp);

    // usleep(1000000);
  }



  // Once robot has stopped, make feature measurements
  if (auto_select_flag) {
    scene->auto_select_feature();
  }

  if (scene->get_no_selected() != 0)
  {
    scene->predict_measurements(); 

    make_measurements(scene, sim_or_rob); 

    if (scene->get_successful_measurement_vector_size() != 0) {
      kalman->total_update_filter(scene); 
      scene->normalise_state();
    }
  }

  three_d_disp->draw_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
    display_true_features((Simulation *) sim_or_rob, three_d_disp);
  }

  display_estimated_features(scene, three_d_disp);
  display_estimated_vehicle(scene, three_d_disp);

  hor_mat_free(local_u);

  return 0;
}

#endif

// Normal version of go_vehicle: now local, with calls to view simulator
int go_vehicle(int auto_select_flag)
{
#ifdef _ROBOT_
  // For display in view simulator  
  Hor_Matrix *vv;
  float xf, zf, rot;
  int robot;
#endif

  sim_or_rob->set_control(u, delta_t);
  
  kalman->predict_filter(scene, u, delta_t);

#ifdef _ROBOT_
  // Display current position in view simulator
  vv = scene->get_xv();
  xf = matel(vv, 2, 1);
  zf = matel(vv, 1, 1);
  rot = matel(vv, 4, 1);
  robot = VISION;
  vsim_trans_robot(robot, xf, zf, rot);
#endif

  if (auto_select_flag) {
    scene->auto_select_feature();
  }

  if (scene->get_no_selected() != 0)
  {
    scene->predict_measurements(); 

    sim_or_rob->wait_for_end_of_motion(u);
    sim_or_rob->stop_vehicle();

    make_measurements(scene, sim_or_rob);

    if (scene->get_successful_measurement_vector_size() != 0) {
      kalman->total_update_filter(scene);
      scene->normalise_state();
    }

  }
  else
  {
    sim_or_rob->wait_for_end_of_motion(u);
    sim_or_rob->stop_vehicle();
  }

  three_d_disp->draw_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
    display_true_features((Simulation *) sim_or_rob, three_d_disp);
  }

  display_estimated_features(scene, three_d_disp);
  display_estimated_vehicle(scene, three_d_disp);

#ifdef _ROBOT_
  // Display current position in view simulator
  vv = scene->get_xv();
  xf = matel(vv, 2, 1);
  zf = matel(vv, 1, 1);
  rot = matel(vv, 4, 1);
  robot = VISION;
  vsim_trans_robot(robot, xf, zf, rot);
#endif

  return 0;
}


// Called from forms: standard go with no auto select
void go_using_set_parameters(FL_OBJECT *ob, long data)
{
  set_all_vehicle_parameters_from_sliders();

  for (int steps = 0; steps < no_of_steps; steps++)
  {
    go_vehicle(0);
  }

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

void navigate_to_next_waypoint(FL_OBJECT *ob, long data)
{
  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle(1);
  }

  waypoints->increment_current_waypoint();
  highlight_waypoint_browser_line();

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

void navigate_to_next_waypoint_using_rollpitch(FL_OBJECT *ob, long data)
{
#ifdef _THREED_
  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle_rollpitch(1);
  }

  waypoints->increment_current_waypoint();
  highlight_waypoint_browser_line();

#ifdef _INSPECT_
  inspector->update_form ();
#endif

#else
  cout << "Not implemented in 2D navigation mode." << endl;
#endif
}

/****************Delete the last feature selected with the mouse**************/

void delete_feature(FL_OBJECT *ob, long data)
{
  delete_feature(scene, three_d_disp);

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

// Auto-select feature for measurement
void auto_select_feature(FL_OBJECT *ob, long data)
{
  scene->auto_select_feature();

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}


// Print State
void print_robot_state(FL_OBJECT *ob, long data)
{
  scene->print_robot_state();
}

void print_feature_state(FL_OBJECT *ob, long data)
{
  scene->print_marked_feature_state();
}

void print_whole_state(FL_OBJECT *ob, long data)
{
  scene->print_whole_state();
}

// Output State to File
void output_state(FL_OBJECT *ob, long data)
{
  scene->output_state_to_file();
}

#ifndef _ROBOT_
// Print True State
void print_true_state(FL_OBJECT *ob, long data)
{
  simulation->print_true_vehicle();
}
#endif

/**********************Redefine Axes at Current Position**********************/

void zero_axes(FL_OBJECT *ob, long data)
{
  zero_axes(scene, sim_or_rob, waypoints, three_d_disp);

  display_waypoints();

#ifdef _INSPECT_
  inspector->update_form();
#endif
}


/*******************Get Measurement from Roll/Pitch Sensor********************/

#ifdef _THREED_
void measure_roll_pitch(FL_OBJECT *ob, long data)
{
  scene->predict_internal_measurement();
  if (sim_or_rob->make_internal_measurement(
                                roll_pitch_sensor_internal_measurement_model, 
					    scene->get_zv(), scene->get_hv(),
					    scene->get_Sv()) == 0)
    scene->successful_internal_measurement();
  else
    scene->failed_internal_measurement();

  if (scene->get_successful_internal_measurement_flag()) {
    kalman->update_filter_internal_measurement(scene, 
                               roll_pitch_sensor_internal_measurement_model);
    scene->normalise_state();
  }

  display_estimated_features(scene, three_d_disp);
  display_estimated_vehicle(scene, three_d_disp);
  three_d_disp->draw_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
    display_true_features((Simulation *) sim_or_rob, three_d_disp);
  }

  cout << "Roll-pitch measurement: predicted " << scene->get_hv() 
       << "Measured " << scene->get_zv() << endl;

}

void go_vehicle_roll_pitch(FL_OBJECT *ob, long data)
{
  go_vehicle_rollpitch(0);
}

#else
void measure_roll_pitch(FL_OBJECT *ob, long data)
{
  cout << "Roll-pitch sensor not active in 2D mode." << endl;
}

void go_vehicle_roll_pitch(FL_OBJECT *ob, long data)
{
  cout << "Roll-pitch sensor not active in 2D mode." << endl;
}
#endif

void output_postscript(FL_OBJECT *ob, long data)
{
#ifdef _ROBOT_
  postscript->output_postscript(scene);
#else
  postscript->output_postscript(scene, simulation);
#endif

  cout << "Output postscript to file " << ps_filename << "." << endl;
}
          
#ifdef _ROBOT_
/**********************Control Functions for the Robot************************/

void set_pan(FL_OBJECT *ob, long data)
{
  robot->set_pan(fl_get_slider_value(ob) * M_PI / 180.0);
}

void zero_pan(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formrob->slider_set_pan, 0.0);
  robot->set_pan(0.0);
}

void set_elevation(FL_OBJECT *ob, long data)
{
  robot->set_elevation(fl_get_slider_value(ob) * M_PI / 180.0);
}

void zero_elevation(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formrob->slider_set_elevation, 0.0);
  robot->set_elevation(0.0);
}

void show_head_odometry(FL_OBJECT *ob, long data)
{
  robot->read_head_odometry();
  robot->print_head_angles();
}

void show_robot_odometry(FL_OBJECT *ob, long data)
{
  robot->print_robot_odometry();
}

void grab_images(FL_OBJECT *ob, long data)
{
  robot->grab_and_display_images();
}

void read_images(FL_OBJECT *ob, long data)
{
  robot->read_images();
}

void write_images(FL_OBJECT *ob, long data)
{
  robot->write_images();
}

void write_patch(FL_OBJECT *ob, long data)
{
  robot->write_patch();
}

/*****************************Initialise a Feature****************************/

/* Called by either init_man_selected_feature or init_auto_selected_feature */

int initialise_point_feature()
{
  cout << "Making measurement to initialise new feature." << endl;

  initialise_feature(scene, sim_or_rob, three_d_disp, "HEAD_POINT", NULL,
		     number_of_feature_measurement_models, 
		     feature_measurement_model_array);

#ifdef _INSPECT_
  inspector->update_form ();
#endif

  return 0;
}

/* Initialise the feature which has been selected in the left image */

void init_man_selected_point_feature(FL_OBJECT *ob, long data)
{
  initialise_point_feature();

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

// Try to find a feature in the current field of view
int init_auto_point_feature()
{
  int ubest[PATCHES_TO_AUTO_FIND], vbest[PATCHES_TO_AUTO_FIND];
  double evbest[PATCHES_TO_AUTO_FIND];

  robot->rob_find_best_n_patches(PATCHES_TO_AUTO_FIND, 
				      ubest, vbest, evbest);

  // Need to make sure we come back to the same head position each time
  robot->note_current_head_position();

  for (int i = 0; i < PATCHES_TO_AUTO_FIND; i++)
  {
    robot->set_left_image_selection(ubest[i], vbest[i]);

    if (initialise_point_feature() == 0)
    {
      cout << "Successful with best feature " << i << " from image." << endl;
      return 1;
    }

    robot->return_to_noted_head_position();
  }
  cout << "No good feature found in this position." << endl;
  return -1;
}

void init_auto_selected_point_feature(FL_OBJECT *ob, long data)
{
  init_auto_point_feature();

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

/* Automatically initialise NEW_FEATURES (number of) new features */
int initialise_auto_new_point_features()
{ 
  int count = 0;

  for (int i = 0; i < NEW_FEATURES; i++)
  {
    // Move head to clever position
    robot->move_head_to_new_point_feature_search_position(i);
    if (init_auto_point_feature() > 0)
      count++;
  }

  cout << "Initialised " << count << " new features." << endl;

#ifdef _INSPECT_
  inspector->update_form ();
#endif

  return 0;
}

/*****************************************************************************/

#endif // End of robot-only functions

/******************************Main Control Loop******************************/

int main_control_loop()
{
  // Enter main loop
  FL_OBJECT *obj;
  XEvent event;

  for (;;)
  {
    // Check events from Xforms
    while ((obj=fl_check_forms()) != NULL);

    // Check events from 3D tool
    while (XtAppPending(app_con))
    {
      XtAppNextEvent(app_con, &event);
      XtDispatchEvent(&event);

      // Slightly ugly hack so that can change feature colours immediately
      if (please_redisplay_features_flag)
      {
	display_estimated_features(scene, three_d_disp);
	please_redisplay_features_flag = 0;
      }
    }
  }

  return 0;
}

/************************************Quit*************************************/

void quit(FL_OBJECT *ob, long data)
{
  // Delete sim_or_rob class so hardware is shut down in robot case
  delete sim_or_rob;

  exit(0);
}

/*****************************************************************************/
