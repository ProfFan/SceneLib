/*  SceneApp: applications for sequential localisation and map-building

    ETLDouble/control.cc
    Copyright (C) 2000 Andrew Davison and Nobuyuki Kita
    ajd@robots.ox.ac.uk
    nkita@etl.go.jp
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

#include "models_double.h"
#include "models_interrobot.h"

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

#include <OB/CORBA.h>
#include <OB/Util.h>
#include <OB/CosNaming.h>
#include <OB/CosEventChannelAdmin.h>
#include <OB/CosEventComm_skel.h>
#include <OB/Reactor.h>
#include "VisionSimulator.h"
#include "modsim.h"
#include "common.h"

#include "formrobmode.h"
#include "formrob.h"
#include "robot.h"
#include "double_robot.h"
#else
#include "formsim.h"
#endif


#include "control_general.h"
#include "control.h"



/**********************Main class pointers defined here***********************/

// 2 Robots: 2 single motion models; one double motion model encompassing both
// Here the types of the two models is the same but this need not be the case
// Double_Motion_Model assumes there are 2 distinct single models so use two
// here even if they are the same
Turn_Move_TwoD_Motion_Model *turn_move_twod_motion_model1; 
Turn_Move_TwoD_Motion_Model *turn_move_twod_motion_model2; 

Double_Motion_Model *double_motion_model;
Motion_Model *motion_model;

Head_Point_Feature_Measurement_Model *head_point_feature_measurement_model;

int number_of_feature_measurement_models = 1;
Feature_Measurement_Model **feature_measurement_model_array;

Double_Marker_Internal_Measurement_Model 
                              *double_marker_internal_measurement_model;
Internal_Measurement_Model *internal_measurement_model;

// For dual robot/simulation modes
// sim_or_rob is set to either robot or simulation
// sim_or_rob used for common things; simulation or robot for specific
Sim_Or_Rob *sim_or_rob;
#ifdef _ROBOT_
Double_Robot *double_robot;
Robot *robot;
#else
Simulation *simulation;
#endif

Three_D_Display *three_d_disp;
Scene *scene;
Kalman *kalman;

#ifdef _INSPECT_
Scene_Inspect *inspector;
#endif

char *initial_state_file = "initial_state";
#ifndef _ROBOT_
char *true_data_file = "scene_features";
#endif
char *known_features_file = "known_features";
char *waypoints_file = "waypoints";

char *light_table_file = "light_table";

/*****************************Display Parameters******************************/

// Global display pointer
Display *display;

/* Horatio colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;

/*****************************Control Parameters******************************/

// Initial values for these are read from xform in set_up_control()

// Elements of u: (v_control1, S_control1, T_control1,
//                 v_control1, S_control1, T_control1) 
Hor_Matrix *u;

int no_of_steps;
double delta_t;

// Flags set if steering and turret sliders are locked together
int steering_turret_lock_flag, steering_turret_lock_flag2;

/****************************Navigation Waypoints*****************************/

Waypoints *waypoints;

// Lighting waypoints
Hor_Matrix *light_table[1000];

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

  double_robot = new Double_Robot(fd_formrob, zero_axes_flag, 
				  reduced_image_flag,
				  head_point_feature_measurement_model,
				  head_point_feature_measurement_model->Ii,
				  head_point_feature_measurement_model->Hh,
				  argc, argv);
  robot = double_robot;

  return 0;
}
#else

// Version of initialise_simulation unique to DOUBLE case
// This version of initialise_simulation differentiated by arguments
// from one in control_general.cc
int initialise_simulation()
{
  ifstream infile(true_data_file, ios::in);
  // assert(infile != (void *) NULL);

  char type_buf[100];

  // Read in initial vehicle states
  infile >> type_buf;
  assert(strcmp(type_buf, motion_model->motion_model_type) == 0);

  Hor_Matrix *initial_xv = hor_mat_alloc(motion_model->STATE_SIZE, 1);
  Hor_Matrix *initial_xv1 = 
             hor_mat_alloc(double_motion_model->motion_model1->STATE_SIZE, 1);
  Hor_Matrix *initial_xv2 = 
             hor_mat_alloc(double_motion_model->motion_model2->STATE_SIZE, 1);

  // Vehicle 1
  infile >> type_buf;
  assert(strcmp(type_buf, 
                double_motion_model->motion_model1->motion_model_type) == 0);  
  for (int r = 0; r < double_motion_model->motion_model1->STATE_SIZE; r++)
    infile >> matel(initial_xv1, r + 1, 1);

  // Vehicle 2
  infile >> type_buf;
  assert(strcmp(type_buf, 
                double_motion_model->motion_model2->motion_model_type) == 0);  
  for (int r = 0; r < double_motion_model->motion_model2->STATE_SIZE; r++)
    infile >> matel(initial_xv2, r + 1, 1);

  // Combine into total state vector
  hor_matq_insert_chunky1(initial_xv1, initial_xv, 0);
  hor_matq_insert_chunky1(initial_xv2, initial_xv, 
			  double_motion_model->motion_model1->STATE_SIZE);

  simulation = new Simulation(motion_model, initial_xv);
  hor_mat_free_list(initial_xv, initial_xv1, initial_xv2, NULL);

  // Read scene data points in from file
  while (infile)
  {
    infile >> type_buf;
    if (!infile) break;

    Feature_Measurement_Model *f_m_m = 
                find_feature_measurement_model(type_buf,
			                 number_of_feature_measurement_models, 
                                         feature_measurement_model_array);

    Hor_Matrix *initial_yi = hor_mat_alloc(f_m_m->FEATURE_STATE_SIZE, 1);
    for (int i = 0; i < f_m_m->FEATURE_STATE_SIZE; i++)
      infile >> matel(initial_yi, i + 1, 1);

    simulation->add_new_true_feature(f_m_m, initial_yi);

    hor_mat_free(initial_yi);
  }

  return 0;
}
#endif

// Version of initialise_scene unique to DOUBLE case
// This version of initialise_scene differentiated by arguments
// from one in control_general.cc
int initialise_scene()
{
  Hor_Matrix *initial_xv = hor_mats_zero(motion_model->STATE_SIZE, 1);
  Hor_Matrix *initial_Pxx = hor_mats_zero(motion_model->STATE_SIZE,
					  motion_model->STATE_SIZE); 
  Hor_Matrix *initial_xv1 = 
             hor_mat_alloc(double_motion_model->motion_model1->STATE_SIZE, 1);
  Hor_Matrix *initial_xv2 = 
             hor_mat_alloc(double_motion_model->motion_model2->STATE_SIZE, 1);

  // Read initial state from file
  ifstream infile(initial_state_file, ios::in);
  // if (infile != (void *) NULL)
  {
    cout << "Reading initial robot state from file " << initial_state_file
	 << endl;

    char type_buf[100];

    // Double
    infile >> type_buf;
    assert(strcmp(type_buf, motion_model->motion_model_type) == 0);

    // Vehicle 1
    infile >> type_buf;
    assert(strcmp(type_buf, 
		  double_motion_model->motion_model1->motion_model_type) == 0);
    for (int r = 0; r < double_motion_model->motion_model1->STATE_SIZE; r++)
      infile >> matel(initial_xv1, r + 1, 1);

    // Vehicle 2
    infile >> type_buf;
    assert(strcmp(type_buf, 
	          double_motion_model->motion_model2->motion_model_type) == 0);
    for (int r = 0; r < double_motion_model->motion_model2->STATE_SIZE; r++)
      infile >> matel(initial_xv2, r + 1, 1);

    // Combine into total state vector
    hor_matq_insert_chunky1(initial_xv1, initial_xv, 0);
    hor_matq_insert_chunky1(initial_xv2, initial_xv, 
			    double_motion_model->motion_model1->STATE_SIZE);

    for (int r = 0; r < motion_model->STATE_SIZE; r++)
      for (int c = 0; c < motion_model->STATE_SIZE; c++)
	infile >> matel(initial_Pxx, r + 1, c + 1);
  }
  // else
  //   cout << "WARNING: no file " << initial_state_file 
	//  << "; initialising state and covariance to zero." << endl;

  scene = new Scene_Single(initial_xv, initial_Pxx, motion_model, 
		    double_marker_internal_measurement_model);
  hor_mat_free_list(initial_xv, initial_Pxx, NULL);

  return 0;
}

XtAppContext app_con;

int initialise_light_table()
{
  ifstream infile(light_table_file, ios::in);
  // if (infile == NULL)
  // {
  //   cout << "No file " << light_table_file 
	//  << " --- aborting navigation." << endl;
  //   return -1;
  // }

  // Format light_table file: 
  // MOTION_MODEL_DIMENSIONALITY_TYPE
  // xp1...
  // xp2...

  char type_buf[100];
  infile >> type_buf;
  if(strcmp(type_buf, double_motion_model->motion_model_type) != 0)
  {
    cerr << "Mismatched motion model type: aborting." << endl;
    return -1;
  }
  infile >> type_buf;
  if(strcmp(type_buf, double_motion_model->motion_model1->
	    motion_model_dimensionality_type) != 0)
  {
    cerr << "Mismatched motion model 1 dimension: aborting." << endl;
    return -1;
  }
  infile >> type_buf;
  if(strcmp(type_buf, double_motion_model->motion_model2->
	    motion_model_dimensionality_type) != 0)
  {
    cerr << "Mismatched motion model 2 dimension: aborting." << endl;
    return -1;
  }
  
  int no_of_light_table = 0;

  while (infile)
  {
    // Single array hold vectors containing both waypoints stacked
    light_table[no_of_light_table]  
        = hor_mat_alloc(4, 1);
    for (int i = 0; i < 4; i++)
      infile >> matel(light_table[no_of_light_table], i + 1, 1);
    if (!infile) break;

    no_of_light_table++;
  }

  cout << "Read " << no_of_light_table << " light tabel." << endl;

  assert(no_of_light_table == waypoints->get_no_of_waypoints());

  return 0;
}

int set_up_control(int argc, char **argv)
{
  display = fl_get_display();

  turn_move_twod_motion_model1 = new Turn_Move_TwoD_Motion_Model;
  turn_move_twod_motion_model2 = new Turn_Move_TwoD_Motion_Model;

  double_motion_model = new Double_Motion_Model(turn_move_twod_motion_model1, 
					 turn_move_twod_motion_model2);
  motion_model = double_motion_model;

  Escher_Head_Point_Feature_Measurement_Model 
          *escher_head_point_feature_measurement_model = 
               new Escher_Head_Point_Feature_Measurement_Model (motion_model);

  head_point_feature_measurement_model = 
                          escher_head_point_feature_measurement_model;

  // Make array of different available feature measurement models
  number_of_feature_measurement_models = 1;
  feature_measurement_model_array = new Feature_Measurement_Model*[number_of_feature_measurement_models];
  feature_measurement_model_array[0] = head_point_feature_measurement_model;

  double_marker_internal_measurement_model = 
                  new Double_Marker_Internal_Measurement_Model(
                         motion_model, head_point_feature_measurement_model); 
  internal_measurement_model = 
     (Internal_Measurement_Model *) double_marker_internal_measurement_model;

  initialise_scene();

#ifdef _INSPECT_
  fd_forminspect = create_form_forminspect ();
  inspector = new Scene_Inspect (fd_forminspect, scene);
#endif

  kalman = new Kalman;

#ifdef _INSPECT_
  // Create kalman parameter form
  fd_formkalman = create_form_formkalman ();
#endif

  // Read values of control parameters, delta_t and no_of_steps from form
  u = hor_mat_alloc(motion_model->CONTROL_SIZE, 1);

  set_all_vehicle_parameters_from_sliders();

  steering_turret_lock_flag = 
                    fl_get_button(fd_formcom->button_lock_steering_turret);
  steering_turret_lock_flag2 = 
                    fl_get_button(fd_formcom->button_lock_steering_turret2);

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
  initialise_simulation();
  sim_or_rob = simulation;
#endif

  initialise_known_features(known_features_file,
			    number_of_feature_measurement_models, 
			    feature_measurement_model_array, 
			    sim_or_rob, scene);

  // Initialise waypoints class
  waypoints = new Waypoints(waypoints_file, motion_model);

  initialise_waypoint_browser(waypoints);

#ifdef _ROBOT_
  initialise_light_table();
#endif

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

// Vehicle 1
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

void lock_steering_turret2(FL_OBJECT *ob, long data)
{
  steering_turret_lock_flag2 = fl_get_button(ob);
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

// Vehicle 2
void set_v2(FL_OBJECT *ob, long data)
{
  matel(u, 4, 1) = fl_get_slider_value(ob);
}

void zero_v2(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_v2, 0.0);
  matel(u, 4, 1) = 0.0;
}

void set_S2(FL_OBJECT *ob, long data)
{
  matel(u, 5, 1) = fl_get_slider_value(ob) * M_PI / 180.0;
  if (steering_turret_lock_flag2)
  {
    matel(u, 6, 1) = matel(u, 5, 1);
    fl_set_slider_value(fd_formcom->slider_set_T2, 
			matel(u, 5, 1) * 180.0 / M_PI);
  }
}

void zero_S2(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_S2, 0.0);
  matel(u, 5, 1) = 0.0;
  if (steering_turret_lock_flag2)
  {
    matel(u, 6, 1) = matel(u, 5, 1);
    fl_set_slider_value(fd_formcom->slider_set_T2, 
			matel(u, 5, 1) * 180.0 / M_PI);
  }
}

void set_T2(FL_OBJECT *ob, long data)
{
  matel(u, 6, 1) = fl_get_slider_value(ob) * M_PI / 180.0;
  if (steering_turret_lock_flag2)
  {
    matel(u, 5, 1) = matel(u, 6, 1);
    fl_set_slider_value(fd_formcom->slider_set_S2, 
			matel(u, 6, 1) * 180.0 / M_PI);
  }
}

void zero_T2(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formcom->slider_set_T2, 0.0);
  matel(u, 6, 1) = 0.0;
  if (steering_turret_lock_flag2)
  {
    matel(u, 5, 1) = matel(u, 6, 1);
    fl_set_slider_value(fd_formcom->slider_set_S2, 
			matel(u, 6, 1) * 180.0 / M_PI);
  }
}

// General
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
  matel(u, 4, 1) = fl_get_slider_value(fd_formcom->slider_set_v2);
  matel(u, 5, 1) = fl_get_slider_value(fd_formcom->slider_set_S2) 
                                 * M_PI / 180.0;
  matel(u, 6, 1) = fl_get_slider_value(fd_formcom->slider_set_T2) 
                                 * M_PI / 180.0;
  delta_t = fl_get_slider_value(fd_formcom->slider_set_delta_t);
  no_of_steps = (int) fl_get_slider_value(fd_formcom->slider_set_steps);

  return 0;
}


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
  xf = matel(vv, 7, 1);
  zf = matel(vv, 6, 1);
  rot = matel(vv, 9, 1);
  robot = LIGHT;
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

    if (scene->get_successful_measurement_vector_size() != 0)
      kalman->total_update_filter(scene);
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
  xf = matel(vv, 7, 1);
  zf = matel(vv, 6, 1);
  rot = matel(vv, 9, 1);
  robot = LIGHT;
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

int observe_second_robot()
{
  cout << "Making measurement of second robot." << endl;
  scene->predict_internal_measurement();

  if (double_marker_internal_measurement_model->feasibility_test(
                       scene->get_xv(), scene->get_hv()) != 0)
  {
    cerr << "Second robot not visible: can't make measurement." << endl;
    return -1;
  }

  if (sim_or_rob->make_internal_measurement(
                                    double_marker_internal_measurement_model, 
					    scene->get_zv(), scene->get_hv(),
					    scene->get_Sv()) == 0)
    scene->successful_internal_measurement();
  else
    scene->failed_internal_measurement();

  if (scene->get_successful_internal_measurement_flag())
    kalman->update_filter_internal_measurement(scene, 
                               double_marker_internal_measurement_model);

  display_estimated_features(scene, three_d_disp);
  display_estimated_vehicle(scene, three_d_disp);
  three_d_disp->draw_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
    display_true_features((Simulation *) sim_or_rob, three_d_disp);
  }

  return 0;
}

// Make inter-robot observation
void observe_second_robot(FL_OBJECT *ob, long data)
{
  observe_second_robot();
}

void navigate_to_next_waypoint(FL_OBJECT *ob, long data)
{
#ifdef _ROBOT_
  char speech_text[100];
  char number[10];
  strcpy(speech_text, "Arrived at waypoint ");
  sprintf(number, "%d", waypoints->get_current_waypoint());
  strcat(speech_text, number);
#endif

  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle(1);
  }

#ifdef _ROBOT_
  double_robot->set_light_control(
                       light_table[waypoints->get_current_waypoint()]);
  usleep(1000000);

  double_robot->say(speech_text);
#endif

  waypoints->increment_current_waypoint();
  highlight_waypoint_browser_line();

#ifdef _INSPECT_
  inspector->update_form ();
#endif
}

void waypoint_check_correct(FL_OBJECT *ob, long data)
{
#ifdef _ROBOT_
  char speech_text[100];
  char number[10];
  strcpy(speech_text, "Arrived at waypoint ");
  sprintf(number, "%d", waypoints->get_current_waypoint());
  strcat(speech_text, number);
#endif

  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle(1);
  }

    cout << "Measuring second robot." << endl;
#ifdef _ROBOT_
    double_robot->say("Measuring second robot.");
#endif

  usleep(2000000);
  
  observe_second_robot();

  usleep(2000000);

  cout << "Correcting position." << endl;
#ifdef _ROBOT_
  double_robot->say("Correcting position.");
#endif

  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle(1);
  }

#ifdef _ROBOT_
  double_robot->set_light_control(
                         light_table[waypoints->get_current_waypoint()]);
  usleep(1000000);

  double_robot->say(speech_text);
#endif

  waypoints->increment_current_waypoint();
  highlight_waypoint_browser_line();

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

/****************Redefine Axes at Current Waypoint Position*******************/

void zero_axes(FL_OBJECT *ob, long data)
{
  zero_axes(scene, sim_or_rob, waypoints, three_d_disp);

  display_waypoints();

#ifdef _INSPECT_
  inspector->update_form();
#endif
}

#ifdef _ROBOT_
/*********************Control Functions for the Robot*************************/

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
