/*  SceneApp: applications for sequential localisation and map-building

    Camera/control.cc
    Copyright (C) 2001 Andrew Davison
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

// Special models for this application
#include "models_impulse.h"
#include "models_camera.h"


#include "3d.h"
#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"

#include "kalman.h"
#include "sim_or_rob.h"
#include "simul.h"
#include "forms.h"
#include "formcom.h"

#include "postscript.h"
#include "waypoints.h"

#ifdef _INSPECT_
// At the moment, kalman_form and scene_inspector are tangled and must be used
// together
#include "formkalman.h"
#include "kalman_form.h"
#include "forminspect.h"
#include "scene_inspect.h"
#endif


#ifdef _ROBOT_

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
Impulse_ThreeD_Motion_Model *impulse_threed_motion_model;

Motion_Model *motion_model;

Camera_Point_Feature_Measurement_Model *camera_point_feature_measurement_model;

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


char *initial_state_file = "initial_state";
#ifndef _ROBOT_
char *true_data_file = "scene_features";
#endif

char *known_features_file = "known_features";

/*****************************Display Parameters******************************/

// Global display pointer
Display *display;

/* Horatio colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;

/*****************************Control Parameters******************************/

// In this application u will have no contents
Hor_Matrix *u;

int no_of_steps;
double delta_t = 1.0 / 30.0;

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
  robot = new Robot(fd_formrob);

  return 0;
}
#endif

XtAppContext app_con;

int set_up_control(int argc, char **argv)
{
  display = fl_get_display();

  impulse_threed_motion_model = new Impulse_ThreeD_Motion_Model;
  motion_model = impulse_threed_motion_model;

  camera_point_feature_measurement_model = 
    new Camera_Point_Feature_Measurement_Model (motion_model);

  // Make array of different available feature measurement models
  number_of_feature_measurement_models = 1;
  feature_measurement_model_array = new (Feature_Measurement_Model *)
                              [number_of_feature_measurement_models];
  feature_measurement_model_array[0] = camera_point_feature_measurement_model;

  Hor_Matrix *initial_xv = hor_mat_alloc(motion_model->STATE_SIZE, 1);
  Hor_Matrix *initial_Pxx = hor_mat_alloc(motion_model->STATE_SIZE,
					  motion_model->STATE_SIZE);   

  read_in_initial_state(initial_xv, initial_Pxx, initial_state_file, 
			motion_model);

  scene = new Scene_Single(initial_xv, initial_Pxx, motion_model);

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
  fl_set_slider_value(fd_formcom->slider_set_steps, no_of_steps);

  return 0;
}

int set_all_vehicle_parameters_from_sliders()
{
  no_of_steps = (int) fl_get_slider_value(fd_formcom->slider_set_steps);

  return 0;
}

// Normal version of go_vehicle
int go_vehicle(int auto_select_flag)
{
  struct timeval tod1;
  gettimeofday(&tod1, NULL);
  double time1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;

  sim_or_rob->set_control(u, delta_t);
  
  kalman->predict_filter(scene, u, delta_t);

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

  struct timeval tod2;
  gettimeofday(&tod2, NULL);
  double time2 = tod2.tv_sec + tod2.tv_usec / 1000000.0;

  cout << "Time take for 1 step: " << time2 - time1 << " seconds." << endl;

  return 0;
}


// Called from forms: standard go with no auto select
void go_step_by_step(FL_OBJECT *ob, long data)
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

// Go with auto_select
void go_auto_select(FL_OBJECT *ob, long data)
{
  set_all_vehicle_parameters_from_sliders();

  for (int steps = 0; steps < no_of_steps; steps++)
  {
    go_vehicle(1);
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

void zero_axes(FL_OBJECT *ob, long data)
{
  zero_axes(scene, sim_or_rob, NULL, three_d_disp);

#ifdef _INSPECT_
  inspector->update_form();
#endif
}


void output_postscript(FL_OBJECT *ob, long data)
{
#ifdef _ROBOT_
  postscript->output_postscript(scene);
#else
  postscript->output_postscript(scene, simulation);
#endif

  cout << "Output postscript to file " << ps_filename << "." << endl;
}
          

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
