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

// Main file where everything is controlled from

#include <general_headers.h>
#include "models_base.h"
#include "models_oned.h"
#include "3d.h"
#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"
#include "kalman.h"
#include "sim_or_rob.h"
#include "simul.h"

#include "waypoints.h"
#include "forms.h"
#include "formoned.h"
#include "waypoint_browser.h"

// At the moment, kalman_form and scene_inspector are tangled and must be used
// together
#include "formkalman.h"
#include "forminspect.h"
#include "scene_inspect.h"
#include "kalman_form.h"

#include "control_general.h"
#include "control.h"



/**********************Main class pointers defined here***********************/

// Define one generic type motion model, but potentially several 
// specific feature models
Simple_OneD_Motion_Model *simple_oned_motion_model;
Motion_Model *motion_model;
OneD_Point_Feature_Measurement_Model *oned_point_feature_measurement_model;

int number_of_feature_measurement_models = 1;
Feature_Measurement_Model **feature_measurement_model_array;

Simulation *simulation;
Sim_Or_Rob *sim_or_rob;
Three_D_Display *three_d_disp;
Scene *scene;
Kalman *kalman;

Scene_Inspect *inspector;

char *initial_state_file = "initial_state";
char *true_data_file = "scene_features";
char *known_features_file = "known_features";
char *waypoints_file = "waypoints";

/*****************************Display Parameters******************************/

// Global display pointer
Display *display;

/* Horatio colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;

/*****************************Control Parameters******************************/

// Elements of u: (v_control)
Hor_Matrix *u;

int no_of_steps;

double delta_t;

/****************************Navigation Waypoints*****************************/

Waypoints *waypoints;

/************************Initialise stuff in this file************************/

static FD_formoned *fd_formoned;

static FD_formkalman *fd_formkalman;
static FD_forminspect *fd_forminspect;

void pass_fd_pointer_oned(FD_formoned *fd_fo)
{
  fd_formoned = fd_fo;
}

XtAppContext app_con;

int set_up_control()
{
  display = fl_get_display();

  simple_oned_motion_model = new Simple_OneD_Motion_Model;
  motion_model = simple_oned_motion_model;
  oned_point_feature_measurement_model = 
                 new OneD_Point_Feature_Measurement_Model (motion_model);

  // Make array of different available feature measurement models
  number_of_feature_measurement_models = 1;
  feature_measurement_model_array = new Feature_Measurement_Model*[number_of_feature_measurement_models];
  feature_measurement_model_array[0] = oned_point_feature_measurement_model;

  Hor_Matrix *initial_xv = hor_mats_zero(motion_model->STATE_SIZE, 1);
  Hor_Matrix *initial_Pxx = hor_mats_zero(motion_model->STATE_SIZE,
					  motion_model->STATE_SIZE);   

  read_in_initial_state(initial_xv, initial_Pxx, initial_state_file, 
			motion_model);

  scene = new Scene_Single(initial_xv, initial_Pxx, motion_model);

  hor_mat_free_list(initial_xv, initial_Pxx, NULL);

  fd_forminspect = create_form_forminspect ();
  inspector = new Scene_Inspect(fd_forminspect, scene);

  kalman = new Kalman;

  // Create kalman parameter form
  fd_formkalman = create_form_formkalman ();

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
  char *argv = "nnn";
  int argc = 0;
  dummy = XtAppInitialize(&app_con, "horatio", NULL, ZERO,
			  &argc, &argv, fallback_resources, NULL, ZERO);
  GC gc = fl_get_gc();
  hor_display_initialise ( display, gc,
			   SteelBlue, Green, Red, LightSeaGreen, thistle );

  three_d_disp = new Three_D_Display(dummy, display);

  initialise_simulation(simulation, true_data_file, motion_model, 
			number_of_feature_measurement_models, 
			feature_measurement_model_array);
  sim_or_rob = simulation;

  initialise_known_features(known_features_file,
			    number_of_feature_measurement_models, 
			    feature_measurement_model_array, 
			    sim_or_rob, scene);

  // Initialise waypoints class
  waypoints = new Waypoints(waypoints_file, motion_model);

  initialise_waypoint_browser(waypoints);

  //  simulation->print_true_features();
  display_true_features(simulation, three_d_disp);
  display_true_vehicle(simulation, three_d_disp);  

  display_estimated_vehicle(scene, three_d_disp);

  // Now everything's declared, pass pointers to kalman form
  pass_all_ptrs (kalman, scene, three_d_disp, sim_or_rob,
		 inspector, fd_formkalman);

  // Display initial state on inspect form
  inspector->update_form ();

  return 0;
}

/********* Callbacks for when buttons to open other forms are clicked ********/


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

/**********Called from 3d.cc when feature is selected with the mouse**********/

int please_redisplay_features_flag = 0;

int true_feature_selected(int i)
{
  // Get the identifier for this feature (always a pointer)
  void *id = (void *) simulation->find_identifier(i);

  // See if it is a feature we already know about
  if (scene->find_feature(id))
  {
    if (!scene->toggle_feature(id))
      cerr << "Trouble toggling feature with identifier" << id << endl;
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

  inspector->update_form ();

  return 0;
}

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

  inspector->update_form ();

  return 0;
}

/**********************Set vehicle control parameters*************************/

void set_v(FL_OBJECT *ob, long data)
{
  matel(u, 1, 1) = fl_get_slider_value(ob);
}

void zero_v(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formoned->slider_set_v, 0.0);
  matel(u, 1, 1) = 0.0;
}

void set_delta_t(FL_OBJECT *ob, long data)
{
  delta_t = fl_get_slider_value(ob);
}

void one_second(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formoned->slider_set_delta_t, 1.0);
  delta_t = 1.0;
}

void set_steps(FL_OBJECT *ob, long data)
{
  no_of_steps = (int) fl_get_slider_value(ob);
}

void one_steps(FL_OBJECT *ob, long data)
{
  fl_set_slider_value(fd_formoned->slider_set_steps, 1.0);
  no_of_steps = 1;
}

int set_all_vehicle_sliders_from_parameters()
{
  fl_set_slider_value(fd_formoned->slider_set_v, matel(u, 1, 1));
  fl_set_slider_value(fd_formoned->slider_set_delta_t, delta_t);
  fl_set_slider_value(fd_formoned->slider_set_steps, no_of_steps);

  return 0;
}

int set_all_vehicle_parameters_from_sliders()
{
  matel(u, 1, 1) = fl_get_slider_value(fd_formoned->slider_set_v);
  delta_t = fl_get_slider_value(fd_formoned->slider_set_delta_t);
  no_of_steps = (int) fl_get_slider_value(fd_formoned->slider_set_steps);

  return 0;
}

void go_vehicle(FL_OBJECT *ob, long data)
{
  set_all_vehicle_parameters_from_sliders();

  for (int steps = 0; steps < no_of_steps; steps++)
  {
    go_vehicle(sim_or_rob, scene, kalman, three_d_disp, u, delta_t, 0);
  }

  inspector->update_form ();
}

void navigate_to_next_waypoint(FL_OBJECT *ob, long data)
{
  while(scene->get_motion_model()->navigate_to_waypoint(scene->get_xv(), 
				 waypoints->get_current_waypoint_vector(), 
			         u, delta_t) != 1)
  {
    go_vehicle(sim_or_rob, scene, kalman, three_d_disp, u, delta_t, 1);
  }

  waypoints->increment_current_waypoint();
  highlight_waypoint_browser_line();

  inspector->update_form ();
}

/****************Delete the last feature selected with the mouse**************/

void delete_feature(FL_OBJECT *ob, long data)
{
  delete_feature(scene, three_d_disp);

  inspector->update_form ();
}

// Print State
void print_state(FL_OBJECT *ob, long data)
{
  scene->print_whole_state();
}

// Output State to File
void output_state(FL_OBJECT *ob, long data)
{
  scene->output_state_to_file();
}

// Print True State
void print_true_state(FL_OBJECT *ob, long data)
{
  simulation->print_true_vehicle();
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

/****************Redefine Axes at Current Waypoint Position*******************/

void zero_axes(FL_OBJECT *ob, long data)
{
  zero_axes(scene, sim_or_rob, waypoints, three_d_disp);

  display_waypoints();
  inspector->update_form();
}

/************************************Quit*************************************/

void quit(FL_OBJECT *ob, long data)
{
  // Delete robot class so hardware is shut down in robot case
  delete sim_or_rob;

  exit(0);
}

/*****************************************************************************/
