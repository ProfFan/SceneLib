/*  Scene: software for sequential localisation and map-building

    control_general.cc
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

// A collection of general functions operating on the main classes which
// are common to many applications and are therefore grouped together here.

#include <general_headers.h>

#include "models_base.h"
#include "models_double.h"
#include "3d.h"
#include "feature.h"
#include "scene_base.h"
#include "kalman.h"
#include "sim_or_rob.h"
#include "simul.h"
#include "waypoints.h"
#include "control_general.h"

/************************Find Feature Measurement Model***********************/

Feature_Measurement_Model *find_feature_measurement_model(char *type,
	     int number_of_feature_measurement_models,
	     Feature_Measurement_Model **feature_measurement_model_array)
{
  for (int i = 0; i < number_of_feature_measurement_models; i++)
  {
    if (strcmp(feature_measurement_model_array[i]->feature_type, type) == 0)
      return feature_measurement_model_array[i];
  }
  // There is a bug if we reach here
  bool FEATURE_MEASUREMENT_MODEL_NOT_FOUND = false;
  assert (FEATURE_MEASUREMENT_MODEL_NOT_FOUND);
  return NULL;
}

/*************************Display things in 3D Tool***************************/

int display_estimated_features(Scene *scene, Three_D_Display *three_d_disp)
{
  for (Feature *fp = scene->get_first_feature_ptr(); fp; fp = fp->get_next())
  {
    if (strcmp(
        fp->get_feature_measurement_model()->feature_dimensionality_type, 
        "POINT") == 0)
    {
      fp->get_feature_measurement_model()->func_yipose_and_Pyiyipose(
                                                               fp->get_y(),
							       fp->get_Pyy());

      if (fp->get_selected_flag())
      {
	three_d_disp->draw_estimated_selected_point(
			 fp->get_feature_measurement_model()->yiposeRES, 
			 fp->get_label());
	three_d_disp->draw_estimated_selected_point_covariance(
			 fp->get_feature_measurement_model()->yiposeRES, 
		         fp->get_feature_measurement_model()->PyiyiposeRES, 
			 fp->get_label());
      }
      else
      {
	three_d_disp->draw_estimated_point(
                         fp->get_feature_measurement_model()->yiposeRES, 
		       	 fp->get_label());
	three_d_disp->draw_estimated_point_covariance(
		       	 fp->get_feature_measurement_model()->yiposeRES, 
		         fp->get_feature_measurement_model()->PyiyiposeRES, 
	       		 fp->get_label());
      }
    }
  }

  return 0;
}

int display_estimated_selected_features(Scene *scene, Three_D_Display *three_d_disp)
{
  for (Feature *sfp = scene->get_first_selected_ptr(); sfp; 
                                                sfp = sfp->get_next_selected())
  {
    if (strcmp(
        sfp->get_feature_measurement_model()->feature_dimensionality_type, 
        "POINT") == 0)
    {
      sfp->get_feature_measurement_model()->func_yipose_and_Pyiyipose(
                                                             sfp->get_y(),
							     sfp->get_Pyy());

      three_d_disp->draw_estimated_point(
                         sfp->get_feature_measurement_model()->yiposeRES, 
    			 sfp->get_label());
      three_d_disp->draw_estimated_point_covariance(
                         sfp->get_feature_measurement_model()->yiposeRES, 
		         sfp->get_feature_measurement_model()->PyiyiposeRES, 
			 sfp->get_label());
    }
  }

  return 0;
}

int display_estimated_vehicle(Scene *scene, Three_D_Display *three_d_disp)
{
  scene->get_motion_model()->func_xpose_and_Rpose(scene->get_xv());
  three_d_disp->draw_estimated_robot(scene->get_motion_model()->xposeRES, 
    scene->get_motion_model()->RposeRES);

  // Draw second vehicle in double case
  if (strcmp(scene->get_motion_model()->motion_model_type, "DOUBLE") == 0)
  {
    Double_Motion_Model *double_motion_model =
                           (Double_Motion_Model *) (scene->get_motion_model());
    double_motion_model->func_xpose2_and_Rpose2(scene->get_xv());
    three_d_disp->draw_estimated_robot2(double_motion_model->xpose2RES, 
					double_motion_model->Rpose2RES);
  }

  return 0;
}

int display_true_features(Simulation *simulation, Three_D_Display *three_d_disp)
{
  for (Simulation_Feature *sfp = simulation->get_first_feature(); sfp; 
                                          sfp = sfp->get_next())
  {
    if (strcmp(
        sfp->get_feature_measurement_model()->feature_dimensionality_type, 
        "POINT") == 0)
    {
      // Send dummy covariance matrix here
      sfp->get_feature_measurement_model()->func_yipose_and_Pyiyipose(
                              sfp->get_yi(),
			      sfp->get_feature_measurement_model()->Temp_FF1);

      three_d_disp->draw_true_point(
                           sfp->get_feature_measurement_model()->yiposeRES, 
				    sfp->get_label());
    }
  }

  return 0;
}

int display_true_vehicle(Simulation *simulation, Three_D_Display *three_d_disp)
{
  simulation->get_motion_model()->func_xpose_and_Rpose(simulation->get_xv());
  three_d_disp->draw_true_robot(simulation->get_motion_model()->xposeRES, 
				simulation->get_motion_model()->RposeRES);

  if (strcmp(simulation->get_motion_model()->motion_model_type, "DOUBLE") == 0)
  {
    Double_Motion_Model *double_motion_model = (Double_Motion_Model *) 
                                   (simulation->get_motion_model());
    double_motion_model->func_xpose2_and_Rpose2(simulation->get_xv());
    three_d_disp->draw_true_robot2(double_motion_model->xpose2RES, 
				   double_motion_model->Rpose2RES);
  }

  return 0;
}

/****************************Initialise Simulation****************************/

int initialise_simulation(Simulation *&simulation, char *true_data_file,
			  Motion_Model *motion_model, 
			  int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array)
{
  ifstream infile(true_data_file, ios::in);
  // assert(infile != (void *) NULL);

  char type_buf[100];

  // Read in initial vehicle state
  infile >> type_buf;
  assert(strcmp(type_buf, motion_model->motion_model_type) == 0);

  Hor_Matrix *initial_xv = hor_mat_alloc(motion_model->STATE_SIZE, 1);
  for (int i = 0; i < motion_model->STATE_SIZE; i++)
    infile >> matel(initial_xv, i + 1, 1);
  simulation = new Simulation(motion_model, initial_xv);
  hor_mat_free(initial_xv);

  // Check for special command EVENSPACE
  // Now generalised for POINTS: after EVENSPACE in file, put type of FEATURE 
  // to fill with.
  // Then the correct number of parameters for the dimension of that feature

  int pos = infile.tellg ();
  string command;
  infile >> command;
  if (command == string ("EVENSPACE")) {
    infile >> type_buf;
    Feature_Measurement_Model *f_m_m = 
          find_feature_measurement_model(type_buf,
					 number_of_feature_measurement_models, 
					 feature_measurement_model_array);
    // Evenspace only works for POINT features so far
    assert(strcmp(f_m_m->feature_dimensionality_type, "POINT") == 0);

    // Lazy generalisation!
    switch(f_m_m->POSE_SIZE) {
    case 1: {
      double xmin, xmax, step;
      infile >> xmin >> xmax >> step;

      for (double x = xmin; x <= xmax; x += step) {
	    Hor_Matrix *initial_yi = hor_mat_alloc (1, 1);
	    hor_matq_fill (initial_yi, x);

	    simulation->add_new_true_feature (f_m_m, initial_yi);
      }

      break;
    }
    case 2: {
      double xmin, ymin, xmax, ymax, step;
      infile >> xmin >> ymin >> xmax >> ymax >> step;

      for (double x = xmin; x <= xmax; x += step) {
	for (double y = ymin; y <= ymax; y += step) {
	    Hor_Matrix *initial_yi = hor_mat_alloc (2, 1);
	    hor_matq_fill (initial_yi, x, y);

	    simulation->add_new_true_feature (f_m_m, initial_yi);
	}
      }

      break;
    }
    case 3: {
      double xmin, ymin, zmin, xmax, ymax, zmax, step;
      infile >> xmin >> ymin >> zmin >> xmax >> ymax >> zmax >> step;

      for (double x = xmin; x <= xmax; x += step) {
	for (double y = ymin; y <= ymax; y += step) {
	  for (double z = zmin; z <= zmax; z += step) {
	    Hor_Matrix *initial_yi = hor_mat_alloc (3, 1);
	    hor_matq_fill (initial_yi, x, y, z);

	    simulation->add_new_true_feature (f_m_m, initial_yi);
	  }
	}
      }

      break;
    }

    } // end of switch


  }
  else infile.seekg (pos);   // Unread what we just read

  // Read scene data points in from file
  while (infile)
  {
    infile >> type_buf;
    if (!infile) break;

    Feature_Measurement_Model *f_m_m = find_feature_measurement_model(type_buf,
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

/******************************Initialise Scene*******************************/

int read_in_initial_state(Hor_Matrix *initial_xv, Hor_Matrix *initial_Pxx,
			  char *initial_state_file, Motion_Model *motion_model)
{
  assert(initial_xv->rows == motion_model->STATE_SIZE && 
	 initial_xv->cols == 1 && 
	 initial_Pxx->rows == motion_model->STATE_SIZE &&
	 initial_Pxx->cols == motion_model->STATE_SIZE); 

  hor_matq_zero(initial_xv);
  hor_matq_zero(initial_Pxx);

  // Read initial state from file
  ifstream infile(initial_state_file, ios::in);
  // if (infile != (void *) NULL)
  {
    cout << "Reading initial robot state from file " << initial_state_file
	 << endl;

    char type_buf[100];
    infile >> type_buf;
    assert(strcmp(type_buf, motion_model->motion_model_type) == 0);

    for (int r = 0; r < motion_model->STATE_SIZE; r++)
      infile >> matel(initial_xv, r + 1, 1);
    for (int r = 0; r < motion_model->STATE_SIZE; r++)
      for (int c = 0; c < motion_model->STATE_SIZE; c++)
	infile >> matel(initial_Pxx, r + 1, c + 1);
  }
  // else
  //   cout << "WARNING: no file " << initial_state_file 
	//  << "; initialising state and covariance to zero." << endl;

  return 0;
}

/******************************Make Measurements******************************/

int make_measurements(Scene *scene, Sim_Or_Rob *sim_or_rob)
{
  Feature *sfp = scene->get_first_selected_ptr();
  if (sfp == NULL)
  {
    cerr << "No selected feature list." << endl;
    return -1;
  }

  scene->starting_measurements(); 

  for (; sfp; sfp = sfp->get_next_selected())
  {
    cout << "S" << sfp->get_S();
    if (sim_or_rob->measure_feature(sfp->get_identifier(), sfp->get_z(),
				    sfp->get_h(), sfp->get_S()) != 0) {
      scene->failed_measurement_of_feature(sfp);
    }
    else
      scene->successful_measurement_of_feature(sfp); {
      cout << "h" << sfp->get_h() << "z" << sfp->get_z();
    }
  }

  return 0;
}

/**********************************Go Vehicle*********************************/

int go_vehicle(Sim_Or_Rob *sim_or_rob, Scene *scene, Kalman *kalman,
	       Three_D_Display *three_d_disp, Hor_Matrix *u, double delta_t,
	       int auto_select_flag)
{
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

    make_measurements(scene, sim_or_rob); // In this file

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

  return 0;
}

/**************************Delete marked feature******************************/

int delete_feature(Scene *scene, Three_D_Display *three_d_disp)
{
  three_d_disp->remove_estimated_point(scene->get_marked_feature_label());
  three_d_disp->remove_estimated_point_covariance(
                                       scene->get_marked_feature_label());
  scene->delete_feature();

  return 0;
}

/**************************Initialise Known Features**************************/

int initialise_known_features(char *known_features_file, 
			      int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array,
			      Sim_Or_Rob *sim_or_rob, Scene *scene)
{
  ifstream infile(known_features_file, ios::in);
  // if (infile == NULL)
  // {
  //   cout << "No file " << known_features_file 
	//  << " --- no prior-known features." << endl;
  //   return -1;
  // }

  // Format for known_points file: 
  // FEATURE_TYPE known_feature_label feature_coordinates... xp_orig coords...

  char type_buf[100];
  int known_feature_label;

  while (infile)
  {
    infile >> type_buf;
    if (!infile) break;

    Feature_Measurement_Model *f_m_m = 
                find_feature_measurement_model(type_buf,
			                 number_of_feature_measurement_models, 
                                         feature_measurement_model_array);

    infile >> known_feature_label;

    Hor_Matrix *yi = hor_mat_alloc(f_m_m->FEATURE_STATE_SIZE, 1);

    for (int i = 0; i < f_m_m->FEATURE_STATE_SIZE; i++)
      infile >> matel(yi, i + 1, 1);

    Hor_Matrix *xp_orig = hor_mat_alloc(
		     scene->get_motion_model()->POSITION_STATE_SIZE, 1);

    for (int i = 0; i < scene->get_motion_model()->POSITION_STATE_SIZE; i++)
      infile >> matel(xp_orig, i + 1, 1);

    void *identifier = sim_or_rob->initialise_known_feature(f_m_m, yi,
							known_feature_label);
    if (identifier == NULL)
    {
      cerr << "Trouble reading known feature " << known_feature_label 
	   << " identifier : skipping." << endl;
    }
    else
    {
      scene->add_new_known_feature(identifier, yi, xp_orig, f_m_m, 
				   known_feature_label);
      cout << "Added known feature with known feature label " 
	   << known_feature_label << endl;
    }

    hor_mat_free_list(yi, xp_orig, NULL);
  }  

  return 0;
}

/******************************Initialise Feature*****************************/

int initialise_feature(Scene *scene, Sim_Or_Rob *sim_or_rob, 
		       Three_D_Display *three_d_disp,
		       char *feature_type, void *id, 
		       int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array)
{
  Feature_Measurement_Model *f_m_m = 
          find_feature_measurement_model(feature_type,
			                 number_of_feature_measurement_models, 
                                         feature_measurement_model_array);
  return initialise_feature (scene, sim_or_rob, three_d_disp, id, f_m_m);
}


int initialise_feature (Scene *scene, Sim_Or_Rob *sim_or_rob, 
			Three_D_Display *three_d_disp,
			void *id, Feature_Measurement_Model *f_m_m)
{
  Hor_Matrix *z = hor_mat_alloc(f_m_m->MEASUREMENT_SIZE, 1); 

  cout << "Making measurement to initialise new " << f_m_m->feature_type 
       << " feature." << endl;

  if (sim_or_rob->make_initial_measurement_of_feature(z, id, f_m_m) == -1)
    cout << "Failed in initialisation of feature." << endl;
  else
    scene->add_new_feature(id, z, f_m_m);

  hor_mat_free(z);

  scene->print_selected_features();

  display_estimated_features(scene, three_d_disp);

  return 0;
}


/*********************************Zero Axes***********************************/

// Redefine axes at the current robot position
// Change Scene, Simulation (if it exists), Waypoints

int zero_axes(Scene *scene, Sim_Or_Rob *sim_or_rob, Waypoints *waypoints,
	      Three_D_Display *three_d_disp)
{
  if (waypoints != NULL) {
    // I am not sure if it really makes sense to zero waypoints like this  
    waypoints->zero_axes(scene->get_xv());
  }

  scene->zero_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    Simulation *simulation = (Simulation *) sim_or_rob;
    simulation->zero_axes();
  }

  three_d_disp->draw_axes();

  if (strcmp(sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle((Simulation *) sim_or_rob, three_d_disp);
    display_true_features((Simulation *) sim_or_rob, three_d_disp);
  }

  display_estimated_features(scene, three_d_disp);
  display_estimated_vehicle(scene, three_d_disp);

  return 0;
}
