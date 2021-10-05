/*  Scene: software for sequential localisation and map-building

    control_general.h
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

// Some master control functions

/************************Find Feature Measurement Model***********************/

Feature_Measurement_Model *find_feature_measurement_model(char *type,
	     int number_of_feature_measurement_models,
	     Feature_Measurement_Model **feature_measurement_model_array);

/*************************Display things in 3D Tool***************************/

int display_estimated_features(Scene *scene, Three_D_Display *three_d_disp);
int display_estimated_selected_features(Scene *scene, 
					Three_D_Display *three_d_disp);
int display_estimated_vehicle(Scene *scene, Three_D_Display *three_d_disp);
int display_true_features(Simulation *sim_or_rob, Three_D_Display *three_d_disp);
int display_true_vehicle(Simulation *sim_or_rob, Three_D_Display *three_d_disp);

/****************************Initialise Simulation****************************/

int initialise_simulation(Simulation *&sim_or_rob, char *true_data_file,
			  Motion_Model *motion_model, 
			  int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array);

/*************************Read in Initial State File**************************/

int read_in_initial_state(Hor_Matrix *initial_xv, Hor_Matrix *initial_Pxx,
	       	  char *initial_state_file, Motion_Model *motion_model);

/******************************Make Measurements******************************/

int make_measurements(Scene *scene, Sim_Or_Rob *sim_or_rob);

/**********************************Go Vehicle*********************************/

int go_vehicle(Sim_Or_Rob *sim_or_rob, Scene *scene, Kalman *kalman,
	       Three_D_Display *three_d_disp, Hor_Matrix *u, double delta_t,
	       int auto_select_flag);

/**************************Delete marked feature******************************/

int delete_feature(Scene *scene, Three_D_Display *three_d_disp);

/**************************Initialise Known Features**************************/

int initialise_known_features(char *known_features_file, 
			      int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array,
			      Sim_Or_Rob *sim_or_rob, Scene *scene);

/******************************Initialise Feature*****************************/

int initialise_feature(Scene *scene, Sim_Or_Rob *sim_or_rob, 
		       Three_D_Display *three_d_disp,
		       char *feature_type, void *id, 
		       int number_of_feature_measurement_models, 
                  Feature_Measurement_Model **feature_measurement_model_array);

int initialise_feature (Scene *scene, Sim_Or_Rob *sim_or_rob, 
			Three_D_Display *three_d_disp,
			void *id, Feature_Measurement_Model *f_m_m);

int initialise_visible_features (Simulation *simul, Scene *scene,
				 Three_D_Display *three_d_disp);

/*********************************Zero Axes***********************************/

int zero_axes(Scene *scene, Sim_Or_Rob *sim_or_rob, Waypoints *waypoints,
	      Three_D_Display *three_d_disp);
