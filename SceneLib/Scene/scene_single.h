/*  Scene: software for sequential localisation and map-building

    scene_single.h
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Additions from Joss Knight
    joss@robots.ox.ac.uk
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


/** Single coupled-covariance map */
class Scene_Single : public Scene {
  friend class Feature; 
  // Constructor 
 public:
  Scene_Single(Hor_Matrix *initial_xv, 
	Hor_Matrix *initial_Pxx,
	Motion_Model *m_m);

  // Alternative constructor allows an internal measurement model to be 
  // specified
  Scene_Single(Hor_Matrix *initial_xv, 
	Hor_Matrix *initial_Pxx,
	Motion_Model *m_m, Internal_Measurement_Model *i_m_m);

 protected:
  int scene_constructor_bookkeeping(Hor_Matrix *initial_xv, 
	     Hor_Matrix *initial_Pxx);

  // Motion model pointer: passed to constructor
 protected:
  Motion_Model *motion_model;

  // Main data
 protected:
  Hor_Matrix *xv;
  Hor_Matrix *Pxx;
  Feature *first_feature_ptr;
  Feature *last_feature_ptr;
  Feature *first_selected_ptr;
  Feature *last_selected_ptr;
  int no_features;
  int no_selected;
  int next_free_label; 

  Hor_Matrix ***Matrix_Array; /* Array of pointers to matrix elements. 
				 This is not really necessary because 
				 they're all stored in linked lists but it
				 makes some things easier. Allocated when 
				 we add or delete a feature */
  int Matrix_Array_dimension; // Store so we can free it up
  int total_state_size;       /* Calculate at the same time as Matrix_Array:
				 the size of the total state vector */

  int successful_measurement_vector_size;
                                 /* Recalculate when we make measurements */

  // General access functions
 public:
  Motion_Model *get_motion_model() {return motion_model;}
  Hor_Matrix *get_xv() {return xv;}
  Hor_Matrix *get_Pxx() {return Pxx;}
  int get_no_selected() {return no_selected;}
  int get_no_features() {return no_features;}
  int get_total_state_size() {return total_state_size;}
  int get_successful_measurement_vector_size() 
                             {return successful_measurement_vector_size;}
  void set_successful_measurement_vector_size(int smvs) 
                             {successful_measurement_vector_size = smvs;}
  Feature *get_first_feature_ptr() {return first_feature_ptr;}
  Feature *get_first_selected_ptr() {return first_selected_ptr;}
  int get_marked_feature_label() {return marked_feature_label;}
  int get_Pyjyk (int j, int k, Hor_Matrix *Pyjyk);
  Hor_Matrix *get_Pyjyk_ptr (int j, int k);

  // Adding / deleting features
 public:
  int add_new_feature(Identifier id, Hor_Matrix *y, 
		      Feature_Measurement_Model *f_m_m);
  int add_new_known_feature(Identifier id, Hor_Matrix *y_known, 
			    Hor_Matrix *xp_o, 
		 Feature_Measurement_Model *f_m_m, int known_feature_label);
  int delete_feature();
  int exterminate_features();
 protected:
  int add_new_feature_bookeeping(Feature *nf);


  // Finding features from the list
 public:
  Feature *find_feature(Identifier id);
  Feature *find_feature_lab(int lab);


  // Selecting / deselecting features
 public:
  int select_feature(Feature *fp);
  int deselect_feature(Feature *fp);
  int toggle_feature(Identifier id);
  int toggle_feature_lab(int lab);

  // Auto-select feature
 public:
  int auto_select_feature();

  // Output info on current features
 public:
  int print_current_features();
  int print_selected_features();
  int print_covariances();
  int print_robot_state();
  int print_marked_feature_state();
  int print_whole_state();
  int output_state_to_file(int counter, char *filename);
  int output_state_to_file(int counter);
  int output_state_to_file();
 protected:
  int output_counter;


  // Functions for doing stuff with the total state and covariance
 public:
  int construct_total_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M);
  int fill_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M);

 public:
  int construct_total_state(Hor_Matrix *V);
  int construct_total_covariance(Hor_Matrix *M);
  int fill_states(Hor_Matrix *V);
  int fill_covariances(Hor_Matrix *M);
 protected:  
  int fill_matrix_pointer_array();

  // Do bookkeeping when measurements are made
 public:
  int starting_measurements();
  int failed_measurement_of_feature(Feature *fp);
  int successful_measurement_of_feature(Feature *fp);
  void age_all_features (Feature *);

  // Form matrices for measurement update
 public:
  int construct_total_measurement_stuff(Hor_Matrix *nu_tot, 
					Hor_Matrix *dh_by_dx_tot,
					Hor_Matrix *R);
  int predict_single_feature_measurements(Feature *sfp);
  int predict_single_feature_measurements (Feature_Measurement_Model *model,
					   Hor_Matrix *y,
					   Hor_Matrix *xp, Hor_Matrix *h,
					   Hor_Matrix *dh_by_dy,
					   Hor_Matrix *dh_by_dxv,
					   Hor_Matrix *R);
 public:
  int predict_measurements();

  // Test a certain feature for measurability
 public:
  int test_for_visibility(Feature *fp);

  // Work out innovation covariances for measurement update
 protected:
  int work_out_cartesian_innovation_covariance(Feature *sfp);
  int work_out_angular_innovation_covariance(Feature *sfp);

  // Mark a feature: for deleting, steering round, etc.
 protected:
  int marked_feature_label; /* The label of the last feature selected 
                               with the mouse */
 public:
  int mark_feature_by_lab(int lab);
  int mark_first_feature();

  // For using internal measurement models
 protected:
  Internal_Measurement_Model *internal_measurement_model;
  Hor_Matrix *hv, *zv, *nuv, *dhv_by_dxv, *Rv, *Sv;
  int successful_internal_measurement_flag;
 public:
  int predict_internal_measurement();
  Internal_Measurement_Model *get_internal_measurement_model()
          {return internal_measurement_model;}
  
  Hor_Matrix *get_hv() {return hv;}
  Hor_Matrix *get_zv() {return zv;}
  Hor_Matrix *get_Sv() {return Sv;}

  int successful_internal_measurement();
  int failed_internal_measurement();
  int get_successful_internal_measurement_flag()
    {return successful_internal_measurement_flag;}

 public:
  int construct_total_internal_measurement_stuff(Hor_Matrix *nu_tot, 
						 Hor_Matrix *dh_by_dx_tot,
						 Hor_Matrix *R_tot);

  int zero_axes();

  int normalise_state();
};

/*****************************************************************************/
