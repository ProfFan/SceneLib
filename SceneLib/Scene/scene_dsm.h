/*  Scene: software for sequential localisation and map-building
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

// Scene classes to support DSM: Decoupled Stochastic Mapping

/** Individual submap class: basically a normal Scene class but with
    DSM trimmings */
class Scene_DSM_Submap : public Scene_Single {
  friend class Scene_DSM;
 public:
  Scene_DSM_Submap(Hor_Matrix *initial_xv, Hor_Matrix *initial_Pxx,
		   Motion_Model *m_m,
		   Hor_Matrix *b_box_xp1, Hor_Matrix *b_box_xp2,
		   int sm_number);
  // Alternative constructor allows an internal measurement model to be 
  // specified
  Scene_DSM_Submap(Hor_Matrix *initial_xv, Hor_Matrix *initial_Pxx,
		   Motion_Model *m_m,
		   Hor_Matrix *b_box_xp1, Hor_Matrix *b_box_xp2,
		   int sm_number, Internal_Measurement_Model *i_m_m);
  int constructor_bookkeeping(Hor_Matrix *b_box_xp1, 
			      Hor_Matrix *b_box_xp2,
			      int sm_number);

 protected: 
  // Submap details
  int submap_number;
  Scene_DSM_Submap *next_submap_pointer;

  // Bounding box limits
  Hor_Matrix *bounding_box_xp1, *bounding_box_xp2;

};

/** Bounding Class: provides the public interface for DSM and looks 
    after the individual submap classes */

class Scene_DSM : public Scene {
 public:
  // Constructor 
  Scene_DSM(Hor_Matrix *initial_xv, 
	    Hor_Matrix *initial_Pxx,
	    Motion_Model *m_m,
	    Hor_Matrix *b_b_size);
  // Alternative constructor allows an internal measurement model to be 
  // specified
  Scene_DSM(Hor_Matrix *initial_xv, 
	    Hor_Matrix *initial_Pxx,
	    Motion_Model *m_m, Hor_Matrix *b_b_size,
	    Internal_Measurement_Model *i_m_m);
  int constructor_bookkeeping(Hor_Matrix *initial_xv, 
		     Hor_Matrix *initial_Pxx,
		     Motion_Model *m_m,
		     Hor_Matrix *b_b_size);

 protected:
  Motion_Model *motion_model;
  Internal_Measurement_Model *internal_measurement_model;

  // How big submaps are to be
  Hor_Matrix *bounding_box_size;
  Hor_Matrix *initial_xp;

  // Check if we are still in current submap
  int check_submap();
  
  // Two types of update for switching between maps
  int cross_map_relocation(Scene_DSM_Submap *submapA, Scene_DSM_Submap *submapB);
  int cross_map_updating(Scene_DSM_Submap *submapA, Scene_DSM_Submap *submapB);

  // Linked list of submap classes
  int no_of_submaps;
  Scene_DSM_Submap *current_submap_pointer;
  Scene_DSM_Submap *first_submap_pointer;
  Scene_DSM_Submap *last_submap_pointer;

  // Internal bookkeeping
  int add_new_submap(Hor_Matrix *initial_xv, 
		     Hor_Matrix *initial_Pxx);
  int find_bounding_box_of_position(Hor_Matrix *xp, 
				    Hor_Matrix *bounding_box_xp1,
				    Hor_Matrix *bounding_box_xp2);
  Scene_DSM_Submap *find_known_submap(Hor_Matrix *xp);



  // Public interface of Scene adapted
  // General access functions
 public:
  // Fires off submap-checking
  int vehicle_state_has_been_changed();

  Motion_Model *get_motion_model() 
    {return current_submap_pointer->get_motion_model();}
  Hor_Matrix *get_xv() 
    {return current_submap_pointer->get_xv();}
  Hor_Matrix *get_Pxx()
    {return current_submap_pointer->get_Pxx();}
  int get_no_selected()
    {return current_submap_pointer->get_no_selected();}
  int get_no_features()
    {return current_submap_pointer->get_no_features();}
  int get_total_state_size()
    {return current_submap_pointer->get_total_state_size();}
  int get_successful_measurement_vector_size() 
    {return current_submap_pointer->get_successful_measurement_vector_size();}
  void set_successful_measurement_vector_size(int smvs) 
    {current_submap_pointer->set_successful_measurement_vector_size(smvs);}
  Feature *get_first_feature_ptr()
    {return current_submap_pointer->get_first_feature_ptr();}
  Feature *get_first_selected_ptr()
    {return current_submap_pointer->get_first_selected_ptr();}
  int get_marked_feature_label()
    {return current_submap_pointer->get_marked_feature_label();}

  // Adding / deleting features
 public:
  int add_new_feature(Identifier id, Hor_Matrix *y, 
		      Feature_Measurement_Model *f_m_m)
    {return current_submap_pointer->add_new_feature(id, y, f_m_m);}
  int add_new_known_feature(Identifier id, Hor_Matrix *y_known, Hor_Matrix *xp_o, 
		 Feature_Measurement_Model *f_m_m, int known_feature_label)
    {return current_submap_pointer->add_new_known_feature(id, y_known, xp_o,
					      f_m_m, known_feature_label);}
  int delete_feature()
    {return current_submap_pointer->delete_feature();}
  int exterminate_features()
    {return current_submap_pointer->exterminate_features();}
 public:
  Feature *find_feature(Identifier id)
    {return current_submap_pointer->find_feature(id);}
  Feature *find_feature_lab(int lab)
    {return current_submap_pointer->find_feature_lab(lab);}


  // Selecting / deselecting features
 public:
  int select_feature(Feature *fp)
    {return current_submap_pointer->select_feature(fp);}
  int deselect_feature(Feature *fp)
    {return current_submap_pointer->deselect_feature(fp);}
  int toggle_feature(Identifier id)
    {return current_submap_pointer->toggle_feature(id);}
  int toggle_feature_lab(int lab)
    {return current_submap_pointer->toggle_feature_lab(lab);}

  // Auto-select feature
 public:
  int auto_select_feature()
    {return current_submap_pointer->auto_select_feature();}

  // Output info on current features
 public:
  int print_current_features()
    {return current_submap_pointer->print_current_features();}
  int print_selected_features()
    {return current_submap_pointer->print_selected_features();}
  int print_covariances()
    {return current_submap_pointer->print_covariances();}
  int print_robot_state()
    {return current_submap_pointer->print_robot_state();}
  int print_marked_feature_state()
    {return current_submap_pointer->print_marked_feature_state();}
  int print_whole_state()
    {return current_submap_pointer->print_whole_state();}
  int output_state_to_file(int counter, char *filename)
    {return current_submap_pointer->output_state_to_file(counter, filename);}
  int output_state_to_file(int counter)
    {return current_submap_pointer->output_state_to_file(counter);}
  int output_state_to_file()
    {return current_submap_pointer->output_state_to_file();}


  // Functions for doing stuff with the total state and covariance
 public:
  int construct_total_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M)
    {return current_submap_pointer->construct_total_state_and_covariance(V, M);}
  int fill_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M)
    {return current_submap_pointer->fill_state_and_covariance(V, M);}
  int construct_total_state(Hor_Matrix *V)
    {return current_submap_pointer->construct_total_state(V);}
  int construct_total_covariance(Hor_Matrix *M)
    {return current_submap_pointer->construct_total_covariance(M);}
  int fill_states(Hor_Matrix *V)
    {return current_submap_pointer->fill_states(V);}
  int fill_covariances(Hor_Matrix *M)
    {return current_submap_pointer->fill_covariances(M);}

  // Do bookkeeping when measurements are made
 public:
  int starting_measurements()
    {return current_submap_pointer->starting_measurements();}
  int failed_measurement_of_feature(Feature *fp)
    {return current_submap_pointer->failed_measurement_of_feature(fp);}
  int successful_measurement_of_feature(Feature *fp)
    {return current_submap_pointer->successful_measurement_of_feature(fp);}

  // Form matrices for measurement update
  int construct_total_measurement_stuff(Hor_Matrix *nu_tot, 
					Hor_Matrix *dh_by_dx_tot,
					Hor_Matrix *R)
    {return current_submap_pointer->construct_total_measurement_stuff(nu_tot,
						 dh_by_dx_tot, R);}
 public:
  int predict_measurements()
    {return current_submap_pointer->predict_measurements();}

 public:
  int test_for_visibility(Feature *fp)
    {return current_submap_pointer->test_for_visibility(fp);}

 public:
  int mark_feature_by_lab(int lab)
    {return current_submap_pointer->mark_feature_by_lab(lab);}
  int mark_first_feature()
    {return current_submap_pointer->mark_first_feature();}

 public:
  int predict_internal_measurement()
    {return current_submap_pointer->predict_internal_measurement();}
  Internal_Measurement_Model *get_internal_measurement_model()
    {return current_submap_pointer->get_internal_measurement_model();}
  
  Hor_Matrix *get_hv()
    {return current_submap_pointer->get_hv();}
  Hor_Matrix *get_zv()
    {return current_submap_pointer->get_zv();}
  Hor_Matrix *get_Sv()
    {return current_submap_pointer->get_Sv();}

  int successful_internal_measurement()
    {return current_submap_pointer->successful_internal_measurement();}
  int failed_internal_measurement()
    {return current_submap_pointer->failed_internal_measurement();}
  int get_successful_internal_measurement_flag()
    {return current_submap_pointer->get_successful_internal_measurement_flag();}

  int construct_total_internal_measurement_stuff(Hor_Matrix *nu_tot, 
						 Hor_Matrix *dh_by_dx_tot,
						 Hor_Matrix *R_tot)
    {return current_submap_pointer->construct_total_internal_measurement_stuff(
                     nu_tot, dh_by_dx_tot, R_tot);}


};
