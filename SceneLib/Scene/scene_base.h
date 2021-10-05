/*  Scene: software for sequential localisation and map-building

    scene_base.h
    Copyright (C) 2001 Andrew Davison
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

/** Base Class for Scene Hierarchy
   An interface to the derived classes: 
   this class contains only virtual functions.
*/

class Scene {
 public:
  /**@name General access functions */
  //@{
  /// Get pointer to motion model
  virtual Motion_Model *get_motion_model() = 0;
  /// Get pointer to internal measurement model
  virtual Internal_Measurement_Model *get_internal_measurement_model() = 0;
  /// Get pointer to robot state
  virtual Hor_Matrix *get_xv() = 0;
  /// Get pointer to robot covariance
  virtual Hor_Matrix *get_Pxx() = 0;
  /// Number of selected features
  virtual int get_no_selected() = 0;
  /// Total number of features
  virtual int get_no_features() = 0;
  /// Total size of state vector
  virtual int get_total_state_size() = 0;
  ///
  virtual int get_successful_measurement_vector_size() = 0;
  ///
  virtual void set_successful_measurement_vector_size(int smvs) = 0;
  ///
  virtual Feature *get_first_feature_ptr() = 0;
  ///
  virtual Feature *get_first_selected_ptr() = 0;
  ///
  virtual int get_marked_feature_label() = 0;
  ///
  virtual Hor_Matrix *get_hv() = 0;
  ///
  virtual Hor_Matrix *get_zv() = 0;
  ///
  virtual Hor_Matrix *get_Sv() = 0;
  //@}

  /**@name Bookkeeping */ 
  //@{
  /** Call this when the vehicle state is changed to potentially fire
      submap changes, etc. */
  virtual int vehicle_state_has_been_changed() {return 0;}
  //@}

  /**@name Accessing blocks of the covariance matrix */
  //@{ 
  ///
  virtual int get_Pyjyk (int j, int k, Hor_Matrix *Pyjyk) = 0;
  ///
  virtual Hor_Matrix *get_Pyjyk_ptr (int j, int k) = 0;
  //@}

  /**@name Adding / deleting features*/
  //@{
  ///
  virtual int add_new_feature(Identifier id, Hor_Matrix *y, 
		      Feature_Measurement_Model *f_m_m) = 0;
  ///
  virtual int add_new_known_feature(Identifier id, Hor_Matrix *y_known, 
			  Hor_Matrix *xp_o, Feature_Measurement_Model *f_m_m, 
				    int known_feature_label) = 0;
  ///
  virtual int delete_feature() = 0;
  ///
  virtual int exterminate_features() = 0;
  //@}

  /**@name Finding features from the list */
  //@{
  ///
  virtual Feature *find_feature(Identifier id) = 0;
  ///
  virtual Feature *find_feature_lab(int lab) = 0;
  //@}

  /**@name Selecting / deselecting features */
  //@{
  ///
  virtual int select_feature(Feature *fp) = 0;
  ///
  virtual int deselect_feature(Feature *fp) = 0;
  ///
  virtual int toggle_feature(Identifier id) = 0;
  ///
  virtual int toggle_feature_lab(int lab) = 0;
  //@}

  /**@name Auto-select feature */
  //@{
  ///
  virtual int auto_select_feature() = 0;
  //@}

  /**@name Output info on current features */
  //@{
  ///
  virtual int print_current_features() = 0;
  ///
  virtual int print_selected_features() = 0;
  ///
  virtual int print_covariances() = 0;
  ///
  virtual int print_robot_state() = 0;
  ///
  virtual int print_marked_feature_state() = 0;
  ///
  virtual int print_whole_state() = 0;
  ///
  virtual int output_state_to_file(int counter, char *filename) = 0;
  ///
  virtual int output_state_to_file(int counter) = 0;
  ///
  virtual int output_state_to_file() = 0;
  //@}

  /**@name Manipulating total state and covariance */
  //@{
  ///
  virtual int construct_total_state_and_covariance(Hor_Matrix *V, 
						   Hor_Matrix *P) = 0;
  ///
  virtual int fill_state_and_covariance(Hor_Matrix *V, Hor_Matrix *P) = 0;
  ///
  virtual int construct_total_state(Hor_Matrix *V) = 0;
  ///
  virtual int construct_total_covariance(Hor_Matrix *M) = 0;
  ///
  virtual int fill_states(Hor_Matrix *V) = 0;
  ///
  virtual int fill_covariances(Hor_Matrix *M) = 0;
  //@}

  /**@name Bookkeeping when measurements are made */
  //@{
  ///
  virtual int starting_measurements() = 0;
  ///
  virtual int failed_measurement_of_feature(Feature *fp) = 0;
  ///
  virtual int successful_measurement_of_feature(Feature *fp) = 0;
  //@}

  /**@name Form matrices for measurement update */
  //@{
  ///
  virtual int construct_total_measurement_stuff(Hor_Matrix *nu_tot, 
						Hor_Matrix *dh_by_dx_tot,
						Hor_Matrix *R) = 0;
  ///
  virtual int predict_single_feature_measurements (Feature *fp) = 0;
  /// This one is probably redundant
  virtual int predict_single_feature_measurements 
        (Feature_Measurement_Model *model, Hor_Matrix *y, 
	 Hor_Matrix *xp, Hor_Matrix *h, Hor_Matrix *dh_by_dy,
	 Hor_Matrix *dh_by_dxv, Hor_Matrix *R) = 0;
  virtual int predict_measurements() = 0;
  //@}

  /**@name Test a feature for measurability */
  //@{
  /** @return 0 if visible, > 0 otherwise */
  virtual int test_for_visibility(Feature *fp) = 0;
  //@}

  /**@name Mark feature
     For possible deletion or other manipulation */
  //@{
  /// Mark by integer label
  virtual int mark_feature_by_lab(int lab) = 0;
  /// Mark first feature in list
  virtual int mark_first_feature() = 0;
  //@}

  /**@name Internal Measurements */
  //@{
  /// Report a successful measurement
  virtual int successful_internal_measurement() = 0;
  /// Repost a failed measurement
  virtual int failed_internal_measurement() = 0;
  /// Test success
  virtual int get_successful_internal_measurement_flag() = 0;
  ///
  virtual int predict_internal_measurement() = 0;
  /// Get ready for an internal measurement
  virtual int construct_total_internal_measurement_stuff(Hor_Matrix *nu_tot, 
						   Hor_Matrix *dh_by_dx_tot,
						   Hor_Matrix *R_tot) = 0;
  //@}

  /**@name Zero axes 
     Redefine the coordinate frame at the current robot position */
  //@{
  ///
  virtual int zero_axes() = 0;
  //@}

  /**@name Normalise the state
     If there are any redundant representations, make sure that they are 
     in the right form and change the covariances accordingly */
  //@{
  ///
  virtual int normalise_state() = 0;
  //@}
};
