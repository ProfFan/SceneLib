/*  Scene: software for sequential localisation and map-building

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

/* Kalman class contains the filtering functions. */

class Kalman {
 public:
  // Constants
  static const int DEFAULT_JGHK_AGE = 10;
  static const double DEFAULT_NEBOT_C1 = 0.0001;
  static const double DEFAULT_NEBOT_C2 = 0.001;

  // Public types
  enum filter_method {SLOW, FAST_FULL, JGHK, NEBOT, BAD_FILTER_METHOD};
  struct filter_params {
    filter_params () :
      jghk_age_limit (DEFAULT_JGHK_AGE),
      nebot_c1 (DEFAULT_NEBOT_C1),
      nebot_c2 (DEFAULT_NEBOT_C2),
      features_retained (0),
      filter_time (0),
      total_filter_time (0)
    {}
    filter_params (const filter_params& fp) :
      jghk_age_limit (fp.jghk_age_limit),
      nebot_c1 (fp.nebot_c1),
      nebot_c2 (fp.nebot_c2),
      features_retained (fp.features_retained),
      filter_time (fp.filter_time),
      total_filter_time (fp.total_filter_time)
    {}

    int jghk_age_limit;
    double nebot_c1;
    double nebot_c2;

    // Performance info
    int features_retained;
    double filter_time;
    double total_filter_time;
  };

  // Constructors and destructors
  Kalman ();                                           // Constructor
  Kalman (bool maintain_separate_state_in, Scene *start_scene);

  // Top level filters that filter either once or twice
  int predict_filter(Scene *scene, Hor_Matrix *u, double delta_t);
  int total_update_filter(Scene *scene);
  int update_filter_internal_measurement(Scene *scene, 
                      Internal_Measurement_Model *internal_measurement_model);

 private:
  // Intermediates that select filter method
  int predict_filter_choose (Scene *scene, Hor_Matrix *u, double delta_t,
			     filter_method how_to_filter, filter_params& params);
  int total_update_filter_choose (Scene *scene, filter_method how_to_filter,
				  filter_params& params);
  int update_filter_internal_measurement_choose (Scene *scene, 
                      Internal_Measurement_Model *internal_measurement_model,
		      filter_method how_to_filter, filter_params& params);

  // Slow versions (originals)
  int predict_filter_slow (Scene *scene, Hor_Matrix *u, double delta_t);
  int total_update_filter_slow (Scene *scene);
  int update_filter_internal_measurement_slow (Scene *scene, 
                      Internal_Measurement_Model *internal_measurement_model);

  // Fast versions added by jghk 17/9/00, generalised from old ajd code
  int predict_filter_fast (Scene *scene, Hor_Matrix *u, double delta_t,
			   filter_method how_to_filter, filter_params& params);
  int total_update_filter_fast (Scene *scene, filter_method how_to_filter,
				filter_params& params);
  //  int update_filter_internal_measurement_fast (Scene *scene, 
  //                      Internal_Measurement_Model *internal_measurement_model,
  //                      filter_method how_to_filter, filter_params& params);

 public:
  // Access
  int set_filter_method (bool which_filter, filter_method new_method);
  //   So that saved state and covariance can be viewed from outside
  void set_whether_to_maintain_separate_state (bool yn, Scene *start_scene) {
    maintain_separate_state = yn;
    initialise_separate_state (start_scene);
  }
  bool query_maintain_separate_state () {return maintain_separate_state;}
  int switch_to_saved_state (Scene *scene);

  int set_filter_params (bool which_filter, int jghk_age_limit,
			 double nebot_c1, double nebot_c2);
  filter_params get_filter_params (bool which_filter);

 private:

  // The methods used for filtering
  filter_method filter1;
  filter_method filter2;
  // Parameters for filtering;
  filter_params filter1_params;
  filter_params filter2_params;

  // Maintain separate state and covariance for testing purposes?
  bool maintain_separate_state;
  bool switch_filters1and2;
  Hor_Matrix *x_saved;
  Hor_Matrix *P_saved;

 private:
  // Utility functions
  int update_filter_slow_separate (Scene *scene, Hor_Matrix **px, Hor_Matrix **pP);
  Hor_Matrix *update_state (Hor_Matrix *state, Hor_Matrix *P1, Hor_Matrix *P2,
			    Hor_Matrix *dh_by_dxv, Hor_Matrix *dh_by_dy,
			    Hor_Matrix *Sinv, Hor_Matrix *nu,
			    bool transpose_P1 = false);
  Hor_Matrix *update_covariance (Hor_Matrix *Px, Hor_Matrix *P1,
				 Hor_Matrix *P2, Hor_Matrix *P3,
				 Hor_Matrix *P4, Hor_Matrix *A,
				 Hor_Matrix *B,  Hor_Matrix *C,
				 Hor_Matrix *D, Hor_Matrix *R,
				 bool transpose_P1 = false,
				 bool transpose_P3 = true);
  bool update_Pyy_nebot (Hor_Matrix *Pyy, Hor_Matrix *Pxy,
			 Hor_Matrix *Pyiyk, Hor_Matrix *Pykyi,
			 Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C,
			 Hor_Matrix *D, Hor_Matrix *R,
			 double c1, double c2);
  int change_covariances (Scene *scene, Hor_Matrix *Pxx_new);
  int copy_covariances_to_new (Scene *scene);
  void reincorporate_features (Scene *scene, filter_params params);
  void initialise_separate_state (Scene *scene);
  int switch_saved_for_current_state (Scene *scene);
};


