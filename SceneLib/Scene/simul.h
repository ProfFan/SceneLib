/*  Scene: software for sequential localisation and map-building

    simul.h
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

// Simulation Class

class Simulation_Feature { 
friend class Simulation;
 public:
  Simulation_Feature(int lab, Feature_Measurement_Model *f_m_m, 
		     Hor_Matrix *yi);

  Hor_Matrix *get_yi() {return yi;}
  int get_label() {return label;}
  Simulation_Feature *get_next() {return next;}
  Feature_Measurement_Model *get_feature_measurement_model() 
    {return feature_measurement_model;}

 protected:
  Feature_Measurement_Model *feature_measurement_model;
  Simulation_Feature *next;
  Hor_Matrix *yi;
  int label;
};


class Simulation : public Sim_Or_Rob {
public:
  Simulation(Motion_Model *m_m, Hor_Matrix *initial_xv);    
  ~Simulation();

  // Redefined virtual functions
  int measure_feature(void *id, Hor_Matrix *z, Hor_Matrix *h, Hor_Matrix *S);

  int set_control(Hor_Matrix *u, double delta_t);
  int continue_control(Hor_Matrix *u, double delta_t);
  double wait_for_end_of_motion(Hor_Matrix *u);
  int stop_vehicle();
  void *initialise_known_feature(Feature_Measurement_Model *f_m_m,
				 Hor_Matrix *yi,
				 int known_feature_label);
  int make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
					  Feature_Measurement_Model *f_m_m);

  int make_internal_measurement(Internal_Measurement_Model 
             *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
	     Hor_Matrix *Sv);

  int print_true_features();
  int print_true_vehicle();

  // Access functions
  Motion_Model *get_motion_model() {return motion_model;}
  int get_no_true_features() {return no_features;}
  Simulation_Feature *get_first_feature() {return first_feature;}
  Hor_Matrix *get_xv() {return xv;}

  void *find_identifier(int label);
  Feature_Measurement_Model *find_feature_measurement_model(int label);

  /* Functions so we can load and unload the true state if required */
  int construct_total_true_state(Hor_Matrix *V);
  int fill_true_state(Hor_Matrix *V);

  int add_new_true_feature(Feature_Measurement_Model *f_m_m, Hor_Matrix *yi);

  // Zero axes at current position
  int zero_axes();

protected:
  int total_state_size;
  Motion_Model *motion_model;
          
  Hor_Matrix *xv;  // Simulation "true" vehicle parameters
  Simulation_Feature *first_feature;
  Simulation_Feature *last_feature;
  int no_features;
  int next_label;                  // For allocating labels to true features
};
