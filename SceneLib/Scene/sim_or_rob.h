/*  Scene: software for sequential localisation and map-building

    sim_or_rob.h
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

// Base Class for Simulation or Robot mode classes

class Sim_Or_Rob {
public:
  Sim_Or_Rob(char *s_o_r_type)
    : simulation_or_robot_type(s_o_r_type) 
    {}
  virtual ~Sim_Or_Rob() {}

  virtual int measure_feature(void *id, Hor_Matrix *z, Hor_Matrix *h, 
			      Hor_Matrix *S) = 0;
  virtual int set_control(Hor_Matrix *u, double delta_t) = 0;
  virtual int continue_control(Hor_Matrix *u, double delta_t) = 0;
  virtual double wait_for_end_of_motion(Hor_Matrix *u) = 0;
  virtual int stop_vehicle() = 0;
  virtual void *initialise_known_feature(Feature_Measurement_Model *f_m_m,
					 Hor_Matrix *yi,
					 int known_feature_label) = 0;
  virtual int make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
				  Feature_Measurement_Model *f_m_m) = 0;

  // Not pure virtual: NULL version defined if not redefined
  virtual int make_internal_measurement(Internal_Measurement_Model 
             *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
	     Hor_Matrix *Sv);

  char *simulation_or_robot_type;
};
