/*  Scene: software for sequential localisation and map-building

    simul.cc
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

#include <general_headers.h>
#include "models_base.h"
#include "sim_or_rob.h"
#include "simul.h"

Simulation_Feature::Simulation_Feature(int lab, Feature_Measurement_Model *f_m_m,
				       Hor_Matrix *yi_passed)
  : feature_measurement_model(f_m_m)
{
  label = lab;
  yi = hor_mats_copy(yi_passed);
  next = NULL;
}

// Constructor
Simulation::Simulation(Motion_Model *m_m, Hor_Matrix *initial_xv)
  : Sim_Or_Rob("SIMULATION"), motion_model(m_m)
{
  assert(initial_xv->rows == motion_model->STATE_SIZE);
  xv = hor_mats_copy(initial_xv);

  total_state_size = motion_model->STATE_SIZE;

  no_features = 0;
  next_label = 0;
  first_feature = NULL;
  last_feature = NULL;

  /* Seed the random number generator */
  srand(time(NULL));
}

// Destructor
Simulation::~Simulation()
{
  hor_mat_free(xv);
}

/*******************************Measurements**********************************/

int Simulation::measure_feature(void *id, Hor_Matrix *z, 
				Hor_Matrix *h, Hor_Matrix *S)
{
  Simulation_Feature *sfp;

  for (sfp = first_feature; sfp; sfp = sfp->next)
    if ( (void *) &(sfp->label) == id)
      break;

  assert (sfp);
  assert (z->rows == sfp->feature_measurement_model->MEASUREMENT_SIZE && 
	  z->cols == 1);
  assert (h->rows == sfp->feature_measurement_model->MEASUREMENT_SIZE && 
	  h->cols == 1);
  assert (S->rows == sfp->feature_measurement_model->MEASUREMENT_SIZE &&
	  S->cols == sfp->feature_measurement_model->MEASUREMENT_SIZE);
  // h and S not used in the simulation here
  
  motion_model->func_xp(xv);
  sfp->feature_measurement_model->func_hi_noisy(sfp->yi, motion_model->xpRES);

  hor_matq_copy(sfp->feature_measurement_model->hi_noisyRES, z);

  return 0;
}

/*****************************Vehicle Movement********************************/

int Simulation::set_control(Hor_Matrix *u, double delta_t)
{
  motion_model->func_fv_noisy(xv, u, delta_t);
  
  hor_matq_copy(motion_model->fv_noisyRES, xv);

  return 0;
}

/* In simulation, continue_control does exactly the same thing as 
   set_control */
int Simulation::continue_control(Hor_Matrix *u, double delta_t)
{
  set_control(u, delta_t);

  return 0;
}

// Wait for end of motion and stop vehicle: in simulation these are 
// dummy functions
double Simulation::wait_for_end_of_motion(Hor_Matrix *u)
{
  return 0.0;
}

int Simulation::stop_vehicle()
{
  return 0;
}

void *Simulation::initialise_known_feature(Feature_Measurement_Model *f_m_m,
					   Hor_Matrix *yi,
					   int known_feature_label)
{
  int true_label = add_new_true_feature(f_m_m, yi);
  return (void *) find_identifier(true_label);
}

int Simulation::make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
				      	  Feature_Measurement_Model *f_m_m)
{
  measure_feature((int *) id, z, z, f_m_m->Temp_MM1);

  return 0;
}

// Make internal measurement according to internal measurement model
int Simulation::make_internal_measurement(Internal_Measurement_Model 
             *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
	     Hor_Matrix *Sv)
{
  assert (zv->rows == internal_measurement_model->MEASUREMENT_SIZE && 
	  zv->cols == 1);
  assert (hv->rows == internal_measurement_model->MEASUREMENT_SIZE && 
	  hv->cols == 1);
  assert (Sv->rows == internal_measurement_model->MEASUREMENT_SIZE &&
	  Sv->cols == internal_measurement_model->MEASUREMENT_SIZE);
  // h and S not used in the simulation here
  
  internal_measurement_model->func_hv_noisy(xv);

  hor_matq_copy(internal_measurement_model->hv_noisyRES, zv);

  return 0;
}


// Find the identifier of a feature with a certain label
// Identifier is just the pointer to this label
void *Simulation::find_identifier(int label)
{
  for (Simulation_Feature *sfp = first_feature; sfp; sfp = sfp->next)
    if (sfp->label == label)
      return (void *) &(sfp->label);
      
  cerr << "Error with Simulation::find_identifier()." << endl;
  return NULL;
}

// Find the feature measurement model associated with a certain feature
Feature_Measurement_Model *Simulation::find_feature_measurement_model(int label)
{
  for (Simulation_Feature *sfp = first_feature; sfp; sfp = sfp->next)
    if (sfp->label == label)
      return sfp->feature_measurement_model;
      
  cerr << "Error with Simulation::find_feature_measurement_model()." << endl;
  return NULL;
}

// Add a feature
int Simulation::add_new_true_feature(Feature_Measurement_Model *f_m_m,
				     Hor_Matrix *yi)
{
  Simulation_Feature *s = new Simulation_Feature(next_label, 
						 f_m_m, yi);

  if(first_feature == NULL)
    first_feature = s;
  else
    last_feature->next = s;
  
  last_feature = s;
  
  no_features++;
  total_state_size += f_m_m->FEATURE_STATE_SIZE;

  return next_label++;
}

/*****************************Output Functions********************************/

// Function to print out the true scene points
int Simulation::print_true_features()
{
  cout << "Number of features: " << no_features << endl;

  for (Simulation_Feature *sf = first_feature; sf; sf = sf->next)
    cout << "Feature " << sf->label << " has structure:" 
	 << sf->yi;

  return 0;
}

// Function to print out the vehicle position
int Simulation::print_true_vehicle()
{
  cout << "True vehicle state:" << xv;

  return 0;
}

/*******************************Zero Axes*************************************/

/* Zero the coordinate frame so that the vehicle is as the centre */
int Simulation::zero_axes()
{
  motion_model->func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(motion_model->xpRES);

  // Go through features
  for (Simulation_Feature *sfp = first_feature; sfp; sfp = sfp->next)
  { 
    sfp->feature_measurement_model->
        func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(sfp->get_yi(), 
								local_xp);
    hor_matq_copy(sfp->feature_measurement_model->zeroedyiRES, sfp->get_yi());
  }

  // Do robot state last
  motion_model->func_zeroedxv_and_dzeroedxv_by_dxv(get_xv());
  hor_matq_copy(motion_model->zeroedxvRES, get_xv());

  hor_mat_free_list(local_xp, NULL);

  return 0;
}

/******************Functions for Loading and Unloading State******************/

int Simulation::construct_total_true_state(Hor_Matrix *V)
{
  assert (V->rows == total_state_size && V->cols == 1);

  int y_position = 0;

  hor_matq_insert_chunky1(xv, V, y_position);
  y_position += motion_model->STATE_SIZE;

  int y_feature_no = 0;

  for (Simulation_Feature *f = first_feature; f; f = f->next)
  {
    hor_matq_insert_chunky1(f->yi, V, y_position);
    y_feature_no++;
    y_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (y_feature_no == no_features && y_position == total_state_size);

  return 0;
}

int Simulation::fill_true_state(Hor_Matrix *V)
{
  assert (V->rows == total_state_size && V->cols == 1);

  int y_position = 0;

  hor_matq_extract_chunky1(V, xv, y_position);
  y_position += motion_model->STATE_SIZE;

  int y_feature_no = 0;

  for (Simulation_Feature *f = first_feature; f; f = f->next)
  {
    hor_matq_extract_chunky1(V, f->yi, y_position);
    y_feature_no++;
    y_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (y_feature_no == no_features && y_position == total_state_size);

  return 0;
}
