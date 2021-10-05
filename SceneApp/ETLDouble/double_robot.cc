/*  SceneApp: applications for sequential localisation and map-building

    double_robot.cc
    Copyright (C) 2000 Andrew Davison and Nobuyuki Kita
    ajd@robots.ox.ac.uk
    nkita@etl.go.jp
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

#include "general_headers.h"
#include "forms.h"
#include "module.h"
extern "C" {
#include "rb.h"  // For SHM communications
}
#include "formrob.h"

#include "models_base.h"
#include "models_head.h"

#include "sim_or_rob.h"
#include "robot.h"
#include "double_robot.h"

void entryCommands(){}

char *marker_patch_file = "marker_patch.mit";

Double_Robot::Double_Robot(FD_formrob *fd_fr, int zero_axes_flag, 
			   int reduced_im_flag, 
			   Head_Point_Feature_Measurement_Model
			   *head_point_feature_measurement_model,
			   const double Ii, const double Hh,
			   int argc, char **argv)
  : Robot(fd_fr, zero_axes_flag, reduced_im_flag, 
	  head_point_feature_measurement_model, Ii, Hh, argc, argv)
{
  state_shm = (struct State_Vector *)
              map_share_mem(SHM_PORT,sizeof(struct State_Vector), 0);

  marker_identifier = (void *) hor_read_image(marker_patch_file);
  if (marker_identifier == NULL)
    cerr << "Unable to read marker patch file " << marker_patch_file 
	 << "; inter-robot measurement disabled. " << endl;
}

int Double_Robot::set_control(Hor_Matrix *u, double delta_t)
{
  // First Robot
  v_control = matel(u, 1, 1);                     // In metres
  dS_control = matel(u, 2, 1) * 180.0 / M_PI;     // In degrees
  dT_control = matel(u, 3, 1) * 180.0 / M_PI;     // In degrees

  zero_nomadic_state_vector();
  required_distance = fabs(matel(u, 1, 1) * delta_t);

  increment_vehicle_steering_angle(dS_control);
  increment_vehicle_turret_angle(dT_control);
  set_vehicle_velocity(v_control);

  // Second Robot
  state_shm->speed = matel(u, 4, 1);                       // In metres
  state_shm->steering = matel(u, 5, 1) * 180.0 / M_PI;     // In degrees
  state_shm->turret = matel(u, 6, 1) * 180.0 / M_PI;       // In degrees
  state_shm->duration = delta_t;                           // In seconds

  // Lighting controls: nothing for now
  state_shm->pan = 0.0; 
  state_shm->tilt = 0.0;
  state_shm->height = 0.0;
  state_shm->sw = 0;

  state_shm->execute = 1;                                  // Go!

  return 0;
}

int Double_Robot::set_light_control(Hor_Matrix *light)
{
  // First Robot : No action

  // Second Robot : No motion
  state_shm->speed = 0.0;   
  state_shm->steering = 0.0;
  state_shm->turret = 0.0;  
  state_shm->duration = 0.0;

  // Lighting controls: controlled by lighting table
  state_shm->pan = matel(light, 1, 1); 
  state_shm->tilt = matel(light, 2, 1);
  state_shm->height = matel(light, 3, 1);
  state_shm->sw = (int)matel(light, 4, 1);

  state_shm->execute = 1;                                  // Go!

  return 0;
}


double Double_Robot::wait_for_end_of_motion(Hor_Matrix *u)
{
  // First Robot
  do
  {
    current_distance = get_distance_from_odometry();
  }
  while (current_distance < required_distance);

  return 0.0;
}

int Double_Robot::stop_vehicle()
{
  // First Robot
  set_vehicle_velocity(0.0);
  
  // Also wait until steering and turret have stopped
  wait_for_nomad_stop(); 

  // Wait for Second Robot as well
  while (state_shm->finished == 0);

  return 0;
}


int Double_Robot::make_internal_measurement(Internal_Measurement_Model 
                           *internal_measurement_model, Hor_Matrix *z, 
			   Hor_Matrix *h, Hor_Matrix *S)
{
  if (marker_identifier == NULL)
  {
    cerr << "No marker patch defined: inter-robot measurement disabled." 
	 << endl;
    return -1;
  }

  if (strcmp(internal_measurement_model->internal_type, "DOUBLE_MARKER") == 0)
    return measure_feature(marker_identifier, z, h, S);
  else
  {
    cerr << "Error in Double_Robot::make_internal_measurement" << endl;
    return -1;
  }
}
  
