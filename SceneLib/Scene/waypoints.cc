/*  Scene: software for sequential localisation and map-building

    waypoint.cc
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

#include "general_headers.h"
#include "models_base.h"
#include "models_double.h"
#include "waypoints.h"

#include <algorithm>

// Waypoints format now normal state rather than position state
Waypoints::Waypoints(char *waypoints_file, Motion_Model *m_m)
  : motion_model(m_m)
{
  current_waypoint = 0;

  ifstream infile(waypoints_file, ios::in);
  // if (infile == NULL)
  // {
  //   cout << "No file " << waypoints_file 
	//  << " --- no initial waypoints." << endl;
  //   return;
  // }

  // Format waypoints file: 
  // MOTION_MODEL_TYPE
  // xv(1)
  // xv(2)
  // ...
  char type_buf[100];

  // Special format for DOUBLE robot motion model
  // DOUBLE
  // MOTION_MODEL1_TYPE
  // MOTION_MODEL2_TYPE
  // xv1(1) xv2(1)
  // xv1(2) xv2(2)
  // ...
  if(strcmp("DOUBLE", motion_model->motion_model_type) == 0) {
    Double_Motion_Model *double_motion_model = 
                   (Double_Motion_Model *) motion_model;
    infile >> type_buf;
    if(strcmp(type_buf, double_motion_model->motion_model_type) != 0) {
      cerr << "Mismatched motion model type: aborting." << type_buf << ", " << double_motion_model->motion_model_type << endl;
      return;
    }
    infile >> type_buf;
    if(strcmp(type_buf, double_motion_model->motion_model1->
	      motion_model_type) != 0) {
      cerr << "Mismatched motion model 1: aborting." << endl;
      return;
    }
    infile >> type_buf;
    if(strcmp(type_buf, double_motion_model->motion_model2->
	      motion_model_type) != 0) {
      cerr << "Mismatched motion model 2: aborting." << endl;
      return;
    }
  }
  else {
    infile >> type_buf;
    if(strcmp(type_buf, motion_model->motion_model_type) != 0) {
      cerr << "Mismatched motion model: aborting."  << type_buf << ", " << motion_model->motion_model_type << endl;
      return;
    }
  }

  while (infile) {
    Hor_Matrix *new_waypoint =
      hor_mat_alloc(motion_model->STATE_SIZE, 1);

    for (int i = 0; i < motion_model->STATE_SIZE; i++)
      infile >> matel(new_waypoint, i + 1, 1);
    if (!infile) break;

    waypoints.push_back(new_waypoint);
  }

  cout << "Read " << get_no_of_waypoints() << " waypoints." << endl;
}

void Waypoints::increment_current_waypoint()
{
  current_waypoint++;
  if (current_waypoint >= get_no_of_waypoints())
  {
    cout << "Reached last waypoint." << endl
	 << "Resetting to first waypoint." << endl;
    current_waypoint = 0;
  }
}

void Waypoints::delete_current_waypoint()
{
  hor_mat_free(waypoints[current_waypoint]);

  waypoints.erase(find(waypoints.begin(), waypoints.end(), &(waypoints[current_waypoint])));

  if (current_waypoint > get_no_of_waypoints() - 1)
    current_waypoint = get_no_of_waypoints() -1;
  if (current_waypoint < 0)
    current_waypoint = 0;

  return;
}

void Waypoints::add_waypoint_before_current(Hor_Matrix *new_waypoint)
{
  waypoints.insert(find(waypoints.begin(), waypoints.end(), &(waypoints[current_waypoint])), new_waypoint);
}

void Waypoints::add_waypoint_after_current(Hor_Matrix *new_waypoint)
{
  if (get_no_of_waypoints() == 0)
    current_waypoint = -1;

  waypoints.insert(find(waypoints.begin(), waypoints.end(), &(waypoints[current_waypoint + 1])), new_waypoint);

  current_waypoint++;
}

int Waypoints::zero_axes(Hor_Matrix *xv)
{
  motion_model->func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(motion_model->xpRES);

  for (unsigned int i = 0; i < waypoints.size(); i++) {
    motion_model->func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef
      (waypoints[i], local_xp);

    hor_matq_copy(motion_model->xvredefRES, waypoints[i]);
  }

  hor_mat_free_list(local_xp, 0);

  return 0;
}
