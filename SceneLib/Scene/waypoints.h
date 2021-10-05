/*  Scene: software for sequential localisation and map-building

    waypoint.h
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

// Support for navigation waypoints: points along a route along which
// it is required for a robot to pass
// Basically just saves a list of vectors of state size

class Waypoints {
 public:
  // Constructor
  Waypoints(char *waypoints_file, Motion_Model *motion_model);

  // Setters and Getters
  int get_no_of_waypoints() const {return waypoints.size();}
  int get_current_waypoint() const {return current_waypoint;}
  void set_current_waypoint(int i) {current_waypoint = i;}

  Motion_Model *get_motion_model() {return motion_model;}

  Hor_Matrix *get_current_waypoint_vector() const 
     {return waypoints[current_waypoint];}
  Hor_Matrix *operator[](int i) const {return waypoints[i];}

  // Go to next waypoint, looping if neccesary
  void increment_current_waypoint();

  // Adding and deleting waypoints
  void delete_current_waypoint();
  void add_waypoint_before_current(Hor_Matrix *new_waypoint);
  void add_waypoint_after_current(Hor_Matrix *new_waypoint);

  // Zero axes: may be flawed to do this I think
  int zero_axes(Hor_Matrix *xv);

 protected:
  Motion_Model *motion_model;

  int current_waypoint;

  vector<Hor_Matrix *>waypoints;
};
