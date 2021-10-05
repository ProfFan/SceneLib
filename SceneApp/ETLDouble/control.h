/*  SceneApp: applications for sequential localisation and map-building
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
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

// Headers for control.cc

// Functions called from xforms don't need headers here because it 
// provides its own header files

// Also, functions called from 3D tool included in 3d.h

// Called from main.cc
void pass_fd_pointer_com(FD_formcom *fd_fc);
#ifdef _ROBOT_
void pass_fd_pointer_rob(FD_formrob *fd_fr);
#else
void pass_fd_pointer_sim(FD_formsim *fd_fs);
#endif

int set_up_control();
int main_control_loop();
int set_all_vehicle_parameters_from_sliders();
