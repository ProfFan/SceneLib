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

// A replacement for the Robot Class for the Double case

class Double_Robot : public Robot {
 public:
  Double_Robot(FD_formrob *fd_fr, int zero_axes_flag, int reduced_im_flag,
	       Head_Point_Feature_Measurement_Model
	       *head_point_feature_measurement_model,
	       const double Ii, const double Hh,
	       int argc, char **argv);

  // Redefined Functions
  int set_control(Hor_Matrix *u, double delta_t);

  // Control lighting unit
  int set_light_control(Hor_Matrix *light_table);

  //  int continue_control(Hor_Matrix *u, double delta_t);
  double wait_for_end_of_motion(Hor_Matrix *u);
  int stop_vehicle();

  // For making internal measurements
  int make_internal_measurement(Internal_Measurement_Model 
				*internal_measurement_model, Hor_Matrix *z, 
				Hor_Matrix *h, Hor_Matrix *S);
 private:
  void *marker_identifier;
};
