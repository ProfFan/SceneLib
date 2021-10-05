/*  Scene: software for sequential localisation and map-building

    postscript.h
    Copyright (C) 2001 Andrew Davison
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

const double estimated_grey = 0.7;
const double estimated_linewidth = 1.0; 
const double true_grey = 0.0;
const double true_linewidth = 1.0;
const double axis_grey = 0.0;
const double axis_linewidth = 1.1;

const double Hh = 1.3; // Height of robot
    
const double NO_SIGMA_SHOW_POSTSCRIPT = 3.0; // Standard deviations to show
                                             // in covariance ellipses

char *const ps_filename = "ps_scene.ps";

class Postscript {
public:
  Postscript();
  int output_postscript(Scene *scene, Simulation *simulation);
  int output_postscript(Scene *scene);
  int output_postscript(Simulation *simulation);

  /* For trajectory picture */
  int Postscript::output_postscript_and_leave_file_open(Scene *scene, 
						       Simulation *simulation);
  int output_postscript_and_leave_file_open(Scene *scene);
  int close_file();
  int just_draw_vehicle(Scene *scene, Simulation *simulation);
  int Postscript::just_draw_vehicle(Scene *scene);



  int set_projection(double alpha, double e);
  int set_postscript_parameters(double xoffset, double yoffset, 
				double scale, int xsize, int ysize);
  int print_current_parameters();
  int set_axis_limits(double x_neg, double x_pos, 
		      double y_neg, double y_pos, 
		      double z_neg, double z_pos);
  int print_current_axis_limits();
private:
  int XSIZE;
  int YSIZE;
  double XOFFSET;
  double YOFFSET;
  double SCALE;

  double X_AXIS_NEG, X_AXIS_POS, Y_AXIS_NEG, Y_AXIS_POS, 
         Z_AXIS_NEG, Z_AXIS_POS;

  FILE *stream;

  int draw_estimates(Scene *scene);
  int draw_true(Simulation *simulation);
  int draw_axes();
  int draw_robot(Hor_Matrix *rpose, Hor_Matrix *Rpose, 
		 double grey, double linewidth);
  int draw_covariance(Hor_Matrix *point, Hor_Matrix *covariance, 
		      double grey, double linewidth);
  int draw_point(Hor_Matrix *point, double grey, double linewidth);
  int draw_line(Hor_Matrix *point1, Hor_Matrix *point2, double grey, 
		double linewidth);

  // For drawing the vehicle
  Hor_Matrix *vehicle_vertex_ptrs[8];
  Hor_Matrix *lab_vertex_ptrs[8];

  Hor_Matrix *r, *R;

  // Stuff for drawing covariances
  int j;
  double axis_length;

  Hor_Matrix *PP;
  Hor_Matrix *Temp31A, *Temp31B;

  double eigenvalues[3];
  double spare_array[3];

  double eigenvalues2[2];
  double spare_array2[2];

  Hor_Matrix *projection, *projectionT;
  Hor_Matrix *Psimple;
  Hor_Matrix *p1_2D, *p2_2D, *PP_2D, *Temp23;
  Hor_Matrix *MC21, *MC10, *MC20;
};

