/*  Scene: software for sequential localisation and map-building

    postscript.cc
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

/* Functions to draw postscript representing the current state held in scene */
/* So far only works for point features */

#include <general_headers.h>
#include "models_base.h"
#include "sim_or_rob.h"
#include "simul.h"
#include "feature.h"
#include "scene_base.h"
#include "postscript.h"

Postscript::Postscript()
{
  for(int i=0; i < 8; i++)
    lab_vertex_ptrs[i] = hor_mat_alloc(3, 1);

  vehicle_vertex_ptrs[1] = hor_mats_fill(3, 1, -0.25, Hh, -0.25);
  vehicle_vertex_ptrs[2] = hor_mats_fill(3, 1, 0.25, Hh, -0.25);
  vehicle_vertex_ptrs[3] = hor_mats_fill(3, 1, -0.25, 0.0, -0.25);
  vehicle_vertex_ptrs[4] = hor_mats_fill(3, 1, 0.25, 0.0, -0.25);
  vehicle_vertex_ptrs[5] = hor_mats_fill(3, 1, -0.25, Hh, 0.25);
  vehicle_vertex_ptrs[6] = hor_mats_fill(3, 1, 0.25, Hh, 0.25);
  vehicle_vertex_ptrs[7] = hor_mats_fill(3, 1, -0.25, 0.0, 0.25);
  vehicle_vertex_ptrs[0] = hor_mats_fill(3, 1, 0.25, 0.0, 0.25);

  r = hor_mat_alloc(3, 1);
  R = hor_mat_alloc(3, 3);

  PP = hor_mat_alloc(3, 3);
  Temp31A = hor_mat_alloc(3, 1);
  Temp31B = hor_mat_alloc(3, 1);

  projection = hor_mats_fill(2, 3, 
			    -1.0, 0.0, 0.0,
			     0.0, 0.0, -1.0);
  projectionT = hor_mats_transpose(projection);
  p1_2D = hor_mat_alloc(2, 1);
  p2_2D = hor_mat_alloc(2, 1);
  PP_2D = hor_mat_alloc(2, 2);
  Temp23 = hor_mat_alloc(2, 3);

  MC21 = hor_mat_alloc(3, 3);
  MC10 = hor_mat_alloc(3, 3);
  MC20 = hor_mat_alloc(3, 3);
  Psimple = hor_mats_fill(2, 3,
			 -1.0,  0.0, 0.0,
			  0.0, -1.0, 0.0);

  // Set default values for these
  XSIZE = 160;
  YSIZE = 160;
  XOFFSET = 80.0;
  YOFFSET = 120.0;
  SCALE = 20.0;

  Z_AXIS_POS = 9.5;
  Z_AXIS_NEG = -2.0;
  X_AXIS_POS = 3.5;
  X_AXIS_NEG = -8.3;
  Y_AXIS_POS = 3.5;
  Y_AXIS_NEG = -1.5;
}


int Postscript::draw_estimates(Scene *scene)
{
  scene->get_motion_model()->func_xpose_and_Rpose(scene->get_xv());
  draw_robot(scene->get_motion_model()->xposeRES, 
	     scene->get_motion_model()->RposeRES, 
	     estimated_grey, estimated_linewidth);

  for (Feature *fp = scene->get_first_feature_ptr(); fp; fp = fp->get_next())
  {
    if (strcmp(
        fp->get_feature_measurement_model()->feature_dimensionality_type, 
        "POINT") == 0) {
    draw_point(fp->get_y(), estimated_grey, estimated_linewidth);
    draw_covariance(fp->get_y(), fp->get_Pyy(), 
		    estimated_grey, estimated_linewidth);
    }
    else
      cout << "Can't draw non-point feature." << endl;
  }

  return 0;
}

int Postscript::draw_true(Simulation *simulation)
{
  simulation->get_motion_model()->func_xpose_and_Rpose(simulation->get_xv());
  draw_robot(simulation->get_motion_model()->xposeRES, 
	     simulation->get_motion_model()->RposeRES, 
	     true_grey, true_linewidth);
  
  for (Simulation_Feature *sfp = simulation->get_first_feature(); sfp; 
       sfp = sfp->get_next()) {
    if (strcmp(
        sfp->get_feature_measurement_model()->feature_dimensionality_type, 
        "POINT") == 0) {
      draw_point(sfp->get_yi(), true_grey, true_linewidth);
    }
    else
      cout << "Can't draw non-point feature." << endl;
  }

  return 0;
}

int Postscript::draw_axes()
{
  Hor_Matrix *end1 = hor_mat_alloc(3, 1);
  Hor_Matrix *end2 = hor_mat_alloc(3, 1);

  /* Draw Axes */
  /* z axis */
  hor_matq_fill(end1, 0.0, 0.0, Z_AXIS_NEG);
  hor_matq_fill(end2, 0.0, 0.0, Z_AXIS_POS);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, 0.0, 0.0, Z_AXIS_POS);
  hor_matq_fill(end2, 0.1, 0.0, Z_AXIS_POS - 0.2);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, 0.0, 0.0, Z_AXIS_POS);
  hor_matq_fill(end2, -0.1, 0.0, Z_AXIS_POS - 0.2);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, 0.0, 0.0, Z_AXIS_POS);
  hor_matq_fill(end2, 0.0, 0.1, Z_AXIS_POS - 0.2);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, 0.0, 0.0, Z_AXIS_POS);
  hor_matq_fill(end2, 0.0, -0.1, Z_AXIS_POS - 0.2);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* x axis */
  hor_matq_fill(end1, X_AXIS_NEG, 0.0, 0.0);
  hor_matq_fill(end2, X_AXIS_POS, 0.0, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, X_AXIS_POS, 0.0, 0.0);
  hor_matq_fill(end2, X_AXIS_POS - 0.2, 0.0, 0.1);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, X_AXIS_POS, 0.0, 0.0);
  hor_matq_fill(end2, X_AXIS_POS - 0.2, 0.0, -0.1);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, X_AXIS_POS, 0.0, 0.0);
  hor_matq_fill(end2, X_AXIS_POS - 0.2, 0.1, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, X_AXIS_POS, 0.0, 0.0);
  hor_matq_fill(end2, X_AXIS_POS - 0.2, -0.1, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* y axis */
  hor_matq_fill(end1, 0.0, Y_AXIS_NEG, 0.0);
  hor_matq_fill(end2, 0.0, Y_AXIS_POS, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, 0.0, Y_AXIS_POS, 0.0);
  hor_matq_fill(end2, 0.1, Y_AXIS_POS - 0.2, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, 0.0, Y_AXIS_POS, 0.0);
  hor_matq_fill(end2, -0.1, Y_AXIS_POS - 0.2, 0.0);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  /* Draw little arrow */
  hor_matq_fill(end1, 0.0, Y_AXIS_POS, 0.0);
  hor_matq_fill(end2, 0.0, Y_AXIS_POS - 0.2, 0.1);
  draw_line(end1, end2, axis_grey, axis_linewidth);
  hor_matq_fill(end1, 0.0, Y_AXIS_POS, 0.0);
  hor_matq_fill(end2, 0.0, Y_AXIS_POS - 0.2, -0.1);
  draw_line(end1, end2, axis_grey, axis_linewidth);

  hor_mat_free_list(end1, end2, NULL);

  return 0;
}

/* Overloaded function output_postscript to draw: 
   just estimates or 
   estimates and ground truth or 
   just ground truth. */

int Postscript::output_postscript(Scene *scene)
{
  stream = hor_ps_open(ps_filename);
  hor_ps_init(XSIZE, YSIZE, stream);

  draw_estimates(scene);
  draw_axes();

  hor_ps_close(stream);

  return 0;
}

int Postscript::output_postscript(Scene *scene, Simulation *simulation)
{
  stream = hor_ps_open(ps_filename);
  hor_ps_init(XSIZE, YSIZE, stream);

  draw_estimates(scene);
  draw_true(simulation);
  draw_axes();

  hor_ps_close(stream);

  return 0;
}

int Postscript::output_postscript(Simulation *simulation)
{
  stream = hor_ps_open(ps_filename);
  hor_ps_init(XSIZE, YSIZE, stream);

  draw_true(simulation);
  draw_axes();

  hor_ps_close(stream);

  return 0;
}

/*****************************************************************************/

/* Special (i.e. hacked) functions to draw trajectory pictures */

int Postscript::output_postscript_and_leave_file_open(Scene *scene)
{
  stream = hor_ps_open(ps_filename);
  hor_ps_init(XSIZE, YSIZE, stream);

  draw_estimates(scene);
  draw_axes();

  return 0;
}

int Postscript::output_postscript_and_leave_file_open(Scene *scene, 
						      Simulation *simulation)
{
  stream = hor_ps_open(ps_filename);
  hor_ps_init(XSIZE, YSIZE, stream);

  draw_estimates(scene);
  draw_true(simulation);
  draw_axes();

  return 0;
}

int Postscript::close_file()
{
  hor_ps_close(stream);

  return 0;
}

int Postscript::just_draw_vehicle(Scene *scene, Simulation *simulation)
{
  scene->get_motion_model()->func_xpose_and_Rpose(scene->get_xv());
  draw_robot(scene->get_motion_model()->xposeRES, 
	     scene->get_motion_model()->RposeRES, 
	     estimated_grey, estimated_linewidth);
  
  scene->get_motion_model()->func_xpose_and_Rpose(simulation->get_xv());
  draw_robot(scene->get_motion_model()->xposeRES, 
	     scene->get_motion_model()->RposeRES, 
	     true_grey, true_linewidth);

  return 0;
}

int Postscript::just_draw_vehicle(Scene *scene)
{
  scene->get_motion_model()->func_xpose_and_Rpose(scene->get_xv());
  draw_robot(scene->get_motion_model()->xposeRES, 
	     scene->get_motion_model()->RposeRES, 
	     estimated_grey, estimated_linewidth);
  
  return 0;
}

/*****************************************************************************/

int Postscript::draw_robot(Hor_Matrix *rpose, Hor_Matrix *Rpose, 
			   double grey, double linewidth)
{
  hor_matq_copy(rpose, r);
  hor_matq_copy(Rpose, R);

  for(int i=0; i < 8; i++) 
  {
    hor_matq_prod2( R, vehicle_vertex_ptrs[i], lab_vertex_ptrs[i] );
    hor_matq_add2( r, lab_vertex_ptrs[i], lab_vertex_ptrs[i] );
  }

  /* 12 lines in vehicle */
  draw_line(lab_vertex_ptrs[1], lab_vertex_ptrs[2], grey, linewidth);
  draw_line(lab_vertex_ptrs[2], lab_vertex_ptrs[4], grey, linewidth);
  draw_line(lab_vertex_ptrs[4], lab_vertex_ptrs[3], grey, linewidth);
  draw_line(lab_vertex_ptrs[3], lab_vertex_ptrs[1], grey, linewidth);
  draw_line(lab_vertex_ptrs[2], lab_vertex_ptrs[6], grey, linewidth);
  draw_line(lab_vertex_ptrs[4], lab_vertex_ptrs[0], grey, linewidth);
  draw_line(lab_vertex_ptrs[3], lab_vertex_ptrs[7], grey, linewidth);
  draw_line(lab_vertex_ptrs[1], lab_vertex_ptrs[5], grey, linewidth);
  draw_line(lab_vertex_ptrs[5], lab_vertex_ptrs[6], grey, linewidth);
  draw_line(lab_vertex_ptrs[6], lab_vertex_ptrs[0], grey, linewidth);
  draw_line(lab_vertex_ptrs[0], lab_vertex_ptrs[7], grey, linewidth);
  draw_line(lab_vertex_ptrs[7], lab_vertex_ptrs[5], grey, linewidth);

  return 0;
}

int Postscript::draw_covariance(Hor_Matrix *point, Hor_Matrix *covariance,
				double grey, double linewidth)
{
  hor_matq_copy(covariance, PP);
  hor_matq_prod2(projection, point, p1_2D);
  hor_matq_prod3(projection, PP, projectionT, Temp23, PP_2D);

  // Draw an ellipse in postscript
  hor_matq_tred2(PP_2D, eigenvalues2, spare_array2);
  hor_matq_tqli(eigenvalues2, spare_array2, PP_2D);
    
  hor_matq_eigsrt(eigenvalues2, PP_2D);

  double angle = atan2(-matel(PP_2D, 2, 1), matel(PP_2D, 1, 1));

  hor_ps_setgray(grey, stream);
  hor_ps_setlinewidth(linewidth, stream);

  hor_ps_ellipse(SCALE * matel(p1_2D, 1, 1) + XOFFSET, 
		 SCALE * matel(p1_2D, 2, 1) + YOFFSET,
	   SCALE * NO_SIGMA_SHOW_POSTSCRIPT * sqrt(eigenvalues2[0]),
	   SCALE * NO_SIGMA_SHOW_POSTSCRIPT * sqrt(eigenvalues2[1]),
		 angle, stream);

  return 0;
}

/* These functions draw_point and draw_line will be used for actually
   drawing everything. The points arguments they take will be in 3D 
   coordinates, and they will do the projection to 2D and scaling.
   Will not draw a point if it is outside the diagram boundary.
*/
int Postscript::draw_point(Hor_Matrix *point, double grey, double linewidth)
{
  hor_ps_setgray(grey, stream);
  hor_ps_setlinewidth(linewidth, stream);

  hor_matq_prod2(projection, point, p1_2D);

  double x_to_plot = SCALE * matel(p1_2D, 1, 1) + XOFFSET;
  double y_to_plot = SCALE * matel(p1_2D, 2, 1) + YOFFSET;

  if (x_to_plot >= 0.0 && x_to_plot <= XSIZE &&
      y_to_plot >= 0.0 && y_to_plot <= YSIZE)
    hor_ps_circle(x_to_plot,
		  y_to_plot,
		  1.0, stream);

  return 0;
}

int Postscript::draw_line(Hor_Matrix *point1, Hor_Matrix *point2, double grey, 
			  double linewidth)
{
  hor_ps_setgray(grey, stream);
  hor_ps_setlinewidth(linewidth, stream);

  hor_matq_prod2(projection, point1, p1_2D);
  hor_matq_prod2(projection, point2, p2_2D);

  hor_ps_line(SCALE * matel(p1_2D, 1, 1) + XOFFSET,
	      SCALE * matel(p1_2D, 2, 1) + YOFFSET,
	      SCALE * matel(p2_2D, 1, 1) + XOFFSET,
	      SCALE * matel(p2_2D, 2, 1) + YOFFSET, stream);

  return 0;
}

/* Set the projection matrix based on "head" angles input */
int Postscript::set_projection(double alpha, double e)
{
  hor_matq_fill(MC21, 
		1.0, 0.0,     0.0,
		0.0, cos(e), -sin(e),
		0.0, sin(e),  cos(e));
  hor_matq_fill(MC10,
		cos(alpha), 0.0, -sin(alpha),
		0.0,        1.0,  0.0,
		sin(alpha), 0.0,  cos(alpha));
  hor_matq_prod2(MC21, MC10, MC20);
  hor_matq_prod2(Psimple, MC20, projection);
  hor_matq_transpose(projection, projectionT);

  return 0;
}

int Postscript::set_postscript_parameters(double xoffset, double yoffset, 
                                          double scale, 
                                          int xsize, int ysize)
{
  XSIZE = xsize;
  YSIZE = ysize;
  SCALE = scale;
  XOFFSET = xoffset;
  YOFFSET = yoffset;

  return 0;
}

int Postscript::print_current_parameters()
{
  cout << "Current values: " << XSIZE << " " << YSIZE << " " << SCALE << " " 
       << XOFFSET << " " << YOFFSET << endl;

  return 0;
}

int Postscript::set_axis_limits(double x_neg, double x_pos, 
				double y_neg, double y_pos, 
				double z_neg, double z_pos)
{
  X_AXIS_NEG = x_neg;
  X_AXIS_POS = x_pos;
  Y_AXIS_NEG = y_neg;
  Y_AXIS_POS = y_pos;
  Z_AXIS_NEG = z_neg;
  Z_AXIS_POS = z_pos;

  return 0;
}

int Postscript::print_current_axis_limits()
{
  cout << "Current values: " << X_AXIS_NEG << " " << X_AXIS_POS << " " 
       << Y_AXIS_NEG << " " << Y_AXIS_POS << " " << Z_AXIS_NEG << " " 
       << Z_AXIS_POS << endl;

  return 0;
}
