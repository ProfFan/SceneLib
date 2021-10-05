/*  SceneApp: applications for sequential localisation and map-building

    Camera/robot.h
    Copyright (C) 2001 Andrew Davison
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

// Robot class to interface with hardware
/* Horatio colour identifiers */
extern u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, 
  Cyan;

/**********************************Constants**********************************/

const int BOXSIZE = 15;              /* Side length of square image patches */

int write_patch();
int display_image();
int read_next_image();

/*****************************************************************************/

class Robot : public Sim_Or_Rob{
 public:
  Robot(FD_formrob *fd_fr);
  ~Robot();

  // Redefined virtual functions
  int measure_feature(void *id, Hor_Matrix *z, 
		      Hor_Matrix *h, Hor_Matrix *S);
  int set_control(Hor_Matrix *u, double delta_t);
  int continue_control(Hor_Matrix *u, double delta_t) {return 0;}
  double wait_for_end_of_motion(Hor_Matrix *u) {return 0.0;}

  int stop_vehicle() {return 0;}
  void *initialise_known_feature(Feature_Measurement_Model *f_m_m,
				 Hor_Matrix *yi,
				 int known_feature_label);

  int make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
				  Feature_Measurement_Model *f_m_m) {return 0;}


  // Make measurement from roll/pitch sensor
  // int make_internal_measurement(Internal_Measurement_Model 
  //            *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
  //	     Hor_Matrix *Sv);

 protected:

  // So we can access XForms
  FD_formrob *fd_formrob;

};


int canvas_button(FL_OBJECT *ob, Window win, int w, int h,
		  XEvent *xev, void *d);
int drawbox(int x1, int y1, int width, int height, unsigned long colour);
int canvas_expose(FL_OBJECT *ob, Window win, int w, int h,
		  XEvent *xev, void *d);
