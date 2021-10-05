/*  SceneApp: applications for sequential localisation and map-building

    Camera/robot.cc
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

#include <general_headers.h>

#include "forms.h"
#include "formrob.h"

#include "improc.h"
#include "correlate.h"

#include "models_base.h"
#include "sim_or_rob.h"
#include "robot.h"

// Display stuff
Display *display_;

Hor_Image *image;

// XImages for displaying
XImage *ximage;
GC gc_;
Window wtwin_;

XColor black, red, green, blue, white;
int depth, bpp, screen;

// Postions in images
int uu, vv;

char *image_basename = "Sequence/test";
int current_image_number = 0;

const int image_size_x = 320;
const int image_size_y = 240;

/*******************************Robot Class***********************************/

// Constructor

Robot::Robot(FD_formrob *fd_fr)
  : Sim_Or_Rob("ROBOT"),
    fd_formrob(fd_fr)
{
  fl_add_canvas_handler(fd_formrob->canvas, ButtonPress, 
			canvas_button, NULL);
  fl_add_canvas_handler(fd_formrob->canvas, Expose, canvas_expose, NULL);

  display_ = fl_get_display();
  wtwin_ = FL_ObjWin(fd_formrob->canvas);

  XGCValues gc_values;
  gc_values.graphics_exposures = False;
  gc_ = XCreateGC(display_, wtwin_, GCGraphicsExposures, &gc_values);  
  screen = DefaultScreen(display_);
  depth = DefaultDepth(display_, screen);

  cerr << "Screen depth = " << depth << endl;

  read_next_image();
  display_image();
}

Robot::~Robot()
{

}

char *known_point_patch_stem = "known_patch";

int Robot::set_control(Hor_Matrix *u, double delta_t)
{
  read_next_image();

  display_image();

  return 0;
}

int Robot::measure_feature(void *id, Hor_Matrix *z, 
		      Hor_Matrix *h, Hor_Matrix *S)
{
  Hor_Matrix *Sinv = hor_mats_inv(S);

  xor_elliptical_search_region(image, Sinv, 
			      vecel(h, 1), vecel(h, 2), BOXSIZE);
  display_image();
  xor_elliptical_search_region(image, Sinv, 
			      vecel(h, 1), vecel(h, 2), BOXSIZE);

  int u_found, v_found;

  if (elliptical_search(image, (Hor_Image *) id, 
			Sinv, &u_found, &v_found, vecel(h, 1), vecel(h, 2), 
			BOXSIZE) != 0)
  {
    cout << "Feature not successfully matched." << endl;
    return -1;
  }

  // Display patch in left image
  drawbox( u_found - (BOXSIZE - 1) / 2, v_found - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Red );

  hor_matq_fill(z, (double) u_found, (double) v_found);

  hor_mat_free(Sinv);

  return 0;
}

// Initialise known feature
void *Robot::initialise_known_feature(Feature_Measurement_Model *f_m_m,
			       Hor_Matrix *yi,
			       int known_feature_label)
{
  // Get identifying feature patch from file
  // So far only handle points
  if(strcmp("CAMERA_POINT", f_m_m->feature_type) != 0)
  {
    cerr << "Known point not a CAMERA_POINT: passing." << endl;
    return NULL;
  }

  char name[100];

  strcpy(name, known_point_patch_stem);
  sprintf(name, "%s%d.pgm", known_point_patch_stem, known_feature_label);

  return (void *) hor_read_pgm_image(name);
}

// Little service function to copy image region centred at uu, vv into patch
void copy_into_patch(Hor_Image *im, Hor_Image *patch, int uu, int vv)
{
  for(int r = 0; r < BOXSIZE; r++)        
    for(int c = 0; c < BOXSIZE; c++)
      patch->array.uc[r][c] = 
	    im->array.uc[r + vv - (BOXSIZE - 1) / 2][c + uu - (BOXSIZE - 1) / 2];
}

/*******************************Display Images********************************/

// Header for horatio function which isn't normally called externally
extern "C" {
XImage *hor_convert_image_to_X_format ( Hor_Image *imptr,
				        Display   *display,
				        int        scrnum,
				        int        depth,
				        int        hor_subsample_c,
				        int        hor_subsample_r,
				        int        pixel_size_factor,
				        va_list   *aptr );



}

int display_image()
{
  ximage = hor_convert_image_to_X_format ( image,
					    display_,
					    screen,
					    depth,
					    1,
					    1,
					    1,
					    NULL);

  XPutImage(display_, wtwin_, gc_, ximage,
	    0, 0, 0, 0, image_size_x, image_size_y);

  XSync(display_, 0);

  return 0;
}

/**************************Write Image Patch to Disk**************************/

int write_patch()
{
  Hor_Image *hip = hor_alloc_image(BOXSIZE, BOXSIZE, HOR_U_CHAR, NULL);

  if (uu >= 0) {
    /* Copy the selected patch to the save space patch */
    copy_into_patch(image, hip, uu, vv);
    hor_write_pgm_image("patch.pgm", hip);
    cout << "Written patch patch.pgm" << endl;
  }
  else {
    cout << "No patch selected." << endl;
  }

  hor_free_image(hip);
  return 0;
}

/****************************Read Images from Disk****************************/

int read_next_image()
{
  if (image != NULL)
    hor_free_image(image);

  static char image_name[100];

  sprintf(image_name, "%s%03d.pgm", image_basename, current_image_number);

  if(!(image = hor_read_pgm_image(image_name)))
    cerr << "Failed to read image " << image_name << endl;
  else
    cout << "Read image " << image_name  << endl;

  current_image_number++;

  return 0;
}

void read_next_image(FL_OBJECT *ob, long data)
{
  read_next_image();
  display_image();
}

void write_patch(FL_OBJECT *ob, long data)
{
  write_patch();
}

/***************************Non-Member Functions******************************/

int drawbox(int x1, int y1, int width, int height, unsigned long colour)
{
  XSetForeground(display_, gc_, colour);
  XSetLineAttributes(display_, gc_, 1, LineSolid, CapButt, 
		     JoinMiter);
  XDrawLine(display_, wtwin_, gc_, x1, y1, x1 + width, y1);
  XDrawLine(display_, wtwin_, gc_, x1 + width, y1, 
	    x1 + width, y1 + height);
  XDrawLine(display_, wtwin_, gc_, x1 + width, y1 + height, 
	    x1, y1 + height);
  XDrawLine(display_, wtwin_, gc_, x1, y1 + height, x1, y1);
  XFlush(display_);

  return 0;
}

int canvas_button(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  // If button is pressed in image, select region
  int x = xev->xbutton.x, y = xev->xbutton.y;

  // First make sure selected box is within image
  if (y < (BOXSIZE - 1)/2)
    y = (BOXSIZE - 1)/2;
  if (y > image_size_y - (BOXSIZE - 1)/2)
    y = image_size_y - (BOXSIZE - 1)/2;

  if (x < (BOXSIZE - 1)/2)
    x = (BOXSIZE - 1)/2;
  if (x > image_size_x - (BOXSIZE - 1)/2)
    x = image_size_x - (BOXSIZE - 1)/2;

  uu = x;
  vv = y;

  cout << "Selected box centred at " << uu << ", " << vv 
       << "." << endl;

  drawbox( uu - (BOXSIZE - 1) / 2, vv - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Cyan);

  return 0;
}

int canvas_expose(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  display_image();
  return 0;
}

