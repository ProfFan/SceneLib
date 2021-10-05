/*  SceneApp: applications for sequential localisation and map-building

    Camera/grab.cc
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

/***********Functions related to image grabbing, displaying, saving***********/

#include <general_headers.h>

#include "module.h"

#include "forms.h"
#include "formrob.h"
#include "grab.h"

/******************Static variables for the whole system**********************/

/* Flag which keeps track of whether the meteor is open */
char meteor_flag = CLOSED;

/****************************Global X Stuff***********************************/

FD_formrob *fd_formrob;
static Display *display;

/******************************Image Dimensions*******************************/

/* In stereo, it is an array of u_short */
int grab_size_x, grab_size_y;

/* image_size_x and image_size_y are the dimensions of the images 
   displayed and saved */
int image_size_x = MAXCOLS / 2, image_size_y = MAXROWS / 2;

/***************************Image Transformation******************************/

// With the ETL lenses, we need to unwarp to perspective images 
// according to a lookup-table loaded from file. This is done if
// unwarp_flag is set to 1. In init_grabbing its value is copied
// from UNWARP_FLAG in grab.h
// USE UNWARP_FLAG = 0 FOR NORMAL LENSES!!!
int unwarp_flag;
char *lookup_filename = "table"; // Lookup table filename

// Where we store the lookup tables for unwarping
int **array_uinL, **array_vinL, **array_uinR, **array_vinR;

/**********************************Images*************************************/

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

/* Horatio colour identifiers */
extern u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, 
  Cyan;

// Grab into here (SHORT image which is interleaved stereo)
unsigned short **grab_array;

// Images for processing
Hor_Image *hor_imageL, *hor_imageR;

// XImages for displaying
XImage *ximageL, *ximageR;
GC gc_L, gc_R;
Window wtwin_L, wtwin_R;

XColor black, red, green, blue, white;
int depth, bpp, screen;

// Pointers to postions in images
int *uLptr, *vLptr, *uRptr, *vRptr;

int BOXSIZE;

/***********************************Timing************************************/

// For checking out speed of display and stuff
struct timeval tod1, tod2;
double time1, time2;

/****************************Draw lines on images*****************************/

// Left
void drawline(int x1, int y1, int x2, int y2, unsigned long colour, 
	      GC gc_, Window wtwin_)
{
    XSetForeground(display, gc_, colour);
    XSetLineAttributes(display, gc_, 1, LineSolid, CapButt, 
		       JoinMiter);
    XDrawLine(display, wtwin_, gc_, x1, y1, x2, y2);
    XFlush(display);
    return;
}
int drawlineL(int x1, int y1, int x2, int y2, unsigned long colour)
{
  drawline(x1, y1, x2, y2, colour, gc_L, wtwin_L);
  return 0;
}
int drawlineR(int x1, int y1, int x2, int y2, unsigned long colour)
{
  drawline(x1, y1, x2, y2, colour, gc_R, wtwin_R);
  return 0;
}

void drawbox(int x1, int y1, int width, int height,
	      unsigned long colour, GC gc_, Window wtwin_)
{
    XSetForeground(display, gc_, colour);
    XSetLineAttributes(display, gc_, 1, LineSolid, CapButt, 
		       JoinMiter);
    XDrawLine(display, wtwin_, gc_, x1, y1, x1 + width, y1);
    XDrawLine(display, wtwin_, gc_, x1 + width, y1, 
	      x1 + width, y1 + height);
    XDrawLine(display, wtwin_, gc_, x1 + width, y1 + height, 
	      x1, y1 + height);
    XDrawLine(display, wtwin_, gc_, x1, y1 + height, x1, y1);
    XFlush(display);
    return;
}
int drawboxL(int x1, int y1, int width, int height, unsigned long colour)
{
  drawbox(x1, y1, width, height, colour, gc_L, wtwin_L);
  return 0;
}
int drawboxR(int x1, int y1, int width, int height, unsigned long colour)
{
  drawbox(x1, y1, width, height, colour, gc_R, wtwin_R);
  return 0;
}

/********************************Canvas Events********************************/

int canvas_button(XEvent *xev, int *this_x, int *this_y, 
		  int *other_x, int *other_y, char *which)
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

  *this_x = x;
  *this_y = y;
  *other_x = -1;
  *other_y = -1;

  cout << "Selected box centred at " << x << ", " << y 
       << " in " << which << " image." << endl;
  
  return 0;
}

int canvas_buttonL(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  canvas_button(xev, uLptr, vLptr, uRptr, vRptr, "left");
  drawboxL( *uLptr - (BOXSIZE - 1) / 2, *vLptr - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Cyan);

  return 0;
}

int canvas_buttonR(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  canvas_button(xev, uRptr, vRptr, uLptr, vLptr, "right");
  drawboxR( *uRptr - (BOXSIZE - 1) / 2, *vRptr - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Cyan);

  return 0;
}

int canvas_exposeL(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  g_display_images();
  return 0;
}

int canvas_exposeR(FL_OBJECT *ob, Window win, int w, int h,
		   XEvent *xev, void *d)
{
  g_display_images();
  return 0;
}

/**********************Display Images on XForms Canvases**********************/

int g_display_images()
{
  gettimeofday(&tod1, NULL);
  time1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
  
  ximageL = hor_convert_image_to_X_format ( hor_imageL,
					    display,
					    screen,
					    depth,
					    1,
					    1,
					    1,
					    NULL);

  ximageR = hor_convert_image_to_X_format ( hor_imageR,
					    display,
					    screen,
					    depth,
					    1,
					    1,
					    1,
					    NULL);

  XPutImage(display, wtwin_L, gc_L, ximageL,
	    0, 0, 0, 0, image_size_x, image_size_y);
  XPutImage(display, wtwin_R, gc_R, ximageR,
	    0, 0, 0, 0, image_size_x, image_size_y);

  XSync(display, 0);

  gettimeofday(&tod2, NULL);
  time2 = tod2.tv_sec + tod2.tv_usec / 1000000.0;

  //  cerr << "Time for display = " << time2 - time1 << " sec." << endl;

  return 0;
}	    

/****************************Write Images to Disk*****************************/

int g_write_images()
{
  hor_write_image("outL", hor_imageL);
  hor_write_image("outR", hor_imageR);

  cout << "Written images outL, outR." << endl;

  return 0;
}	    

/****************************Read Images from Disk****************************/

int g_read_images()
{
  char *left_name = "inL";
  char *right_name = "inR";

  // Test them first
  Hor_Image *im_test = hor_read_image(left_name);
  if (im_test != NULL && im_test->width == image_size_x &&
      im_test->height == image_size_y && im_test->type == HOR_U_CHAR)
  {
    memcpy(hor_imageL->array.uc[0], im_test->array.uc[0], 
	   image_size_x * image_size_y);
  }
  else
    cerr << "Problem reading left image " << left_name << endl 
	 << "Not found, or image type or size wrong." << endl;
  hor_free_image(im_test);
    
  im_test = hor_read_image(right_name);
  if (im_test != NULL && im_test->width == image_size_x &&
      im_test->height == image_size_y && im_test->type == HOR_U_CHAR)
  {
    memcpy(hor_imageR->array.uc[0], im_test->array.uc[0], 
	   image_size_x * image_size_y);
  }
  else
    cerr << "Problem reading right image " << right_name << endl 
	 << "Not found, or image type or size wrong." << endl;
  hor_free_image(im_test);

  g_display_images();
  return 0;
}

/*******************Copy from grabbed buffer into 2 images********************/

// Inefficient like this, but what we do is copy into both
// Hor_Images (for processing) and XForms images for displaying
// Do both because XForms images are short int rather than byte
int copy_images()
{
  u_char *hor_im_baseL = hor_imageL->array.uc[0];
  u_char *hor_im_baseR = hor_imageR->array.uc[0];

  // Do the copy backwards (faster)
  u_char *grab_im_ptr = (u_char *)(grab_array[0] + 
				   grab_size_x * grab_size_y - 2);
  u_char *hor_im_ptrL = hor_im_baseL + image_size_x * image_size_y - 1;
  u_char *hor_im_ptrR = hor_im_baseR + image_size_x * image_size_y - 1;

  // Normal lenses: straight copy
  if (!unwarp_flag)
  {
    int ycounter = image_size_y - 1;
    while (ycounter >= 0)
    {
      int xcounter = image_size_x - 1;    
      while (xcounter >= 0)
      {
	*hor_im_ptrL-- = *grab_im_ptr++;
	*hor_im_ptrR-- = *grab_im_ptr--;

	grab_im_ptr -= 4; 

	xcounter--;
      }
      ycounter--; 
    }
  }
  // Unusual lenses: copy via lookup table
  else
  {
    int *array_uinL_ptr = array_uinL[0] + image_size_x * image_size_y - 1;
    int *array_vinL_ptr = array_vinL[0] + image_size_x * image_size_y - 1;
    int *array_uinR_ptr = array_uinR[0] + image_size_x * image_size_y - 1;
    int *array_vinR_ptr = array_vinR[0] + image_size_x * image_size_y - 1;

    // Offset positions for grab image
    u_short *grab_im_ptr_base = grab_array[0];
    int grab_im_offsetL;
    int grab_im_offsetR;

    while (hor_im_ptrL >= hor_im_baseL)
    {
      grab_im_offsetL = *array_vinL_ptr-- * grab_size_x 
	                + *array_uinL_ptr--;
      grab_im_offsetR = *array_vinR_ptr-- * grab_size_x 
	                + *array_uinR_ptr--;
	  
      *hor_im_ptrL-- =	*((u_char *) (grab_im_ptr_base + grab_im_offsetL));
      *hor_im_ptrR-- = 	*((u_char *) (grab_im_ptr_base + grab_im_offsetR) + 1);
    }

  }

  return 0;
}

int local_open_meteor()
{
  cout << "Opening module in STEREO mode." << endl;    

  if (forms_init_meteor( 3, 1, NULL, NULL, &grab_size_x, &grab_size_y )==NULL)
  {
    cerr << "WARNING: can't open meteor. " << endl;
    // exit(1);
  }

  meteor_flag = OPEN;
  cout << "Meteor opened." << endl;

  return 0;
}

/************************************Grab*************************************/

int g_grab_images()
{
  gettimeofday(&tod1, NULL);
  time1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
  
  grab_array[0] = (u_short *) forms_single_grab();

  gettimeofday(&tod2, NULL);
  time2 = tod2.tv_sec + tod2.tv_usec / 1000000.0;

  //  cerr << "Time for grab = " << time2 - time1 << " sec." << endl;
  
  copy_images();

  gettimeofday(&tod1, NULL);
  time1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
  
  //  cerr << "Time for copy = " << time1 - time2 << " sec." << endl;

  return 0;
}


/********************************Init Grabbing********************************/

int init_grabbing(FD_formrob *fd_fr, Hor_Image **image, int *uLp, int *vLp, 
		  int *uRp, int *vRp, int BS)
{
  fd_formrob = fd_fr;

  BOXSIZE = BS;

  uLptr = uLp; vLptr = vLp; uRptr = uRp; vRptr = vRp;

  grab_array = new (u_short *) [grab_size_y];

  hor_imageL = hor_alloc_image(image_size_x, image_size_y, HOR_U_CHAR, NULL);
  hor_imageR = hor_alloc_image(image_size_x, image_size_y, HOR_U_CHAR, NULL);

  image[LEFT] = hor_imageL;
  image[RIGHT] = hor_imageR; 
  
  fl_add_canvas_handler(fd_formrob->canvasL, ButtonPress, 
			canvas_buttonL, NULL);
  fl_add_canvas_handler(fd_formrob->canvasR, ButtonPress, 
			canvas_buttonR, NULL);
  fl_add_canvas_handler(fd_formrob->canvasL, Expose, canvas_exposeL, NULL);
  fl_add_canvas_handler(fd_formrob->canvasR, Expose, canvas_exposeR, NULL);


  display = fl_get_display();
  wtwin_L = FL_ObjWin(fd_formrob->canvasL);
  wtwin_R = FL_ObjWin(fd_formrob->canvasR);

  XGCValues gc_values;
  gc_values.graphics_exposures = False;
  gc_L = XCreateGC(display, wtwin_L, GCGraphicsExposures, &gc_values);  
  gc_R = XCreateGC(display, wtwin_R, GCGraphicsExposures, &gc_values);  
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);

  cerr << "Screen depth = " << depth << endl;

  if (meteor_flag == CLOSED)
    local_open_meteor();

  unwarp_flag = UNWARP_FLAG; // Constant set in grab.h
  ifstream infile(lookup_filename, ios::in);

  if (infile != NULL && unwarp_flag)
  {
    cerr << "Setting up image-unwarping to get perspective images." << endl;
    int isx, isy, gsx, gsy;
    infile >> isx >> isy >> gsx >> gsy;
    assert (isx == image_size_x && isy == image_size_y);
    
    // Allocate arrays: make sure rows are consecutive in memory
    // so we can move through quickly with pointers
    int row, col;
    array_uinL = new (int *)[image_size_y];
    array_uinL[0] = new (int) [image_size_x * image_size_y];
    for (row = 0; row < image_size_y; row++)
      array_uinL[row] = array_uinL[0] + row * image_size_x;

    array_vinL = new (int *)[image_size_y];
    array_vinL[0] = new (int) [image_size_x * image_size_y];
    for (row = 0; row < image_size_y; row++)
      array_vinL[row] = array_vinL[0] + row * image_size_x;

    array_uinR = new (int *)[image_size_y];
    array_uinR[0] = new (int) [image_size_x * image_size_y];
    for (row = 0; row < image_size_y; row++)
      array_uinR[row] = array_uinR[0] + row * image_size_x;

    array_vinR = new (int *)[image_size_y];
    array_vinR[0] = new (int) [image_size_x * image_size_y];
    for (row = 0; row < image_size_y; row++)
      array_vinR[row] = array_vinR[0] + row * image_size_x;

    // Now read in data
    double thetamaxdeg;
    infile >> thetamaxdeg;
    cout << "Reading lookup table for angular half-range " << thetamaxdeg
	 << endl << "PLEASE WAIT." << endl;

    for (row = 0; row < image_size_y; row++)
      for (col = 0; col < image_size_x; col++)
	infile >> array_uinL[row][col];

    for (row = 0; row < image_size_y; row++)
      for (col = 0; col < image_size_x; col++)
	infile >> array_vinL[row][col];

    for (row = 0; row < image_size_y; row++)
      for (col = 0; col < image_size_x; col++)
	infile >> array_uinR[row][col];

    for (row = 0; row < image_size_y; row++)
      for (col = 0; col < image_size_x; col++)
	infile >> array_vinR[row][col];

    cout << "Finished reading lookup table." << endl;
  }
  else
  {
    unwarp_flag = 0;
    cout << "UNWARP_FLAG not set or could not find lookup table file " 
	 << lookup_filename << "." << endl
	 << "No image warping (normal cameras and lenses)." << endl;
  }

  return 0;
}

int close_grabbing(void)
{ 
   if(meteor_flag == OPEN)
   {
     forms_close_meteor();
     meteor_flag = CLOSED;
   }

   return 0;
}
