/***********Functions related to image grabbing, displaying, saving***********/

/* All the very most general headers are now in this file */

#include <general_headers.h>

// Simulator Related

#include <OB/CORBA.h>
#include <OB/Util.h>
#include <OB/CosNaming.h>
#include <OB/CosEventChannelAdmin.h>
#include <OB/CosEventComm_skel.h>
#include <OB/Reactor.h>
#include "VisionSimulator.h"
#include "FishEye.h"

#include "vsimulator.h"
#include "module.h"
#include "modsim.h"
#include "common.h"
#include "forms.h"

/***********************Variables for simulator************************/
int now_displaying_monitor_flag = 0;
int now_displaying_depth_flag = 0;
int mwindow_flag = CLOSED;
float front_val = 20.0, back_val = 500.0;

/****************************Global X Stuff***********************************/

static Display *display;
static int depth, bytes_per_pixel, screen;
extern int defaultDepth(void);

/********************X Stuff for the simulator window*************************/

static XImage *mimage_L, *mdepth_L, *mimage_R, *mdepth_R;
static XImage *ximage_L, *ximage_R;
static Window mtwin, mtwin_L, mtwin_R;
static GC mgc_L, mgc_R;

/******************************Image Dimensions*******************************/

/* In stereo, it is an array of u_short */
//extern int grab_size_x, grab_size_y;

/* image_size_x and image_size_y are the dimensions of the images 
   displayed and saved */
int vimage_size_x = MAXCOLS / 2, vimage_size_y = MAXROWS / 2;

/* window_size_x and window_size_y are the dimensions of the display
   window. If in stereo mode, window_size_x is doubled */
int window_size_x, window_size_y;

static char sim_name[100] = "test";

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

// Images for processing
Hor_Image *hor_vimageL, *hor_vimageR;

int setup_monitor_window(void) 
{
    XGCValues gc_values;
    XSizeHints sizehints;
    Visual *vis;
    XColor *palette;
    Colormap wtcolormap;
    XSetWindowAttributes set_attributes;
    int pixelNum, numCols;

    display = fl_get_display();
    mtwin = XCreateSimpleWindow(display, DefaultRootWindow(display),
				 0, 0, vimage_size_x * 2, vimage_size_y, 2,
				 BlackPixel(display, DefaultScreen(display)),
				 BlackPixel(display, DefaultScreen(display))); 
    mtwin_L = XCreateSimpleWindow(display, mtwin,
				 0, 0, vimage_size_x, vimage_size_y, 2,
				 BlackPixel(display, DefaultScreen(display)),
				 BlackPixel(display, DefaultScreen(display))); 
    mtwin_R = XCreateSimpleWindow(display, mtwin,
				 320, 0, vimage_size_x, vimage_size_y, 2,
				 BlackPixel(display, DefaultScreen(display)),
				 BlackPixel(display, DefaultScreen(display))); 
    set_attributes.backing_store = Always;
    XChangeWindowAttributes(display, mtwin_L, CWBackingStore, &set_attributes);
    XChangeWindowAttributes(display, mtwin_R, CWBackingStore, &set_attributes);

    sizehints.flags = PMaxSize|PAspect|PResizeInc;
    sizehints.max_width = window_size_x*2;
    sizehints.max_height = window_size_y;
    sizehints.min_aspect.x = 1;
    sizehints.min_aspect.y = 2;
    sizehints.max_aspect.x = 10;
    sizehints.max_aspect.y = 1;
    sizehints.width_inc = 8;
    sizehints.height_inc = 2;
    XSetNormalHints(display, mtwin, &sizehints);

    XMapWindow(display, mtwin);
    XMapWindow(display, mtwin_L);
    XMapWindow(display, mtwin_R);
    XSelectInput(display, mtwin_L, 
		 ButtonPressMask|ButtonReleaseMask|ButtonMotionMask);
    XSelectInput(display, mtwin_R, 
		 ButtonPressMask|ButtonReleaseMask|ButtonMotionMask);

    gc_values.graphics_exposures = False;
    mgc_L = XCreateGC(display, mtwin_L, GCGraphicsExposures, &gc_values);  
    mgc_R = XCreateGC(display, mtwin_R, GCGraphicsExposures, &gc_values);  

    screen = DefaultScreen(display);
    vis = DefaultVisual(display, screen);
    bytes_per_pixel =  1;
    depth = DefaultDepth(display, screen);
    fprintf(stderr, "depth=%d\n", depth);

    bytes_per_pixel = (depth==15 || depth==16) ? 2 : (depth==24 || depth==32) ? 4 : 1;

    mimage_L = XCreateImage(display, vis,
			  depth, ZPixmap, 0, NULL, 
			  vimage_size_x, vimage_size_y, 
			  32, vimage_size_x * bytes_per_pixel);
    free(mimage_L->data);
    mdepth_L = XCreateImage(display, vis,
			  depth, ZPixmap, 0, NULL, 
			  vimage_size_x, vimage_size_y, 
			  32, vimage_size_x * bytes_per_pixel);
    free(mdepth_L->data);
    mimage_R = XCreateImage(display, vis,
			  depth, ZPixmap, 0, NULL, 
			  vimage_size_x, vimage_size_y, 
			  32, vimage_size_x * bytes_per_pixel);
    free(mimage_R->data);
    mdepth_R = XCreateImage(display, vis,
			  depth, ZPixmap, 0, NULL, 
			  vimage_size_x, vimage_size_y, 
			  32, vimage_size_x * bytes_per_pixel);
    free(mdepth_R->data);

    printf("Color mode = %d\n", vis->c_class);
    if (vis->c_class == PseudoColor) {
      //    if (1) {

      fprintf(stderr, "Using PseudoColor visual (making colour map)\n");

      /* make colour map */
      wtcolormap = XCreateColormap(display, DefaultRootWindow(display),
				   vis, AllocAll);

      numCols = 256;
      palette = (XColor *)malloc(numCols * sizeof(XColor));
      for (pixelNum=0; pixelNum < numCols; pixelNum++) {
	palette[pixelNum].pixel=pixelNum;
	palette[pixelNum].red=pixelNum<<8;
	palette[pixelNum].green=pixelNum<<8;
	palette[pixelNum].blue=pixelNum<<8;
	palette[pixelNum].flags=DoRed|DoGreen|DoBlue;
      }

      palette[RED].red = 65535; palette[RED].green = 0; palette[RED].blue = 0;
      palette[GREEN].red = 0; palette[GREEN].green = 65535; 
      palette[GREEN].blue = 0; palette[BLUE].red = 0; palette[BLUE].green = 0;
      palette[BLUE].blue = 65535;

      XStoreColors(display, wtcolormap, palette, numCols);
      XSetWindowColormap(display, mtwin, wtcolormap);
      XSetWindowColormap(display, mtwin_L, wtcolormap);
      XSetWindowColormap(display, mtwin_R, wtcolormap);

    } else {

      fprintf(stderr, "Using DirectColor visual\n");

    }

    mwindow_flag = OPEN;

    hor_colourmap_setup ( display, 2, 6,
			  "Red",       &Red,       "Green",         &Green,
			  "Blue",      &Blue,      "Yellow",        &Yellow,
			  "SteelBlue", &SteelBlue, "LightSeaGreen", &LightSeaGreen,
			  "thistle",   &thistle,   "Cyan",          &Cyan,
			  NULL );
    GC gc = fl_get_gc();
    
    hor_display_initialise ( display, gc,
			     SteelBlue, Green, Red, LightSeaGreen, thistle );
  
    hor_vimageL = hor_alloc_image(vimage_size_x, vimage_size_y, HOR_U_CHAR, NULL);
    hor_vimageR = hor_alloc_image(vimage_size_x, vimage_size_y, HOR_U_CHAR, NULL);

    return 0;
}


int close_monitor_window(void)
{
  if (mimage_L != NULL) free(mimage_L);
  if (mimage_R != NULL) free(mimage_R);
  
  if (mtwin_L != 0) XDestroyWindow(display, mtwin_L);
  if (mtwin_R != 0) XDestroyWindow(display, mtwin_R);
  if (mtwin != 0) XDestroyWindow(display, mtwin);

  mwindow_flag = CLOSED;

  return 0;
}

void update_monitor(void)
{
  if (mwindow_flag == OPEN){

    get_view_of_cameras(mdepth_L, mdepth_R, mimage_L, mimage_R);

    /*
    if(defaultDepth()==24){
      int con_L[vimage_size_x * vimage_size_y];
      int con_R[vimage_size_x * vimage_size_y];
      int i, j, ii, p;
      
      for(i = 0; i < vimage_size_y; i++){
	ii = vimage_size_x * i;
	for(j = 0; j < vimage_size_x; j++){
	  p = (int)mimage_L->data[ii + j];
	  con_L[ii + j] = p + p*256 + p*256*256;
	  p = (int)mimage_R->data[ii + j];
	  con_R[ii + j] = p + p*256 + p*256*256;
	}
      }

    mimage_L->data = (char *)con_L;
    mimage_R->data = (char *)con_R;
    }
    else{
      if(defaultDepth()==16){
	short int con_L[vimage_size_x * vimage_size_y];
	short int con_R[vimage_size_x * vimage_size_y];
	int i, j, ii, p;
	
	for(i = 0; i < vimage_size_y; i++){
	  ii = vimage_size_x * i;
	  for(j = 0; j < vimage_size_x; j++){
	    p = (int)mimage_L->data[ii + j];
	    con_L[ii + j] = p; // + p*256 + p*256*256;
	    p = (int)mimage_R->data[ii + j];
	    con_R[ii + j] = p; // + p*256 + p*256*256;
	  }
	}

	mimage_L->data = (char *)con_L;
	mimage_R->data = (char *)con_R;
      }
    }   
*/

    hor_vimageL->array.uc[0] = (u_char *) mimage_L->data;
    hor_vimageR->array.uc[0] = (u_char *) mimage_R->data;

    ximage_L = hor_convert_image_to_X_format ( hor_vimageL,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    ximage_R = hor_convert_image_to_X_format ( hor_vimageR,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    XPutImage(display, mtwin_L, mgc_L, ximage_L,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
    XPutImage(display, mtwin_R, mgc_R, ximage_R,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
  
    XSync(display, 0);
  }

  return;
}

int show_monitor()
{
  if (mwindow_flag == CLOSED)
  {
    setup_monitor_window();
  }

  get_view_of_cameras(mdepth_L, mdepth_R, mimage_L, mimage_R);

  hor_colourmap_setup ( display, 2, 6,
		      "Red",       &Red,       "Green",         &Green,
		      "Blue",      &Blue,      "Yellow",        &Yellow,
		      "SteelBlue", &SteelBlue, "LightSeaGreen", &LightSeaGreen,
		      "thistle",   &thistle,   "Cyan",          &Cyan,
			NULL );

  GC gc = fl_get_gc();

  hor_display_initialise ( display, gc,
			   SteelBlue, Green, Red, LightSeaGreen, thistle );
  
  hor_vimageL->array.uc[0] = (u_char *) mimage_L->data;
  hor_vimageR->array.uc[0] = (u_char *) mimage_R->data;

    ximage_L = hor_convert_image_to_X_format ( hor_vimageL,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    ximage_R = hor_convert_image_to_X_format ( hor_vimageR,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    XPutImage(display, mtwin_L, mgc_L, ximage_L,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
    XPutImage(display, mtwin_R, mgc_R, ximage_R,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
  
  XSync(display, 0);

  return 0;
}

int close_monitor(void){

  close_monitor_window();

  return 0;
}

int capture_monitor(Hor_Image *imL, Hor_Image *imR)
{
  if (mwindow_flag == CLOSED)
  {
    setup_monitor_window();
  }

  get_view_of_cameras(mdepth_L, mdepth_R, mimage_L, mimage_R);

  hor_vimageL->array.uc[0] = (u_char *) mimage_L->data;
  hor_vimageR->array.uc[0] = (u_char *) mimage_R->data;

    ximage_L = hor_convert_image_to_X_format ( hor_vimageL,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    ximage_R = hor_convert_image_to_X_format ( hor_vimageR,
					       display,
					       screen,
					       depth,
					       1,
					       1,
					       1,
					       NULL);

    XPutImage(display, mtwin_L, mgc_L, ximage_L,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
    XPutImage(display, mtwin_R, mgc_R, ximage_R,
	      0, 0, 0, 0, vimage_size_x, vimage_size_y);
  
    XSync(display, 0);

  imL = hor_vimageL;
  imR = hor_vimageR;

  return 0;
}

   
/******************************Callback functions*****************************/
void start_gray_monitor(FL_OBJECT *ob, long data)
{
  show_monitor();
  now_displaying_monitor_flag = 1;

  return;
}

void stop_gray_monitor(FL_OBJECT *ob, long data)
{
  close_monitor();
  now_displaying_monitor_flag = 0;

  return;
}

void start_depth_monitor(FL_OBJECT *ob, long data)
{
  //  show_depth();
  now_displaying_depth_flag = 1;

  return;
}

void stop_depth_monitor(FL_OBJECT *ob, long data)
{
  //  close_depth();
  now_displaying_depth_flag = 0;

  return;
}

void start_top_view(FL_OBJECT *ob, long data)
{
  /* fill-in code for callback */
}

void stop_top_view(FL_OBJECT *ob, long data)
{
  /* fill-in code for callback */
}

void set_front(FL_OBJECT *ob, long data)
{
  front_val = fl_get_counter_value(ob);
}

void set_back(FL_OBJECT *ob, long data)
{
  back_val = fl_get_counter_value(ob);
}

void set_clip(FL_OBJECT *ob, long data)
{
  set_clip_distance(front_val, back_val);
}

#if 0
void write_gray_monitor(FL_OBJECT *ob, long data)
{
  //  int i,j;
  char *view_buff, fname1[100];

  tfBitsPerSample=  8;  tfSamplesPerPixel=  1;
  tfPlanarConfiguration=  1; tfPhotometricInterpretation=  2;
  tfGrayResponseUnit=  2; tfImageDepth= 8;
  tfImageWidth = vimage_size_x;  tfImageHeight = vimage_size_y;
  tfBytesPerRow=vimage_size_x*tfSamplesPerPixel;

  strcpy(fname1,"image/");
  strcat(fname1,sim_name); 
  strcat(fname1,"L"); 

  view_buff = get_view_of_camera(LEFT);
  myWriteTiff((unsigned short)1,view_buff,fname1);

  strcpy(fname1,"image/");
  strcat(fname1,sim_name); 
  strcat(fname1,"R"); 

  view_buff = get_view_of_camera(RIGHT);
  myWriteTiff((unsigned short)1,view_buff,fname1);

  return;
}
#endif

void sim_filename_input(FL_OBJECT *ob, long data)
{
  strcpy(sim_name, fl_get_input(ob));

  return;
}
