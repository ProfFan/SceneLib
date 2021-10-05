/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>
#include <X11/Xaw/Form.h>
#include <X11/Xow/Canvas.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/tool.h"

static Widget   *canvas;
static Hor_Bool  canvases_initialised = HOR_FALSE;

static Hor_Assoc_Label string_label;

static Hor_Assoc_Label *canvas_label;
static Hor_Bool         canvas_labels_initialised = HOR_FALSE;
static int              current_canvas_label;

static int no_canvases;
static int canvas_size;

static XGCValues gc_val; /* made global static so that fields will be
                            initialised to zero */

/*******************
*   void @fill_canvas_panel ( Widget panel,
*                            int canvas_size, int no_canvases,
*                            int canvases_to_a_side )
*
*   Creates desired number of canvases in a panel and adds button function
*   window below it.
********************/
void fill_canvas_panel ( Widget panel,
			 int    loc_canvas_size,
			 int    loc_no_canvases,
			 int    canvases_to_a_side )
{
   int    c, r, n;
   Widget bottom_left_canvas;

   if ( canvases_initialised )
      hor_error ( "illegal repeated call (hor_fill_canvas_panel)", HOR_FATAL );

   canvas_size = loc_canvas_size;
   no_canvases = loc_no_canvases;

   canvas = hor_malloc_ntype ( Widget, no_canvases );
   canvas[0] = XtVaCreateManagedWidget ( "canvas", canvasWidgetClass, panel,
					 XtNheight, canvas_size,
					 XtNwidth,  canvas_size, NULL );
   bottom_left_canvas = canvas[0];
   for ( n = 1, c = 1; c < canvases_to_a_side; c++, n++ )
      canvas[n] = XtVaCreateManagedWidget ( "canvas", canvasWidgetClass, panel,
					    XtNheight, canvas_size,
					    XtNwidth,  canvas_size,
					    XtNfromHoriz, canvas[n-1], NULL );

   for ( r = 1; (r < canvases_to_a_side) && (n < no_canvases); r++ )
   {
      canvas[n] = XtVaCreateManagedWidget ( "canvas", canvasWidgetClass, panel,
				     XtNheight, canvas_size,
				     XtNwidth,  canvas_size,
				     XtNfromVert, canvas[n-canvases_to_a_side],
				     NULL );
      bottom_left_canvas = canvas[n];
      n++;
      for ( c = 1; (c < canvases_to_a_side) && (n < no_canvases); c++, n++ )
	 canvas[n] = XtVaCreateManagedWidget ( "canvas", canvasWidgetClass,
			             panel,
				     XtNheight, canvas_size,
				     XtNwidth,  canvas_size,
				     XtNfromHoriz, canvas[n-1],
				     XtNfromVert, canvas[n-canvases_to_a_side],
				     NULL );
   }

   /* create button function window below canvases */
   string_label = hor_display_set_string ( panel, XtNfromVert,
					   bottom_left_canvas, NULL );
   canvases_initialised = HOR_TRUE;
   canvas_labels_initialised = HOR_FALSE;
}

/*******************
*   void @register_canvases ( Display *display,
*                            u_long   image_background_colour,
*                            u_long   image_border_colour,
*                            u_long   region_border_colour,
*                            u_long   below_threshold_colour,
*                            u_long   above_threshold_colour )
*
*   Registers all created canvases by calling hor_display_set_window() on each
*   one. hor_display_reset_window() is then called to reset the graphics window
*   to the first (top-left).
********************/
void register_canvases ( Display *display,
			 u_long   image_background_colour,
			 u_long   image_border_colour,
			 u_long   region_border_colour,
			 u_long   below_threshold_colour,
			 u_long   above_threshold_colour )
{
   int                   n;
   Window                canvas_window;
   XSetWindowAttributes  sw;

   if ( !canvases_initialised )
      hor_error ( "canvases not initialised (register_canvases)",
		  HOR_FATAL );

   if ( canvas_labels_initialised )
      hor_error ( "illegal repeated call (register_canvases)", HOR_FATAL );

   canvas_label = hor_malloc_ntype ( Hor_Assoc_Label, no_canvases );
   for ( n = 0; n < no_canvases; n++ )
   {
      /* set up graphics context stuff */
      canvas_window = XtWindow ( canvas[n] );
      if ( n == 0 )
	 hor_display_initialise ( display,
			      XCreateGC ( display, canvas_window, 0, &gc_val ),
			      image_background_colour, image_border_colour,
			      region_border_colour,
			      below_threshold_colour, above_threshold_colour );
      
      /* tell graphics module which window to draw on */
      canvas_label[n] = hor_display_set_window ( canvas_window, canvas[n],
						 XC_cross_reverse,
						 string_label );

      /* set other window attributes */
      sw.backing_store = Always ;
      XChangeWindowAttributes ( display, canvas_window, CWBackingStore, &sw);
   }

   hor_free ( (void *) canvas );
   canvas_labels_initialised = HOR_TRUE;
   current_canvas_label = 0;
   hor_display_reset_window ( canvas_label[current_canvas_label] );

   /* set mouse callback functions for first canvas */
   hor_display_set_mouse_functions ( canvas_label[current_canvas_label],
				hor_region_start,  hor_region_finish, "Region",
				hor_region_cancel, NULL,              "Cancel",
				hor_region_select, NULL,              "Select",
				hor_region_moving, NULL, NULL, NULL );

}

/*******************
*   void @change_canvas(void)
*
*   Switch canvas. Canvases are switched in "raster-scan" order.
********************/
void change_canvas(void)
{
   int      width, height;
   Hor_Bool ok;

   if ( !canvas_labels_initialised )
      hor_error ( "canvas labels not initialised (hor_change_canvas)",
		  HOR_FATAL );

   /* discard any pending image ROIs */
   hor_region_clear();

   /* remove mouse event callback function for old canvas */
   hor_display_remove_mouse_functions ( canvas_label[current_canvas_label] );

   /* read canvas parameters to be transferred to new canvas */
   ok = hor_display_get_dims ( &width, &height );

   /* switch to new canvas */
   current_canvas_label++;
   if ( current_canvas_label == no_canvases )
      current_canvas_label = 0;

   hor_display_reset_window ( canvas_label[current_canvas_label] );
   if ( ok ) hor_display_set_params ( width, height );

   /* set mouse callback functions for new canvas */
   hor_display_set_mouse_functions ( canvas_label[current_canvas_label],
				hor_region_start,  hor_region_finish, "Region",
				hor_region_cancel, NULL,              "Cancel",
				hor_region_select, NULL,              "Select",
				hor_region_moving, NULL, NULL, NULL );
}
