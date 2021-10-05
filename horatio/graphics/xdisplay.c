/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* Modified Feb 1996 by Ian Reid (ian@robots.ox.ac.uk) to cope with 
   different screen depths, 8, 16 and 24.  Added #ifdef GET_IMAGE 
   since 24 bits screens require a x_to_image_colourmap which is
   way too large */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Label.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Label.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

#include "ximage.h"

#ifdef HOR_MSDOS
#define M_PI 3.1415926535897932384626433
#endif

#define MAX_LABEL_SIZE 8

/* functions to make mouse event strings */
static void make_padded_label ( const char *label, char *padded_label )
{
   int length = strlen(label), i, pad;

   if ( length > MAX_LABEL_SIZE )
   {
      hor_warning ( "label too long (make_padded_label)" );
      pad = 0;
   }
   else
      pad = MAX_LABEL_SIZE - length;

   for ( i = 0; i < pad; i++ )
      padded_label[i] = ' ';

   for ( ; i < MAX_LABEL_SIZE; i++ )
      padded_label[i] = label[i-pad];

   padded_label[MAX_LABEL_SIZE] = '\0';
}

static void form_single_label ( const char *left_label,
			        const char *middle_label,
			        const char *right_label, char *string )
{
   char left_padded[MAX_LABEL_SIZE+1];
   char middle_padded[MAX_LABEL_SIZE+1];
   char right_padded[MAX_LABEL_SIZE+1];

   make_padded_label ( left_label,   left_padded );
   make_padded_label ( middle_label, middle_padded );
   make_padded_label ( right_label,  right_padded );

   sprintf ( string, "%s / %s / %s", left_padded, middle_padded, right_padded);
}

static char *form_blank_label_string ( char *s )
{
   form_single_label ( "", "", "", s );
   return s;
}

static char *form_null_label_string ( char *s )
{
   form_single_label ( "Null", "Null", "Null", s );
   return s;
}

/* canvas string module */

typedef struct
{
   Hor_Assoc_Label canvas_label; /* currently set canvas for this string */
   Widget      string;
} String_Def;

static Hor_Assoc_List  string_list = NULL;
static Hor_Assoc_Label next_string = HOR_ASSOC_START;

/*******************
*   Hor_Assoc_Label @hor_display_set_string ( Widget parent, ... )
*   Hor_Assoc_Label @hor_display_delete_string ( Hor_Assoc_Label string_label )
*
*   hor_display_set_string() creates and registers a string widget for canvases
*                            and returns a label for it. The variable argument
*                            list is composed of pairs of String's and
*                            XtArgVal's terminated by NULL, as in a call to
*                            XtVaCreateManagedWidget(). They modify default
*                            resources of the widget upon creation.
*
*   hor_display_delete_string() unregisters the given string widget.
********************/
Hor_Assoc_Label hor_display_set_string ( Widget parent, ... )
{
   char        s[3*MAX_LABEL_SIZE+7];
   String_Def *new_string_def = hor_malloc_type(String_Def);
   Arg         args[100];
   String      resource_name;
   XtArgVal    value;
   Cardinal    no_args = ZERO;

   va_list ap;

   /* read argument list */
   va_start ( ap, parent );
   for(;;)
   {
      resource_name = va_arg ( ap, String );
      if ( resource_name == NULL ) break;

      value = va_arg ( ap, XtArgVal );
      XtSetArg ( args[no_args], resource_name, value );
      no_args++;
   }

   va_end(ap);

   new_string_def->string = XtCreateManagedWidget ( "String", labelWidgetClass,
						    parent, args, no_args );
   new_string_def->canvas_label = HOR_ASSOC_ERROR;
   string_list = hor_assoc_insert ( string_list, next_string,
				    (void *) new_string_def );

   /* initialise mouse function string */
   XtVaSetValues ( new_string_def->string, XtNlabel,
		   form_blank_label_string(s), NULL );
   return next_string++;
}

void hor_display_delete_string ( Hor_Assoc_Label string_label )
{
   if ( hor_assoc_remove ( &string_list, string_label, hor_free_func )
        == HOR_ASSOC_ERROR )
      hor_error ( "invalid string label (hor_display_delete_string)", HOR_FATAL );
}

static Display *display;
static GC       graphics_context;
static u_long   image_background_colour;
static u_long   image_border_colour;
static u_long   region_border_colour;
static Hor_Bool hor_display_initialised = HOR_FALSE;

typedef struct
{
   /* display state variables */
   int   user_width;            /* width and height parameters in user */
   int   user_height;           /* coordinates */
   int   canvas_top_left_c;     /* offset of image origin relative to canvas
				   origin in canvas pixel x coordinates */
   int   canvas_top_left_r;     /* ditto for y */
   int   canvas_bottom_right_c; /* bottom right corner of image   */
   int   canvas_bottom_right_r; /* in canvas coordinates          */

   int   pixel_size_factor;     /* size of internal image pixel when
				   displayed on canvas. Negative values mean
				   that pixels are reduced in size */
   float pixel_size_factor_f;   /* floating point version of above */
} Window_State_Vars;

typedef struct
{
   /* display initialisation variables */
   Window            canvas_window;
   Widget            canvas;
   int               scrnum;
   int               depth;
   u_long            xor_colour;
   Window_State_Vars state_vars;
   Hor_Bool          state_vars_init;

   /* mouse event fields */
   Hor_Assoc_Label canvas_label;
   Hor_Assoc_Label string_label;
   hor_button_func left_button_down_function;
   hor_button_func left_button_up_function;
   hor_button_func middle_button_down_function;
   hor_button_func middle_button_up_function;
   hor_button_func right_button_down_function;
   hor_button_func right_button_up_function;
   hor_button_func mouse_motion_function;
   hor_window_func enter_window_function;
   hor_window_func leave_window_function;

   void       *client_data;
   char        string[3*MAX_LABEL_SIZE+7];
} Graphics_Window;

/* list of registered graphics windows */
static Hor_Assoc_List graphics_window_list = NULL;

/* label of next new graphics window */
static Hor_Assoc_Label next_graphics_window_label = HOR_ASSOC_START;

/* window that graphics is currently displayed in */
static Graphics_Window *current_window;
static Hor_Assoc_Label  current_window_label = HOR_ASSOC_ERROR;

/* store static versions of window parameters to speed up drawing */
static Window canvas_window;
static int    canvas_width;
static int    canvas_height;
static int    scrnum;
static int    depth;
static u_long xor_colour;

static int   user_width, user_height;
static int   canvas_top_left_c;
static int   canvas_top_left_r;
static int   canvas_bottom_right_c;
static int   canvas_bottom_right_r;
static int   pixel_size_factor;
static float pixel_size_factor_f;

static Hor_Bool display_params_initialised = HOR_FALSE; /* requires window size
							   scaling to be set
							   to HOR_TRUE */

static void set_current_state_vars ( Window_State_Vars *state_vars )
{
   /* set local statics (for speedup only) */
   user_width            = state_vars->user_width;
   user_height           = state_vars->user_height;
   canvas_top_left_c     = state_vars->canvas_top_left_c;
   canvas_top_left_r     = state_vars->canvas_top_left_r;
   canvas_bottom_right_c = state_vars->canvas_bottom_right_c;
   canvas_bottom_right_r = state_vars->canvas_bottom_right_r;
   pixel_size_factor     = state_vars->pixel_size_factor;
   pixel_size_factor_f   = state_vars->pixel_size_factor_f;
}

static void set_current_window ( Graphics_Window *window,
				 Hor_Assoc_Label  window_label )
{
   Dimension width, height;

   current_window       = window;
   current_window_label = window_label;
   if ( window->state_vars_init )
   {
      set_current_state_vars ( &window->state_vars );
      display_params_initialised = HOR_TRUE;
   }
   else
      display_params_initialised = HOR_FALSE;

   /* set local statics (for speedup only) */
   canvas_window = window->canvas_window;
   XtVaGetValues (window->canvas, XtNwidth,  &width, XtNheight, &height, NULL);
   canvas_width  = (int) width;
   canvas_height = (int) height;
   scrnum        = window->scrnum;
   depth         = window->depth;
   xor_colour    = window->xor_colour;
}

static char defaultTranslations[] = "#override\n<EnterWindow>: canvasAction()\n<LeaveWindow>: canvasAction()";
static XtTranslations mytranslations;

/*******************
*   void @hor_display_initialise ( Display *display, GC graphics_context,
*                                 u_long image_background_colour,
*                                 u_long image_border_colour,
*                                 u_long region_border_colour,
*                                 u_long below_threshold_colour,
*                                 u_long above_threshold_colour )
*
*   Initializes X display and graphics context. Must be called before other
*   graphics functions. The colour arguments specify a standard colours
*   for displaying images. Arguments:
*
*   display:                 Connection to X server.
*   graphics_context:        A graphics context to be shared by all canvases.
*   image_background_colour: Colour of canvas background.
*   image_border_colour:     Colour of the line drawn around an image.
*   region_border_colour:    Colour of a highlighted region boundary.
*   below_threshold_colour:  Colours shown when displaying an image pixel
*   above_threshold_colour:  below/above specified display limits.
*
*   Note that only one graphics context is used.
********************/
void hor_display_initialise ( Display *loc_display, GC loc_graphics_context,
			      u_long loc_image_background_colour,
			      u_long loc_image_border_colour,
			      u_long loc_region_border_colour,
			      u_long loc_below_threshold_colour,
			      u_long loc_above_threshold_colour )
{
   display                 = loc_display;
   graphics_context        = loc_graphics_context;
   image_background_colour = loc_image_background_colour;
   image_border_colour     = loc_image_border_colour;
   region_border_colour    = loc_region_border_colour;
   hor_set_threshold_colours ( loc_below_threshold_colour,
			       loc_above_threshold_colour );
   mytranslations = XtParseTranslationTable ( defaultTranslations );
   hor_display_initialised = HOR_TRUE;
}

/*****************
 *  Modified by davison@etl.go.jp to just comment out the 
 *  translations bit when some functions are to be used in
 *  an xforms application
 *****************/
void hor_display_initialise_xforms ( Display *loc_display, 
				     GC loc_graphics_context,
				     u_long loc_image_background_colour,
				     u_long loc_image_border_colour,
				     u_long loc_region_border_colour,
				     u_long loc_below_threshold_colour,
				     u_long loc_above_threshold_colour )
{
   display                 = loc_display;
   graphics_context        = loc_graphics_context;
   image_background_colour = loc_image_background_colour;
   image_border_colour     = loc_image_border_colour;
   region_border_colour    = loc_region_border_colour;
   hor_set_threshold_colours ( loc_below_threshold_colour,
			       loc_above_threshold_colour );
   // mytranslations = XtParseTranslationTable ( defaultTranslations );
   hor_display_initialised = HOR_TRUE;
}

/* handle_mouse_event() is called on any mouse event, calls functions specified
                        by hor_display_set_mouse_functions(), passing as
			arguments he coordinates of the current mouse position.
			*/
static void handle_mouse_event ( Widget widget, XtPointer client,
				 XtPointer data )
{
   XEvent          *event = (XEvent          *) data;
   Graphics_Window *gw    = (Graphics_Window *) client;

   switch ( event->type )
   {
      String_Def *string_def;

      case EnterNotify:
      string_def=(String_Def *) hor_assoc_find (string_list, gw->string_label);
      if ( string_def != NULL ) /* if NULL, string must have been removed */
      {
	 XtVaSetValues ( string_def->string, XtNlabel, gw->string, NULL );
	 string_def->canvas_label = gw->canvas_label;
      }

      if ( gw->enter_window_function != NULL )
	 gw->enter_window_function ( gw->client_data );

      break;

      case LeaveNotify:
      string_def=(String_Def *) hor_assoc_find (string_list, gw->string_label);
      if ( string_def != NULL ) /* if NULL, string must have been removed */
      {
	 char s[3*MAX_LABEL_SIZE+7];

	 XtVaSetValues ( string_def->string, XtNlabel,
			 form_blank_label_string(s), NULL );
	 string_def->canvas_label = HOR_ASSOC_ERROR;
      }

      if ( gw->leave_window_function != NULL )
	 gw->leave_window_function ( gw->client_data );

      break;

      case ButtonPress:
      switch ( event->xbutton.button )
      {
         case Button1:
	 if ( gw->left_button_down_function != NULL )
	    gw->left_button_down_function ( event->xbutton.x,
					    event->xbutton.y,
					    gw->client_data );
	 break;

         case Button2:
	 if ( gw->middle_button_down_function != NULL )
	    gw->middle_button_down_function ( event->xbutton.x,
					      event->xbutton.y,
					      gw->client_data );
	 break;

         case Button3:
	 if ( gw->right_button_down_function != NULL )
	    gw->right_button_down_function ( event->xbutton.x,
					     event->xbutton.y,
					     gw->client_data );
	 break;
      }

      break;

      case ButtonRelease:
      switch ( event->xbutton.button )
      {
         case Button1:
	 if ( gw->left_button_up_function != NULL )
	    gw->left_button_up_function ( event->xbutton.x,
					  event->xbutton.y,
					  gw->client_data );
	 break;

         case Button2:
	 if ( gw->middle_button_up_function != NULL )
	    gw->middle_button_up_function ( event->xbutton.x,
					    event->xbutton.y,
					    gw->client_data );
	 break;

         case Button3:
	 if ( gw->right_button_up_function != NULL )
	    gw->right_button_up_function ( event->xbutton.x,
					   event->xbutton.y,
					   gw->client_data );
	 break;
      }

      break;

      case MotionNotify:
      if ( gw->mouse_motion_function != NULL )
	 gw->mouse_motion_function ( event->xbutton.x, event->xbutton.y,
				     gw->client_data );

      break;
   }
}

/*******************
*   Hor_Assoc_Label @hor_display_set_window ( Window      canwin,
*                                            Widget      canvas,
*                                            u_int       cursor_shape,
*                                            Hor_Assoc_Label string_label )
*   void @hor_display_reset_window ( Hor_Assoc_Label graphics_window_label )
*   Hor_Assoc_Label @hor_display_get_window(void)
*   void @hor_display_delete_window ( Hor_Assoc_Label graphics_window_label )
*
*   hor_display_set_window() sets up a new graphics window, sets the current
*   display window to it and returns an label which can be used to reset the
*   current window to it at a later stage using hor_display_reset_window().
*   The new window is added to the list of registered windows.
*   The arguments specify the canvas window (canwin), the canvas, the cursor
*   shape and a label for the mouse function string widget. Allowable
*   cursor identifiers are in the include file <X11/cursorfont.h>. The last
*   argument can be can be HOR_ASSOC_ERROR if no string label is required,
*   but otherwise it must be a label returned by a call to
*   hor_display_set_string().
*
*   hor_display_reset_window() sets the current display window to the
*   registered window with the specified label. Switch between canvases using
*   this function.
*
*   hor_display_get_window() returns the label of the current canvas.
*
*   hor_display_delete_window() removes the graphics window with the specified
*   label from the window list. Use this when a popup window with a canvas
*   is unmapped, for instance.
********************/
Hor_Assoc_Label hor_display_set_window ( Window      canwin,
					 Widget      canvas,
					 u_int       cursor_shape,
					 Hor_Assoc_Label string_label )
{
   Graphics_Window *new_window;

   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_window)",
		  HOR_FATAL );

   /* add canvas entry action */
   XtOverrideTranslations ( canvas, mytranslations );

   /* check that provided string label is registered */
   if ( string_label != HOR_ASSOC_ERROR )
      if ( hor_assoc_find ( string_list, string_label ) == NULL )
	 hor_error ( "unknown string label (hor_display_set_window)",
		     HOR_FATAL );

   new_window = hor_malloc_type ( Graphics_Window );
   new_window->canvas_window   = canwin;
   new_window->canvas          = canvas;
   new_window->scrnum          = DefaultScreen ( display );
   new_window->depth           = DefaultDepth  ( display, new_window->scrnum );
   new_window->xor_colour      = hor_power_of_two ( new_window->depth-1 );
   new_window->state_vars_init = HOR_FALSE;
   new_window->canvas_label    = next_graphics_window_label;
   new_window->string_label    = string_label;

   graphics_window_list = hor_assoc_insert ( graphics_window_list,
					     next_graphics_window_label,
					     new_window );
   set_current_window ( new_window, next_graphics_window_label );

   /* set cursor for graphics canvas */
   XDefineCursor ( display, canvas_window,
		   XCreateFontCursor ( display, cursor_shape ) );

   /* set mouse handling callback function */
   XtAddCallback ( canvas, XtNcallback, handle_mouse_event,
		   (XtPointer) new_window );

   /* initialise individual mouse event functions */
   new_window->left_button_down_function   = NULL;
   new_window->left_button_up_function     = NULL;
   new_window->middle_button_down_function = NULL;
   new_window->middle_button_up_function   = NULL;
   new_window->right_button_down_function  = NULL;
   new_window->right_button_up_function    = NULL;
   new_window->mouse_motion_function       = NULL;
   new_window->enter_window_function       = NULL;
   new_window->leave_window_function       = NULL;
   new_window->client_data                 = NULL;

   /* initialise mouse function string */
   form_null_label_string ( new_window->string );
   
   return ( next_graphics_window_label++ );
}

void hor_display_reset_window ( Hor_Assoc_Label graphics_window_label )
{
   if ( graphics_window_label == current_window_label ) return;

   if ( graphics_window_label == HOR_ASSOC_ERROR )
   {
      current_window_label = HOR_ASSOC_ERROR;
      display_params_initialised = HOR_FALSE;
      return;
   }
   else
   {
      void *hor_assoc_data;

      /* find window in registered window list */
      if ( (hor_assoc_data = hor_assoc_find ( graphics_window_list,
					      graphics_window_label )) == NULL)
	 hor_error ("invalid graphics window label (hor_display_reset_window)",
		    HOR_FATAL);

      set_current_window ( (Graphics_Window *) hor_assoc_data,
			   graphics_window_label );
   }
}

Hor_Assoc_Label hor_display_get_window(void)
{
   return current_window_label;
}

void hor_display_delete_window ( Hor_Assoc_Label graphics_window_label )
{
   Graphics_Window *graphics_window;

   graphics_window = hor_assoc_find ( graphics_window_list,
				      graphics_window_label);
   if ( graphics_window == NULL )
      hor_error ( "invalid graphics window label (hor_display_delete_window)",
		  HOR_FATAL );
				  
   if ( graphics_window_label == current_window_label )
   {
      /* reset cursor to standard one */
      XUndefineCursor ( display, canvas_window );

      /* set mouse handling callback function */
      XtRemoveCallback ( current_window->canvas, XtNcallback,
			 handle_mouse_event, (XtPointer) current_window );

      if ( current_window->string_label != HOR_ASSOC_ERROR )
      {
	 String_Def *string_def;

	 string_def = (String_Def *) hor_assoc_find (string_list,
						 current_window->string_label);
	 if ( string_def == NULL )
	    hor_warning ( "invalid string label (hor_display_delete_window)" );
	 else
	 {
	    form_null_label_string ( current_window->string );
	    XtVaSetValues ( string_def->string, XtNlabel,
			    current_window->string, NULL );
	 }
      }

      current_window_label = HOR_ASSOC_ERROR;
      display_params_initialised = HOR_FALSE;
      hor_warning("deleted current canvas (hor_display_delete_window)");
   }

   hor_assoc_remove ( &graphics_window_list, graphics_window_label,
		      hor_free_func );
}

/*******************
*   void @hor_display_set_mouse_functions ( Hor_Assoc_Label canvas_label,
*                                          hor_button_func left_down_func,
*                                          hor_button_func left_up_func,
*                                          const char *left_label,
*                                          hor_button_func middle_down_func,
*                                          hor_button_func middle_up_func,
*                                          const char *middle_label,
*                                          hor_button_func right_down_func,
*                                          hor_button_func right_up_func,
*                                          const char *right_label,
*                                          hor_button_func mouse_motion_func,
*                                          hor_window_func enter_window_func,
*                                          hor_window_func leave_window_func,
*                                          void       *data )
*   void @hor_display_remove_mouse_functions ( Hor_Assoc_Label canvas_label )
*
*   hor_display_set_mouse_functions() sets the functions to call for the 
*                                     various mouse events for the specified
*                                     canvas. The provided data pointer is
*                                     passed to every callback procedure.
*
*   hor_display_remove_mouse_functions() removes the callback functions for the
*                                        specified canvas.
********************/
void hor_display_set_mouse_functions ( Hor_Assoc_Label canvas_label,
				       hor_button_func left_down_func,
				       hor_button_func left_up_func,
				       const char *left_label,
				       hor_button_func middle_down_func,
				       hor_button_func middle_up_func,
				       const char *middle_label,
				       hor_button_func right_down_func,
				       hor_button_func right_up_func,
				       const char *right_label,
				       hor_button_func mouse_motion_func,
				       hor_window_func enter_window_func,
				       hor_window_func leave_window_func,
				       void       *data )
{
   Graphics_Window *gw=(Graphics_Window *)hor_assoc_find(graphics_window_list,
							 canvas_label );
   String_Def *sd;

   if ( gw == NULL )
      hor_error ( "invalid graphics window label (hor_display_set_mouse_functions)", HOR_FATAL );

   form_single_label ( left_label, middle_label, right_label, gw->string );

   gw->left_button_down_function   = left_down_func;
   gw->left_button_up_function     = left_up_func;
   gw->middle_button_down_function = middle_down_func;
   gw->middle_button_up_function   = middle_up_func;
   gw->right_button_down_function  = right_down_func;
   gw->right_button_up_function    = right_up_func;
   gw->mouse_motion_function       = mouse_motion_func;
   gw->enter_window_function       = enter_window_func;
   gw->leave_window_function       = leave_window_func;
   gw->client_data                 = data;

   /* reset string if necessary */
   if ( (sd = hor_assoc_find ( string_list, gw->string_label )) == NULL )
      return;

   if ( sd->canvas_label == gw->canvas_label )
      XtVaSetValues ( sd->string, XtNlabel, gw->string, NULL );
}

void hor_display_remove_mouse_functions ( Hor_Assoc_Label canvas_label )
{
   hor_display_set_mouse_functions ( canvas_label, NULL, NULL, "Null",
				                   NULL, NULL, "Null",
				                   NULL, NULL, "Null",
				     NULL, NULL, NULL, NULL);
}

/*******************
*   Hor_Bool @hor_display_set_params ( int width, int height )
*   Hor_Bool @hor_display_canvas_set_params ( Hor_Assoc_Label canvas_label,
*                                            int width, int height )
*   Hor_Bool @hor_display_get_params ( int *top_left_c,     int *top_left_r,
*                                     int *bottom_right_c, int *bottom_right_r)
*   Hor_Bool @hor_display_get_dims ( int *width_ptr, int *height_ptr )
*
*   hor_display_set_params() sets up the relationship between user image
*   coordinates and display image coordinates. The width and height arguments
*   specify the dimensions of the rectangle in which subsequent graphics
*   commands will be drawn. You must call hor_display_set_params(),
*   hor_display_image() or hor_display_canvas_set_params() to set up display
*   parameters for each window before using other graphics functions.
*
*   hor_display_canvas_set_params() does what hor_display_set_params() does in
*   a specific canvas.
*
*   hor_display_get_params() returns the corner positions of the current image
*   display window in canvas coordinates. Returns HOR_TRUE if canvas has been
*   initialized, prints an error and returns HOR_FALSE otherwise.
*
*   hor_display_get_dims() returns the width and height originally passed to
*   hor_display_set_params().
********************/
Hor_Bool hor_display_set_params ( int width, int height )
                                  /* dimensions of internal image */
{
   int                width_in_canvas, height_in_canvas;
   Window_State_Vars *vars;

   if ( current_window_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_GRAPHICS_DISPLAY_WINDOW_NOT_SET;
      return HOR_FALSE;
   }

   if ( width < 1 || height < 1 )
   {
      hor_errno = HOR_GRAPHICS_ILLEGAL_INTERNAL_DIMENSIONS;
      return HOR_FALSE;
   }

   vars = &current_window->state_vars;
   if ( width > canvas_width || height > canvas_height )
   {
      vars->pixel_size_factor = -hor_imax ( 1 + (width-1)/canvas_width,
					    1 + (height-1)/canvas_height );
      width_in_canvas  = (width-1)/(-vars->pixel_size_factor) + 1;
      height_in_canvas = (height-1)/(-vars->pixel_size_factor) + 1;
   }
   else
   {
      vars->pixel_size_factor = hor_imin ( canvas_width/width,
					   canvas_height/height );
      width_in_canvas  = width*vars->pixel_size_factor;
      height_in_canvas = height*vars->pixel_size_factor;
   }

   vars->user_width  = width;
   vars->user_height = height;
   vars->canvas_top_left_c = (canvas_width - width_in_canvas)/2;
   vars->canvas_top_left_r = (canvas_height - height_in_canvas)/2;

   vars->canvas_bottom_right_c = vars->canvas_top_left_c + width_in_canvas;
   vars->canvas_bottom_right_r = vars->canvas_top_left_r + height_in_canvas;

   vars->pixel_size_factor_f = (float) vars->pixel_size_factor;
   current_window->state_vars_init = HOR_TRUE;
   display_params_initialised = HOR_TRUE;

   set_current_state_vars ( vars );
   return HOR_TRUE;
}

Hor_Bool hor_display_canvas_set_params ( Hor_Assoc_Label canvas_label,
					 int width, int height )
{
   Hor_Assoc_Label old_label = current_window_label;
   Hor_Bool        result;

   hor_display_reset_window ( canvas_label );
   result = hor_display_set_params ( width, height );
   hor_display_reset_window ( old_label );

   return result;
}

Hor_Bool hor_display_get_params ( int *top_left_c,     int *top_left_r,
				  int *bottom_right_c, int *bottom_right_r )
{
   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return HOR_FALSE;
   }

   *top_left_c     = canvas_top_left_c;
   *top_left_r     = canvas_top_left_r;
   *bottom_right_c = canvas_bottom_right_c;
   *bottom_right_r = canvas_bottom_right_r;
   return HOR_TRUE;
}

Hor_Bool hor_display_get_dims ( int *width_ptr, int *height_ptr )
{
   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return HOR_FALSE;
   }

   *width_ptr  = user_width;
   *height_ptr = user_height;
   return HOR_TRUE;
}

/*******************
*   void @hor_display_set_cursor ( u_int cursor )
*
*   hor_display_set_cursor() sets the cursor shape in the current
*   display canvas. Allowable cursor identifiers are in the include
*   file <X11/cursorfont.h>.
********************/
void hor_display_set_cursor ( u_int cursor )
{
   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_cursor)",
		  HOR_FATAL );

   XDefineCursor ( display, canvas_window,
		   XCreateFontCursor ( display, cursor ) );
}

/*******************
*   void @hor_display_set_function ( Hor_Display_Function display_function )
*   void @hor_display_set_colour ( u_long colour )
*   void @hor_display_set_line_width ( u_int line_width )
*
*   hor_display_set_function() sets the type of drawing required. Valid values
*   of display_function are currently HOR_DISPLAY_COPY and HOR_DISPLAY_XOR.
*   Default value is HOR_DISPLAY_COPY.
*
*   hor_display_set_colour() sets the drawing colour to one of the colours
*   allocated from the colour map (see colourmap.c). Default colour is 0.
*
*   hor_display_set_line_width() sets the line width for subsequent drawing
*   commands. Default value is 0 (actually gives width of one)
********************/
static Hor_Display_Function current_function = HOR_DISPLAY_COPY;
static u_long               current_colour   = 0;

void hor_display_set_function ( Hor_Display_Function display_function )
{
   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_function)",
		  HOR_FATAL );

   if ( current_colour != HOR_NEUTRAL_COLOUR )
      switch ( display_function )
      {
	 case HOR_DISPLAY_COPY:
	 XSetFunction ( display, graphics_context, GXcopy );
	 break;

	 case HOR_DISPLAY_XOR:
	 XSetFunction ( display, graphics_context, GXxor );
	 break;
      }

   current_function = display_function;
}

void hor_display_set_colour ( u_long colour )
{
   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_colour)",
		  HOR_FATAL );

   if ( colour == HOR_NEUTRAL_COLOUR )
      XSetFunction ( display, graphics_context, GXnoop );
   else
   {
      XSetForeground ( display, graphics_context, colour );
      if ( current_colour == HOR_NEUTRAL_COLOUR )
      {
	 current_colour = colour;
	 hor_display_set_function ( current_function );
	 return;
      }
   }

   current_colour = colour;
}

void hor_display_set_line_width ( u_int line_width )
{
   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_line_width)",
		  HOR_FATAL );

   XSetLineAttributes ( display, graphics_context, line_width,
		        LineSolid, CapButt, JoinMiter );
}

/*******************
*   Hor_Bool @hor_display_image     ( Hor_Image     *imptr,
*                                    int hor_subsample_ratio, ... )
*   Hor_Bool @hor_display_sub_image ( Hor_Sub_Image *sub_imptr,
*                                    int hor_subsample_ratio, ... )
*   Hor_Bool @hor_display_subsampled_image ( int c0, int r0,
*                                           int hor_subsample_c,
*                                           int hor_subsample_r,
*                                           Hor_Image *image, ... )
*
*   hor_display_image() displays an image (imptr) in the current display
*   window, and sets up the display scaling parameters by calling
*   hor_display_set_params(). The subsampling ratio argument should be used if
*   the image has been subsampled from an original and subsequent graphics
*   commands are to be scaled to the original size. Extra arguments are
*   required to convert the internal image to X-format image.
*
*   hor_display_sub_image() displays a sub-image, assuming the display
*   parameters have been set with hor_display_set_params() (maybe via
*   hor_display_image()). The assumption is that the sub-image offsets and
*   dimensions are relative to the full image width and height arguments to
*   hor_display_set_params().
*
*   hor_display_subsampled_image() displays a subsampled image in the
*   coordinate frame of the original image. The offsets are relative to the
*   original image. Again it assumes hor_display_set_params() has been called.
*
*   The variable argument list ... should contain two thresholds of type double
*   if the image to be displayed is of type HOR_INT or HOR_FLOAT. These
*   threshold are used to convert the image into X internal format, and denote
*   image values that will appear black and white on the canvas. All functions
*   return HOR_TRUE on success, HOR_FALSE with error message on failure.
********************/
Hor_Bool hor_display_image ( Hor_Image *imptr, int hor_subsample_ratio, ... )
{
   /* memorise current display settings */
   Hor_Display_Function old_function = current_function;
   u_long               old_colour   = current_colour;

   XImage *ximage;
   va_list ap;

   if ( imptr == NULL )
   {
      hor_errno = HOR_GRAPHICS_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( hor_subsample_ratio < 1 )
   {
      hor_errno = HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO;
      return HOR_FALSE;
   }

   if ( !hor_display_set_params ( hor_subsample_ratio*imptr->width,
				  hor_subsample_ratio*imptr->height ) )
      return HOR_FALSE;

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( 0 );

   va_start ( ap, hor_subsample_ratio );
   ximage = hor_convert_image_to_X_format ( imptr, display, scrnum, depth,
					    hor_subsample_ratio,
					    hor_subsample_ratio,
					    pixel_size_factor, &ap );
   va_end(ap);
   if ( ximage == NULL ) return HOR_FALSE;

   hor_display_clear ( image_background_colour );
   hor_display_X_format_image ( ximage, display, canvas_window,
			        graphics_context,
			        canvas_top_left_c, canvas_top_left_r );

   /* free display format image */
   XDestroyImage ( ximage );
/*   (yzpwj1 = *(((int *)(ximage->data))-2), yzpwj2 = *(((int *)(ximage->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, ximage->data, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3);*/

   hor_display_flush();

   /* restore display settings */
   hor_display_set_function ( old_function );
   hor_display_set_colour ( old_colour );

   return HOR_TRUE;
}

Hor_Bool hor_display_sub_image ( Hor_Sub_Image *sub_imptr,
				 int hor_subsample_ratio, ... )
{
   /* memorise current display settings */
   Hor_Display_Function old_function = current_function;
   u_long               old_colour   = current_colour;

   XImage *ximage;
   int     c0 = sub_imptr->c0, r0 = sub_imptr->r0;
   va_list ap;

   if ( sub_imptr == NULL )
   {
      hor_errno = HOR_GRAPHICS_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( hor_subsample_ratio < 1 )
   {
      hor_errno = HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO;
      return HOR_FALSE;
   }

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return HOR_FALSE;
   }

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( 0 );

   va_start ( ap, hor_subsample_ratio );
   ximage = hor_convert_image_to_X_format ( &sub_imptr->image,
					    display, scrnum, depth,
					    hor_subsample_ratio,
					    hor_subsample_ratio,
					    pixel_size_factor, &ap );
   va_end(ap);
   if ( ximage == NULL ) return HOR_FALSE;

   /* display image on canvas */
   hor_display_set_function ( HOR_DISPLAY_COPY );
   if ( pixel_size_factor > 0 )
      XPutImage ( display, canvas_window, graphics_context,
		  ximage, 0, 0,
		  canvas_top_left_c + c0*hor_subsample_ratio*pixel_size_factor,
		  canvas_top_left_r + r0*hor_subsample_ratio*pixel_size_factor,
		  ximage->width, ximage->height );
   else
      XPutImage(display, canvas_window, graphics_context,
		ximage, 0, 0,
		canvas_top_left_c+c0*hor_subsample_ratio/(-pixel_size_factor),
		canvas_top_left_r+r0*hor_subsample_ratio/(-pixel_size_factor),
		ximage->width, ximage->height );

   /* free display format image */
   XDestroyImage ( ximage );
/*   (yzpwj1 = *(((int *)(ximage->data))-2), yzpwj2 = *(((int *)(ximage->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, ximage->data, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3);*/

   hor_display_flush();

   /* restore display settings */
   hor_display_set_function ( old_function );
   hor_display_set_colour ( old_colour );

   return HOR_TRUE;
}

Hor_Bool hor_display_subsampled_image ( int c0, int r0, int hor_subsample_c,
				        int hor_subsample_r,
				        Hor_Image *image, ... )
{
   /* memorise current display settings */
   Hor_Display_Function old_function = current_function;
   u_long               old_colour   = current_colour;

   XImage *ximage;
   va_list ap;

   if ( image == NULL )
   {
      hor_errno = HOR_GRAPHICS_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( hor_subsample_c < 1 || hor_subsample_r < 1 )
   {
      hor_errno = HOR_GRAPHICS_ILLEGAL_SUBSAMPLING_RATIO;
      return HOR_FALSE;
   }

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return HOR_FALSE;
   }

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( 0 );

   va_start ( ap, image );
   ximage = hor_convert_image_to_X_format ( image, display, scrnum, depth,
					    hor_subsample_c, hor_subsample_r,
					    pixel_size_factor, &ap );
   va_end(ap);
   if ( ximage == NULL ) return HOR_FALSE;

   /* display image on canvas */
   hor_display_set_function ( HOR_DISPLAY_COPY );
   if ( pixel_size_factor > 0 )
      XPutImage(display, canvas_window, graphics_context,
		ximage, 0, 0,
		canvas_top_left_c + (c0 - hor_subsample_c/2)*pixel_size_factor,
		canvas_top_left_r + (r0 - hor_subsample_r/2)*pixel_size_factor,
		ximage->width, ximage->height );
   else
      XPutImage(display, canvas_window, graphics_context,
		ximage, 0, 0,
		canvas_top_left_c+(c0-hor_subsample_c/2)/(-pixel_size_factor),
		canvas_top_left_r+(r0-hor_subsample_r/2)/(-pixel_size_factor),
		ximage->width, ximage->height );

   /* free display format image */
   XDestroyImage ( ximage );
/*   (yzpwj1 = *(((int *)(ximage->data))-2), yzpwj2 = *(((int *)(ximage->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, ximage->data, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3);*/

   hor_display_flush();

   /* restore display settings */
   hor_display_set_function ( old_function );
   hor_display_set_colour ( old_colour );

   return HOR_TRUE;
}

static Hor_Assoc_List  font_list       = NULL;
static Hor_Assoc_Label next_font_label = HOR_ASSOC_START;

/*******************
*   Hor_Assoc_Label  @hor_display_load_font      ( const char *font_name )
*   void             @hor_display_set_font       ( Hor_Assoc_Label font_label )
*   XFontStruct     *@hor_display_get_fontstruct ( Hor_Assoc_Label font_label )
*   void @hor_display_text        ( int c, int r, const char *string )
*   void @hor_display_canvas_text ( int c, int r, const char *string )
*
*   hor_display_load_font() adds the named font to an internal font list for
*   writing text on a canvas, sets the current font to the new font,
*   and returns an label for the new font which can be used to switch back
*   to the font at another time using hor_display_set_font().
*
*   hor_display_set_font() sets the font for canvas text writing to the font
*   with given label.
*
*   hor_display_get_fontstruct() returns the font structure for the font
*   with given label.
*
*   hor_display_text() writes the given string on the current display window
*   using the current font at the given coordinates in the user coordinates.
*
*   hor_display_canvas_text() acts like hor_display_text() but the offsets are
*   in absolute canvas coordinates.
********************/
Hor_Assoc_Label hor_display_load_font ( const char *font_name )
{
   XFontStruct *fontstruct;

   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_load_font)",
		  HOR_FATAL );

   fontstruct = XLoadQueryFont ( display, font_name );
   if ( fontstruct == NULL )
   {
      hor_errno = HOR_GRAPHICS_NON_EXISTENT_FONT_NAME;
      return HOR_ASSOC_ERROR;
   }

   font_list = hor_assoc_insert ( font_list, next_font_label, fontstruct );
   XSetFont ( display, graphics_context, fontstruct->fid );
   return ( next_font_label++ );
}

void hor_display_set_font ( Hor_Assoc_Label font_label )
{
   void *hor_assoc_data;

   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_set_font)",
		  HOR_FATAL );

   if ( (hor_assoc_data = hor_assoc_find ( font_list, font_label )) == NULL )
      hor_error ( "invalid font label (hor_display_set_font)", HOR_FATAL );

   XSetFont (display, graphics_context, ((XFontStruct *) hor_assoc_data)->fid);
}

XFontStruct *hor_display_get_fontstruct ( Hor_Assoc_Label font_label )
{
   void *hor_assoc_data;

   if ( current_window_label == HOR_ASSOC_ERROR )
      hor_error ( "display window not set (hor_display_get_fontstruct)",
		  HOR_FATAL );

   if ( (hor_assoc_data = hor_assoc_find ( font_list, font_label )) == NULL )
      hor_error ( "invalid font label (hor_display_get_fontstruct)",
		  HOR_FATAL );

   return ( (XFontStruct *) hor_assoc_data );
}

void hor_display_text ( int c, int r, const char *string )
{
   XTextItem xtextitem;

   if ( !display_params_initialised )
      hor_error ( "display parameters not set (hor_display_text)", HOR_FATAL );

   if ( font_list == NULL )
      hor_error ( "no fonts loaded (hor_display_text)", HOR_FATAL );

   xtextitem.chars  = (char *) string;
   xtextitem.nchars = strlen ( string );
   xtextitem.delta  = 0;
   xtextitem.font   = None;
   if ( pixel_size_factor > 0 )
      XDrawText ( display, canvas_window, graphics_context,
		  canvas_top_left_c + c*pixel_size_factor,
		  canvas_top_left_r + r*pixel_size_factor, &xtextitem, 1 );
   else
      XDrawText ( display, canvas_window, graphics_context,
		  canvas_top_left_c + c/(-pixel_size_factor),
		  canvas_top_left_r + r/(-pixel_size_factor), &xtextitem, 1 );
}

void hor_display_canvas_text ( int c, int r, const char *string )
{
   XTextItem xtextitem;

   if ( !display_params_initialised )
      hor_error ( "display parameters not set (hor_display_canvas_text)",
		  HOR_FATAL);

   if ( font_list == NULL )
      hor_error ( "no fonts loaded (hor_display_canvas_text)", HOR_FATAL );

   xtextitem.chars  = (char *) string;
   xtextitem.nchars = strlen ( string );
   xtextitem.delta  = 0;
   xtextitem.font   = None;
   XDrawText ( display, canvas_window, graphics_context,
	       canvas_top_left_c + c, canvas_top_left_r + r, &xtextitem, 1 );
}

/*******************
*   void @hor_display_clear ( u_long background_colour )
*
*   Clears the current window to the given colour.
********************/
void hor_display_clear ( u_long background_colour )
{
   /* memorise current display settings */
   Hor_Display_Function old_function = current_function;
   u_long               old_colour = current_colour;

   if ( !display_params_initialised )
      hor_error ( "display parameters not set (hor_display_clear)",
		  HOR_FATAL );

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( background_colour );
   hor_display_fill_canvas_rectangle ( 0, 0, canvas_width, canvas_height );
   /* N.B. there is probably a quicker way to do this using XClearWindow(),
      but I couldn't get it to work. */

   if ( display_params_initialised )
   {
      hor_display_set_colour ( image_border_colour );
      hor_display_draw_canvas_rectangle ( canvas_top_left_c-1,
					  canvas_top_left_r-1,
			       canvas_bottom_right_c - canvas_top_left_c + 2,
			       canvas_bottom_right_r - canvas_top_left_r + 2 );
   }

   /* restore display settings */
   hor_display_set_function ( old_function );
   hor_display_set_colour ( old_colour );
}

/*******************
*   void @hor_display_flush(void)
*
*   Flushes any pending display events.
********************/
void hor_display_flush(void)
{
   if ( !hor_display_initialised )
      hor_error ( "display not initialised (hor_display_flush)", HOR_FATAL );

   XFlush ( display );
}

/*******************
*   void @hor_display_point_convert ( int  canvas_c, int  canvas_r,
*                                    int *c_ptr,    int *r_ptr )
*   void @hor_display_region_convert ( int  canvas_c1, int  canvas_r1,
*                                     int  canvas_c2, int  canvas_r2,
*                                     int *c1_ptr,    int *r1_ptr,
*                                     int *c2_ptr,    int *r2_ptr )
*
*   hor_display_point_convert() converts a point on the current canvas to user
*   coordinates.
*
*   hor_display_region_convert() converts a rectangular region on the current
*   canvas to user coordinates.
********************/
void hor_display_point_convert ( int  canvas_c, int  canvas_r,
				 int *c_ptr,    int *r_ptr )
{
   if ( !display_params_initialised )
      hor_error("display params not initialised (hor_display_point_convert)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
   {
      *c_ptr = (canvas_c - canvas_top_left_c)/pixel_size_factor;
      *r_ptr = (canvas_r - canvas_top_left_r)/pixel_size_factor;
   }
   else
   {
      *c_ptr = (canvas_c - canvas_top_left_c)*(-pixel_size_factor);
      *r_ptr = (canvas_r - canvas_top_left_r)*(-pixel_size_factor);
   }
}

void hor_display_region_convert ( int  canvas_c1, int  canvas_r1,
				  int  canvas_c2, int  canvas_r2,
				  int *c1_ptr,    int *r1_ptr,
				  int *c2_ptr,    int *r2_ptr )
{
   int temp;

   if ( !display_params_initialised )
      hor_error("display params not initialised (hor_display_region_convert)",
		HOR_FATAL );

   if ( canvas_c1 > canvas_c2 )
   { temp = canvas_c1; canvas_c1 = canvas_c2; canvas_c2 = temp; }

   if ( canvas_r1 > canvas_r2 )
   { temp = canvas_r1; canvas_r1 = canvas_r2; canvas_r2 = temp; }

   if ( pixel_size_factor > 0 )
   {
      *c1_ptr = (canvas_c1 - canvas_top_left_c)/pixel_size_factor;
      *r1_ptr = (canvas_r1 - canvas_top_left_r)/pixel_size_factor;
      *c2_ptr = (canvas_c2 - canvas_top_left_c)/pixel_size_factor + 1;
      *r2_ptr = (canvas_r2 - canvas_top_left_r)/pixel_size_factor + 1;
   }
   else
   {
      *c1_ptr = (canvas_c1 - canvas_top_left_c)*(-pixel_size_factor);
      *r1_ptr = (canvas_r1 - canvas_top_left_r)*(-pixel_size_factor);
      *c2_ptr = (canvas_c2 - canvas_top_left_c + 1)*(-pixel_size_factor);
      *r2_ptr = (canvas_r2 - canvas_top_left_r + 1)*(-pixel_size_factor);
   }
}

/*******************
*   Hor_Bool @hor_display_within_image ( int c, int r )
*
*   Returns HOR_TRUE if given point in canvas coordinates lies within the user
*   coordinate frame.
********************/
Hor_Bool hor_display_within_image ( int c, int r )
{
   if ( !display_params_initialised ) return HOR_FALSE;

   if ( c < canvas_top_left_c || c > canvas_bottom_right_c ||
        r < canvas_top_left_r || r > canvas_bottom_right_r )
      return HOR_FALSE;
   else
      return HOR_TRUE;
}

/*******************
*   typedef struct {
*      int c;
*      int r;
*   } @Hor_2D_Vertex;
*
*   void @hor_display_plot ( int c, int r )
*   void @hor_display_line ( float c1, float r1, float c2, float r2 )
*   void @hor_display_draw_rectangle ( int c, int r, int width, int height )
*   void @hor_display_fill_rectangle ( int c, int r, int width, int height )
*   void @hor_display_draw_canvas_rectangle ( int c,     int r,
*                                            int width, int height )
*   void @hor_display_fill_canvas_rectangle ( int c,     int r,
*                                            int width, int height )
*   void @hor_display_fill_polygon ( Hor_List vertex_list ) (list of
*                                                           Hor_2D_Vertex's)
*
*   void @hor_display_highlight_region ( int c0, int r0, int width, int height)
*   void @hor_display_fill_diamond ( int c, int r, float half_size )
*   void @hor_display_draw_circle ( int c, int r, int radius )
*   void @hor_display_draw_circle_actual_size ( int c, int r, int radius )
*   void @hor_display_fill_circle ( int c, int r, int radius )
*   void @hor_display_fill_circle_actual_size ( int c, int r, int radius )
*   void @hor_display_draw_ellipse ( int c, int r, float axis1, float axis2,
*                                    float angle )
*   void @hor_display_draw_ellipse_actual_size ( int c, int r,
*                                               float axis1, float axis2,
*                                               float angle )
*
*   Graphics drawing commands for current graphics window.
*   Most arguments are in user coordinates.
*   Exceptions are the ..._canvas_... functions, which use absolute canvas
*   coordinates, and the ..._actual_size functions, which place the centre of
*   the circle/ellipse at (c,r) in user coordinates, but the size arguments
*   are in canvas coordinates.
********************/
void hor_display_plot ( int c, int r )
{
   if ( !display_params_initialised )
      hor_error("display parameters not initialised (hor_display_plot)",HOR_FATAL);

   if ( pixel_size_factor == 1 )
      XDrawPoint ( display, canvas_window, graphics_context,
		   canvas_top_left_c + c, canvas_top_left_r + r );
   else if ( pixel_size_factor > 0 )
      hor_display_fill_rectangle ( c, r, 1, 1 );
   else
      hor_display_fill_rectangle(c, r, -pixel_size_factor, -pixel_size_factor);
}

void hor_display_line ( float c1, float r1, float c2, float r2 )
{
   if ( !display_params_initialised )
      hor_error("display parameters not initialised (hor_display_line)",HOR_FATAL);

   if ( pixel_size_factor > 0 )
      XDrawLine ( display, canvas_window, graphics_context,
		  canvas_top_left_c + (int) (c1*pixel_size_factor_f),
		  canvas_top_left_r + (int) (r1*pixel_size_factor_f),
		  canvas_top_left_c + (int) (c2*pixel_size_factor_f),
		  canvas_top_left_r + (int) (r2*pixel_size_factor_f) );
   else
      XDrawLine ( display, canvas_window, graphics_context,
		  canvas_top_left_c + (int) (c1/(-pixel_size_factor_f)),
		  canvas_top_left_r + (int) (r1/(-pixel_size_factor_f)),
		  canvas_top_left_c + (int) (c2/(-pixel_size_factor_f)),
		  canvas_top_left_r + (int) (r2/(-pixel_size_factor_f)) );
}

void hor_display_draw_rectangle ( int c, int r, int width, int height )
{
   if ( !display_params_initialised )
      hor_error("display params not initialised (hor_display_draw_rectangle)",
		HOR_FATAL);

   if ( width < 0 || height < 0 ) return;

   if ( pixel_size_factor > 0 )
   {
      width  = width*pixel_size_factor  - 1; if ( width  <= 0 ) width  = 1;
      height = height*pixel_size_factor - 1; if ( height <= 0 ) height = 1;
      XDrawRectangle ( display, canvas_window, graphics_context,
		       canvas_top_left_c + c*pixel_size_factor,
		       canvas_top_left_r + r*pixel_size_factor,
		       width, height );
   }
   else
   {
      width  = width/(-pixel_size_factor)  - 1; if ( width  <= 0 ) width  = 1;
      height = height/(-pixel_size_factor) - 1; if ( height <= 0 ) height = 1;
      XDrawRectangle ( display, canvas_window, graphics_context,
		       canvas_top_left_c + c/(-pixel_size_factor),
		       canvas_top_left_r + r/(-pixel_size_factor),
		       width, height );
   }
}

void hor_display_fill_rectangle ( int c, int r, int width, int height )
{
   if ( !display_params_initialised )
      hor_error("display params not initialised (hor_display_fill_rectangle)",
	      HOR_FATAL );

   if ( width < 0 || height < 0 ) return;

   if ( pixel_size_factor > 0 )
   {
      width  = width*pixel_size_factor;  if ( width  <= 0 ) width  = 1;
      height = height*pixel_size_factor; if ( height <= 0 ) height = 1;
      XFillRectangle ( display, canvas_window, graphics_context,
		       canvas_top_left_c + c*pixel_size_factor,
		       canvas_top_left_r + r*pixel_size_factor,
		       width, height );
   }
   else
   {
      width  = width/(-pixel_size_factor);  if ( width  <= 0 ) width  = 1;
      height = height/(-pixel_size_factor); if ( height <= 0 ) height = 1;
      XFillRectangle ( display, canvas_window, graphics_context,
		       canvas_top_left_c + c/(-pixel_size_factor),
		       canvas_top_left_r + r/(-pixel_size_factor),
		       width, height );
   }
}
		    
void hor_display_draw_canvas_rectangle ( int c, int r, int width, int height )
{
   if ( !display_params_initialised )
      hor_error("display params not set (hor_display_draw_canvas_rectangle)",
		HOR_FATAL );

   if ( width == 0 || height == 0 ) return;

   /* check for negative rectangle dimensions */
   if ( width < 0 )
   { c += width; width = -width; }

   if ( height < 0 )
   { r += height; height = -height; }

   XDrawRectangle ( display, canvas_window,
		    graphics_context, c, r, width, height );
}

void hor_display_fill_canvas_rectangle ( int c, int r, int width, int height )
{
   if ( !display_params_initialised )
      hor_error("display params not set (hor_display_fill_canvas_rectangle)",
		HOR_FATAL );

   /* check for negative rectangle dimensions */
   if ( width < 0 )
   { c += width; width = -width; }

   if ( height < 0 )
   { r += height; height = -height; }

   XFillRectangle ( display, canvas_window,
		    graphics_context, c, r, width, height );
}

void hor_display_fill_polygon ( Hor_List vertex_list )
{

    int
	list_length,
	xpoint_index;

    Hor_List
	list;

    Hor_2D_Vertex
	*vertex_ptr;

    XPoint
	*xpoint_array;

    if (!display_params_initialised)
	hor_error("display params not initialised (hor_display_fill_polygon)",
		  HOR_FATAL);

    list_length = hor_list_size (vertex_list);

    xpoint_array = hor_malloc (list_length * sizeof (xpoint_array [0]));

    xpoint_index = 0;
    for (list = vertex_list; list != NULL; list = list-> next)
    {
	vertex_ptr = (Hor_2D_Vertex *) list-> contents;

	if (pixel_size_factor > 0)
	{
	    xpoint_array [xpoint_index]. x = canvas_top_left_c + vertex_ptr-> c * pixel_size_factor;
	    xpoint_array [xpoint_index]. y = canvas_top_left_r + vertex_ptr-> r * pixel_size_factor;
	    xpoint_index++;
	}

	else
	{
	    xpoint_array [xpoint_index]. x = canvas_top_left_c + vertex_ptr-> c / (-pixel_size_factor);
	    xpoint_array [xpoint_index]. y = canvas_top_left_r + vertex_ptr-> r / (-pixel_size_factor);
	    xpoint_index++;
	}
    }

    XFillPolygon (display, canvas_window, graphics_context, xpoint_array, list_length,
                  Convex, CoordModeOrigin);

    hor_free ((void *) xpoint_array);
}

/* hor_display_highlight_region(): highlights an image region */
void hor_display_highlight_region ( int c0, int r0, int width, int height )
{
   int canvas_c0, canvas_r0, width_in_canvas, height_in_canvas;

   if ( !display_params_initialised )
      hor_error("display params not init'ed (hor_display_highlight_region)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
   {
      canvas_c0 = canvas_top_left_c + c0*pixel_size_factor;
      canvas_r0 = canvas_top_left_r + r0*pixel_size_factor;
      width_in_canvas  = width*pixel_size_factor;
      height_in_canvas = height*pixel_size_factor;
   }
   else
   {
      canvas_c0 = canvas_top_left_c + c0/(-pixel_size_factor);
      canvas_r0 = canvas_top_left_r + r0/(-pixel_size_factor);
      width_in_canvas  = width/(-pixel_size_factor);
      height_in_canvas = height/(-pixel_size_factor);
   }

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( region_border_colour );
   hor_display_draw_canvas_rectangle ( canvas_c0 - 1, canvas_r0 - 1,
				   width_in_canvas + 1, height_in_canvas + 1 );
   hor_display_flush();
}

void hor_display_fill_diamond ( int c, int r, float half_size )
{
   XPoint vertices[4];
   int    chalf_size;

   if ( pixel_size_factor > 0 )
      chalf_size = (int) (half_size*pixel_size_factor + 0.5);
   else
      chalf_size = (int) (half_size/(-pixel_size_factor) + 0.5);

   if ( !display_params_initialised )
      hor_error("display params not initialised (hor_display_fill_diamond)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
   {
      vertices[0].x = canvas_top_left_c + c*pixel_size_factor - chalf_size;
      vertices[0].y = canvas_top_left_r + r*pixel_size_factor;
      vertices[1].x = canvas_top_left_c + c*pixel_size_factor;
      vertices[1].y = canvas_top_left_r + r*pixel_size_factor + chalf_size;
      vertices[2].x = canvas_top_left_c + c*pixel_size_factor + chalf_size;
      vertices[2].y = canvas_top_left_r + r*pixel_size_factor;
      vertices[3].x = canvas_top_left_c + c*pixel_size_factor;
      vertices[3].y = canvas_top_left_r + r*pixel_size_factor - chalf_size;
   }
   else
   {
      vertices[0].x = canvas_top_left_c + c/(-pixel_size_factor) - chalf_size;
      vertices[0].y = canvas_top_left_r + r/(-pixel_size_factor);
      vertices[1].x = canvas_top_left_c + c/(-pixel_size_factor);
      vertices[1].y = canvas_top_left_r + r/(-pixel_size_factor) + chalf_size;
      vertices[2].x = canvas_top_left_c + c/(-pixel_size_factor) + chalf_size;
      vertices[2].y = canvas_top_left_r + r/(-pixel_size_factor);
      vertices[3].x = canvas_top_left_c + c/(-pixel_size_factor);
      vertices[3].y = canvas_top_left_r + r/(-pixel_size_factor) - chalf_size;
   }

   XFillPolygon ( display, canvas_window, graphics_context, vertices, 4,
		  Convex, CoordModeOrigin );
}

void hor_display_draw_circle ( int c, int r, int radius )
{
   if ( !display_params_initialised )
      hor_error("display parameters not initialised (hor_display_draw_circle)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
      XDrawArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + (c-radius)*pixel_size_factor
		 + pixel_size_factor/2,
		 canvas_top_left_r + (r-radius)*pixel_size_factor
		 + pixel_size_factor/2,
		 2*radius*pixel_size_factor, 2*radius*pixel_size_factor,
		 0, 23040 );
   else
      XDrawArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + (c-radius)/(-pixel_size_factor),
		 canvas_top_left_r + (r-radius)/(-pixel_size_factor),
		 2*radius/(-pixel_size_factor), 2*radius/(-pixel_size_factor),
		 0, 23040 );
}
		    
void hor_display_draw_circle_actual_size ( int c, int r, int radius )
{
   if ( !display_params_initialised )
      hor_error("display prms not init. (hor_display_draw_circle_actual_size)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
      XDrawArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + c*pixel_size_factor - radius
		 + pixel_size_factor/2,
		 canvas_top_left_r + r*pixel_size_factor - radius
		 + pixel_size_factor/2,
		 2*radius, 2*radius, 0, 23040 );
   else
      XDrawArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + c/(-pixel_size_factor) - radius,
		 canvas_top_left_r + r/(-pixel_size_factor) - radius,
		 2*radius, 2*radius, 0, 23040 );
}

void hor_display_fill_circle ( int c, int r, int radius )
{
   if ( !display_params_initialised )
      hor_error("display parameters not initialised (hor_display_fill_circle)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
      XFillArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + (c-radius)*pixel_size_factor
		 + pixel_size_factor/2,
		 canvas_top_left_r + (r-radius)*pixel_size_factor
		 + pixel_size_factor/2,
		 2*radius*pixel_size_factor, 2*radius*pixel_size_factor,
		 0, 23040 );
   else
      XFillArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + (c-radius)/(-pixel_size_factor),
		 canvas_top_left_r + (r-radius)/(-pixel_size_factor),
		 2*radius/(-pixel_size_factor), 2*radius/(-pixel_size_factor),
		 0, 23040 );
}
		    
void hor_display_fill_circle_actual_size ( int c, int r, int radius )
{
   if ( !display_params_initialised )
      hor_error("display prms not init. (hor_display_fill_circle_actual_size)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
      XFillArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + c*pixel_size_factor - radius
		 + pixel_size_factor/2,
		 canvas_top_left_r + r*pixel_size_factor - radius
		 + pixel_size_factor/2,
		 2*radius, 2*radius, 0, 23040 );
   else
      XFillArc ( display, canvas_window, graphics_context,
		 canvas_top_left_c + c/(-pixel_size_factor) - radius,
		 canvas_top_left_r + r/(-pixel_size_factor) - radius,
		 2*radius, 2*radius, 0, 23040 );
}

#define SECTIONS 200

/* hor_display_draw_ellipse(): draws an ellipse: angle measures
                               orientation of major axis
			       anticlockwise from x(c) axis */
void hor_display_draw_ellipse ( int c, int r, float axis1, float axis2,
			        float angle )
{
   if ( !display_params_initialised )
      hor_error("display prms not init.(hor_display_draw_ellipse)", HOR_FATAL);

   if ( pixel_size_factor > 0 )
      hor_display_draw_ellipse_actual_size ( c, r, axis1*pixel_size_factor_f,
					           axis2*pixel_size_factor_f,
					     angle );
   else
      hor_display_draw_ellipse_actual_size ( c, r, -axis1/pixel_size_factor_f,
					           -axis2/pixel_size_factor_f,
					     angle );
}

void hor_display_draw_ellipse_actual_size ( int c, int r,
					    float axis1, float axis2,
					    float angle )
{
   float oldc, oldr, newc, newr, ca, sa, theta, theta_unit, mct, mst;
   int   section, c0, r0;

   if ( !display_params_initialised )
      hor_error("display prms not init.(hor_display_draw_ellipse_actual_size)",
		HOR_FATAL );

   if ( pixel_size_factor > 0 )
   {
      c0 = canvas_top_left_c + c*pixel_size_factor;
      r0 = canvas_top_left_r + r*pixel_size_factor;
   }
   else
   {
      c0 = canvas_top_left_c + c/(-pixel_size_factor);
      r0 = canvas_top_left_r + r/(-pixel_size_factor);
   }

   ca = cos(angle);
   sa = sin(angle);
   theta = 0.0;
   mct = axis1*cos(theta);
   mst = axis2*sin(theta);
   oldc =   mct*ca + mst*sa;
   oldr = - mct*sa + mst*ca;
   theta_unit = 2.0*M_PI / (float) SECTIONS;
   for ( theta = theta_unit, section = 1; section <= SECTIONS;
         theta += theta_unit, section++ )
   {
      mct = axis1*cos(theta);
      mst = axis2*sin(theta);
      newc =   mct*ca + mst*sa;
      newr = - mct*sa + mst*ca;
      XDrawLine ( display, canvas_window, graphics_context,
		  c0 + (int) (oldc + 0.5), r0 + (int) (oldr + 0.5),
		  c0 + (int) (newc + 0.5), r0 + (int) (newr + 0.5) );
      oldc = newc; oldr = newr;
   }
}

/*******************
*   u_long @hor_display_get_pixel ( int c, int r )
*
*   Returns the colour pixel value of a specified point on the current canvas.
********************/
u_long hor_display_get_pixel ( int c, int r )
{
   XImage *ximage;
   u_long  pixel;

   if ( !display_params_initialised )
      hor_error("display parameters not initialised (hor_display_get_pixel)",
		HOR_FATAL);

   if ( pixel_size_factor > 0 )
      ximage = XGetImage ( display, canvas_window,
			   canvas_top_left_c + c*pixel_size_factor,
			   canvas_top_left_r + r*pixel_size_factor,
			   1, 1, AllPlanes, ZPixmap );
   else
      ximage = XGetImage ( display, canvas_window,
			   canvas_top_left_c + c/(-pixel_size_factor),
			   canvas_top_left_r + r/(-pixel_size_factor),
			   1, 1, AllPlanes, ZPixmap );

   pixel = XGetPixel ( ximage, 0, 0 );
   XDestroyImage ( ximage );
   return ( pixel );
}

/*******************
*   u_long @hor_display_get_xor_colour(void)
*
*   Returns the colour used in exclusive-or pixel graphics operations.
********************/
u_long hor_display_get_xor_colour(void)
{
   if ( current_window_label == HOR_ASSOC_ERROR )
      hor_error ( "display window not set (hor_display_get_xor_colour)",
		  HOR_FATAL );

   return xor_colour;
}

static Hor_Assoc_List  display_state_list       = NULL;
static Hor_Assoc_Label next_display_state_label = HOR_ASSOC_START;

typedef struct
{
   Hor_Assoc_Label   window_label;
   Window_State_Vars state_vars;
   XImage           *stored_display;
} Display_State;

/*******************
*   Hor_Assoc_Label @hor_display_store_state(void)
*   void            @hor_display_recall_state  ( Hor_Assoc_Label display_label)
*   void            @hor_display_destroy_state ( Hor_Assoc_Label display_label)
*
*   Functions for saving and redisplaying the contents of the current
*   graphics window.
*
*   hor_display_store_state() stores the current window contents and returns a
*   label which can be used to redisplay later
*   by calling hor_display_recall_state().
*
*   hor_display_recall_state() redisplays the previously stored contents of a
*   window on the window that was stored (even if it is not the current
*   window).
*
*   hor_display_destroy_state() frees the memory for a previously stored window
*   state.
********************/
Hor_Assoc_Label hor_display_store_state(void)
{
   Display_State *new_display_state;
   XImage        *ximage;

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return HOR_ASSOC_ERROR;
   }

   ximage = XGetImage ( display, canvas_window,
		        canvas_top_left_c, canvas_top_left_r,
		        canvas_bottom_right_c - canvas_top_left_c,
		        canvas_bottom_right_r - canvas_top_left_r,
		        AllPlanes, ZPixmap );
   if ( ximage == (XImage *) NULL )
   {
      hor_errno = HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED;
      return HOR_ASSOC_ERROR;
   }

   new_display_state = hor_malloc_type ( Display_State );
   new_display_state->window_label   = current_window_label;
   new_display_state->state_vars     = current_window->state_vars;
   new_display_state->stored_display = ximage;
   display_state_list = hor_assoc_insert ( display_state_list,
					   next_display_state_label,
					   new_display_state );
   return ( next_display_state_label++ );
}

void hor_display_recall_state ( Hor_Assoc_Label display_label )
{
   void            *assoc_data = hor_assoc_find ( display_state_list,
						  display_label );
   Display_State   *display_state;
   Graphics_Window *window, *old_window = current_window;
   Hor_Assoc_Label  old_label = current_window_label;

   if ( assoc_data == NULL )
   {
      hor_warning ("non-existent display state %d (hor_display_recall_state)",
		   display_label );
      return;
   }

   display_state = (Display_State *) assoc_data;
   assoc_data = hor_assoc_find ( graphics_window_list,
				 display_state->window_label );
   if ( assoc_data == NULL )
   {
      hor_warning("non-existent graphics window %d (hor_display_recall_state)",
		  display_state->window_label );
      return;
   }

   window = (Graphics_Window *) assoc_data;
   window->state_vars = display_state->state_vars;
   set_current_window ( window, display_state->window_label );

   hor_display_set_function ( HOR_DISPLAY_COPY );
   XPutImage ( display, canvas_window, graphics_context,
	       display_state->stored_display,
	       0, 0, canvas_top_left_c, canvas_top_left_r,
	       canvas_bottom_right_c - canvas_top_left_c,
	       canvas_bottom_right_r - canvas_top_left_r );
   set_current_window ( old_window, old_label );
   hor_display_flush();
}

static void display_free_state ( void *data )
{
   Display_State *display_state = (Display_State *) data;

   XDestroyImage ( display_state->stored_display );
   hor_free ( (void *) display_state );
}

void hor_display_destroy_state ( Hor_Assoc_Label display_label )
{
   if ( hor_assoc_remove(&display_state_list, display_label,display_free_state)
        == HOR_ASSOC_ERROR )
      hor_warning("display state %d doesn't exist (hor_display_destroy_state)",
		  display_label);
}

extern u_long                  *image_to_x_colourmap;
extern Hor_Bool                 x_to_image_init;

extern Hor_X_to_Image_ColourMap x_to_image_colourmap;

static Hor_Image *x_to_image_convert ( XImage *ximage )
{
   Hor_Image *image = NULL;
   int factor, line_offset, r, width = ximage->width, height = ximage->height;

   if ( !x_to_image_init )
      hor_error ( "X-image conversion not initialized (x_to_image_convert)",
		  HOR_FATAL );

   if ( x_to_image_colourmap.uc == NULL ) {
       hor_warning("x_to_image_convert disabled since x_to_image_colourmap not allocated\n");
       return NULL;
   }

   switch ( hor_get_image_display_format() )
   {
      case HOR_U_CHAR:
      {
	 u_char *imp;

	 image = hor_alloc_image ( width, height, HOR_U_CHAR, NULL );
	 switch ( ximage->depth )
	 {
	    case 8:
	    {
	       u_char *ximp, *ximend;

	       factor = ximage->bitmap_pad/8;
	       line_offset = factor-1 - ((width-1) % factor);

	       for ( r = 0, imp = image->array.uc[0],
		     ximp = (u_char *) ximage->data; r < height; r++ )
	       {
		  for ( ximend = ximp + width; ximp != ximend; imp++, ximp++ )
		     *imp = x_to_image_colourmap.uc[*ximp];

		  ximp += line_offset;
	       }
	    }
	    break;

	    default:
	    hor_error ( "illegal depth (x_to_image_convert)", HOR_FATAL );
	    break;
	 }
	 break;
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp;

	 image = hor_alloc_image ( ximage->width, ximage->height,
				   HOR_U_SHORT, NULL );
	 switch ( ximage->depth )
	 {
	    case 8:
	    {
	       u_char *ximp, *ximend;

	       factor = ximage->bitmap_pad/8;
	       line_offset = factor-1 - ((width-1) % factor);

	       for ( r = 0, imp = image->array.us[0],
		     ximp = (u_char *) ximage->data; r < height; r++ )
	       {
		  for ( ximend = ximp + width; ximp != ximend; imp++, ximp++ )
		     *imp = x_to_image_colourmap.us[*ximp];

		  ximp += line_offset;
	       }
	    }
	    break;

	    default:
	    hor_error ( "illegal depth (x_to_image_convert)", HOR_FATAL );
	    break;
	 }
	 break;
      }
      break;

      default:
      hor_error ( "illegal image format (x_to_image_convert)", HOR_FATAL );
      break;
   }

   return image;
}


static XImage *image_to_x_convert ( Hor_Image *image )
{
   XImage *ximage;
   char   *xdata = NULL;
   int     size = image->width*image->height;

   if ( !x_to_image_init )
      hor_error ( "X-image conversion not initialized (image_to_x_convert)",
		  HOR_FATAL );

   if ( image->type != hor_get_image_display_format() )
   {
      hor_errno = HOR_GRAPHICS_WRONG_TYPE_IMAGE;
      return ( (XImage *) NULL );
   }

   switch ( depth )
   {
      case 8:
      {
	 u_char *ximp;

	 xdata = hor_malloc_ntype ( char, size );
	 switch ( hor_get_image_display_format() )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *imp, *imend;

	       for ( ximp = (u_char *) xdata, imp = image->array.uc[0],
		     imend = image->array.uc[0] + size; imp != imend;
		     ximp++, imp++ )
		  *ximp = image_to_x_colourmap[*imp];
	    }
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *imp, *imend;

	       for ( ximp = (u_char *) xdata, imp = image->array.us[0],
		     imend = image->array.us[0] + size; imp != imend;
		     ximp++, imp++ )
		  *ximp = image_to_x_colourmap[*imp];
	    }
	    break;

	    default:
	    hor_error ( "illegal image format (image_to_x_convert)",
		        HOR_FATAL );
	    break;
	 }
      }
      break;

      default:
      hor_error ( "illegal depth (image_to_x_convert)", HOR_FATAL );
      break;
   }

   ximage = XCreateImage ( display, DefaultVisual (display, scrnum),
                           depth, ZPixmap, 0, xdata,
                           image->width, image->height, depth, 0 );
   if ( ximage == (XImage *) NULL )
      hor_errno = HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED;

   return ximage;
}


/*******************
*   Hor_Image *@hor_display_get_image(void)
*
*   Converts canvas contents into internal format image, using same
*   pixel format as hor_display_write_to_file() and
*   hor_display_read_from_file().
********************/
Hor_Image *hor_display_get_image(void)
{
   XImage *ximage;
   Hor_Image  *image;

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return NULL;
   }

   ximage = XGetImage ( display, canvas_window,
		        canvas_top_left_c, canvas_top_left_r,
		        canvas_bottom_right_c - canvas_top_left_c,
		        canvas_bottom_right_r - canvas_top_left_r,
		        AllPlanes, ZPixmap );
   if ( ximage == (XImage *) NULL )
   {
      hor_errno = HOR_GRAPHICS_X_DISPLAY_FUNCTION_FAILED;
      return NULL;
   }

   image = x_to_image_convert ( ximage );
   XDestroyImage ( ximage );
   return image;
}


/*******************
*   Hor_Bool @hor_display_write_to_file  ( const char *base_name )
*   Hor_Bool @hor_display_read_from_file ( const char *base_name )
*
*   File I/O functions for graphics window contents.
*
*   hor_display_write_to_file() writes the contents of the current graphics
*   window to the given file. It can then be read back and displayed using
*   hor_display_read_from_file().
********************/
Hor_Bool hor_display_write_to_file ( const char *base_name )
{
   Hor_Image *image = hor_display_get_image();
   Hor_Bool   result;

   if ( image != NULL )
   {
      result = hor_write_image ( base_name, image );
      hor_free_image ( image );
      return result;
   }
   else
      return HOR_FALSE;
}


Hor_Bool hor_display_read_from_file ( const char *base_name )
{
   XImage *ximage;
   Hor_Image  *image;

   image = hor_read_image ( base_name );
   if ( image == NULL )
   {
      hor_errno = HOR_GRAPHICS_NON_EXISTENT_IMAGE;
      return HOR_FALSE;
   }

   if ( !hor_display_set_params ( image->width, image->height ) )
   {
      hor_free_image ( image );
      return HOR_FALSE;
   }

   if ( (ximage = image_to_x_convert ( image )) == (XImage *) NULL )
   {
      hor_free_image ( image );
      return HOR_FALSE;
   }

   hor_display_clear ( image_background_colour );
   XPutImage ( display, canvas_window, graphics_context,
               ximage, 0, 0, canvas_top_left_c, canvas_top_left_r,
	       ximage->width, ximage->height );
   XDestroyImage ( ximage );
/*   (yzpwj1 = *(((int *)(ximage->data))-2), yzpwj2 = *(((int *)(ximage->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, ximage->data, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3);*/
   hor_free_image ( image );
   return HOR_TRUE;
}

/*******************
*   Hor_List @hor_display_make_movie ( const char *root_name, ... )
*
*   Reads an image sequence in standard file name format root_name??.suffix
*   or root_name.???.suffix, where ??/??? is a two/three-digit number,
*   starting with zero, and suffix is either not present or .mit/.iff/etc.
*   Sets the canvas parameters to prepare to display the images and returns
*   a list of X-format images.
*
*   The variable argument list ... should contain two thresholds of type double
*   if the image to be displayed is of type HOR_INT or HOR_FLOAT. These
*   threshold are used to convert the image into X internal format, and denote
*   image values that will appear black and white on the canvas.
********************/
Hor_List hor_display_make_movie ( const char *root_name, ... )
{
   char       string[100];
   int        image_count;
   Hor_List   movie = NULL;
   Hor_Image *imptr;
   int        init_width = 0, init_height = 0; /* to be checked with every
						  image in sequence */
   Hor_Image_Type init_type = HOR_POINTER;
   va_list    ap;

   if ( current_window_label == HOR_ASSOC_ERROR )
      hor_error ( "display window not set (hor_display_make_movie)",
		  HOR_FATAL );

   for ( image_count = 0;; image_count++ )
   {
      if ( (imptr = hor_read_next_image ( root_name, image_count,
					  HOR_NON_FATAL )) == NULL )
	 break;

      if ( image_count == 0 )
      {
	 init_width  = imptr->width;
	 init_height = imptr->height;
	 init_type   = imptr->type;

	 if ( !hor_display_set_params ( imptr->width, imptr->height ) )
	 {
	    hor_free_image ( imptr );
	    break;
	 }
      }
      else
	 if ( imptr->width != init_width || imptr->height != init_height ||
	      imptr->type  != init_type )
	 {
	    hor_warning ( "incompatible images: movie terminated" );
	    hor_free_image ( imptr );
	    break;
	 }

      va_start ( ap, root_name );
      movie = hor_insert ( movie, (void *)
		 hor_convert_image_to_X_format(imptr, display, scrnum, depth,
					       1, 1, pixel_size_factor, &ap ));
      va_end(ap);

      if ( hor_node_contents(movie) == NULL )
      {
	 hor_warning ( "image %d cannot be displayed: movie terminated",
		   image_count );
	 hor_free_image ( imptr );
	 movie = hor_delete_first ( &movie, NULL );
	 break;
      }
      
      hor_free_image ( imptr );
   }

   switch ( image_count )
   {
      case 0:  hor_message ( "zero length movie" ); return NULL; break;
      case 1:  hor_message ( "one image: not much of a movie" ); break;
      case 2:  hor_message ( "two images: pretty pathetic movie" ); break;
      default: sprintf ( string, "movie consists of %d images", image_count );
	       hor_message ( string ); break;
   }

   hor_display_clear ( image_background_colour );
   return ( hor_reverse ( movie ) );
}

/*******************
*   void @hor_display_make_movie_image ( Hor_Image *image, ... )
*
*   Makes and returns an X-format image from an Horatio format image.
*   Assumes that the canvas parameters have been set to the size of the image.
*   The variable argument list ... should contain two thresholds of type double
*   if the image to be displayed is of type HOR_INT or HOR_FLOAT. These
*   thresholds are used to convert the image into X internal format, and denote
*   image values that will appear black and white on the canvas.
********************/
XImage *hor_display_make_movie_image ( Hor_Image *image, ... )
{
   XImage *ximage;
   va_list ap;

   if ( !display_params_initialised )
   {
      hor_errno = HOR_GRAPHICS_CANVAS_NOT_INITIALIZED;
      return NULL;
   }

   if ( image->width != user_width || image->height != user_height )
   {
      hor_errno = HOR_GRAPHICS_INCOMPATIBLE_IMAGE;
      return NULL;
   }

   va_start ( ap, image );
   ximage = hor_convert_image_to_X_format ( image, display, scrnum, depth,
				        1, 1, pixel_size_factor, &ap );
   va_end(ap);
   return ximage;
}


/*******************
*   Hor_List hor_make_movie_from_images ( Hor_List image_sequence )
*
*   Makes a movie from a list of (Hor_Image *)'s.
********************/
Hor_List hor_make_movie_from_images ( Hor_List image_sequence )
{
   Hor_List  Xsequence = NULL, list_ptr;
   Hor_Image *imptr;

   imptr = hor_node_contents (image_sequence);
   hor_display_set_params(imptr->width, imptr->height);

   for (list_ptr = image_sequence; list_ptr != NULL; list_ptr = list_ptr->next)
   { 
      imptr = (Hor_Image *) list_ptr;
      Xsequence = hor_insert (Xsequence, 
			      (void *) hor_display_make_movie_image (imptr));
   }

   return (hor_reverse (Xsequence));
}

/*******************
*   void @hor_display_show_movie_image ( void *data )
*
*   Displays a single X-format image on the current window.
*   Assumes the display parameters are still those set by
*   hor_display_make_movie().
********************/
void hor_display_show_movie_image ( XImage *ximage )
{
   hor_display_X_format_image ( ximage, display, canvas_window,
			        graphics_context,
			        canvas_top_left_c, canvas_top_left_r );
}

static void destroy_X_image ( void *data )
{
   XDestroyImage ( (XImage *) data );
}

/*******************
*   void @hor_display_destroy_movie ( Hor_List movie )
*
*   Destroys a movie created by hor_display_make_movie, and restores original
*   state of display.
********************/
void hor_display_destroy_movie ( Hor_List movie )
{
   hor_free_list ( movie, destroy_X_image );
/*   (yzpwj1 = *(((int *)(movie[image_count]->data))-2), yzpwj2 = *(((int *)(movie[image_count]->data))-1), printf("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = 1, movie[image_count]->data, hor_test_free(), yzpwj1, yzpwj2, 

__LINE__, __FILE__ ),yzpwj3);*/
}

#if 0
/* functions below are required for linking to succeed. They are never called,
   but just in case they are called they exit with an error. Try without them
   when a new version of gcc/X windows is installed */

size_t mbstowcs (wchar_t *__pwcs, const char *__s, size_t __n)
{
   hor_error ( "called illegal function (mbstowcs)", HOR_FATAL );
   return ( (size_t) NULL ); /* avoids compiler hor_warning */
}

size_t wcstombs (char *__s, const wchar_t *__pwcs, size_t __n)
{
   hor_error ( "called illegal function (wcstombs)", HOR_FATAL );
   return ( (size_t) NULL ); /* avoids compiler hor_warning */
}
#endif
