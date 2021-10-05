/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#include <X11/Shell.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/PanelTe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/PanelText.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

/* default parameter value specifications */
static char max_dist_default[HOR_EDIT_STRING_SIZE]          =   "10";
static char cos_thresh_default[HOR_EDIT_STRING_SIZE]        =    "0.966";
static char size_thresh_default[HOR_EDIT_STRING_SIZE]       =   "80.0";
static char iterations_default[HOR_EDIT_STRING_SIZE]        =    "3";
static char trajectory_length_default[HOR_EDIT_STRING_SIZE] = "1000";

static Hor_Bool initialised = HOR_FALSE;
static Widget      max_dist_text;
static Widget      cos_thresh_text;
static Widget      size_thresh_text;
static Widget      iterations_text;
static Widget      trajectory_length_text;

static void set_defaults(void)
{
   hor_set_widget_value ( max_dist_text,           max_dist_default );
   hor_set_widget_value ( cos_thresh_text,         cos_thresh_default );
   hor_set_widget_value ( size_thresh_text,        size_thresh_default );
   hor_set_widget_value ( iterations_text,         iterations_default );
   hor_set_widget_value ( trajectory_length_text,  trajectory_length_default );
}

/*******************
*   void @hor_create_bog_line_match_popup ( Widget parent )
*   Hor_Bool @hor_set_bog_line_match_defaults ( Hor_BLM_Process_Params prms,
*                                              int trajectory_length )
*   Hor_Bool @hor_get_bog_line_match_params(Hor_BLM_Process_Params *proc_prms,
*                                          Hor_LM_Output_Params   *out_prms )
*
*   void @hor_set_bog_line_match_colours ( u_long last_colour,
*                                         u_long prev_colour,
*                                         u_long join_colour )
*
*   Line matching parameter popup panel functions.
*
*   hor_create_bog_line_match_popup() creates the popup parameter window and
*                                     returns the button that pops it up. The
*                                     provided name is printed on the button.
*                                     The variable argument list is a NULL-
*                                     terminated list of X resource name/
*                                     value pairs for the button.
*   hor_set_bog_line_match_defaults() sets the default values of the
*                                     parameters to the given values rather
*                                     than the built-in values.
*   hor_get_bog_line_match_params() writes the parameters into the given
*                                   structure.
*   hor_set_bog_line_match_colours() sets colours for displaying line matches.
********************/
Widget hor_create_bog_line_match_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_bog_line_match_popup)",
		  HOR_FATAL );

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( name, commandWidgetClass,
				    parent, args, num_args );
   hor_free ( (void *) args );
   popup_frame = XtVaCreatePopupShell ( "Params", transientShellWidgetClass,
                                        button, NULL );
   popup_panel = XtVaCreateManagedWidget("Params Popup", formWidgetClass,
					 popup_frame, NULL);

   popup_data->x           = (Position) 30;
   popup_data->y           = (Position)  0;
   popup_data->popup_frame = popup_frame;
   XtAddCallback ( button, XtNcallback, hor_show_popup, popup_data );

   last = XtVaCreateManagedWidget ( "Bog-standard Line Matcher",
				    labelWidgetClass, popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Match Window:             " );
   max_dist_text = last;

   last = hor_create_text ( popup_panel, last, "Orient. cosine Threshold: " );
   cos_thresh_text = last;

   last = hor_create_text ( popup_panel, last, "Length Ratio Theshold:    " );
   size_thresh_text = last;

   last = hor_create_text ( popup_panel, last, "Number of Iterations:     " );
   iterations_text = last;

   last = hor_create_text ( popup_panel, last, "Trajectory Display Length:" );
   trajectory_length_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_bog_line_match_defaults ( Hor_BLM_Process_Params params,
					   int trajectory_length )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%d", params.dist_thresh );
   if ( strlen(temp) < max_len ) strcpy ( max_dist_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.cos_thresh );
   if ( strlen(temp) < max_len ) strcpy ( cos_thresh_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.size_thresh );
   if ( strlen(temp) < max_len ) strcpy ( size_thresh_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.iterations );
   if ( strlen(temp) < max_len ) strcpy ( iterations_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", trajectory_length );
   if ( strlen(temp) < max_len ) strcpy ( trajectory_length_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_max_dist(ip) \
       hor_read_int_param ( hor_get_widget_value(max_dist_text), \
			    "bog-standard line matcher max_dist", \
			    hor_int_pos, HOR_NON_FATAL, ip )

#define get_cos_thresh(fp) \
       hor_read_float_param ( hor_get_widget_value(cos_thresh_text), \
			      "bog-standard line matcher orientation cosine threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_size_thresh(fp) \
       hor_read_float_param ( hor_get_widget_value(size_thresh_text), \
			      "bog-standard line matcher size threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_iterations(ip) \
       hor_read_int_param ( hor_get_widget_value(iterations_text), \
			    "bog-standard line matcher number of iterations", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_trajectory_length(ip) \
       hor_read_int_param ( hor_get_widget_value(trajectory_length_text), \
			    "bog-standard line match trajectory display length", \
			    hor_int_pos, HOR_NON_FATAL, ip )

static u_long   last_colour, prev_colour, join_colour;
static Hor_Bool colours_set = HOR_FALSE;

void hor_set_bog_line_match_colours ( u_long loc_last_colour,
				      u_long loc_prev_colour,
				      u_long loc_join_colour )
{
   last_colour = loc_last_colour;
   prev_colour = loc_prev_colour;
   join_colour = loc_join_colour;
   colours_set = HOR_TRUE;
}

Hor_Bool hor_get_bog_line_match_params ( Hor_BLM_Process_Params *proc_prms,
					 Hor_LM_Output_Params   *out_prms )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_bog_line_match_params)",
		  HOR_FATAL );

   if ( !get_max_dist           ( &proc_prms->dist_thresh )          ||
        !get_cos_thresh         ( &proc_prms->cos_thresh )           ||
        !get_size_thresh        ( &proc_prms->size_thresh )          ||
        !get_iterations         ( &proc_prms->iterations )           ||
        !get_trajectory_length  ( &out_prms->trajectory_length ) )
      return HOR_FALSE;

   if ( !colours_set )
      hor_error ( "colours not set (hor_get_bog_line_match_params)",
		  HOR_FATAL );

   out_prms->last_colour = last_colour;
   out_prms->prev_colour = prev_colour;
   out_prms->join_colour = join_colour;
   return HOR_TRUE;
}
