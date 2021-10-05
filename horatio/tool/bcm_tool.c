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
static char range_default[HOR_EDIT_STRING_SIZE]              =    "5";
static char imdiff_ratio_thres_default[HOR_EDIT_STRING_SIZE] =    "0.3";
static char iterations_default[HOR_EDIT_STRING_SIZE]         =    "3";
static char trajectory_length_default[HOR_EDIT_STRING_SIZE]  = "1000";

static Hor_Bool initialised = HOR_FALSE;
static Widget      range_text;
static Widget      imdiff_ratio_thres_text;
static Widget      iterations_text;
static Widget      trajectory_length_text;

static void set_defaults(void)
{
   hor_set_widget_value ( range_text,              range_default );
   hor_set_widget_value ( imdiff_ratio_thres_text, imdiff_ratio_thres_default);
   hor_set_widget_value ( iterations_text,         iterations_default );
   hor_set_widget_value ( trajectory_length_text,  trajectory_length_default );
}

/*******************
*   void @hor_create_bog_corner_match_popup ( Widget parent )
*   Hor_Bool @hor_set_bog_corner_match_defaults ( Hor_BCM_Process_Params prms,
*                                                int trajectory_length )
*   Hor_Bool @hor_get_bog_corner_match_params(Hor_BCM_Process_Params *proc_prms,
*                                            Hor_CM_Output_Params   *out_prms )
*
*   void @hor_set_bog_corner_match_colours ( u_long trajectory_colour,
*                                           u_long dot_colour,
*                                           u_long old_traj_colour )
*
*   Corner matching parameter popup panel functions.
*
*   hor_create_bog_corner_match_popup() creates the popup parameter window and
*                                       returns the button that pops it up. The
*                                       provided name is printed on the button.
*                                       The variable argument list is a NULL-
*                                       terminated list of X resource name/
*                                       value pairs for the button.
*   hor_get_bog_corner_match_params()   writes the parameters into the given
*                                       structure.
*   hor_set_bog_corner_match_colours()  sets colours for displaying corner
*                                       matches.
********************/
Widget hor_create_bog_corner_match_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_bog_corner_match_popup)",
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

   last = XtVaCreateManagedWidget ( "Bog-standard Corner Matcher",
				    labelWidgetClass, popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Range:                    " );
   range_text = last;

   last = hor_create_text ( popup_panel, last, "Image Diff. Ratio Thres.: " );
   imdiff_ratio_thres_text = last;

   last = hor_create_text ( popup_panel, last, "Number of Iterations:     " );
   iterations_text = last;

   last = hor_create_text ( popup_panel, last, "Trajectory Display Length:" );
   trajectory_length_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_bog_corner_match_defaults ( Hor_BCM_Process_Params params,
					     int trajectory_length )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%d", params.range );
   if ( strlen(temp) < max_len ) strcpy ( range_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.imdiff_ratio_thres );
   if ( strlen(temp) < max_len ) strcpy ( imdiff_ratio_thres_default, temp );
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

#define get_range(ip) \
       hor_read_int_param ( hor_get_widget_value(range_text), \
			    "bog-standard corner match range", \
			    hor_int_pos, HOR_NON_FATAL, ip )

#define get_imdiff_ratio_thres(fp) \
       hor_read_float_param ( hor_get_widget_value(imdiff_ratio_thres_text), \
			      "bog-standard corner match xy-derivative threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_iterations(ip) \
       hor_read_int_param ( hor_get_widget_value(iterations_text), \
			    "bog-standard corner match number of iterations", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_trajectory_length(ip) \
       hor_read_int_param ( hor_get_widget_value(trajectory_length_text), \
			    "bog-standard corner match trajectory display length", \
			    hor_int_pos, HOR_NON_FATAL, ip )

static u_long   trajectory_colour, dot_colour, old_traj_colour;
static Hor_Bool colours_set = HOR_FALSE;

void hor_set_bog_corner_match_colours ( u_long loc_trajectory_colour,
				        u_long loc_dot_colour,
				        u_long loc_old_traj_colour )
{
   trajectory_colour = loc_trajectory_colour;
   dot_colour        = loc_dot_colour;
   old_traj_colour   = loc_old_traj_colour;
   colours_set = HOR_TRUE;
}

Hor_Bool hor_get_bog_corner_match_params ( Hor_BCM_Process_Params *proc_prms,
					   Hor_CM_Output_Params  *out_prms )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_bog_corner_match_params)",
		  HOR_FATAL );

   if ( !get_range              ( &proc_prms->range )                ||
        !get_imdiff_ratio_thres ( &proc_prms->imdiff_ratio_thres )   ||
        !get_iterations         ( &proc_prms->iterations )           ||
        !get_trajectory_length  ( &out_prms->trajectory_length ) )
      return HOR_FALSE;

   if ( !colours_set )
      hor_error ( "colours not set (hor_get_bog_corner_match_params)",
		  HOR_FATAL );

   out_prms->trajectory_colour = trajectory_colour;
   out_prms->dot_colour        = dot_colour;
   /* Commented out by Ian 28/9/95
      out_prms->old_traj_colour   = old_traj_colour; */
   return HOR_TRUE;
}
