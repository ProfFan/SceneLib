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
#include "horatio/process.h"
#include "horatio/tool.h"

/* default parameter value specifications */
static char patch_size_default[HOR_EDIT_STRING_SIZE]    = "20";
static char patch_density_default[HOR_EDIT_STRING_SIZE] =  "2";
static char max_motion_default[HOR_EDIT_STRING_SIZE]    =  "5";
static char display_scale_default[HOR_EDIT_STRING_SIZE] =  "1.0";

static Hor_Bool initialised = HOR_FALSE;
static Widget      patch_size_text;
static Widget      patch_density_text;
static Widget      max_motion_text;
static Widget      display_scale_text;

static void set_defaults(void)
{
   hor_set_widget_value ( patch_size_text,     patch_size_default );
   hor_set_widget_value ( patch_density_text,  patch_density_default );
   hor_set_widget_value ( max_motion_text,     max_motion_default );
   hor_set_widget_value ( display_scale_text,  display_scale_default );
}

/*******************
*   Widget @hor_create_correlation_popup ( String name, Widget parent, ... )
*   Hor_Bool @hor_set_correlation_defaults ( Hor_CL_Process_Params params,
*                                           float scale )
*   Hor_Bool @hor_get_correlation_params ( Hor_CL_Process_Params *params )
*   void @hor_set_correlation_colours ( u_long display_colour )
*
*   Correlation process parameter popup panel functions.
*
*   hor_create_correlation_popup() creates the popup parameter window and
*                                  returns the button that pops it up. The
*                                  provided name is printed on the button.
*                                  The variable argument list is a
*                                  NULL-terminated list of X resource name/
*                                  value pairs for the button.
*   hor_set_correlation_defaults() sets the default values of the parameters to
*                                  the given values rather than the built-in
*                                  values.
*   hor_get_correlation_params()   reads the parameters into the given
*                                 structure.
*   hor_set_correlation_colours() sets the colours used to display
*                                 correlation results.
********************/
Widget hor_create_correlation_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_correlation_popup)", HOR_FATAL );

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( name, commandWidgetClass,
				    parent, args, num_args );
   hor_free ( (void *) args );
   popup_frame = XtVaCreatePopupShell ( "Params", transientShellWidgetClass,
				        button, NULL );
   popup_panel = XtVaCreateManagedWidget ( "Params Popup", formWidgetClass,
					   popup_frame, NULL );

   popup_data->x           = (Position) 30;
   popup_data->y           = (Position)  0;
   popup_data->popup_frame = popup_frame;
   XtAddCallback ( button, XtNcallback, hor_show_popup, popup_data );

   last = XtVaCreateManagedWidget ( "Correlation", labelWidgetClass,
				    popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Patch Size:        " );
   patch_size_text = last;

   last = hor_create_text ( popup_panel, last, "Patch Density:     " );
   patch_density_text = last;

   last = hor_create_text ( popup_panel, last, "Max. Motion:       " );
   max_motion_text = last;

   last = hor_create_text ( popup_panel, last, "Display Scale:     " );
   display_scale_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_correlation_defaults ( Hor_CL_Process_Params params,
				        float scale )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%d", params.patch_size );
   if ( strlen(temp) < max_len ) strcpy ( patch_size_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.patch_density );
   if ( strlen(temp) < max_len ) strcpy ( patch_density_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.max_motion );
   if ( strlen(temp) < max_len ) strcpy ( max_motion_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", scale );
   if ( strlen(temp) < max_len ) strcpy ( display_scale_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_patch_size(ip) \
       hor_read_int_param ( hor_get_widget_value(patch_size_text), \
			    "correlation patch size", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_patch_density(ip) \
       hor_read_int_param ( hor_get_widget_value(patch_density_text), \
			    "correlation patch density", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_max_motion(ip) \
       hor_read_int_param ( hor_get_widget_value(max_motion_text), \
			    "correlation maximum motion", \
			    hor_int_pos, HOR_NON_FATAL, ip )

#define get_display_scale(fp) \
       hor_read_float_param ( hor_get_widget_value(display_scale_text), \
			      "correlation display scale", \
			      hor_float_pos, HOR_NON_FATAL, fp )

static u_long   display_colour;
static Hor_Bool colours_set = HOR_FALSE;

Hor_Bool hor_get_correlation_params ( Hor_CL_Process_Params *params )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_correlation_params)", HOR_FATAL );

   if ( !colours_set )
      hor_error ( "colours not initialized (hor_get_correlation_params)",
		  HOR_FATAL );

   if ( !get_patch_size     ( &params->patch_size )     ||
        !get_patch_density  ( &params->patch_density )  ||
        !get_max_motion     ( &params->max_motion )     ||
        !get_display_scale  ( &params->display_scale ) )
      return HOR_FALSE;

   if ( params->patch_size % params->patch_density != 0 )
   {
      hor_error ( "patch size must be a multiple of patch density",
		  HOR_NON_FATAL );
      return HOR_FALSE;
   }

   params->display_colour = display_colour;
   return HOR_TRUE;
}

void hor_set_correlation_colours ( u_long loc_display_colour )
{
   display_colour = loc_display_colour;
   colours_set = HOR_TRUE;
}
