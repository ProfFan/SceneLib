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
static char sigma_default[HOR_EDIT_STRING_SIZE]              = "1.5";
static char gauss_size_default[HOR_EDIT_STRING_SIZE]         = "4";
static char rc_deriv_threshold_default[HOR_EDIT_STRING_SIZE] = "4.0";
static char t_deriv_threshold_default[HOR_EDIT_STRING_SIZE]  = "0.00001";
static char display_scale_default[HOR_EDIT_STRING_SIZE]      = "6.0";
static char display_increment_default[HOR_EDIT_STRING_SIZE]  = "1";

static Hor_Bool initialised = HOR_FALSE;
static Widget      sigma_text;
static Widget      gauss_size_text;
static Widget      rc_deriv_threshold_text;
static Widget      t_deriv_threshold_text;
static Widget      display_scale_text;
static Widget      display_increment_text;

static void set_defaults(void)
{
   hor_set_widget_value ( sigma_text,              sigma_default );
   hor_set_widget_value ( gauss_size_text,         gauss_size_default );
   hor_set_widget_value ( rc_deriv_threshold_text, rc_deriv_threshold_default);
   hor_set_widget_value ( t_deriv_threshold_text,  t_deriv_threshold_default );
   hor_set_widget_value ( display_scale_text,      display_scale_default );
   hor_set_widget_value ( display_increment_text,  display_increment_default );
}

/*******************
*   Widget @hor_create_image_flow_popup ( String name, Widget parent, ... )
*   Hor_Bool @hor_set_image_flow_defaults ( Hor_FL_Process_Params params,
*                                          float scale, int increment )
*   Hor_Bool @hor_get_image_flow_params ( Hor_FL_Process_Params *proc_prms,
*                                        Hor_FL_Output_Params  *out_prms )
*   void @hor_set_image_flow_colours ( u_long start_colour, u_long line_colour)
*
*   Image flow parameter popup panel functions.
*
*   hor_create_image_flow_popup() creates the popup parameter window and
*                                 returns the button that pops it up. The
*                                 provided name is printed on the button.
*                                 The variable argument list is a
*                                 NULL-terminated list of X resource name/
*                                 value pairs for the button.
*   hor_set_image_flow_defaults() sets the default values of the parameters to
*                                 the given values rather than the built-in
*                                 values.
*   hor_get_image_flow_params()   writes the parameters into the given
*                                 structure.
*   hor_set_image_flow_colours()  sets the colours used to display image flow.
********************/
Widget hor_create_image_flow_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_image_flow_popup)", HOR_FATAL );

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

   last = XtVaCreateManagedWidget ( "Image Flow", labelWidgetClass,
				    popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Sigma:               " );
   sigma_text = last;

   last = hor_create_text ( popup_panel, last, "Gauss Size:          " );
   gauss_size_text = last;

   last = hor_create_text ( popup_panel, last, "xy-Derivative Thres.:" );
   rc_deriv_threshold_text = last;

   last = hor_create_text ( popup_panel, last, "t-Derivative Thres.: " );
   t_deriv_threshold_text = last;

   last = hor_create_text ( popup_panel, last, "Display Scale:       " );
   display_scale_text = last;

   last = hor_create_text ( popup_panel, last, "Display Increment:   " );
   display_increment_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_image_flow_defaults ( Hor_FL_Process_Params params,
				       float scale, int increment )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%f", params.sigma );
   if ( strlen(temp) < max_len ) strcpy ( sigma_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.gauss_size );
   if ( strlen(temp) < max_len ) strcpy ( gauss_size_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.rc_deriv_threshold );
   if ( strlen(temp) < max_len ) strcpy ( rc_deriv_threshold_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.t_deriv_threshold );
   if ( strlen(temp) < max_len ) strcpy ( t_deriv_threshold_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", scale );
   if ( strlen(temp) < max_len ) strcpy ( display_scale_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", increment );
   if ( strlen(temp) < max_len ) strcpy ( display_increment_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_sigma(fp) \
       hor_read_float_param ( hor_get_widget_value(sigma_text), \
			      "image flow sigma", hor_float_pos, \
			      HOR_NON_FATAL, fp )

#define get_gauss_size(ip) \
       hor_read_int_param ( hor_get_widget_value(gauss_size_text), \
			    "image flow gauss size", \
			    hor_int_abs_pos_even, HOR_NON_FATAL, ip )

#define get_rc_deriv_threshold(fp) \
       hor_read_float_param ( hor_get_widget_value(rc_deriv_threshold_text), \
			      "image flow xy-derivative threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_t_deriv_threshold(fp) \
       hor_read_float_param ( hor_get_widget_value(t_deriv_threshold_text), \
			      "image flow t-derivative threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_display_scale(fp) \
       hor_read_float_param ( hor_get_widget_value(display_scale_text), \
			      "image flow display scale", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_display_increment(ip) \
       hor_read_int_param ( hor_get_widget_value(display_increment_text), \
		            "image flow display increment", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

static u_long start_colour, line_colour;
static Hor_Bool   colours_set = HOR_FALSE;

void hor_set_image_flow_colours ( u_long loc_start_colour,
				  u_long loc_line_colour )
{
   start_colour = loc_start_colour;
   line_colour  = loc_line_colour;
   colours_set  = HOR_TRUE;
}

Hor_Bool hor_get_image_flow_params ( Hor_FL_Process_Params *proc_prms,
				     Hor_FL_Output_Params  *out_prms )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_image_flow_params)", HOR_FATAL );

   if ( !get_sigma              ( &proc_prms->sigma )                ||
        !get_gauss_size         ( &proc_prms->gauss_size )           ||
        !get_rc_deriv_threshold ( &proc_prms->rc_deriv_threshold )   ||
        !get_t_deriv_threshold  ( &proc_prms->t_deriv_threshold )    ||
        !get_display_scale      ( &out_prms->scale )                 ||
        !get_display_increment  ( &out_prms->increment ) )
      return HOR_FALSE;

   if ( !colours_set )
      hor_error ( "colours not set (hor_get_image_flow_params)",
		  HOR_FATAL );

   out_prms->start_colour = start_colour;
   out_prms->line_colour  = line_colour;
   return HOR_TRUE;
}
