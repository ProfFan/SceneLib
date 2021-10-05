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
static char patch_size_default[HOR_EDIT_STRING_SIZE]     =   "4";
static char patch_density_default[HOR_EDIT_STRING_SIZE]  =   "4";
static char low_threshold_default[HOR_EDIT_STRING_SIZE]  =   "0.05";
static char threshold_step_default[HOR_EDIT_STRING_SIZE] =   "1.3";
static char high_threshold_default[HOR_EDIT_STRING_SIZE] =   "2.0";
static char sd_scale_default[HOR_EDIT_STRING_SIZE]       = "300.0";

static Hor_Bool initialised = HOR_FALSE;
static Widget      patch_size_text;
static Widget      patch_density_text;
static Widget      low_threshold_text;
static Widget      threshold_step_text;
static Widget      high_threshold_text;
static Widget      sd_scale_text;

static void set_defaults ( void )
{
   hor_set_widget_value ( patch_size_text,     patch_size_default );
   hor_set_widget_value ( patch_density_text,  patch_density_default );
   hor_set_widget_value ( low_threshold_text,  low_threshold_default );
   hor_set_widget_value ( threshold_step_text, threshold_step_default );
   hor_set_widget_value ( high_threshold_text, high_threshold_default );
   hor_set_widget_value ( sd_scale_text,       sd_scale_default );
}

/*******************
*   Widget @hor_create_image_segment_popup ( String name, Widget parent, ... )
*   Hor_Bool @hor_set_image_segment_defaults ( Hor_IS_Process_Params params )
*   Hor_Bool @hor_get_image_segment_params ( Hor_IS_Process_Params *params )
*   void @hor_set_image_segment_colours ( u_long loc_thres_colour )
*
*   Region-based image segmentation process parameter popup panel functions.
*
*   hor_create_image_segment_popup() creates the popup parameter window and
*                                    returns the button that pops it up. The
*                                    provided name is printed on the button.
*                                    The variable argument list is a
*                                    NULL-terminated list of X resource name/
*                                    value pairs for the button. 
*   hor_set_image_segment_defaults() sets the default values of the parameters
*                                    to the given values rather than the
*                                    built-in values.
*   hor_get_image_segment_params()   reads the parameters into the given
*                                    structure.
*   hor_set_image_segment_colours() sets the colours used to display image
*                                   segmentation results.
********************/
Widget hor_create_image_segment_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_image_segment_popup)", HOR_FATAL);

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

   last = XtVaCreateManagedWidget ( "Image Segmentation", labelWidgetClass,
				    popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Patch Size:        " );
   patch_size_text = last;

   last = hor_create_text ( popup_panel, last, "Patch Density:     " );
   patch_density_text = last;

   last = hor_create_text ( popup_panel, last, "Low Threshold:     " );
   low_threshold_text = last;

   last = hor_create_text ( popup_panel, last, "Threshold Step:    " );
   threshold_step_text = last;

   last = hor_create_text ( popup_panel, last, "High Threshold:    " );
   high_threshold_text = last;

   last = hor_create_text ( popup_panel, last, "S.D. Display Scale:" );
   sd_scale_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_image_segment_defaults ( Hor_IS_Process_Params params )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%d", params.patch_size );
   if ( strlen(temp) < max_len ) strcpy ( patch_size_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.patch_density );
   if ( strlen(temp) < max_len ) strcpy ( patch_density_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.low_threshold );
   if ( strlen(temp) < max_len ) strcpy ( low_threshold_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.threshold_step );
   if ( strlen(temp) < max_len ) strcpy ( threshold_step_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.high_threshold );
   if ( strlen(temp) < max_len ) strcpy ( high_threshold_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.sd_scale );
   if ( strlen(temp) < max_len ) strcpy ( sd_scale_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_patch_size(ip) \
       hor_read_int_param ( hor_get_widget_value(patch_size_text), \
			    "image segment patch size", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_patch_density(ip) \
       hor_read_int_param ( hor_get_widget_value(patch_density_text), \
			    "image segment patch density", \
			    hor_int_abs_pos, HOR_NON_FATAL, ip )

#define get_low_threshold(fp) \
       hor_read_float_param ( hor_get_widget_value(low_threshold_text), \
			      "image segment low threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_threshold_step(fp) \
       hor_read_float_param ( hor_get_widget_value(threshold_step_text), \
			      "image segment low threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_high_threshold(fp) \
       hor_read_float_param ( hor_get_widget_value(high_threshold_text), \
			      "image segment high threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_sd_scale(fp) \
       hor_read_float_param ( hor_get_widget_value(sd_scale_text), \
			      "image segment s.d. display scale", \
			      hor_float_pos, HOR_NON_FATAL, fp )

static u_long   thres_colour;
static Hor_Bool colours_set = HOR_FALSE;

Hor_Bool hor_get_image_segment_params ( Hor_IS_Process_Params *params )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_image_segment_params)", HOR_FATAL);

   if ( !colours_set )
      hor_error ( "colours not initialized (hor_get_image_segment_params)",
		  HOR_FATAL );

   if ( !get_patch_size     ( &params->patch_size )     ||
        !get_patch_density  ( &params->patch_density )  ||
        !get_low_threshold  ( &params->low_threshold )  ||
        !get_threshold_step ( &params->threshold_step ) ||
        !get_high_threshold ( &params->high_threshold ) ||
        !get_sd_scale       ( &params->sd_scale ) )
      return HOR_FALSE;

   if ( params->patch_size % params->patch_density != 0 )
   {
      hor_error ("patch size must be a multiple of patch density",
		  HOR_NON_FATAL );
      return HOR_FALSE;
   }

   if ( params->low_threshold >= params->high_threshold )
   {
      hor_error ( "low threshold must be smaller than high threshold",
		  HOR_NON_FATAL );
      return HOR_FALSE;
   }

   params->thres_colour = thres_colour;
   return HOR_TRUE;
}

void hor_set_image_segment_colours ( u_long loc_thres_colour )
{
   thres_colour = loc_thres_colour;
   colours_set = HOR_TRUE;
}
