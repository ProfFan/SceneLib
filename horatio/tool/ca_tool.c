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
#include <X11/Xow/oldnames.h>
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
static char sigma_default[HOR_EDIT_STRING_SIZE]        =  "1.0";
static char gauss_size_default[HOR_EDIT_STRING_SIZE]   =  "3";
static char low_thres_default[HOR_EDIT_STRING_SIZE]    =  "2.0";
static char high_thres_default[HOR_EDIT_STRING_SIZE]   =  "6.0";
static char length_thres_default[HOR_EDIT_STRING_SIZE] = "10";

static Hor_Bool initialised = HOR_FALSE;
static Widget sigma_text;
static Widget gauss_size_text;
static Widget low_thres_text;
static Widget high_thres_text;
static Widget length_thres_text;

static void set_defaults(void)
{
   hor_set_widget_value ( sigma_text,            sigma_default );
   hor_set_widget_value ( gauss_size_text,       gauss_size_default );
   hor_set_widget_value ( low_thres_text,        low_thres_default );
   hor_set_widget_value ( high_thres_text,       high_thres_default );
   hor_set_widget_value ( length_thres_text,     length_thres_default );
}

/*******************
*   Widget @hor_create_canny_popup ( String name, Widget parent, ... )
*   Hor_Bool @hor_set_canny_defaults ( Hor_CA_Process_Params params )
*   Hor_Bool @hor_get_canny_params ( Hor_CA_Process_Params *params )
*
*   void @hor_set_canny_colours ( u_long string_colour,
*                                u_long discard_colour,
*                                u_long left_term_colour,
*                                u_long right_term_colour)
*   void @hor_get_canny_colours ( Hor_CA_Output_Params *params )
*
*   Canny operator parameter popup panel functions.
*
*   hor_create_canny_popup() creates the popup parameter window and returns
*                            the button that pops it up. The provided name
*                            is printed on the button. The variable argument
*                            list is a NULL-terminated list of X resource name/
*                            value pairs for the button.
*   hor_set_canny_defaults() sets the default values of the parameters to
*                            the given values rather than the built-in values.
*   hor_get_canny_params()   reads the Canny parameters into the given
*                            structure.
*   hor_set_canny_colours() sets the colours used to display Canny edge
*                           strings.
*   hor_get_canny_colours() returns the colours in a structure.
********************/
Widget hor_create_canny_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_canny_popup)", HOR_FATAL );

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

   last = XtVaCreateManagedWidget ( "Canny Edge Detector", labelWidgetClass,
				    popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Sigma:           " );
   sigma_text = last;

   last = hor_create_text ( popup_panel, last, "Gauss Mask Size: " );
   gauss_size_text = last;

   last = hor_create_text ( popup_panel, last, "Low Threshold:   " );
   low_thres_text = last;

   last = hor_create_text ( popup_panel, last, "High Threshold:  " );
   high_thres_text = last;

   last = hor_create_text ( popup_panel, last, "Length Threshold:" );
   length_thres_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_canny_defaults ( Hor_CA_Process_Params params )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%f", params.sigma );
   if ( strlen(temp) < max_len ) strcpy ( sigma_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.gauss_size );
   if ( strlen(temp) < max_len ) strcpy ( gauss_size_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.low_thres );
   if ( strlen(temp) < max_len ) strcpy ( low_thres_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.high_thres );
   if ( strlen(temp) < max_len ) strcpy ( high_thres_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.length_thres );
   if ( strlen(temp) < max_len ) strcpy ( length_thres_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_sigma(fp) \
       hor_read_float_param ( hor_get_widget_value(sigma_text), \
			      "canny sigma", hor_float_pos, HOR_NON_FATAL, fp )

#define get_gauss_size(ip) \
       hor_read_int_param ( hor_get_widget_value(gauss_size_text), \
			    "canny gaussian mask size", \
			    hor_int_pos, HOR_NON_FATAL, ip )

#define get_low_thres(fp) \
       hor_read_float_param ( hor_get_widget_value(low_thres_text), \
			      "canny low threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_high_thres(fp) \
       hor_read_float_param ( hor_get_widget_value(high_thres_text), \
			      "canny high threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_length_thres(ip) \
       hor_read_int_param ( hor_get_widget_value(length_thres_text), \
			    "canny string length threshold", \
			    hor_int_pos, HOR_NON_FATAL, ip )

Hor_Bool hor_get_canny_params ( Hor_CA_Process_Params *params )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_canny_params)", HOR_FATAL );

   if ( !get_sigma            ( &params->sigma )      ||
        !get_gauss_size       ( &params->gauss_size ) ||
        !get_low_thres        ( &params->low_thres )  ||
        !get_high_thres       ( &params->high_thres ) ||
        !get_length_thres     ( &params->length_thres ) )
      return HOR_FALSE;

   if ( params->low_thres >= params->high_thres )
   {
      hor_error ( "low threshold must be smaller than high threshold",
		  HOR_NON_FATAL );
      return HOR_FALSE;
   }

   return HOR_TRUE;
}

static Hor_ED_Output_Params output_params;
static Hor_Bool             colours_set = HOR_FALSE;

void hor_set_canny_colours ( u_long string_colour,    u_long discard_colour,
			     u_long left_term_colour, u_long right_term_colour)
{
   output_params.string_colour     = string_colour;
   output_params.discard_colour    = discard_colour;
   output_params.left_term_colour  = left_term_colour;
   output_params.right_term_colour = right_term_colour;
   colours_set = HOR_TRUE;
}

void hor_get_canny_colours ( Hor_ED_Output_Params *params )
{
   if ( !colours_set )
      hor_error ( "not initialised (hor_get_canny_colours)", HOR_FATAL );

   *params = output_params;
}
