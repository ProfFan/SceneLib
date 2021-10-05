/* ------------------------------------------------------------------ */
/*                                                                    */
/* Module which contains the procedures to enter the parameters       */
/* used for the orthogonal regression                                 */
/*                                                                    */
/* Code adapted from horatio pc_tool.c  (Phil McLauchlan)             */
/*                                                                    */
/* David Djian. July 1994                                             */
/*                                                                    */
/* lf_tool.c                                                          */
/*                                                                    */
/* ------------------------------------------------------------------ */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/PanelText.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

/* default parameter value specifications */
static char no_points_default[HOR_EDIT_STRING_SIZE]        =  "3";
static char sigma_default[HOR_EDIT_STRING_SIZE]            =  "0.2";
static char fit_thres_default[HOR_EDIT_STRING_SIZE]        =  "0.05";

static Hor_Bool initialised = HOR_FALSE;
static Widget      no_points_text;
static Widget      sigma_text;
static Widget      fit_thres_text;

static void set_defaults(void)
{
   hor_set_widget_value ( no_points_text,          no_points_default );
   hor_set_widget_value ( sigma_text,              sigma_default );
   hor_set_widget_value ( fit_thres_text,          fit_thres_default );
}

/*******************
*   Widget @hor_create_line_fit_popup ( String name, Widget parent, ... )
*   Hor_Bool @hor_set_line_fit_defaults ( OR_Process_Params params )
*   Hor_Bool @hor_get_line_fit_params ( OR_Process_Params *params )
*   void @hor_set_line_fit_colours ( u_long line_colour )
*   void @hor_get_line_fit_colours ( OR_Output_Params *params )
*
*   Orthogonal regression line fitter parameter popup panel functions.
*
*   hor_create_line_fit_popup() creates the popup parameter window and
*                               returns the button that pops it up. The
*                               provided name is printed on the button.
*                               The variable argument list is a
*                               NULL-terminated list of X resource name/
*                               value pairs for the button.
*   hor_set_line_fit_defaults() sets the default values of the
*                               parameters to the given values rather
*                               than the built-in values.
*   hor_get_line_fit_params()   reads the parameters into the given
*                               structure.
*
*   hor_set_line_fit_colours() sets the colours used to display lines.
*   hor_get_line_fit_colours() returns the colours in a structure.
********************/
Widget hor_create_line_fit_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_line_fit_popup)",
		  HOR_FATAL );

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( name, commandWidgetClass,
                                    parent, args, num_args );
   hor_free ( (void *) args );
   popup_frame = XtVaCreatePopupShell ("Params", transientShellWidgetClass,
                                       button, NULL );
   popup_panel = XtVaCreateManagedWidget("Params Popup", formWidgetClass,
					 popup_frame, NULL);

   popup_data->x           = (Position) 30;
   popup_data->y           = (Position)  0;
   popup_data->popup_frame = popup_frame;
   XtAddCallback ( button, XtNcallback, hor_show_popup, popup_data );

   last = XtVaCreateManagedWidget ( "Orthog. regression",
				    labelWidgetClass, popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Number of points to fit:   " );
   no_points_text = last;

   last = hor_create_text ( popup_panel, last, "Edge Std. Dev.:            " );
   sigma_text = last;

   last = hor_create_text ( popup_panel, last, "Fitting Threshold:         " );
   fit_thres_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_line_fit_defaults ( Hor_LF_Process_Params params )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%d", params.no_points );
   if ( strlen(temp) < max_len ) strcpy ( no_points_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.sigma );
   if ( strlen(temp) < max_len ) strcpy ( sigma_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.fit_thres );
   if ( strlen(temp) < max_len ) strcpy ( fit_thres_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_no_points(ip) \
       hor_read_int_param ( hor_get_widget_value(no_points_text), \
			    "Orthog. regression number of points", \
			    hor_int_pos, HOR_NON_FATAL, ip )

#define get_sigma(fp) \
       hor_read_float_param ( hor_get_widget_value(sigma_text), \
			      "Orthog. regression edge std. dev.", \
			      hor_float_pos, HOR_NON_FATAL, fp )

#define get_fit_thres(fp) \
       hor_read_float_param ( hor_get_widget_value(fit_thres_text), \
			      "Orthog. regression fit threshold", \
			      hor_float_pos, HOR_NON_FATAL, fp )

Hor_Bool hor_get_line_fit_params ( Hor_LF_Process_Params *params )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_line_fit_params)",
		  HOR_FATAL );

   if ( !get_no_points          ( &params->no_points )     ||
        !get_sigma              ( &params->sigma )         ||
        !get_fit_thres          ( &params->fit_thres ) )
      return HOR_FALSE;

   return HOR_TRUE;
}

static Hor_LI_Output_Params output_params;
static Hor_Bool             colours_set = HOR_FALSE;

void hor_set_line_fit_colours ( u_long line_colour )
{
   output_params.line_colour = line_colour;
   colours_set = HOR_TRUE;
}

void hor_get_line_fit_colours ( Hor_LI_Output_Params *params )
{
   if ( !colours_set )
      hor_error ( "not initialised (hor_get_line_fit_colours)",
		  HOR_FATAL );

   *params = output_params;
}
