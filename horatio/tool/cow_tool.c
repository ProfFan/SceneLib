/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

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
static char edge_thresh_default[HOR_EDIT_STRING_SIZE]    =  "5000.0";
static char corner_thresh_default[HOR_EDIT_STRING_SIZE]  = "50000.0";
static char patch_size_default[HOR_EDIT_STRING_SIZE]     =   "13";

static Hor_Bool initialised = HOR_FALSE;
static Widget      edge_thresh_text;
static Widget      corner_thresh_text;
static Widget      patch_size_text;

static void set_defaults(void)
{
   hor_set_widget_value ( edge_thresh_text,   edge_thresh_default );
   hor_set_widget_value ( corner_thresh_text, corner_thresh_default );
   hor_set_widget_value ( patch_size_text,    patch_size_default );
}

/*******************
*   Widget @hor_create_wang_corner_popup ( String name, Widget parent )
*   Hor_Bool @hor_set_wang_corner_defaults ( Hor_COW_Process_Params params )
*   bool @hor_get_wang_corner_params ( Hor_COW_Process_Params *params )
*   void @hor_set_wang_corner_colours ( u_long corner_colour )
*   void @hor_get_wang_corner_colours ( Hor_COW_Output_Params *params )
*
*   Wang/Brady corner detector parameter popup panel functions.
*
*   hor_create_wang_corner_popup() creates the popup parameter window. 
*   hor_set_wang_corner_defaults() sets the default values of the
*                                  parameters to the given values rather
*                                  than the built-in values.
*   hor_get_wang_corner_params() reads the parameters into the given structure.
*
*   hor_set_wang_corner_colours() sets the colours used to display corners.
*   hor_get_wang_corner_colours() returns the colours in a structure.
********************/
Widget hor_create_wang_corner_popup ( String name, Widget parent, ... )
{
   Widget  button, last, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   if ( initialised )
      hor_error ( "repeated call (hor_create_wang_corner_popup)", HOR_FATAL );

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( name, commandWidgetClass,
                                    parent, args, num_args );
   hor_free ( (void *) args );
   popup_frame = XtVaCreatePopupShell ("Params", transientShellWidgetClass,
				       button, NULL);
   popup_panel = XtVaCreateManagedWidget("Params Popup", formWidgetClass,
					 popup_frame, NULL);

   popup_data->x           = (Position) 30;
   popup_data->y           = (Position)  0;
   popup_data->popup_frame = popup_frame;
   XtAddCallback ( button, XtNcallback, hor_show_popup, popup_data );

   last = XtVaCreateManagedWidget ( "Wang/Brady Corner Detector",
				    labelWidgetClass, popup_panel, NULL );

   last = hor_create_text ( popup_panel, last, "Edge threshold:   " );
   edge_thresh_text = last;

   last = hor_create_text ( popup_panel, last, "Corner threshold:" );
   corner_thresh_text = last;

   last = hor_create_text ( popup_panel, last, "Image Patch Size:  " );
   patch_size_text = last;

   hor_create_reset_done_buttons ( popup_panel, last, set_defaults );
   set_defaults();
   initialised = HOR_TRUE;
   return button;
}

Hor_Bool hor_set_wang_corner_defaults ( Hor_COW_Process_Params params )
{
   char temp[300];
   int  max_len = HOR_EDIT_STRING_SIZE-1;

   sprintf ( temp, "%f", params.edge_thresh );
   if ( strlen(temp) < max_len ) strcpy ( edge_thresh_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%f", params.corner_thresh );
   if ( strlen(temp) < max_len ) strcpy ( corner_thresh_default, temp );
   else return HOR_FALSE;

   sprintf ( temp, "%d", params.patch_size );
   if ( strlen(temp) < max_len ) strcpy ( patch_size_default, temp );
   else return HOR_FALSE;

   if ( initialised ) set_defaults();
   return HOR_TRUE;
}

#define get_edge_thresh(fp) \
           hor_read_float_param ( hor_get_widget_value(edge_thresh_text), \
				  "Wang/Brady corner edge threshold", \
				  hor_float_pos, HOR_NON_FATAL, fp )

#define get_corner_thresh(fp) \
           hor_read_float_param ( hor_get_widget_value(corner_thresh_text), \
				  "Wang/Brady corner strength threshold", \
				  hor_float_pos, HOR_NON_FATAL, fp )

#define get_patch_size(ip) \
           hor_read_int_param ( hor_get_widget_value(patch_size_text), \
			        "Wang/Brady corner image patch_size", \
			        hor_int_abs_pos_odd, HOR_NON_FATAL, ip )

Hor_Bool hor_get_wang_corner_params ( Hor_COW_Process_Params *params )
{
   if ( !initialised )
      hor_error ( "not initialised (hor_get_wang_corner_params)", HOR_FATAL );

   if ( !get_edge_thresh        ( &params->edge_thresh ) ||
        !get_corner_thresh      ( &params->corner_thresh ) ||
        !get_patch_size         ( &params->patch_size ) )
      return HOR_FALSE;

   return HOR_TRUE;
}

static Hor_CO_Output_Params output_params;
static Hor_Bool             colours_set = HOR_FALSE;

void hor_set_wang_corner_colours ( u_long corner_colour )
{
   output_params.corner_colour = corner_colour;
   colours_set = HOR_TRUE;
}

void hor_get_wang_corner_colours ( Hor_CO_Output_Params *params )
{
   if ( !colours_set )
      hor_error ( "not initialised (hor_get_wang_corner_colours)", HOR_FATAL );

   *params = output_params;
}
