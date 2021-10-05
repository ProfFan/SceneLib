/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/PanelTe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/PanelText.h>
#endif

#include "horatio/global.h"
#include "horatio/tool.h"

static Widget ul_c_velocity, lr_c_velocity;
static Widget ul_r_velocity, lr_r_velocity;

static void set_defaults(void)
{
   hor_set_widget_value ( ul_c_velocity, "0" );
   hor_set_widget_value ( ul_r_velocity, "0" );
   hor_set_widget_value ( lr_c_velocity, "0" );
   hor_set_widget_value ( lr_r_velocity, "0" );
}

/*******************
*   Widget @hor_fill_velocity_panel ( Widget parent, ... )
*   int @hor_get_upper_left_c_velocity(void)
*   int @hor_get_upper_left_r_velocity(void)
*   int @hor_get_lower_right_c_velocity(void)
*   int @hor_get_lower_right_r_velocity(void)
*   void @hor_set_velocity_defaults(void)
*
*   Returns a panel filled with region corner velocity selection panel
*   functions.
********************/
Widget hor_fill_velocity_panel ( Widget parent, ... )
{
   Widget  label, button, frame, panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( "Corner Velocities", commandWidgetClass,
				    parent, args, num_args );
   hor_free ( (void *) args );

   frame = XtVaCreatePopupShell ( "Corner Velocities",
				  transientShellWidgetClass, button, NULL );
   panel = XtVaCreateManagedWidget ( "Corner Velocities", formWidgetClass,
				     frame, NULL );

   popup_data->popup_frame = frame;
   popup_data->x = 30;
   popup_data->y =  0;
   XtAddCallback (button, XtNcallback, hor_show_popup, (XtPointer) popup_data);

   label = XtVaCreateManagedWidget ( "Corner Velocities", labelWidgetClass,
				     panel, NULL );
   
   /* create text windows for corner velocities */
   ul_c_velocity  = XtVaCreateManagedWidget ( "upper left column:  ",
					      panelTextWidgetClass, panel,
					      XtNvalue, "0",
					      XtNfromVert, label,
					      NULL );
   ul_r_velocity  = XtVaCreateManagedWidget ( "upper left row:     ",
					      panelTextWidgetClass, panel,
					      XtNvalue, "0",
					      XtNfromVert, ul_c_velocity,
					      NULL );
   lr_c_velocity  = XtVaCreateManagedWidget ( "lower right column: ",
					      panelTextWidgetClass, panel,
					      XtNvalue, "0",
					      XtNfromVert, ul_r_velocity,
					      NULL );
   lr_r_velocity  = XtVaCreateManagedWidget ( "lower right row:    ",
					      panelTextWidgetClass, panel,
					      XtNvalue, "0",
					      XtNfromVert, lr_c_velocity,
					      NULL );
   hor_create_reset_done_buttons ( panel, lr_r_velocity, set_defaults );
   return button;
}

int hor_get_upper_left_c_velocity(void)
{
   int result;

   if ( sscanf ( XowPanelTextGetValueString(ul_c_velocity),
		 "%d", &result ) != 1 )
   {
      hor_warning ( "could not read upper left column vel.: setting it to 0" );
      return 0;
   }

   return result;
}

int hor_get_upper_left_r_velocity(void)
{
   int result;

   if ( sscanf ( XowPanelTextGetValueString(ul_r_velocity),
		 "%d", &result ) != 1 )
   {
      hor_warning ( "could not read upper left row vel.: setting it to 0" );
      return 0;
   }

   return result;
}

int hor_get_lower_right_c_velocity(void)
{
   int result;

   if ( sscanf ( XowPanelTextGetValueString(lr_c_velocity),
		 "%d", &result ) != 1 )
   {
      hor_warning ( "could not read lower right column vel.: setting it to 0");
      return 0;
   }

   return result;
}

int hor_get_lower_right_r_velocity(void)
{
   int result;

   if ( sscanf ( XowPanelTextGetValueString(lr_r_velocity),
		 "%d", &result ) != 1 )
   {
      hor_warning ( "could not read lower right row vel.: setting it to 0" );
      return 0;
   }

   return result;
}

void hor_set_velocity_defaults(void)
{
   XtVaSetValues ( ul_c_velocity,  XtNvalue, "0", NULL );
   XtVaSetValues ( ul_r_velocity,  XtNvalue, "0", NULL );
   XtVaSetValues ( lr_c_velocity,  XtNvalue, "0", NULL );
   XtVaSetValues ( lr_r_velocity,  XtNvalue, "0", NULL );
}
