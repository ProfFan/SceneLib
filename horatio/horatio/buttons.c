/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/PanelText.h>

#include "horatio/global.h"
#include "horatio/image.h"
#include "horatio/tool.h"

#include "ops_prms.h"
#include "commands.h"

static Widget base_name_widget;

static Widget fill_base_name_panel ( Widget parent, const char *base_name, ...)
{
   int     num_args;
   Arg    *args;
   va_list ap;
   Widget  panel;

   /* count number of variable arguments */
   va_start ( ap, base_name );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   /* create base name panel text window */
   panel = XtCreateManagedWidget ( "Base Name Panel", formWidgetClass,
				   parent, args, num_args );
   hor_free ( (void *) args );
   base_name_widget = XtVaCreateManagedWidget ( "Base Name:",
					        panelTextWidgetClass, panel,
					        XtNvalue, (char *) base_name,
					        NULL );
   return panel;
}

/*******************
*   char *@get_base_name(void)
*
*   Returns currently selected image file base name.
********************/
char *get_base_name(void)
{
   return ( XowPanelTextGetValueString ( base_name_widget ) );
}

static Widget radio_group;

/*******************
*   Hor_Image_Format get_image_format(void)
*
*   Returns currently selected image file output format.
********************/
Hor_Image_Format get_image_format(void)
{
   caddr_t set_radio_data = XawToggleGetCurrent ( radio_group );

   if ( strcmp ( set_radio_data, "IFF" ) == 0 ) return HOR_IFF_FORMAT;
   if ( strcmp ( set_radio_data, "MIT" ) == 0 ) return HOR_MIT_FORMAT;
   hor_error ( "unexpected image format string (get_image_format)", HOR_FATAL);
   return HOR_IFF_FORMAT; /* necessary to avoid compiler hor_warning */
}

static Widget fill_write_image_format_panel ( Widget parent, ... )
{
   Widget         label, last, panel;
   Widget         mit_toggle, iff_toggle;
   String         toggle_translations = "<Btn1Down>,<Btn1Up>:set() notify()";
   XtTranslations translation_table;

   int     num_args;
   Arg    *args;
   va_list ap;

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   panel = XtCreateManagedWidget ( "Image Format Panel", formWidgetClass,
				   parent, args, num_args );
   label = XtVaCreateManagedWidget ( "Output Image Format:", labelWidgetClass,
				     panel, NULL );
   hor_free ( (void *) args );
   last = label;
   translation_table = XtParseTranslationTable ( toggle_translations );

   mit_toggle = XtVaCreateManagedWidget ( "MIT", toggleWidgetClass, panel,
					  XtNfromHoriz, last, XtNstate, True,
					  NULL );
   XtOverrideTranslations ( mit_toggle, translation_table );
   last = mit_toggle;
   radio_group = mit_toggle;
   iff_toggle = XtVaCreateManagedWidget ( "IFF", toggleWidgetClass, panel,
					  XtNradioGroup, radio_group,
					  XtNfromHoriz, last, NULL );
   XtOverrideTranslations ( iff_toggle, translation_table );
   last = iff_toggle;
   return panel;
}

/*******************
*   void @fill_complicated_panel ( XtAppContext app_con,
*                                 Widget panel, const char *base_name,
*                                 XtCallbackProc quit_proc )
*
*   Fills panel with base file name editable text window, image format choice
*   window, operations and parameters panel, corner velocity panel and
*   command button panel.
********************/
void fill_complicated_panel ( XtAppContext app_con,
			      Widget panel, const char *base_name,
			      XtCallbackProc quit_proc )
{
   Widget base_name_panel, write_image_format_panel;
   Widget ops_params_panel, velocity_panel, command_panel;

   /* create base name panel */
   base_name_panel = fill_base_name_panel ( panel, base_name, NULL );

   /* create image format panel */
   write_image_format_panel = fill_write_image_format_panel ( panel,
					       XtNfromVert, base_name_panel,
					       NULL );

   /* create panel for image operations and parameters */
   ops_params_panel = fill_ops_params_panel ( panel,
					 XtNfromVert, write_image_format_panel,
					 NULL );

   /* create velocity panel */
   velocity_panel = hor_fill_velocity_panel ( panel,
					      XtNfromVert, ops_params_panel,
					      NULL );

   /* create command panel */
   command_panel = fill_command_panel ( app_con, panel, quit_proc,
				        XtNfromVert, velocity_panel,
				        NULL );
}
