/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/PanelText.h>

#include "horatio/global.h"
#include "horatio/tool.h"

#include "ops_prms.h"

static int adjust_code_on ( int bit_field, int toggle_mask, void *data )
{
   switch ( toggle_mask )
   {
      case CANNY_CODE:
      bit_field &= IMAGE_FLOW_MASK & IMAGE_SEGMENT_MASK & CORRELATION_MASK;
      break;

      case LINE_FIT_CODE:
      bit_field |= CANNY_CODE;
      bit_field &= CANNY_CODE | LINE_FIT_CODE | LINE_MATCH_CODE;
      break;

      case LINE_SEGMENT_CODE:
      bit_field |= CANNY_CODE;
      bit_field &= CANNY_CODE | LINE_SEGMENT_CODE | LINE_MATCH_CODE;
      break;

      case PLESSEY_CORNER_CODE:
      bit_field &= IMAGE_FLOW_MASK & IMAGE_SEGMENT_MASK & CORRELATION_MASK;
      if ( bit_field & (BOG_CORNER_MATCH_CODE | BD_CORNER_MATCH_CODE) )
	 bit_field &= SMITH_CORNER_MASK & WANG_CORNER_MASK;
      break;

      case SMITH_CORNER_CODE:
      bit_field &= IMAGE_FLOW_MASK & IMAGE_SEGMENT_MASK & CORRELATION_MASK;
      if ( bit_field & (BOG_CORNER_MATCH_CODE | BD_CORNER_MATCH_CODE) )
	 bit_field &= PLESSEY_CORNER_MASK & WANG_CORNER_MASK;
      break;

      case WANG_CORNER_CODE:
      bit_field &= IMAGE_FLOW_MASK & IMAGE_SEGMENT_MASK & CORRELATION_MASK;
      if ( bit_field & (BOG_CORNER_MATCH_CODE | BD_CORNER_MATCH_CODE) )
	 bit_field &= PLESSEY_CORNER_MASK & SMITH_CORNER_MASK;
      break;

      case BOG_CORNER_MATCH_CODE:
      if ( bit_field & PLESSEY_CORNER_CODE )
	 bit_field = PLESSEY_CORNER_CODE | BOG_CORNER_MATCH_CODE;
      else if ( bit_field & SMITH_CORNER_CODE )
	 bit_field = SMITH_CORNER_CODE | BOG_CORNER_MATCH_CODE;
      else if ( bit_field & WANG_CORNER_CODE )
	 bit_field = WANG_CORNER_CODE | BOG_CORNER_MATCH_CODE;
      else
	 bit_field = PLESSEY_CORNER_CODE | BOG_CORNER_MATCH_CODE;
      break;

      case BD_CORNER_MATCH_CODE:
      if ( bit_field & PLESSEY_CORNER_CODE )
	 bit_field = PLESSEY_CORNER_CODE | BD_CORNER_MATCH_CODE;
      else if ( bit_field & SMITH_CORNER_CODE )
	 bit_field = SMITH_CORNER_CODE | BD_CORNER_MATCH_CODE;
      else if ( bit_field & WANG_CORNER_CODE )
	 bit_field = WANG_CORNER_CODE | BD_CORNER_MATCH_CODE;
      else
	 bit_field = PLESSEY_CORNER_CODE | BD_CORNER_MATCH_CODE;
      break;

      case LINE_MATCH_CODE:
      if ( bit_field & LINE_FIT_CODE )
	 bit_field = CANNY_CODE | LINE_FIT_CODE | LINE_MATCH_CODE;
      else if ( bit_field & LINE_SEGMENT_CODE )
	 bit_field = CANNY_CODE | LINE_SEGMENT_CODE | LINE_MATCH_CODE;
      else
	 bit_field = CANNY_CODE | LINE_FIT_CODE | LINE_MATCH_CODE;
      break;

      case IMAGE_FLOW_CODE:    bit_field = IMAGE_FLOW_CODE;    break;
      case IMAGE_SEGMENT_CODE: bit_field = IMAGE_SEGMENT_CODE; break;
      case CORRELATION_CODE:   bit_field = CORRELATION_CODE;   break;

      default: break;
   }

   return bit_field;
}

static int adjust_code_off ( int bit_field, int toggle_mask, void *data )
{
   switch ( toggle_mask )
   {
      case CANNY_CODE:
      bit_field &= LINE_FIT_MASK & LINE_SEGMENT_MASK & LINE_MATCH_MASK;
      break;

      case LINE_FIT_CODE:
      case LINE_SEGMENT_CODE:
      bit_field &= LINE_MATCH_MASK;
      break;

      case PLESSEY_CORNER_CODE:
      case SMITH_CORNER_CODE:
      case WANG_CORNER_CODE:
      bit_field &= BOG_CORNER_MATCH_MASK & BD_CORNER_MATCH_MASK;
      break;

      default: break;
   }

   return bit_field;
}

#define OPS_TOGGLE_CHOICES 12
static String ops_toggle_choices[] = {
        "Canny",
	"OR Line Fitter",
	"WD Line Fitter",
        "Plessey Corner",
        "Smith Corner",
        "Wang Corner",
	"Bog Corner Matcher",
	"Dead Corner Matcher",
	"Line Matcher",
	"Image Flow",
	"Image Segment",
	"Correlation",
     };

static Widget ops_togglegroup = NULL;

static Widget ops_panel ( Widget parent, ... )
{
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);
   Widget button, frame, panel, last;

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( "Parameters", commandWidgetClass,
				    parent, args, num_args );
   hor_free ( (void *) args );
   frame = XtVaCreatePopupShell ( "Parameters", transientShellWidgetClass,
				   button, NULL );
   panel = XtVaCreateManagedWidget ( "Parameters", formWidgetClass,
				     frame, NULL );

   popup_data->popup_frame = frame;
   popup_data->x = 40;
   popup_data->y = 100;
   XtAddCallback (button, XtNcallback, hor_show_popup, (XtPointer) popup_data);

   last = XtVaCreateManagedWidget ( "Parameters", labelWidgetClass,
				    panel, NULL );
   /* create parameter pop-up buttons */
   last = hor_create_canny_popup            ( "Canny", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_line_fit_popup         ( "OR Line Fitter", panel,
					      XtNfromVert, last, NULL );
   last = XtVaCreateManagedWidget ( "", commandWidgetClass,
				    panel, XtNfromVert, last, NULL );
   XtSetMappedWhenManaged ( last, FALSE );
   last = hor_create_plessey_corner_popup   ( "Plessey Corner", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_smith_corner_popup     ( "Smith Corner", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_wang_corner_popup      ( "Wang-Brady Corner", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_bog_corner_match_popup ( "Bog Corner Matcher", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_bd_corner_match_popup  ( "Dead Corner Matcher", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_bog_line_match_popup   ( "Line Matcher", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_image_flow_popup       ( "Image Flow", panel,
					      XtNfromVert, last, NULL );
   last = hor_create_image_segment_popup    ( "Image Segment",  panel,
					      XtNfromVert, last, NULL );
   last = hor_create_correlation_popup      ( "Correlation", panel,
					      XtNfromVert, last, NULL );
   hor_create_done_button ( panel, last );
   return button;
}

/*******************
*   Widget @fill_ops_params_panel ( Widget parent, ... )
*
*   Returns a panel filled with Horatio process toggles and parameter popup
*   buttons.
********************/
Widget fill_ops_params_panel ( Widget parent, ... )
{
   Widget  button, panel;
   int     num_args;
   Arg    *args;
   va_list ap;

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   /* create panel for image operation toggle group and parameter popups */
   panel = XtCreateManagedWidget ( "Panel", formWidgetClass, parent,
				   args, num_args );
   hor_free ( (void *) args );

   /* create operations toggle group popup panel */
   button = hor_create_togglegroup_widget ( "Operations", panel,
					    ops_toggle_choices,
					    OPS_TOGGLE_CHOICES, 15, 100,
					    adjust_code_on, adjust_code_off,
					    NULL, NULL );
   ops_togglegroup = button;

   button = ops_panel ( panel, XtNfromHoriz, button, NULL );
   return panel;
}

/*******************
*   int @get_ops_toggle_state(void)
*
*   Returns current state of image processing toggles as a bit-field.
********************/
int get_ops_toggle_state(void)
{
   if ( ops_togglegroup == NULL ) return 0;
   else
      return hor_get_togglegroup ( ops_togglegroup );
}
