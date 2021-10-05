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
#include <X11/Xaw/Box.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Box.h>

#include <X11/Xow/Canvas.h>
#include <X11/Xow/PanelTe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Box.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Box.h>

#include <X11/Xow/Canvas.h>
#include <X11/Xow/PanelText.h>
#endif

#include "horatio/global.h"
#include "horatio/tool.h"

#define MAX_X_VARARGS 200

/*******************
*   int @hor_convert_X_args ( va_list *aptr, ArgList *arglist )
*
*   Converts a NULL-terminated variable argument list of X resource name,
*   value pairs into an array of Arg's, which is allocated using hor_malloc()
*   and written into arg. The number of argument pairs is returned.
*   va_start() must be called before calling hor_convert_X_args(), and va_end()
*   afterwards.
********************/
int hor_convert_X_args ( va_list *aptr, ArgList *arglist )
{
   Arg *args = hor_malloc_ntype ( Arg, MAX_X_VARARGS );
   String str;
   int i;

   for ( i = 0; i < MAX_X_VARARGS; i++ )
   {
      if ( (str = va_arg ( *aptr, String )) == NULL ) break;
      XtSetArg ( args[i], str, va_arg(*aptr,XtArgVal) );
   }

   if ( i == MAX_X_VARARGS )
      hor_error ( "too many arguments (hor_convert_X_args)", HOR_FATAL );

   *arglist = args;
   return i;
}

/*******************
*   hor_show_popup ( Widget button, XtPointer client_data,
*                                   XtPointer call_data )
*
*   Pops up a popup window. The client_data argument should be a pointer
*   to a Hor_Popup_Data structure.
********************/
void hor_show_popup ( Widget button, XtPointer client_data,
		                     XtPointer call_data )
{
   Position        x, y;
   Hor_Popup_Data *pd = (Hor_Popup_Data *) client_data;
   Widget          popup_frame = pd->popup_frame;

   XtTranslateCoords ( button, pd->x, pd->y, &x, &y );
   XtVaSetValues ( popup_frame, 
		   XtNx, x,
		   XtNy, y,
		   NULL );
   XtPopup ( popup_frame, XtGrabNone );
}

/*******************
*   void @hor_hide_popup ( Widget    widget,
*                         XtPointer client_data, XtPointer call_data )
*
*   Hides a popup window -- assumes that the button hit to hide it
*   is a 2nd generation child of the popup frame (i.e. a child
*   of the panel contained in the frame).
********************/
void hor_hide_popup ( Widget widget,
		      XtPointer client_data, XtPointer call_data )
{
   XtPopdown(XtParent(XtParent(widget)));
}

/*******************
*   void @hor_create_done_button ( Widget popup_panel, Widget last )
*
*   Places "Done" button in a popup panel, below the provided last widget.
*   The "Done" button calls hor_hide_popup() to make the popup panel disappear.
********************/
void hor_create_done_button ( Widget popup_panel, Widget last )
{
   Widget done_button;

   done_button = XtVaCreateManagedWidget("Done", commandWidgetClass,
					 popup_panel,
					 XtNfromVert,  last,
					 NULL);
   XtAddCallback ( done_button, XtNcallback, hor_hide_popup, NULL );
}

/*******************
*   void @hor_create_reset_done_buttons ( Widget popup_panel, Widget last,
*                                        void (*reset_func) (void) )
*
*   Places "Reset" and "Done" buttons in a popup panel, below the provided last
*   widget. The "Done" button calls hor_hide_popup() to make the popup panel
*   disappear, "Reset" calls user-provided function reset_func().
*   If reset_func() is NULL then only a "Done" button is created.
********************/
void hor_create_reset_done_buttons ( Widget popup_panel, Widget last,
				     void (*reset_func) (void) )
{
   Widget reset_button = NULL, done_button;

   if ( reset_func != NULL )
   {
      reset_button = XtVaCreateManagedWidget ( "Reset", commandWidgetClass,
					       popup_panel,
					       XtNfromVert, last,
					       NULL );
      XtAddCallback ( reset_button, XtNcallback,
		     (XtCallbackProc) reset_func, NULL );
   }

   done_button = XtVaCreateManagedWidget ( "Done", commandWidgetClass,
					   popup_panel,
					   XtNfromVert,  last,
					   XtNfromHoriz, reset_button,
					   NULL);
   XtAddCallback ( done_button, XtNcallback,
		   (XtCallbackProc) hor_hide_popup, NULL );
}
