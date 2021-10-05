/***********************************************************
Created by altering the dialog widget to allow horizontal
or vertical arrangement on labels and their values.

Mark Bush <bush@ox.prg>
&
Jon Tombs <jon@ox.robots>
******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)PanelText.h	1.5  9/11/91*/

#ifndef _PanelText_h
#define _PanelText_h


#ifdef FUNCPROTO
#define XOW_USE_PROTO
#define ARGS(a) a
#else
#define ARGS(a) ()
#endif

#include <X11/Xaw/Form.h>

/* To avoid name space clashes I now prefix all Xow widgets/typedefs with Xow
   This will pull in the compatbility header file */

#ifdef XOW_OLDNAMES
#include <X11/Xow/oldnames.h>
#endif

/***********************************************************************
 *
 * PanelText Widget
 *
 ***********************************************************************/

/* Parameters:

 Name		     Class		RepType		Default Value
 ----		     -----		-------		-------------
 background	     Background		Pixel		XtDefaultBackground
 border		     BorderColor	Pixel		XtDefaultForeground
 borderWidth	     BorderWidth	Dimension	1
 destroyCallback     Callback		Pointer		NULL
 height		     Height		Dimension	computed at create
 icon		     Icon		Pixmap		0
 label		     Label		String		NULL
 mappedWhenManaged   MappedWhenManaged	Boolean		True
 sensitive	     Sensitive		Boolean		True
 value		     Value		String		NULL
 width		     Width		Dimension	computed at create
 x		     Position		Position	0
 y		     Position		Position	0

*/

#define XtCIcon "Icon"

#define XtNicon "icon"

#define XtNtextBorderWidth "textBorderWidth"
#define XtCTextBorderWidth "TextBorderWidth"

#define XtNtextLength "textLength"
#define XtCTextLength "TextLength"

#define XtCHorizontalLayout "HorizontalLayout"

#define XtNhorizontalLayout "horizontalLayout"

typedef struct _XowPanelTextClassRec	*XowPanelTextWidgetClass;
typedef struct _XowPanelTextRec	*XowPanelTextWidget;

extern WidgetClass XowpanelTextWidgetClass;

extern void XowPanelTextAddButton ARGS ((Widget parent,
					    String name,
					    XtCallbackProc function,
					    XtPointer client_data));

/* provide a Motif-like convenience function */

extern Widget XowCreatePanelText ARGS((Widget parent,
				       char *name,
				       ArgList args,
				       int nargs));



extern char *XowPanelTextGetValueString ARGS(( Widget ));

/* Compatability */
#define XawPanelTextAddButton XowPanelTextAddButton
#define XawPanelTextGetValueString XowPanelTextGetValueString

#endif /* _PanelText_h */
/* DON'T ADD STUFF AFTER THIS #endif */

