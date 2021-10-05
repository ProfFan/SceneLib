/***********************************************************
Created by altering the dialog widget to allow a toggle
with a label displaying the result and a menu of the
choices.

Mark Bush <bush@ox.prg>
******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)ToggleChoice.h	1.4  9/11/91*/

#ifndef _ToggleChoice_h
#define _ToggleChoice_h

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
 * ToggleChoice Widget
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

#define XtCHorizontalLayout "HorizontalLayout"
#define XtNhorizontalLayout "horizontalLayout"

#define XtCChoices "Choices"
#define XtNchoices "choices"

#define XtCNumberChoices "NumberChoices"
#define XtNnumberChoices "numberChoices"

typedef struct _XowToggleChoiceClassRec	*XowToggleChoiceWidgetClass;
typedef struct _XowToggleChoiceRec	*XowToggleChoiceWidget;

extern WidgetClass XowtoggleChoiceWidgetClass;

extern void XowToggleChoiceAddButton ARGS((Widget parent,
					   String name,
					   XtCallbackProc function,
					   XtPointer client_data));

extern char *XowToggleChoiceGetValueString ARGS((Widget));

/* provide a Motif-like convenience function */

extern Widget XowCreateToggleChoice ARGS((Widget parent,
					  char *name,
					  ArgList args,
					  int nargs));



/* Compatability */
#define XawToggleChoiceAddButton XowToggleChoiceAddButton
#define XawToggleChoiceGetValueString XowToggleChoiceGetValueString

#endif /* _ToggleChoice_h */
/* DON'T ADD STUFF AFTER THIS #endif */
