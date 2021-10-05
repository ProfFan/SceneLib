/*SccsID (c) Jon Tomb & Mark Bush @(#)Canvas.h	1.4  9/11/91*/
#ifndef _Canvas_h
#define _Canvas_h


#ifdef FUNCPROTO
#define XOW_USE_PROTO
#define ARGS(a) a
#else
#define ARGS(a) ()
#endif

/* To avoid name space clashes I now prefix all Xow widgets/typedefs with Xow
   This will pull in the compatbility header file */

#ifdef XOW_OLDNAMES
#include <X11/Xow/oldnames.h>
#endif

/****************************************************************
 *
 * Canvas widget
 *
 ****************************************************************/

/* Resources:

 Name		     Class		RepType		Default Value
 ----		     -----		-------		-------------
 background	     Background		Pixel		XtDefaultBackground
 border		     BorderColor	Pixel		XtDefaultForeground
 borderWidth	     BorderWidth	Dimension	1
 destroyCallback     Callback		Pointer		NULL
 height		     Height		Dimension	0
 mappedWhenManaged   MappedWhenManaged	Boolean		True
 sensitive	     Sensitive		Boolean		True
 width		     Width		Dimension	0
 x		     Position		Position	0
 y		     Position		Position	0

*/

/* define any special resource names here that are not in <X11/StringDefs.h> */

#define XtNdrawingColor	    "drawingColor"
#define XtNexposeCallback   "exposeCallback"

extern Pixel	CanvasColor ARGS (( Widget ));
extern Font	CanvasFont ARGS (( Widget ));

#define XtNcanvasResource "canvasResource"

#define XtCCanvasResource "CanvasResource"

/* declare specific CanvasWidget class and instance datatypes */

typedef struct _XowCanvasClassRec *	XowCanvasWidgetClass;
typedef struct _XowCanvasRec *		XowCanvasWidget;

/* declare the class constant */

extern WidgetClass XowcanvasWidgetClass;


/* provide a Motif-like convenience function */

extern Widget XowCreateCanvas ARGS((Widget parent,
				    char *name,
				    ArgList args,
				    int nargs));



#endif /* _Canvas_h */
