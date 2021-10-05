/***********************************************************
Created by altering the dialog widget to allow horizontal
or vertical arrangement on labels and their values.


******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)PanelSlider.h	1.5  9/11/91*/

#ifndef _PanelSlider_h
#define _PanelSlider_h

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

#ifndef _Slider_h
#include <X11/Xow/Slider.h>
#endif
/***********************************************************************
 *
 * PanelSlider Widget
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

#define XtCMin "Min"

#define XtNmin "min"

#define XtCMax "Max"

#define XtNmax "max"

#define XtCHorizontalLayout "HorizontalLayout"

#define XtNhorizontalLayout "horizontalLayout"
#define XtNsliderLength "sliderLength"
#define XtCSliderLength "SliderLength"

#define XtNstepSize  "stepSize"
#define XtCStepSize  "StepSize"
#define XtNsteps "steps"
#define XtCSteps "Steps"
#define XtNsliderLength "sliderLength"
#define XtCSliderLength "SliderLength"

#define XtNsliderProc "sliderProc"
#define XtCSliderProc "SliderProc"

typedef struct _XowPanelSliderClassRec	*XowPanelSliderWidgetClass;
typedef struct _XowPanelSliderRec	*XowPanelSliderWidget;

extern WidgetClass XowpanelSliderWidgetClass;

extern void XowPanelSliderAddButton ARGS ((Widget Parent,
					   String name,
					   XtCallbackProc function,
					   XtPointer client_data ));

extern char *XowPanelSliderGetValueString ARGS(( Widget )); 

extern Widget XowCreatePanelSlider ARGS((Widget parent, 
					 char *name, 
					 ArgList args, 
					 int nargs));


#endif /* _PanelSlider_h */
/* DON'T ADD STUFF AFTER THIS #endif */
