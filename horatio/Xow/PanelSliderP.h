/***********************************************************
Created by altering the dialog widget to allow horizontal
or vertical arrangement on labels and their values.

Mark Bush <bush@ox.prg>
******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)PanelSliderP.h	1.4  9/11/91*/

/* Private definitions for PanelSlider widget */

#ifndef _PanelSliderP_h
#define _PanelSliderP_h

#include <X11/Xow/PanelSlider.h>
#include <X11/Xaw/FormP.h>

String valStr[2] = {
"  [ %3d ] ",
"  [ %.2f ] ",
};
typedef struct {
      int empty;
} XowPanelSliderClassPart;

typedef struct _XowPanelSliderClassRec {
    CoreClassPart	core_class;
    CompositeClassPart	composite_class;
    ConstraintClassPart	constraint_class;
    FormClassPart	form_class;
    XowPanelSliderClassPart	panelSlider_class;
} XowPanelSliderClassRec;

extern XowPanelSliderClassRec XowpanelSliderClassRec;

typedef struct _XowPanelSliderPart {
    /* resources */
    String	label;		/* description of the panelSlider	*/
    int	       value;		/* for the user response	*/
    XtCallbackList    sliderProc;
    Boolean	horizontalLayout; /* horizontal or vertical?    */
    Boolean	interger; /* horizontal or vertical?    */
    int       sliderLength;
    int       min;
    int       max;
    int	      stepSize;
    int	      steps;
    /* private data */
    Widget	minW;		/* widget to display the icon	*/
    Widget	maxW;		/* widget to display description*/
    Widget	valueW;		/* user response SliderWidget	*/
    Widget      sliderW;
    Widget	labelW;		/* user response SliderWidget	*/    
} XowPanelSliderPart;

typedef struct _XowPanelSliderRec {
    CorePart		core;
    CompositePart	composite;
    ConstraintPart	constraint;
    FormPart		form;
    XowPanelSliderPart		panelSlider;
} XowPanelSliderRec;

typedef struct {
      int empty;
} XowPanelSliderConstraintsPart;

typedef struct _XowPanelSliderConstraintsRec {
    FormConstraintsPart	  form;
    XowPanelSliderConstraintsPart panelSlider;
} XowPanelSliderConstraintsRec, *XowPanelSliderConstraints;

#endif /* _PanelSliderP_h */
