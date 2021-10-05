/***********************************************************
Created by altering the dialog widget to allow horizontal
or vertical arrangement on labels and their values.

Mark Bush <bush@ox.prg>
******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)PanelTextP.h	1.5  9/11/91*/

/* Private definitions for PanelText widget */

#ifndef _PanelTextP_h
#define _PanelTextP_h

#include <X11/Xow/PanelText.h>
#include <X11/Xaw/FormP.h>

typedef struct {
      int empty;
} XowPanelTextClassPart;

typedef struct _XowPanelTextClassRec {
    CoreClassPart	core_class;
    CompositeClassPart	composite_class;
    ConstraintClassPart	constraint_class;
    FormClassPart	form_class;
    XowPanelTextClassPart	panelText_class;
} XowPanelTextClassRec;

extern XowPanelTextClassRec XowpanelTextClassRec;

typedef struct _XowPanelTextPart {
    /* resources */
    String	label;		/* description of the panelText	*/
    String	value;		/* for the user response	*/
    Pixmap	icon;		/* icon bitmap			*/
    Boolean	horizontalLayout; /* horizontal or vertical?    */
    int	        textBorderWidth;
    int		textLength;
    /* private data */
    Widget	iconW;		/* widget to display the icon	*/
    Widget	labelW;		/* widget to display description*/
    Widget	valueW;		/* user response TextWidget	*/
} XowPanelTextPart;

typedef struct _XowPanelTextRec {
    CorePart		core;
    CompositePart	composite;
    ConstraintPart	constraint;
    FormPart		form;
    XowPanelTextPart	panelText;
} XowPanelTextRec;

typedef struct {
      int empty;
} XowPanelTextConstraintsPart;

typedef struct _XowPanelTextConstraintsRec {
    FormConstraintsPart	  form;
    XowPanelTextConstraintsPart panelText;
} XowPanelTextConstraintsRec, *XowPanelTextConstraints;

#endif /* _PanelTextP_h */
