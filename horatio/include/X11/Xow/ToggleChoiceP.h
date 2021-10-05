/***********************************************************
Created by altering the dialog widget to allow a toggle
with a label displaying the result and a menu of the
choices.

Mark Bush <bush@ox.prg>
******************************************************************/
/*SccsID (c) Jon Tomb & Mark Bush @(#)ToggleChoiceP.h	1.6  10/16/91*/

/* Private definitions for ToggleChoice widget */

#ifndef _ToggleChoiceP_h
#define _ToggleChoiceP_h

#include <X11/Xow/ToggleChoice.h>
#include <X11/Xaw/FormP.h>

typedef struct {
      int empty;
} XowToggleChoiceClassPart;

typedef struct _XowToggleChoiceClassRec {
    CoreClassPart	core_class;
    CompositeClassPart	composite_class;
    ConstraintClassPart	constraint_class;
    FormClassPart	form_class;
    XowToggleChoiceClassPart	toggleChoice_class;
} XowToggleChoiceClassRec;

extern XowToggleChoiceClassRec XowtoggleChoiceClassRec;

typedef struct _XowToggleChoicePart {
    /* resources */
    String	label;		/* description of toggleChoice	*/
    Pixmap	icon;		/* icon bitmap			*/
    String	*choices;	/* Choice entries               */
    int		num_choices;	/* Number of choice entries     */
    XFontStruct *font;		/* Font to display choices      */
    /* private data */
    Widget	iconW;		/* widget to display the icon	*/
    Widget	labelW;		/* widget to display description*/
    Widget	toggleW;	/* toggle button		*/
    Widget	valueW;		/* user response TextWidget	*/
    Widget	menuW;		/* menu of choices              */
    Pixmap	toggle1;	/* two pixmaps to display	*/
    Pixmap	toggle2;	/*		toggle image	*/
    int		index;		/* Current choice               */
    Boolean	inWindow;	/* Is pointer in window?        */
    XtCallbackList	toggle_callback;        
} XowToggleChoicePart;

typedef struct _XowToggleChoiceRec {
    CorePart		core;
    CompositePart	composite;
    ConstraintPart	constraint;
    FormPart		form;
    XowToggleChoicePart	toggleChoice;
} XowToggleChoiceRec;

typedef struct {
      int empty;
} XowToggleChoiceConstraintsPart;

typedef struct _XowToggleChoiceConstraintsRec {
    FormConstraintsPart	  form;
    XowToggleChoiceConstraintsPart toggleChoice;
} XowToggleChoiceConstraintsRec, *XowToggleChoiceConstraints;

#endif /* _ToggleChoiceP_h */
