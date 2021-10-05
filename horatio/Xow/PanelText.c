/***********************************************************
Created by altering the dialog widget to allow horizontal
or vertical arrangement on labels and their values.

Mark Bush <bush@ox.prg>

The dialog widget implementation file contains the following
notice:

Copyright 1987, 1988 by Digital Equipment Corporation, Maynard, Massachusetts,
and the Massachusetts Institute of Technology, Cambridge, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Digital or MIT not be
used in advertising or publicity pertaining to distribution of the
software without specific, written prior permission.  

DIGITAL DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL
DIGITAL BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR
ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE.
******************************************************************/

/* NOTE: THIS IS NOT A WIDGET!  Rather, this is an interface to a widget.
   It implements policy, and gives a (hopefully) easier-to-use interface
   than just directly making your own form. */

static char SccsID[]="(c) Jon Tomb & Mark Bush @(#)PanelText.c	1.7	3/8/91";

#include <X11/Xlib.h>
#include <X11/Xos.h>
#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>
#include <X11/Xmu/Misc.h>

#include <X11/Xaw/XawInit.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/Command.h>	
#include <X11/Xow/PanelTextP.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Cardinals.h>

/*
 * After we have set the string in the value widget we set the
 * string to a magic value.  So that when a SetValues request is made
 * on the panelText value we will notice it, and reset the string.
 */

#define MAGIC_VALUE ((char *) 3)

#define streq(a,b) (strcmp( (a), (b) ) == 0)

#define offset(field) XtOffset(PanelTextWidget, panelText.field)

static XtResource resources[] = {
  {XtNlabel, XtCLabel, XtRString, sizeof(String),
     offset(label), XtRString, NULL},
  {XtNvalue, XtCValue, XtRString, sizeof(String),
     offset(value), XtRString, NULL},
  {XtNicon, XtCIcon, XtRPixmap, sizeof(Pixmap),
     offset(icon), XtRImmediate, 0},
  {XtNtextLength, XtCTextLength, XtRInt, sizeof(int),
     offset(textLength), XtRImmediate, (caddr_t) 100},     
  {XtNtextBorderWidth, XtCTextBorderWidth, XtRInt, sizeof(int),
     offset(textBorderWidth), XtRImmediate, (caddr_t) 1},
  {XtNhorizontalLayout, XtCHorizontalLayout, XtRBoolean, sizeof(Boolean),
     offset(horizontalLayout), XtRImmediate, (caddr_t) True},
};
#undef offset

static void Initialize(), ConstraintInitialize(), CreatePanelTextValueWidget();
static Boolean SetValues();

PanelTextClassRec panelTextClassRec = {
  { /* core_class fields */
    /* superclass         */    (WidgetClass) &formClassRec,
    /* class_name         */    "PanelText",
    /* widget_size        */    sizeof(PanelTextRec),
    /* class_initialize   */    XawInitializeWidgetSet,
    /* class_part init    */    NULL,
    /* class_inited       */    FALSE,
    /* initialize         */    Initialize,
    /* initialize_hook    */    NULL,
    /* realize            */    XtInheritRealize,
    /* actions            */    NULL,
    /* num_actions        */    0,
    /* resources          */    resources,
    /* num_resources      */    XtNumber(resources),
    /* xrm_class          */    NULLQUARK,
    /* compress_motion    */    TRUE,
    /* compress_exposure  */    TRUE,
    /* compress_enterleave*/    TRUE,
    /* visible_interest   */    FALSE,
    /* destroy            */    NULL,
    /* resize             */    XtInheritResize,
    /* expose             */    XtInheritExpose,
    /* set_values         */    SetValues,
    /* set_values_hook    */    NULL,
    /* set_values_almost  */    XtInheritSetValuesAlmost,
    /* get_values_hook    */    NULL,
    /* accept_focus       */    NULL,
    /* version            */    XtVersion,
    /* callback_private   */    NULL,
    /* tm_table           */    NULL,
    /* query_geometry     */	XtInheritQueryGeometry,
    /* display_accelerator*/	XtInheritDisplayAccelerator,
    /* extension          */	NULL
  },
  { /* composite_class fields */
    /* geometry_manager   */   XtInheritGeometryManager,
    /* change_managed     */   XtInheritChangeManaged,
    /* insert_child       */   XtInheritInsertChild,
    /* delete_child       */   XtInheritDeleteChild,
    /* extension          */   NULL
  },
  { /* constraint_class fields */
    /* subresourses       */   NULL,
    /* subresource_count  */   0,
    /* constraint_size    */   sizeof(PanelTextConstraintsRec),
    /* initialize         */   ConstraintInitialize,
    /* destroy            */   NULL,
    /* set_values         */   NULL,
    /* extension          */   NULL
  },
  { /* form_class fields */
    /* layout             */   XtInheritLayout
  },
  { /* panelText_class fields */
    /* empty              */   0
  }
};

WidgetClass panelTextWidgetClass = (WidgetClass)&panelTextClassRec;

/* ARGSUSED */
static void Initialize(request, new)
Widget request, new;
{
    PanelTextWidget ptw = (PanelTextWidget)new;
    Arg arglist[9];
    Cardinal num_args = 0;

    XtSetArg(arglist[num_args], XtNborderWidth, 0); num_args++;
    XtSetArg(arglist[num_args], XtNleft, XtChainLeft); num_args++;

    if (ptw->panelText.icon != (Pixmap)0) {
	XtSetArg(arglist[num_args], XtNbitmap, ptw->panelText.icon); num_args++;
	XtSetArg(arglist[num_args], XtNright, XtChainLeft); num_args++;
	ptw->panelText.iconW =
	    XtCreateManagedWidget( "icon", labelWidgetClass,
				   new, arglist, num_args );
	num_args = 1;
	XtSetArg(arglist[num_args], XtNfromHoriz, ptw->panelText.iconW);num_args++;
    } else ptw->panelText.iconW = (Widget)NULL;

    XtSetArg(arglist[num_args], XtNlabel, ptw->panelText.label); num_args++;
/*    XtSetArg(arglist[num_args], XtNright, XtChainRight); num_args++;*/

    ptw->panelText.labelW = XtCreateManagedWidget(ptw->core.name,
						  labelWidgetClass,
						  new, arglist, num_args);

    if (ptw->panelText.iconW != (Widget)NULL &&
	(ptw->panelText.labelW->core.height < ptw->panelText.iconW->core.height)) {
	XtSetArg( arglist[0], XtNheight, ptw->panelText.iconW->core.height );
	XtSetValues( ptw->panelText.labelW, arglist, ONE );
    }
    if (ptw->panelText.value == NULL)
	ptw->panelText.value = "";

    CreatePanelTextValueWidget( (Widget) ptw);
}

/* ARGSUSED */
static void ConstraintInitialize(request, new)
Widget request, new;
{
    PanelTextWidget ptw = (PanelTextWidget)new->core.parent;
    PanelTextConstraints constraint = (PanelTextConstraints)new->core.constraints;

    if (!XtIsSubclass(new, commandWidgetClass))	/* if not a button */
	return;					/* then just use defaults */

    constraint->form.left = constraint->form.right = XtChainLeft;
    if (ptw->panelText.valueW == NULL) 
      constraint->form.vert_base = ptw->panelText.labelW;
    else
      constraint->form.vert_base = ptw->panelText.valueW;

    if (ptw->composite.num_children > 1) {
	WidgetList children = ptw->composite.children;
	Widget *childP;
        for (childP = children + ptw->composite.num_children - 1;
	     childP >= children; childP-- ) {
	    if (*childP == ptw->panelText.labelW || *childP == ptw->panelText.valueW)
	        break;
	    if (XtIsManaged(*childP) &&
		XtIsSubclass(*childP, commandWidgetClass)) {
	        constraint->form.horiz_base = *childP;
		break;
	    }
	}
    }
}

#define ICON 0
#define LABEL 1
#define NUM_CHECKS 2

/* ARGSUSED */
static Boolean SetValues(current, request, new, in_args, in_num_args)
Widget current, request, new;
ArgList in_args;
Cardinal *in_num_args;
{
    PanelTextWidget w = (PanelTextWidget)new;
    PanelTextWidget old = (PanelTextWidget)current;
    Arg args[5];
    Cardinal num_args;
    int i;
    Boolean checks[NUM_CHECKS];

    for (i = 0; i < NUM_CHECKS; i++)
	checks[i] = FALSE;

    for (i = 0; i < *in_num_args; i++) {
	if (streq(XtNicon, in_args[i].name))
	    checks[ICON] = TRUE;
	if (streq(XtNlabel, in_args[i].name))
	    checks[LABEL] = TRUE;
    }

    if (checks[ICON]) {
	if (w->panelText.icon != (Pixmap)0) {
	    XtSetArg( args[0], XtNbitmap, w->panelText.icon );
	    if (old->panelText.iconW != (Widget)NULL) {
		XtSetValues( old->panelText.iconW, args, ONE );
	    } else {
		XtSetArg( args[1], XtNborderWidth, 0);
		XtSetArg( args[2], XtNleft, XtChainLeft);
		XtSetArg( args[3], XtNright, XtChainLeft);
		w->panelText.iconW =
		    XtCreateWidget( "icon", labelWidgetClass,
				    new, args, FOUR );
		((PanelTextConstraints)w->panelText.labelW->core.constraints)->
		    form.horiz_base = w->panelText.iconW;
		XtManageChild(w->panelText.iconW);
	    }
	} else {
	    ((PanelTextConstraints)w->panelText.labelW->core.constraints)->
		    form.horiz_base = (Widget)NULL;
	    XtDestroyWidget(old->panelText.iconW);
	    w->panelText.iconW = (Widget)NULL;
	}
    }

    if ( checks[LABEL] ) {
        num_args = 0;
        XtSetArg( args[num_args], XtNlabel, w->panelText.label ); num_args++;
	if (w->panelText.iconW != (Widget)NULL &&
	    (w->panelText.labelW->core.height <= w->panelText.iconW->core.height)) {
	    XtSetArg(args[num_args], XtNheight, w->panelText.iconW->core.height);
	    num_args++;
	}
	XtSetValues( w->panelText.labelW, args, num_args );
    }

    if ( w->panelText.value != old->panelText.value ) {
        if (w->panelText.value == NULL)  /* only get here if it
					  wasn't NULL before. */
	    XtDestroyWidget(old->panelText.valueW);
	else if (old->panelText.value == NULL) { /* create a new value widget. */
	    w->core.width = old->core.width;
	    w->core.height = old->core.height;
#ifdef notdef
/* this would be correct if Form had the same semantics on Resize
 * as on MakeGeometryRequest.  Unfortunately, Form botched it, so
 * any subclasses will currently have to deal with the fact that
 * we're about to change our real size.
 */
	    w->form.resize_in_layout = False; 
	    CreatePanelTextValueWidget( (Widget) w);
	    w->core.width = w->form.preferred_width;
	    w->core.height = w->form.preferred_height;
	    w->form.resize_in_layout = True;
#else /*notdef*/
	    CreatePanelTextValueWidget( (Widget) w);
#endif /*notdef*/
	}
	else {			/* Widget ok, just change string. */
	    Arg args[1];
	    XtSetArg(args[0], XtNstring, w->panelText.value);
	    XtSetValues(w->panelText.valueW, args, ONE);
	    XtSetArg(args[0], XtNinsertPosition, strlen(w->panelText.value));
	    XtSetValues(w->panelText.valueW, args, ONE);
	    w->panelText.value = MAGIC_VALUE;
	}
    }
    
    return False;
}

/*	Function Name: CreatePanelTextValueWidget
 *	Description: Creates the panelText widgets value widget.
 *	Arguments: w - the panelText widget.
 *	Returns: none.
 *
 *	must be called only when w->panelText.value is non-nil.
 */

static void
CreatePanelTextValueWidget(w)
Widget w;
{
    PanelTextWidget ptw = (PanelTextWidget) w;    
    Arg arglist[10];
    Cardinal num_args = 0;
    Boolean horizontal;

    XtSetArg(arglist[0], XtNhorizontalLayout, &horizontal);
    XtGetValues((Widget) ptw, arglist, ONE);
    
#ifdef notdef
    XtSetArg(arglist[num_args], XtNwidth,
	     ptw->panelText.labelW->core.width); num_args++; /* ||| hack */
#endif /*notdef*/
    XtSetArg(arglist[num_args], XtNstring, ptw->panelText.value);     num_args++;
    XtSetArg(arglist[num_args], XtNinsertPosition, strlen(ptw->panelText.value));
								num_args++;
/*    XtSetArg(arglist[num_args], XtNresizable, True);              num_args++;*/
/* XtSetArg(arglist[num_args], XtNresize, XawtextResizeWidth);    num_args++;*/
    XtSetArg(arglist[num_args], XtNwidth, ptw->panelText.textLength); num_args++;
    /* JON 1/3/91 */
    XtSetArg(arglist[num_args], XtNeditType, XawtextEdit);        num_args++;
    XtSetArg(arglist[num_args], XtNborderWidth, ptw->panelText.textBorderWidth);  num_args++;
    if (horizontal)
	{XtSetArg(arglist[num_args], XtNfromHoriz, ptw->panelText.labelW); num_args++;}
    else
	{XtSetArg(arglist[num_args], XtNfromVert, ptw->panelText.labelW);  num_args++;}
/*    XtSetArg(arglist[num_args], XtNleft, XtChainLeft);            num_args++;*/
/*    XtSetArg(arglist[num_args], XtNright, XtChainRight);          num_args++;*/
    
    ptw->panelText.valueW = XtCreateWidget("value",asciiTextWidgetClass,
				       w, arglist, num_args);

    /* if the value widget is being added after buttons,
     * then the buttons need new layout constraints.
     */
    if (ptw->composite.num_children > 1) {
	WidgetList children = ptw->composite.children;
	Widget *childP;
        for (childP = children + ptw->composite.num_children - 1;
	     childP >= children; childP-- ) {
	    if (*childP == ptw->panelText.labelW || *childP == ptw->panelText.valueW)
		continue;
	    if (XtIsManaged(*childP) &&
		XtIsSubclass(*childP, commandWidgetClass)) {
	        ((PanelTextConstraints)(*childP)->core.constraints)->
		    form.vert_base = ptw->panelText.valueW;
	    }
	}
    }
    XtManageChild(ptw->panelText.valueW);

/* 
 * Value widget gets the keyboard focus.
 */

    XtSetKeyboardFocus(w, ptw->panelText.valueW);
    ptw->panelText.value = MAGIC_VALUE;
}


void 
XowPanelTextAddButton(panelText, name, function, param)
Widget panelText;
String name;
XtCallbackProc function;
XtPointer param;
{
/*
 * Correct Constraints are all set in ConstraintInitialize().
 */
    Widget button;

    button = XtCreateManagedWidget( name, commandWidgetClass, panelText, 
				    NULL, (Cardinal) 0 );

    if (function != NULL)	/* don't add NULL callback func. */
	XtAddCallback(button, XtNcallback, function, param);
}


char *XowPanelTextGetValueString(w)
Widget w;
{
    Arg args[1];
    char * value;

    XtSetArg(args[0], XtNstring, &value);
    XtGetValues(((PanelTextWidget)w)->panelText.valueW, args, ONE);
    return(value);
}

void
XowPanelTextSetValueString(w, val)
Widget w;
String val;
{
    Arg args[1];

    XtSetArg(args[0], XtNvalue, val);
    XtSetValues(w, args, ONE);
}
