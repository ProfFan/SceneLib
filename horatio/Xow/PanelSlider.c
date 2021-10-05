/***********************************************************
 The start of a slider widget X, based on the scrollbar widget
 from Xaw

 Jon Tombs
 ***********************************************************/
/* This is based on the Xaw Dialog Widget */
/*
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

static char SccsID[]="(c) Jon Tomb & Mark Bush @(#)PanelSlider.c	1.8	7/26/91";

#include <X11/Xlib.h>
#include <X11/Xos.h>
#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>
#include <X11/Xmu/Misc.h>

#include <X11/Xaw/XawInit.h>
#include <X11/Xaw/Command.h>	
#include <X11/Xow/PanelSliderP.h>
#include <X11/Xow/Slider.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Cardinals.h>

#define streq(a,b) (strcmp( (a), (b) ) == 0)

#define offset(field) XtOffset(PanelSliderWidget, panelSlider.field)

static XtResource resources[] = {
  {XtNlabel, XtCLabel, XtRString, sizeof(String),
     offset(label), XtRString, NULL},
  {XtNvalue, XtCValue, XtRInt, sizeof(int),
     offset(value), XtRInt, (caddr_t) 0},
  {XtNsliderProc, XtCCallback, XtRCallback, sizeof(caddr_t),
     offset(sliderProc), XtRCallback, NULL},
  {XtNmax, XtCMax, XtRInt, sizeof(int),
     offset(max), XtRImmediate, (caddr_t) 100},
  {XtNmin, XtCMin, XtRInt, sizeof(int),
     offset(min), XtRImmediate, (caddr_t) 0},
  {XtNstepSize, XtCStepSize, XtRInt, sizeof(int),
     offset(stepSize), XtRImmediate,(caddr_t) 1},
  {XtNsliderLength, XtCSliderLength, XtRInt, sizeof(int),
     offset(sliderLength), XtRImmediate,(caddr_t) 200},
};
#undef offset

static void Initialize(), ConstraintInitialize(), CreatePanelSliderValueWidget();
static Boolean SetValues();

PanelSliderClassRec panelSliderClassRec = {
  { /* core_class fields */
    /* superclass         */    (WidgetClass) &formClassRec,
    /* class_name         */    "PanelSlider",
    /* widget_size        */    sizeof(PanelSliderRec),
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
    /* constraint_size    */   sizeof(PanelSliderConstraintsRec),
    /* initialize         */   ConstraintInitialize,
    /* destroy            */   NULL,
    /* set_values         */   NULL,
    /* extension          */   NULL
  },
  { /* form_class fields */
    /* layout             */   XtInheritLayout
  },
  { /* panelSlider_class fields */
    /* empty              */   0
  }
};
static void slideMoved();
static int range;
WidgetClass panelSliderWidgetClass = (WidgetClass)&panelSliderClassRec;

/* ARGSUSED */
static void Initialize(request, new)
Widget request, new;
{
    PanelSliderWidget psw = (PanelSliderWidget)new;
    Arg arglist[9];
    Cardinal num_args = 0;
    char buf[15];
    int range, step; /* step is the step of the Slider (ie in pixels) */

    range = psw->panelSlider.max -  psw->panelSlider.min;
 

    XtSetArg(arglist[num_args], XtNborderWidth, 0); num_args++;
 
    psw->panelSlider.labelW = XtCreateManagedWidget(psw->core.name,
						    labelWidgetClass,
                                                    new, arglist, num_args);

    XtSetArg(arglist[num_args], XtNfromHoriz, psw->panelSlider.labelW); num_args++;
    psw->panelSlider.valueW = XtCreateManagedWidget(valStr[0], labelWidgetClass,
                                   new, arglist, num_args );
    num_args--;
    range = psw->panelSlider.max -  psw->panelSlider.min ;
    showValue(psw,/*for AZ psw->panelSlider.min + */ psw->panelSlider.value);
    XtSetArg(arglist[num_args], XtNfromHoriz, psw->panelSlider.valueW); num_args++;
    sprintf(buf,"%d", psw->panelSlider.min);
	sprintf(buf,"%d",psw->panelSlider.min);
    psw->panelSlider.minW = XtCreateManagedWidget(buf, labelWidgetClass,
				   new, arglist, num_args );

    num_args--;
    XtSetArg(arglist[num_args], XtNfromHoriz, psw->panelSlider.minW);num_args++;
    if ( psw->panelSlider.stepSize == 0)
      psw->panelSlider.stepSize = 1;

    if (psw->panelSlider.stepSize * range > psw->panelSlider.sliderLength)
    {
      psw->panelSlider.stepSize = (int) ( 0.5 + ((float) range) /
				   (float) psw->panelSlider.sliderLength);
      if ( psw->panelSlider.stepSize == 0)
	 psw->panelSlider.stepSize = 1;
      psw->panelSlider.sliderLength = range / psw->panelSlider.stepSize;
      step = 1;
    }
    else
    {
	 step = psw->panelSlider.sliderLength / range;
	 psw->panelSlider.sliderLength = step * range;
    }

					
    XtSetArg(arglist[num_args], XtNlength, psw->panelSlider.sliderLength);
				    num_args++;
    XtSetArg(arglist[num_args], XtNvalue,
	     (psw->panelSlider.value - psw->panelSlider.min ) /*AZ*/ /
		  psw->panelSlider.stepSize);
    num_args++;

    XtSetArg(arglist[num_args], XtNstep, step);
				 num_args++;

    if (psw->panelSlider.sliderLength <  range)
      XtSetArg(arglist[num_args], XtNsteps, range / psw->panelSlider.stepSize);
    else
      XtSetArg(arglist[num_args], XtNsteps, range);
				 num_args++;
    XtSetArg(arglist[num_args], XtNborderWidth, 1); num_args++;
    psw->panelSlider.sliderW = XtCreateManagedWidget("slider",
						    sliderWidgetClass,
                                                    new, &arglist[1], num_args - 1);
    XtAddCallback(psw->panelSlider.sliderW, XtNslideProc, slideMoved, NULL);
    num_args-= 4;

    XtSetArg(arglist[num_args], XtNfromHoriz, psw->panelSlider.sliderW);num_args++;
    sprintf(buf,"%d", psw->panelSlider.max);

    psw->panelSlider.maxW = XtCreateManagedWidget(buf, labelWidgetClass,
                                   new, arglist, num_args );

   

}

/* ARGSUSED */
static void ConstraintInitialize(request, new)
Widget request, new;
{
    PanelSliderWidget psw = (PanelSliderWidget)new->core.parent;
    PanelSliderConstraints constraint = (PanelSliderConstraints)new->core.constraints;

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
    PanelSliderWidget w = (PanelSliderWidget)new;
    PanelSliderWidget old = (PanelSliderWidget)current;
    Arg args[5];
    Cardinal num_args;
    int i;
    Boolean checks[NUM_CHECKS];
    char buf[125];

    for (i = 0; i < NUM_CHECKS; i++)
	checks[i] = FALSE;

    for (i = 0; i < *in_num_args; i++) {
	if (streq(XtNmin, in_args[i].name))
	    checks[ICON] = TRUE;
	if (streq(XtNlabel, in_args[i].name))
	    checks[LABEL] = TRUE;
    }

    if (checks[ICON]) {
	if (w->panelSlider.minW != NULL) {
	    if (old->panelSlider.minW != (Widget)NULL) {
		XtSetValues( old->panelSlider.minW, args, ONE );
	    } else {
		XtSetArg( args[1], XtNborderWidth, 0);
		XtSetArg( args[2], XtNleft, XtChainLeft);
		XtSetArg( args[3], XtNright, XtChainLeft);
		w->panelSlider.minW =
		    XtCreateWidget( "min", labelWidgetClass,
				    new, args, FOUR );
		((PanelSliderConstraints)w->panelSlider.labelW->core.constraints)->
		    form.horiz_base = w->panelSlider.minW;
		XtManageChild(w->panelSlider.minW);
	    }
	} else {
	    ((PanelSliderConstraints)w->panelSlider.labelW->core.constraints)->
		    form.horiz_base = (Widget)NULL;
	    XtDestroyWidget(old->panelSlider.minW);
	    w->panelSlider.minW = (Widget)NULL;
	}
    }

    if ( checks[LABEL] ) {
        num_args = 0;
        XtSetArg( args[num_args], XtNlabel, w->panelSlider.label ); num_args++;
	if (w->panelSlider.minW != (Widget)NULL &&
	    (w->panelSlider.labelW->core.height <= w->panelSlider.minW->core.height)) {
	    XtSetArg(args[num_args], XtNheight, w->panelSlider.minW->core.height);
	    num_args++;
	}
	XtSetValues( w->panelSlider.labelW, args, num_args );
    }


    if ( w->panelSlider.value != old->panelSlider.value ) {
	    Arg args[1];
	    sprintf(buf, valStr[0], w->panelSlider.value);
	    XtSetArg(args[0], XtNlabel, buf);
	    XtSetValues(w->panelSlider.valueW, args, ONE);
   	    XtSetArg(args[0], XtNvalue, (w->panelSlider.value - w->panelSlider.min) / w->panelSlider.stepSize);
	    XtSetValues(w->panelSlider.sliderW, args, ONE);
    }
    
    return False;
}

/*	Function Name: CreatePanelSliderValueWidget
 *	Description: Creates the panelSlider widgets value widget.
 *	Arguments: w - the panelSlider widget.
 *	Returns: none.
 *
 *	must be called only when w->panelSlider.value is non-nil.
 */

static void
CreatePanelSliderValueWidget(w)
Widget w;
{
    PanelSliderWidget psw = (PanelSliderWidget) w;    
    Arg arglist[10];
    Cardinal num_args = 0;
    Boolean horizontal;

    XtSetArg(arglist[0], XtNhorizontalLayout, &horizontal);
    XtGetValues((Widget) psw, arglist, ONE);
    

    XtSetArg(arglist[num_args], XtNstring, psw->panelSlider.value);     num_args++;
    XtSetArg(arglist[num_args], XtNresizable, True);              num_args++;
    if (horizontal)
	{XtSetArg(arglist[num_args], XtNfromHoriz, psw->panelSlider.labelW); num_args++;}
    else
	{XtSetArg(arglist[num_args], XtNfromVert, psw->panelSlider.labelW);  num_args++;}
    XtSetArg(arglist[num_args], XtNleft, XtChainLeft);            num_args++;
    XtSetArg(arglist[num_args], XtNright, XtChainRight);          num_args++;
    
    psw->panelSlider.valueW = XtCreateWidget("value",labelWidgetClass,
				       w, arglist, num_args);


    XtManageChild(psw->panelSlider.valueW);

/* 
 * Value widget gets the keyboard focus.
 */

    XtSetKeyboardFocus(w, psw->panelSlider.valueW);

}



XowPanelSliderGetValue(w)
Widget w;
{
    return ((PanelSliderWidget)w)->panelSlider.value;
    
}

static void
slideMoved(w, closure, call_data)
Widget w;
XtPointer closure, call_data;
{
    int a = (int) call_data;
    PanelSliderWidget ptw = (PanelSliderWidget) XtParent(w);   

    a *= ptw->panelSlider.stepSize;
    a += ptw->panelSlider.min;
    showValue(ptw, a); 
    XtCallCallbacks((Widget) ptw, XtNsliderProc,(XtPointer) a);
}

showValue(ptw, val)
PanelSliderWidget ptw;
int val;
{
   Arg arglist[9];
   char buf[15];
   float num;

   sprintf(buf, valStr[0], val);

   XtSetArg(arglist[0], XtNlabel, buf);
   XtSetValues(ptw->panelSlider.valueW, arglist, ONE);
}
   
   
