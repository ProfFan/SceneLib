/***********************************************************
Created by altering the dialog widget to allow a toggle
with a label displaying the result and a menu of the
choices.

Mark Bush <bush@ox.prg> & Jon Tombs <jon@ox.robots>

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

static char SccsID[]="(c) Jon Tomb & Mark Bush %W%\t%G%";
/* NOTE: THIS IS NOT A WIDGET!  Rather, this is an interface to a widget.
   It implements policy, and gives a (hopefully) easier-to-use interface
   than just directly making your own form. */


#include <X11/Xlib.h>
#include <X11/Xos.h>
#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>
#include <X11/Xmu/Misc.h>

#include <X11/Xaw/XawInit.h>
#include <X11/Xaw/Command.h>	
#include <X11/Xow/ToggleChoiceP.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/ToggleP.h>
#include <X11/Xaw/SimpleMenu.h>
#include <X11/Xaw/SmeBSB.h>

#ifndef streq
#define streq(a,b) (strcmp( (a), (b) ) == 0)
#endif

#define offset(field) XtOffset(ToggleChoiceWidget, toggleChoice.field)

static XtResource resources[] = {
  {XtNlabel, XtCLabel, XtRString, sizeof(String),
     offset(label), XtRString, NULL},
  {XtNicon, XtCIcon, XtRPixmap, sizeof(Pixmap),
     offset(icon), XtRImmediate, 0},
  {XtNchoices, XtCChoices, XtRPointer, sizeof(char **),
     offset(choices), XtRString, NULL},
  {XtNnumberChoices, XtCNumberChoices, XtRInt,  sizeof(int),
     offset(num_choices), XtRImmediate, (caddr_t)0},
  {XtNfont,  XtCFont, XtRFontStruct, sizeof(XFontStruct *),
     offset(font),XtRString, "XtDefaultFont"},
  {XtNindex, XtCIndex, XtRInt, sizeof(int),
     offset(index), XtRImmediate, (caddr_t)0},
  {XtNcallback, XtCCallback, XtRCallback, sizeof(XtCallbackList),
   offset(toggle_callback), XtRCallback, NULL},
};
#undef offset

static void Initialize(), ConstraintInitialize();
static void CreateToggleChoiceValueWidget(), CreateToggleChoiceToggleWidget();
static void CreateToggleChoiceMenuWidget();
static void ToggleChange(), ToggleSet();
static void ToggleEnterWindow(), ToggleLeaveWindow();
static void AddWindowEvents();
static Boolean SetValues();

#define DISPLAY XtDisplay((Widget)tcw)
#define WINDOW  DefaultRootWindow(DISPLAY)

static Boolean toggleBitmap1;
static String  defaultChoices = "unset";
/*static String *choices;*/
/*static int     num_choices;*/
static Dimension tog_width;

static XtActionsRec actionsList[] =
{
  {"nextChoice",	ToggleChange},
  {"enterWindow",	ToggleEnterWindow},
  {"leaveWindow",	ToggleLeaveWindow},
};

static char defaultTranslations[] =
    "<EnterWindow>:         enterWindow()  \n\
     <LeaveWindow>:         leaveWindow()  \n\
     <Btn1Down>,<Btn1Up>:   nextChoice(1)  \n\
     <Btn2Down>,<Btn2Up>:   nextChoice(-1) \n\
     <Btn3Down>:            XawPositionSimpleMenu(menu) MenuPopup(menu)";

ToggleChoiceClassRec toggleChoiceClassRec = {
  { /* core_class fields */
    /* superclass         */    (WidgetClass) &formClassRec,
    /* class_name         */    "ToggleChoice",
    /* widget_size        */    sizeof(ToggleChoiceRec),
    /* class_initialize   */    XawInitializeWidgetSet,
    /* class_part init    */    NULL,
    /* class_inited       */    FALSE,
    /* initialize         */    Initialize,
    /* initialize_hook    */    NULL,
    /* realize            */    XtInheritRealize,
    /* actions            */    actionsList,
    /* num_actions        */    XtNumber(actionsList),
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
    /* tm_table           */    defaultTranslations,
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
    /* constraint_size    */   sizeof(ToggleChoiceConstraintsRec),
    /* initialize         */   ConstraintInitialize,
    /* destroy            */   NULL,
    /* set_values         */   NULL,
    /* extension          */   NULL
  },
  { /* form_class fields */
    /* layout             */   XtInheritLayout
  },
  { /* toggleChoice_class fields */
    /* empty              */   0
  }
};

WidgetClass toggleChoiceWidgetClass = (WidgetClass)&toggleChoiceClassRec;

/*
 * Definition of pixmaps used for the toggle.
 */
#define toggle1_width 15
#define toggle1_height 15
static char toggle1_bits[] = {
   0x00, 0x00, 0x78, 0x02, 0x70, 0x07, 0x78, 0x0c, 0x4c, 0x18, 0x06, 0x30,
   0x06, 0x30, 0x06, 0x30, 0x06, 0x30, 0x06, 0x30, 0x0c, 0x19, 0x18, 0x0f,
   0x70, 0x07, 0x20, 0x0f, 0x00, 0x00};

#define toggle2_width 15
#define toggle2_height 15
static char toggle2_bits[] = {
   0x00, 0x00, 0xe0, 0x03, 0xf0, 0x07, 0x18, 0x2c, 0x0c, 0x38, 0x06, 0x38,
   0x04, 0x3c, 0x00, 0x00, 0x1e, 0x10, 0x0e, 0x30, 0x0e, 0x18, 0x1a, 0x0c,
   0xf0, 0x07, 0xe0, 0x03, 0x00, 0x00};

/* ARGSUSED */
static void Initialize(request, new)
Widget request, new;
{
    ToggleChoiceWidget tcw = (ToggleChoiceWidget)new;
    Arg arglist[10];
    Cardinal num_args = 0;

    XtSetArg(arglist[num_args], XtNborderWidth, 0); num_args++;
    XtSetArg(arglist[num_args], XtNleft, XtChainLeft); num_args++;

    if (tcw->toggleChoice.icon != (Pixmap)0) {
	XtSetArg(arglist[num_args], XtNbitmap, tcw->toggleChoice.icon); num_args++;
	XtSetArg(arglist[num_args], XtNright, XtChainLeft); num_args++;
	tcw->toggleChoice.iconW =
	    XtCreateManagedWidget( "icon", labelWidgetClass,
				   new, arglist, num_args );
	num_args = 1;
	XtSetArg(arglist[num_args], XtNfromHoriz, tcw->toggleChoice.iconW);num_args++;
    } else tcw->toggleChoice.iconW = (Widget)NULL;

    XtSetArg(arglist[num_args], XtNlabel, tcw->toggleChoice.label); num_args++;
/*    XtSetArg(arglist[num_args], XtNright, XtChainRight); num_args++;*/

    tcw->toggleChoice.labelW = XtCreateManagedWidget(tcw->core.name,
						     labelWidgetClass,
						     new, arglist, num_args);

    AddWindowEvents((Widget) tcw->toggleChoice.labelW);

    if (tcw->toggleChoice.iconW != (Widget)NULL &&
	(tcw->toggleChoice.labelW->core.height < tcw->toggleChoice.iconW->core.height)) {
	XtSetArg( arglist[0], XtNheight, tcw->toggleChoice.iconW->core.height );
	XtSetValues( tcw->toggleChoice.labelW, arglist, ONE );
    }

    tcw->toggleChoice.toggle1 = XCreateBitmapFromData(DISPLAY,
					WINDOW,
					(char *)toggle1_bits, 16, 16);

    tcw->toggleChoice.toggle2 = XCreateBitmapFromData(DISPLAY,
					WINDOW,
					(char *)toggle2_bits, 16, 16);

    if (tcw->toggleChoice.choices == NULL)
    {
	tcw->toggleChoice.choices = &defaultChoices;
	tcw->toggleChoice.num_choices = 1;
    }
	
    tcw->toggleChoice.inWindow = False;
    
    CreateToggleChoiceToggleWidget((Widget) tcw);

    CreateToggleChoiceValueWidget((Widget) tcw);

    CreateToggleChoiceMenuWidget((Widget) tcw);
}

/* ARGSUSED */
static void ConstraintInitialize(request, new)
Widget request, new;
{
    ToggleChoiceWidget tcw = (ToggleChoiceWidget)new->core.parent;
    ToggleChoiceConstraints constraint = (ToggleChoiceConstraints)new->core.constraints;

    if (!XtIsSubclass(new, commandWidgetClass))	/* if not a button */
	return;					/* then just use defaults */
	
    constraint->form.left = constraint->form.right = XtChainLeft;
    if (tcw->toggleChoice.valueW == NULL)
      constraint->form.vert_base = tcw->toggleChoice.labelW;
    else
      constraint->form.vert_base = tcw->toggleChoice.valueW;

    if (tcw->composite.num_children > 1) {
	WidgetList children = tcw->composite.children;
	Widget *childP;
        for (childP = children + tcw->composite.num_children - 1;
	     childP >= children; childP-- ) {
	    if (*childP == tcw->toggleChoice.labelW ||
		*childP == tcw->toggleChoice.valueW ||
		*childP == tcw->toggleChoice.toggleW ||
		*childP == tcw->toggleChoice.menuW)
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
#define FONT 2
#define INDEX 3
#define NUM_CHECKS 4

/* ARGSUSED */
static Boolean SetValues(current, request, new, in_args, in_num_args)
Widget current, request, new;
ArgList in_args;
Cardinal *in_num_args;
{
    ToggleChoiceWidget w = (ToggleChoiceWidget)new;
    ToggleChoiceWidget old = (ToggleChoiceWidget)current;
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
	if (streq(XtNfont, in_args[i].name))
	    checks[FONT] = TRUE;
	if (streq(XtNindex, in_args[i].name))
	    checks[INDEX] = TRUE;
    }

    if (checks[ICON]) {
	if (w->toggleChoice.icon != (Pixmap)0) {
	    XtSetArg( args[0], XtNbitmap, w->toggleChoice.icon );
	    if (old->toggleChoice.iconW != (Widget)NULL) {
		XtSetValues( old->toggleChoice.iconW, args, ONE );
	    } else {
		XtSetArg( args[1], XtNborderWidth, 0);
		XtSetArg( args[2], XtNleft, XtChainLeft);
		XtSetArg( args[3], XtNright, XtChainLeft);
		w->toggleChoice.iconW =
		    XtCreateWidget( "icon", labelWidgetClass,
				    new, args, FOUR );
		((ToggleChoiceConstraints)w->toggleChoice.labelW->core.constraints)->
		    form.horiz_base = w->toggleChoice.iconW;
		XtManageChild(w->toggleChoice.iconW);
	    }
	} else {
	    ((ToggleChoiceConstraints)w->toggleChoice.labelW->core.constraints)->
		    form.horiz_base = (Widget)NULL;
	    XtDestroyWidget(old->toggleChoice.iconW);
	    w->toggleChoice.iconW = (Widget)NULL;
	}
    }

    if ( checks[LABEL] ) {
        num_args = 0;
        XtSetArg( args[num_args], XtNlabel, w->toggleChoice.label ); num_args++;
	if (w->toggleChoice.iconW != (Widget)NULL &&
	    (w->toggleChoice.labelW->core.height <= w->toggleChoice.iconW->core.height)) {
	    XtSetArg(args[num_args], XtNheight, w->toggleChoice.iconW->core.height);
	    num_args++;
	}
	XtSetValues( w->toggleChoice.labelW, args, num_args );
    }

    if (checks[FONT])
    {
	num_args = 0;
	XtSetArg(args[num_args], XtNfont, w->toggleChoice.font); num_args++;
	XtSetValues(w->toggleChoice.labelW, args, num_args);
	XtSetValues(w->toggleChoice.valueW, args, num_args);
    }

    if (checks[INDEX])
    {
	Boolean tempInWindow;
	Cardinal zero = 0;
	
	w->toggleChoice.index = w->toggleChoice.index % w->toggleChoice.num_choices;
	
	while (w->toggleChoice.index < 0)
	    w->toggleChoice.index += w->toggleChoice.num_choices;

	num_args = 0;
	XtSetArg(args[num_args], XtNlabel, w->toggleChoice.choices[w->toggleChoice.index]);
	XtSetValues(w->toggleChoice.valueW, args, num_args);

	w->toggleChoice.index -= 1;

	tempInWindow = w->toggleChoice.inWindow;
	w->toggleChoice.inWindow = True;

	ToggleChange((Widget) w, (XEvent *) NULL,
		     (String *) NULL, (Cardinal *) &zero);

	w->toggleChoice.inWindow = tempInWindow;
					     
    }
    
    return False;
}

/*	Function Name: CreateToggleChoiceValueWidget
 *	Description: Creates the toggleChoice widgets value widget.
 *	Arguments: w - the toggleChoice widget.
 *	Returns: none.
 */

static void
CreateToggleChoiceValueWidget(w)
Widget w;
{
    ToggleChoiceWidget tcw = (ToggleChoiceWidget) w;    
    Arg arglist[10];
    Cardinal num_args = 0;
    int current = 0, length = 0, i, this_len;
    String *new_choices;

    new_choices = (String *) calloc(256, 1);

#ifdef notdef
    XtSetArg(arglist[num_args], XtNwidth,
	     tcw->toggleChoice.labelW->core.width); num_args++; /* ||| hack */
#endif /*notdef*/
    tcw->toggleChoice.index = 0;
    tcw->toggleChoice.num_choices = tcw->toggleChoice.num_choices;

    for (i=0; i<tcw->toggleChoice.num_choices; i++)
    {
	this_len = XTextWidth(tcw->toggleChoice.font,
			      tcw->toggleChoice.choices[i],
			      strlen(tcw->toggleChoice.choices[i]));
	length = ((length<this_len)? this_len: length);
    }

    tog_width = length + XTextWidth (tcw->toggleChoice.font, "  ", 2);

    for (i=0; i<tcw->toggleChoice.num_choices; i++)
    {
	new_choices[i] = (String) malloc(1 + (sizeof(char) *
					 strlen(tcw->toggleChoice.choices[i])));
	strcpy(new_choices[i], tcw->toggleChoice.choices[i]);
    }
    tcw->toggleChoice.choices = &new_choices[0];
    
    XtSetArg(arglist[num_args], XtNlabel, tcw->toggleChoice.choices[current]);
							 num_args++;
    XtSetArg(arglist[num_args], XtNresizable, False);    num_args++;
    XtSetArg(arglist[num_args], XtNfromHoriz, tcw->toggleChoice.toggleW);
							 num_args++;
/*    XtSetArg(arglist[num_args], XtNleft, XtChainLeft);   num_args++;*/
/*    XtSetArg(arglist[num_args], XtNright, XtChainRight); num_args++;*/
    XtSetArg(arglist[num_args], XtNwidth, tog_width);    num_args++;

    tcw->toggleChoice.valueW = XtCreateWidget("value",labelWidgetClass,
				       w, arglist, num_args);

    AddWindowEvents((Widget) tcw->toggleChoice.valueW);

    XtManageChild(tcw->toggleChoice.valueW);
}

static void
CreateToggleChoiceToggleWidget(w)
Widget w;
{
    ToggleChoiceWidget tcw = (ToggleChoiceWidget) w;    
    Arg arglist[10];
    Cardinal num_args = 0;
    XtTranslations tr;

    XtSetArg(arglist[num_args], XtNfromHoriz, tcw->toggleChoice.labelW); num_args++;
    XtSetArg(arglist[num_args], XtNbitmap, tcw->toggleChoice.toggle1); num_args++;
/*    XtSetArg(arglist[num_args], XtNleft, XtChainLeft);            num_args++;*/
/*    XtSetArg(arglist[num_args], XtNright, XtChainRight);          num_args++;*/
    XtSetArg(arglist[num_args], XtNborderWidth, 0);               num_args++;
    
    tcw->toggleChoice.toggleW = XtCreateManagedWidget("toggle",
						     labelWidgetClass,
						     w, arglist, num_args);

    AddWindowEvents((Widget) tcw->toggleChoice.toggleW);

    XtManageChild(tcw->toggleChoice.toggleW);

    toggleBitmap1 = True;
}

static void
CreateToggleChoiceMenuWidget(w)
Widget w;
{
    ToggleChoiceWidget tcw = (ToggleChoiceWidget) w;
    Arg arglist[10];
    Cardinal num_args = 0;
    Widget sw;
    int i;

    tcw->toggleChoice.menuW = XtCreatePopupShell("menu", simpleMenuWidgetClass,
						w, arglist, num_args);

    
    for (i=0; i<tcw->toggleChoice.num_choices; i++)
    {
	XtSetArg(arglist[num_args], XtNlabel, tcw->toggleChoice.choices[i]);

	sw = XtCreateManagedWidget(tcw->toggleChoice.choices[i], smeBSBObjectClass,
				   (Widget) tcw->toggleChoice.menuW,
				   arglist, ONE);
	XtAddCallback(sw, XtNcallback, ToggleSet, (XtPointer) i);
    }
}

static void
ToggleChange(w,event,params,num_params)
Widget w;
XEvent *event;          /* unused */
String *params;         /* unused */
Cardinal *num_params;   /* unused */
{
    ToggleChoiceWidget tcw;
    Arg arglist[10];
    int offset;

    if (XtIsSubclass(w, labelWidgetClass))
	tcw = (ToggleChoiceWidget) XtParent(w);
    else
	tcw = (ToggleChoiceWidget) w;

    if (! tcw->toggleChoice.inWindow)
	return;

    if (*num_params > 0)
	offset = atoi(*params);
    else
	offset = 1;
	
    if (toggleBitmap1)
	XtSetArg(arglist[0], XtNbitmap, tcw->toggleChoice.toggle2);
    else
	XtSetArg(arglist[0], XtNbitmap, tcw->toggleChoice.toggle1);

    tcw->toggleChoice.index = (tcw->toggleChoice.index + offset) % tcw->toggleChoice.num_choices;
    while (tcw->toggleChoice.index < 0)
	tcw->toggleChoice.index += tcw->toggleChoice.num_choices;
	
    toggleBitmap1 = !toggleBitmap1;
    
    XtSetArg(arglist[1], XtNstate, False);
    XtSetValues(tcw->toggleChoice.toggleW, arglist, TWO);

    XtSetArg(arglist[0], XtNlabel, tcw->toggleChoice.choices[tcw->toggleChoice.index]);
    XtSetValues(tcw->toggleChoice.valueW, arglist, ONE);
    XtCallCallbacks((Widget)tcw, XtNcallback, (XtPointer) tcw->toggleChoice.index);    
}

static void
ToggleSet(w, c, d)
Widget w;
XtPointer c, d;
{
    ToggleChoiceWidget tcw;
    Boolean tempInWindow;
    Cardinal zero = 0;

    tcw = (ToggleChoiceWidget) XtParent(XtParent(w));
    
    tempInWindow = tcw->toggleChoice.inWindow;
    tcw->toggleChoice.inWindow = True;
    
    tcw->toggleChoice.index = (int) c - 1;
    ToggleChange((Widget) tcw, (XEvent *) NULL,
		 (String *) NULL, (Cardinal *) &zero);

    tcw->toggleChoice.inWindow = tempInWindow;

}

static void
ToggleEnterWindow(w,event,params,num_params)
Widget w;
XEvent *event;          /* unused */
String *params;         /* unused */
Cardinal *num_params;   /* unused */
{
    ToggleChoiceWidget tcw;

    if (XtIsSubclass(w, labelWidgetClass))
	tcw = (ToggleChoiceWidget) XtParent(w);
    else
	tcw = (ToggleChoiceWidget) w;

    tcw->toggleChoice.inWindow = (Boolean) True;
}

static void
ToggleLeaveWindow(w,event,params,num_params)
Widget w;
XEvent *event;          /* unused */
String *params;         /* unused */
Cardinal *num_params;   /* unused */
{
    ToggleChoiceWidget tcw;

    if (XtIsSubclass(w, labelWidgetClass))
	tcw = (ToggleChoiceWidget) XtParent(w);
    else
	tcw = (ToggleChoiceWidget) w;

    tcw->toggleChoice.inWindow = False;
}

static void
AddWindowEvents(w)
Widget w;
{
    XtTranslations tr;
    
    tr = XtParseTranslationTable(
		"<EnterWindow>:         enterWindow() \n\
		 <LeaveWindow>:         leaveWindow() \n\
		 <Btn1Down>,<Btn1Up>:   nextChoice(1) \n\
		 <Btn2Down>,<Btn2Up>:   nextChoice(-1)");
    XtOverrideTranslations(w, tr);
}

void 
XowToggleChoiceAddButton(toggleChoice, name, function, param)
Widget toggleChoice;
String name;
XtCallbackProc function;
XtPointer param;
{
/*
 * Correct Constraints are all set in ConstraintInitialize().
 */
    Widget button;

    button = XtCreateManagedWidget( name, commandWidgetClass, toggleChoice, 
				    NULL, (Cardinal) 0 );

    if (function != NULL)	/* don't add NULL callback func. */
        XtAddCallback(button, XtNcallback, function, param);
}


char *XowToggleChoiceGetValueString(w)
Widget w;
{
    Arg args[1];
    char * value;

    XtSetArg(args[0], XtNlabel, &value);
    XtGetValues( ((ToggleChoiceWidget)w)->toggleChoice.valueW, args, ONE);
    return(value);
}
