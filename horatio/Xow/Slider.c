
/***********************************************************

******************************************************************/

static char SccsID[]="(c) Jon Tomb & Mark Bush @(#)Slider.c	1.7\t4/2/91";
/* This is based on the ScrollBar widget in the mit Xaw library */
/***********************************************************
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

/* ScrollBar.c */
/* created by weissman, Mon Jul  7 13:20:03 1986 */
/* converted by swick, Thu Aug 27 1987 */


#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>

#include <X11/Xaw/XawInit.h>
#include <X11/Xow/SliderP.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Cardinals.h>
#include <stdio.h>

/* Private definitions. */

static char defaultTranslations[] =
    "<Btn1Down>:   StartSlide(Forward) MovesSlide()  \n\
     <Btn2Down>:   StartSlide(Continuous) MoveSlide() NotifySlide() \n\
     <Btn3Down>:   StartSlide(Backward) MovesSlide()\n\
     <Btn2Motion>: MoveSlide() NotifySlide() \n\
     <BtnUp>:      NotifySlide(Proportional) EndSlide()";

#ifdef bogusSlideKeys
    /* examples */
    "<KeyPress>f:  StartSlide(Forward) NotifySlide(FullLength) EndSlide()"
    "<KeyPress>b:  StartSlide(Backward) NotifySlide(FullLength) EndSlide()"
#endif


#define Offset(field) XtOffset(SliderWidget, field)

static XtResource resources[] = {
  {XtNlength, XtCLength, XtRDimension, sizeof(Dimension),
       Offset(slider.length), XtRImmediate, (caddr_t) 100},
  {XtNsteps, XtCsteps, XtRInt, sizeof(int),
       Offset(slider.steps), XtRImmediate, (caddr_t) 100},
  {XtNthickness, XtCThickness, XtRDimension, sizeof(Dimension),
       Offset(slider.thickness), XtRImmediate, (caddr_t) 14},
  {XtNstep, XtCstep, XtRInt, sizeof(int),
       Offset(slider.step), XtRImmediate, (caddr_t) 1},       
  {XtNorientation, XtCOrientation, XtROrientation, sizeof(XtOrientation),
      Offset(slider.orientation), XtRImmediate, (caddr_t) XtorientHorizontal},
  {XtNslideProc, XtCCallback, XtRCallback, sizeof(caddr_t),
       Offset(slider.slideProc), XtRCallback, NULL},
  {XtNjumpProc, XtCCallback, XtRCallback, sizeof(caddr_t),
       Offset(slider.jumpProc), XtRCallback, NULL},
  {XtNpixmap, XtCPixmap, XtRPixmap, sizeof(Pixmap),
       Offset(slider.pixmap), XtRImmediate, (XtPointer) XtUnspecifiedPixmap},
  {XtNforeground, XtCForeground, XtRPixel, sizeof(Pixel),
       Offset(slider.foreground), XtRString, XtDefaultForeground},
  {XtNvalue, XtCValue, XtRInt, sizeof(int),
       Offset(slider.value), XtRImmediate, (caddr_t) 1},
  {XtNscrollVCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.verCursor), XtRString, "sb_v_double_arrow"},
  {XtNscrollHCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.horCursor), XtRString, "sb_h_double_arrow"},
  {XtNscrollUCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.upCursor), XtRString, "sb_up_arrow"},
  {XtNscrollDCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.downCursor), XtRString, "sb_down_arrow"},
  {XtNscrollLCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.leftCursor), XtRString, "sb_left_arrow"},
  {XtNscrollRCursor, XtCCursor, XtRCursor, sizeof(Cursor),
       Offset(slider.rightCursor), XtRString, "sb_right_arrow"},
};

static void ClassInitialize();
static void Initialize();
static void Destroy();
static void Realize();
static void Resize();
static void Redisplay();
static Boolean SetValues();

static void StartSlide();
static void MovesSlide();
static void MoveSlide();
static void NotifySlide();
static void EndSlide();

static XtActionsRec actions[] = {
	{"StartSlide",		StartSlide},
	{"MovesSlide",		MovesSlide},
	{"MoveSlide",		MoveSlide},	
	{"NotifySlide",	        NotifySlide},
	{"EndSlide",		EndSlide},
	{NULL,NULL}
};


SliderClassRec sliderClassRec = {
/* core fields */
    /* superclass       */      (WidgetClass) &widgetClassRec,
    /* class_name       */      "Slider",
    /* size             */      sizeof(SliderRec),
    /* class_initialize	*/	ClassInitialize,
    /* class_part_init  */	NULL,
    /* class_inited	*/	FALSE,
    /* initialize       */      Initialize,
    /* initialize_hook  */	NULL,
    /* realize          */      Realize,
    /* actions          */      actions,
    /* num_actions	*/	XtNumber(actions),
    /* resources        */      resources,
    /* num_resources    */      XtNumber(resources),
    /* xrm_class        */      NULLQUARK,
    /* compress_motion	*/	TRUE,
    /* compress_exposure*/	TRUE,
    /* compress_enterleave*/	TRUE,
    /* visible_interest */      FALSE,
    /* destroy          */      Destroy,
    /* resize           */      Resize,
    /* expose           */      Redisplay,
    /* set_values       */      SetValues,
    /* set_values_hook  */	NULL,
    /* set_values_almost */	XtInheritSetValuesAlmost,
    /* get_values_hook  */	NULL,
    /* accept_focus     */      NULL,
    /* version          */	XtVersion,
    /* callback_private */      NULL,
    /* tm_table         */      defaultTranslations,
    /* query_geometry	*/	XtInheritQueryGeometry,
    /* display_accelerator*/	XtInheritDisplayAccelerator,
    /* extension        */	NULL
};

WidgetClass sliderWidgetClass = (WidgetClass)&sliderClassRec;

#define NoButton -1
#define PICKLENGTH(widget, x, y) \
    ((widget->slider.orientation == XtorientHorizontal) ? x : y)
#define MIN(x,y)	((x) < (y) ? (x) : (y))
#define MAX(x,y)	((x) > (y) ? (x) : (y))

static void ClassInitialize()
{
    static XtConvertArgRec screenConvertArg[] = {
        {XtWidgetBaseOffset, (caddr_t) XtOffset(Widget, core.screen),
	     sizeof(Screen *)}
    };

    XawInitializeWidgetSet();
    XtAddConverter( XtRString, XtROrientation, XmuCvtStringToOrientation,
		    NULL, (Cardinal)0 );
    XtAddConverter( XtRString, XtRPixmap, XmuCvtStringToBitmap,
		   screenConvertArg, XtNumber(screenConvertArg));
}



static FillArea(w, top, end, thumb)
  SliderWidget w;
  Position top, end;
  int thumb;
{

    switch(thumb) {
	/* Fill the new Slide location */
     case 1:
	 XFillRectangle(XtDisplay(w), XtWindow(w),
			   w->slider.gc, top, 1, end - top,
			   w->core.height-2);	
	break;
	 /* Clear the old Slide location */
      case 0:
	 XClearArea(XtDisplay(w), XtWindow(w), top, 1,
		       end - top, w->core.height-2, FALSE);
	
    }
}


/* Paint the thumb in the area specified by w->top and
   w->shown.  The old area is erased.  The painting and
   erasing is done cleverly so that no flickering will occur. */

static void PaintSlide( w )
  SliderWidget w;
{
    Position oldtop, newtop;

    oldtop = w->slider.topLoc;
    newtop = w->slider.value;
    
    w->slider.topLoc = newtop;

    if (XtIsRealized((Widget)w)) {
	if (newtop < oldtop) FillArea(w, newtop, oldtop, 0);
	if (newtop > oldtop) FillArea(w, oldtop, newtop, 1);
    }
    XFlush(XtDisplay(w));
}


static void SetDimensions(w)
    SliderWidget w;
{
    if (w->slider.orientation == XtorientVertical) {
	w->slider.length = w->core.height;
	w->slider.thickness = w->core.width;
    }
    else {
	w->slider.length = w->core.width;
	w->slider.thickness = w->core.height;
    }
}

/*	Function Name: Destroy
 *	Description: Called as the slider is going away...
 *	Arguments: w - the slider.
 *	Returns: nonw
 */

static void
Destroy(w)
Widget w;
{
    SliderWidget sbw = (SliderWidget) w;
    
    XtReleaseGC(w, sbw->slider.gc);
}

/*	Function Name: CreateGC
 *	Description: Creates the GC.
 *	Arguments: w - the slider widget.
 *	Returns: none. 
 */

static void
CreateGC(w)
Widget w;
{
    SliderWidget sbw = (SliderWidget) w;
    XGCValues gcValues;
    XtGCMask mask;
    unsigned int depth = 1;

    if (sbw->slider.pixmap == XtUnspecifiedPixmap) {
        sbw->slider.pixmap = XmuCreateStippledPixmap (XtScreen(w), 
							(Pixel) 1, (Pixel) 0,
							depth);
    }
    else if (sbw->slider.pixmap != None) {
	Window root;
	int x, y;
	unsigned int width, height, bw;
	if (XGetGeometry(XtDisplay(w), sbw->slider.pixmap, &root, &x, &y,
			 &width, &height, &bw, &depth) == 0) {
	    XtAppError(XtWidgetToApplicationContext(w),
	       "Slider Widget: Could not get geometry of pixmap pixmap.");
	}
    }

    gcValues.foreground = sbw->slider.foreground;
    gcValues.background = sbw->core.background_pixel;
    mask = GCForeground | GCBackground;


    if (sbw->slider.pixmap != None) {
	if (depth == 1) {
	    gcValues.fill_style = FillOpaqueStippled;
	    gcValues.stipple = sbw->slider.pixmap;
	    mask |= GCFillStyle | GCStipple;
	}
	else {
	    gcValues.fill_style = FillTiled;
	    gcValues.tile = sbw->slider.pixmap;
	    mask |= GCFillStyle | GCTile;
	}
    }
    sbw->slider.gc = XtGetGC( w, mask, &gcValues);
}

/* ARGSUSED */
static void Initialize( request, new )
   Widget request;		/* what the client asked for */
   Widget new;			/* what we're going to give him */
{
    SliderWidget w = (SliderWidget) new;
    Cardinal num_args = 0;
	
    CreateGC(new);

    if (w->core.width == 0)
	w->core.width = (w->slider.orientation == XtorientVertical)
	    ? w->slider.thickness : w->slider.length;

    if (w->core.height == 0)
	w->core.height = (w->slider.orientation == XtorientHorizontal)
	    ? w->slider.thickness : w->slider.length;

    w->slider.value *= w->slider.step;
    SetDimensions( w );
    w->slider.direction = 0;
    w->slider.topLoc = w->slider.value;
    PaintSlide(w);
    
    
}
static void Realize( gw, valueMask, attributes )
   Widget gw;
   Mask *valueMask;
   XSetWindowAttributes *attributes;
{
    SliderWidget w = (SliderWidget) gw;

    w->slider.inactiveCursor =
      (w->slider.orientation == XtorientVertical)
	? w->slider.verCursor
	: w->slider.horCursor;

    attributes->cursor = w->slider.inactiveCursor;
    *valueMask |= CWCursor;
    
    XtCreateWindow( gw, InputOutput, (Visual *)CopyFromParent,
		    *valueMask, attributes );
}

/* ARGSUSED */
static Boolean 
SetValues( current, request, desired )
Widget current,		/* what I am */
       request,		/* what he wants me to be */
       desired;		/* what I will become */
{
    SliderWidget w = (SliderWidget) current;
    SliderWidget dw = (SliderWidget) desired;
    Boolean redraw = FALSE;

/*
 * If these values are outside the acceptable range ignore them...
 */
   dw->slider.value *= w->slider.step;
   if ((dw->slider.value) < 0 || (dw->slider.value > dw->slider.length))
   {
      fprintf(stderr,"out of range set values : %d \n", dw->slider.value);
      dw->slider.value = w->slider.value;
   }
      

/*
 * Change colors and stuff...
 */

    if ( XtIsRealized (desired) ) {
	if ( (w->slider.foreground != dw->slider.foreground) ||
	    (w->core.background_pixel != dw->core.background_pixel) ||
	    (w->slider.pixmap != dw->slider.pixmap) ) 
	{
	    XtReleaseGC((Widget) w, (XtPointer) w->slider.pixmap);
	    CreateGC( (Widget) dw);
	    redraw = TRUE;
	}
	if (w->slider.value != dw->slider.value)
	    redraw = TRUE;
    }
	   
    return( redraw );
}

static void Resize( gw )
   Widget gw;
{
    /* ForgetGravity has taken care of background, but pixmap may
     * have to move as a result of the new size. */
    SetDimensions( (SliderWidget)gw );
    Redisplay( gw, (XEvent*)NULL, (Region)NULL );
}


/* ARGSUSED */
static void Redisplay( gw, event, region )
   Widget gw;
   XEvent *event;
   Region region;
{
    SliderWidget w = (SliderWidget) gw;
    int x, y;
    unsigned int width, height;

    if (w->slider.orientation == XtorientHorizontal) {
	x = 1;
	y = 1;
	height = w->core.height - 2;
	width = w->slider.topLoc;
    } else {
	x = 1;
	y = 1;
	height = w->slider.topLoc;
	width = w->core.width - 2;
    }

    if ( (region == NULL) ||
	 (XRectInRegion(region, x, y, width, height) != RectangleOut) ) {
	/* Forces entire pixmap to be painted. */
	w->slider.topLoc = 1;
	PaintSlide( w ); 
    }
}


/* ARGSUSED */
static void StartSlide( gw, event, params, num_params )
  Widget gw;
  XEvent *event;
  String *params;		/* direction: Back|Forward|Smooth */
  Cardinal *num_params;		/* we only support 1 */
{
    SliderWidget w = (SliderWidget) gw;
    Cursor cursor;
    char direction;

    if (w->slider.direction != 0) return; /* if we're already slideing */
    if (*num_params > 0) direction = *params[0];
    else		 direction = 'C';

    w->slider.direction = direction;

    switch( direction ) {
	case 'B':
	case 'b':	cursor =  w->slider.rightCursor; break;

	case 'F':
	case 'f':	cursor =  w->slider.leftCursor; break;

	case 'C':
	case 'c':	cursor =  w->slider.upCursor; break;

	default:	return; /* invalid invocation */
    }

    XDefineCursor(XtDisplay(w), XtWindow(w), cursor);

    XFlush(XtDisplay(w));
}


static Boolean CompareEvents( oldEvent, newEvent )
    XEvent *oldEvent, *newEvent;
{
#define Check(field) if (newEvent->field != oldEvent->field) return False;

    Check(xany.display);
    Check(xany.type);
    Check(xany.window);

    switch( newEvent->type ) {
      case MotionNotify:
		Check(xmotion.state); break;
      case ButtonPress:
      case ButtonRelease:
		Check(xbutton.state);
		Check(xbutton.button); break;
      case KeyPress:
      case KeyRelease:
		Check(xkey.state);
		Check(xkey.keycode); break;
      case EnterNotify:
      case LeaveNotify:
		Check(xcrossing.mode);
		Check(xcrossing.detail);
		Check(xcrossing.state); break;
    }
#undef Check

    return True;
}

struct EventData {
	XEvent *oldEvent;
	int count;
};

static Bool PeekNotifyEvent( dpy, event, args )
    Display *dpy;
    XEvent *event;
    char *args;
{
    struct EventData *eventData = (struct EventData*)args;

    return ((++eventData->count == QLength(dpy)) /* since PeekIf blocks */
	    || CompareEvents(event, eventData->oldEvent));
}


static Boolean LookAhead( w, event )
    Widget w;
    XEvent *event;
{
    XEvent newEvent;
    struct EventData eventData;

    if (QLength(XtDisplay(w)) == 0) return False;

    eventData.count = 0;
    eventData.oldEvent = event;

    XPeekIfEvent(XtDisplay(w), &newEvent, PeekNotifyEvent, (char*)&eventData);

    if (CompareEvents(event, &newEvent))
	return True;
    else
	return False;
}


static void ExtractPosition( event, x, y )
    XEvent *event;
    Position *x, *y;		/* RETURN */
{
    switch( event->type ) {
      case MotionNotify:
		*x = event->xmotion.x;	 *y = event->xmotion.y;	  break;
      case ButtonPress:
      case ButtonRelease:
		*x = event->xbutton.x;   *y = event->xbutton.y;   break;
      default:
		  break;
    }
}

static void NotifySlide( gw, event, params, num_params   )
   Widget gw;
   XEvent *event;
   String *params;		/* style: Proportional|FullLength */
   Cardinal *num_params;	/* we only support 1 */
{
    static CalledBack = -9999999;
    SliderWidget w = (SliderWidget) gw;
    int x, y;

    if (w->slider.direction == 0) return; /* if no StartSlide */

    if (LookAhead(gw, event)) return;

    if (w->slider.value != CalledBack)
    {
      x = w->slider.value / w->slider.step;
      XtCallCallbacks( gw, XtNslideProc,(XtPointer) x);
      CalledBack = w->slider.value;
    }
    
}

/* ARGSUSED */
static void EndSlide(gw, event, params, num_params )
   Widget gw;
   XEvent *event;		/* unused */
   String *params;		/* unused */
   Cardinal *num_params;	/* unused */
{
    SliderWidget w = (SliderWidget) gw;

    XDefineCursor(XtDisplay(w), XtWindow(w), w->slider.inactiveCursor);
    XFlush(XtDisplay(w));

    w->slider.direction = 0;
}


/* ARGSUSED */
static void MoveSlide( gw, event, params, num_params )
   Widget gw;
   XEvent *event;
   String *params;		/* unused */
   Cardinal *num_params;	/* unused */
{
    SliderWidget w = (SliderWidget) gw;
    Position x, y;

    if (w->slider.direction == 0) return; /* if no StartSlide */

    if (LookAhead(gw, event)) return;

    ExtractPosition(event, &x, &y );

    if (x < 0)
      x = 0;
    if (x > w->slider.length)
      x = w->slider.length;

    w->slider.value = x;
    PaintSlide(w);
    XFlush(XtDisplay(w));	/* re-draw it before Notifying */
}

/* ARGSUSED */
static void MovesSlide( gw, event, params, num_params )
   Widget gw;
   XEvent *event;
   String *params;		/* unused */
   Cardinal *num_params;	/* unused */
{
    SliderWidget w = (SliderWidget) gw;

    if (w->slider.direction == 0) return; /* if no StartSlide */

    if (LookAhead(gw, event)) return;

    if (w->slider.direction != 'F')
    {
      if (w->slider.value + w->slider.step < w->slider.length - 1)
	 w->slider.value += w->slider.step;
      else
	 w->slider.value = w->slider.length;
    }
    else
    {
      if (w->slider.value - w->slider.step > 0)      
	 w->slider.value -= w->slider.step;
      else
	 w->slider.value = 0;
    }
    PaintSlide(w);
    XFlush(XtDisplay(w));	/* re-draw it before Notifying */
}


