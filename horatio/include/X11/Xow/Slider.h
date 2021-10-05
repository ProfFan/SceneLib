/* from
* $XConsortium: Scrollbar.h,v 1.1 89/12/15 11:40:43 kit Exp $
*/

/*SccsID (c) Jon Tomb & Mark Bush @(#)Slider.h	1.6  9/11/91*/

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

#ifndef _Slider_h
#define _Slider_h

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
 * Slider Widget
 *
 ****************************************************************/

#include <X11/Xmu/Converters.h>

/* Parameters:

 Name		     Class		RepType		Default Value
 ----		     -----		-------		-------------
 background	     Background		Pixel		White
 border		     BorderColor	Pixel		Black
 borderWidth	     BorderWidth	Dimension	1
 destroyCallback     Callback		Function		NULL
 foreground	     Color		Pixel		Black
 height		     Height		Dimension	length or thickness
 jumpProc	     Callback		Function	NULL
 length		     Length		Dimension	1
 mappedWhenManaged   MappedWhenManaged	Boolean		True
 orientation	     Orientation	XtOrientation	XtorientVertical
 reverseVideo	     ReverseVideo	Boolean		False
 slideDCursor	     Cursor		Cursor		XC_sb_down_arrow
 slideHCursor	     Cursor		Cursor		XC_sb_h_double_arrow
 slideLCursor	     Cursor		Cursor		XC_sb_left_arrow
 slideProc	     Callback		Function	NULL
 slideRCursor	     Cursor		Cursor		XC_sb_right_arrow
 slideUCursor	     Cursor		Cursor		XC_sb_up_arrow
 slideVCursor	     Cursor		Cursor		XC_sb_v_double_arrow
 sensitive	     Sensitive		Boolean		True
 shown		     Shown		float		0.0
 thickness	     Thickness		Dimension	14
 thumb		     Thumb		Pixmap		Grey
 value		     Value		float		0.0
 width		     Width		Dimension	thickness or length
 x		     Position		Position	0
 y		     Position		Position	0

*/

/* 
 * Most things we need are in StringDefs.h 
 */

#define XtCstep	     "Step"
#define XtCsteps     "Steps"
#define XtNsteps     "steps"
#define XtNstep	     "step"
#define XtNslideProc "slideProc"

typedef struct _XowSliderRec	  *XowSliderWidget;
typedef struct _XowSliderClassRec *XowSliderWidgetClass;

extern WidgetClass XowsliderWidgetClass;


/* provide a Motif-like convenience function */

extern Widget XowCreateSlider ARGS((Widget parent,
				    char *name,
				    ArgList args,
				    int nargs));



#endif /* _Slider_h */
