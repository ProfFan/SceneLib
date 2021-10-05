/* from
 * $XConsortium: ScrollbarP.h,v 1.1 89/12/15 11:41:03 kit Exp $
 */

/*SccsID (c) Jon Tomb & Mark Bush @(#)SliderP.h	1.5  9/11/91*/

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

#ifndef _SliderP_h
#define _SliderP_h

#include <X11/Xow/Slider.h>

typedef struct {
     /* public */
    Pixel	  foreground;	/* thumb foreground color */
    XtOrientation orientation;	/* horizontal or vertical */
    XtCallbackList slideProc;	/* proportional slide */
    XtCallbackList jumpProc;	/* jump adjust */
    Pixmap	  pixmap;	/* thumb color */
    Cursor	  upCursor;	/* slide up cursor */
    Cursor	  downCursor;	/* slide down cursor */
    Cursor	  leftCursor;	/* slide left cursor */
    Cursor	  rightCursor;	/* slide right cursor */
    Cursor	  verCursor;	/* slide vertical cursor */
    Cursor	  horCursor;	/* slide horizontal cursor */
    int		  stepSize;     /* How much to move in a step */
    int		  value;        /* the length value */
    int		  steps;        /* number of steps in length */
    int		  step;        /* size to step  */        
    Boolean       interger;     /* float or interger */
    Dimension	  length;	/* either height or width */
    Dimension	  thickness;	/* either width or height */
     /* private */
    Position	  topLoc;	/* Pixel that corresponds to top */
    Cursor	  inactiveCursor; /* The normal cursor for slider */
    char	  direction;	/* a slide has started; which direction */
    GC		  gc;		/* a (shared) gc */
} XowSliderPart;

typedef struct _XowSliderRec {
    CorePart		core;
    XowSliderPart	slider;
} XowSliderRec;

typedef struct {
      int empty;
} XowSliderClassPart;

typedef struct _XowSliderClassRec {
    CoreClassPart		core_class;
    XowSliderClassPart		slider_class;
} XowSliderClassRec;

extern XowSliderClassRec XowsliderClassRec;

#endif /* _SliderP_h */
