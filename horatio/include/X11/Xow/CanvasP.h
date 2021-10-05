/*SccsID (c) Jon Tomb & Mark Bush @(#)CanvasP.h	1.4  9/11/91*/

#ifndef _CanvasP_h
#define _CanvasP_h

#include <X11/Xow/Canvas.h>
/* include superclass private header file */
#include <X11/CoreP.h>

/* define unique representation types not found in <X11/StringDefs.h> */

#define XtRCanvasResource "CanvasResource"


typedef struct {
    Pixel color;
    XFontStruct* font;
    XtCallbackList expose_callback;
    XtCallbackList canvasAction_callback;
} XowCanvasPart;


typedef struct {
    int empty;
} XowCanvasClassPart;

typedef struct _XowCanvasClassRec {
    CoreClassPart	core_class;
    XowCanvasClassPart	canvas_class;
} XowCanvasClassRec;

extern XowCanvasClassRec XowcanvasClassRec;


typedef struct _XowCanvasRec {
    CorePart		core;
    XowCanvasPart	canvas;
} XowCanvasRec;

#endif /* _CanvasP_h */
