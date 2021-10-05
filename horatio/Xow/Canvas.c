
static char SccsID[]="(c) Jon Tomb & Mark Bush %W%	%G%";
/* Based on the example window widget from the Xaw programers guide */
#include <X11/IntrinsicP.h>
#include <X11/StringDefs.h>
#include <X11/Xow/CanvasP.h>

static XtResource resources[] = {
#define offset(field) XtOffset(CanvasWidget, canvas.field)
   /* {name, class, type, size, offset, default_type, default_addr}, */
   {XtNdrawingColor, XtCColor, XtRPixel, sizeof(Pixel),
   offset(color), XtRString, XtDefaultForeground},
   {XtNfont, XtCFont, XtRFontStruct, sizeof(XFontStruct *),
   offset(font), XtRString, XtDefaultFont},
   {XtNexposeCallback, XtCCallback, XtRCallback, sizeof(XtCallbackList),
   offset(expose_callback), XtRCallback, NULL},
   {XtNcallback, XtCCallback, XtRCallback, sizeof(XtCallbackList),
   offset(canvasAction_callback), XtRCallback, NULL},
#undef offset
};

static void CanvasAction( /* Widget, XEvent*, String*, Cardinal* */ );

static void 
InputAction(w, event, params, num_params)
   Widget  w;
   XEvent *event;
   String *params;
   Cardinal *num_params;
{
   XtCallCallbacks(w, XtNcallback, (caddr_t) event);
}


static XtActionsRec actions[] =
{
   /* {name, procedure}, */
   {"canvasAction", InputAction},
};

static char translations[] =
"<Key>:         canvasAction() \n\
 <BtnDown>:     canvasAction() \n\
 <BtnUp>:       canvasAction() \n\
 <BtnMotion>:   canvasAction() \n\
 <Motion>:     canvasAction() \
";

static void 
Redisplay(w, event, region)
   Widget  w;
   XEvent *event;
   Region  region;
{
   XtCallCallbacks(w, XtNexposeCallback, (caddr_t) region);
}

CanvasClassRec canvasClassRec = {
   {                            /* core fields */
       /* superclass             */ (WidgetClass) & widgetClassRec,
       /* class_name             */ "Canvas",
       /* widget_size            */ sizeof(CanvasRec),
       /* class_initialize               */ NULL,
       /* class_part_initialize  */ NULL,
       /* class_inited           */ FALSE,
       /* initialize             */ NULL,
       /* initialize_hook                */ NULL,
       /* realize                        */ XtInheritRealize,
       /* actions                        */ actions,
       /* num_actions            */ XtNumber(actions),
       /* resources              */ resources,
       /* num_resources          */ XtNumber(resources),
       /* xrm_class              */ NULLQUARK,
       /* compress_motion                */ TRUE,
       /* compress_exposure      */ TRUE,
       /* compress_enterleave    */ TRUE,
       /* visible_interest               */ FALSE,
       /* destroy                        */ NULL,
       /* resize                         */ NULL,
       /* expose                         */ Redisplay,
       /* set_values             */ NULL,
       /* set_values_hook                */ NULL,
       /* set_values_almost      */ XtInheritSetValuesAlmost,
       /* get_values_hook                */ NULL,
       /* accept_focus           */ NULL,
       /* version                        */ XtVersion,
       /* callback_private               */ NULL,
       /* tm_table                       */ translations,
       /* query_geometry                 */ XtInheritQueryGeometry,
       /* display_accelerator    */ XtInheritDisplayAccelerator,
       /* extension              */ NULL
   },
   {                            /* canvas fields */
       /* empty                  */ 0
   }
};

Pixel 
WindowColor(w)
   Widget  w;
{
   return (((CanvasWidget) w)->canvas.color);
}

Font 
WindowFont(w)
   Widget  w;
{
   return (((CanvasWidget) w)->canvas.font->fid);
}

WidgetClass canvasWidgetClass = (WidgetClass) & canvasClassRec;
