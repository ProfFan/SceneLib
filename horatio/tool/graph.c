/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/Shell.h>
#include <X11/StringDe.h>
#include <X11/cursorfo.h>

#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/Canvas.h>
#else
#include <X11/Intrinsic.h>
#include <X11/Shell.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/Canvas.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/graphics.h"
#include "horatio/tool.h"

typedef struct graph_def
{
   Widget          parent;      /* parent of popup graph widget */
   Widget          popup_frame; /* transient panel frame */
   Widget          popup_panel; /* form widget containing canvases */
   Widget          canvas;      /* canvas on which graph is drawn */
   Hor_Assoc_Label label;       /* label for defining canvas to graphics
				   library */
   u_long          background;  /* background colour */
   int       width,   height;   /* dimensions of graph canvas */
   float     width_f, height_f; /* floating-point versions of above */
   Hor_Bool        in_use;      /* whether graph is currently displayed */

   Hor_Bool  params_set;            /* whether offset and scaling parameters
				       are set */
   float t_low, t_range, t_scale;   /* horizontal offset, range and scaling */
   float f_low, f_scale;            /* vertical offset and scaling */

   Hor_Assoc_List trace_list;  /* list of traces on the same graph */
   Hor_Bool       bottom_left; /* HOR_TRUE if this graph is at the bottom left of
				  the panel it is in */
   Widget string;      /* string widget for displaying mouse position */
} Graph_Def;

static Hor_Assoc_List  graph_list = NULL;
static Hor_Assoc_Label next_label = HOR_ASSOC_START;

/*******************
*   Hor_Assoc_Label @hor_create_graph ( Widget parent,
*                                      Hor_Assoc_Label horiz_label,
*                                      Hor_Assoc_Label vert_label,
*                                      int width, int height,
*                                      u_long background_colour )
*
*   Creates a popup graph panel for dynamic graph drawing. The returned label
*   for the panel is then used in subsequent calls to hor_popup_graph() in
*   order to invoke the panel. The parent will typically be the button that
*   will invoke the panel.
********************/
Hor_Assoc_Label hor_create_graph ( Widget parent,
				   Hor_Assoc_Label horiz_label,
				   Hor_Assoc_Label vert_label,
				   int width, int height,
				   u_long background_colour )
{
   Graph_Def *def = hor_malloc_type ( Graph_Def ), *adj_def;
   Arg        args[4];
   Cardinal   no_args = ZERO;

   if ( width <= 0 || height <= 0 )
      hor_error ( "illegal graph dimensions (hor_create_graph)", HOR_FATAL );

   /* only create new popup window if new graph is not specified as being
      adjacent to a previously registered graph */
   if ( horiz_label == HOR_ASSOC_ERROR && vert_label == HOR_ASSOC_ERROR )
   {
      def->parent      = parent;
      def->popup_frame = XtVaCreatePopupShell ( "Graph",
					        transientShellWidgetClass,
					        parent, NULL );
      def->popup_panel = XtVaCreateManagedWidget ( "form", formWidgetClass,
						   def->popup_frame, NULL );
      def->bottom_left = HOR_TRUE;
      def->string = XtVaCreateManagedWidget ( "String", labelWidgetClass,
					      def->popup_panel, NULL );
   }
   else
   {
      def->popup_frame = def->popup_panel = def->string = NULL;
      def->bottom_left = HOR_FALSE;
   }

   XtSetArg ( args[no_args], XtNwidth,  width ); no_args++;
   XtSetArg ( args[no_args], XtNheight, height ); no_args++;
   if ( horiz_label != HOR_ASSOC_ERROR )
   {
      if ( (adj_def = hor_assoc_find ( graph_list, horiz_label )) == NULL )
	 hor_error("illegal horizontal adjacent graph (hor_create_graph)",HOR_FATAL);

      if ( def->popup_frame == NULL )
      {
	 def->popup_frame = adj_def->popup_frame;
	 def->popup_panel = adj_def->popup_panel;
	 def->string      = adj_def->string;
      }
      else if ( def->popup_panel != adj_def->popup_panel )
	 hor_error ( "adjacencies defined between graphs on different panels (hor_create_graph)", HOR_FATAL );

      XtSetArg ( args[no_args], XtNfromHoriz, adj_def->canvas ); no_args++;
   }

   if ( vert_label != HOR_ASSOC_ERROR )
   {
      if ( (adj_def = hor_assoc_find ( graph_list, vert_label )) == NULL )
	 hor_error("illegal vertical adjacent graph (hor_create_graph)",HOR_FATAL);

      if ( def->popup_frame == NULL )
      {
	 def->popup_frame = adj_def->popup_frame;
	 def->popup_panel = adj_def->popup_panel;
	 def->string      = adj_def->string;
      }
      else if ( def->popup_panel != adj_def->popup_panel )
	 hor_error ( "adjacencies defined between graphs on different panels (hor_create_graph)", HOR_FATAL );

      XtSetArg ( args[no_args], XtNfromVert, adj_def->canvas ); no_args++;

      /* check if this is the new bottom left graph in the panel */
      if ( horiz_label == HOR_ASSOC_ERROR && adj_def->bottom_left )
      {
	 def->bottom_left     = HOR_TRUE;
	 adj_def->bottom_left = HOR_FALSE;
      }
   }

   def->canvas = XtCreateManagedWidget ( "canvas", canvasWidgetClass,
					 def->popup_panel, args, no_args );

   def->background = background_colour;
   def->width      = width;
   def->width_f    = (float) width;
   def->height     = height;
   def->height_f   = (float) height;
   def->in_use     = HOR_FALSE;
   def->params_set = HOR_FALSE;
   def->label      = HOR_ASSOC_ERROR;
   graph_list = hor_assoc_insert ( graph_list, next_label, (void *) def );
   return next_label++;
}

static void update_position ( int c, int r, void *data )
{
   Graph_Def *def = (Graph_Def *) data;
   char s[100];

   sprintf ( s, "t=%7.2f f=%7.2f",
	     ((float) c + 0.5F)/def->t_scale + def->t_low,
	     (def->height_f - (float) r - 0.5F)/def->f_scale + def->f_low );
   XtVaSetValues ( def->string, XtNlabel, s, NULL );
}

static void reset_string ( void *data )
{
   Graph_Def *def = (Graph_Def *) data;

   XtVaSetValues ( def->string, XtNlabel, "t=        f=       ", NULL );
}

/*******************
*   void @hor_register_graphs ( Display *display )
*
*   Once all the graphs have been created, this must be called to create the
*   string label for position display and registering all the graphs with
*   the graphics library.
********************/
void hor_register_graphs ( Display *display )
{
   Hor_Assoc_List           list;
   Graph_Def           *def;
   XSetWindowAttributes sw;
   Hor_Assoc_Label          old_label = hor_display_get_window();

   sw.backing_store = Always;
   for ( list = graph_list; list != NULL; list = list->next )
   {
      def = (Graph_Def *) hor_assoc_data(list);

      if ( def->bottom_left ) /* create position display string window */
	 XtVaSetValues ( def->string, XtNfromVert, def->canvas,
			              XtNlabel,    "t=        f=       ",
			              NULL );
   }

   for ( list = graph_list; list != NULL; list = list->next )
   {
      def = (Graph_Def *) hor_assoc_data(list);

      XtRealizeWidget ( def->popup_frame );
      def->label = hor_display_set_window ( XtWindow(def->canvas), def->canvas,
				        XC_cross_reverse, HOR_ASSOC_ERROR );
      hor_display_set_params ( def->width, def->height );
      hor_display_set_mouse_functions ( def->label,
				    NULL, NULL, "",
				    NULL, NULL, "",
				    NULL, NULL, "", update_position,
				    NULL, reset_string, def );

      /* tell X to update graph when is is redisplayed */
      XChangeWindowAttributes ( display, XtWindow(def->canvas),
			        CWBackingStore, &sw);
   }

   hor_display_reset_window ( old_label );
}

typedef struct
{
   Hor_DList start_point;
   Hor_DList first_point;
   Hor_DList last_point;
   Hor_DList end_point;
} Trace_Def;

/*******************
*   Hor_Bool @hor_popup_graph ( Widget button, Hor_Assoc_Label graph_label,
*                              float t_range, float f_low, float f_high, ... )
*
*   Causes a graph to appear on the screen. The button argument is used to
*   place the graph panel. It will normally be the button that invoked the,
*   graph, but can be NULL, in which case the parent widget passed into
*   hor_create_graph() is used to place the panel.
*   The variable argument list is a NULL-terminated list of pointers of type
*   Hor_Assoc_Label *, one for each trace on the graph.
********************/
Hor_Bool hor_popup_graph ( Widget button, Hor_Assoc_Label graph_label,
			   float t_range, float f_low, float f_high, ... )
{
   Graph_Def   *def;
   Position     x, y;
   Hor_Assoc_Label *trace_label;
   Trace_Def   *tdef;
   va_list      ap;
   Hor_Assoc_List   list;
   Hor_Bool         in_use;
   Graph_Def       *gdef;
   Hor_Assoc_Label  old_label = hor_display_get_window();

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_popup_graph)",
		  HOR_FATAL, graph_label );

   if ( def->label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( t_range <= 0.0F )
   {
      hor_errno = HOR_TOOL_GRAPH_NEGATIVE_TIME_RANGE;
      return HOR_FALSE;
   }

   if ( f_low >= f_high )
   {
      hor_errno = HOR_TOOL_GRAPH_NEGATIVE_F_RANGE;
      return HOR_FALSE;
   }

   if ( def->params_set )
   {
      hor_errno = HOR_TOOL_GRAPH_ALREADY_SET_UP;
      return HOR_FALSE;
   }

   if ( !(in_use = def->in_use) )
   {
      for ( list = graph_list; list != NULL; list = list->next )
      {
	 gdef = (Graph_Def *) hor_assoc_data(list);
	 if ( gdef->popup_panel == def->popup_panel ) /* includes def */
	 {
	    gdef->trace_list = NULL;
	    gdef->params_set = HOR_FALSE;
	    gdef->in_use     = HOR_TRUE;
	    hor_display_reset_window ( gdef->label );
	    hor_display_clear ( gdef->background );
	 }
      }
   }

   hor_display_reset_window ( old_label );

   /* read argument list */
   va_start ( ap, f_high );
   for(;;)
   {
      trace_label = va_arg ( ap, Hor_Assoc_Label * );
      if ( trace_label == NULL ) break;

      tdef = hor_malloc_type ( Trace_Def );
      tdef->start_point = tdef->end_point =
      tdef->first_point = tdef->last_point = NULL;
      *trace_label = next_label;
      def->trace_list = hor_assoc_insert (def->trace_list, next_label++, tdef);
   }

   va_end(ap);

   /* set up horizontal and vertical scaling and offset parameters */
   def->t_low = 0.0; def->t_range = t_range;
   def->t_scale = def->width_f/t_range;
   def->f_low = f_low; def->f_scale = def->height_f/(f_high-f_low);
   def->params_set = HOR_TRUE;

   if ( !in_use )
   {
      if ( button == NULL ) button = def->parent;
      XtTranslateCoords( button, (Position) 30, (Position) 0, &x, &y );
      XtVaSetValues ( def->popup_frame,
		      XtNx, x,
		      XtNy, y,
		      NULL );
      XtPopup(def->popup_frame, XtGrabNone);
   }

   return HOR_TRUE;
}

typedef struct
{
   float  t, f;   /* coordinates of point */
   u_long colour; /* colour of line between this point and the previous point*/
} Graph_Point;

static void free_graph ( void *ptr )
{
   hor_dfree_list ( ((Trace_Def *) ptr)->last_point, hor_free_func );
   hor_free ( ptr );
}

/*******************
*   Hor_Bool @hor_popdown_graph ( Hor_Assoc_Label graph_label )
*
*   Causes a graph to disppear, losing all the data displayed in it.
********************/
Hor_Bool hor_popdown_graph ( Hor_Assoc_Label graph_label )
{
   Graph_Def *def, *gdef;
   Hor_Assoc_List list;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_popdown_graph)",
		  HOR_FATAL, graph_label );

   if ( !def->in_use ) return HOR_FALSE;

   for ( list = graph_list; list != NULL; list = list->next )
   {
      gdef = (Graph_Def *) hor_assoc_data(list);
      if ( gdef->popup_panel == def->popup_panel ) /* includes def */
      {
	 hor_assoc_free ( gdef->trace_list, free_graph );
	 gdef->params_set = HOR_FALSE;
	 gdef->in_use     = HOR_FALSE;
      }
   }

   XtPopdown ( def->popup_frame );
   return HOR_TRUE;
}

static void draw_graphs ( Hor_Assoc_List trace_list, Hor_Bool colour_flag,
			  float t_low, float t_scale )
{
   Trace_Def   *tdef;
   Hor_DList        first_point;
   Graph_Point *point;
   float        old_t, old_f, new_t, new_f;
   u_long       old_colour, new_colour;

   if ( trace_list == NULL ) return;

   tdef = (Trace_Def *) hor_assoc_data ( trace_list );

   first_point = tdef->first_point;
   if ( first_point != NULL )
   {
      point = (Graph_Point *) first_point->contents;
      old_t = (point->t - t_low)*t_scale;
      old_f = point->f;
      old_colour = point->colour;
      if ( colour_flag ) hor_display_set_colour ( old_colour );
      for ( first_point = first_point->next; first_point != NULL;
	    first_point = first_point->next )
      {
	 point = (Graph_Point *) first_point->contents;
	 new_t      = (point->t - t_low)*t_scale;
	 new_f      = point->f;
	 new_colour = point->colour;
	 if ( colour_flag && new_colour != old_colour )
	    hor_display_set_colour ( new_colour );

	 hor_display_line ( old_t, old_f, new_t, new_f );
	 old_t = new_t; old_f = new_f; old_colour = new_colour;
      }
   }

   draw_graphs ( hor_assoc_next ( trace_list ), colour_flag, t_low, t_scale );
}

static void first_point_forward ( Hor_Assoc_List trace_list, float t_low )
{
   Trace_Def   *tdef;
   Graph_Point *point;
   Hor_DList        plist;

   if ( trace_list == NULL ) return;

   tdef = (Trace_Def *) hor_assoc_data(trace_list);
   for ( plist = tdef->first_point; plist != NULL;
	 plist = plist->next )
   {
      point = (Graph_Point *) plist->contents;
      if ( point->t >= t_low ) break;
   }

   if ( plist == NULL ) tdef->first_point = tdef->end_point;
   else if ( plist != tdef->first_point )
           tdef->first_point = hor_prev_dnode(plist);

   first_point_forward ( hor_assoc_next(trace_list), t_low );
}

static Hor_Bool initialize_min_max_ft ( Hor_Assoc_List trace_list,
				        float *t_min, float *t_max,
				        float *f_min, float *f_max )
{
   Trace_Def   *tdef;
   Graph_Point *point;

   for ( ; trace_list != NULL; trace_list = hor_assoc_next(trace_list) )
   {
      tdef = hor_assoc_data(trace_list);
      if ( tdef->first_point != NULL )
      {
	 point = (Graph_Point *) tdef->first_point->contents;
	 *t_min = *t_max = point->t;
	 *f_min = *f_max = point->f;
	 return HOR_TRUE;
      }
   }

   return HOR_FALSE;
}

static Hor_Bool find_min_max_tf ( Hor_Assoc_List trace_list,
				  float *t_min, float *t_max,
				  float *f_min, float *f_max )
{
   Trace_Def   *tdef;
   Hor_DList    plist;
   Graph_Point *point;

   if ( !initialize_min_max_ft ( trace_list, t_min, t_max, f_min, f_max ) )
      return HOR_FALSE;

   for ( ; trace_list != NULL; trace_list = hor_assoc_next(trace_list) )
   {
      tdef = hor_assoc_data(trace_list);
      for ( plist = tdef->first_point; plist != NULL; plist = plist->next )
      {
	 point = (Graph_Point *) plist->contents;
	 if ( point->t < *t_min ) *t_min = point->t;
	 if ( point->t > *t_max ) *t_max = point->t;
	 if ( point->f < *f_min ) *f_min = point->f;
	 if ( point->f > *f_max ) *f_max = point->f;
      }
   }

   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_graph_point ( Hor_Assoc_Label graph_label,
*                              Hor_Assoc_Label trace_label,
*                              float t, float f, u_long colour )
*
*   Adds new point to graph, returning HOR_TRUE on success, HOR_FALSE on failure.
********************/
Hor_Bool hor_graph_point ( Hor_Assoc_Label graph_label,
			   Hor_Assoc_Label trace_label, float t, float f,
			   u_long colour )
{
   Graph_Def   *def;
   Trace_Def   *tdef;
   Graph_Point *point;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_graph_point)",
		  HOR_FATAL, graph_label );

   if ( !def->in_use )
      hor_error ( "graph not in use (hor_graph_point)", HOR_FATAL );

   if ( !def->params_set )
      hor_error ( "graph parameters not set (hor_graph_point)", HOR_FATAL );

   tdef = (Trace_Def *) hor_assoc_find ( def->trace_list, trace_label );
   if ( tdef == NULL )
      hor_error ( "undefined graph trace label %d (hor_graph_point)",
		  HOR_FATAL, trace_label );

   /* create new point */
   point = hor_malloc_type(Graph_Point);
   point->t      = t;
   point->f      = def->height_f - (f - def->f_low)*def->f_scale;
   point->colour = colour;

   if ( tdef->last_point == NULL )
   {
      tdef->start_point = tdef->end_point =
      tdef->first_point = tdef->last_point = hor_dmake_straight((void *)point);
      def->t_low = t;
   }
   else
   {
      float t_min, t_max, f_min, f_max;
      Hor_Assoc_Label old_label = hor_display_get_window();

      /* compare new time with last time */
      if ( t < ((Graph_Point *) tdef->end_point->contents)->t )
      {
	 hor_errno = HOR_TOOL_GRAPH_TIME_REVERSAL;
	 hor_free ( (void *) point );
	 return HOR_FALSE;
      }

      /* reset time if current display (set by hor_graph_reset_time())
	 is off right hand edge */
      find_min_max_tf ( def->trace_list, &t_min, &t_max, &f_min, &f_max );
      if ( def->t_low > t_max ) def->t_low = t_max - def->t_range;

      hor_display_reset_window ( def->label );
      hor_display_set_function ( HOR_DISPLAY_COPY );
      hor_display_set_line_width ( 0 );
      if ( t > def->t_low + def->t_range ) /* must reset time offset */
      {
	 /* erase old graphs */
	 hor_display_set_colour ( def->background );
	 draw_graphs ( def->trace_list, HOR_FALSE, def->t_low, def->t_scale );

	 /* reset time offset and discard points falling off left edge of
	    graph */
	 def->t_low = t - def->t_range;
	 first_point_forward ( def->trace_list, def->t_low );

	 /* re-draw graphs with new point added */
	 tdef->last_point = tdef->end_point
	                  = hor_dinsert_after(tdef->end_point, (void *) point);
	 draw_graphs ( def->trace_list, HOR_TRUE, def->t_low, def->t_scale );
      }
      else /* simply draw line between previous point and new point */
      {
	 Graph_Point *end_point;

	 end_point = (Graph_Point *) tdef->end_point->contents;
	 hor_display_set_colour ( colour );
	 hor_display_line ( (end_point->t - def->t_low)*def->t_scale,
		        end_point->f,
		        (point->t - def->t_low)*def->t_scale, point->f );

	 /* add point to list */
	 tdef->end_point = tdef->last_point
	                 = hor_dinsert_after (tdef->end_point, (void *) point);
      }

      hor_display_reset_window ( old_label );
   }

   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_reset_graph ( Hor_Assoc_Label graph_label )
*
*   Clears graph and removes points from each trace.
********************/
Hor_Bool hor_reset_graph ( Hor_Assoc_Label graph_label )
{
   Graph_Def      *def;
   Hor_Assoc_Label old_label = hor_display_get_window();
   Hor_Assoc_List  tlist;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_reset_graph)",
		  HOR_FATAL, graph_label );

   if ( def->label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->params_set )
   {
      hor_errno = HOR_TOOL_GRAPH_NOT_SET_UP;
      return HOR_FALSE;
   }

   def->t_low = 0.0;
   for ( tlist = def->trace_list; tlist != NULL; tlist = tlist->next )
   {
      Trace_Def *tdef = (Trace_Def *) tlist->data;
      hor_dfree_list ( tdef->last_point, hor_free_func );
      tdef->start_point = tdef->end_point =
      tdef->first_point = tdef->last_point = NULL;
   }

   hor_display_reset_window ( def->label );
   hor_display_clear ( def->background );
   hor_display_reset_window ( old_label );
   return HOR_TRUE;
}

/*******************
*   void @hor_graph_reset_time ( Hor_Assoc_Label graph_label, float t_low )
*
*   Shifts graph so that given time becomes left edge of graph.
********************/
void hor_graph_reset_time ( Hor_Assoc_Label graph_label, float t_low )
{
   Graph_Def  *def;
   Trace_Def  *tdef;
   Hor_Assoc_List  trace_list;
   Hor_Assoc_Label old_label = hor_display_get_window();

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_graph_point)",
		  HOR_FATAL, graph_label );

   if ( !def->in_use )
      hor_error ( "graph not in use (hor_graph_point)", HOR_FATAL );

   if ( !def->params_set )
      hor_error ( "graph parameters not set (hor_graph_point)", HOR_FATAL );

   /* erase old graphs */
   hor_display_reset_window ( def->label );
   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_line_width ( 0 );
   hor_display_set_colour ( def->background );
   draw_graphs ( def->trace_list, HOR_FALSE, def->t_low, def->t_scale );

   for ( trace_list = def->trace_list; trace_list != NULL;
	 trace_list = hor_assoc_next(trace_list) )
   {
      tdef = (Trace_Def *) hor_assoc_data(trace_list);
      tdef->first_point = tdef->start_point;
      tdef->last_point  = tdef->end_point;
   }

   def->t_low = t_low;
   draw_graphs ( def->trace_list, HOR_TRUE, def->t_low, def->t_scale );
   hor_display_reset_window ( old_label );
}

/*******************
*   Hor_Bool @hor_graph_in_use ( Hor_Assoc_Label graph_label )
*
*   Returns HOR_FALSE if a registered (by hor_create_graph()) graph
*   is not in use, HOR_TRUE if it is, i.e. if hor_popup_graph() has been called
*   for it and the "Done" button not yet pressed.
********************/
Hor_Bool hor_graph_in_use ( Hor_Assoc_Label graph_label )
{
   Graph_Def *def;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_graph_in_use)",
		  HOR_FATAL, graph_label );

   return ( def->in_use );
}

/*******************
*   Hor_Bool @hor_redisplay_graph ( Hor_Assoc_Label graph_label )
*
*   Causes a popup graph to appear on the screen.
*   It should only be called during use of the graph, i.e. between the calls
*   to hor_popup_graph() and pressing the "Done" button.
*   It pops up the graph to the front.
********************/
Hor_Bool hor_redisplay_graph ( Hor_Assoc_Label graph_label )
{
   Graph_Def *def;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_redisplay_graph)",
		  HOR_FATAL, graph_label );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   XtPopup(def->popup_frame, XtGrabNone);
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_write_graph ( Hor_Assoc_Label graph_label,
*                              const char *file_name )
*
*   Writes the current contents of the specified graph to a file in gtool
*   format.
********************/
Hor_Bool hor_write_graph ( Hor_Assoc_Label graph_label, const char *file_name )
{
   Graph_Def     *def;
   float          t_min, t_max, f_min, f_max;
   FILE          *fdesc;
   Trace_Def     *tdef;
   Hor_DList      plist;
   Graph_Point   *point;
   Hor_Assoc_List trace_list;

   def = (Graph_Def *) hor_assoc_find ( graph_list, graph_label );
   if ( def == NULL )
      hor_error ( "graph with label %d does not exist (hor_write_graph)",
		  HOR_FATAL, graph_label );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( !find_min_max_tf ( def->trace_list, &t_min, &t_max, &f_min, &f_max ) )
   {
      hor_errno = HOR_TOOL_GRAPH_NO_POINTS;
      return HOR_FALSE;
   }

   fdesc = fopen ( file_name, "w" );
   if ( fdesc == NULL )
   {
      hor_errno = HOR_TOOL_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   fprintf ( fdesc, "# Range of horizontal (time) axis (%f,%f)\n",
	     t_min, t_max );
   fprintf ( fdesc, "# Range of vertical axis (%f,%f)\n",
	     def->f_low + (def->height_f - f_min)/def->f_scale,
	     def->f_low + (def->height_f - f_max)/def->f_scale );

   for ( trace_list = def->trace_list; trace_list != NULL;
	 trace_list = hor_assoc_next(trace_list) )
   {
      tdef = hor_assoc_data(trace_list);
      fprintf ( fdesc, "\n" ); /* separator between sets of data */
      for ( plist = tdef->start_point; plist != NULL; plist = plist->next )
      {
	 point = (Graph_Point *) plist->contents;
	 fprintf ( fdesc, "%f %f\n", point->t,
		   def->f_low + (def->height_f - point->f)/def->f_scale );
      }
   }

   fclose ( fdesc );
   return HOR_TRUE;
}
