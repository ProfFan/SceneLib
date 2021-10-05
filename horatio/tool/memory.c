/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#include <X11/Shell.h>

#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Form.h>

#include <X11/Xow/PanelTe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Form.h>

#include <X11/Xow/PanelText.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/tool.h"

#define DISPLAY_NUMBER_DEFAULT "1"

typedef struct
{
   XtAppContext app_con;
   Widget       parent;
   Widget       popup_frame;
   Widget       done_button;
   Widget       time_interval_text;
   Widget       display_number_text;

   Hor_Bool  in_use;
   Hor_List  list;
   Hor_DList result_list;
   int       list_length;
   int       current_pos;
   Hor_DList current_node;

   void (*display_func)(void *);
   void (*select_func)(void *);
   void (*wrap_forwards_func)(void *);
   void (*wrap_backwards_func)(void *);
   void (*free_func)(Hor_List);

   double default_time_interval;

   Hor_Bool     timeout_flag;
   u_long       timeout_interval;
   XtIntervalId timeout_id;
} Panel_Def;

static Hor_Assoc_List  panel_list       = NULL;
static Hor_Assoc_Label next_panel_label = HOR_ASSOC_START;

static void set_defaults ( Widget w, XtPointer client_data,
			             XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;
   char       string[20];

   sprintf ( string, "%1.3f", def->default_time_interval );
   hor_set_widget_value ( def->time_interval_text,  string );
   hor_set_widget_value ( def->display_number_text, DISPLAY_NUMBER_DEFAULT );
}

#define get_time_interval( w, fp ) \
   hor_read_float_param ( hor_get_widget_value(w), "time interval", \
			  hor_float_pos, HOR_NON_FATAL, fp )

#define get_display_number( w, ip ) \
   hor_read_int_param ( hor_get_widget_value(w), "display number", \
		        hor_int_abs_pos, HOR_NON_FATAL, ip )

static void hide_panel ( Widget widget,
			 XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtPopdown(XtParent(XtParent(widget)));
   if ( def->timeout_flag )
   {
      XtRemoveTimeOut ( def->timeout_id );
      def->timeout_flag = HOR_FALSE;
   }

   hor_dfree_nodes ( def->result_list );
   if ( def->free_func != NULL ) def->free_func ( def->list );

   def->in_use = HOR_FALSE;
}

static void prev ( Panel_Def *def )
{
   char string[20];

   if ( def->current_pos == 0 )
   {
      hor_message ( "wrapping around to the end of the sequence" );
      if ( def->wrap_backwards_func != NULL )
	 def->wrap_backwards_func(def->current_node->prev->contents);

      def->current_pos = def->list_length - 1;
   }
   else
      def->current_pos--;

   def->current_node = def->current_node->prev;
   def->display_func ( def->current_node->contents );
   sprintf ( string, "%d", def->current_pos + 1 );
   hor_set_widget_value ( def->display_number_text, string );
}

static void prev_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;

   prev ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 200,
                                       prev_timeout_proc, (XtPointer) def );
}

static void prev_proc ( Widget w, XtPointer client_data, XtPointer call_data );

static void prev_stop ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, prev_stop, client_data );
   XtAddCallback    ( w, XtNcallback, prev_proc, client_data );
   if ( def->timeout_flag ) return;

   XtRemoveTimeOut ( def->timeout_id );
}

static void prev_proc ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, prev_proc, client_data );
   XtAddCallback    ( w, XtNcallback, prev_stop, client_data );
   if ( def->timeout_flag )
   {
      hor_warning ( "not while in movie/reverse mode" );
      return;
   }

   prev ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 1000,
				       prev_timeout_proc, (XtPointer) def );
}

static void next ( Panel_Def *def )
{
   char string[20];

   if ( def->current_pos == def->list_length - 1 )
   {
      hor_message ( "wrapping around to the beginning of the sequence" );
      if ( def->wrap_forwards_func != NULL )
	 def->wrap_forwards_func (def->current_node->next->contents);

      def->current_pos = 0;
   }
   else
      def->current_pos++;

   def->current_node = def->current_node->next;
   def->display_func ( def->current_node->contents );
   sprintf ( string, "%d", def->current_pos + 1 );
   hor_set_widget_value ( def->display_number_text, string );
}

static void next_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;

   next ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 200,
                                       next_timeout_proc, (XtPointer) def );
}

static void next_proc ( Widget w, XtPointer client_data, XtPointer call_data );

static void next_stop ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, next_stop, client_data );
   XtAddCallback    ( w, XtNcallback, next_proc, client_data );
   if ( def->timeout_flag ) return;

   XtRemoveTimeOut ( def->timeout_id );
}

static void next_proc ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, next_proc, client_data );
   XtAddCallback    ( w, XtNcallback, next_stop, client_data );
   if ( def->timeout_flag )
   {
      hor_warning ( "not while in movie/reverse mode" );
      return;
   }

   next ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 1000,
				       next_timeout_proc, (XtPointer) def );
}

#define WRAP_TIME 1000 /* milliseconds */

static void reverse_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;
   
   prev ( def );
   if ( def->current_pos == 0 )
      def->timeout_id = XtAppAddTimeOut(def->app_con,
					def->timeout_interval + WRAP_TIME,
					reverse_timeout_proc, (XtPointer) def);
   else
      def->timeout_id = XtAppAddTimeOut(def->app_con, def->timeout_interval,
					reverse_timeout_proc, (XtPointer) def);
}

static void reverse_proc ( Widget w, XtPointer client_data,
			   XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;
   float      time_interval;

   if ( def->timeout_flag )
   {
      XtRemoveTimeOut ( def->timeout_id );
      def->timeout_flag = HOR_FALSE;
   }

   if ( !get_time_interval ( def->time_interval_text, &time_interval ) )
      return;

   def->timeout_interval = (u_long) (time_interval*1000.0F) + 10;
   if ( def->current_pos == 0 )
      def->timeout_id = XtAppAddTimeOut(def->app_con,
					def->timeout_interval + WRAP_TIME,
					reverse_timeout_proc, (XtPointer) def);
   else
      def->timeout_id = XtAppAddTimeOut(def->app_con, def->timeout_interval,
					reverse_timeout_proc, (XtPointer) def);
   def->timeout_flag = HOR_TRUE;
}

static void movie_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;
   
   next ( def );
   if ( def->current_pos == def->list_length - 1 )
      def->timeout_id = XtAppAddTimeOut ( def->app_con,
					  def->timeout_interval + WRAP_TIME,
					  movie_timeout_proc, (XtPointer) def);
   else
      def->timeout_id = XtAppAddTimeOut ( def->app_con, def->timeout_interval,
					  movie_timeout_proc, (XtPointer) def);
}

static void movie_proc ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;
   float      time_interval;

   if ( def->timeout_flag )
   {
      XtRemoveTimeOut ( def->timeout_id );
      def->timeout_flag = HOR_FALSE;
   }

   if ( !get_time_interval ( def->time_interval_text, &time_interval ) )
      return;

   def->timeout_interval = (u_long) (time_interval*1000.0F) + 10;
   if ( def->current_pos == def->list_length - 1 )
      def->timeout_id = XtAppAddTimeOut ( def->app_con,
					  def->timeout_interval + WRAP_TIME,
					  movie_timeout_proc, (XtPointer) def);
   else
      def->timeout_id = XtAppAddTimeOut ( def->app_con, def->timeout_interval,
					  movie_timeout_proc, (XtPointer) def);

   def->timeout_flag = HOR_TRUE;
}

static void stop_proc ( Widget w, XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   if ( !def->timeout_flag )
   {
      hor_warning ( "not in movie/reverse mode" );
      return;
   }

   XtRemoveTimeOut ( def->timeout_id );
   def->timeout_flag = HOR_FALSE;
}

static void decrement ( Panel_Def *def )
{
   int  display_number;
   char string[20];

   if ( !get_display_number ( def->display_number_text, &display_number ) )
      return;

   if ( display_number == 1 )
   {
      hor_message ( "wrapping around to the end of the sequence" );
      display_number = def->list_length;
   }
   else
      display_number--;

   sprintf ( string, "%1d", display_number );
   hor_set_widget_value ( def->display_number_text, string );
}

static void decrement_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;

   decrement ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 200,
                                       decrement_timeout_proc,
				       (XtPointer) def );
}

static void decrement_proc ( Widget w,
			     XtPointer client_data, XtPointer call_data );

static void decrement_stop ( Widget w,
			     XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, decrement_stop, client_data );
   XtAddCallback    ( w, XtNcallback, decrement_proc, client_data );
   if ( def->timeout_flag ) return;

   XtRemoveTimeOut ( def->timeout_id );
}

static void decrement_proc ( Widget w,
			     XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, decrement_proc, client_data );
   XtAddCallback    ( w, XtNcallback, decrement_stop, client_data );

   if ( def->timeout_flag )
   {
      hor_warning ( "not while in movie/reverse mode" );
      return;
   }

   decrement ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 1000,
				       decrement_timeout_proc,
				       (XtPointer) def );
}

static void increment ( Panel_Def *def )
{
   int  display_number;
   char string[20];

   if ( !get_display_number ( def->display_number_text, &display_number ) )
      return;

   if ( display_number >= def->list_length )
   {
      hor_message ( "wrapping around to the beginning of the sequence" );
      display_number = 1;
   }
   else
      display_number++;

   sprintf ( string, "%1d", display_number );
   hor_set_widget_value ( def->display_number_text, string );
}

static void increment_timeout_proc ( XtPointer client_data, XtIntervalId *id )
{
   Panel_Def *def = (Panel_Def *) client_data;

   increment ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 200,
                                       increment_timeout_proc,
				       (XtPointer) def );
}

static void increment_proc ( Widget w,
			     XtPointer client_data, XtPointer call_data );

static void increment_stop ( Widget w,
			     XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, increment_stop, client_data );
   XtAddCallback    ( w, XtNcallback, increment_proc, client_data );
   if ( def->timeout_flag ) return;

   XtRemoveTimeOut ( def->timeout_id );
}

static void increment_proc ( Widget w,
			     XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   XtRemoveCallback ( w, XtNcallback, increment_proc, client_data );
   XtAddCallback    ( w, XtNcallback, increment_stop, client_data );

   if ( def->timeout_flag )
   {
      hor_warning ( "not while in movie/reverse mode" );
      return;
   }

   increment ( def );
   def->timeout_id = XtAppAddTimeOut ( def->app_con, 1000,
				       increment_timeout_proc,
				       (XtPointer) def );
}

static void display_proc ( Widget w,
			   XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;
   int        display_number;

   if ( !get_display_number ( def->display_number_text, &display_number ) )
      return;

   if ( display_number > def->list_length )
   {
      char string[20];

      hor_warning ( "resetting to end of list" );
      display_number = def->list_length;
      sprintf ( string, "%1d", display_number );
      hor_set_widget_value ( def->display_number_text, string );
   }

   if ( def->current_pos < display_number-1 )
      while ( def->current_pos < display_number-1 )
      {
	 def->current_node = def->current_node->next;
	 def->current_pos++;
      }
   else
      while ( def->current_pos > display_number-1 )
      {
	 def->current_node = def->current_node->prev;
	 def->current_pos--;
      }

   def->display_func ( def->current_node->contents );
}

static void select_proc ( Widget w,
			  XtPointer client_data, XtPointer call_data )
{
   Panel_Def *def = (Panel_Def *) client_data;

   if ( def->select_func != NULL )
      def->select_func ( def->current_node->contents );
}

static char defaultTranslations[] = "#override\n<LeaveWindow>: notify() reset()\n<Btn1Down>: set() notify()\n<Btn1Up>: notify() unset()";


/*******************
*   Hor_Assoc_Label @hor_create_memory_panel ( XtAppContext app_con,
*                                             Widget parent )
*
*   Creates a popup movie control panel for interactive display of a list
*   of arbitrary objects. The returned label for the panel is then used
*   in subsequent calls to hor_popup_memory_panel() in order to invoke the
*   panel for a specific list of objects.
********************/
Hor_Assoc_Label hor_create_memory_panel ( XtAppContext app_con, Widget parent )
{
   Panel_Def *def = hor_malloc_type ( Panel_Def );
   Widget     popup_panel;
   Widget     prev, next, reverse, movie, stop, select, reset;
   Widget     decrement, increment, display;
   XtTranslations mytranslations;

   def->timeout_flag = HOR_FALSE;
   def->app_con = app_con;
   def->parent = parent;
   def->popup_frame = XtVaCreatePopupShell ("Memory",
					    transientShellWidgetClass,
					    parent, NULL);
   popup_panel = XtVaCreateManagedWidget("Params Popup", formWidgetClass,
					 def->popup_frame, NULL);

   prev = XtVaCreateManagedWidget ( "Prev", commandWidgetClass,
				    popup_panel,
				    NULL);
   XtAddCallback ( prev, XtNcallback, prev_proc, def );

   next = XtVaCreateManagedWidget ( "Next", commandWidgetClass,
				    popup_panel,
				    XtNfromHoriz, prev,
				    NULL);
   XtAddCallback ( next, XtNcallback, next_proc, def );

   reverse = XtVaCreateManagedWidget ( "Reverse", commandWidgetClass,
				       popup_panel,
				       XtNfromHoriz, next,
				       NULL);
   XtAddCallback ( reverse, XtNcallback, reverse_proc, def );

   movie = XtVaCreateManagedWidget ( "Movie", commandWidgetClass,
				     popup_panel,
				     XtNfromHoriz, reverse,
				     NULL);
   XtAddCallback ( movie, XtNcallback, movie_proc, def );

   stop = XtVaCreateManagedWidget ( "Stop", commandWidgetClass,
				    popup_panel,
				    XtNfromHoriz, movie,
				    NULL);
   XtAddCallback ( stop, XtNcallback, stop_proc, def );

   def->time_interval_text  = hor_create_text ( popup_panel, prev,
					        "Time Interval: ");
   def->display_number_text = hor_create_text ( popup_panel,
					        def->time_interval_text,
					        "Display Number:");

   decrement = XtVaCreateManagedWidget ( "Decrement", commandWidgetClass,
					 popup_panel,
					 XtNfromVert, def->display_number_text,
					 NULL );
   XtAddCallback ( decrement, XtNcallback, decrement_proc, def );

   increment = XtVaCreateManagedWidget ( "Increment", commandWidgetClass,
					 popup_panel,
					 XtNfromVert, def->display_number_text,
					 XtNfromHoriz, decrement,
					 NULL );
   XtAddCallback ( increment, XtNcallback, increment_proc, def );

   /* amend translation table to call callback procedure on button press
      and leave window events */
   mytranslations = XtParseTranslationTable ( defaultTranslations );
   XtOverrideTranslations ( prev, mytranslations );
   XtOverrideTranslations ( next, mytranslations );
   XtOverrideTranslations ( decrement, mytranslations );
   XtOverrideTranslations ( increment, mytranslations );

   display = XtVaCreateManagedWidget ( "Display", commandWidgetClass,
				       popup_panel,
				       XtNfromVert, def->display_number_text,
				       XtNfromHoriz, increment,
				       NULL );
   XtAddCallback ( display, XtNcallback, display_proc, def );

   select = XtVaCreateManagedWidget ( "Select", commandWidgetClass,
				      popup_panel,
				      XtNfromVert, decrement,
				      NULL);
   XtAddCallback ( select, XtNcallback, select_proc, def );

   reset = XtVaCreateManagedWidget ( "Reset", commandWidgetClass,
				     popup_panel,
				     XtNfromVert, decrement,
				     XtNfromHoriz, select,
				     NULL);
   XtAddCallback ( reset, XtNcallback, set_defaults, def );

   def->done_button = XtVaCreateManagedWidget("Done", commandWidgetClass,
					      popup_panel,
					      XtNfromVert,  decrement,
					      XtNfromHoriz, reset,
					      NULL);
   XtAddCallback ( def->done_button, XtNcallback, hide_panel, def );

   def->in_use = HOR_FALSE;
   panel_list = hor_assoc_insert ( panel_list, next_panel_label, (void *) def);
   return next_panel_label++;
}

/*******************
*   Hor_Bool @hor_popup_memory_panel ( Widget button,
*                                     Hor_Assoc_Label panel_label,
*                                     Hor_List list,
*                                     void (*display_func)(void *),
*                                     void (*select_func)(void *),
*                                     void (*wrap_forwards_func)(void *),
*                                     void (*wrap_backwards_func)(void *),
*                                     void (*free_func)(Hor_List),
*                                     double time_interval )
*
*   Causes a movie popup panel to appear on the screen. The button argument is
*   used to place the graph panel. It will normally be the button that invoked
*   the, graph, but can be NULL, in which case the parent widget passed into
*   hor_create_memory_panel() is used to place the panel.
*   The panel_label argument is the value returned previously by
*   hor_create_memory_panel(). list is a user-defined list of pointers.
*   display_func() is a function that may be called to display an element of
*   the list, select_func() is called upon selection of the currently
*   displayed element, wrap_forwards_func() is called when the movie wraps
*   back around to the start, wrap_backwards_func() is called when it wraps
*   around in reverse, and free_func() is called upon the "Done" button of the
*   panel being pressed. The list is automatically passed to free_func() and
*   thus the list may be freed under user control, or indeed any other action.
*   select_func() and free_func() can both be NULL to have no effect.
*   The default time interval for movie mode is passed in as the last argument.
********************/
Hor_Bool hor_popup_memory_panel ( Widget button, Hor_Assoc_Label panel_label,
				  Hor_List list,
				  void (*display_func)(void *),
				  void (*select_func)(void *),
				  void (*wrap_forwards_func)(void *),
				  void (*wrap_backwards_func)(void *),
				  void (*free_func)(Hor_List),
				  double time_interval )
{
   Panel_Def *def;
   Position   x, y;

   def = (Panel_Def *) hor_assoc_find ( panel_list, panel_label );
   if ( def == NULL )
      hor_error ("panel with label %d does not exist (hor_popup_memory_panel)",
		 HOR_FATAL, panel_label);

   if ( def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_ALREADY_IN_USE;
      return HOR_FALSE;
   }

   /* construct doubly-linked list of results */
   def->list         = list;
   def->result_list  = hor_dmake_circular_from_list ( list );
   def->list_length  = hor_list_size ( list );
   def->current_pos  = 0;
   def->current_node = def->result_list;

   /* set functions to display a result and free the original list */
   def->display_func        = display_func;
   def->select_func         = select_func;
   def->wrap_forwards_func  = wrap_forwards_func;
   def->wrap_backwards_func = wrap_backwards_func;
   def->free_func           = free_func;

   /* display first result */
   display_func ( def->current_node->contents );
   hor_message ("first result displayed in sequence of %d", def->list_length);

   def->in_use = HOR_TRUE;

   if ( time_interval < 0.0 )
   {
      hor_warning ( "setting timeout interval to zero" );
      time_interval = 0.0;
   }

   def->default_time_interval = time_interval;
   set_defaults ( (Widget) NULL, (XtPointer) def, (XtPointer) NULL );

   if ( button == NULL ) button = def->parent;
   XtTranslateCoords( button, (Position) 30, (Position) 0, &x, &y );
   XtVaSetValues(def->popup_frame, 
		 XtNx, x,
		 XtNy, y,
		 NULL);
   XtPopup(def->popup_frame, XtGrabNone);
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_popdown_memory_panel ( Hor_Assoc_Label panel_label )
*
*   Causes a memory panel to disppear, losing all the data displayed in it.
********************/
Hor_Bool hor_popdown_memory_panel ( Hor_Assoc_Label panel_label )
{
   Panel_Def *def;

   def = (Panel_Def *) hor_assoc_find ( panel_list, panel_label );
   if ( def == NULL )
      hor_error ( "panel with label %d does not exist (hor_popdown_memory_panel)", HOR_FATAL, panel_label );

   if ( !def->in_use ) return HOR_FALSE;

   hide_panel ( def->done_button, (XtPointer) def, NULL );
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_memory_panel_in_use ( Hor_Assoc_Label panel_label )
*
*   Returns HOR_FALSE if a registered (by hor_create_memory_panel()) memory
*   panel is not in use, HOR_TRUE if it is, i.e. if hor_popup_memory_panel()
*   has been called for it and the "Done" button not yet pressed.
********************/
Hor_Bool hor_memory_panel_in_use ( Hor_Assoc_Label panel_label )
{
   Panel_Def *def;

   def = (Panel_Def *) hor_assoc_find ( panel_list, panel_label );
   if ( def == NULL )
      hor_error("panel with label %d does not exist (hor_memory_panel_in_use)",
		HOR_FATAL, panel_label );

   return ( def->in_use );
}

/*******************
*   Hor_Bool @hor_redisplay_memory_panel ( Hor_Assoc_Label panel_label )
*
*   Causes a movie popup panel to appear on the screen.
*   It should only be called during use of the panel, i.e. between the calls
*   to hor_popup_memory_panel() and pressing the "Done" button.
*   It pops up the panel to the front.
********************/
Hor_Bool hor_redisplay_memory_panel ( Hor_Assoc_Label panel_label )
{
   Panel_Def *def;

   def = (Panel_Def *) hor_assoc_find ( panel_list, panel_label );
   if ( def == NULL )
      hor_error("panel with label %d does not exist (hor_redisplay_memory_panel)", HOR_FATAL, panel_label );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   XtPopup(def->popup_frame, XtGrabNone);
   return HOR_TRUE;
}
