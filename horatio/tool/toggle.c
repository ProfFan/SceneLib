/*
 * togglegroup.c
 *
 * create a button with an associated popup containing a set of 
 * toggles and a done button
 *
 * Ian Reid, 9.9.93
 *
 * Horatio changes: Philip Mclauchlan 28.9.93
 */

#include <stdlib.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Toggle.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Toggle.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/tool.h"

typedef struct  {
      int toggle_mask;
      Hor_TG_Callback on_callback, off_callback;
      void *data;
} Hor_Toggle_Data;

typedef struct
{
   Widget button;
   Widget panel;
} TG_Assoc;

static Hor_List tg_assoc = NULL;

static int get_togglegroup ( Widget panel )
{
    int i, num_choices, result = 0;
    WidgetList choices;
    Boolean toggle_state;

    XtVaGetValues(panel,
		  XtNnumChildren, &num_choices,
		  XtNchildren, &choices,
		  NULL);
    choices++;
    num_choices -= 2;

    for (i=0; i<num_choices; i++) {
       XtVaGetValues ( choices[i], XtNstate, &toggle_state, NULL );
       if ( toggle_state == True )
	  result |= (1<<i);
    }

    return result;
}

void set_togglegroup ( Widget panel, int bit_field )
{
    int i, num_choices;
    WidgetList choices;

    XtVaGetValues(panel,
		  XtNnumChildren, &num_choices,
		  XtNchildren, &choices,
		  NULL);
    choices++;
    num_choices -= 2;

    for (i=0; i<num_choices; i++)
       if ( bit_field & (1<<i) )
	  XtVaSetValues ( choices[i], XtNstate, True, NULL );
       else
	  XtVaSetValues ( choices[i], XtNstate, False, NULL );
}

static void tg_callback ( Widget toggle, XtPointer client_data,
			                 XtPointer call_data )
{
   Hor_Toggle_Data *cbd = (Hor_Toggle_Data *) client_data;
   int bit_field = get_togglegroup ( XtParent(toggle) );

   if ( (int)call_data == 1 ) /* toggle just set */
   {
      if ( cbd->on_callback != NULL )
	 bit_field = cbd->on_callback ( bit_field, cbd->toggle_mask,
				        cbd->data );
   }
   else /* toggle just unset */
      if ( cbd->off_callback != NULL )
	 bit_field = cbd->off_callback ( bit_field, cbd->toggle_mask,
					 cbd->data );

   set_togglegroup ( XtParent(toggle), bit_field );
}

/*******************
*   Widget @hor_create_togglegroup_widget (
*               String   name,       (widget name)
*               Widget   parent,     (parent widget)
*               String  *choices,    (list of strings of toggles)
*               int      no_choices, (number in list)
*               Position x,          (default popup position relative
*               Position y,           to button)
*               Hor_TG_Callback on_callback, (callback procedures called when
*               Hor_TG_Callback off_callback  a toggle is switch on/off
*                                             respectively)
*               void           *data         (data pointer passed to the
*                                             callback functions)
*               ...) (NULL-terminated list of resource, value pairs)
*
*   Creates and returns a toggle-group widget, a pop-up panel containing a list
*   of toggles, with the given strings as labels. Rules for switching toggles
*   on or off in arbitrary combinations may appear in the callback functions,
*   which are called when a user-induced toggle state change occurs, and are
*   of the following type:
*
*   int callback ( int bit_field, int toggle, void *data )
*
*   The bit_field argument is the state of the toggle-group after the latest
*   state change, one bit being set for each toggle that is on.
*   The toggle argument identifies the toggle (0,1,2...). The toggle group
*   is set to the returned bit-field. If NULL is passed as either callback
*   function then no extra change of state occurs (i.e. the toggles behave
*   like true toggles).
*
*   The returned value is the command button widget which can be used in
*   the functions hor_get_togglegroup() and hor_set_togglegroup().
********************/
Widget hor_create_togglegroup_widget(String name, 
				     Widget parent, 
				     String *choices,
				     int no_choices,
				     Position x, Position y,
				     Hor_TG_Callback on_callback,
				     Hor_TG_Callback off_callback,
				     void           *data,
				     ...)
{
    Widget button;
    Widget popup_frame, popup_panel, last = NULL;
    Hor_Toggle_Data *cbd;
    Hor_Popup_Data *pcbd;
    int i, num_args=0;
    Arg *args;
    va_list ap;

    /* count number of variable arguments */
    va_start(ap,data);
    for(;;)
    {
       if ( va_arg(ap,String) == NULL ) break;
       (void)va_arg(ap,XtArgVal);
       num_args++;
    }
    va_end(ap);

    /* copy arguments into array args */
    args = hor_malloc_ntype ( Arg, num_args+1 ); /* +1 to avoid NULL */
    va_start(ap,data);
    for (i=0; i<num_args; i++) 
       XtSetArg(args[i],va_arg(ap,String),va_arg(ap,XtArgVal));
    va_end(ap);

    button = XtCreateManagedWidget(name, commandWidgetClass, 
				   parent, args, num_args);
    hor_free ( (void *) args );

    popup_frame = XtVaCreatePopupShell (name,
					transientShellWidgetClass,
					button, 0);
    popup_panel = XtVaCreateManagedWidget(name, 
					  formWidgetClass,
					  popup_frame, 0);

    last = XtVaCreateManagedWidget(name, labelWidgetClass, popup_panel, NULL );

    cbd = hor_malloc_ntype (Hor_Toggle_Data, no_choices);
    for (i=0; i<no_choices; i++) {
	last = XtVaCreateManagedWidget(choices[i], toggleWidgetClass, 
				       popup_panel, 
				       XtNfromVert, last,
				       XtNshapeStyle, XmuShapeOval,
				       NULL );
	cbd[i].toggle_mask  = (1<<i);
	cbd[i].on_callback  = on_callback;
	cbd[i].off_callback = off_callback;
	cbd[i].data         = data;
	XtAddCallback(last, XtNcallback, tg_callback, (XtPointer)(&(cbd[i])));
    }

    pcbd = hor_malloc_type(Hor_Popup_Data);
    pcbd->popup_frame = popup_frame;
    pcbd->x = x;
    pcbd->y = y;
    XtAddCallback (button, XtNcallback, hor_show_popup, (XtPointer)pcbd);

    {
       TG_Assoc *new = hor_malloc_type(TG_Assoc);

       new->button = button;
       new->panel = popup_panel;
       tg_assoc = hor_insert ( tg_assoc, (void *) new );
    }

    hor_create_done_button ( popup_panel, last );
    return button;
}


/*******************
*   int @hor_get_togglegroup ( Widget button )
*
*   Returns the state of the given toggle group as a bit-field.
********************/
int hor_get_togglegroup ( Widget button )
{
   Hor_List list;
   Widget   panel = NULL;

   for ( list = tg_assoc; list != NULL; list = list->next )
   {
      TG_Assoc *assoc = (TG_Assoc *) list->contents;
      if ( assoc->button == button ) panel = assoc->panel;
   }

   if ( panel == NULL )
      hor_error ( "illegal button (hor_get_togglegroup)", HOR_FATAL );

    return get_togglegroup ( panel );
}


/*******************
*   void @hor_set_togglegroup ( Widget button, int bit_field )
*
*   Sets the state of the given toggle group to the given bit-field.
********************/
void hor_set_togglegroup ( Widget button, int bit_field )
{
   Hor_List list;
   Widget   panel = NULL;

   for ( list = tg_assoc; list != NULL; list = list->next )
   {
      TG_Assoc *assoc = (TG_Assoc *) list->contents;
      if ( assoc->button == button ) panel = assoc->panel;
   }

   if ( panel == NULL )
      hor_error ( "illegal button (hor_set_togglegroup)", HOR_FATAL );

   set_togglegroup ( panel, bit_field );
}
