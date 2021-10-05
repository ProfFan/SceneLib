/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/popup.h */

#include <stdarg.h>

#ifdef _XtIntrinsic_h

int hor_convert_X_args ( va_list *aptr, Arg **arg );

/*******************
*   typedef struct
*   {
*      Widget   popup_frame; (frame for popup window)
*      Position x, y;        (display position relative to parent)
*   } Hor_Popup_Data;
*
*   Structure containing popup information.
********************/
typedef struct
{
   Widget   popup_frame;
   Position x, y;
} Hor_Popup_Data;

void hor_show_popup ( Widget widget,
		      XtPointer client_data, XtPointer call_data );

/*******************
*   void  @hor_set_widget_value ( Widget *widget, const char *string )
*   char *@hor_get_widget_value ( Widget *widget )
*
*   hor_set_widget_value() sets the string value of the widget to the string.
*   hor_get_widget_value() returns the string value of the widget.
*   Both are implemented as macros.
********************/
#define hor_set_widget_value(wid,string) XtVaSetValues(wid, XtNvalue, string,0)
/*typedef char * (*XowFuncCast)(Widget); needed because of fussy g++ */
#define hor_get_widget_value(wid) XowPanelTextGetValueString(wid)

/*******************
*   Widget @hor_create_text ( Widget popup_panel, Widget last,
*                            const char *name )
*
*   Returns a panel-text widget with the given popup panel as parent.
*   New widget is placed below the given "last" widget and assigned the
*   given name.
********************/
#define hor_create_text( popup_panel, last, name ) \
          XtVaCreateManagedWidget ( name, panelTextWidgetClass, popup_panel, \
				    XtNfromVert, last, 0 )

void hor_hide_popup ( Widget widget,
		      XtPointer client_data, XtPointer call_data );
void hor_create_done_button ( Widget popup_panel, Widget last );
void hor_create_reset_done_buttons ( Widget popup_panel, Widget last,
				     void (*reset_func) (void) );

#endif /* _XtIntrinsic_h */

