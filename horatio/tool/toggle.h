/*
 * toggle group module:
 * to create a button with an associated popup containing a set of 
 * toggles and a done button
 *
 * Ian Reid, 9.9.93
 *
 * Horatio changes: Philip McLauchlan 30.9.93
 */

/* included from tool/toggle.h */

typedef int (*Hor_TG_Callback) (int bit_field, int toggle_mask, void *data);

#ifdef _XtIntrinsic_h

Widget hor_create_togglegroup_widget(String name,             /* widget name */
				     Widget parent,           /* parent widget */
				     String *choices,         /* list of strings of toggles */
				     int no_choices,          /* number in list */
				     Position x,              /* default popup position rel've */
				     Position y,              /*  to button */
				     Hor_TG_Callback on_callback, /* callback procedures */
				     Hor_TG_Callback off_callback, /* for toggle switching */
				     void *data, /* data to be passed to callbacks */
				     ...);      /* NULL-terminated list of resource,value pairs */
int  hor_get_togglegroup ( Widget button );
void hor_set_togglegroup ( Widget button, int bit_field );

#endif /* _XtIntrinsic_h */
