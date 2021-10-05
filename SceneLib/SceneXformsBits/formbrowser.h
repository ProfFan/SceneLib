#ifndef FD_formbrowser_h_
#define FD_formbrowser_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/

extern void waypoint_selected(FL_OBJECT *, long);
extern void delete_waypoint(FL_OBJECT *, long);
extern void add_waypoint_before(FL_OBJECT *, long);
extern void add_waypoint_after(FL_OBJECT *, long);


/**** Forms and Objects ****/

typedef struct {
	FL_FORM *formbrowser;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *waypoint_browser;
	FL_OBJECT *button_delete_waypoint;
	FL_OBJECT *button_add_waypoint_before;
	FL_OBJECT *input_waypoint;
	FL_OBJECT *button_add_waypoint_after;
} FD_formbrowser;

extern FD_formbrowser * create_form_formbrowser(void);

#endif /* FD_formbrowser_h_ */
