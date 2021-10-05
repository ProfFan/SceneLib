#ifndef FD_formoned_h_
#define FD_formoned_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/

extern void set_v(FL_OBJECT *, long);
extern void zero_v(FL_OBJECT *, long);
extern void set_steps(FL_OBJECT *, long);
extern void one_steps(FL_OBJECT *, long);
extern void delete_feature(FL_OBJECT *, long);
extern void go_vehicle(FL_OBJECT *, long);
extern void print_state(FL_OBJECT *, long);
extern void quit(FL_OBJECT *, long);
extern void output_state(FL_OBJECT *, long);
extern void print_true_state(FL_OBJECT *, long);
extern void one_second(FL_OBJECT *, long);
extern void set_delta_t(FL_OBJECT *, long);
extern void navigate_to_next_waypoint(FL_OBJECT *, long);
extern void show_inspect_form(FL_OBJECT *, long);
extern void show_filter_form(FL_OBJECT *, long);
extern void zero_axes(FL_OBJECT *, long);


/**** Forms and Objects ****/

typedef struct {
	FL_FORM *formoned;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *slider_set_v;
	FL_OBJECT *button_zero_v;
	FL_OBJECT *slider_set_steps;
	FL_OBJECT *button_one_steps;
	FL_OBJECT *button_delete_feature;
	FL_OBJECT *button_go_vehicle;
	FL_OBJECT *button_print_state;
	FL_OBJECT *button_quit;
	FL_OBJECT *button_output_state;
	FL_OBJECT *button_print_true_state;
	FL_OBJECT *button_one_second;
	FL_OBJECT *slider_set_delta_t;
	FL_OBJECT *button_navigate_to_next_waypoint;
	FL_OBJECT *button_show_inspect;
	FL_OBJECT *button_show_filter;
	FL_OBJECT *button_zero_axes;
} FD_formoned;

extern FD_formoned * create_form_formoned(void);

#endif /* FD_formoned_h_ */
