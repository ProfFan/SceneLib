/** Header file generated with fdesign on Thu Mar  8 22:06:21 2001.**/

#ifndef FD_formcom_h_
#define FD_formcom_h_

/** Callbacks, globals and object handlers **/
extern void set_v(FL_OBJECT *, long);
extern void set_S(FL_OBJECT *, long);
extern void set_T(FL_OBJECT *, long);
extern void zero_v(FL_OBJECT *, long);
extern void zero_S(FL_OBJECT *, long);
extern void zero_T(FL_OBJECT *, long);
extern void set_delta_t(FL_OBJECT *, long);
extern void one_second(FL_OBJECT *, long);
extern void delete_feature(FL_OBJECT *, long);
extern void go_using_set_parameters(FL_OBJECT *, long);
extern void print_robot_state(FL_OBJECT *, long);
extern void quit(FL_OBJECT *, long);
extern void output_state(FL_OBJECT *, long);
extern void set_steps(FL_OBJECT *, long);
extern void one_steps(FL_OBJECT *, long);
extern void auto_select_feature(FL_OBJECT *, long);
extern void print_feature_state(FL_OBJECT *, long);
extern void print_whole_state(FL_OBJECT *, long);
extern void navigate_to_next_waypoint(FL_OBJECT *, long);
extern void lock_steering_turret(FL_OBJECT *, long);
extern void show_filter_form(FL_OBJECT *, long);
extern void show_inspect_form(FL_OBJECT *, long);
extern void zero_axes(FL_OBJECT *, long);
extern void measure_roll_pitch(FL_OBJECT *, long);
extern void go_vehicle_roll_pitch(FL_OBJECT *, long);
extern void output_postscript(FL_OBJECT *, long);
extern void navigate_to_next_waypoint_using_rollpitch(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *formcom;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *slider_set_v;
	FL_OBJECT *slider_set_S;
	FL_OBJECT *slider_set_T;
	FL_OBJECT *button_zero_v;
	FL_OBJECT *button_zero_S;
	FL_OBJECT *button_zero_T;
	FL_OBJECT *slider_set_delta_t;
	FL_OBJECT *button_one_second;
	FL_OBJECT *button_delete_feature;
	FL_OBJECT *button_go_using_set_parameters;
	FL_OBJECT *button_print_robot_state;
	FL_OBJECT *button_quit;
	FL_OBJECT *button_output_state;
	FL_OBJECT *slider_set_steps;
	FL_OBJECT *button_auto_select_feature;
	FL_OBJECT *button_print_feature_state;
	FL_OBJECT *button_print_whole_state;
	FL_OBJECT *button_navigate_to_next_waypoint;
	FL_OBJECT *button_lock_steering_turret;
	FL_OBJECT *button_show_filter;
	FL_OBJECT *button_show_inspect;
	FL_OBJECT *button_zero_axes;
	FL_OBJECT *button_measure_roll_pitch;
	FL_OBJECT *button_go_vehicle_roll_pitch;
	FL_OBJECT *button_output_postscript;
	FL_OBJECT *button_navigate_to_next_waypoint_using_rollpitch;
} FD_formcom;

extern FD_formcom * create_form_formcom(void);

#endif /* FD_formcom_h_ */
