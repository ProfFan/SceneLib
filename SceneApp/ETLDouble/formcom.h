#ifndef FD_formcom_h_
#define FD_formcom_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/

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
extern void set_steps(FL_OBJECT *, long);
extern void one_steps(FL_OBJECT *, long);
extern void auto_select_feature(FL_OBJECT *, long);
extern void print_feature_state(FL_OBJECT *, long);
extern void print_whole_state(FL_OBJECT *, long);
extern void navigate_to_next_waypoint(FL_OBJECT *, long);
extern void set_v2(FL_OBJECT *, long);
extern void set_S2(FL_OBJECT *, long);
extern void set_T2(FL_OBJECT *, long);
extern void zero_v2(FL_OBJECT *, long);
extern void zero_S2(FL_OBJECT *, long);
extern void zero_T2(FL_OBJECT *, long);
extern void observe_second_robot(FL_OBJECT *, long);
extern void waypoint_check_correct(FL_OBJECT *, long);
extern void output_state(FL_OBJECT *, long);
extern void lock_steering_turret(FL_OBJECT *, long);
extern void lock_steering_turret2(FL_OBJECT *, long);
extern void zero_axes(FL_OBJECT *, long);
extern void show_inspect_form(FL_OBJECT *, long);
extern void show_filter_form(FL_OBJECT *, long);


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
	FL_OBJECT *slider_set_steps;
	FL_OBJECT *button_auto_select_feature;
	FL_OBJECT *button_print_feature_state;
	FL_OBJECT *button_print_whole_state;
	FL_OBJECT *button_navigate_to_next_waypoint;
	FL_OBJECT *slider_set_v2;
	FL_OBJECT *slider_set_S2;
	FL_OBJECT *slider_set_T2;
	FL_OBJECT *button_zero_v2;
	FL_OBJECT *button_zero_S2;
	FL_OBJECT *button_zero_T2;
	FL_OBJECT *button_observe_second_robot;
	FL_OBJECT *button_waypoint_check_correct;
	FL_OBJECT *button_output_state;
	FL_OBJECT *button_lock_steering_turret;
	FL_OBJECT *button_lock_steering_turret2;
	FL_OBJECT *button_zero_axes;
	FL_OBJECT *button_show_inspect;
	FL_OBJECT *button_show_filter;
} FD_formcom;

extern FD_formcom * create_form_formcom(void);

#endif /* FD_formcom_h_ */
