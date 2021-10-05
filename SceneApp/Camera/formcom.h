/** Header file generated with fdesign on Tue Apr 17 17:45:24 2001.**/

#ifndef FD_formcom_h_
#define FD_formcom_h_

/** Callbacks, globals and object handlers **/
extern void delete_feature(FL_OBJECT *, long);
extern void go_step_by_step(FL_OBJECT *, long);
extern void print_robot_state(FL_OBJECT *, long);
extern void quit(FL_OBJECT *, long);
extern void output_state(FL_OBJECT *, long);
extern void set_steps(FL_OBJECT *, long);
extern void one_steps(FL_OBJECT *, long);
extern void auto_select_feature(FL_OBJECT *, long);
extern void print_feature_state(FL_OBJECT *, long);
extern void print_whole_state(FL_OBJECT *, long);
extern void show_filter_form(FL_OBJECT *, long);
extern void show_inspect_form(FL_OBJECT *, long);
extern void zero_axes(FL_OBJECT *, long);
extern void output_postscript(FL_OBJECT *, long);
extern void go_auto_select(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *formcom;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *button_delete_feature;
	FL_OBJECT *button_go_step_by_step;
	FL_OBJECT *button_print_robot_state;
	FL_OBJECT *button_quit;
	FL_OBJECT *button_output_state;
	FL_OBJECT *slider_set_steps;
	FL_OBJECT *button_auto_select_feature;
	FL_OBJECT *button_print_feature_state;
	FL_OBJECT *button_print_whole_state;
	FL_OBJECT *button_show_filter;
	FL_OBJECT *button_show_inspect;
	FL_OBJECT *button_zero_axes;
	FL_OBJECT *button_output_postscript;
	FL_OBJECT *button_go_auto_select;
} FD_formcom;

extern FD_formcom * create_form_formcom(void);

#endif /* FD_formcom_h_ */
