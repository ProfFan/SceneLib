#ifndef FD_formanalyse_h_
#define FD_formanalyse_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/

extern void quit(FL_OBJECT *, long);
extern void set_step(FL_OBJECT *, long);
extern void input_viewing_direction(FL_OBJECT *, long);
extern void input_postscript_parameters(FL_OBJECT *, long);
extern void input_postscript_axis_limits(FL_OBJECT *, long);
extern void output_postscript(FL_OBJECT *, long);
extern void output_ground_truth_postscript(FL_OBJECT *, long);
extern void output_trajectory_picture(FL_OBJECT *, long);
extern void print_robot_state(FL_OBJECT *, long);
extern void print_whole_state(FL_OBJECT *, long);
extern void print_true_state(FL_OBJECT *, long);
extern void print_feature_state(FL_OBJECT *, long);


/**** Forms and Objects ****/

typedef struct {
	FL_FORM *formanalyse;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *button_quit;
	FL_OBJECT *counter_set_step;
	FL_OBJECT *button_input_viewing_direction;
	FL_OBJECT *button_input_postscript_parameters;
	FL_OBJECT *button_input_postscript_axis_limits;
	FL_OBJECT *button_output_postscript;
	FL_OBJECT *button_output_ground_truth_postscript;
	FL_OBJECT *button_output_trajectory_picture;
	FL_OBJECT *button_print_robot_state;
	FL_OBJECT *button_print_whole_state;
	FL_OBJECT *button_print_true_state;
	FL_OBJECT *button_print_feature_state;
} FD_formanalyse;

extern FD_formanalyse * create_form_formanalyse(void);

#endif /* FD_formanalyse_h_ */
