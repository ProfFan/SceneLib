/** Header file generated with fdesign on Fri Feb  2 10:16:18 2001.**/

#ifndef FD_formkalman_h_
#define FD_formkalman_h_

/** Callbacks, globals and object handlers **/
extern void change_filter_type(FL_OBJECT *, long);
extern void set_filter_params(FL_OBJECT *, long);
extern void switch_to_saved_state(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *formkalman;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *choice_filter_type1;
	FL_OBJECT *choice_filter_type2;
	FL_OBJECT *counter_jghk_age_limit;
	FL_OBJECT *button_display_filter2_state;
	FL_OBJECT *button_display_filter1_state;
	FL_OBJECT *text_features_retained1;
	FL_OBJECT *text_filter_time1;
	FL_OBJECT *text_total_time1;
	FL_OBJECT *text_features_retained2;
	FL_OBJECT *text_filter_time2;
	FL_OBJECT *text_total_time2;
	FL_OBJECT *input_nebot_c1;
	FL_OBJECT *input_nebot_c2;
} FD_formkalman;

extern FD_formkalman * create_form_formkalman(void);

#endif /* FD_formkalman_h_ */
