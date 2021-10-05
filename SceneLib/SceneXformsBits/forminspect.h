/** Header file generated with fdesign on Thu Oct 12 15:17:06 2000.**/

#ifndef FD_forminspect_h_
#define FD_forminspect_h_

/** Callbacks, globals and object handlers **/
extern void change_cell_size(FL_OBJECT *, long);
extern void scroll_covariance(FL_OBJECT *, long);
extern void center_on_selected(FL_OBJECT *, long);
extern void scroll_diagonally(FL_OBJECT *, long);
extern void change_precision(FL_OBJECT *, long);
extern void redraw_form(FL_OBJECT *, long);
extern void hide_form(FL_OBJECT *, long);
extern void switch_norm_type(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *forminspect;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *text_selected_covariance;
	FL_OBJECT *frame_covariance;
	FL_OBJECT *counter_cell_size;
	FL_OBJECT *scrollbar_covariance_horizontal;
	FL_OBJECT *scrollbar_covariance_vertical;
	FL_OBJECT *button_center;
	FL_OBJECT *button_diagonal_scroll;
	FL_OBJECT *counter_precision;
	FL_OBJECT *button_redraw_form;
	FL_OBJECT *button_close;
	FL_OBJECT *button_norm_volume;
	FL_OBJECT *button_norm_frobenius;
	FL_OBJECT *button_norm_trace;
	FL_OBJECT *button_norm_hinf;
} FD_forminspect;

extern FD_forminspect * create_form_forminspect(void);

#endif /* FD_forminspect_h_ */
