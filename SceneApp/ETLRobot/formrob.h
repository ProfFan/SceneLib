#ifndef FD_formrob_h_
#define FD_formrob_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/

extern void grab_images(FL_OBJECT *, long);
extern void set_pan(FL_OBJECT *, long);
extern void zero_pan(FL_OBJECT *, long);
extern void set_elevation(FL_OBJECT *, long);
extern void zero_elevation(FL_OBJECT *, long);
extern void init_man_selected_point_feature(FL_OBJECT *, long);
extern void init_auto_selected_point_feature(FL_OBJECT *, long);
extern void write_images(FL_OBJECT *, long);
extern void read_images(FL_OBJECT *, long);
extern void write_patch(FL_OBJECT *, long);
extern void show_head_odometry(FL_OBJECT *, long);
extern void show_robot_odometry(FL_OBJECT *, long);


/**** Forms and Objects ****/

typedef struct {
	FL_FORM *formrob;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *button_grab_images;
	FL_OBJECT *slider_set_pan;
	FL_OBJECT *button_zero_pan;
	FL_OBJECT *slider_set_elevation;
	FL_OBJECT *button_zero_elevation;
	FL_OBJECT *button_init_man_selected_point_feature;
	FL_OBJECT *button_init_auto_selected_point_feature;
	FL_OBJECT *canvasL;
	FL_OBJECT *canvasR;
	FL_OBJECT *button_write_images;
	FL_OBJECT *button_read_images;
	FL_OBJECT *button_write_patch;
	FL_OBJECT *button_show_head_odometry;
	FL_OBJECT *button_show_robot_odometry;
} FD_formrob;

extern FD_formrob * create_form_formrob(void);

#endif /* FD_formrob_h_ */
