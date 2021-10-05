#ifndef FD_formrobmode_h_
#define FD_formrobmode_h_
/* Header file generated with fdesign. */

/** Callback routines and free object handlers **/



/**** Forms and Objects ****/

typedef struct {
	FL_FORM *formrobmode;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *button_zero_axes;
	FL_OBJECT *button_reduced_image;
	FL_OBJECT *button_done;
} FD_formrobmode;

extern FD_formrobmode * create_form_formrobmode(void);

#endif /* FD_formrobmode_h_ */
