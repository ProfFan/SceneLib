/** Header file generated with fdesign on Wed May 23 01:56:42 2001.**/

#ifndef FD_formrob_h_
#define FD_formrob_h_

/** Callbacks, globals and object handlers **/
extern void read_next_image(FL_OBJECT *, long);
extern void write_patch(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *formrob;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *canvas;
	FL_OBJECT *button_read_next_image;
	FL_OBJECT *button_write_patch;
} FD_formrob;

extern FD_formrob * create_form_formrob(void);

#endif /* FD_formrob_h_ */
