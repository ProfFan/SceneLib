/** Header file generated with fdesign on Mon Apr 16 23:34:54 2001.**/

#ifndef FD_formsim_h_
#define FD_formsim_h_

/** Callbacks, globals and object handlers **/
extern void print_true_state(FL_OBJECT *, long);


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *formsim;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *button_print_true_state;
} FD_formsim;

extern FD_formsim * create_form_formsim(void);

#endif /* FD_formsim_h_ */
