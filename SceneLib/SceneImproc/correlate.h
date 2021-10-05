double and_correlate(register int x0, register int y0, register int x0lim,
		     register int y0lim, register int x1, register int y1,
		     Hor_Image *p0, Hor_Image *p1);

double and_correlate2(register int x0, register int y0, register int x0lim,
		      register int y0lim, register int x1, register int y1,
		      Hor_Image *p0, Hor_Image *p1);

double and_correlate2_warning(register int x0, register int y0, 
			      register int x0lim, register int y0lim, 
			      register int x1, register int y1,
			      Hor_Image *p0, Hor_Image *p1,
			      double *sd0ptr, double *sd1ptr);

double and_correlate3(register int x0, register int y0, register int x0lim,
		      register int y0lim, register int x1, register int y1,
		      Hor_Image *p0, Hor_Image *p1);

double and_correlate4(register int x0, register int y0, register int x0lim,
		      register int y0lim, register int x1, register int y1,
		      Hor_Image *p0, Hor_Image *p1);


