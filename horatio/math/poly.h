/* Copyright 1993 Philip Torr (phst@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/poly.h */

int hor_solve_quadratic ( double a, double b, double c,
			  double *x1, double *x2 );
int hor_solve_cubic ( double a, double b, double c, double d,
		      double *x1, double *x2, double *x3 );
