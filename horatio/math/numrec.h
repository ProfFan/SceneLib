/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/numrec.h */

void hor_ludcmp ( double **a, int n, int *indx, double *d, double *vv );
void hor_lubksb ( double **a, int n, int *indx, double **b );
void hor_ludcmpf ( float **a, int n, int *indx, float *d, float *vv );
void hor_lubksbf ( float **a, int n, int *indx, float *b );
double hor_gammln ( double xx );
void hor_gser ( double *gamser, double a, double x, double *gln );
void hor_gcf ( double *gammcf, double a, double x, double *gln );
double hor_gammp ( double a, double x );
double hor_gammq ( double a, double x );
double hor_erfc ( double x );
void hor_tred2 ( double **a, int n, double d[], double e[] );
void hor_tqli ( double d[], double e[], int n, double **z );
void hor_eigsrt ( double *d, double **v, int n );
Hor_Bool hor_svdcmp ( double **a, int m, int n, double *w, double **v );
Hor_Bool hor_gaussj ( double **a, int n, double **b, int m );
void hor_covsrt ( double **covar, int ma, int lista[], int mfit );
void hor_mrqcof ( double x[], double y[], double sig[], int ndata,
		  double a[], int ma, int lista[], int mfit,
		  double **alpha, double beta[], double *chisq,
		  void (*funcs)(double, double *, double *, double *, int) );
Hor_Bool hor_mrqmin ( double x[], double y[], double sig[], int ndata,
		      double a[], int ma, int lista[], int mfit,
		      double **covar, double **alpha, double *chisq,
		      void (*funcs)(double, double *, double *, double *, int),
		      double *alamda );
void hor_svbksb ( double **u, double *w, double **v,
		  int m, int n, double **b, double **x );
