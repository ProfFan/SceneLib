/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_MATH_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/math.h */

int hor_sgn ( double );
double hor_sqr( double );

#ifdef HOR_TRANSPUTER
float hor_sqrf ( float );
double cbrt ( double );
#else
#define hor_sqrtf sqrt
#define hor_sqrf hor_sqr
#define hor_fabsf fabs
#endif

/*******************
*   double @HOR_DISTANCE_2D ( double x1, double y1, double x2, double y2 )
*   double @HOR_DISTANCE_3D ( double x1, double y1, double z1,
*                            double x2, double y2, double z2 )
*
*   float @HOR_DISTANCE_2Df ( float x1, float y1, float x2, float y2 )
*   float @HOR_DISTANCE_3Df ( float x1, float y1, float z1,
*                            float x2, float y2, float z2 )
*
*   Euclidean distance measures, implemented as macros (each argument is
*   evaluated twice). HOR_DISTANCE_2Df() and HOR_DISTANCE_3Df() are only for
*   transputers.
********************/
#define HOR_DISTANCE_2D(x1,y1,x2,y2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
#define HOR_DISTANCE_3D(x1,y1,z1,x2,y2,z2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

#ifdef HOR_TRANSPUTER
#define HOR_DISTANCE_2Df(x1,y1,x2,y2) sqrtf((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
#define HOR_DISTANCE_3Df(x1,y1,z1,x2,y2,z2) sqrtf((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))
#endif

/*******************
*   @HOR_SIGN ( x )     @HOR_SIGNF ( x )     @HOR_SIGND ( x )
*
*   Returns the sign of the argument, 1 for x>0, -1 for x<0 and 0 for x=0.
*   Implemented as a macro. HOR_SIGNF and HOR_SIGND are for float and double
*   arguments, and return float and double results.
********************/
#define HOR_SIGN(x)  ((x) > 0    ? 1    : ((x) < 0    ? -1    : 0   ))
#define HOR_SIGNF(x) ((x) > 0.0F ? 1.0F : ((x) < 0.0F ? -1.0F : 0.0F))
#define HOR_SIGND(x) ((x) > 0.0  ? 1.0  : ((x) < 0.0  ? -1.0  : 0.0 ))

#ifdef HOR_TRANSPUTER
#define M_PI  3.14159265358979323846
#define M_PIf 3.14159265358979323846F
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/power2.h */

int hor_power_of_two             ( int power );
int hor_highest_power_of_two_leq ( int num );
int hor_int_log_to_base_two      ( int num );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/num_comp.h */

/*******************
*   @hor_min2 ( n1, n2 )   @hor_min3 ( n1, n2, n3 )
*   @hor_max2 ( n1, n2 )   @hor_max3 ( n1, n2, n3 )
*
*   Number comparison macros for speed with simple expressions.
********************/
#define hor_min2(n1,n2) ((n2 < n1) ? n2 : n1)
#define hor_max2(n1,n2) ((n2 > n1) ? n2 : n1)
#define hor_min3(n1,n2,n3) ((n2<n1)?((n3<n2) ? n3 : n2) : ((n3<n1) ? n3 : n1))
#define hor_max3(n1,n2,n3) ((n2>n1)?((n3>n2) ? n3 : n2) : ((n3>n1) ? n3 : n1))

/* functions for complex expressions */
int    hor_imin ( int    n1, int    n2 );
int    hor_imax ( int    n1, int    n2 );
float  hor_fmin ( float  n1, float  n2 );
float  hor_fmax ( float  n1, float  n2 );
double hor_dmin ( double n1, double n2 );
double hor_dmax ( double n1, double n2 );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/matrix.h */

/*******************
*   typedef struct
*   {
*      int      rsize; (maximum vertical size of matrix)
*      int      csize; (maximum horizontal size of matrix)
*      int      rows;  (vertical size of matrix actually used (rows <= rsize))
*      int      cols;  (horiz.   size of matrix actually used (cols <= csize))
*      double **m;     (matrix data)
*   } Hor_Matrix;
*
*   Definition of matrix structure.
********************/
typedef struct
{
   int rsize, csize, rows, cols;
   double **m;
} Hor_Matrix;

Hor_Matrix *hor_mat_alloc     ( int rows, int cols );
void        hor_mat_free      ( Hor_Matrix *M );
void        hor_mat_free_list ( Hor_Matrix *M, ... );
void        hor_mat_print     ( Hor_Matrix *M, const char *string );

/* mats_... slow matrix functions: do allocation for result matrices */
Hor_Matrix *hor_mats_fill      ( int rows, int cols, ... );
Hor_Matrix *hor_mats_copy      ( Hor_Matrix *M );
Hor_Matrix *hor_mats_add2      ( Hor_Matrix *A, Hor_Matrix *B );
Hor_Matrix *hor_mats_sub       ( Hor_Matrix *A, Hor_Matrix *B );
Hor_Matrix *hor_mats_prod2     ( Hor_Matrix *A, Hor_Matrix *B );
Hor_Matrix *hor_mats_prod3     ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C );
Hor_Matrix *hor_mats_scale     ( Hor_Matrix *M, double factor );
Hor_Matrix *hor_mats_transpose ( Hor_Matrix *M );
Hor_Matrix *hor_mats_lud       ( Hor_Matrix *M );
Hor_Matrix *hor_mats_lud_inv   ( Hor_Matrix *M_LUD );
Hor_Matrix *hor_mats_prod_lu   ( Hor_Matrix *M_LUD );
Hor_Matrix *hor_mats_prod_ul   ( Hor_Matrix *M_LUD );
Hor_Matrix *hor_mats_inv       ( Hor_Matrix *M );
Hor_Matrix *hor_mats_solve     ( Hor_Matrix *A, Hor_Matrix *b );
Hor_Matrix *hor_mats_gen_solve ( Hor_Matrix *A, Hor_Matrix *b, Hor_Matrix *x );
Hor_Matrix *hor_mats_pseud_inv ( Hor_Matrix *A );
Hor_Matrix *hor_mats_identity  ( int size );
Hor_Matrix *hor_mats_zero      ( int rows, int cols );
Hor_Matrix *hor_mats_diagonal  ( int size, ... );
Hor_Matrix *hor_mats_extract ( Hor_Matrix *source,
			       int c0, int r0, int width, int height );

/* matq_... quick matrix functions: take allocated result matrices as
                                    arguments. All matrix sizes are checked */
Hor_Matrix *hor_matq_fill      ( Hor_Matrix *M, ... );
Hor_Matrix *hor_matq_copy      ( Hor_Matrix *M, Hor_Matrix *N );
Hor_Matrix *hor_matq_add2      ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C );
Hor_Matrix *hor_matq_sub       ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C );
Hor_Matrix *hor_matq_prod2     ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C );
Hor_Matrix *hor_matq_prod3     ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C,
				 Hor_Matrix *WSP, Hor_Matrix *D );
Hor_Matrix *hor_matq_scale       ( Hor_Matrix *M, double factor );
Hor_Matrix *hor_matq_transpose   ( Hor_Matrix *M, Hor_Matrix *MT );
Hor_Matrix *hor_matq_lud         ( Hor_Matrix *M, Hor_Matrix *M_LUD );
Hor_Matrix *hor_matq_lud_inv     ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_inv );
Hor_Matrix *hor_matq_prod_lu     ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_lu );
Hor_Matrix *hor_matq_prod_ul     ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_ul );
Hor_Matrix *hor_matq_solve_lower (Hor_Matrix *L, Hor_Matrix *x, Hor_Matrix *y);
Hor_Matrix *hor_matq_solve_upper (Hor_Matrix *L, Hor_Matrix *x, Hor_Matrix *y);
Hor_Matrix *hor_matq_inv         ( Hor_Matrix *M,         Hor_Matrix *M_LUD,
				   Hor_Matrix *M_LUD_inv, Hor_Matrix *M_inv );
Hor_Matrix *hor_matq_ludcmp      ( Hor_Matrix *A, int *indx, double *d,
				   double *vv );
Hor_Matrix *hor_matq_lubksb      ( Hor_Matrix *A, int *indx, Hor_Matrix *b );
Hor_Bool    hor_matq_tred2       ( Hor_Matrix *A, double *d, double *e );
Hor_Bool    hor_matq_tqli        ( double *d, double *e, Hor_Matrix *Z );
Hor_Bool    hor_matq_eigsrt      ( double *d, Hor_Matrix *V );
Hor_Bool    hor_matq_svdcmp      ( Hor_Matrix *A, double *W, Hor_Matrix *V );
Hor_Bool    hor_matq_gaussj      ( Hor_Matrix *A, Hor_Matrix *B );
Hor_Bool    hor_matq_mrqmin      ( double x[], double y[], double sig[],
				   int ndata, double a[], int ma, int lista[],
				   int mfit, Hor_Matrix *covar,
				   Hor_Matrix *alpha, double *chisq,
				   void (*funcs)(double, double *, double *,
						 double *, int),
				   double *alamda );
Hor_Matrix *hor_matq_gen_solve ( Hor_Matrix *A, Hor_Matrix *b,
				 Hor_Matrix *U, Hor_Matrix *V, double *W,
				 Hor_Matrix *x );

Hor_Matrix *hor_matq_identity    ( Hor_Matrix *M );
Hor_Matrix *hor_matq_zero        ( Hor_Matrix *M );
Hor_Matrix *hor_matq_zero_lower  ( Hor_Matrix *M );
Hor_Matrix *hor_matq_zero_upper  ( Hor_Matrix *M );
Hor_Matrix *hor_matq_diagonal    ( Hor_Matrix *M, ... );
Hor_Matrix *hor_matq_extract ( Hor_Matrix *source,
			       int c0, int r0, int width, int height,
			       Hor_Matrix *dest );

Hor_Matrix *hor_mat_swap_rows ( Hor_Matrix *M, int row1, int row2 );
Hor_Matrix *hor_mat_swap_cols ( Hor_Matrix *M, int col1, int col2 );

Hor_Matrix *hor_mat_insert ( Hor_Matrix *source, int cs, int rs,
			     Hor_Matrix *dest,   int cd, int rd,
			     int width, int height );
Hor_Matrix *hor_mat_prod2_offset ( Hor_Matrix *M1, Hor_Matrix *M2,
				   Hor_Matrix *dest,
				   int offset_x, int offset_y );
Hor_Matrix *hor_mat_prod2_part ( Hor_Matrix *M1, int c1, int r1, int h1,
				 Hor_Matrix *M2, int c2, int r2, int w2,
				 int common, Hor_Matrix *dest );
Hor_Matrix *hor_mat_prod3_part (Hor_Matrix *M1, Hor_Matrix *M2, Hor_Matrix *M3,
				int offset_x, int offset_y,
				Hor_Matrix *work, Hor_Matrix *dest );
Hor_Matrix *hor_mat_inc_offset ( Hor_Matrix *dest, Hor_Matrix *source,
				 int offset_x, int offset_y );
Hor_Matrix *hor_mat_dec_offset ( Hor_Matrix *dest, Hor_Matrix *source,
				 int offset_x, int offset_y );
double hor_mat_det ( Hor_Matrix *M );
double hor_mat_trace ( Hor_Matrix *M );
double hor_mat_Fnorm ( Hor_Matrix *M );

void hor_mat_read ( Hor_Matrix *M, ... );

Hor_Matrix *hor_matq_set_dims ( Hor_Matrix *M, int rows, int cols );

/*******************
*   Hor_Bool @hor_mat_is_square  ( Hor_Matrix *M )
*   Hor_Bool @hor_mat_test_size  ( Hor_Matrix *M, int rows, int cols )
*   Hor_Bool @hor_mat_same_size2 ( Hor_Matrix *A, Hor_Matrix *B )
*   Hor_Bool @hor_mat_same_size3 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C)
*   Hor_Bool @hor_mat_fits_in    ( Hor_Matrix *A, Hor_Matrix *B )
*   Hor_Bool @hor_mat_big_enough ( Hor_Matrix *M, int rows, int cols )
*
*   Hor_Matrix tests.
*
*   hor_mat_is_square()  returns HOR_TRUE if M is square, HOR_FALSE otherwise.
*   hor_mat_test_size()  returns HOR_TRUE if M has given dimensions (rows &
*                        cols fields of M).
*   hor_mat_same_size2() returns HOR_TRUE if A & B are the same size (rows &
*                        cols fields).
*   hor_mat_same_size3() returns HOR_TRUE if A, B & C have same size.
*   hor_mat_fits_in()    returns HOR_TRUE if A (rows & cols fields) fits in the
*                        allocated size of B (rsize & csize fields).
*   hor_mat_big_enough() returns HOR_TRUE if allocated dimensions of M are >=
*                        the rows and cols arguments.
*
*   All are implemented as macros.
********************/
#define hor_mat_is_square(M) ((M)->rows == (M)->cols)
#define hor_mat_test_size(M,rs,cs) ((M)->rows == (rs) && (M)->cols == (cs))
#define hor_mat_same_size2(A,B) ((A)->rows == (B)->rows && (A)->cols == (B)->cols)
#define hor_mat_same_size3(A,B,C) (hor_mat_same_size2(A,B) && hor_mat_same_size2(B,C))
#define hor_mat_fits_in(A,B) ((A)->rows <= (B)->rsize && (A)->cols <= (B)->csize)
#define hor_mat_big_enough(M,rs,cs) ((M)->rsize >= (rs) && (M)->csize >= (cs))

/*******************
*   Hor_Matrix *@hor_mat_increment ( Hor_Matrix *A, Hor_Matrix *B )
*   Hor_Matrix *@hor_mat_decrement ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Respectively increment and decrement the entries of A by B and return A.
*
*   Both are implemeted as macros, calling respectively hor_matq_add2() and
*   hor_matq_sub().
********************/
#define hor_mat_increment(A,B) hor_matq_add2(A,B,A)
#define hor_mat_decrement(A,B) hor_matq_sub(A,B,A)
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/vector.h */

double      hor_vec_scalar_prod ( Hor_Matrix *x, Hor_Matrix *y );
double      hor_vec_max_coord   ( Hor_Matrix *x );
Hor_Matrix *hor_vecs_cross_prod ( Hor_Matrix *x, Hor_Matrix *y );
Hor_Matrix *hor_vecs_unit       ( Hor_Matrix *x );
Hor_Matrix *hor_vecq_cross_prod ( Hor_Matrix *x, Hor_Matrix *y, Hor_Matrix *z);
Hor_Matrix *hor_vecq_unit       ( Hor_Matrix *x );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/gauss.h */

/* Commented out by davison@etl.go.jp 98/8/14: conflicts with later
   C libraries */
/* long random(void); */

/*******************
*   double @hor_drandom(void)
*
*   Returns random number between -1 and 1.
********************/
#define hor_drandom() ((double)(random() - 1073741824)/1073741824.0)

double hor_gauss_rand ( double mean, double st_dev );
/* Copyright 1994 Philip McLauchlan (pm@robots.oxford.ac.uk) and
                  David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/orth_reg.h */

#ifdef _HORATIO_LIST_

void hor_oreg_line_init ( Hor_Assoc_Label label, int dim );

#define hor_oreg_line_free(label) (hor_scatter_free(label))
#define hor_oreg_line_reset(label) (hor_scatter_reset(label))
#define hor_oreg_line_data(label,Scrow) (hor_scatter_data((label),(Scrow)))
#define hor_oreg_line_data_m(label,Scmat) (hor_scatter_data_m((label),(Scmat)))
#define hor_oreg_line_data_v hor_scatter_data_v

int hor_oreg_line_solve ( Hor_Assoc_Label label, 
			  float sigma, float conf_level,
			  int no_init_points,
			  double *v_direct, double *centroid );

#endif /* _HORATIO_LIST_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/chi2.h */

double hor_normal_prob ( double mean, double st_dev, double value );
double hor_normal_thres ( double confidence_level );
double hor_chi_2_prob ( double chi_2, int df );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/kalman.h */

void hor_kalman_init ( Hor_Matrix *x0, /* initial value of state vector     */
		       Hor_Matrix *P0, /* prior covariance or x0            */
		       int     usize,  /* size of control vector            */
		       int     max_zsize ); /* maximum size of observation
					       vector*/
void hor_kalman_predict ( Hor_Matrix *F, /* state transition matrix          */
			  Hor_Matrix *G, /* input (control) transition matrix*/
			  Hor_Matrix *u, /* control vector                   */
			  Hor_Matrix *Q ); /* prediction error covariance    */
Hor_Bool hor_kalman_update ( Hor_Matrix *z, /* observation vector           */
			     Hor_Matrix *H, /* observation matrix           */
			     Hor_Matrix *R, /* observation covariance       */
			     double  gate_thres ); /* threshold on validation
						      gate */
Hor_Bool hor_kalman_step ( Hor_Matrix *F, /* state transition matrix */
			   Hor_Matrix *G, /* input (control) transition
					     matrix */
			   Hor_Matrix *u, /* control vector */
			   Hor_Matrix *Q, /* prediction error covariance */
			   Hor_Matrix *z, /* observation vector */
			   Hor_Matrix *H, /* observation matrix */
			   Hor_Matrix *R, /* observation covariance */
			   double  gate_thres ); /* threshold on validation
						    gate */

Hor_Bool hor_ext_kalman_update ( Hor_Matrix *z,
				 void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
						Hor_Matrix *grad_hT),
				 Hor_Matrix *R, double gate_thres );
Hor_Bool hor_ext_kalman_step ( Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
			       Hor_Matrix *Q, Hor_Matrix *z,
			       void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					      Hor_Matrix *grad_hT),
			       Hor_Matrix *R, double  gate_thres );

Hor_Matrix *hor_kalman_state_vector(void);
Hor_Matrix *hor_kalman_covariance(void);
void        hor_kalman_print_state(void);

/* multiple filter functions */

/*******************
*   typedef enum { @HOR_EKF_BASIC, @HOR_EKF_ITERATED, @HOR_EKF_RECURSIVE }
*      @Hor_EKF_Mode;
*
*   Modes for running Extended Kalman Filter:
*
*   HOR_EKF_BASIC:     Basic linear EKF.
*   HOR_EKF_ITERATED:  Iterated extended Kalman filter.
*   HOR_EKF_RECURSIVE: Recursively iterated extended Kalman filter.
********************/
typedef enum { HOR_EKF_BASIC, HOR_EKF_ITERATED, HOR_EKF_RECURSIVE }
   Hor_EKF_Mode;

#ifdef _HORATIO_LIST_

void hor_kf_init ( Hor_Assoc_Label label, Hor_Matrix *x0, Hor_Matrix *P0,
		   int usize, int max_zsize );
void hor_kf_predict ( Hor_Assoc_Label label, Hor_Matrix *F,
		      Hor_Matrix *G, Hor_Matrix *u, Hor_Matrix *Q );
Hor_Bool hor_kf_update ( Hor_Assoc_Label label, Hor_Matrix *z, Hor_Matrix *H,
			 Hor_Matrix *R, double gate_thres );
Hor_Bool hor_kf_step ( Hor_Assoc_Label label,
		       Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
		       Hor_Matrix *Q, Hor_Matrix *z, Hor_Matrix *H,
		       Hor_Matrix *R, double  gate_thres );

void hor_ekf_set_mode ( Hor_Assoc_Label label, Hor_EKF_Mode mode );
void hor_ekf_set_iterations ( Hor_Assoc_Label label, int iterations );
void hor_ekf_set_threshold ( Hor_Assoc_Label label, double threshold );
void hor_ekf_predict ( Hor_Assoc_Label label,
		       void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				      Hor_Matrix *f, Hor_Matrix *grad_fT,
				      void *data),
		       Hor_Matrix *u, Hor_Matrix *Q, void *predict_data );
Hor_Bool hor_ekf_update ( Hor_Assoc_Label label, Hor_Matrix *z,
			  void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					 Hor_Matrix *grad_hT, void *data),
			  Hor_Matrix *R, double gate_thres, void *update_data);
Hor_Bool hor_ekf_step ( Hor_Assoc_Label label,
		        void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				       Hor_Matrix *f, Hor_Matrix *grad_fT,
				       void *data),
		        Hor_Matrix *u, Hor_Matrix *Q, void *predict_data,
		        Hor_Matrix *z,
		        void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
				       Hor_Matrix *grad_hT, void *data),
		        Hor_Matrix *R, double  gate_thres, void *update_data );

Hor_Matrix *hor_kf_state_vector ( Hor_Assoc_Label label );
Hor_Matrix *hor_kf_covariance   ( Hor_Assoc_Label label );
void        hor_kf_print_state  ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/vsdf.h */

#ifdef _HORATIO_LIST_
void hor_vsdf_init ( Hor_Assoc_Label label,
		     Hor_Matrix *xf0, int xf_type, Hor_Matrix *Pf0inv,
		     Hor_Matrix *xd0, int xd_type,
		     void *user_state, int max_y_size, int max_z_size,
		     int max_n, Hor_Bool use_optimal_dynamic_vsdf );
Hor_Bool hor_vsdf_initialised(void);
void hor_vsdf_switch ( Hor_Assoc_Label label );
void hor_vsdf_free ( Hor_Assoc_Label label );
#endif /* _HORATIO_LIST_ */

void *hor_vsdf_get_user_state(void);
void hor_vsdf_set_user_state ( void *new_user_state );

int hor_vsdf_add_state ( Hor_Matrix *y0, int y_type,
			 Hor_Matrix *T0inv, int *index_ptr );

/*******************
*   #define @HOR_VSDF_UNINIT_OFFSET 16383 (offset for start index of
*                                          uninitialized local states)
*   #define @HOR_VSDF_ERROR            -1 (index and return value signifying
*                                          an error)
*   #define @HOR_VSDF_NOT_PRESENT      -2 (index and return value signifying
*                                          lack of presence in VSDF)
*   #define @HOR_VSDF_OUTLIER          -3 (index and return value signifying
*                                          an outlier detected in the VSDF)
********************/
#define HOR_VSDF_UNINIT_OFFSET 16383
#define HOR_VSDF_ERROR            -1
#define HOR_VSDF_NOT_PRESENT      -2
#define HOR_VSDF_OUTLIER          -3

int hor_vsdf_addu_state ( int *index_ptr );
int hor_vsdf_uninit_frames ( int index );
int hor_vsdf_init_state ( int index, int iterations, double conf_level,
			  Hor_Matrix *(*y0_func)(Hor_Matrix  *xf, int  xf_type,
						 Hor_Matrix **xd, int *xd_type,
						 Hor_Matrix **z,  int  *z_type,
						 void       **user_data, int k,
						 void        *user_state,
						 int  *y_type) );
void hor_vsdf_remove_state ( int index, Hor_Bool keep_data );
Hor_Bool hor_vsdf_test_state ( int index, double conf_level );

void hor_vsdf_obs_h ( int index, Hor_Matrix *z, int z_type, Hor_Matrix *Rinv,
		      void (*h_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type, int z_type,
				      Hor_Matrix *h,  Hor_Matrix *D,
				      Hor_Matrix *Dp, Hor_Matrix *E,
				      void *user_data, void *user_state),
		      void *user_data );
void hor_vsdf_obs_F ( int index, Hor_Matrix *z, int z_type,
		      Hor_Matrix *N, int F_size,
		      void (*F_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type,
				      Hor_Matrix *z,  int  z_type,
				      Hor_Matrix *F,  Hor_Matrix *D,
				      Hor_Matrix *Dp, Hor_Matrix *E,
				      Hor_Matrix *Fz,
				      void *user_data, void *user_state),
		      void *user_data );
void hor_vsdf_obs_h_num ( int index, Hor_Matrix *z, int z_type,
			  Hor_Matrix *Rinv,
			  void (*hn_func) (Hor_Matrix *xf, int xf_type,
					   Hor_Matrix *xd, int xd_type,
					   Hor_Matrix *y,  int  y_type,
					   int  z_type, Hor_Matrix *h,
					   void *user_data, void *user_state),
			  void *user_data );

void hor_vsdf_store(void);
Hor_Bool hor_vsdf_batch ( Hor_Bool (*init_func)(Hor_Matrix  *xf, int *xf_type,
						Hor_Matrix **xd,
						int         *xd_type, int k,
						Hor_Matrix **y,  int *y_type,
						                 int n,
						Hor_Matrix **yu, int *yu_type,
						                 int nu,
						Hor_Matrix ***zu,
						int         **zu_type,
						void       ***data_u,
						void *user_data,
						void *user_state),
			  int iterations1, int iterations2,
			  double conf_level, void *user_data );

Hor_Bool hor_vsdf_change_frame(Hor_Bool (*change_func)(Hor_Matrix  *xf,
						       int *xf_type,
						       Hor_Matrix *Af,
						       Hor_Matrix  *xd,
						       int *xd_type,
						       int n,
						       Hor_Matrix **y,
						       int *y_type,
						       Hor_Matrix **vf,
						       Hor_Matrix **C));

typedef enum { HOR_MOTION_UPDATE, HOR_DROID_UPDATE, HOR_FULL_UPDATE }
   Hor_VSDF_Update_Type;
Hor_Bool *hor_vsdf_update ( int dynamic_iterations, double conf_level,
			    Hor_VSDF_Update_Type update_type,
			    void (*update_func)(Hor_Matrix  *xf, int  xf_type,
						Hor_Matrix  *xd, int *xd_type,
						Hor_Matrix **y,  int *y_type,
						Hor_Matrix **C,
						Hor_Matrix **z,  int *z_type,
						void       **zdata,
						Hor_Bool    *z_accept,
						int          n,
						void *user_data,
						void *user_state),
			    void (*reset_func)(Hor_Matrix  *xf, int  xf_type,
					       Hor_Matrix  *xd, int *xd_type,
					       Hor_Matrix **y,  int  *y_type,
					       Hor_Matrix **C,
					       Hor_Matrix **z,  int *z_type,
					       void       **zdata,
					       Hor_Bool    *z_accept,
					       int          n,
					       void *user_data,
					       void *user_state),
			    void *user_data );

void hor_vsdf_scale ( double factor );
void hor_vsdf_reset(void);

typedef struct
{
   Hor_Matrix *xd;
   int         xd_type;
   Hor_Matrix *Ad;
} Hor_VSDF_XD_Def;

int         hor_vsdf_get_k(void);
Hor_Matrix *hor_vsdf_get_xf(void);
int         hor_vsdf_get_xf_type(void);
Hor_Matrix *hor_vsdf_get_xd(void);
int         hor_vsdf_get_xd_type(void);
#ifdef _HORATIO_LIST_
Hor_List    hor_vsdf_get_xdlist(void); /* returns a list of pointers
					  to Hor_VSDF_XD_Def's */
#endif
Hor_Matrix *hor_vsdf_get_Pf(void);
Hor_Matrix *hor_vsdf_get_Af(void);
Hor_Matrix *hor_vsdf_get_Pd(void);
Hor_Matrix *hor_vsdf_get_Ad(void);
double      hor_vsdf_get_J(void);
int         hor_vsdf_get_DOF(void);
int         hor_vsdf_get_n(void);
int         hor_vsdf_get_max_ysize(void);
int         hor_vsdf_get_max_zsize(void);
Hor_Matrix *hor_vsdf_get_y      ( int index );
int         hor_vsdf_get_y_type ( int index );
Hor_Matrix *hor_vsdf_get_T      ( int index );
Hor_Matrix *hor_vsdf_get_C      ( int index );
int         hor_vsdf_get_obs_k  ( int index );
void       *hor_vsdf_get_data   ( int index );
double      hor_vsdf_get_Ji     ( int index );
int         hor_vsdf_get_DOFi   ( int index );
Hor_Matrix *hor_vsdf_get_z      ( int index );
int         hor_vsdf_get_z_type ( int index );

void hor_vsdf_print_state(void);
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/mat_io.h */


#ifdef HOR_TRANSPUTER
#ifdef _channel_h

Hor_Matrix *HorChanInMatrix          ( Channel *c );
void        HorChanInAllocatedMatrix ( Channel *c, Hor_Matrix *M );
void        HorChanOutMatrix         ( Channel *c, Hor_Matrix *M );

#endif
#endif

#ifndef HOR_REDUCED_LIB

Hor_Matrix *hor_read_matrix           ( int fd );
void        hor_read_allocated_matrix ( int fd, Hor_Matrix *M );
void        hor_write_matrix          ( int fd, Hor_Matrix *M );

#endif
/* Copyright 1993 Philip Torr (phst@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/poly.h */

int hor_solve_quadratic ( double a, double b, double c,
			  double *x1, double *x2 );
int hor_solve_cubic ( double a, double b, double c, double d,
		      double *x1, double *x2, double *x3 );
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/pseudo.h */

#ifdef _HORATIO_LIST_

void hor_pseudo_inv_init ( Hor_Assoc_Label label, int Xrows, int Xcols );
void hor_pseudo_inv_free ( Hor_Assoc_Label label );
void hor_pseudo_inv_reset ( Hor_Assoc_Label label );
void hor_pseudo_inv_data ( Hor_Assoc_Label label, double *Arow, double *Brow );
void hor_pseudo_inv_data_m ( Hor_Assoc_Label label,
			     Hor_Matrix *Amat, Hor_Matrix *Bmat );
void hor_pseudo_inv_data_v ( Hor_Assoc_Label label, ... );
Hor_Matrix *hor_pseudo_inv_solve ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
/* Copyright 1994 Philip McLauchlan (pm@robots.oxford.ac.uk) and
                  David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/eigen.h */

#ifdef _HORATIO_LIST_

void hor_eigen_init ( Hor_Assoc_Label label, int Awidth );
void hor_eigen_free ( Hor_Assoc_Label label );
void hor_eigen_reset ( Hor_Assoc_Label label );
void hor_eigen_data ( Hor_Assoc_Label label, double *Arow );
void hor_eigen_data_m ( Hor_Assoc_Label label, Hor_Matrix *Amat );
void hor_eigen_data_v ( Hor_Assoc_Label label, ... );
void hor_eigen_solve ( Hor_Assoc_Label label,
		       Hor_Matrix *eigenvectors, double *eigenvalues );
Hor_Matrix *hor_eigen_ATA ( Hor_Assoc_Label label );

void hor_scatter_init ( Hor_Assoc_Label label, int Scwidth );
void hor_scatter_free ( Hor_Assoc_Label label );
void hor_scatter_reset ( Hor_Assoc_Label label );
void hor_scatter_data ( Hor_Assoc_Label label, double *Scrow );
void hor_scatter_data_m ( Hor_Assoc_Label label, Hor_Matrix *Scmat );
void hor_scatter_data_v ( Hor_Assoc_Label label, ... );
void hor_scatter_solve ( Hor_Assoc_Label label,
		       Hor_Matrix *eigenvectors, 
			double *eigenvalues, 
			double *centroid );
int hor_scatter_dim ( Hor_Assoc_Label label );
int hor_scatter_get_n ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
