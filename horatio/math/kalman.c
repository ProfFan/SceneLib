/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#ifdef HOR_TRANSPUTER
#include <stdiored.h>
#else
#include <stdio.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"

static int max_xsize = 0; /* maximum size of state vector   */
static int max_zsize = 0; /* maximum size of observation vector   */
static int max_usize = 0; /* maximum size of control vector   */
static Hor_Matrix *xp;    /* predicted state vector               */
static Hor_Matrix *Pinv;  /* state vector inverse covariance      */
static Hor_Matrix *Pp;    /* prediction state vector covariance   */
static Hor_Matrix *FT;    /* transpose of state transition matrix */
static Hor_Matrix *HT;    /* transpose of observation matrix      */
static Hor_Matrix *S;     /* innovation variance                  */
static Hor_Matrix *Sinv;  /* inverse of innovation variance       */
static Hor_Matrix *W;     /* gain matrix                          */
static Hor_Matrix *WT;    /* transpose of gain matrix             */
static Hor_Matrix *v;     /* innovation z-H*xp                    */
static Hor_Matrix *vT;    /* transpose of innovation              */

/* EKF matrices */
static Hor_Matrix *grad_fT, *h, *grad_hT;

/* workspace matrices */
static Hor_Matrix *work1, *work2, *work3;

static void static_init ( int xsize, int zsize, int usize )
{
   int max_size;

   if ( max_xsize > 0 )
      hor_mat_free_list ( work3, work2, work1, grad_hT, h, grad_fT, vT, v,
			  WT, W, Sinv, S, HT, FT, Pp, Pinv, xp, NULL );

   max_xsize = xsize;
   max_zsize = zsize;
   max_usize = usize;

   xp   = hor_mat_alloc ( xsize,     1 );
   Pinv = hor_mat_alloc ( xsize, xsize );
   Pp   = hor_mat_alloc ( xsize, xsize );
   FT   = hor_mat_alloc ( xsize, xsize );
   HT   = hor_mat_alloc ( xsize, zsize );
   S    = hor_mat_alloc ( zsize, zsize );
   Sinv = hor_mat_alloc ( zsize, zsize );
   W    = hor_mat_alloc ( xsize, zsize );
   WT   = hor_mat_alloc ( zsize, xsize );
   v    = hor_mat_alloc ( zsize,     1 );
   vT   = hor_mat_alloc (     1, zsize );

   /* EKF stuff */
   grad_fT    = hor_mat_alloc ( xsize, xsize );
   h          = hor_mat_alloc ( zsize,     1 );
   grad_hT    = hor_mat_alloc ( zsize, xsize );
   
   /* allocate workspace */
   max_size = hor_max3 ( xsize, zsize, usize );
   work1  = hor_mat_alloc ( max_size, max_size );
   work2  = hor_mat_alloc ( max_size, max_size );
   work3  = hor_mat_alloc ( max_size, max_size );
}

typedef struct
{
   int xsize;     /* size of state vector */
   Hor_Matrix *x; /* state vector */
   Hor_Matrix *P; /* state vector covariance */
   int usize;     /* size of control vector */
   int max_zsize; /* maximum size of observation vector */

   /* for extended KF */
   Hor_EKF_Mode mode;

   /* parameters for termination criteria for iterated versions of EKF */
   int    iterations; /* of iterated/recursively iterated EKF. */
   double threshold;  /* threshold on state change magnitude. */
} KF_State;

static void filter_init ( KF_State *state, Hor_Matrix *x0, Hor_Matrix *P0,
			  int usize, int max_zsize )
{
   /* free old state */
   if ( state->xsize != 0 ) hor_mat_free_list ( state->P, state->x, NULL );

   /* check dimensions of all vectors & matrices */
   if ( x0->cols != 1 )
      hor_error ( "x vector non-column (%d) (filter_init)", HOR_FATAL,
		  x0->cols );

   if ( P0->rows != x0->rows || P0->cols != x0->rows )
      hor_error ( "covariance matrix wrong size (%d,%d) %d (filter_init)",
		  HOR_FATAL, P0->rows, P0->cols, x0->rows );

   if ( usize < 0 )
      hor_error ( "control vector illegal size %d (filter_init)",
		  HOR_FATAL, usize );

   if ( max_zsize <= 0 )
      hor_error ( "observation vector illegal maximum size %d (filter_init)",
		  HOR_FATAL, max_zsize );

   state->xsize      = x0->rows;
   state->x          = hor_mats_copy ( x0 );
   state->P          = hor_mats_copy ( P0 );
   state->usize      = usize;
   state->max_zsize  = max_zsize;
   state->mode       = HOR_EKF_BASIC;
   state->iterations = 3;
   state->threshold  = 0.0;
}

static void filter_predict  ( KF_State *state, Hor_Matrix *F, Hor_Matrix *G,
			                       Hor_Matrix *u, Hor_Matrix *Q )
{
   /* check dimensions of all vectors & matrices */
   if ( !hor_mat_test_size ( F, state->xsize, state->xsize ) )
      hor_error ( "state trans. mat. wrong size (%d,%d) %d (filter_predict)",
		  HOR_FATAL, F->rows, F->cols, state->xsize );

   if ( state->usize > 0 )
   {
      if ( !hor_mat_test_size ( G, state->xsize, state->usize ) )
	 hor_error("input trans. mat. wrong size (%d,%d) %d %d (filter_predict)", HOR_FATAL, G->rows, G->cols, state->xsize, state->usize);

      if ( !hor_mat_test_size ( u, state->usize, 1 ) )
	 hor_error ( "control vec. wrong size (%d,%d) %d 1 (filter_predict)",
		     HOR_FATAL, u->rows, u->cols, state->usize );
   }

   if ( !hor_mat_test_size ( Q, state->xsize, state->xsize ) )
      hor_error ( "prediction error cov. wrong size (%d,%d) %d (filter_predict)", HOR_FATAL, Q->rows, Q->cols, state->xsize );

   /* perform state vector prediction: xp = F*x + G*u */
   if ( state->usize > 0 )
      hor_matq_add2 ( hor_matq_prod2 ( F, state->x, work1 ),
		      hor_matq_prod2 ( G, u, work2 ), xp );
   else /* no control vector u: xp = F*x */
      hor_matq_prod2 ( F, state->x, xp );

   /* perform state covariance prediction: Pp = F*P*F^T + Q */
   hor_matq_transpose ( F, FT );
   hor_matq_add2 ( hor_matq_prod3 ( F, state->P, FT, work1,
				    work2 ), Q, Pp );

   /* copy predictions into state and covariance */
   hor_matq_copy ( xp, state->x );
   hor_matq_copy ( Pp, state->P );
}

static Hor_Bool filter_update ( KF_State *state, Hor_Matrix *v, Hor_Matrix *H,
			        Hor_Matrix *R, double gate_thres )
{
   hor_matq_transpose ( v, vT );

   if ( !hor_mat_test_size ( R, v->rows, v->rows ) )
      hor_error ( "obs. covariance wrong size (%d,%d) %d (filter_update)",
		  HOR_FATAL, R->rows, R->cols, v->rows );

   /* calculate innovation covariance S = H*P*H^T + R */
   hor_matq_transpose ( H, HT );
   hor_matq_add2 ( hor_matq_prod3 ( H, state->P, HT, work1, work2 ), R, S);

   /* calculate Sinv = S^-1 */
   hor_matq_inv ( S, work1, work2, Sinv );

   /* calculate validation gate vT*S^-1*v */
   hor_matq_prod3 ( vT, Sinv, v, work1, work2 );
   if ( hor_chi_2_prob ( work2->m[0][0], v->rows ) < gate_thres )
      return HOR_FALSE;

   /* calculate gain matrix W = P*HT*S^-1 */
   hor_matq_prod3 ( state->P, HT, Sinv, work1, W );
   hor_matq_transpose ( W, WT );

   /* calculate new state vector x = x + W*v */
   hor_matq_add2 ( state->x, hor_matq_prod2 ( W, v, work1 ), state->x );

   /* calculate new covariance P = P - WSW^T */
   hor_matq_sub ( state->P, hor_matq_prod3 ( W, S, WT, work1, work2 ),
		  state->P );

   return HOR_TRUE; /* update successful */
}

static KF_State filter = { 0, NULL, NULL, 0, 0, HOR_EKF_BASIC, 3, 0.0 };

/*******************
*   void @hor_kalman_init ( Hor_Matrix *x0,     (initial value of state vector)
*                          Hor_Matrix *P0,      (prior covariance or x0)
*                          int     usize,      (size of control vector)
*                          int     max_zsize ) (maximum size of observation
*                                               vector)
*
*   Initializes Kalman filter with state vector x0 and covariance matrix P0
*   for time step k=0. Also provided are the sizes of the control vector
*   ( u in state prediction equation x <- F*x + G*u ) and observation vector
*   ( z in measurement equation z = H*x ). If there is no control vector u
*   usize should be zero.
********************/
void hor_kalman_init ( Hor_Matrix *x0, Hor_Matrix *P0,
		       int usize, int loc_max_zsize )
{
   /* initialise static variables */
   if ( x0->rows > max_xsize || loc_max_zsize > max_zsize ||
        usize > max_usize )
      static_init ( x0->rows, loc_max_zsize, usize );

   /* initialise state variables */
   filter_init ( &filter, x0, P0, usize, max_zsize );
}

/*******************
*   void @hor_kalman_predict ( Hor_Matrix *F,  (state transition matrix)
*                             Hor_Matrix *G,  (input transition matrix)
*                             Hor_Matrix *u,  (control vector)
*                             Hor_Matrix *Q ) (prediction error covariance)
*
*   Performs state and covariance prediction for time step k+1:
*
*   State prediction:      x <- F*x + G*u
*   Covariance prediction: P <- F*P*F^T + Q
*
*   hor_kalman_predict() can be called repeatedly to obtain n-step ahead
*   prediction.
********************/
void hor_kalman_predict ( Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
			  Hor_Matrix *Q )
{
   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_kalman_predict)", HOR_FATAL );

   filter_predict ( &filter, F, G, u, Q );
}

/*******************
*   Hor_Bool @hor_kalman_update ( Hor_Matrix *z, (observation vector)
*                                Hor_Matrix *H,  (observation matrix)
*                                Hor_Matrix *R,  (observation covariance)
*                                double gate_thres ) (threshold on validation
*                                                     gate)
*
*   Performs state and covariance update given a new observation:
*
*   State innovation:      v <- z - Hx
*   Innovation covariance: S <- H*P*H^T + R
*   Gain matrix:           W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   Innovation is used to calculate validation gate v^T*S^-1*v, which is
*   a chi-squared variable on zsize DOF, where zsize is the size of z.
*   If validation gate is too large the iteration is aborted and HOR_FALSE
*   returned, otherwise HOR_TRUE is returned.
*
*   If there is no control vector ( usize=0 in hor_kalman_init() ), NULL can
*   be passed for G and u. The state prediction equation given above then
*   becomes  x <- F*x.
*
*   hor_kalman_update() can be called repeatedly, producing a Kalman filter
*   without prediction.
********************/
Hor_Bool hor_kalman_update ( Hor_Matrix *z, Hor_Matrix *H, Hor_Matrix *R,
			     double gate_thres )
{
   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_kalman_update)", HOR_FATAL );

   if ( z->rows > filter.max_zsize || z->cols != 1 )
      hor_error("observation vector wrong size (%d,%d) %d (hor_kalman_update)",
		HOR_FATAL, z->rows, z->cols, filter.max_zsize );

    if ( !hor_mat_test_size ( H, z->rows, filter.xsize ) )
      hor_error( "obs. matrix wrong size (%d,%d) %d %d (hor_kalman_update)",
		 HOR_FATAL, H->rows, H->cols, z->rows, filter.xsize );

   /* calculate innovation v = z - H*x */
   hor_matq_sub ( z, hor_matq_prod2 ( H, filter.x, work1 ), v );

   return ( filter_update ( &filter, v, H, R, gate_thres ) );
}

/*******************
*   Hor_Bool @hor_kalman_step ( Hor_Matrix *F, (state transition matrix)
*                              Hor_Matrix *G, (input (control) transition
*                                              matrix)
*                              Hor_Matrix *u, (control vector)
*                              Hor_Matrix *Q, (prediction error covariance)
*                              Hor_Matrix *z, (observation vector)
*                              Hor_Matrix *H, (observation matrix)
*                              Hor_Matrix *R, (observation covariance)
*                              double  gate_thres ) (threshold on validation
*                                                    gate)
*
*   Iterates the Kalman filter for time step k+1:
*
*   State prediction:      x <- F*x + G*u
*   Covariance prediction: P <- F*P*F^T + Q
*   State innovation:      v <- z - Hx
*   Innovation covariance: S <- H*P*H^T + R
*   Gain matrix:           W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   Innovation is used to calculate validation gate v^T*S^-1*v, which is
*   a chi-squared variable on zsize DOF, where zsize is the size of z.
*   If validation gate is too large the iteration is aborted and HOR_FALSE
*   returned, otherwise HOR_TRUE is returned.
*
*   If there is no control vector ( usize=0 in hor_kalman_init() ), NULL can
*   be passed for G and u. The state prediction equation given above then
*   becomes  x <- F*x.
*
*   hor_kalman_step() calls hor_kalman_predict() and hor_kalman_update().
********************/
Hor_Bool hor_kalman_step ( Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
			   Hor_Matrix *Q, Hor_Matrix *z, Hor_Matrix *H,
			   Hor_Matrix *R, double  gate_thres )
{
   hor_kalman_predict ( F, G, u, Q );
   return ( hor_kalman_update ( z, H, R, gate_thres ) );
}

/*******************
*   Hor_Bool @hor_ext_kalman_update ( Hor_Matrix *z,
*                                    void (*h_func)(Hor_Matrix *x,
*                                                   Hor_Matrix *h,
*                                                   Hor_Matrix *grad_hT),
*                                    Hor_Matrix *R, double gate_thres )
*   Hor_Bool @hor_ext_kalman_step (Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
*                                 Hor_Matrix *Q, Hor_Matrix *z,
*                                 void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
*                                                Hor_Matrix *grad_hT),
*                                 Hor_Matrix *R, double  gate_thres )
*
*   Obsolete functions to perform the extended Kalman filter, replaced
*   by hor_ekf_...() functions below.
*
*   hor_ext_kalman_step() includes the initial prediction steps as in
*   hor_kalman_step():    x <- F*x + G*u,    P <- F*P*F^T + Q.
*
*   We label the current (predicted) state vector x as x0. The measurement
*   vector z is a non-linear function of the state x:
*
*      z = h(x) + noise
*
*   We linearise this equation about the predicted state estimate x0
*   to obtain the innovation
*
*      v = z - h(x0) = noise
*
*   where H is set to (grad x)^T evaluated at x0. The linear update proceeds
*   as in hor_kalman_step().
********************/
Hor_Bool hor_ext_kalman_update ( Hor_Matrix *z,
				 void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
						Hor_Matrix *grad_hT),
				 Hor_Matrix *R, double gate_thres )
{
   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_ext_kalman_update)", HOR_FATAL );

   /* calculate h function and the transpose of its gradient w.r.t. x,
      both evaluated at the current state vector x (identified as x0) */
   h->rows = grad_hT->rows = z->rows;
   grad_hT->cols = filter.xsize;
   h_func ( filter.x, h, grad_hT );

   /* calculate v = z - h(x0) */
   hor_matq_sub ( z, h, v );

   return (filter_update ( &filter, v, grad_hT, R, gate_thres ));
}

Hor_Bool hor_ext_kalman_step ( Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
			       Hor_Matrix *Q, Hor_Matrix *z,
			       void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					      Hor_Matrix *grad_hT),
			       Hor_Matrix *R, double  gate_thres )
{
   hor_kalman_predict ( F, G, u, Q );
   return ( hor_ext_kalman_update ( z, h_func, R, gate_thres ) );
}

/*******************
*   Hor_Matrix *@hor_kalman_state_vector(void)
*   Hor_Matrix *@hor_kalman_covariance(void)
*   void        @hor_kalman_print_state(void)
*
*   Functions to read internal state of Kalman filter.
*
*   hor_kalman_state_vector() returns the current state vector x.
*
*   hor_kalman_covariance() returns the current state covariance P.
*
*   hor_kalman_print_state() prints the current state vector x and covariance
*                            matrix P.
********************/
Hor_Matrix *hor_kalman_state_vector(void)
{
   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_kalman_state_vector)", HOR_FATAL );

   return filter.x;
}

Hor_Matrix *hor_kalman_covariance(void)
{
   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_kalman_covariance)", HOR_FATAL );

   return filter.P;
}

void hor_kalman_print_state(void)
{
   int r;

   if ( filter.xsize == 0 )
      hor_error ( "not initialised (hor_kalman_print_state)", HOR_FATAL );

   hor_print ( "x = (" );
   for ( r = 0; r < filter.xsize; r++ )
      hor_print ( " %f", filter.x->m[r][0] );
   
   hor_print ( " )^T\n" );
   hor_mat_print ( filter.P, "covariance matrix P" );
}

/* new filter functions: */

static Hor_Assoc_List filter_list = NULL;

/*******************
*   void @hor_kf_init ( Hor_Assoc_Label label, (filter identifier)
*                      Hor_Matrix *x0,        (initial value of state vector)
*                      Hor_Matrix *P0,        (prior covariance or x0)
*                      int     usize,         (size of control vector)
*                      int     max_zsize )    (maximum size of observation
*                                              vector)
*
*   Initializes Kalman filter with state vector x0 and covariance matrix P0
*   for time step k=0. Also provided are the sizes of the control vector
*   ( u in state prediction equation x <- F*x + G*u ) and observation vector
*   ( z in measurement equation z = H*x ). If there is no control vector u
*   usize should be zero. Multiple filters can be run by providing different
*   labels
********************/
void hor_kf_init ( Hor_Assoc_Label label, Hor_Matrix *x0, Hor_Matrix *P0,
		   int usize, int loc_max_zsize )
{
   KF_State *state;

   /* initialise static variables */
   if ( x0->rows > max_xsize || loc_max_zsize > max_zsize ||
        usize > max_usize )
      static_init ( x0->rows, loc_max_zsize, usize );

   /* initialise state variables */
   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
   {
      /* new filter */
      state = hor_malloc_type ( KF_State );
      state->xsize = 0; /* labels new state as unitialised */
      filter_list = hor_assoc_insert ( filter_list, label, (void *) state );
   }

   filter_init ( state, x0, P0, usize, max_zsize );
}

/*******************
*   void @hor_kf_predict ( Hor_Assoc_Label label, (filter identifier)
*                         Hor_Matrix     *F,     (state transition matrix)
*                         Hor_Matrix     *G,     (input transition matrix)
*                         Hor_Matrix     *u,     (control vector)
*                         Hor_Matrix     *Q )    (prediction error covariance)
*
*   Performs state and covariance prediction on given filter for time step k+1:
*
*   State prediction:      x <- F*x + G*u
*   Covariance prediction: P <- F*P*F^T + Q
*
*   hor_kf_predict() can be called repeatedly to obtain n-step ahead
*   prediction.
********************/
void hor_kf_predict ( Hor_Assoc_Label label, Hor_Matrix *F,
		      Hor_Matrix *G, Hor_Matrix *u, Hor_Matrix *Q )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_kf_predict)", HOR_FATAL,
		  label );

   filter_predict  ( state, F, G, u, Q );
}

/*******************
*   Hor_Bool @hor_kf_update ( Hor_Assoc_Label label, (filter identifier)
*                            Hor_Matrix     *z,     (observation vector)
*                            Hor_Matrix     *H,     (observation matrix)
*                            Hor_Matrix     *R,     (observation covariance)
*                            double gate_thres )    (threshold on validation
*                                                    gate)
*
*   Performs state and covariance update given a new observation:
*
*   State innovation:      v <- z - Hx
*   Innovation covariance: S <- H*P*H^T + R
*   Gain matrix:           W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   Innovation is used to calculate validation gate v^T*S^-1*v, which is
*   a chi-squared variable on zsize DOF, where zsize is the size of z.
*   If validation gate is too large the iteration is aborted and HOR_FALSE
*   returned, otherwise HOR_TRUE is returned.
*
*   If there is no control vector ( usize=0 in hor_kf_init() ), NULL can
*   be passed for G and u. The state prediction equation given above then
*   becomes  x <- F*x.
*
*   hor_kf_update() can be called repeatedly, producing a Kalman filter
*   without prediction.
********************/
Hor_Bool hor_kf_update ( Hor_Assoc_Label label, Hor_Matrix *z, Hor_Matrix *H,
			 Hor_Matrix *R, double gate_thres )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_kf_update)", HOR_FATAL,
		  label );

   if ( z->rows > state->max_zsize || z->cols != 1 )
      hor_error("observation vector wrong size (%d,%d) %d (hor_kf_update)",
		HOR_FATAL, z->rows, z->cols, state->max_zsize );

    if ( !hor_mat_test_size ( H, z->rows, state->xsize ) )
      hor_error( "obs. matrix wrong size (%d,%d) %d %d (hor_kf_update)",
		 HOR_FATAL, H->rows, H->cols, z->rows, state->xsize );

   /* calculate innovation v = z - H*x */
   hor_matq_sub ( z, hor_matq_prod2 ( H, state->x, work1 ), v );

   return ( filter_update ( state, v, H, R, gate_thres ) );
}

/*******************
*   Hor_Bool @hor_kf_step ( Hor_Assoc_Label label, (filter identifier)
*                          Hor_Matrix      *F,    (state transition matrix)
*                          Hor_Matrix      *G,    (input (control) transition
*                                                  matrix)
*                          Hor_Matrix      *u,    (control vector)
*                          Hor_Matrix      *Q,    (prediction error covariance)
*                          Hor_Matrix      *z,    (observation vector)
*                          Hor_Matrix      *H,    (observation matrix)
*                          Hor_Matrix      *R,    (observation covariance)
*                          double gate_thres )    (threshold on validation
*                                                  gate)
*
*   Iterates the Kalman filter for time step k+1:
*
*   State prediction:      x <- F*x + G*u
*   Covariance prediction: P <- F*P*F^T + Q
*   State innovation:      v <- z - Hx
*   Innovation covariance: S <- H*P*H^T + R
*   Gain matrix:           W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   Innovation is used to calculate validation gate v^T*S^-1*v, which is
*   a chi-squared variable on zsize DOF, where zsize is the size of z.
*   If validation gate is too large the iteration is aborted and HOR_FALSE
*   returned, otherwise HOR_TRUE is returned.
*
*   If there is no control vector ( usize=0 in hor_kf_init() ), NULL can
*   be passed for G and u. The state prediction equation given above then
*   becomes  x <- F*x.
*
*   hor_kf_step() calls hor_kf_predict() and hor_kf_update().
********************/
Hor_Bool hor_kf_step ( Hor_Assoc_Label label,
		       Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
		       Hor_Matrix *Q, Hor_Matrix *z, Hor_Matrix *H,
		       Hor_Matrix *R, double  gate_thres )
{
   hor_kf_predict ( label, F, G, u, Q );
   return ( hor_kf_update ( label, z, H, R, gate_thres ) );
}

/*******************
*   void @hor_ekf_set_mode       ( Hor_Assoc_Label label, Hor_EKF_Mode mode )
*   void @hor_ekf_set_iterations ( Hor_Assoc_Label label, int iterations )
*   void @hor_ekf_set_threshold  ( Hor_Assoc_Label label, double threshold )
*
*   Extended Kalman filter mode/parameter setting functions.
*
*   hor_ekf_set_mode() specifies one of @HOR_EKF_BASIC (the default),
*                      @HOR_EKF_ITERATED or @HOR_EKF_RECURSIVE.
*   hor_ekf_set_iterations() sets the number of Newton-Raphson iterations
*                            used to find maximum of state pdf for iterated
*                            versions of the EKF. The default is 3.
*   hor_ekf_set_threshold() sets the threshold on state change for termination
*                           of Newton-Raphson iterations, by default zero.
********************/
void hor_ekf_set_mode ( Hor_Assoc_Label label, Hor_EKF_Mode mode )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_ekf_set_mode)", HOR_FATAL,
		  label );

   state->mode = mode;
}

void hor_ekf_set_iterations ( Hor_Assoc_Label label, int iterations )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_ekf_set_iterations)",
		  HOR_FATAL, label );

   state->iterations = iterations;
}

void hor_ekf_set_threshold ( Hor_Assoc_Label label, double threshold )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_ekf_set_threshold)",
		  HOR_FATAL, label );

   state->threshold = threshold;
}

/*******************
*   void @hor_ekf_predict ( Hor_Assoc_Label label, (filter identifier)
*                          void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
*                                         Hor_Matrix *f, Hor_Matrix *grad_fT,
*                                         void *data),
*                          Hor_Matrix *u, Hor_Matrix *Q, void *predict_data )
*
*   Performs the prediction stage of the extended Kalman filter.
*   Prediction is a general function f of the previous state vector x0 and
*   a control vector u:
*
*   x = f(x0,u) + noise (covariance Q).
*
*   To first order, state and covariance prediction is accomplished by
*   updating as follows:
*
*   x <- f(x,u),    P <- F*P*F^T + Q
*
*   where F = (grad f)^T evaluated at x0.
*
*   The predict_data pointer is user-defined data passed to f_func().
********************/
void hor_ekf_predict ( Hor_Assoc_Label label,
		       void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				      Hor_Matrix *f, Hor_Matrix *grad_fT,
				      void *data),
		       Hor_Matrix *u, Hor_Matrix *Q, void *predict_data )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_ekf_predict)", HOR_FATAL,
		  label );

   /* check dimensions of all vectors & matrices */
   if ( state->usize > 0 )
      if ( !hor_mat_test_size ( u, state->usize, 1 ) )
	 hor_error ( "control vec. wrong size (%d,%d) %d 1 (hor_ekf_predict)",
		     HOR_FATAL, u->rows, u->cols, state->usize );

   if ( !hor_mat_test_size ( Q, state->xsize, state->xsize ) )
      hor_error ( "prediction error cov. wrong size (%d,%d) %d (hor_ekf_predict)", HOR_FATAL, Q->rows, Q->cols, state->xsize );

   /* perform state vector prediction: xp = f(x,u) */
   xp->rows = grad_fT->rows = grad_fT->cols = state->xsize;
   f_func ( state->x, u, xp, grad_fT, predict_data );

   /* perform state covariance prediction: Pp = F*P*F^T + Q,
      where F=(grad f)^T */
   hor_matq_transpose ( grad_fT, FT );
   hor_matq_add2 ( hor_matq_prod3 ( grad_fT, state->P, FT,
				    work1, work2 ),
		   Q, Pp );

   /* copy predictions into state and covariance */
   hor_matq_copy ( xp, state->x );
   hor_matq_copy ( Pp, state->P );
}

/*******************
*   Hor_Bool @hor_ekf_update ( Hor_Assoc_Label label, (filter identifier)
*                             Hor_Matrix *z,
*                             void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
*                                            Hor_Matrix *grad_hT, void *data),
*                             Hor_Matrix *R, double gate_thres,
*                             void *update_data )
*
*   Incorporates a measurement into the extended Kalman filter.
*
*   For the different EKF modes the update goes as follows:
*
*   Let the original state vectors and covariances (usually predicted by a
*   previous call to hor_(e)kf_predict()) be x0 and P0.
*
*   HOR_EKF_BASIC (basic EKF):
*
*   State innovation:      v <- z - h(x0)
*   Plant matrix:          H <- (grad h)^T evaluated at x0
*   Innovation covariance: S <- H*P0*H^T + R
*   Gain matrix:           W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   HOR_EKF_ITERATED (iterated EKF):
*
*   State innovation:      v <- z - h(x0)
*   State update:          x <- x0
*   Repeat
*      Plant matrix:          H <- (grad h)^T evaluated at x
*      Innovation covariance: S <- H*P0*H^T + R
*      Gain matrix:           W <- P0*H^T*S^-1
*      Covariance update:     P <- P0 - W*S*W^T
*      State update:          x <- x0 + P*H^T*R^-1*v
*   While no. of iterations is less than desired number (default 3)
*         and size of state change is greater than a threshold (default 0.0)
*         (size measured as the square root of sum of squares of elements
*          of the vector difference between old and new state vectors)
*
*   HOR_EKF_RECURSIVE (recursively iterated EKF):
*
*   State update:          x <- x0
*   Repeat
*      Plant matrix:          H <- (grad h)^T evaluated at x
*      Innovation covariance: S <- H*P0*H^T + R
*      Gain matrix:           W <- P0*H^T*S^-1
*      Covariance update:     P <- P0 - W*S*W^T
*      State innovation:      v <- z - h(x)
*      State update:          x <- x + P*H^T*R^-1*v - P*P0^-1*(x-x0)
*   While no. of iterations is less than desired number (default 3)
*         and size of state change is greater than a threshold (default 0.0)
*         (size measured as the square root of sum of squares of elements
*          of the vector difference between old and new state vectors)
*
*   The innovation is used to calculate a validation gate v^T*S^-1*v, which is
*   a chi-squared variable on zsize DOF, where zsize is the size of z.
*   If validation gate is too large the iteration is aborted and HOR_FALSE
*   returned, otherwise HOR_TRUE is returned.
*
*   The update_data pointer is user-defined data passed to h_func().
********************/
Hor_Bool hor_ekf_update ( Hor_Assoc_Label label, Hor_Matrix *z,
			  void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					 Hor_Matrix *grad_hT, void *data),
			  Hor_Matrix *R, double gate_thres, void *update_data )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_ekf_update)", HOR_FATAL,
		  label );

   h->rows = grad_hT->rows = z->rows;
   grad_hT->cols = state->xsize;

   switch ( state->mode )
   {
      int    i;
      double thres_2;

      case HOR_EKF_BASIC:
      /* calculate h function and the transpose of its gradient w.r.t. x,
	 both evaluated at the current state vector x (identified as x0). */
      h_func ( state->x, h, grad_hT, update_data );

      /* calculate innovation v = z - h(x0) */
      hor_matq_sub ( z, h, v );

      return (filter_update ( state, v, grad_hT, R, gate_thres));
      break;

      case HOR_EKF_ITERATED:
      i = 0;
      thres_2 = state->threshold*state->threshold;
      h_func ( state->x, h, grad_hT, update_data );
      hor_matq_sub ( z, h, v );

      hor_matq_copy ( state->x, xp );
      for(;;)
      {
	 hor_matq_transpose ( grad_hT, HT );

	 /* calculate covariance matrix Pi for current iteration */
	 hor_matq_add2 ( hor_matq_prod3 ( grad_hT, state->P, HT, work1, work2),
			 R, S );
	 hor_matq_inv ( S, work1, work2, Sinv );

	 hor_matq_prod3 ( state->P, HT, Sinv, work1, W );
	 hor_matq_transpose ( W, WT );
	 hor_matq_sub ( state->P, hor_matq_prod3 ( W, S, WT, work1, work2 ),
		        Pp );

	 /* update state, first storing it in FT */
	 hor_matq_copy ( xp, FT );
	 hor_matq_prod2 ( Pp, HT, W );
	 hor_matq_inv ( R, work1, work2, S );
	 hor_matq_add2 ( state->x, hor_matq_prod3 ( W, S, v, work1, work2 ),
			 xp );

	 if ( ++i == state->iterations ) break;
	 hor_matq_sub ( xp, FT, work1 );
	 if ( hor_vec_scalar_prod ( work1, work1 ) < thres_2 ) break;

	 h_func ( xp, h, grad_hT, update_data );
      }

      /* calculate validation gate vT*S^-1*v */
      hor_matq_transpose ( v, vT );
      hor_matq_prod3 ( vT, Sinv, v, work1, work2 );
      if ( hor_chi_2_prob ( work2->m[0][0], v->rows ) < gate_thres )
	 return HOR_FALSE;

      /* update state and covariance to final iterated values */
      hor_matq_copy ( xp, state->x );
      hor_matq_copy ( Pp, state->P );
      break;

      case HOR_EKF_RECURSIVE:
      i = 0;
      thres_2 = state->threshold*state->threshold;
      hor_matq_copy ( state->x, xp );
      hor_matq_inv ( state->P, work1, work2, Pinv );
      for(;;)
      {
	 h_func ( xp, h, grad_hT, update_data );
	 hor_matq_transpose ( grad_hT, HT );

	 /* calculate covariance matrix Pi for current iteration */
	 hor_matq_add2 ( hor_matq_prod3 ( grad_hT, state->P, HT, work1, work2),
			 R, S );
	 hor_matq_inv ( S, work1, work2, Sinv );
	 hor_matq_prod3 ( state->P, HT, Sinv, work1, W );
	 hor_matq_transpose ( W, WT );
	 hor_matq_sub ( state->P, hor_matq_prod3 ( W, S, WT, work1, work2 ),
		        Pp );

	 /* update state, first storing it in FT */
	 hor_matq_copy ( xp, FT );
	 hor_matq_prod2 ( Pp, HT, W );
	 hor_matq_inv ( R, work1, work2, S );
	 hor_matq_sub ( z, h, v );
	 hor_matq_add2 ( FT, hor_matq_prod3 ( W, S, v, work1, work2 ), xp );
	 hor_matq_sub ( FT, state->x, work1 );
	 hor_mat_decrement ( xp, hor_matq_prod3 ( Pp, Pinv, work1, work2,
						  work3 ) );

	 if ( ++i == state->iterations ) break;
	 hor_matq_sub ( xp, FT, work1 );
	 if ( hor_vec_scalar_prod ( work1, work1 ) < thres_2 ) break;
      }

      /* calculate validation gate vT*S^-1*v */
      hor_matq_transpose ( v, vT );
      hor_matq_prod3 ( vT, Sinv, v, work1, work2 );
      if ( hor_chi_2_prob ( work2->m[0][0], v->rows ) < gate_thres )
	 return HOR_FALSE;

      /* update state and covariance to final iterated values */
      hor_matq_copy ( xp, state->x );
      hor_matq_copy ( Pp, state->P );
      break;

      default:
      hor_error ( "illegal EKF mode (hor_ekf_update)", HOR_FATAL );
      break;
   }

   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_ekf_step ( Hor_Assoc_Label label, (filter identifier)
*                           void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
*                                          Hor_Matrix *f, Hor_Matrix *grad_fT,
*                                          void *data),
*                           Hor_Matrix *u, Hor_Matrix *Q, void *predict_data,
*                           Hor_Matrix *z,
*                           void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
*                                          Hor_Matrix *grad_hT, void *data),
*                           Hor_Matrix *R, double  gate_thres,
*                           void *update_data )
*
*   Iterates the extended Kalman filter for time step k+1:
*
*   State prediction:      x <- f(x,u)
*   Covariance prediction: P <- F*P*F^T + Q, where F = (grad f)^T
*   State innovation:      v <- z - h(x)
*   Innovation covariance: S <- H*P*H^T + R, where H = (grad h)^T
*   Gain matrix            W <- P*H^T*S^-1
*   State update:          x <- x + W*v
*   Covariance update:     P <- P - W*S*W^T
*
*   If there is no control vector ( usize=0 in hor_kf_init() ), NULL can
*   be passed for u.
*
*   hor_ekf_step() calls hor_ekf_predict() and hor_ekf_update().
*
*   The predict_data and update_data pointers are user-defined data passed to
*   f_func() and h_func() respectively.
********************/
Hor_Bool hor_ekf_step ( Hor_Assoc_Label label,
		        void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				       Hor_Matrix *f, Hor_Matrix *grad_fT,
				       void *data),
		        Hor_Matrix *u, Hor_Matrix *Q, void *predict_data,
		        Hor_Matrix *z,
		        void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
				       Hor_Matrix *grad_hT, void *data),
		        Hor_Matrix *R, double  gate_thres, void *update_data )
{
   hor_ekf_predict ( label, f_func, u, Q, predict_data );
   return ( hor_ekf_update ( label, z, h_func, R, gate_thres, update_data ) );
}

/*******************
*   Hor_Matrix *@hor_kf_state_vector ( Hor_Assoc_Label label )
*   Hor_Matrix *@hor_kf_covariance   ( Hor_Assoc_Label label )
*   void        @hor_kf_print_state  ( Hor_Assoc_Label label )
*
*   Functions to read internal state of specific Kalman filter.
*
*   hor_kf_state_vector() returns the current state vector x.
*   hor_kf_covariance() returns the current state covariance P.
*   hor_kf_print_state() prints the current state vector x and covariance
*                        matrix P.
********************/
Hor_Matrix *hor_kf_state_vector ( Hor_Assoc_Label label )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_kf_state_vector)", HOR_FATAL,
		  label );

   return state->x;
}

Hor_Matrix *hor_kf_covariance ( Hor_Assoc_Label label )
{
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_kf_covariance)", HOR_FATAL,
		  label );

   return state->P;
}

void hor_kf_print_state ( Hor_Assoc_Label label )
{
   int       r;
   KF_State *state;

   if ( (state = (KF_State *) hor_assoc_find ( filter_list, label )) == NULL )
      hor_error ( "filter %d not initialised (hor_kf_print_state)", HOR_FATAL,
		  label );

   hor_print ( "x = (" );
   for ( r = 0; r < state->xsize; r++ )
      hor_print ( " %f", state->x->m[r][0] );
   
   hor_print ( " )^T\n" );
   hor_mat_print ( state->P, "covariance matrix P" );
}
