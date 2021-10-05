/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"

#if 1
#define USE_LEV_MARQ
#endif

#define el(M,i,j) ((M)->m[i][j])

typedef enum { H_TYPE, F_TYPE, H_NUM_TYPE } Obs_Type;
typedef union  {
   /* for H_TYPE */
   void (*h) (Hor_Matrix *xf, int xf_type, Hor_Matrix *xd, int xd_type,
	      Hor_Matrix *y,  int  y_type, int z_type, Hor_Matrix *h,
	      Hor_Matrix *Df, Hor_Matrix *Dd, Hor_Matrix *E,
	      void *user_data, void *user_state);

   /* for F_TYPE */
   void (*F) (Hor_Matrix *xf, int xf_type, Hor_Matrix *xd, int xd_type,
	      Hor_Matrix *y,  int  y_type, Hor_Matrix *z,  int  z_type,
	      Hor_Matrix *F, Hor_Matrix *Df, Hor_Matrix *Dd, Hor_Matrix *E,
	      Hor_Matrix *Fz, void *user_data, void *user_state);

   /* for H_NUM_TYPE */
   void (*hn) (Hor_Matrix *xf, int xf_type, Hor_Matrix *xd, int xd_type,
	       Hor_Matrix *y,  int  y_type, int z_type, Hor_Matrix *h,
	       void *user_data, void *user_state);
} Obs_Func;

/* structure for remembering observations in batch mode */
typedef struct
{
   int k, z_type;
   Hor_Matrix *z, *Rinv, *N;
   Obs_Type obs_type;
   Obs_Func func;
   int F_size; /* only used for F_TYPE observation */

   void *user_data;

   /* extra fields for workspace */
   Hor_Matrix *Bd, *BdT;
} Batch_Obs;

/* following structure is a complete definition of a VSDF local state */
typedef struct
{
   int         k;
   Hor_Matrix *y;
   int         y_type;

   Hor_Matrix *z, *Rinv, *N;
   int         z_type;
   int         ysize; /* size of local state vector */
   Hor_Matrix *Ad, *Am;
   Hor_Matrix *Bf, *BfT;
   Hor_Matrix *Bd, *BdT;
   Hor_Matrix *C,  *Cinv;

   Hor_Matrix *Dd, *DdT, *v, *DfRv, *DdRv, *ERv, *CERv, *ydiff;

   Obs_Type obs_type;
   Obs_Func func;
   int F_size; /* only used for F_TYPE observation */

   void *user_data;

   /* fields to calculate residual and discard local states */
   Hor_Matrix *Af, *a_BCb, *BCB, *b, *zRz;
   int         dof;

   Hor_Matrix *y0;    /* remember in case of calling hor_vsdf_batch() */
   Hor_Matrix *T0inv; /* remember in case filter is reset */

   /* list of observations stored for batch mode */
   Hor_List zlist;

   /* residual and DOF for batch mode */
   Hor_Matrix *batch_J;
   int         batch_dof;

   /* pointer to index value */
   int *index_ptr;
} Local_State;

/* structure for unitialized local state */
typedef struct
{
   int data_frames;
   int k;

   /* list of Batch_Obs observations */
   Hor_List zlist;

   /* pointer to index value */
   int *index_ptr;
} Uninit_State;

typedef struct
{
   Hor_Bool *z_accept;

   int          xfsize;  /* size of fixed global state vector */
   Hor_Matrix  *xf;      /* fixed global state vector */
   int          xf_type; /* user-defined type of parametrization */

   int          xdsize;      /* size of dynamic global state vector */
   int          max_xdsize;  /* maximum size of dynamic global state vector */
   Hor_Matrix  *xd;          /* dynamic global state vector */
   int          xd_type;     /* user-defined type of parametrization */

   int          k;           /* time step reached by VSDF */
   int          max_n;       /* maximum number of local states */
   Local_State *local_state; /* array of local states */
   int          n;           /* number of local states */

   Uninit_State *uninit_state; /* array of unitialized local states */
   int           nu;     /* number of unitialized local states */

   int max_ysize, max_zsize;
   Hor_Matrix  *Df, *DfT, *E, *ET, *Fz, *FzT, *Rinv_v;

   Hor_Matrix  *J;     /* chi-squared residual */
   int          dof;   /* degrees-of-freedom for residual J */

   Hor_Matrix *Af; /* fixed global state vector inverse covariance */
   Hor_Matrix *Ad; /* dynamic global state vector inverse covariance */
   Hor_Matrix *Am; /* global state vector inverse covariance cross term */
   Hor_Matrix *alpha, *alpha_inv, *beta, *betaT, *gamma, *gamma_inv;
   Hor_Matrix *vxf, *xfdiff, *vxd;

   /* following fields only apply to optimal dynamic VSDF */
   Hor_Bool    use_optimal_dynamic_vsdf;
   int         disc_n;
   Hor_Matrix *Bf, *BfT, *Bd, *BdT, *BAB, *BABk, *AAA, *AAB, *BAA;

   /* remember initial state vector in case hor_vsdf_batch() is called */
   Hor_Matrix *xf0;

   /* remember initial inverse covariance in case filter is reset */
   Hor_Matrix *Pf0inv;

   /* fields for state initialisation */
   int stored; /* number of frames of stored observations */

   /* history of dynamic part of global state vector */
   Hor_List xdlist;

   /* user state variables */
   void *user_state;
} VSDF_State;

/* workspace */
static Hor_Matrix *J; /* temporary chi-squared residual */
static int max_size = 0;
static Hor_Matrix *work1, *work2, *work3, *work4, *work5, *work6, *work7;

/* big matrices are only for optimal dynamic VSDF */
static Hor_Matrix *Kinv = NULL, *big1, *big2, *big3, *BAA = NULL;

/* list of initialised VSDF's */
static Hor_Assoc_List vsdf_list = NULL;

/* currently operating VSDF */
static VSDF_State *current_vsdf = NULL;

static void free_batchz ( void *ptr )
{
   Batch_Obs *batchz = (Batch_Obs *) ptr;

   if ( batchz == NULL ) return;

   if ( batchz->Bd != NULL )
      hor_mat_free_list ( batchz->BdT, batchz->Bd, NULL );

   /* put batchz->N at end of list because it might be NULL */
   hor_mat_free_list ( batchz->Rinv, batchz->z, batchz->N, NULL );
   hor_free ( (void *) batchz );
}

static void free_local_state (Local_State *ls, Hor_Matrix *xf, Hor_Matrix *xd)
{
   hor_mat_free ( ls->batch_J );
   hor_free_list ( ls->zlist, free_batchz ); ls->zlist = NULL;

   if ( xf != NULL && xd != NULL ) hor_mat_free ( ls->Am );
   if ( xf != NULL ) hor_mat_free_list ( ls->zRz, ls->b, ls->BCB, ls->a_BCb,
					 ls->Af, ls->BfT, ls->Bf, ls->DfRv,
					 NULL );
   if ( xd != NULL ) hor_mat_free_list ( ls->BdT, ls->Bd, ls->Ad,
					 ls->DdT, ls->Dd, ls->DdRv, NULL );

   if ( ls->T0inv != NULL ) hor_mat_free ( ls->T0inv );
   hor_mat_free_list ( ls->y0, ls->ydiff, ls->CERv, ls->ERv, ls->v, ls->Cinv,
		       ls->C, ls->N, ls->Rinv, ls->z, ls->y, NULL );
}

static void free_uninit_state ( Uninit_State *us )
{
   hor_free_list ( us->zlist, free_batchz );
}

static void xd_def_free ( Hor_VSDF_XD_Def *xd_def )
{
   hor_mat_free_list ( xd_def->xd, xd_def->Ad, NULL );
   hor_free ( (void *) xd_def );
}

static void free_vsdf_state ( VSDF_State *vs )
{
   int i;

   /* free unitialized local states */
   for ( i = vs->nu-1; i >= 0; i-- )
      free_uninit_state ( &vs->uninit_state[i] );
   
   hor_free ( (void *) vs->uninit_state );

   /* free local states */
   for ( i = vs->n+vs->disc_n-1; i >= 0; i-- )
      free_local_state ( &vs->local_state[i], vs->xf, vs->xd );

   hor_free ( (void *) vs->local_state );

   hor_free_list ( vs->xdlist, (void (*)(void *)) xd_def_free );
   vs->xdlist = NULL;

   /* free matrices */
   hor_mat_free_list ( vs->J, vs->Rinv_v, vs->FzT, vs->Fz,
		       vs->ET, vs->E, NULL );

   if ( vs->xf != NULL && vs->xd != NULL )
      hor_mat_free_list ( vs->betaT, vs->beta, vs->Am, NULL );

   if ( vs->xd != NULL )
      hor_mat_free_list ( vs->vxd, vs->gamma_inv, vs->gamma, vs->Ad, vs->xd,
			  NULL );

   if ( vs->xf != NULL )
      hor_mat_free_list ( vs->Pf0inv, vs->xf0, vs->vxf, vs->xfdiff,
			  vs->alpha_inv, vs->alpha, vs->Af, vs->DfT, vs->Df,
			  vs->xf, NULL );

   hor_free ( (void *) vs->z_accept );

   if ( vs->use_optimal_dynamic_vsdf ) {
      hor_mat_free_list ( vs->BABk, vs->BAB, vs->BdT, vs->Bd, NULL );

      if ( vs->xf != NULL )
	 hor_mat_free_list ( vs->BAA, vs->AAB, vs->AAA, vs->BfT, vs->Bf,
			     NULL);
   }
}

#define SMALL_NUMBER 1.0e-10
static void fill_diagonal_small ( Hor_Matrix *M )
{
   int size = M->rows, i;

   hor_matq_zero ( M );
   for ( i = 0; i < size; i++ ) el(M,i,i) = SMALL_NUMBER;
}

#ifdef USE_LEV_MARQ
static Hor_Matrix *hor_matq_scale_diagonal ( Hor_Matrix *M, double factor )
{
   int i, size = M->rows;

   if ( !hor_mat_is_square ( M ) )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   for ( i = 0; i < size; i++ ) el(M,i,i) *= factor;
 
   return M;
}
#endif

/*******************
*   void @hor_vsdf_init ( Hor_Assoc_Label label,  (identifier for VSDF)
*                        Hor_Matrix *xf0         (initial value of fixed global
*                                                state vector)
*                        int         xf_type,    (user-defined type of
*                                                 parametrisation used in xf)
*                        Hor_Matrix *Pf0inv,     (prior covariance or xf0)
*                        Hor_Matrix *xd0,        (initial value of dynamic
*                                                 global state vector)
*                        int         xd_type,    (user-defined type of
*                                                 parametrisation used in xd)
*                        void       *user_state, (user state: can be NULL)
*                        int         max_ysize,  (maximum size of local state
*                                                 vectors)
*                        int         max_zsize,  (maximum size of observation
*                                                 vectors)
*                        int         max_n,      (maximum number of local
*                                                 states)
*                        Hor_Bool    use_optimal_dynamic_vsdf )
*
*   Initializes variable state-dimension Kalman filter with fixed global state
*   vector xf0 and associated inverse covariance Pf0inv, and dynamic global
*   state vector xd0. Multiple VSDF's in the same application are supported
*   by the label argument. hor_vsdf_init() sets the currently operating VSDF
*   to the one initialised; switching between VSDFs is achieved by
*   hor_vsdf_switch(). Either but not both of xf0 and xd0 may be NULL to
*   indicate their absence in this application. If xf0 is NULL then Pf0inv may
*   also be NULL. If not then Pf0inv may be NULL anyway to indicate zero
*   information about xf0, equivalent to Pf0inv being zero. Also given are the
*   maximum sizes of the local state vectors and observation vector, and the
*   maximum number of local states to allow for.
********************/
void hor_vsdf_init ( Hor_Assoc_Label label,
		     Hor_Matrix *xf0, int xf_type, Hor_Matrix *Pf0inv,
		     Hor_Matrix *xd0, int xd_type,
		     void *user_state, int max_ysize, int max_zsize, int max_n,
		     Hor_Bool use_optimal_dynamic_vsdf )
{
   int new_max_size, big_size = max_n*max_ysize;
   VSDF_State *vs;

   if ( max_n > HOR_VSDF_UNINIT_OFFSET )
      hor_error ( "too many local states %d (max. %d) (hor_vsdf_init)",
		  HOR_FATAL, max_n, HOR_VSDF_UNINIT_OFFSET );

   if ( (vs = (VSDF_State *) hor_assoc_find ( vsdf_list, label )) == NULL )
   {
      vs = hor_malloc_type(VSDF_State);
      vsdf_list = hor_assoc_insert ( vsdf_list, label, (void *) vs );
   }
   else
      free_vsdf_state ( vs );

   hor_errno = HORATIO_OK;
   vs->max_ysize = max_ysize;
   vs->max_zsize = max_zsize;
   vs->max_n     = max_n;

   vs->z_accept = hor_malloc_ntype ( Hor_Bool, max_n );
   vs->user_state = user_state;

   /* initialise chi-squared degrees-of freedom */
   vs->dof = 0;

   vs->xf_type = xf_type;
   if ( xf0 != NULL )
   {
      if ( xf0->cols != 1 )
	 hor_error ( "xf vector non-column (%d) (hor_vsdf_init)", HOR_FATAL,
		     xf0->cols );

      vs->xfsize = xf0->rows;
      if ( Pf0inv != NULL )
	 if ( !hor_mat_test_size ( Pf0inv, vs->xfsize, vs->xfsize ) )
	    hor_error ( "covariance matrix wrong size (%d,%d) %d (hor_vsdf_init)", HOR_FATAL, Pf0inv->rows, Pf0inv->cols, vs->xfsize );

      vs->xf  = hor_mats_copy ( xf0 );
      vs->Df  = hor_mat_alloc ( max_zsize, vs->xfsize );
      vs->DfT = hor_mat_alloc ( vs->xfsize, max_zsize );
      vs->xf0 = hor_mats_copy ( xf0 );
      if ( Pf0inv == NULL ) {
	 vs->Af = hor_mats_zero ( vs->xfsize, vs->xfsize );
	 fill_diagonal_small ( vs->Af );
	 vs->Pf0inv = NULL;
      }
      else {
	 vs->Af     = hor_mats_copy ( Pf0inv );
	 vs->Pf0inv = hor_mats_copy ( Pf0inv );
      }

      vs->alpha     = hor_mats_copy ( vs->Af );
      vs->alpha_inv = hor_mats_inv ( vs->Af );
      vs->vxf       = hor_mat_alloc ( vs->xfsize, 1 );
      vs->xfdiff    = hor_mat_alloc ( vs->xfsize, 1 );

      if ( Pf0inv == NULL ) vs->dof -= vs->xfsize;
   }
   else
   {
      vs->xfsize = 0;
      vs->xf = vs->Df = vs->DfT = vs->xf0 = vs->Af = vs->Pf0inv =
      vs->alpha = vs->alpha_inv = vs->vxf = vs->xfdiff = NULL;
   }

   vs->xd_type = xd_type;
   if ( xd0 != NULL )
   {
      if ( xd0->cols != 1 )
	 hor_error ( "xd vector non-column (%d) (hor_vsdf_init)", HOR_FATAL,
		     xd0->cols );

      vs->xdsize = xd0->rows;
      vs->max_xdsize = xd0->rsize;
      vs->xd  = hor_mats_copy ( xd0 );
      vs->Ad = hor_mat_alloc ( vs->max_xdsize, vs->max_xdsize );

      vs->gamma     = hor_mat_alloc ( vs->max_xdsize, vs->max_xdsize );
      vs->gamma_inv = hor_mat_alloc ( vs->max_xdsize, vs->max_xdsize );
      vs->vxd       = hor_mat_alloc ( vs->max_xdsize, 1 );
   }
   else
   {
      vs->xdsize = vs->max_xdsize = 0;
      vs->xd = vs->Ad = vs->gamma = vs->gamma_inv = vs->vxd = NULL;
   }

   if ( xf0 != NULL && xd0 != NULL )
   {
      vs->Am    = hor_mat_alloc (     vs->xfsize, vs->max_xdsize );
      vs->beta  = hor_mat_alloc (     vs->xfsize, vs->max_xdsize );
      vs->betaT = hor_mat_alloc ( vs->max_xdsize,     vs->xfsize );
   }
   else
      vs->Am = vs->beta = vs->betaT = NULL;

   vs->E      = hor_mat_alloc ( max_zsize, max_ysize );
   vs->ET     = hor_mat_alloc ( max_ysize, max_zsize );
   vs->Fz     = hor_mat_alloc ( max_zsize, max_zsize );
   vs->FzT    = hor_mat_alloc ( max_zsize, max_zsize );
   vs->Rinv_v = hor_mat_alloc ( max_zsize, 1 );

   /* residual calculation matrices */
   vs->J = hor_mats_zero ( 1, 1 );

   /* allocate arrays for local states */
   vs->local_state = hor_malloc_ntype ( Local_State, max_n );

   if ( vs->local_state == NULL )
      hor_error ( "local state allocation failed (hor_vsdf_init)", HOR_FATAL );

   /* initialise number of local states to zero */
   vs->n = 0;

   /* allocate array of unitialized local states */
   vs->uninit_state = hor_malloc_ntype ( Uninit_State, max_n );

   if ( vs->uninit_state == NULL )
      hor_error ("uninitialized local state allocation failed (hor_vsdf_init)",
		 HOR_FATAL);

   /* initialise number of local states to zero */
   vs->nu = 0;

   /* initialise time step */
   vs->k = 1;

   /* initialise batch list of dynamic global state vectors */
   if ( xd0 != NULL )
   {
      Hor_VSDF_XD_Def *xd_def = hor_malloc_type(Hor_VSDF_XD_Def);

      xd_def->xd      = hor_mats_copy(xd0);
      xd_def->xd_type = xd_type;
      xd_def->Ad      = NULL;
      vs->xdlist = hor_insert ( NULL, (void *) xd_def );
   }
   else vs->xdlist = NULL;

   /* re-allocate workspace if necessary */
   new_max_size = hor_max2 ( hor_max2 ( vs->xfsize, vs->max_xdsize ),
			     hor_max2 ( max_ysize, max_zsize ) );
   if ( new_max_size > max_size )
   {
      if ( max_size > 0 )
	 hor_mat_free_list ( work7, work6, work5, work4, work3, work2, work1,
			     J, NULL );

      max_size = new_max_size;
      J = hor_mat_alloc ( 1, 1 );
      work1 = hor_mat_alloc ( max_size, max_size );
      work2 = hor_mat_alloc ( max_size, max_size );
      work3 = hor_mat_alloc ( max_size, max_size );
      work4 = hor_mat_alloc ( max_size, max_size );
      work5 = hor_mat_alloc ( max_size, max_size );
      work6 = hor_mat_alloc ( max_size, max_size );
      work7 = hor_mat_alloc ( max_size, max_size );
   }

   vs->disc_n = 0;
   if ( use_optimal_dynamic_vsdf && xd0 != NULL )
   {
      vs->use_optimal_dynamic_vsdf = HOR_TRUE;
      if ( xf0 != NULL ) {
	 vs->Bf   = hor_mats_zero ( vs->xfsize,   big_size );
	 vs->BfT  = hor_mats_zero (   big_size, vs->xfsize );
	 vs->AAA  = hor_mats_zero ( vs->xfsize, vs->xfsize );
	 vs->AAB  = hor_mats_zero ( vs->xfsize,   big_size );
	 vs->BAA  = hor_mats_zero (   big_size, vs->xfsize );
      }
      else vs->Bf = vs->BfT = vs->AAA = vs->AAB = vs->BAA = NULL;

      vs->Bd   = hor_mats_zero ( vs->max_xdsize,       big_size );
      vs->BdT  = hor_mats_zero (       big_size, vs->max_xdsize );
      vs->BAB  = hor_mats_zero (       big_size,       big_size );
      vs->BABk = hor_mats_zero (       big_size,       big_size );

      if ( Kinv != NULL )
	 if ( big_size > Kinv->rows )
	 {
	    hor_mat_free_list ( big3, big2, big1, Kinv, NULL );
	    Kinv = NULL;
	 }

      if ( BAA != NULL ) {
	 hor_mat_free ( BAA );
	 BAA = NULL;
      }

      if ( xf0 != NULL ) BAA = hor_mat_alloc ( big_size, vs->xfsize );

      if ( Kinv == NULL ) {
	 Kinv = hor_mat_alloc ( big_size, big_size );
	 big1 = hor_mat_alloc ( big_size, big_size );
	 big2 = hor_mat_alloc ( big_size, big_size );
	 big3 = hor_mat_alloc ( big_size, big_size );
      }
   }
   else
   {
      vs->use_optimal_dynamic_vsdf = HOR_FALSE;
      vs->Bf = vs->BfT = vs->AAA = vs->AAB = vs->BAA = vs->Bd = vs->BdT =
      vs->BAB = vs->BABk = NULL;
   }

   vs->stored = 0;

   current_vsdf = vs;
   if ( hor_errno != HORATIO_OK ) hor_perror ( "hor_vsdf_init" );
}

/*******************
*   Hor_Bool @hor_vsdf_initialised(void)
*
*   Returns HOR_TRUE if the VSDF has been initialized, HOR_FALSE otherwise.
********************/
Hor_Bool hor_vsdf_initialised(void)
{
    return (current_vsdf != NULL);
}

/*******************
*   void @hor_vsdf_free ( Hor_Assoc_Label label )   (identifier for VSDF)
*
*   Frees data associated with VSDF.
********************/
void hor_vsdf_free ( Hor_Assoc_Label label )
{
   if ( hor_assoc_remove ( &vsdf_list, label,
			   (void (*)(void *)) free_vsdf_state )
        == HOR_ASSOC_ERROR )
      hor_error ( "illegal VSDF label (hor_vsdf_free)", HOR_FATAL );
}

/*******************
*   void @hor_vsdf_switch ( Hor_Assoc_Label label )
*
*   Switches currently operating VSDF.
********************/
void hor_vsdf_switch ( Hor_Assoc_Label label )
{
   current_vsdf = (VSDF_State *) hor_assoc_find ( vsdf_list, label );
   if ( current_vsdf == NULL )
      hor_error ( "illegal VSDF label (hor_vsdf_switch)", HOR_FATAL );
}

/*******************
*   void *@hor_vsdf_get_user_state(void)
*   void  @hor_vsdf_set_user_state ( void *new_user_state )
*
*   Functions to return and set the VSDF user state variables
********************/
void *hor_vsdf_get_user_state(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_user_state)", HOR_FATAL );

   return vs->user_state;
}

void hor_vsdf_set_user_state ( void *new_user_state )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_set_user_state)", HOR_FATAL );

   vs->user_state = new_user_state;
}

/*******************
*   int @hor_vsdf_add_state ( Hor_Matrix *y0, (initial value of new
*                                             state vector)
*                            int y_type, (user-defined representation type)
*                            Hor_Matrix *T0inv, (prior inverse covariance
*                                                of y0)
*                            int *index_ptr ) (user pointer holding index)
*
*   Adds new local state to list, providing initial values
*   of state vector and covariance. The returned value is the index of the
*   new state into the local state vector list. T0inv may be NULL, in which
*   case zero information about the accuracy of y0 is assumed, equivalent to
*   T0inv being zero.
*
*   index_ptr is a user pointer that is designated to hold the value of the
*   index (the returned value). Since the index may change when
*   hor_vsdf_remove_state() is called on another state, the updated value of
*   the index is in that case written into the pointer to keep the user
*   program consistent. If this feature is not going to be used, NULL
*   may be passed for index_ptr.
********************/
int hor_vsdf_add_state ( Hor_Matrix *y0, int y_type,
			 Hor_Matrix *T0inv, int *index_ptr )
{
   VSDF_State  *vs;
   Local_State *ls_new;
   int          ysize = y0->rows;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_add_state)", HOR_FATAL );

   if ( ysize < 1 || ysize > vs->max_ysize )
      hor_error ( "illegal vector size %d (hor_vsdf_add_state)",
		  HOR_FATAL, ysize );

   if ( y0->cols != 1 )
      hor_error ( "y vector non-column (%d) (hor_vsdf_add_state)",
		  HOR_FATAL, y0->cols);

   if ( T0inv != NULL )
      if ( !hor_mat_test_size ( T0inv, ysize, ysize ) )
	 hor_error ( "cov. matrix wrong size (%d,%d) %d (hor_vsdf_add_state)",
		     HOR_FATAL, T0inv->rows, T0inv->cols, y0->rows );

   if ( vs->n + vs->disc_n == vs->max_n )
      hor_error ( "too many local states (hor_vsdf_add_state)",
		  HOR_FATAL );

   ls_new = vs->local_state + vs->n;

   /* set time step last observation of new state to an illegal value */
   ls_new->k = -1;

   hor_errno = HORATIO_OK;

   ls_new->y       = hor_mats_copy ( y0 );
   ls_new->y_type  = y_type;
   ls_new->z       = hor_mat_alloc ( vs->max_zsize, 1 );
   ls_new->ysize   = ysize;
   ls_new->dof     = 0;

   ls_new->index_ptr = index_ptr;

   ls_new->y0 = hor_mats_copy ( y0 );
   if ( T0inv == NULL ) {
      ls_new->C = hor_mat_alloc ( ysize, ysize );
      fill_diagonal_small ( ls_new->C );
      ls_new->dof -= ysize;
      vs->dof     -= ysize;
      ls_new->T0inv = NULL;
   }
   else {
      ls_new->C     = hor_mats_copy ( T0inv );
      ls_new->T0inv = hor_mats_copy ( T0inv );
   }

   ls_new->Cinv  = hor_mats_inv ( ls_new->C );
   ls_new->v     = hor_mat_alloc ( vs->max_zsize, 1 ); /* innovation */
   ls_new->ERv   = hor_mat_alloc (         ysize, 1 );
   ls_new->CERv  = hor_mat_alloc (         ysize, 1 );
   ls_new->ydiff = hor_mat_alloc (         ysize, 1 );
   ls_new->Rinv  = hor_mat_alloc ( vs->max_zsize, vs->max_zsize );
   ls_new->N     = hor_mat_alloc ( vs->max_zsize, vs->max_zsize );

   if ( vs->xf != NULL ) {
      ls_new->Af    = hor_mats_zero ( vs->xfsize, vs->xfsize );
      ls_new->a_BCb = hor_mats_zero ( vs->xfsize,          1 );
      ls_new->BCB   = hor_mats_zero ( vs->xfsize, vs->xfsize );
      ls_new->b     = hor_mats_prod2 ( ls_new->C, ls_new->y );
      ls_new->zRz   = hor_mats_prod2 ( hor_matq_transpose ( ls_new->y, work1 ),
				       ls_new->b );
      ls_new->Bf    = hor_mats_zero ( vs->xfsize,      ysize );
      ls_new->BfT   = hor_mats_zero (      ysize, vs->xfsize );
      ls_new->DfRv  = hor_mat_alloc ( vs->xfsize,          1 );
   }
   else
      ls_new->Af = ls_new->a_BCb = ls_new->BCB = ls_new->b = ls_new->zRz =
	 ls_new->Bf = ls_new->BfT = ls_new->DfRv = NULL;

   if ( vs->xd != NULL ) {
      ls_new->Dd   = hor_mat_alloc ( vs->max_zsize,  vs->max_xdsize );
      ls_new->DdT  = hor_mat_alloc ( vs->max_xdsize,  vs->max_zsize );
      ls_new->Ad   = hor_mats_zero ( vs->max_xdsize, vs->max_xdsize );
      ls_new->Bd   = hor_mats_zero ( vs->max_xdsize,          ysize );
      ls_new->BdT  = hor_mats_zero (          ysize, vs->max_xdsize );
      ls_new->DdRv = hor_mat_alloc ( vs->max_xdsize,              1 );
   }
   else
      ls_new->Dd = ls_new->DdT = ls_new->Ad = ls_new->Bd = ls_new->BdT =
	 ls_new->DdRv = NULL;

   if ( vs->xf != NULL && vs->xd != NULL )
      ls_new->Am = hor_mats_zero ( vs->xfsize, vs->max_xdsize );
   else
      ls_new->Am = NULL;

   ls_new->zlist = NULL;
   ls_new->batch_J = hor_mats_zero ( 1, 1 );
   ls_new->batch_dof = 0;

   if ( hor_errno != HORATIO_OK )
   {
      hor_perror ( "hor_vsdf_add_state" );
      if ( index_ptr != NULL ) *index_ptr = HOR_VSDF_ERROR;
      return HOR_VSDF_ERROR;
   }

   if ( vs->use_optimal_dynamic_vsdf ) {
      int max_ysize = vs->max_ysize, p;
      int offset1 = vs->n*max_ysize, offset2 = vs->disc_n*max_ysize;

      /* shift elements of BAB matrix corresponding to "discarded" local states
	 outward to fit new local state, by swapping with zero elements
	 on the outside */
      vs->BAB->rows = vs->BAB->cols = offset1+offset2+max_ysize;
      for ( p = max_ysize-1; p >= 0; p-- )
	 hor_mat_swap_rows ( vs->BAB, offset1+p, offset1+offset2+p );

      for ( p = max_ysize-1; p >= 0; p-- )
	 hor_mat_swap_cols ( vs->BAB, offset1+p, offset1+offset2+p );

      if ( vs->xf != NULL ) {
	 /* shift elements of AAB and BAA outwards as well */
	 vs->AAB->cols = vs->BAA->rows = offset1+offset2+max_ysize;
	 for ( p = max_ysize-1; p >= 0; p-- ) {
	    hor_mat_swap_rows ( vs->AAB, offset1+p, offset1+offset2+p );
	    hor_mat_swap_cols ( vs->BAA, offset1+p, offset1+offset2+p );
	 }
      }
   }

   if ( index_ptr != NULL ) *index_ptr = vs->n;

   /* increment number of local states */
   return ( vs->n++ );
}

/*******************
*   int @hor_vsdf_addu_state ( int *index_ptr ) (pointer holding index value)
*
*   Adds a new local state to the list of uninitialized states.
*   Observations can be made using hor_vsdf_obs_?(), and the state is
*   initialised by a call to hor_vsdf_init_state(). The returned value is
*   the index into the array of unitialized states used for subsequent calls
*   to hor_vsdf_obs_h() and hor_vsdf_obs_F(). The indices for unitialized
*   local states start from  @HOR_VSDF_UNINIT_OFFSET to distinguish them
*   from the initialized local state vectors.
*
*   index_ptr is a user pointer designated to hold the value and the VSDF
*   index (the returned value). Since the index may change when
*   hor_vsdf_remove_state() or hor_vsdf_init_state() is called on
*   another state, the updated value of the index is in that case written
*   into the pointer to keep the user program consistent.
*   If this feature is not going to be used, NULL may be passed for index_ptr.
********************/
int hor_vsdf_addu_state ( int *index_ptr )
{
   VSDF_State   *vs;
   Uninit_State *us_new;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_addu_state)", HOR_FATAL );

   if ( vs->nu == vs->max_n )
      hor_error ( "too many local states (hor_vsdf_addu_state)",
		  HOR_FATAL );

   us_new = vs->uninit_state + vs->nu;

   /* set time step last observation of new state to an illegal value */
   us_new->k = -1;

   /* initialize number of measurements to zero */
   us_new->data_frames = 0;

   /* initialize list of observations to NULL */
   us_new->zlist = NULL;

   us_new->index_ptr = index_ptr;
   if ( index_ptr != NULL ) *index_ptr = HOR_VSDF_UNINIT_OFFSET + vs->nu;

   return (HOR_VSDF_UNINIT_OFFSET + vs->nu++);
}

/*******************
*   int @hor_vsdf_uninit_frames (int index) (index into list of
*                                           uninitialized local states)
*
*   Returns the number of observations made on the given uninitialized
*   local state, which can be used to decide whether enough data has been
*   gathered about it to initialize it with hor_vsdf_init_state().
********************/
int hor_vsdf_uninit_frames ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_uninit_frames)", HOR_FATAL );

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_uninit_frames)", HOR_FATAL, index, vs->nu);

   return vs->uninit_state[index].data_frames;
}

#define DELTA 0.001

/* fill measurement function h() and Jacobians Df, Dd, and E using numerical
   differentiation */
static void h_Jacob_num ( Hor_Matrix *xf, int xf_type,
			  Hor_Matrix *xd, int xd_type,
			  Hor_Matrix  *y, int  y_type, int z_size, int z_type,
			  void (*hn_func) (Hor_Matrix *xf, int xf_type,
					   Hor_Matrix *xd, int xd_type,
					   Hor_Matrix *y,  int  y_type,
					   int  z_type, Hor_Matrix *h,
					   void *user_data,
					   void *user_state),
			  Hor_Matrix *h, Hor_Matrix *Df, Hor_Matrix *Dd,
			  Hor_Matrix *E, void *user_data, void *user_state )
{
   int i, count = 0;
   double delta;

   work1->rows = work2->rows = h->rows = z_size;
   work1->cols = work2->cols = 1;
   hor_matq_zero ( h );

   /* fill in Jacobian of h() w.r.t. fixed global state vector xf */
   if ( xf != NULL && Df != NULL )
   {
      Df->rows = z_size; Df->cols = xf->rows;
      for ( i = 0; i < xf->rows; i++ )
      {
	 delta = 0.000001;
	 el(xf,i,0) += delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work1,
		   user_data, user_state );
	 hor_mat_increment ( h, work1 ); count++;
	 el(xf,i,0) -= 2.0*delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work2,
		   user_data, user_state );
	 hor_mat_increment ( h, work2 ); count++;
	 el(xf,i,0) += delta;
	 hor_matq_sub ( work1, work2, work3 );
	 hor_matq_scale ( work3, 0.5/delta );
	 hor_mat_insert ( work3, 0, 0, Df, i, 0, 1, z_size );
      }
   }

   /* fill in Jacobian of h() w.r.t. dynamic global state vector xd */
   if ( xd != NULL && Dd != NULL )
   {
      Dd->rows = z_size; Dd->cols = xd->rows;
      for ( i = 0; i < xd->rows; i++ )
      {
	 delta = 0.000001;
	 el(xd,i,0) += delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work1,
		   user_data, user_state );
	 hor_mat_increment ( h, work1 ); count++;
	 el(xd,i,0) -= 2.0*delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work2,
		   user_data, user_state );
	 hor_mat_increment ( h, work2 ); count++;
	 el(xd,i,0) += delta;
	 hor_matq_sub ( work1, work2, work3 );
	 hor_matq_scale ( work3, 0.5/delta );
	 hor_mat_insert ( work3, 0, 0, Dd, i, 0, 1, z_size );
      }
   }

   if ( y != NULL && E != NULL )
   {
      E->rows = z_size; E->cols = y->rows;
      for ( i = 0; i < y->rows; i++ )
      {
	 delta = 0.000001;
	 el(y,i,0) += delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work1,
		   user_data, user_state );
	 hor_mat_increment ( h, work1 ); count++;
	 el(y,i,0) -= 2.0*delta;
	 hn_func ( xf, xf_type, xd, xd_type, y, y_type, z_type, work2,
		   user_data, user_state );
	 hor_mat_increment ( h, work2 ); count++;
	 el(y,i,0) += delta;
	 hor_matq_sub ( work1, work2, work3 );
	 hor_matq_scale ( work3, 0.5/delta );
	 hor_mat_insert ( work3, 0, 0, E, i, 0, 1, z_size );
      }
   }

   /* rescale measurement function h() as average */
   hor_matq_scale ( h, 1.0/((double) count) );
}

static void vsdf_call_func ( Obs_Type type, Obs_Func func, Hor_Matrix *z,
			     int z_type, Hor_Matrix *v, int F_size,
			     Hor_Matrix *Rinv, Hor_Matrix *N,
		Hor_Bool Df_flag, Hor_Matrix *xf, int xf_type, Hor_Matrix *Df,
		Hor_Bool Dd_flag, Hor_Matrix *xd, int xd_type, Hor_Matrix *Dd,
		Hor_Bool  E_flag, Hor_Matrix  *y, int  y_type, Hor_Matrix  *E,
			     Hor_Matrix *Fz, Hor_Matrix *FzT,
			     void *user_data, void *user_state )
{
   int zsize = z->rows;

   switch ( type ) {
      case H_TYPE:
      v->rows = zsize;
      if ( Df_flag ) Df->rows = zsize; else Df = NULL;
      if ( Dd_flag ) { Dd->rows = zsize; Dd->cols = xd->rows; } else Dd = NULL;
      if (  E_flag ) {  E->rows = zsize;  E->cols =  y->rows; } else  E = NULL;
      func.h ( xf, xf_type, xd, xd_type, y, y_type, z_type, v, Df, Dd, E,
	       user_data, user_state );
      hor_matq_sub ( z, v, v ); /* innovation */
      break;

      case F_TYPE:
      v->rows = Fz->rows = F_size;
      Fz->cols = zsize;
      if ( Df_flag ) Df->rows = F_size; else Df = NULL;
      if ( Dd_flag ) {Dd->rows = F_size; Dd->cols = xd->rows;} else Dd = NULL;
      if (  E_flag ) { E->rows = F_size;  E->cols =  y->rows;} else  E = NULL;
      func.F ( xf, xf_type, xd, xd_type, y, y_type, z, z_type, v, Df, Dd, E,
	       Fz, user_data, user_state );
      hor_matq_transpose ( Fz, FzT );
      hor_matq_prod3 ( Fz, N, FzT, work1, work2 );
      hor_matq_inv ( work2, work1, work3, Rinv );
      hor_matq_scale ( v, -1.0 );
      break;

      case H_NUM_TYPE:
      v->rows = zsize;
      if ( Df_flag ) Df->rows = zsize; else Df = NULL;
      if ( Dd_flag ) { Dd->rows = zsize; Dd->cols = xd->rows; } else Dd = NULL;
      if (  E_flag ) {  E->rows = zsize;  E->cols =  y->rows; } else  E = NULL;
      h_Jacob_num ( xf, xf_type, xd,  xd_type, y, y_type, zsize, z_type,
		    func.hn, v, Df, Dd, E, user_data, user_state );
      hor_matq_sub ( z, v, v ); /* innovation */
      break;

      default:
      hor_error ( "illegal observation type (vsdf_call_func)", HOR_FATAL );
      break;
   }
}

/*******************
*   int @hor_vsdf_init_state ( int index, int iterations, double conf_level,
*                             Hor_Matrix *(*y0_func)(Hor_Matrix  *xf,
*                                                    int          xf_type,
*                                                    Hor_Matrix **xd,
*                                                    int         *xd_type,
*                                                    Hor_Matrix **z,
*                                                    int         *z_type,
*                                                    void **user_data, int k,
*                                                    void  *user_state,
*                                                    int   *y_type) )
*
*   Transfers a local state from the list of uninitialized states to the list
*   of initialized states. The provided y0_func() function should provide a
*   starting point for fitting the state vector to the observations previously
*   provided by calls to hor_vsdf_obs_h() and hor_vsdf_obs_F(), and y_type
*   should be set as the desired user-defined representation type for y.
*   y0_func() can also signal that it does not wish to initalize the state
*   vector (e.g. because there is not enough data) by returning NULL, in which
*   case hor_vsdf_init_state() returns the original index into the
*   unitialized states. Otherwise the returned value is the index of the new
*   local state in the local state list, created by a call to
*   hor_vsdf_add_state(). The unitialized state is removed by a call to
*   hor_vsdf_remove_state(). On any error occurring HOR_VSDF_ERROR is returned.
*
*   The least-squares fitting is iterated the given number of times, and
*   a Chi^2 test applied to the residual of the result, using the given
*   confidence level. If the test is failed then no change to the state lists
*   occurs, and HOR_VSDF_OUTLIER is returned.
*
*   After the final call to hor_vsdf_obs_?(), the batch state update
*   function hor_vsdf_batch() should be called.
********************/
int hor_vsdf_init_state ( int index, int iterations, double conf_level,
			  Hor_Matrix *(*y0_func)(Hor_Matrix  *xf, int  xf_type,
						 Hor_Matrix **xd, int *xd_type,
						 Hor_Matrix **z,  int  *z_type,
						 void       **user_data, int k,
						 void        *user_state,
						 int         *y_type) )
{
   VSDF_State   *vs;
   Uninit_State *us;
   int           it, j, k = 0, result, dof = 0, kinit;
   int          *xd_type = NULL, y_type = 0, *z_type = NULL;
   Hor_Matrix  **xd = NULL, **z, *y, *C;
   Hor_Matrix   *v, *ERv;
   void        **user_data;
   Hor_List      zlist, xdlist = NULL;
   Batch_Obs    *bz;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_init_state)", HOR_FATAL );

   if ( iterations < 1 )
      hor_error ( "at least 1 iteration (hor_vsdf_init_state)", HOR_FATAL );

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_init_state)",
		  HOR_FATAL, index, vs->nu );

   us = &vs->uninit_state[index];

   /* transfer lists of observations, dynamic state vectors and data pointers
      to arrays */
   if ( vs->xd != NULL )
   {
      xd      = hor_malloc_ntype ( Hor_Matrix *, us->data_frames );
      xd_type = hor_malloc_ntype ( int,          us->data_frames );
      xdlist = vs->xdlist;
      k = vs->k - 1;
   }

   z         = hor_malloc_ntype ( Hor_Matrix *, us->data_frames );
   z_type    = hor_malloc_ntype (          int, us->data_frames );
   user_data = hor_malloc_ntype ( void       *, us->data_frames );

   for ( zlist = us->zlist, j = us->data_frames-1; zlist != NULL;
	 zlist = zlist->next )
   {
      bz = (Batch_Obs *) zlist->contents;
      if ( vs->xd != NULL )
      {
	 Hor_VSDF_XD_Def *xd_def;

	 if ( bz->k > k )
	    hor_error ( "illegal observation k-value %d (hor_vsdf_init_state)",
			bz->k, HOR_FATAL );
      
	 while ( k != bz->k )
	 {
	    xdlist = xdlist->next;
	    k--;
	 }

	 xd_def = (Hor_VSDF_XD_Def *) xdlist->contents;
	 xd[j]      = xd_def->xd;
	 xd_type[j] = xd_def->xd_type;
      }

      z[j]         = bz->z;
      z_type[j]    = bz->z_type;
      user_data[j] = bz->user_data;
      j--;
   }

   j++;
   if ( vs->xd != NULL )
      y = y0_func ( vs->xf, vs->xf_type, xd+j, xd_type+j, z+j, z_type+j,
		    user_data+j, (kinit = us->data_frames-j),
		    vs->user_state, &y_type );
   else
      y = y0_func ( vs->xf, vs->xf_type, NULL, 0, z+j, z_type+j,
		    user_data+j, (kinit = us->data_frames-j),
		    vs->user_state, &y_type );

   hor_free_multiple ( z_type, z, NULL );

   if ( y == NULL )
   {
      if ( vs->xd != NULL ) hor_free_multiple ( xd_type, xd, NULL );
      return (HOR_VSDF_UNINIT_OFFSET+index);
   }

   hor_errno = HORATIO_OK;

   C      = hor_mat_alloc ( y->rows, y->rows );
   v      = hor_mat_alloc ( vs->max_zsize, 1 );
   ERv    = hor_mat_alloc ( y->rows, 1 );

   for ( it = 1; it <= iterations; it++ )
   {
      hor_matq_zero ( C );
      hor_matq_zero ( ERv );
      if ( it == iterations )
      {
	 hor_matq_zero ( J );
	 dof = 0;
      }

      vs->E->cols = y->rows;
      for ( zlist = us->zlist, j = us->data_frames-1; zlist != NULL;
	    zlist = zlist->next, j-- )
      {
	 Hor_Matrix *xdj = NULL;
	 int    xd_typej = 0;

	 bz = (Batch_Obs *) zlist->contents;
      
	 if ( vs->xd != NULL ) { xdj = xd[j]; xd_typej = xd_type[j]; }

	 vsdf_call_func ( bz->obs_type, bz->func, bz->z, bz->z_type, v,
			  bz->F_size, bz->Rinv, bz->N,
			  HOR_FALSE, vs->xf, vs->xf_type,  NULL,
			  HOR_FALSE,    xdj,    xd_typej,  NULL,
			  HOR_TRUE,       y,      y_type, vs->E,
			  vs->Fz, vs->FzT,
			  bz->user_data, vs->user_state );

	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_init_state(1))" ); return -1; }

	 hor_matq_transpose ( vs->E, vs->ET );
	 hor_mat_increment ( C,
		    hor_matq_prod3 ( vs->ET, bz->Rinv, vs->E, work1, work2 ) );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_init_state(2))" ); return -1; }

	 hor_matq_prod2 ( bz->Rinv, v, vs->Rinv_v );

	 /* increment residual and DOF */
	 hor_matq_transpose ( v, work1 );
	 if ( it == iterations )
	 {
	    hor_mat_increment ( J, hor_matq_prod2 (work1, vs->Rinv_v, work2) );
	    dof += bz->z->rows;
	 }

	 hor_mat_increment ( ERv, hor_matq_prod2(vs->ET, vs->Rinv_v, work1) );
      }

      hor_matq_inv ( C, work1, work2, work3 );
      hor_mat_increment ( y, hor_matq_prod2 ( work3, ERv, work1 ) );

      if ( it == iterations )
      {
	 /* decrement residual and DOF */
	 hor_matq_transpose ( work1, work2 );
	 hor_mat_decrement ( J,
			     hor_matq_prod3 (work2, C, work1, work3, work4) );
	 dof -= y->rows;
      }
   }

   if ( hor_chi_2_prob ( el(J,0,0), dof ) < conf_level )
   {
      hor_vsdf_remove_state ( HOR_VSDF_UNINIT_OFFSET+index, HOR_FALSE );
      result = HOR_VSDF_OUTLIER;
   }
   else
   {
      int *index_ptr = us->index_ptr;

      hor_vsdf_remove_state ( HOR_VSDF_UNINIT_OFFSET+index, HOR_TRUE);
      result = hor_vsdf_add_state ( y, y_type, C, index_ptr );
      vs->local_state[result].dof = dof;
      vs->local_state[result].user_data = user_data[kinit-1];
   }

   hor_mat_free_list ( ERv, v, C, y, NULL );
   if ( vs->xd != NULL )
      hor_free_multiple ( xd_type, xd, NULL );

   hor_free ( user_data );
   return result;
}

static void hor_vsdf_removeu_state ( VSDF_State *vs, int index,
				     Hor_Bool keep_data )
{
   int *index_ptr;

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error("illegal local state index %d (n=%d) (hor_vsdf_removeu_state)",
		HOR_FATAL, index, vs->nu);

   /* adjust index of state shifted from position vs->n-1 to position index */
   index_ptr = vs->uninit_state[index].index_ptr;
   if ( index_ptr != NULL )
      if ( keep_data ) *index_ptr = HOR_VSDF_NOT_PRESENT;
      else             *index_ptr = HOR_VSDF_OUTLIER;

   free_uninit_state ( &vs->uninit_state[index] );
   vs->uninit_state[index] = vs->uninit_state[vs->nu-1];

   /* adjust index of state shifted from position vs->nu-1 to
      position index */
   index_ptr = vs->uninit_state[index].index_ptr;
   if ( index != vs->nu-1 && index_ptr != NULL )
      *index_ptr = index+HOR_VSDF_UNINIT_OFFSET;

   vs->nu--;
}

/* put new alpha into work1, new alpha_inv into work2, new xf into vs->vxf,
   residual into J, (Af-BC^-1B^T)xf into work4 and state change into
   vs->xfdiff. */
static Hor_Bool calc_changes ( VSDF_State *vs, Local_State *ls )
{
   hor_matq_sub ( vs->alpha, ls->Af, work1 );
   hor_mat_increment ( work1, ls->BCB );
   hor_matq_inv ( work1, work3, work4, work2 );
   if ( hor_errno != HORATIO_OK ) {
      hor_perror ( "(calc_changes)" );
      return HOR_FALSE;
   }

   hor_matq_prod2 ( ls->Af, vs->xf, work4 );
   hor_mat_decrement ( work4, hor_matq_prod2 ( ls->BCB, vs->xf, work3 ) );
   hor_matq_sub ( work4, ls->a_BCb, vs->vxf );
   hor_matq_add2 ( vs->xf, hor_matq_prod2 ( work2, vs->vxf, work3 ), work5 );
   hor_matq_copy ( work3, vs->xfdiff );
   hor_matq_copy ( work5, vs->vxf );

   /* calculate residual difference */
   hor_matq_copy ( ls->zRz, J );
   hor_matq_transpose ( vs->vxf, work7 );
   hor_mat_increment ( J, hor_matq_prod2 ( work7, work4, work5 ) );
   hor_matq_transpose ( vs->xf, work5 );
   hor_mat_decrement ( J, hor_matq_prod2 (hor_matq_add2(work5, work7, work6),
					  ls->a_BCb, work5) );
   hor_matq_transpose ( ls->b, work5 );
   hor_mat_decrement ( J, hor_matq_prod3 ( work5, ls->Cinv, ls->b,
					   work6, work7 ) );
   return HOR_TRUE;
}

/*******************
*   void @hor_vsdf_remove_state ( int index, (index into local state list)
*                                 Hor_Bool keep_data )
*
*   Discards local state with given index into state list.
*   Note that the programmer is expected to maintain knowledge of the
*   indices, given that the last local state in the list replaces the
*   discarded one to fill the gap in the list. It is done this way to
*   avoid any searching to find the desired state.
*
*   If keep_data is HOR_TRUE, the effect of the observations for the removed
*   state are retained and the contents of the index pointer for the removed
*   state is set to HOR_VSDF_NOT_PRESENT, otherwise the index pointer contents
*   is set to HOR_VSDF_OUTLIER and all trace of the removed state is deleted.
*   It only has a material effect on the VSDF in the case of no dynamic part
*   to the global state vector.
*
*   An index >= HOR_VSDF_UNINIT_OFFSET refers to an unitialized state.
********************/
void hor_vsdf_remove_state ( int index, Hor_Bool keep_data )
{
   VSDF_State  *vs;
   Local_State *ls, *ls2, temp;
   int          i;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_remove_state)", HOR_FATAL );

   if ( index >= HOR_VSDF_UNINIT_OFFSET )
   {
      hor_vsdf_removeu_state ( vs, index, keep_data );
      return;
   }

   if ( index < 0 || index >= vs->n )
      hor_error ("illegal local state index %d (n=%d) (hor_vsdf_remove_state)",
		 HOR_FATAL, index, vs->n);

   ls = &vs->local_state[index];
   if ( vs->xf != NULL && keep_data )
      hor_mat_decrement ( vs->Af, ls->BCB );

   if ( vs->xf != NULL && vs->xd == NULL && !keep_data )
   {
      /* put new alpha into work1, new alpha_inv into work2, new xf into
	 vs->vxf, residual into J, (Af-BC^-1B^T)xf into work4 and
	 state change into vs->xfdiff */
      if ( !calc_changes ( vs, ls ) )
	 hor_error ( "aborting (hor_vsdf_remove_state)", HOR_FATAL );

      hor_matq_copy ( vs->vxf, vs->xf );
      hor_matq_copy ( work1, vs->alpha );
      hor_matq_copy ( work2, vs->alpha_inv );
      hor_mat_decrement ( vs->Af, ls->Af );
      hor_mat_decrement ( vs->J, J );
      vs->dof -= ls->dof;

      for ( i = 0, ls2 = vs->local_state; i < vs->n; i++, ls2++ )
	 if ( i != index )
	    hor_mat_decrement ( ls2->y,
		hor_matq_prod3(ls2->Cinv, ls2->BfT, vs->xfdiff, work2, work3));
   }

   if ( ls->index_ptr != NULL ) /* reset contents of index pointer */
      if ( keep_data ) *ls->index_ptr = HOR_VSDF_NOT_PRESENT;
      else             *ls->index_ptr = HOR_VSDF_OUTLIER;

   if ( vs->use_optimal_dynamic_vsdf )
   {
      temp = *ls;
      *ls = vs->local_state[vs->n-1];
      vs->local_state[vs->n-1] = temp;
   }
   else
   {
      free_local_state ( &vs->local_state[index], vs->xf, vs->xd );
      vs->local_state[index] = vs->local_state[vs->n-1];
   }

   /* adjust index of state shifted from position vs->n-1 to position index */
   if ( index != vs->n-1 && ls->index_ptr != NULL ) *ls->index_ptr = index;

   vs->n--;
   if ( vs->use_optimal_dynamic_vsdf ) {
      int max_ysize = vs->max_ysize, p;
      int offset1 = index*max_ysize, offset2 = vs->n*max_ysize;

      vs->disc_n++;
      if ( index == vs->n ) return;

      /* swap around rows and columns of BAB matrix corresponding to the
	 deleted and last undeleted local state */
      for ( p = max_ysize - 1; p >= 0; p++ )
	 hor_mat_swap_rows ( vs->BAB, offset1+p, offset2+p );

      for ( p = max_ysize - 1; p >= 0; p++ )
	 hor_mat_swap_cols ( vs->BAB, offset1+p, offset2+p );

      if ( vs->xf != NULL ) /* do similar swaps for AAB and BAA */
	 for ( p = max_ysize - 1; p >= 0; p++ ) {
	    hor_mat_swap_cols ( vs->AAB, offset1+p, offset2+p );
	    hor_mat_swap_rows ( vs->BAA, offset1+p, offset2+p );
	 }
   }
}

/*******************
*   Hor_Bool @hor_vsdf_test_state ( int index, (index into local state list)
*                                   double conf_level ) (confidence level)
*
*   Tests the local state residuals using the Chi^2 test. If the test is
*   failed at the given confidence level then the local state is removed.
*   HOR_TRUE is returned if the Chi^2  test is passed, HOR_FALSE otherwise.
********************/
Hor_Bool hor_vsdf_test_state ( int index, double conf_level )
{
   VSDF_State  *vs;
   Local_State *ls, *ls2;
   int          i;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_test_state)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_test_state)", HOR_FATAL, index, vs->n );

   if ( vs->xf == NULL )
   {
      hor_warning ( "no fixed global state (hor_vsdf_test_state)" );
      return HOR_TRUE;
   }

   if ( vs->xd != NULL )
   {
      hor_warning ( "can't test dynamic VSDF states (hor_vsdf_test_state)" );
      return HOR_TRUE;
   }

   ls = &vs->local_state[index];
   hor_errno = HORATIO_OK;

   /* put new alpha into work1, new alpha_inv into work2, new xf into
      vs->vxf, residual into J, (Af-BC^-1B^T)xf into work4 and
      state change into vs->xfdiff */
   if ( !calc_changes ( vs, ls ) ) return HOR_FALSE;

   if ( hor_chi_2_prob ( el(J,0,0), ls->dof ) < conf_level )
   {
      int *index_ptr = ls->index_ptr;

      hor_matq_copy ( vs->vxf, vs->xf );
      hor_matq_copy ( work1, vs->alpha );
      hor_matq_copy ( work2, vs->alpha_inv );
      hor_mat_decrement ( vs->Af, ls->Af );
      hor_mat_decrement ( vs->J, J );
      vs->dof -= ls->dof;

      for ( i = 0, ls2 = vs->local_state; i < vs->n; i++, ls2++ )
	 if ( i != index )
	    hor_mat_decrement ( ls2->y,
		hor_matq_prod3(ls2->Cinv, ls2->BfT, vs->xfdiff, work2, work3));

      hor_mat_increment ( vs->Af, ls->BCB ); /* this will be undone by
						hor_vsdf_remove_state(), a bit
						of a hack I know! */
      hor_vsdf_remove_state ( index, HOR_TRUE );
      if ( index_ptr != NULL ) *index_ptr = HOR_VSDF_OUTLIER;
      return HOR_FALSE;
   }

   return HOR_TRUE;
}

/*******************
*   void @hor_vsdf_obs_h(int index,          (index into local state list)
*                       Hor_Matrix *z,      (measurement vector)
*                       int         z_type, (user-defined type of z)
*                       Hor_Matrix *Rinv,   (measurement inverse covariance)
*                       void (*h_func)(Hor_Matrix *xf,(fixed global state)
*                                      int xf_type,   (user defined type of xf)
*                                      Hor_Matrix *xd,(dynamic global st.)
*                                      int xd_type,   (user defined type of xd)
*                                      Hor_Matrix *y, (local state)
*                                      int  y_type,   (user defined type of y)
*                                      int  z_type,   (user defined type of z)
*                                      Hor_Matrix *h, (measurement func.)
*                                      Hor_Matrix *Df,(gradient of h w.r.t. xf)
*                                      Hor_Matrix *Dd,(gradient of h w.r.t. xd)
*                                      Hor_Matrix *E, (gradient of h w.r.t. y)
*                                      void *user_data,  (user pointer)
*                                      void *user_state),(user state variables)
*                       void *user_data ) (user pointer passed into h_func)
*
*   void @hor_vsdf_obs_F(int index,          (index into local state list)
*                       Hor_Matrix *z,      (measurement vector)
*                       int         z_type, (user-defined type of z)
*                       Hor_Matrix *N,      (covariance of z)
*                       int         F_size, (size of F vector)
*                       void (*F_func)(Hor_Matrix *xf,(fixed global state)
*                                      int xf_type,   (user defined type of xf)
*                                      Hor_Matrix *xd,(dynamic global st.)
*                                      int xd_type,   (user defined type of xd)
*                                      Hor_Matrix *y, (local state)
*                                      int  y_type,   (user defined type of y)
*                                      Hor_Matrix *z, (observation vector)
*                                      int  z_type,   (user defined type of z)
*                                      Hor_Matrix *F, (measurement func.)
*                                      Hor_Matrix *Df,(gradient of h w.r.t. xf)
*                                      Hor_Matrix *Dd,(gradient of h w.r.t. xd)
*                                      Hor_Matrix *E, (gradient of h w.r.t. y)
*                                      Hor_Matrix *Fz,(gradient of h w.r.t. z)
*                                      void *user_data,  (user pointer)
*                                      void *user_state),(user state variables)
*                       void *user_data ) (user pointer passed into F_func)
*
*   void @hor_vsdf_obs_h_num ( int index, Hor_Matrix *z, int z_type,
*                             Hor_Matrix *Rinv,
*                             void (*hn_func) (Hor_Matrix *xf, int xf_type,
*                                              Hor_Matrix *xd, int xd_type,
*                                              Hor_Matrix *y,  int  y_type,
*                                              int  z_type, Hor_Matrix *h,
*                                              void *user_data,
*                                              void *user_state),
*                             void *user_data ) (user pointer passed
*                                                into hn_func)
*
*   Functions to incorporate a single observation into the VSDF.
*   If the measurement equation can be written as
*
*   z = h(xf,xd,y) + w  (covariance R)                                 (1)
*
*   then hor_vsdf_obs_h() can be used. z and R^-1 are provided, along with
*   a function h_func to fill in h and its gradients Df, Dd, E, evaluated at
*   xf, xd, y. Note that h_func is called inside hor_vsdf_update().
*
*   The most general form for the state equations is
*
*   F(xf,xd,y,z) = 0.
*
*   If z can be disentangled from F in the form of equation 1, then the h()
*   form can be used and linearised. Otherwise, we use F() directly.
*   This is slower in general, so the hor_vsdf_obs_h() should be used if
*   possible. The general F() form is implemented by hor_vsdf_obs_F().
*   Both the sizes of F and z must be <= the max_z_size argument
*   of hor_vsdf_init(). The size of F (the argument F_size) must be <= the
*   size of z.
*
*   When the VSDF is run with a dynamic part to the global state vector,
*   some calls to h_func/F_func will not require Df, Dd and E to be calculated.
*   This is signalled by the VSDF module setting Df, Dd and E to NULL,
*   and should be tested for inside h_func. If there is no static part to the
*   global state vector then the test on D is redundant.
*
*   An index >= HOR_VSDF_UNINIT_OFFSET refers to an unitialized state.
********************/
static void hor_vsdf_obsu_h ( VSDF_State *vs, int index,
			      Hor_Matrix *z, int z_type, Hor_Matrix *Rinv,
			      void (*h_func) (Hor_Matrix *xf, int xf_type,
					      Hor_Matrix *xd, int xd_type,
					      Hor_Matrix *y,  int  y_type,
					      int z_type, Hor_Matrix *h,
					      Hor_Matrix *Df, Hor_Matrix *Dd,
					      Hor_Matrix *E,
					      void *user_data,
					      void *user_state),
			      void *user_data )
{
   Uninit_State *us;
   Batch_Obs    *bz;

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error ("illegal local state index %d (n=%d) (hor_vsdf_obsu_h)",
		 HOR_FATAL, index, vs->n);

   us = &vs->uninit_state[index];

   /* check whether an observation has already been made on this local state */
   if ( us->k == vs->k )
      hor_error("observation already made on local state %d (hor_vsdf_obsu_h)",
		HOR_FATAL, index);

   /* set time step of observation */
   us->k = vs->k;

   hor_errno = HORATIO_OK;

   /* add new observation to the list of observations for this local state */
   bz = hor_malloc_type(Batch_Obs);
   bz->k        = vs->k;
   bz->z        = hor_mats_copy ( z );
   bz->z_type   = z_type;
   bz->Rinv     = hor_mats_copy ( Rinv );
   bz->N        = NULL;
   bz->obs_type = H_TYPE;
   bz->func.h   = h_func;
   bz->F_size   = 0;
   bz->user_data = user_data;
   bz->Bd = bz->BdT = NULL;
   us->zlist = hor_insert ( us->zlist, (void *) bz );

   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obsu_h" ); return; }

   /* increment number of observations */
   us->data_frames++;
}

void hor_vsdf_obs_h ( int index, Hor_Matrix *z, int z_type, Hor_Matrix *Rinv,
		      void (*h_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type, int z_type,
				      Hor_Matrix *h,  Hor_Matrix *Df,
				      Hor_Matrix *Dd, Hor_Matrix *E,
				      void *user_data, void *user_state),
		      void *user_data )
{
   VSDF_State  *vs;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_obs_h)", HOR_FATAL );

   if ( index >= HOR_VSDF_UNINIT_OFFSET )
   {
      hor_vsdf_obsu_h ( vs, index, z, z_type, Rinv, h_func, user_data );
      return;
   }

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_obs_h)",
		  HOR_FATAL, index, vs->n );

   ls = &vs->local_state[index];

   /* check whether an observation has already been made on this local state */
   if ( ls->k == vs->k )
      hor_error ("observation already made on local state %d (hor_vsdf_obs_h)",
		 HOR_FATAL, index);

   /* set time step of observation */
   ls->k = vs->k;

   hor_errno = HORATIO_OK;

   hor_matq_copy ( z, ls->z );
   ls->z_type = z_type;
   hor_matq_copy ( Rinv, ls->Rinv );
   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obs_h" ); return; }

   ls->obs_type = H_TYPE;
   ls->func.h   = h_func;
   ls->F_size   = 0;
   ls->user_data = user_data;
}

static void hor_vsdf_obsu_F ( VSDF_State *vs, int index, Hor_Matrix *z,
			      int z_type, Hor_Matrix *N, int F_size,
			      void (*F_func) (Hor_Matrix *xf, int xf_type,
					      Hor_Matrix *xd, int xd_type,
					      Hor_Matrix *y,  int  y_type,
					      Hor_Matrix *z,  int  z_type,
					      Hor_Matrix *F,  Hor_Matrix *Df,
					      Hor_Matrix *Dd, Hor_Matrix *E,
					      Hor_Matrix *Fz,
					      void *user_data,
					      void *user_state),
			      void *user_data )
{
   Uninit_State *us;
   Batch_Obs    *bz;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_obsu_F)", HOR_FATAL );

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error ("illegal local state index %d (n=%d) (hor_vsdf_obsu_F)",
		 HOR_FATAL, index, vs->n);

   us = &vs->uninit_state[index];

   /* check whether an observation has already been made on this local state */
   if ( us->k == vs->k )
      hor_error("observation already made on local state %d (hor_vsdf_obsu_F)",
		HOR_FATAL, index);

   /* set time step of observation */
   us->k = vs->k;

   hor_errno = HORATIO_OK;

   /* add new observation to the list of observations for this local state */
   bz = hor_malloc_type(Batch_Obs);
   bz->k        = vs->k;
   bz->z        = hor_mats_copy ( z );
   bz->z_type   = z_type;
   bz->N        = hor_mats_copy ( N );
   bz->Rinv     = hor_mat_alloc ( N->rows, N->cols );
   bz->obs_type = F_TYPE;
   bz->func.F   = F_func;
   bz->F_size   = F_size;
   bz->user_data = user_data;
   bz->Bd = bz->BdT = NULL;
   us->zlist = hor_insert ( us->zlist, (void *) bz );

   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obsu_F" ); return; }

   /* increment number of observations */
   us->data_frames++;
}

void hor_vsdf_obs_F ( int index, Hor_Matrix *z, int z_type,
		      Hor_Matrix *N, int F_size,
		      void (*F_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type,
				      Hor_Matrix *z,  int  z_type,
				      Hor_Matrix *F,  Hor_Matrix *Df,
				      Hor_Matrix *Dd, Hor_Matrix *E,
				      Hor_Matrix *Fz,
				      void *user_data, void *user_state),
		      void *user_data )
{
   VSDF_State  *vs;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_obs_F)", HOR_FATAL );

   if ( index >= HOR_VSDF_UNINIT_OFFSET )
   {
      hor_vsdf_obsu_F ( vs, index, z, z_type, N, F_size, F_func, user_data );
      return;
   }

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_obs_F)",
		  HOR_FATAL, index, vs->n );

   ls = &vs->local_state[index];

   /* check whether an observation has already been made on this local state */
   if ( ls->k == vs->k )
      hor_error ("observation already made on local state %d (hor_vsdf_obs_F)",
		 HOR_FATAL, index);

   if ( F_size > z->rows )
      hor_error ("size of F (%d) greater than size of z (%d) (hor_vsdf_obs_F)",
		 HOR_FATAL, F_size, z->rows);

   /* set time step of observation */
   ls->k = vs->k;

   hor_errno = HORATIO_OK;

   /* update state vector and covariance */

   /* increment inverse covariance and other matrices */
   hor_matq_copy ( z, ls->z );
   ls->z_type = z_type;
   hor_matq_copy ( N, ls->N );
   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obs_F" ); return; }

   ls->obs_type = F_TYPE;
   ls->func.F   = F_func;
   ls->F_size   = F_size;
   ls->user_data = user_data;
}

static void hor_vsdf_obsu_h_num ( VSDF_State *vs, int index,
				  Hor_Matrix *z, int z_type, Hor_Matrix *Rinv,
				  void (*hn_func) (Hor_Matrix *xf, int xf_type,
						   Hor_Matrix *xd, int xd_type,
						   Hor_Matrix *y,  int  y_type,
						   int z_type, Hor_Matrix *h,
						   void *user_data,
						   void *user_state),
				  void *user_data )
{
   Uninit_State *us;
   Batch_Obs    *bz;

   index -= HOR_VSDF_UNINIT_OFFSET;
   if ( index < 0 || index >= vs->nu )
      hor_error ("illegal local state index %d (n=%d) (hor_vsdf_obsu_h_num)",
		 HOR_FATAL, index, vs->n);

   us = &vs->uninit_state[index];

   /* check whether an observation has already been made on this local state */
   if ( us->k == vs->k )
      hor_error("observation already made on local state %d (hor_vsdf_obsu_h_num)", HOR_FATAL, index);

   /* set time step of observation */
   us->k = vs->k;

   hor_errno = HORATIO_OK;

   /* add new observation to the list of observations for this local state */
   bz = hor_malloc_type(Batch_Obs);
   bz->k        = vs->k;
   bz->z        = hor_mats_copy ( z );
   bz->z_type   = z_type;
   bz->Rinv     = hor_mats_copy ( Rinv );
   bz->N        = NULL;
   bz->obs_type = H_NUM_TYPE;
   bz->func.hn  = hn_func;
   bz->user_data = user_data;
   bz->Bd = bz->BdT = NULL;
   us->zlist = hor_insert ( us->zlist, (void *) bz );

   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obsu_h_num" ); return; }

   /* increment number of observations */
   us->data_frames++;
}

void hor_vsdf_obs_h_num ( int index, Hor_Matrix *z, int z_type,
			  Hor_Matrix *Rinv,
			  void (*hn_func) (Hor_Matrix *xf, int xf_type,
					   Hor_Matrix *xd, int xd_type,
					   Hor_Matrix *y,  int  y_type,
					   int  z_type, Hor_Matrix *h,
					   void *user_data, void *user_state),
			  void *user_data )
{
   VSDF_State  *vs;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_obs_h_num)", HOR_FATAL );

   if ( index >= HOR_VSDF_UNINIT_OFFSET )
   {
      hor_vsdf_obsu_h_num ( vs, index, z, z_type, Rinv, hn_func, user_data );
      return;
   }

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (n=%d) (hor_vsdf_obs_h_num)",
		  HOR_FATAL, index, vs->n );

   ls = &vs->local_state[index];

   /* check whether an observation has already been made on this local state */
   if ( ls->k == vs->k )
      hor_error ("observation already made on local state %d (hor_vsdf_obs_h_num)", HOR_FATAL, index);

   /* set time step of observation */
   ls->k = vs->k;

   hor_errno = HORATIO_OK;

   hor_matq_copy ( z, ls->z );
   ls->z_type = z_type;
   hor_matq_copy ( Rinv, ls->Rinv );
   if ( hor_errno != HORATIO_OK )
   { hor_perror ( "hor_vsdf_obs_h" ); return; }

   ls->obs_type  = H_NUM_TYPE;
   ls->func.hn   = hn_func;
   ls->user_data = user_data;
}

/*******************
*   void @hor_vsdf_store(void)
*
*   This marks the start of a new frame of data while in batch mode.
*   More calls to hor_vsdf_obs_?() will follow. The argument is the prior
*   estimate of the dynamic global state vector for the new frame.
*   Batch mode is terminated by calling hor_vsdf_batch().
********************/
void hor_vsdf_store(void)
{
   VSDF_State  *vs;
   Local_State *ls;
   int          i;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_store)", HOR_FATAL );

   if ( vs->k > 1 && vs->stored == 0 )
      hor_error ( "illegal call (hor_vsdf_store())", HOR_FATAL );

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      if ( ls->k == vs->k )
      {
	 Batch_Obs *batchz = hor_malloc_type(Batch_Obs);

	 batchz->k = vs->k;
	 batchz->user_data = ls->user_data;
	 switch ( ls->obs_type )
	 {
	    case H_TYPE:
	    batchz->obs_type = H_TYPE;
	    batchz->z        = hor_mats_copy ( ls->z );
	    batchz->z_type   = ls->z_type;
	    batchz->Rinv     = hor_mats_copy ( ls->Rinv );
	    batchz->N        = NULL;
	    batchz->func.h   = ls->func.h;
	    break;

	    case F_TYPE:
	    batchz->obs_type = F_TYPE;
	    batchz->z        = hor_mats_copy ( ls->z );
	    batchz->z_type   = ls->z_type;
	    batchz->Rinv     = hor_mat_alloc ( ls->N->rows, ls->N->cols );
	    batchz->N        = hor_mats_copy ( ls->N );
	    batchz->func.F   = ls->func.F;
	    batchz->F_size   = ls->F_size;
	    break;

	    case H_NUM_TYPE:
	    batchz->obs_type = H_NUM_TYPE;
	    batchz->z        = hor_mats_copy ( ls->z );
	    batchz->z_type   = ls->z_type;
	    batchz->Rinv     = hor_mats_copy ( ls->Rinv );
	    batchz->N        = NULL;
	    batchz->func.hn  = ls->func.hn;
	    break;

	    default:
	    hor_error ( "illegal obs. type (hor_vsdf_store)", HOR_FATAL );
	    break;
	 }

	 batchz->Bd = batchz->BdT = NULL;
	 ls->zlist = hor_insert ( ls->zlist, (void *) batchz );
      }

   /* increment number of stored frames */
   vs->stored++;

   /* increment time-step */
   vs->k++;
}

/*******************
*   Hor_Bool @hor_vsdf_batch (Hor_Bool (*init_func)(Hor_Matrix  *xf,
*                                                  int         *xf_type,
*                                                  Hor_Matrix **xd,
*                                                  int         *xd_type, int k,
*                                                  Hor_Matrix **y,
*                                                  int        *y_type,  int n,
*                                                  Hor_Matrix **yu,
*                                                  int        *yu_type, int nu,
*                                                  Hor_Matrix ***zu,
*                                                  int         **zu_type,
*                                                  void       ***data_u,
*                                                  void *user_data,
*                                                  void *user_state),
*                            int iterations1, int iterations2,
*                            double conf_level, void *user_data )
*
*   Initializes the global dynamic state vector and any unitialized local
*   state vectors using observations made for recent frames and stored using
*   hor_vsdf_store(). Recursive least-squares iterations are applied using
*   the result of the provided init_func() as the starting point.
*
*   init_func() takes as input the fixed global state vector xf (which may
*   be NULL if none is present), the n initialized local state vectors y,
*   the initial dynamic global state vector xd[0], the measurement vectors zu
*   for the nu unitialized local state vectors, where zu[i][j] is the
*   measurement vector of the i'th unitialized local state in the j'th frame,
*   i = 0...nu-1, j = 0...k-1, and also given the data pointers data_u[i][j]
*   originally passed into hor_vsdf_obs_?() along with the zu's.
*   Any of the zu[i][j] vectors may be NULL to denote a missing measurement.
*   The user pointer argument "user_data" is passed to init_func().
*   init_func() should fill in the (pre-allocated) vectors xd[j] for j=1...k-1
*   (if a dynamic global state vector is being used) and initialize the local
*   state vectors yu[i]. i = 0...nu-1, whose values are passed in sa NULL.
*   If a given local state vector yu[i] is not to be initialized, its value
*   should be left as NULL. The initial dynamic global state vector xd[0]
*   should not be modified. It serves to fix the frame of reference for the
*   rest of the state vector. The return value of init_func signfies whether
*   the initialization has been successfully achieved (HOR_TRUE) or not
*   (HOR_FALSE). If HOR_FALSE is returned then hor_vsdf_batch() aborts and
*   returns NULL.
*
*   Once the state vectors have been initialized, the given number of
*   least-squares iterations (iterations1) are applied. The given confidence
*   level is used to accept or reject local states depending on whether or not
*   their contribution to the residual satisfies a chi^2 test. Once this
*   outlier rejection stage has been completed another series of least-squares
*   iterations are applied (iterations2 of them).
*   hor_vsdf_batch() returns a boolean value indicating whether the VSDF has
*   been successfully initialized (HOR_TRUE) for not (HOR_FALSE).
*
*   hor_vsdf_batch() should be called directly after calling hor_vsdf_store()
*   for the last frame of observations to be processed.
********************/
#ifdef USE_LEV_MARQ
#define LAMBDA_START   0.001
#define LAMBDA_FACTOR 10.0
#endif

Hor_Bool hor_vsdf_batch ( Hor_Bool (*init_func)(Hor_Matrix  *xf, int *xf_type,
						Hor_Matrix **xd,
						int         *xd_type, int k,
						Hor_Matrix **y,
						int         *y_type, int n,
						Hor_Matrix **yu,
						int         *yu_type, int nu,
						Hor_Matrix ***zu,
						int         **zu_type,
						void       ***data_u,
						void *user_data,
						void *user_state),
			  int iterations1, int iterations2,
			  double conf_level, void *user_data )
{
   VSDF_State  *vs;
   Local_State *ls, *ls2;
   Hor_Bool     update_fixed, update_dynamic, update_both;
   int          i, j, it, max_ysize, xdsize = 0, ysize = 0, p, *xd_type = NULL;
   int          iterations = iterations1 + iterations2 + 1, nu;
   Hor_Matrix **xd = NULL, **Ad = NULL, **AmT = NULL;
   Hor_Matrix  *vxdy = NULL, *xddiff = NULL, *L = NULL, *Linv = NULL;
   Hor_Matrix  *DdRv = NULL, *xffac = NULL, *xffacT = NULL;
   Hor_Matrix  *temp1 = NULL, *temp2 = NULL;
   Hor_List     list,     list2;
   Batch_Obs   *bz,      *bz2;
   Hor_Bool     y_first = HOR_FALSE;
   int         *xdoffset = NULL, *yoffset = NULL;
#ifdef USE_LEV_MARQ
   double       lambda, old_resid = 0.0;
#endif

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_batch)", HOR_FATAL );

   if ( iterations1 < 0 || (iterations2 < 0 && conf_level > 0.0) )
      hor_error ( "at least 1 iteration (hor_vsdf_batch)", HOR_FATAL );

   if ( vs->xf != NULL ) update_fixed = HOR_TRUE;
   else                  update_fixed = HOR_FALSE;

   if ( vs->xd != NULL ) update_dynamic = HOR_TRUE;
   else                  update_dynamic = HOR_FALSE;

   if ( update_fixed && update_dynamic ) update_both = HOR_TRUE;
   else                                  update_both = HOR_FALSE;

   if ( vs->stored == 0 || (update_dynamic && vs->stored == 1) )
      hor_error ( "not enough stored data (hor_vsdf_batch)", HOR_FATAL );

   /* create array of dynamic global state vectors */
   if ( update_dynamic )
   {
      xd       = hor_malloc_ntype ( Hor_Matrix *, vs->stored );
      xd_type  = hor_malloc_ntype (          int, vs->stored );
      for ( j = 0; j < vs->stored; j++ )
      {
	 xd[j] = hor_mats_copy ( vs->xd );
	 xd_type[j] = vs->xd_type;
      }
   }
   else xd = NULL;

   /* initialize any unitialized state vectors by a call to init_func() */
   {
      Hor_Matrix  **y = NULL, ***zu = NULL, **yu = NULL;
      int          *y_type = NULL, *yu_type = NULL, **zu_type = NULL;
      void       ***data_u = NULL;
      Hor_Bool      success;
      Uninit_State *us;

      if ( (nu = vs->nu) > 0 )
      {
	 yu       = hor_malloc_ntype ( Hor_Matrix *, nu );
	 yu_type  = hor_malloc_ntype (          int, nu );
	 for ( i = 0; i < nu; i++ ) {
	    yu[i]       = NULL;
	    yu_type[i]  = 0;
	 }
      }

      if ( vs->n  > 0 )
      {
	 y      = hor_malloc_ntype ( Hor_Matrix *, vs->n );
	 y_type = hor_malloc_ntype (          int, vs->n );
	 for ( i = 0; i < vs->n; i++ )
	 {
	    y[i]      = vs->local_state[i].y;
	    y_type[i] = vs->local_state[i].y_type;
	 }
      }

      if ( nu > 0 )
      {
	 zu      = hor_malloc_ntype ( Hor_Matrix **, nu );
	 zu_type = hor_malloc_ntype ( int         *, nu );
	 data_u  = hor_malloc_ntype ( void       **, nu );
	 for ( i = 0, us = vs->uninit_state; i < nu; i++, us++ )
	 {
	    zu[i]      = hor_malloc_ntype ( Hor_Matrix *, vs->stored );
	    zu_type[i] = hor_malloc_ntype ( int         , vs->stored );
	    data_u[i]  = hor_malloc_ntype ( void       *, vs->stored );
	    for ( j = 0; j < vs->stored; j++ )
	    {
	       zu[i][j]      = NULL;
	       zu_type[i][j] = 0;
	       data_u[i][j]  = NULL;
	    }

	    for ( list = us->zlist; list != NULL; list = list->next )
	    {
	       bz = (Batch_Obs *) list->contents;
	       if ( (j=bz->k-1) < 0 ) continue;

	       if ( j >= vs->stored )
		  hor_error ( "illegal observation", HOR_FATAL );

	       zu[i][j]      = bz->z;
	       zu_type[i][j] = bz->z_type;
	       data_u[i][j]  = bz->user_data;
	    }
	 }
      }

      success = init_func ( vs->xf, &vs->xf_type, xd, xd_type, vs->stored,
			    y, y_type, vs->n, yu, yu_type, nu,
			    zu, zu_type, data_u, user_data, vs->user_state );

      /* reset size of dynamic global state vector in case it has been changed
	 by initialization function */
      if ( vs->xd != NULL ) vs->xdsize = xd[vs->stored-1]->rows;

      i = 0;
      us = vs->uninit_state;
      if ( success )
	 while ( i < vs->nu )
	    if ( yu[i] == NULL ) { i++; us++; }
            else
	    {
	       int index, *index_ptr = us->index_ptr;
	       Hor_List zlist = us->zlist;

	       /* remove unitialized state */
	       us->zlist = NULL;
	       hor_vsdf_remove_state ( HOR_VSDF_UNINIT_OFFSET + i, HOR_TRUE );

	       /* transfer state into initialized state list */
	       index = hor_vsdf_add_state ( yu[i], yu_type[i], NULL,
					    index_ptr );

	       /* check for failure */
	       if ( index < 0 )
		  hor_error ( "failed (hor_vsdf_batch())", HOR_FATAL );

	       /* transfer local state from end of list to current position */
	       hor_mat_free ( yu[i] );
	       if ( vs->nu > 0 ) { yu[i]      = yu[vs->nu];
				   yu_type[i] = yu_type[vs->nu]; }
	       else { yu[i] = NULL; yu_type[i] = 0; }
	       
	       /* transfer list of observations from unitialized state to
		  initialized state */
	       vs->local_state[index].zlist = zlist;
	       
	       vs->local_state[index].user_data = data_u[i][vs->stored-1];
	    }


      if ( nu > 0 )
      {
	 for ( i = nu-1; i >= 0; i-- )
	    hor_free_multiple ( data_u[i], zu_type[i], zu[i], NULL );

	 hor_free_multiple ( data_u, zu_type, zu, NULL );
      }

      if ( y != NULL ) hor_free ( y );
      if ( y_type != NULL ) hor_free ( y_type );

      if ( vs->nu > 0 )
	 for ( i = vs->nu-1; i >= 0; i-- )
	    if ( yu[i] != NULL ) hor_mat_free ( yu[i] );

      if ( yu != NULL ) hor_free_multiple ( yu_type, yu, NULL );

      if ( !success )
      {
	 if ( update_dynamic ) {
	    for ( j = 0; j < vs->stored; j++ ) hor_mat_free ( xd[j] );
	    hor_free_multiple ( xd_type, xd, NULL );
	 }

	 return HOR_FALSE;
      }
   }

   max_ysize = vs->max_ysize;
   if ( update_dynamic)
   {
      for ( j = 1, xdsize = 0; j < vs->stored; j++ ) xdsize += xd[j]->rows;

      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 ysize += ls->ysize;

      Ad = hor_malloc_ntype ( Hor_Matrix *, vs->stored );
      for ( j = 1; j < vs->stored; j++ )
	 Ad[j] = hor_mats_zero ( xd[j]->rows, xd[j]->rows );

      xdoffset = hor_malloc_ntype ( int, vs->stored );
      xdoffset[1] = 0;
      for ( j = 2; j < vs->stored; j++ )
	 xdoffset[j] = xdoffset[j-1] + xd[j-1]->rows;

      /* allocate matrices for concatenated dynamic global state vector */
      xddiff = hor_mat_alloc ( xdsize, 1 );
      DdRv   = hor_mat_alloc ( xdsize, 1 );
      if ( xdsize <= ysize )
      {
	 int max_size = hor_imax ( xdsize, vs->xfsize );

	 vxdy   = hor_mat_alloc ( xdsize, 1 );
	 L      = hor_mat_alloc ( xdsize, xdsize );
	 Linv   = hor_mat_alloc ( xdsize, xdsize );
	 temp1  = hor_mat_alloc ( max_size, max_size );
	 temp2  = hor_mat_alloc ( max_size, max_size );
	 y_first = HOR_FALSE;
      }
      else
      {
	 int max_size = hor_imax ( ysize, vs->xfsize );

	 yoffset = hor_malloc_ntype ( int, vs->n );
	 yoffset[0] = 0;
	 for ( i = 1, ls = vs->local_state; i < vs->n; i++, ls++ )
	    yoffset[i] = yoffset[i-1] + ls->ysize;

	 vxdy   = hor_mat_alloc ( ysize, 1 );
	 L      = hor_mat_alloc ( ysize, ysize );
	 Linv   = hor_mat_alloc ( ysize, ysize );
	 temp1  = hor_mat_alloc ( max_size, max_size );
	 temp2  = hor_mat_alloc ( max_size, max_size );
	 y_first = HOR_TRUE;
      }
   }

   if ( update_both )
   {
      if ( y_first )
      {
	 xffac  = hor_mat_alloc ( ysize, vs->xfsize );
	 xffacT = hor_mat_alloc ( vs->xfsize, ysize );
      }
      else
      {
	 xffac  = hor_mat_alloc ( xdsize, vs->xfsize );
	 xffacT = hor_mat_alloc ( vs->xfsize, xdsize );
      }

      if ( y_first || vs->use_optimal_dynamic_vsdf )
      {
	 AmT = hor_malloc_ntype ( Hor_Matrix *, vs->stored );
	 for ( j = 1; j < vs->stored; j++ )
	    AmT[j] = hor_mats_zero ( xd[j]->rows, vs->xfsize );
      }
   }

   hor_errno = HORATIO_OK;
#ifdef USE_LEV_MARQ
   lambda = LAMBDA_START;
#endif

   for ( it = 1; it <= iterations; it++ )
   {
      if ( 1 ) /* reset matrices to their prior state */
      {
	 hor_matq_zero ( vs->J );
	 vs->dof = 0;
	 if ( update_fixed )
	 {
	    if ( vs->Pf0inv == NULL )
	    {
	       hor_matq_zero ( vs->Af );
	       vs->dof -= vs->xfsize;
	    }
	    else hor_matq_copy ( vs->Pf0inv, vs->Af );

	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       hor_matq_zero ( ls->Bf );
	       hor_matq_zero ( ls->BfT );
	    }
	 }

	 if ( update_dynamic )
	    for ( j = 1; j < vs->stored; j++ )
	    {
	       hor_matq_zero ( Ad[j] );
	       vs->dof -= xd[j]->rows;
	       if ( update_fixed && y_first ) hor_matq_zero ( AmT[j] );
	    }

	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->T0inv == NULL )
	    {
	       hor_matq_zero ( ls->C );
	       vs->dof -= ls->ysize;
	    }
	    else hor_matq_copy ( ls->T0inv, ls->C );
      }

      if ( update_fixed )
	 hor_matq_prod2(vs->Af, hor_matq_sub(vs->xf0, vs->xf, work1), vs->vxf);

      if ( update_dynamic ) { hor_matq_zero ( vxdy );
			      hor_matq_zero ( L );
			      hor_matq_zero ( DdRv ); }

      if ( update_both ) if ( y_first ) hor_matq_zero ( xffacT );
                         else           hor_matq_zero ( xffac );

      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      {
	 hor_matq_zero ( ls->batch_J );
	 ls->batch_dof = 0;
	 hor_matq_prod2 ( ls->C, hor_matq_sub(ls->y0, ls->y, work1), ls->ERv );
	 if ( update_fixed ) hor_matq_zero ( ls->DfRv );

	 vs->E->cols = ls->ysize;
	 for ( list = ls->zlist; list != NULL; list = list->next )
	 {
	    Hor_Matrix *xdj = NULL;
	    int    xd_typej = 0;

	    bz = (Batch_Obs *) list->contents;
	    j = bz->k-1;

	    if ( update_dynamic && it == 1 )
	    {
	       bz->Bd  = hor_mat_alloc ( xd[j]->rows,  ls->ysize );
	       bz->BdT = hor_mat_alloc (   ls->ysize, xd[j]->rows );
	    }

	    if ( update_dynamic ) { xdj = xd[j]; xd_typej = xd_type[j]; }

	    vsdf_call_func ( bz->obs_type, bz->func, bz->z, bz->z_type, ls->v,
			     bz->F_size, bz->Rinv, bz->N,
			     update_fixed,   vs->xf, vs->xf_type, vs->Df,
			     update_dynamic,    xdj,    xd_typej, ls->Dd,
			     HOR_TRUE,        ls->y,  ls->y_type,  vs->E,
			     vs->Fz, vs->FzT,
			     bz->user_data, vs->user_state );

	    if ( hor_errno != HORATIO_OK )
	    { hor_perror ( "(hor_vsdf_batch(4))" ); return HOR_FALSE; }

	    hor_matq_transpose ( vs->E, vs->ET );
	    hor_mat_increment ( ls->C,
		    hor_matq_prod3 ( vs->ET, bz->Rinv, vs->E, work1, work2 ) );
	    if ( hor_errno != HORATIO_OK )
	    { hor_perror ( "(hor_vsdf_batch(5))" ); return HOR_FALSE; }

	    hor_matq_prod2 ( bz->Rinv, ls->v, vs->Rinv_v );

	    /* increment residual and DOF */
	    hor_matq_transpose ( ls->v, work1 );
	    hor_mat_increment ( vs->J,
			        hor_matq_prod2 (work1, vs->Rinv_v, work2) );
	    hor_mat_increment ( ls->batch_J, work2 );
	    vs->dof += bz->z->rows;
	    ls->batch_dof += bz->z->rows;

	    if ( update_fixed && it == iterations )
	    {
	       /* increment local residual term zp^T*R^-1*zp */
	       hor_mat_increment(ls->v, hor_matq_prod2(vs->Df, vs->xf, work2));
	       hor_mat_increment(ls->v, hor_matq_prod2(vs->E,  ls->y,  work2));
	       hor_matq_transpose ( ls->v, work2 );
	       hor_mat_increment ( ls->zRz,
		      hor_matq_prod3(work2, bz->Rinv, ls->v, work1, work3) );

	       /* increment b vector */
	       hor_mat_increment ( ls->b,
		    hor_matq_prod3 ( vs->ET, bz->Rinv, ls->v, work1, work2 ) );

	       ls->dof += ls->v->rows;
	    }

	    hor_mat_increment ( ls->ERv, hor_matq_prod2(vs->ET, vs->Rinv_v,
							work1) );

	    if ( update_fixed )
	    {
	       hor_matq_transpose ( vs->Df, vs->DfT );
	       hor_mat_increment ( vs->Af,
		  hor_matq_prod3 ( vs->DfT, bz->Rinv, vs->Df, work2, work3 ) );
	       if ( it == iterations )
	       {
		  hor_mat_increment ( ls->Af, work3 );
		  hor_mat_increment ( ls->a_BCb,
				      hor_matq_prod2 (work3, vs->xf, work4) );
	       }

	       hor_mat_increment ( ls->Bf,
				   hor_matq_prod2 ( work2, vs->E, work3 ) );
	       if ( y_first )
		  hor_mat_dec_offset ( xffacT, work3, yoffset[i], 0 );

	       hor_mat_increment ( ls->DfRv,
			      hor_matq_prod2 ( vs->DfT, vs->Rinv_v, work1 ) );
	    }

	    if ( update_dynamic && j > 0 )
	    {
	       hor_matq_transpose ( ls->Dd, ls->DdT );
	       hor_matq_prod3 ( ls->DdT, bz->Rinv, ls->Dd, work2, work1 );
	       hor_mat_increment ( Ad[j], work1 );
						        
	       hor_matq_prod2 ( work2, vs->E, bz->Bd );
	       hor_matq_transpose ( bz->Bd, bz->BdT );
	       hor_mat_inc_offset ( DdRv, hor_matq_prod2 ( ls->DdT, vs->Rinv_v,
							   work1 ),
				    0, xdoffset[j] );
	    }

	    if ( update_both && j > 0 )
	       if ( y_first )
		  hor_mat_increment ( AmT[j],
			    hor_matq_prod3 ( ls->DdT, bz->Rinv, vs->Df,
					     work2, work1 ) );
	       else
	       {
		  hor_mat_dec_offset ( xffac,
		    hor_matq_prod3 ( ls->DdT, bz->Rinv, vs->Df, work2, work1 ),
				     0, xdoffset[j] );
		  if ( vs->use_optimal_dynamic_vsdf && it == iterations )
		     hor_mat_increment ( AmT[j], work1 );
	       }
	 }
      }

      if ( vs->dof < 0 )
      {
	 if ( update_both )
	 {
	    if ( y_first || vs->use_optimal_dynamic_vsdf )
	    {
	       for ( j = 1; j < vs->stored; j++ ) hor_mat_free ( AmT[j] );
	       hor_free ( AmT );
	    }

	    hor_mat_free_list ( xffacT, xffac, NULL );
	 }

	 if ( update_dynamic )
	 {
	    for ( j = vs->stored-1; j >= 0; j-- ) hor_mat_free ( xd[j] );

	    hor_mat_free_list ( temp2, temp1, DdRv, Linv, L, vxdy, xddiff,
			        NULL );
	    for ( j = 1; j < vs->stored; j++ ) hor_mat_free ( Ad[j] );

	    if ( y_first ) hor_free ( yoffset );
	    hor_free_multiple ( xdoffset, Ad, xd_type, xd, NULL);
	 }

	 hor_error ( "#(observations) < #(parameters) (hor_vsdf_batch)",
		     HOR_NON_FATAL );
	 return HOR_FALSE;
      }

      if ( it == iterations1 )
      {
	 for ( i = 0, ls = vs->local_state; i < vs->n; )
 	    if ( hor_chi_2_prob ( ls->batch_J->m[0][0], ls->batch_dof )
		 < conf_level )
	       hor_vsdf_remove_state ( i, HOR_FALSE );
	    else { i++; ls++; }

#ifdef USE_LEV_MARQ
	 lambda = LAMBDA_START;
#endif
	 continue;
      }

#ifdef USE_LEV_MARQ
      if ( it > 1              && it != iterations1 &&
	   it != iterations1+1 && it != iterations)
	 if ( vs->J->m[0][0] > old_resid )
	 {
	    /* undo previous state changes */
	    if ( update_fixed )
	    {
	       hor_mat_decrement ( vs->xf, vs->xfdiff );
	       hor_matq_zero ( vs->xfdiff );
	    }

	    if ( update_dynamic )
	    {
	       work1->cols = 1;
	       for ( j = 1; j < vs->stored; j++ )
	       {
		  work1->rows = xd[j]->rows;
		  hor_mat_insert ( xddiff, 0, xdoffset[j], work1, 0, 0,
				   1, xd[j]->rows );
		  hor_mat_decrement ( xd[j], work1 );
	       }

	       hor_matq_zero ( xddiff );
	    }

	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       hor_mat_decrement ( ls->y, ls->ydiff );
	       hor_matq_zero ( ls->ydiff );
	    }

	    lambda *= LAMBDA_FACTOR;
	    continue;
	 }
	 else lambda /= LAMBDA_FACTOR;

      old_resid = vs->J->m[0][0];

      /* scale diagonals of structure inverse covariance matrix
	 for Levenberg-Marquardt */
      if ( it != iterations )
      {
	 if ( update_fixed )
	    hor_matq_scale_diagonal ( vs->Af, 1.0+lambda );

	 if ( update_dynamic )
	    for ( j = 1; j < vs->stored; j++ )
	       hor_matq_scale_diagonal ( Ad[j], 1.0+lambda );

	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    hor_matq_scale_diagonal ( ls->C, 1.0+lambda );
      }
#endif /* USE_LEV_MARQ */

      if ( (update_dynamic && it == iterations && vs->use_optimal_dynamic_vsdf)
	   || y_first )
	 /* invert each Ad[j] and copy it back into Ad[j] */
	 for ( j = 1; j < vs->stored; j++ )
	 {
	    hor_matq_inv ( Ad[j], work1, work2, work3 );
	    hor_matq_copy ( work3, Ad[j] );
	 }
	 
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      {
	 hor_matq_inv ( ls->C, work1, work2, ls->Cinv );
	 hor_matq_prod2 ( ls->Cinv, ls->ERv, ls->CERv );

	 if ( update_fixed )
	 {
	    hor_matq_transpose ( ls->Bf, ls->BfT );
	    hor_matq_prod2 ( ls->Bf, ls->CERv, work1 );
	    hor_mat_increment (vs->vxf, hor_matq_sub(ls->DfRv,work1,work2));
	    if ( it == iterations ) hor_mat_increment ( ls->a_BCb, work2 );
	 }
      }

      if ( y_first )
      {
	 if ( it != iterations )
	 {
	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       /* increment L = C - Bd^T*Ad^-1*Bd */
	       hor_mat_inc_offset ( L, ls->C, yoffset[i], yoffset[i] );

	       for ( p = 0, ls2 = vs->local_state; p < vs->n; p++, ls2++ )
	       {
		  list  = ls->zlist;
		  list2 = ls2->zlist;
		  while ( list != NULL && list2 != NULL )
		  {
		     bz  = (Batch_Obs *) list->contents;
		     bz2 = (Batch_Obs *) list2->contents;
		     if ( bz2->k < bz->k )
		     { list = list->next; continue; }
		     else if ( bz2->k > bz->k )
		     { list2 = list2->next; continue; }

		     if ( bz->k != 1 )
			hor_mat_dec_offset ( L,
			       hor_matq_prod3 ( bz->BdT, Ad[bz->k-1], bz2->Bd,
					        work2, work1 ),
					     yoffset[p], yoffset[i] );

		     list  = list->next;
		     list2 = list2->next;
		  }
	       }
				    
	       if ( update_both )
		  for ( list = ls->zlist; list != NULL; list = list->next )
		  {
		     bz = (Batch_Obs *) list->contents;
		     if ( bz->k == 1 ) continue;

		     hor_mat_inc_offset ( xffacT,
			hor_matq_prod3(hor_matq_transpose(AmT[bz->k-1], work1),
				       Ad[bz->k-1], bz->Bd, work2, work3),
				       yoffset[i], 0);
		  }
	    }

	    hor_matq_inv ( L, temp1, temp2, Linv );
	 }

	 /* calculate Bd^T*Ad^-1*Am^T - Bf^T */
	 if ( update_both ) hor_matq_transpose ( xffacT, xffac );

	 /* calculate alpha = Pf^-1 */
	 if ( update_fixed )
	 {
	    hor_matq_copy ( vs->Af, vs->alpha );
	    for ( j = 1; j < vs->stored; j++ )
               hor_mat_decrement ( vs->alpha,
			   hor_matq_prod3 ( hor_matq_transpose(AmT[j], work1),
					    Ad[j], AmT[j], work2, work3 ) );

	    if ( it == iterations )
	       for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	       {
		  hor_matq_prod3 (ls->Bf, ls->Cinv, ls->BfT, work2, ls->BCB);
		  hor_mat_decrement ( ls->a_BCb,
				   hor_matq_prod2(ls->BCB, vs->xf, work2) );
	       }

	    hor_mat_decrement ( vs->alpha,
			hor_matq_prod3 ( xffacT, Linv, xffac, temp1, work1 ) );
	    hor_matq_inv ( vs->alpha, work1, work2, vs->alpha_inv );
	 }

	 /* calculate xddiff = Ad^-1 DdRv */
	 for ( j = 1; j < vs->stored; j++ )
	 {
	    hor_matq_prod2 ( Ad[j], hor_matq_extract ( DdRv, 0, xdoffset[j],
						       1, xd[j]->rows, work1 ),
			     work2 );
	    hor_mat_insert ( work2, 0, 0, xddiff, 0, xdoffset[j],
			     1, xd[j]->rows);
	 }

	 /* calculate vy = L^-1(E^T - Bd^T Ad^-1 Dd^T)R^-1 v */
	 temp1->rows = ysize; temp1->cols = 1;
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 {
	    hor_mat_insert ( ls->ERv, 0, 0,
			     temp1, 0, yoffset[i], 1, ls->ysize );
	    for ( list = ls->zlist; list != NULL; list = list->next )
            {
	       bz = (Batch_Obs *) list->contents;
	       if ( bz->k == 1 ) continue;

	       hor_mat_dec_offset ( temp1,
		  hor_matq_prod2 ( bz->BdT,
		     hor_matq_extract ( xddiff, 0, xdoffset[bz->k-1],
				        1, xd[bz->k-1]->rows, work1 ),
				   work2 ), 0, yoffset[i] );
	    }
	 }

	 hor_matq_prod2 ( Linv, temp1, vxdy );
      }	    
      else
      {
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 {
	    /* decrement L = Ad - Bd*C^-1*Bd^T */
	    if ( update_dynamic && it != iterations )
	       for ( list = ls->zlist; list != NULL; list = list->next )
	       {
		  bz = (Batch_Obs *) list->contents;
		  if ( bz->k == 1 ) continue;

		  for ( list2 = ls->zlist; list2 != NULL; list2 = list2->next )
		  {
		     bz2 = (Batch_Obs *) list2->contents;

		     if ( bz2->k == 1 ) continue;

		     hor_mat_dec_offset ( L,
			      hor_matq_prod3(bz->Bd, ls->Cinv, bz2->BdT,
					     work2, work1),
				       xdoffset[bz2->k-1], xdoffset[bz->k-1] );
		  }
	       }
				    
	    if ( update_both )
	       for ( list = ls->zlist; list != NULL; list = list->next )
	       {
		  bz = (Batch_Obs *) list->contents;
		  if ( bz->k == 1 ) continue;

		  hor_mat_inc_offset ( xffac,
		    hor_matq_prod3 ( bz->Bd, ls->Cinv, ls->BfT, work2, work1 ),
				     0, xdoffset[bz->k-1] );
	       }
	 }

	 if ( update_dynamic && it != iterations )
	 {
	    /* calculate L^-1 */
	    for ( j = 1; j < vs->stored; j++ )
	       hor_mat_inc_offset ( L, Ad[j], xdoffset[j], xdoffset[j] );

	    hor_matq_inv ( L, temp1, temp2, Linv );
	 }

	 /* calculate Bf*C^-1*Bd^T - Am */
	 if ( update_both ) hor_matq_transpose ( xffac, xffacT );

	 /* calculate alpha = Pf^-1 */
	 if ( update_fixed )
	 {
	    hor_matq_copy ( vs->Af, vs->alpha );
	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       hor_mat_decrement ( vs->alpha,
                  hor_matq_prod3 (ls->Bf, ls->Cinv, ls->BfT, work2, ls->BCB) );
	       if ( it == iterations )
		  hor_mat_decrement ( ls->a_BCb,
				      hor_matq_prod2(ls->BCB, vs->xf, work2) );
	    }

	    if ( update_dynamic )
	       hor_mat_decrement ( vs->alpha,
		        hor_matq_prod3 ( xffacT, Linv, xffac, temp1, work1 ) );

	    hor_matq_inv ( vs->alpha, work1, work2, vs->alpha_inv );
	 }

	 /* calculate vxd = -Bd*C^-1*E^T*R^-1*v */
	 if ( update_dynamic && it != iterations )
	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	       for ( list = ls->zlist; list != NULL; list = list->next )
	       {
		  bz = (Batch_Obs *) list->contents;
		  if ( bz->k == 1 ) continue;

		  hor_mat_dec_offset ( vxdy,
				    hor_matq_prod2 ( bz->Bd, ls->CERv, work1 ),
				    0, xdoffset[bz->k-1] );
	       }
      }

      /* update global state vector */
      if ( update_fixed && it != iterations )
      {
	 if ( update_dynamic )
	    if ( y_first )
	    {
	       hor_mat_increment ( vs->vxf,
				   hor_matq_prod2 ( xffacT, vxdy, work1 ) );
	       for ( j = 1; j < vs->stored; j++ )
		  hor_mat_decrement ( vs->vxf,
		     hor_matq_prod2 ( hor_matq_transpose ( AmT[j], work1 ),
			hor_matq_extract ( xddiff, 0, xdoffset[j],
					   1, xd[j]->rows, work2 ), work3 ) );
	    }
	    else
	    {
	       hor_mat_increment ( vs->vxf,
				   hor_matq_prod3 ( xffacT, Linv, DdRv,
						    temp1, work1 ) );
	       hor_mat_increment ( vs->vxf,
				   hor_matq_prod2 (temp1, vxdy, work1) );
	    }

	 hor_mat_increment ( vs->xf, hor_matq_prod2 ( vs->alpha_inv, vs->vxf,
						      work1 ) );
	 hor_matq_copy ( work1, vs->xfdiff ); /* stores state change */
      }

      if ( it != iterations )
	 if ( y_first )
	 {
	    /* update local state vectors */
	    if ( update_fixed )
	       hor_mat_increment ( vxdy,
		    hor_matq_prod3 ( Linv, xffac, vs->xfdiff, temp1, temp2 ) );

	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       hor_matq_extract ( vxdy, 0, yoffset[i], 1, ls->ysize,
				  ls->ydiff );
	       hor_mat_increment ( ls->y, ls->ydiff );
	    }

	    /* update dynamic global state vector */
	    if ( update_fixed )
	       for ( j = 1; j < vs->stored; j++ )
	       {
		  hor_matq_prod3 ( Ad[j], AmT[j], vs->xfdiff, work1, work2 );
		  hor_mat_dec_offset ( xddiff, work2, 0, xdoffset[j] );
	       }

	    
	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	       for ( list = ls->zlist; list != NULL; list = list->next )
	       {
		  bz = (Batch_Obs *) list->contents;
		  if ( bz->k == 1 ) continue;

		  hor_matq_prod3 ( Ad[bz->k-1], bz->Bd, ls->ydiff,
				   work1, work2 );
		  hor_mat_dec_offset ( xddiff, work2, 0, xdoffset[bz->k-1] );
	       }

	    work1->cols = 1;
	    for ( j = 1; j < vs->stored; j++ )
	    {
	       work1->rows = xd[j]->rows;
	       hor_mat_insert ( xddiff, 0, xdoffset[j], work1, 0, 0,
			        1, xd[j]->rows );
	       hor_mat_increment ( xd[j], work1 );
	    }
	 }
	 else
	 {
	    if ( update_dynamic )
	    {
	       hor_mat_increment ( vxdy, DdRv );
	       if ( update_fixed )
		  hor_mat_increment(vxdy,
				    hor_matq_prod2 (xffac, vs->xfdiff, temp1));

	       hor_matq_prod2 ( Linv, vxdy, xddiff ); /* stores state change */
	       work1->cols = 1;
	       for ( j = 1; j < vs->stored; j++ )
	       {
		  work1->rows = xd[j]->rows;
		  hor_mat_insert ( xddiff, 0, xdoffset[j], work1, 0, 0,
				   1, xd[j]->rows );
		  hor_mat_increment ( xd[j], work1 );
	       }
	    }

	    /* update local state vectors */
	    for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    {
	       if ( update_fixed )
		  hor_mat_decrement ( ls->CERv,
		        hor_matq_prod3 (ls->Cinv, ls->BfT, vs->xfdiff,
					work1, work2) );

	       if ( update_dynamic )
		  for ( list = ls->zlist; list != NULL; list = list->next )
		  {
		     Batch_Obs *bz = (Batch_Obs *) list->contents;
		     if ( bz->k == 1 ) continue;

		     hor_matq_extract ( xddiff, 0, xdoffset[bz->k-1], 1,
				       xd[bz->k-1]->rows, work1 );
		     hor_mat_decrement ( ls->CERv,
				   hor_matq_prod3 (ls->Cinv, bz->BdT,
						   work1, work2, work3) );
		  }

	       /* ls->ydiff will contain local state increment */
	       hor_mat_increment ( ls->y, ls->CERv );
	       hor_matq_copy ( ls->CERv, ls->ydiff );
	    }
	 }

      if ( update_dynamic && it == iterations && vs->use_optimal_dynamic_vsdf )
      {
	 /* increment BAB = Bd^T*Ad^-1*Bd */
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    for ( p = 0, ls2 = vs->local_state; p < vs->n; p++, ls2++ )
	       for ( list = ls->zlist; list != NULL; list = list->next )
	       {
		  Batch_Obs *bz = (Batch_Obs *) list->contents;

		  if ( bz->k == 1 || bz->k == 2 ) continue;

		  for ( list2 = ls2->zlist; list2 != NULL;
		        list2 = list2->next )
		  {
		     bz2 = (Batch_Obs *) list2->contents;
		     if ( bz2->k != bz->k ) continue;

		     hor_mat_inc_offset ( vs->BAB,
				       hor_matq_prod3 ( bz2->BdT, Ad[bz->k-1],
						        bz->Bd, work1, work2 ),
					  i*max_ysize, p*max_ysize );
		  }
	       }	       
      }
   }

   if ( update_dynamic )
   {
      if ( vs->k-1 == vs->stored )
      {
	 /* update initial dynamic global state vector */
	 Hor_VSDF_XD_Def *xd_def = (Hor_VSDF_XD_Def *) vs->xdlist->contents;

	 hor_matq_copy ( xd[0], xd_def->xd );
	 xd_def->xd_type = xd_type[0];
      }

      /* add dynamic global state vectors to the stored list */
      for ( j = 1; j < vs->stored; j++ )
      {
	 Hor_VSDF_XD_Def *xd_def = hor_malloc_type(Hor_VSDF_XD_Def);

	 xd_def->xd      = xd[j];
	 xd_def->xd_type = xd_type[j];
	 xd_def->Ad      = hor_mats_copy(Ad[j]);
	 vs->xdlist = hor_insert ( vs->xdlist, (void *) xd_def );
      }

      hor_mat_free ( xd[0] );

      /* copy modified dynamic state vector into structure */
      hor_matq_copy ( xd[vs->stored-1], vs->xd );
      vs->xd_type = xd_type[vs->stored-1];
   }

   if ( update_both && vs->use_optimal_dynamic_vsdf )
   {
      for ( j = 2; j < vs->stored; j++ )
	 hor_mat_increment ( vs->AAA,
			     hor_matq_prod3 ( hor_matq_transpose(AmT[j],work1),
					      Ad[j], AmT[j], work2, work3 ) );
      vs->BAA->rows = vs->n*max_ysize;
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 for ( list = ls->zlist; list != NULL; list = list->next )
	 {
	    Batch_Obs *bz = (Batch_Obs *) list->contents;
	    if ( bz->k == 1 || bz->k == 2 ) continue;

	    hor_matq_prod3 ( bz->BdT, Ad[bz->k-1], AmT[bz->k-1], work1, work2);
	    hor_mat_inc_offset ( vs->BAA, work2, 0, i*max_ysize );
	 }

      hor_matq_transpose ( vs->BAA, vs->AAB );
   }
      
   if ( update_both )
   {
      if ( y_first || vs->use_optimal_dynamic_vsdf )
      {
	 for ( j = 1; j < vs->stored; j++ ) hor_mat_free ( AmT[j] );
	 hor_free ( AmT );
      }

      hor_mat_free_list ( xffacT, xffac, NULL );
   }

   if ( update_dynamic )
   {
      hor_mat_free_list ( temp2, temp1, DdRv, Linv, L, vxdy, xddiff, NULL );
      for ( j = 1; j < vs->stored; j++ ) hor_mat_free ( Ad[j] );

      if ( y_first ) hor_free ( yoffset );
      hor_free_multiple ( xdoffset, Ad, xd_type, xd, NULL );
   }

   vs->stored = 0;

   return HOR_TRUE;
}

/*******************
*   Hor_Bool *hor_vsdf_change_frame(Hor_Bool (*change_func)(Hor_Matrix  *xf,
*                                                           int *xf_type,,
*                                                           Hor_Matrix *Af,
*							    Hor_Matrix  *xd,
*                                                           int *xd_type,
*                                                           int n,
*							    Hor_Matrix **y,
*                                                           int *y_type,
*                                                           Hor_Matrix **Bf,
*							    Hor_Matrix **C))
*
*   Change the reference frame for the global state using the function
*   change func.  This should change the global states which may decrease
*   in size but *not* increase.  It should also perform the requisite
*   changes to the local states and covariances.  The new dynamic state 
*   should be set.  
*   The function returns the same return value as change_func (HOR_TRUE on
*   success and HOR_FALSE on failure
*
********************/
Hor_Bool hor_vsdf_change_frame(Hor_Bool (*change_func)(Hor_Matrix  *xf,
						       int *xf_type,
						       Hor_Matrix *Af,
						       Hor_Matrix  *xd,
						       int *xd_type,
						       int n,
						       Hor_Matrix **y,
						       int *y_type,
						       Hor_Matrix **vf,
						       Hor_Matrix **C))
{
    Hor_Matrix **y, **Bf, **C;
    Hor_Bool retval;
    int i, *y_type;

    if ( current_vsdf == NULL ) {
	hor_error ( "not initialised (hor_vsdf_change_frame)", HOR_NON_FATAL );
	return HOR_FALSE;
    }

    y = hor_malloc_ntype ( Hor_Matrix *, current_vsdf->n );
    y_type = hor_malloc_ntype ( int, current_vsdf->n );
    Bf = hor_malloc_ntype ( Hor_Matrix *, current_vsdf->n );
    C = hor_malloc_ntype ( Hor_Matrix *, current_vsdf->n );
    for (i=0; i<current_vsdf->n; i++) {
	y[i] = current_vsdf->local_state[i].y;
	Bf[i] = current_vsdf->local_state[i].Bf;
	C[i] = current_vsdf->local_state[i].C;
    }

    retval = change_func(current_vsdf->xf, &current_vsdf->xf_type,
			 current_vsdf->Af,
			 current_vsdf->xd, &current_vsdf->xd_type,
			 current_vsdf->n, y, y_type, Bf, C);

    /* reset sizes for new dynamic state dimension */
    current_vsdf->Ad->rows = current_vsdf->Ad->cols = current_vsdf->vxd->rows = 
    current_vsdf->xdsize = current_vsdf->xd->rows;

    if ( current_vsdf->use_optimal_dynamic_vsdf )
	current_vsdf->Bd->rows = current_vsdf->xdsize;

    for ( i = 0; i < current_vsdf->n; i++ ) {
	current_vsdf->local_state[i].Dd->cols = 
	current_vsdf->local_state[i].Bd->rows = 
	current_vsdf->xdsize;
    }

    hor_free_multiple(y, y_type, Bf, C, NULL);

    return retval;
}

/*******************
*   Hor_Bool *@hor_vsdf_update ( int iterations, double conf_level,
*                               enum { HOR_MOTION_UPDATE, HOR_DROID_UPDATE,
*                                      HOR_FULL_UPDATE } update_type )
*                               void (*update_func)(Hor_Matrix  *xf,
*                                                   int          xf_type,
*                                                   Hor_Matrix  *xd,
*                                                   int          xd_type,
*                                                   Hor_Matrix **y,
*                                                   int         *y_type,
*                                                   Hor_Matrix **C,
*                                                   Hor_Matrix **z,
*                                                   int         *z_type,
*                                                   void       **zdata,
*                                                   Hor_Bool    *z_accept,
*                                                   int          n,
*                                                   void *user_data,
*                                                   void *user_state),
*                               void (*reset_func)(Hor_Matrix  *xf,
*                                                  int          xf_type,
*                                                  Hor_Matrix  *xd,
*                                                  int          xd_type,
*                                                  Hor_Matrix **y,
*                                                  int         *y_type,
*                                                  Hor_Matrix **C,
*                                                  Hor_Matrix **z,
*                                                  int         *z_type,
*                                                  void       **zdata,
*                                                  Hor_Bool    *z_accept,
*                                                  void *user_data,
*                                                  void *user_state),
*                               void *user_data );
*
*   VSDF state update function, called after hor_vsdf_obs_h() has been
*   called for every local state vector that is to be updated in the current
*   time step. First the dynamic part of the global state vector is updated
*   using the provided number of update iterations, which work in the same way
*   as the recursively iterated EKF. Then the whole state vector is updated.
*
*   The return value is an array of Hor_Bool's, each element of which is set
*   to HOR_FALSE if the corresponding measurement is rejected by a chi-squared
*   test using the provided confidence level (e.g. 0.05), HOR_TRUE otherwise.
*   The test is applied only if there is a dynamic part to the state vector.
*   If there is only a fixed global state, the outlier test function
*   hor_vsdf_test_state() should be used.
*
*   If the update_type argument is HOR_FULL_UPDATE, the full global and local
*   state update is applied. If it is HOR_DROID_UPDATE, after the dynamic
*   global update iterations described above, only the fixed global states
*   and the local states are updated, in DROID-like fashion. If the value
*   of update_type is HOR_MOTION_UPDATE, the update terminates after the
*   update of the dynamic part of the global state vector, i.e. the fixed
*   global state and the local states are not changed at all.
*
*   If there is no dynamic part to the state vector then the first three
*   arguments are ignored.
*
*   The update_func() argument is an optional function to update the
*   dynamic state vector given the new observations prior to any further
*   update. It can be used to implement any special updating procedure
*   particular to the measurement model. The user_data argument is a user
*   pointer passed to the update function. z_accept is an array of Boolean
*   variables for each observation, each initialized to HOR_TRUE before
*   entering the function. If an observation is to be rejected (e.g. as an
*   outlier) then the corresponding element of z_accept should be set to
*   HOR_FALSE. Note that update_func() should only modify the contents
*   of xd and the z_accept array. The other arguments are read-only.
*   The y, z and zdata arrays are the latest local state vectors,
*   observations and user data pointers. Only local states which have
*   an observation at the latest time step are passed, thus n is <= the total
*   number of local state vectors. update_func() may be passed as NULL to
*   hor_vsdf_update(), in which case both it and the user_data argument are
*   ignored.
********************/
Hor_Bool *hor_vsdf_update ( int iterations, double conf_level,
			    Hor_VSDF_Update_Type update_type,
			    void (*update_func)(Hor_Matrix  *xf,
						int          xf_type,
						Hor_Matrix  *xd,
						int         *xd_type,
						Hor_Matrix **y,
						int         *y_type,
						Hor_Matrix **C,
						Hor_Matrix **z,
						int         *z_type,
						void       **zdata,
						Hor_Bool    *z_accept,
						int          n,
						void *user_data,
						void *user_state),
			    void (*reset_func)(Hor_Matrix  *xf,
					       int          xf_type,
					       Hor_Matrix  *xd,
					       int         *xd_type,
					       Hor_Matrix **y,
					       int         *y_type,
					       Hor_Matrix **C,
					       Hor_Matrix **z,
					       int         *z_type,
					       void       **zdata,
					       Hor_Bool    *z_accept,
					       int          n,
					       void *user_data,
					       void *user_state),
			    void *user_data )
{
   VSDF_State  *vs;
   Local_State *ls;
   int          i, j, max_ysize = 0, big_n = 0;
   Hor_Bool     update_fixed, update_dynamic, update_optimal, update_both;

   /* following arrays for update and reset functions */
   Hor_Matrix **y = NULL;
   int         *y_type = NULL;
   Hor_Matrix **C = NULL;
   Hor_Matrix **z = NULL;
   int         *z_type = NULL, n = 0;
   void       **zdata = NULL;
   Hor_Bool    *z_accept = NULL;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_update)", HOR_FATAL );

   if ( vs->stored > 0 )
      hor_error ( "stored observations (hor_vsdf_update)", HOR_FATAL );

   if ( vs->xf != NULL ) update_fixed = HOR_TRUE;
   else                  update_fixed = HOR_FALSE;

   if ( vs->xd != NULL && vs->k != 1 ) update_dynamic = HOR_TRUE;
   else                                update_dynamic = HOR_FALSE;

   if ( update_fixed && update_dynamic ) update_both = HOR_TRUE;
   else                                  update_both = HOR_FALSE;

   if ( update_dynamic && vs->use_optimal_dynamic_vsdf )
      update_optimal = HOR_TRUE;
   else update_optimal = HOR_FALSE;

   /* initial label all observations as being accepted */
   for ( i = 0; i < vs->n; i++ ) vs->z_accept[i] = HOR_TRUE;

   hor_errno = HORATIO_OK;

   if ( update_func != NULL || reset_func != NULL )
   {
      if ( vs->n > 0 ) {
	 y        = hor_malloc_ntype ( Hor_Matrix *, vs->n );
	 y_type   = hor_malloc_ntype (          int, vs->n );
	 C        = hor_malloc_ntype ( Hor_Matrix *, vs->n );
	 z        = hor_malloc_ntype ( Hor_Matrix *, vs->n );
	 z_type   = hor_malloc_ntype (          int, vs->n );
	 zdata    = hor_malloc_ntype (       void *, vs->n );
	 z_accept = hor_malloc_ntype (     Hor_Bool, vs->n );

	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k )
	    {
	       y[n]        = ls->y;
	       y_type[n]   = ls->y_type;
	       C[n]        = ls->C;
	       z[n]        = ls->z;
	       z_type[n]   = ls->z_type;
	       zdata[n]    = ls->user_data;
	       z_accept[n] = HOR_TRUE;
	       n++;
	    }
      }
   }

   if ( update_dynamic )
   {
      /* update dynamic global state vector using provided function,
	 if non-NULL */
      if ( update_func != NULL )
      {
	 update_func ( vs->xf, vs->xf_type, vs->xd, &vs->xd_type, y, y_type,
		       C, z, z_type, zdata, vs->z_accept, n,
		       user_data, vs->user_state );
	 for ( i = 0, n = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
            if ( ls->k == vs->k )
	       vs->z_accept[i] = z_accept[n++];
	       
	 /* reset size of dynamic global state vector in case it has been
	    changed by the update function */
	 vs->Ad->rows = vs->Ad->cols = vs->vxd->rows = vs->xdsize =
	    vs->xd->rows;
	 if ( update_optimal ) vs->Bd->rows = vs->xdsize;

	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       ls->Dd->cols = ls->Bd->rows = vs->xdsize;
      }

      /* perform RIEKF iterations on dynamic global state vector */
      for ( j = 1; j <= iterations; j++ ) {
	 int dof = 0;

	 hor_matq_zero ( vs->Ad );
	 hor_matq_zero ( vs->vxd );
	 if ( j == iterations ) {
	    hor_matq_zero ( J );
	    dof = 0;
	 }

	 /* calculate Ad=A'=D'^T*R^-1*D' and vs->vxd=D'^T*R^-1*(z-h) */
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	    {
	       vsdf_call_func ( ls->obs_type, ls->func, ls->z, ls->z_type,
			        ls->v, ls->F_size, ls->Rinv, ls->N,
			        HOR_FALSE, vs->xf, vs->xf_type,   NULL,
			        HOR_TRUE,  vs->xd, vs->xd_type, ls->Dd,
			        HOR_FALSE,  ls->y,  ls->y_type,   NULL,
			        vs->Fz, vs->FzT,
			        ls->user_data, vs->user_state );

	       if ( hor_errno != HORATIO_OK )
	       { hor_perror ( "(hor_vsdf_update(1))" ); return NULL; }

	       hor_matq_transpose ( ls->Dd, ls->DdT );
	       hor_mat_increment ( vs->Ad,
				   hor_matq_prod3 ( ls->DdT, ls->Rinv, ls->Dd,
						    work2, ls->Ad ) );
	       hor_matq_prod2 ( ls->Rinv, ls->v, vs->Rinv_v );
	       hor_mat_increment ( vs->vxd,
			       hor_matq_prod2 ( ls->DdT, vs->Rinv_v, work3 ) );

	       if ( j == iterations )
	       {
		  /* increment dynamic state vector fit residual and DOF */
		  hor_matq_transpose ( ls->v, work2 );
		  hor_mat_increment ( J,
				 hor_matq_prod2 ( work2, vs->Rinv_v, work3 ) );
		  dof += ls->z->rows;
	       }
	    }

	 hor_matq_inv ( vs->Ad, work1, work2, work3 );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(2))" ); return NULL; }

	 hor_mat_increment ( vs->xd, hor_matq_prod2 ( work3, vs->vxd, work1 ));
	 hor_matq_copy ( work1, vs->vxd ); /* state change */

	 /* work3 contains Ad^-1 */

	 if ( j == iterations ) {
	    /* adjust residual and DOF */
	    hor_matq_transpose ( vs->vxd, work2 );
	    hor_mat_decrement ( J, hor_matq_prod3 ( work2, vs->Ad, vs->vxd,
						    work1, work2 ) );
	    dof -= vs->xdsize;

	    /* remove points from fit until residual test is passed
	       or until no more points can be removed */
	    while ( dof >= 0 &&
		    hor_chi_2_prob ( el(J,0,0), dof ) < conf_level )
	    {
	       double max_resid = 0.0;
	       int    i_max = 0;

	       /* remove point with highest residual */
	       for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
		  if ( ls->k == vs->k && vs->z_accept[i] ) {
		     hor_mat_decrement ( ls->v,
				  hor_matq_prod2 ( ls->Dd, vs->vxd, work1 ) );
		     hor_matq_transpose ( ls->v, work1 );
		     hor_matq_prod3 ( work1, ls->Rinv, ls->v, work1, work2 );
		     if ( el(work2,0,0) > max_resid )
		     {
			max_resid = el(work2,0,0);
			i_max = i;
		     }
		  }


	       ls = &vs->local_state[i_max];
	       hor_mat_decrement ( vs->Ad, ls->Ad );
	       hor_matq_inv ( vs->Ad, work1, work2, work4 );
	       if ( hor_errno != HORATIO_OK )
	       { hor_perror ( "(hor_vsdf_update(3))" ); return NULL; }

	       hor_matq_prod2 ( ls->Rinv, ls->Dd, work1 );
	       hor_matq_transpose ( work1, work2 );
	       hor_matq_prod3 ( work1, work4, work2, work5, work6 );
	       hor_mat_increment ( work6, ls->Rinv );
	       hor_matq_transpose ( ls->v, work1 );
	       hor_mat_decrement ( J, hor_matq_prod3 ( work1, work6, ls->v,
						       work2, work5 ) );
	       hor_matq_prod2 ( work6, ls->v, work5 );
	       hor_matq_prod2 ( work3, ls->DdT, work1 );
	       hor_mat_decrement ( vs->xd,
				   hor_matq_prod2 (work1, work5, vs->vxd) );
	       hor_matq_scale ( vs->vxd, -1.0 );
	       hor_matq_copy ( work4, work3 );
	       dof -= ls->z->rows;
	       vs->z_accept[i_max] = HOR_FALSE;
	    }

/*	    printf ( "dynamic residual=%f DOF=%d\n", el(J,0,0), dof );*/
	 }
      }
   }

   if ( update_dynamic ) vs->dof -= vs->xdsize;

   if ( update_dynamic )
      if ( update_type == HOR_MOTION_UPDATE )
      {
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] ) {
	       hor_matq_transpose ( ls->v, work1 );
	       hor_mat_increment ( vs->J,
				   hor_matq_prod3 ( work1, ls->Rinv, ls->v,
						    work2, work3 ) );
	       vs->dof += ls->Rinv->rows;
	    }

	 /* increment time-step */
	 vs->k++;
	 return vs->z_accept;
      }
      else if ( update_type == HOR_DROID_UPDATE )
	 update_dynamic = update_optimal = update_both = HOR_FALSE;

   if ( update_fixed ) hor_matq_zero ( vs->vxf );
   if ( update_dynamic ) { hor_matq_zero ( vs->vxd );
			   hor_matq_zero ( vs->Ad ); }
   if ( update_both ) hor_matq_zero ( vs->Am );

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      if ( ls->k == vs->k && vs->z_accept[i] ) {
	 vsdf_call_func ( ls->obs_type, ls->func, ls->z, ls->z_type,
			  ls->v, ls->F_size, ls->Rinv, ls->N,
			  update_fixed,   vs->xf, vs->xf_type, vs->Df,
			  update_dynamic, vs->xd, vs->xd_type, ls->Dd,
			  HOR_TRUE,        ls->y,  ls->y_type,  vs->E,
			  vs->Fz, vs->FzT,
			  ls->user_data, vs->user_state );

	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(4))" ); return NULL; }

	 hor_matq_transpose ( vs->E, vs->ET );
	 hor_mat_increment ( ls->C, hor_matq_prod3 ( vs->ET, ls->Rinv, vs->E,
						     work1, work2 ) );
	 hor_matq_inv ( ls->C, work1, work2, ls->Cinv );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(5))" ); return NULL; }

	 hor_matq_prod2 ( ls->Rinv, ls->v, vs->Rinv_v );
	 hor_matq_prod2 ( vs->ET, vs->Rinv_v, ls->ERv );
	 hor_matq_prod2 ( ls->Cinv, ls->ERv, ls->CERv );

	 /* increment residual */
	 hor_matq_transpose ( ls->v, work3 );
	 hor_mat_increment ( vs->J,
			     hor_matq_prod2 ( work3, vs->Rinv_v, work4 ) );

	 if ( update_fixed )
	 {
	    /* increment local residual term zp^T*R^-1*zp */
	    hor_mat_increment ( ls->v, hor_matq_prod2(vs->Df, vs->xf, work2) );
	    hor_mat_increment ( ls->v, hor_matq_prod2(vs->E,  ls->y,  work2) );
	    hor_matq_transpose ( ls->v, work2 );
	    hor_mat_increment ( ls->zRz, hor_matq_prod3(work2, ls->Rinv, ls->v,
							work1, work3) );

	    /* increment b vector */
	    hor_mat_increment ( ls->b,
		    hor_matq_prod3 ( vs->ET, ls->Rinv, ls->v, work1, work2 ) );
	 }

	 if ( update_fixed )
	 {
	    hor_matq_transpose ( vs->Df, vs->DfT );
	    hor_mat_increment ( vs->Af,
		  hor_matq_prod3 ( vs->DfT, ls->Rinv, vs->Df, work2, work3 ) );
	    hor_mat_increment ( ls->Af, work3 );
	    hor_mat_increment ( ls->a_BCb,
			        hor_matq_prod2 (work3, vs->xf, work4) );
	    hor_mat_increment ( ls->Bf, hor_matq_prod2 (work2, vs->E, work3 ));
	    hor_matq_transpose ( ls->Bf, ls->BfT );
	    hor_matq_prod2 ( vs->DfT, vs->Rinv_v, ls->DfRv );
	    if ( update_optimal )
	       hor_mat_increment ( vs->vxf, ls->DfRv );
	    else {
	       hor_matq_prod2 ( ls->Bf, ls->CERv, work4 );
	       hor_mat_increment ( vs->vxf,
				   hor_matq_sub ( ls->DfRv, work4, work3 ) );
	       hor_mat_increment ( ls->a_BCb, work3 );
	    }
	 }

	 if ( update_dynamic )
	 {
	    hor_matq_transpose ( ls->Dd, ls->DdT );
	    hor_mat_increment ( vs->Ad,
		  hor_matq_prod3 (ls->DdT, ls->Rinv, ls->Dd, work2, ls->Ad) );
	    hor_matq_prod2 ( work2, vs->E, ls->Bd );
	    hor_matq_transpose ( ls->Bd, ls->BdT );
	    hor_matq_prod2 ( ls->DdT, vs->Rinv_v, ls->DdRv );
	    hor_matq_prod2 ( ls->Bd, ls->CERv, work4 );
	    if ( update_optimal )
	       hor_mat_increment ( vs->vxd, ls->DdRv );
	    else
	       hor_mat_increment ( vs->vxd,
				   hor_matq_sub ( ls->DdRv, work4, work3 ) );
	 }

	 if ( update_both )
	    hor_mat_increment ( vs->Am,
		hor_matq_prod3 ( vs->DfT, ls->Rinv, ls->Dd, work2, ls->Am ) );
      }

   if ( update_optimal ) {
      max_ysize = vs->max_ysize;
      big_n = vs->n + vs->disc_n;
      vs->Bd->cols = big_n*max_ysize;

      hor_matq_inv ( vs->Ad, work1, work2, vs->gamma_inv );

      /* fill in Bd and Bd^T */
      hor_matq_zero ( vs->Bd );
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i] )
	    hor_mat_insert ( ls->Bd, 0, 0, vs->Bd, i*max_ysize, 0,
			     ls->ysize, vs->xdsize );

      hor_matq_transpose ( vs->Bd, vs->BdT );

      /* increment BAB = Bd^T*Ad^-1*Bd */
      hor_mat_increment ( vs->BAB,
	       hor_matq_prod3 ( vs->BdT, vs->gamma_inv, vs->Bd, big1,
			        vs->BABk ) );

      /* calculate Kinv = (C - Bd^T*Ad^-1*Bd)^-1 */
      big1->rows = big1->cols = big_n*max_ysize;
      hor_matq_identity ( big1 );
      for ( i = 0, ls = vs->local_state; i < big_n; i++, ls++ )
	 hor_mat_insert ( ls->C, 0, 0, big1, i*max_ysize, i*max_ysize,
			  ls->ysize, ls->ysize );

      hor_mat_decrement ( big1, vs->BAB );
      hor_matq_inv ( big1, big2, big3, Kinv );
   }

   if ( update_fixed ) {
      hor_matq_copy ( vs->Af, vs->alpha );
      if ( update_optimal ) {
	 /* increment AAA = Am*Ad^-1*Am^T */
	 hor_matq_transpose ( vs->Am, work1 );
	 hor_mat_increment ( vs->AAA,
	       hor_matq_prod3 ( vs->Am, vs->gamma_inv, work1, work2, work3 ) );

	 /* increment AAB = Am*Ad^-1*Bd and BAA = AAB^T */
	 hor_mat_increment ( vs->AAB,
	        hor_matq_prod3 ( vs->Am, vs->gamma_inv, vs->Bd, big1, big2 ) );

	 hor_matq_transpose ( vs->AAB, vs->BAA );
	 hor_matq_transpose ( big2, BAA );

	 /* fill in entries of Bf and BfT matrices */
	 vs->Bf->cols = big_n*max_ysize;
	 hor_matq_zero ( vs->Bf );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i] )
	    hor_mat_insert ( ls->Bf, 0, 0, vs->Bf, i*max_ysize, 0,
			     ls->ysize, vs->xfsize );

	 hor_matq_transpose ( vs->Bf, vs->BfT );

	 /* calculate alpha = Pfinv */
	 hor_mat_decrement ( vs->alpha, vs->AAA );
	 hor_mat_decrement ( vs->alpha,
		      hor_matq_prod3 ( vs->AAB, Kinv, vs->BAA, big1, work1 ) );
	 hor_mat_increment ( vs->alpha,
			     hor_matq_prod2 ( big1, vs->BfT, work1 ) );
	 hor_mat_increment ( vs->alpha, hor_matq_transpose ( work1, work2 ) );
	 hor_mat_decrement ( vs->alpha,
		       hor_matq_prod3 ( vs->Bf, Kinv, vs->BfT, big1, work1 ) );
      }
      else
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 {
	    hor_mat_increment ( ls->a_BCb,
			        hor_matq_prod2 (ls->BCB, vs->xf, work2) );
	    hor_mat_decrement ( vs->alpha,
		  hor_matq_prod3 (ls->Bf, ls->Cinv, ls->BfT, work2, ls->BCB) );
	    hor_mat_decrement ( ls->a_BCb,
			        hor_matq_prod2 (ls->BCB, vs->xf, work2) );
	 }

      hor_matq_inv ( vs->alpha, work3, work2, vs->alpha_inv );

      if ( hor_errno != HORATIO_OK )
      { hor_perror ( "(hor_vsdf_update(6))" ); return NULL; }
   }

   if ( update_dynamic )
      if ( update_optimal ) {
	 /* calculate gamma_inv = Adk^-1 + Adk^-1*Bdk*K^-1*Bdk^T*Adk^-1
	    and leave Adk^-1 in work3 */
	 hor_matq_inv ( vs->Ad, work1, work2, work3 );
	 hor_matq_prod2 ( work3, vs->Bd, big1 );
	 hor_matq_transpose ( big1, big2 );
	 hor_matq_copy ( work3, vs->gamma_inv );
	 hor_mat_increment ( vs->gamma_inv,
			 hor_matq_prod3 ( big1, Kinv, big2, big3, work1 ) );
      }
      else {
	 hor_matq_copy ( vs->Ad, vs->gamma );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_decrement ( vs->gamma,
		     hor_matq_prod3(ls->Bd, ls->Cinv, ls->BdT, work2, work3) );

	 hor_matq_inv ( vs->gamma, work4, work3, vs->gamma_inv );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(7))" ); return NULL; }
      }

   if ( update_both )
      if ( update_optimal ) {
	 /* calculate beta = (Bf - Am*Ad^-1*Bd)*K^-1*Bdk^T*Adk^-1 - Amk*Adk^-1
	    remembering that Adk^-1 is still in work3 */
	 hor_matq_prod3 ( Kinv, vs->BdT, work3, big3, big2 );
	 hor_matq_sub ( vs->Bf, vs->AAB, big1 );
	 hor_matq_prod2 ( big1, big2, vs->beta );
	 hor_mat_decrement ( vs->beta,
			     hor_matq_prod2 ( vs->Am, work3, work1 ) );
	 hor_matq_transpose ( vs->beta, vs->betaT );
      }
      else {
	 hor_matq_copy ( vs->Am, vs->beta );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_decrement ( vs->beta,
		    hor_matq_prod3 (ls->Bf, ls->Cinv, ls->BdT, work3, work4) );

	 hor_matq_transpose ( vs->beta, vs->betaT );
      }

   /* update global state vector */
   if ( update_both ) {
      if ( update_optimal ) {
	 /* Ad^-1 is still in work3
	    Set vxf = (Df^T + beta*Dd^T)*R^-1*v */
	 hor_mat_increment ( vs->vxf,
			     hor_matq_prod2 ( vs->beta, vs->vxd, work1 ) );

	 hor_matq_sub ( vs->AAB, vs->Bf, big1 );
	 hor_matq_prod2 ( big1, Kinv, big2 );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_inc_offset ( big2,
				    hor_matq_prod2 ( ls->Bf, ls->Cinv, work1 ),
				    i*max_ysize, 0 );
	 
	 /* set big2 = Bf*K^-1 - Am*Ad^-1*Bd*K^-1 */
	 hor_matq_sub ( vs->Bf, vs->AAB, big1 );
	 hor_matq_prod2 ( big1, Kinv, big2 );

	 /* calculate vxf */
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_decrement ( vs->vxf,
		  hor_mat_prod2_part ( big2, i*max_ysize, 0, vs->xfsize,
				       ls->ERv, 0, 0, 1, ls->ysize, work1 ) );

	 /* adjust fixed global state vector */
	 hor_mat_increment ( vs->xf,
			     hor_matq_prod2 ( vs->alpha_inv, vs->vxf, work1 ));
	 hor_matq_copy ( work1, vs->xfdiff ); /* store state change */

	 /* adjust dynamic global state vector */
	 hor_matq_prod2 ( vs->gamma_inv, vs->vxd, work1 );
	 hor_matq_prod3 ( work3, vs->Bd, Kinv, big1, big2 );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_decrement ( work1,
		   hor_mat_prod2_part ( big2, i*max_ysize, 0, vs->xdsize,
				        ls->ERv, 0, 0, 1, ls->ysize, work2 ) );

	 hor_mat_increment ( work1,
			     hor_matq_prod2 ( vs->betaT, vs->xfdiff, work2 ) );
	 hor_mat_increment ( vs->xd, work1 );
	 hor_matq_copy ( work1, vs->vxd ); /* store state change */
      } else {
	 hor_matq_prod3 ( vs->beta, vs->gamma_inv, vs->betaT, work1, work2 );
	 hor_matq_inv ( hor_matq_sub ( vs->alpha, work2, work3 ),
		        work2, work4, work5 );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(8))" ); return NULL; }

	 hor_matq_sub (vs->vxf, hor_matq_prod2 (work1, vs->vxd, work2), work3);
	 hor_mat_increment ( vs->xf, hor_matq_prod2 ( work5, work3, work6 ) );

	 hor_matq_prod3 ( vs->betaT, vs->alpha_inv, vs->beta, work1, work2 );
	 hor_matq_inv ( hor_matq_sub ( vs->gamma, work2, work3 ),
		        work2, work4, work5 );
	 if ( hor_errno != HORATIO_OK )
	 { hor_perror ( "(hor_vsdf_update(9))" ); return NULL; }

	 hor_matq_sub (vs->vxd, hor_matq_prod2 (work1, vs->vxf, work2), work3);
	 hor_mat_increment ( vs->xd, hor_matq_prod2 ( work5, work3, work4 ) );

	 /* store state changes */
	 hor_matq_copy ( work6, vs->xfdiff );
	 hor_matq_copy ( work4, vs->vxd );
      }
   }
   else if ( update_fixed ) {
      hor_mat_increment ( vs->xf,
			  hor_matq_prod2 ( vs->alpha_inv, vs->vxf, work3 ) );
      hor_matq_copy ( work3, vs->xfdiff ); /* stores state change */
   }
   else if ( update_dynamic ) {
      if ( update_optimal ) {
         /* Ad^-1 is still in work3
	    adjust dynamic global state vector */
	 hor_matq_prod2 ( vs->gamma_inv, vs->vxd, work1 );
	 hor_matq_prod3 ( work3, vs->Bd, Kinv, big1, big2 );
	 for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	    if ( ls->k == vs->k && vs->z_accept[i] )
	       hor_mat_decrement ( work1,
		   hor_mat_prod2_part ( big2, i*max_ysize, 0, vs->xdsize,
					ls->ERv, 0, 0, 1, ls->ysize, work2 ) );

	 hor_mat_increment ( vs->xd, work1 );
	 hor_matq_copy ( work1, vs->vxd ); /* stores state change */
      } else {
	 hor_mat_increment ( vs->xd,
			     hor_matq_prod2 ( vs->gamma_inv, vs->vxd, work3 ));
	 hor_matq_copy ( work3, vs->vxd ); /* stores state change */
      }
   }

   if ( vs->xd != NULL && vs->k != 1 )
   {
      Hor_VSDF_XD_Def *xd_def = hor_malloc_type(Hor_VSDF_XD_Def);

      xd_def->xd      = hor_mats_copy(vs->xd);
      xd_def->xd_type = vs->xd_type;
      xd_def->Ad      = hor_mats_copy(vs->Ad);
      vs->xdlist = hor_insert ( vs->xdlist, (void *) xd_def );
   }

   /* update local state vectors */
   if ( update_optimal ) {
      /* calculate big1 = (C - Bdp^T*Adp^-1*Bdp)^-1 */
      hor_matq_copy ( vs->Ad, work1 );
      hor_mat_increment ( work1, hor_matq_prod3 ( vs->Bd, Kinv, vs->BdT,
						  big1, work2 ) );
      hor_matq_inv ( work1, work2, work3, work4 );
      hor_matq_prod3 ( vs->BdT, work4, vs->Bd, big1, big2 );
      hor_matq_prod3 ( Kinv, big2, Kinv, big1, big3 );
      hor_matq_sub ( Kinv, big3, big1 );

      big2->rows = big_n*max_ysize; big2->cols = 1;
      hor_matq_zero ( big2 );
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i] )
	    hor_mat_insert ( ls->ERv, 0, 0, big2, 0, i*max_ysize,
			     1, ls->ysize );

      if ( update_fixed ) {
	 hor_mat_decrement ( BAA, vs->BfT );
	 hor_mat_increment ( big2, hor_matq_prod2 ( BAA, vs->xfdiff, big3 ) );
      }

      hor_mat_decrement ( big2, hor_matq_prod2 ( vs->BdT, vs->vxd, big3 ) );
      hor_matq_prod2 ( big1, big2, big3 );

      for ( i = 0, ls = vs->local_state; i < big_n; i++, ls++ ) {
	 hor_mat_insert ( big3, 0, i*max_ysize, ls->CERv, 0, 0,
			  1, ls->ysize );
	 hor_mat_increment ( ls->y, ls->CERv );
	 /* ls->CERv now contains local state increment */
      }
   }
   else
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i] ) {
	    if ( update_fixed )
	       hor_mat_decrement ( ls->CERv,
		   hor_matq_prod3(ls->Cinv, ls->BfT, vs->xfdiff, work1,work2));

	    if ( update_dynamic )
	       hor_mat_decrement ( ls->CERv,
		   hor_matq_prod3 (ls->Cinv, ls->BdT, vs->vxd, work1, work2) );

	    /* ls->CERv now contains local state increment */
	    hor_mat_increment ( ls->y, ls->CERv );
	 }
	 else /* no observation, but still update local state */
	    if ( update_fixed )
	       hor_mat_decrement (ls->y,
				  hor_matq_prod3(ls->Cinv, ls->BfT, vs->xfdiff,
						 work1, ls->CERv));

   /* update residual */
   if ( update_fixed ) {
      hor_matq_transpose ( vs->xfdiff, work1 );
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i])
	    hor_mat_decrement ( vs->J,
			        hor_matq_prod2 ( work1, ls->DfRv, work2 ) );
   }

   if ( update_dynamic ) {
      hor_matq_transpose ( vs->vxd, work1 );
      for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k && vs->z_accept[i] )
	    hor_mat_decrement ( vs->J,
			        hor_matq_prod2 ( work1, ls->DdRv, work2 ) );
   }

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      if ( ls->k == vs->k && vs->z_accept[i]) {
	 hor_matq_transpose ( ls->CERv, work1 );
	 hor_mat_decrement ( vs->J,
			     hor_matq_prod2 ( work1, ls->ERv, work2 ) );
      }

   /* increment degrees-of-freedom */
   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      if ( ls->k == vs->k && vs->z_accept[i])
      {
	 vs->dof += ls->Rinv->rows;
	 ls->dof += ls->Rinv->rows;
      }

   if ( reset_func != NULL )
   {
      for ( i = 0, n = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
	 if ( ls->k == vs->k )
	    z_accept[n++] = vs->z_accept[i];

      reset_func ( vs->xf, vs->xf_type, vs->xd, &vs->xd_type, y, y_type, C, z,
		   z_type, zdata, vs->z_accept, n, user_data, vs->user_state );
   }

   if ( y != NULL )
      hor_free_multiple ( z_accept, zdata, z_type, z, C, y_type, y, NULL );

   /* increment time-step */
   vs->k++;

   return vs->z_accept;
}

/*******************
*   void @hor_vsdf_scale ( double factor )
*
*   Multiples all VSDF inverse covariance matrices by the given factor, without
*   Affecting the state vectors. It can be used to inject uncertainty into
*   the VSDF, especially when the current residual is too high.
*   The value of factor must be > 0. It only applies to the local state
*   vectors and the fixed part of the global state vector.
*   The result is the same as if all inverse covariance matrices had been
*   originally multiples by factor. The residual is also adjusted.
********************/
void hor_vsdf_scale ( double factor )
{
   VSDF_State  *vs;
   int          i;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_scale)", HOR_FATAL );

   if ( factor <= 0.0 )
      hor_error ( "illegal factor value %d (hor_vsdf_scale_cov)",
		  HOR_FATAL, factor );

   if ( vs->xf != NULL ) hor_matq_scale ( vs->Af, factor );

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ ) {
      hor_matq_scale ( ls->C,    factor );
      hor_matq_scale ( ls->Cinv, 1.0/factor );
      if ( vs->xf != NULL ) {
	 hor_matq_scale ( ls->Bf,    factor );
	 hor_matq_scale ( ls->BfT,   factor );
	 hor_matq_scale ( ls->Af,    factor );
	 hor_matq_scale ( ls->a_BCb, factor );
	 hor_matq_scale ( ls->BCB,   factor );
	 hor_matq_scale ( ls->b,     factor );
	 hor_matq_scale ( ls->zRz,   factor );
      }
   }

   hor_matq_scale ( vs->J, factor );
}

/*******************
*   void @hor_vsdf_reset(void)
*
*   Resets the VSDF. The inverse covariances of xf and the y's are set to the
*   original covariances passed into hor_vsdf_init() and hor_vsdf_add_state().
*   The residual and degrees-of-freedom are also reset.
********************/
void hor_vsdf_reset(void)
{
   VSDF_State  *vs;
   int          i;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "VSDF not initialised (hor_vsdf_reset)", HOR_FATAL );

   if ( vs->xf != NULL )
      if ( vs->Pf0inv == NULL ) {
	 fill_diagonal_small ( vs->Af );
	 vs->dof = -vs->xfsize;
      }
      else {
	 hor_matq_copy ( vs->Pf0inv, vs->Af );
	 vs->dof = 0;
      }

   hor_matq_zero ( vs->J );

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ ) ls->dof = 0;

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
      if ( ls->T0inv == NULL ) {
	 fill_diagonal_small ( ls->C );
	 ls->dof -= ls->ysize;
	 vs->dof -= ls->ysize;
      }
      else
	 hor_matq_copy ( ls->T0inv, ls->C );

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ ) {
      hor_matq_inv ( ls->C, work1, work2, ls->Cinv );
      hor_matq_zero ( ls->Bf );
      hor_matq_zero ( ls->BfT );
   }
}

/*******************
*   int             @hor_vsdf_get_k(void)
*   Hor_Matrix     *@hor_vsdf_get_xf(void)
*   int             @hor_vsdf_get_xf_type(void)
*   Hor_Matrix     *@hor_vsdf_get_xd(void)
*   int             @hor_vsdf_get_xd_type(void)
*   Hor_List        @hor_vsdf_get_xdlist(void)
*   Hor_Matrix     *@hor_vsdf_get_Pf(void)
*   Hor_Matrix     *@hor_vsdf_get_Af(void)
*   Hor_Matrix     *@hor_vsdf_get_Pd(void)
*   Hor_Matrix     *@hor_vsdf_get_Ad(void)
*   double          @hor_vsdf_get_J(void);
*   int             @hor_vsdf_get_DOF(void);
*   int             @hor_vsdf_get_n(void)
*   Hor_Matrix     *@hor_vsdf_get_y      ( int index );
*   int             @hor_vsdf_get_y_type ( int index );
*   Hor_Matrix     *@hor_vsdf_get_T      ( int index );
*   Hor_Matrix     *@hor_vsdf_get_C      ( int index );
*   int             @hor_vsdf_get_obs_k  ( int index );
*   void           *@hor_vsdf_get_data   ( int index );
*   double          @hor_vsdf_get_Ji     ( int index );
*   int             @hor_vsdf_get_DOFi   ( int index );
*   Hor_Matrix     *@hor_vsdf_get_z      ( int index );
*   int             @hor_vsdf_get_z_type ( int index );
*   void            @hor_vsdf_print_state(void)
*
*   Functions to read internal state of variable state dimension filter.
*
*   hor_vsdf_get_k() returns current time step k.
*   hor_vsdf_get_xf() returns the fixed part of the global state vector.
*   hor_vsdf_get_xf_type() returns the representation type of xf.
*   hor_vsdf_get_xd() returns the dynamic part of the global state vector.
*   hor_vsdf_get_xd_type() returns the representation type of xd.
*   hor_vsdf_get_xdlist() returns the whole history of the dynamic part of
*                         the global state vector as a list of pointers
*                         to @Hor_VSDF_XD_Def structures.
*   hor_vsdf_get_Pf() returns the covariance of the fixed global state.
*   hor_vsdf_get_Af() returns the inverse covariance of the fixed global state.
*   hor_vsdf_get_Pd() returns the covariance of the dynamic global state.
*   hor_vsdf_get_Ad() returns the inverse covariance of the dynamic global
*                     state.
*   hor_vsdf_get_J() returns the Chi^2 residual for the whole system.
*   hor_vsdf_get_DOF() returns the Chi^2 DOF for the whole system.
*   hor_vsdf_get_n() returns the current number of local states.
*   hor_vsdf_get_y() returns the specified local state vector.
*   hor_vsdf_get_y_type() returns the representation type of the specified y.
*   hor_vsdf_get_T() returns the covariance of the specified local state.
*   hor_vsdf_get_C() returns the inverse covariance of the specified local
*                    state.
*   hor_vsdf_get_obs_k() returns the time step of the last observation for
*                        the specified local state. If no observation has been
*                        made for it, -1 is returned.
*   hor_vsdf_get_data() returns the user data pointer passed at the last
*                       call to hor_vsdf_obs_...().
*   hor_vsdf_get_Ji() returns the Chi^2 residual for the specified local state.
*   hor_vsdf_get_DOFi() returns the Chi^2 DOF for the specified local state.
*   hor_vsdf_get_z()      returns the last measurement vector for the specified
*                         local state.
*   hor_vsdf_get_z_type() returns the last measurement vector type for the
*                         specified local state.
*   hor_vsdf_print_state() prints the current state.
********************/
int hor_vsdf_get_k(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_k)", HOR_FATAL );

   return vs->k;
}

Hor_Matrix *hor_vsdf_get_xf(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_xf)", HOR_FATAL );

   return vs->xf;
}

int hor_vsdf_get_xf_type(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_xf_type)", HOR_FATAL );

   return vs->xf_type;
}

Hor_Matrix *hor_vsdf_get_xd(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_xd)", HOR_FATAL );

   return vs->xd;
}

int hor_vsdf_get_xd_type(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_xd_type)", HOR_FATAL );

   return vs->xd_type;
}

Hor_List hor_vsdf_get_xdlist(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_xdlist)", HOR_FATAL );

   return vs->xdlist;
}

Hor_Matrix *hor_vsdf_get_Pf(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_Pf)", HOR_FATAL );

   if ( vs->xf == NULL )
      hor_error ( "no VSDF fixed global state (hor_vsdf_get_Pf)", HOR_FATAL );

   return vs->alpha_inv;
}

Hor_Matrix *hor_vsdf_get_Af(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_Af)", HOR_FATAL );

   if ( vs->xf == NULL )
      hor_error ( "no VSDF fixed global state (hor_vsdf_get_Af)", HOR_FATAL );

   return vs->Af;
}

Hor_Matrix *hor_vsdf_get_Pd(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_Pd)", HOR_FATAL );

   if ( vs->xd == NULL )
      hor_error ( "no VSDF dynamic global state (hor_vsdf_get_Pd)", HOR_FATAL);

   if ( vs->k == 1 )
      hor_error ( "dynamic global state covariance undefined initially (hor_vsdf_get_Pd)", HOR_FATAL );

   return vs->gamma_inv;
}

Hor_Matrix *hor_vsdf_get_Ad(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_Ad)", HOR_FATAL );

   if ( vs->xd == NULL ) return NULL;

   if ( vs->k == 1 )
      hor_error ( "dynamic global state covariance undefined initially (hor_vsdf_get_Ad)", HOR_FATAL );

   return vs->Ad;
}

double hor_vsdf_get_J(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_J)", HOR_FATAL );

   return vs->J->m[0][0];
}

int hor_vsdf_get_DOF(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_DOF)", HOR_FATAL );

   return vs->dof;
}

int hor_vsdf_get_n(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_n)", HOR_FATAL );

   return vs->n;
}

int hor_vsdf_get_max_ysize(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_max_ysize)", HOR_FATAL );

   return vs->max_ysize;
}

int hor_vsdf_get_max_zsize(void)
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_max_zsize)", HOR_FATAL );

   return vs->max_zsize;
}

Hor_Matrix *hor_vsdf_get_y ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_y)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (%d) (hor_vsdf_get_y)",
		  HOR_FATAL, index, vs->n );

   return vs->local_state[index].y;
}

int hor_vsdf_get_y_type ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_y_type)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (%d) (hor_vsdf_get_y_type)",
		  HOR_FATAL, index, vs->n );

   return vs->local_state[index].y_type;
}

Hor_Matrix *hor_vsdf_get_T ( int index )
{
   VSDF_State  *vs;
   Local_State *ls;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_T)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (%d) (hor_vsdf_get_T)",
		  HOR_FATAL, index, vs->n );

   ls = &vs->local_state[index];

   hor_matq_copy ( ls->C, work1 );
   if ( vs->xf != NULL && ls->k > 0 )
      hor_mat_increment ( work1,
	     hor_matq_prod3 ( ls->BfT, vs->alpha_inv, ls->Bf, work2, work3 ) );

   if ( vs->xd != NULL && ls->k == vs->k-1 )
      hor_mat_increment ( work1,
	     hor_matq_prod3 ( ls->BdT, vs->gamma_inv, ls->Bd, work2, work3 ) );

   if ( vs->xf != NULL && vs->xd != NULL && ls->k == vs->k-1 ) {
      hor_matq_prod3 ( ls->BfT, vs->beta, ls->Bd, work2, work3 );
      hor_mat_increment ( work1, hor_matq_scale ( work3, 2.0 ) );
   }

   hor_matq_prod3 ( ls->Cinv, work1, ls->Cinv, work2, work3 );
   if ( hor_errno != HORATIO_OK ) return NULL;

   return work3;
}

Hor_Matrix *hor_vsdf_get_C ( int index )
{
   VSDF_State  *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_C)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (%d) (hor_vsdf_get_C)",
		  HOR_FATAL, index, vs->n );

   return ( vs->local_state[index].C );
}

int hor_vsdf_get_obs_k ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_obs_k)", HOR_FATAL );

   if ( index < 0 || index >= vs->n )
      hor_error ( "illegal local state index %d (%d) (hor_vsdf_get_obs_k)",
		  HOR_FATAL, index, vs->n );

   return vs->local_state[index].k;
}

void *hor_vsdf_get_data ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_data)", HOR_FATAL );

   if ( index >= 0 && index < vs->n )
      return vs->local_state[index].user_data;
   else if ( index >= HOR_VSDF_UNINIT_OFFSET &&
	     index < HOR_VSDF_UNINIT_OFFSET + vs->nu )
   {
      Hor_List zlist = vs->uninit_state[index-HOR_VSDF_UNINIT_OFFSET].zlist;

      if ( zlist == NULL )
	 hor_error ( "no observations for state %d (hor_vsdf_get_data)",
		     HOR_FATAL, index );

      return ( ((Batch_Obs *) zlist->contents)->user_data );
   }
   else hor_error ( "illegal local state index %d (hor_vsdf_get_data)",
		    HOR_FATAL, index );

   return NULL;
}

Hor_Matrix *hor_vsdf_get_z ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_z)", HOR_FATAL );

   if ( index >= 0 && index <= vs->n )
      return vs->local_state[index].z;
   else if ( index >= HOR_VSDF_UNINIT_OFFSET &&
	     index < HOR_VSDF_UNINIT_OFFSET + vs->nu )
   {
      Hor_List zlist = vs->uninit_state[index-HOR_VSDF_UNINIT_OFFSET].zlist;

      if ( zlist == NULL )
	 hor_error ( "no observations for state %d (hor_vsdf_get_z)",
		     HOR_FATAL, index );

      return ( ((Batch_Obs *) zlist->contents)->z );
   }
   else hor_error ( "illegal local state index %d (hor_vsdf_get_z)",
		    HOR_FATAL, index );

   return NULL;
}

int hor_vsdf_get_z_type ( int index )
{
   VSDF_State *vs;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_get_z_type)", HOR_FATAL );

   if ( index >= 0 && index <= vs->n )
      return vs->local_state[index].z_type;
   else if ( index >= HOR_VSDF_UNINIT_OFFSET &&
	     index < HOR_VSDF_UNINIT_OFFSET + vs->nu )
   {
      Hor_List zlist = vs->uninit_state[index-HOR_VSDF_UNINIT_OFFSET].zlist;

      if ( zlist == NULL )
	 hor_error ( "no observations for state %d (hor_vsdf_get_z_type)",
		     HOR_FATAL, index );

      return ( ((Batch_Obs *) zlist->contents)->z_type );
   }
   else hor_error ( "illegal local state index %d (hor_vsdf_get_z_type)",
		    HOR_FATAL, index );

   return 0;
}

void hor_vsdf_print_state(void)
{
   VSDF_State  *vs;
   Local_State *ls;
   int          i;

   if ( (vs = current_vsdf) == NULL )
      hor_error ( "not initialised (hor_vsdf_print_state)", HOR_FATAL );

   if ( vs->xf != NULL )
   {
      hor_matq_transpose ( vs->xf, work1 );
      hor_mat_print ( work1, "fixed global state vector" );
      hor_mat_print ( vs->alpha_inv,  "fixed global state covariance" );
   }

   if ( vs->xd != NULL )
   {
      hor_matq_transpose ( vs->xd, work1 );
      hor_mat_print ( work1, "dynamic global state vector" );
      if ( vs->k > 1 )
	 hor_mat_print ( vs->gamma_inv, "dynamic global state covariance" );
   }

   for ( i = 0, ls = vs->local_state; i < vs->n; i++, ls++ )
   {
      hor_mat_print ( ls->y, "local state vector" );
      hor_mat_print ( hor_vsdf_get_T(i),  "local state covariance" );
   }

   hor_print ( "residual = %f on %d DOF\n", vs->J->m[0][0], vs->dof );
}
