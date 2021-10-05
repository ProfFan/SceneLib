/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. 
		  modified by ddjian Oct 94 to add scatter matrix */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"

/*******************
*   Functions for determining the eigenvalues and eigenvectors of a real
*   symmetric matrix A^T*A calculated from the rows of A which are provided
*   incrementally. In other words, we solve the equation
*
*   (A^T*A)*x = lambda*x
*
*   for eigenvalues lambda and eigenvectors x.
********************/

static int max_Awidth = 0;
static double *offdiag;

static Hor_Matrix *eigen_init ( int Awidth )
{
   Hor_Matrix *ATA;

   ATA = hor_mats_zero ( Awidth, Awidth );

   if ( Awidth > max_Awidth )
   {
      if ( max_Awidth > 0 ) hor_free ( (void *) offdiag );

      offdiag    = hor_malloc_ntype ( double, Awidth );
      max_Awidth = Awidth;
   }

   return ATA;
}

static void eigen_free ( void *ptr )
{
   Hor_Matrix *ATA = (Hor_Matrix *) ptr;

   hor_mat_free ( ATA );
}

static void eigen_inc ( Hor_Matrix *ATA, double *Arow )
{
   int      Awidth, i, j;
   double **ATAm;

   Awidth = ATA->rows;
   ATAm   = ATA->m;

   /* increment ATA */
   for ( i = 0; i < Awidth; i++ )
      for ( j = 0; j < Awidth; j++ ) ATAm[i][j] += Arow[i]*Arow[j];
}

static void eigen_data_m ( Hor_Matrix *Amat, Hor_Matrix *ATA )
{
   int i;

   if ( Amat->cols != ATA->rows )
      hor_error ( "illegal matrix size (eigen_data_m)", HOR_FATAL );

   for ( i = 0; i < Amat->rows; i++ ) eigen_inc ( ATA, Amat->m[i] );
}

static void eigen_data_v ( Hor_Matrix *ATA, va_list *aptr )
{
   int         Awidth, i;
   double     *Arow;

   Awidth = ATA->rows;
   Arow = hor_malloc_ntype ( double, Awidth );
   for ( i = 0; i < Awidth; i++ ) Arow[i] = va_arg ( *aptr, double );

   eigen_inc ( ATA, Arow );
   hor_free ( (void *) Arow );
}

static void eigen_solve ( Hor_Matrix *ATA, Hor_Matrix *eigenvectors,
			double *eigenvalues )
{
   if ( hor_matq_copy ( ATA, eigenvectors ) == NULL )
   {
      hor_perror ( "(eigen_solve)" );
      hor_error ( "aborting", HOR_FATAL );
   }

   hor_matq_tred2 ( eigenvectors, eigenvalues, offdiag );
   hor_matq_tqli ( eigenvalues, offdiag, eigenvectors );
   hor_matq_eigsrt ( eigenvalues, eigenvectors );
}

static Hor_Assoc_List eigen_list = NULL;

/*******************
*   void @hor_eigen_init ( Hor_Assoc_Label label, int Awidth )
*
*   Initialises an eigensystem calculation with given (arbitrary) label,
*   and given width of matrix A.
********************/
void hor_eigen_init ( Hor_Assoc_Label label, int Awidth )
{
   Hor_Matrix *ATA;

   if ( Awidth <= 0 )
      hor_error ( "matrix width must be > 0 (hor_eigen_init)", HOR_FATAL );

   if ( hor_assoc_find ( eigen_list, label ) != NULL )
      hor_eigen_free ( label );

   ATA = eigen_init ( Awidth );
   eigen_list = hor_assoc_insert ( eigen_list, label, (void *) ATA );
}

/*******************
*   void @hor_eigen_free ( Hor_Assoc_Label label )
*
*   Frees data associated with an eigensystem calculation.
********************/
void hor_eigen_free ( Hor_Assoc_Label label )
{
   if ( hor_assoc_remove ( &eigen_list, label, eigen_free ) == HOR_ASSOC_ERROR )
      hor_error ( "illegal eigensystem label (hor_eigen_free)", HOR_FATAL );
}

/*******************
*   void @hor_eigen_reset ( Hor_Assoc_Label label )
*
*   Resets the matrices associated with an eigensystem calculation to zero,
*   equivalent to the situation just after calling hor_eigen_init().
********************/
void hor_eigen_reset ( Hor_Assoc_Label label )
{
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_reset)", HOR_FATAL );

   hor_matq_zero ( ATA );
}

/*******************
*   void @hor_eigen_data ( Hor_Assoc_Label label, double *Arow )
*
*   Provides new row of matrix A, and hence increments ATA by Arow^T*Arow.
********************/
void hor_eigen_data ( Hor_Assoc_Label label, double *Arow )
{
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_data)", HOR_FATAL );

   eigen_inc ( ATA, Arow );
}

/*******************
*   void @hor_eigen_data_m ( Hor_Assoc_Label label, Hor_Matrix *Amat )
*
*   Provides new part of matrix A, and hence increments ATA by Amat^T*Amat.
********************/
void hor_eigen_data_m ( Hor_Assoc_Label label, Hor_Matrix *Amat )
{
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_data_m)", HOR_FATAL );

   eigen_data_m ( Amat, ATA );
}

/*******************
*   void @hor_eigen_data_v ( Hor_Assoc_Label label, ... )
*
*   The same as hor_eigen_data(), except that the data is provided in a
*   variable argument list, one number for each element of the new row of A.
********************/
void hor_eigen_data_v ( Hor_Assoc_Label label, ... )
{
   va_list     ap;
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_data_v)", HOR_FATAL );

   va_start ( ap, label );
   eigen_data_v  ( ATA, &ap );
   va_end(ap);
}

/*******************
*   void @hor_eigen_solve ( Hor_Assoc_Label label,
*                          Hor_Matrix *eigenvectors, double *eigenvalues )
*
*   Finds the eigenvalues and eigenvectors of the matrix A^T*A defined by
*   previous calls to hor_eigen_data() and/or hor_eigen_data_v().
*   The eigenvectors are written into the columns of the provided matrix,
*   and the eigenvalues, sorted into descending order, in the provided array.
*   the size of the eigenvectors matrix must be at least Awidth by Awidth,
*   where Awidth is the width of the matrix A.
********************/
void hor_eigen_solve ( Hor_Assoc_Label label,
		       Hor_Matrix *eigenvectors, double *eigenvalues )
{
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_solve)", HOR_FATAL );

   eigen_solve ( ATA, eigenvectors, eigenvalues );
}


/*******************
*   Hor_Matrix *@hor_eigen_ATA ( Hor_Assoc_Label label )
*
*   Returns a copy of the latest matrix used for eigen-decomposition.
********************/
Hor_Matrix *hor_eigen_ATA ( Hor_Assoc_Label label )
{
   Hor_Matrix *ATA = (Hor_Matrix *) hor_assoc_find ( eigen_list, label );

   if ( ATA == NULL )
      hor_error ( "illegal eigensystem label (hor_eigen_ATA)", HOR_FATAL );

   return hor_mats_copy ( ATA );
}

/*******************
*   Functions for determining the eigenvalues and eigenvectors of a real
*   scatter matrix Sc. The rows of Sc are computed incrementally from
*   each point (Scrow). In other words, we solve the equation
*
*   Sc*x = lambda*x
*
*   for eigenvalues lambda and eigenvectors x.
********************/

typedef struct
{
   Hor_Matrix *Sc;
   int         no_points;
   double     *centroid;
} Scatter_State;

Hor_Assoc_List scatter_list = NULL;

static void scatter_free ( void *ptr )
{
   Scatter_State *state = (Scatter_State *) ptr;

   hor_mat_free ( state->Sc );
   hor_free ( (void *) state->centroid );
   hor_free ( ptr );
}

static void scatter_inc ( Scatter_State *state, double *Scrow )
{
   int Scwidth, i;
   double coef, *Brow, a, b, n;
   Hor_Matrix *M;

   state->no_points++;
   n = (double) state->no_points;
   coef = 1./ sqrt ( n );
   Scwidth = state->Sc->rows;
   Brow = hor_malloc_ntype ( double, Scwidth );
   for ( i = 0; i < Scwidth; i++ )
      Brow [i] = coef * ( Scrow [i] - state->centroid [i] );
   eigen_inc ( state->Sc, Brow );
   b = ( n - 1. ) / n;
   M = hor_mats_scale ( state->Sc, b );
   hor_mat_free ( state->Sc );
   state->Sc = M;
   a = 1. / n;
   for ( i = 0; i < Scwidth; i++ )
      state->centroid [i] = a * Scrow [i] + b * state->centroid [i]; 
   hor_free ( (void *) Brow );
}

/*******************
*   void @hor_scatter_init ( Hor_Assoc_Label label, int Scwidth )
*
*   Initialises a scatter matrix Sc with given (arbitrary) label,
*   and given the width of Sc.
********************/
void hor_scatter_init ( Hor_Assoc_Label label, int Scwidth )
{
   int i;
   Scatter_State *new;

   if ( Scwidth <= 0 )
      hor_error ( "matrix width must be > 0 (hor_scatter_init)", HOR_FATAL );

   if ( hor_assoc_find ( scatter_list, label ) != NULL )
      hor_scatter_free ( label );

   new = hor_malloc_type ( Scatter_State );
   new->Sc = eigen_init ( Scwidth );
   new->no_points = 0;
   new->centroid = hor_malloc_ntype ( double, Scwidth );
   for ( i = 0; i < Scwidth; i++ )
      new->centroid [i] = 0;
   scatter_list = hor_assoc_insert ( scatter_list, label, (void *) new );
}

/*******************
*   void @hor_scatter_free ( Hor_Assoc_Label label )
*
*   Frees data associated with a scatter matrix.
********************/
void hor_scatter_free ( Hor_Assoc_Label label )
{
   if ( hor_assoc_remove ( &scatter_list, label, scatter_free ) == HOR_ASSOC_ERROR )
      hor_error ( "illegal scatter matrix label (hor_scatter_free)", HOR_FATAL );
}

/*******************
*   void @hor_scatter_reset ( Hor_Assoc_Label label )
*
*   Resets a scatter matrix to zero, equivalent to the situation just after 
*   calling hor_scatter_init().
********************/
void hor_scatter_reset ( Hor_Assoc_Label label )
{
   int Scwidth, i;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_reset)", HOR_FATAL );

   hor_matq_zero ( state->Sc );
   state->no_points = 0;
   Scwidth = state->Sc->rows;
   for ( i = 0; i < Scwidth; i++ )
      state->centroid [i] = 0;
}

/*******************
*   void @hor_scatter_data ( Hor_Assoc_Label label, double *Scrow )
*
*   Provides new point to be included in scatter matrix.
********************/
void hor_scatter_data ( Hor_Assoc_Label label, double *Scrow )
{
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_data)", HOR_FATAL );

   scatter_inc ( state, Scrow );
}

/*******************
*   void @hor_scatter_data_m ( Hor_Assoc_Label label, Hor_Matrix *Scmat )
*
*   Provides new matrix of points for scatter matrix Sc.
********************/
void hor_scatter_data_m ( Hor_Assoc_Label label, Hor_Matrix *Scmat )
{
   int i, Xrows;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_data_m)", HOR_FATAL );

   Xrows = state->Sc->rows;

   if ( Scmat->cols != Xrows )
      hor_error ( "illegal A-matrix size (hor_scatter_data_m)", HOR_FATAL );

   for ( i = 0; i < Scmat->rows; i++ ) 
      scatter_inc ( state, Scmat->m[i] );
}

/*******************
*   void @hor_scatter_data_v ( Hor_Assoc_Label label, ... )
*
*   The same as hor_scatter_data(), except that the data is provided in a
*   variable argument list, one number for each new point coordinate.
********************/
void hor_scatter_data_v ( Hor_Assoc_Label label, ... )
{
   int Xrows, i;
   va_list     ap;
   double       *Scrow;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_data_v)", HOR_FATAL );

   Xrows = state->Sc->rows;
   Scrow = hor_malloc_ntype ( double, Xrows );

   va_start ( ap, label );
   for ( i = 0; i < Xrows; i++ ) Scrow[i] = va_arg ( ap, double );
   va_end(ap);

   scatter_inc ( state, Scrow );
   hor_free ( (void *) Scrow );
}

/*******************
*   void @hor_scatter_solve ( Hor_Assoc_Label label,
*                          Hor_Matrix *eigenvectors, 
*                          double *eigenvalues,
*                          double *centroid )
*
*   Finds the eigenvalues, eigenvectors and centroid of scatter matrix Sc  
*   defined by previous calls to hor_scatter_data() and/or hor_scatter_data_v().
*   The eigenvectors are written into the columns of the provided matrix,
*   and the eigenvalues, sorted into descending order, in the provided array.
*   The size of the eigenvectors matrix must be at least Scwidth by Scwidth,
*   where Scwidth is the width of the scatter matrix Sc.
********************/
void hor_scatter_solve ( Hor_Assoc_Label label,
			Hor_Matrix *eigenvectors,
			double *eigenvalues,
			double *centroid )
{
   int Scwidth, i;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_solve)", HOR_FATAL );

   eigen_solve ( state->Sc, eigenvectors, eigenvalues );
   Scwidth = state->Sc->rows;
   for ( i = 0; i < Scwidth; i++ )
      centroid [i] = state->centroid [i];
}

/*******************
*   int @hor_scatter_dim ( Hor_Assoc_Label label )
*
*   Returns the dimension of scatter matrix Sc.
********************/
int hor_scatter_dim ( Hor_Assoc_Label label )
{
   int Scwidth;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_dim)", HOR_FATAL );

   Scwidth = state->Sc->rows;
   return Scwidth;
}

/*******************
*   int @hor_scatter_get_n ( Hor_Assoc_Label label )
*
*   Returns the number of points currently stored in scatter matrix Sc.
********************/
int hor_scatter_get_n ( Hor_Assoc_Label label )
{
   int n;
   Scatter_State *state = (Scatter_State *) hor_assoc_find ( scatter_list, label );

   if ( state == NULL )
      hor_error ( "illegal scatter matrix label (hor_scatter_get_n)", HOR_FATAL );

   n = state->no_points;
   return n;
}
