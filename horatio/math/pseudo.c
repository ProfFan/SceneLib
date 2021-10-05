/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"

/*******************
*   Functions for evaluating the Moore-Penrose pseudo-inverse of a matrix
*   incrementally, one row at a time, using it to solve the matrix equation
*
*       A*X = B
*
*   for X given A and B, where A must have at least as many rows as columns
*   for the inverse to exist. If A is rectangular (more rows than columns)
*   the the solution is found in a least-squares sense, i.e. the resulting
*   X minimises |A*X - B|. This matrix formulation is equivalent to solving
*   the equation
*
*       A*x = b
*
*   where x and b are vectors, but the more general matrix formulation allows
*   for multiple vector solutions to different vector equations simultaneously.
*   The matrices A and B are provided incrementally, a row at a time.
********************/

typedef struct
{
   Hor_Matrix *ATA, *ATB;
} Pseudo_State;

static Hor_Assoc_List list = NULL;

static int max_Xrows = 0;
static Hor_Matrix *ATALUD = NULL, *ATALUDinv = NULL, *ATAinv = NULL;

static void pseudo_inv_free ( void *ptr )
{
   Pseudo_State *state = (Pseudo_State *) ptr;

   hor_mat_free_list ( state->ATB, state->ATA, NULL );
   hor_free ( ptr );
}

/*******************
*   void @hor_pseudo_inv_init ( Hor_Assoc_Label label, int Xrows, int Xcols )
*
*   Initialises a pseudo-inverse calculation with given (arbitrary) label,
*   and given sizes for solution matrix X.
********************/
void hor_pseudo_inv_init ( Hor_Assoc_Label label, int Xrows, int Xcols )
{
   Pseudo_State *new;

   if ( Xrows <= 0 || Xcols <= 0 )
      hor_error ( "matrix sizes must be > 0 (hor_pseudo_inv_init)", HOR_FATAL);

   if ( hor_assoc_find ( list, label ) != NULL )
      hor_pseudo_inv_free ( label );

   new = hor_malloc_type(Pseudo_State);
   new->ATA = hor_mats_zero ( Xrows, Xrows );
   new->ATB = hor_mats_zero ( Xrows, Xcols );

   if ( Xrows > max_Xrows )
   {
      if ( max_Xrows > 0 )
	 hor_mat_free_list ( ATAinv, ATALUDinv, ATALUD, NULL );

      ATALUD    = hor_mat_alloc ( Xrows, Xrows );
      ATALUDinv = hor_mat_alloc ( Xrows, Xrows );
      ATAinv    = hor_mat_alloc ( Xrows, Xrows );
      max_Xrows = Xrows;
   }

   list = hor_assoc_insert ( list, label, (void *) new );
}

/*******************
*   void @hor_pseudo_inv_free ( Hor_Assoc_Label label )
*
*   Frees data associated with a pseudo-inverse calculation.
********************/
void hor_pseudo_inv_free ( Hor_Assoc_Label label )
{
   if ( hor_assoc_remove ( &list, label, pseudo_inv_free ) == HOR_ASSOC_ERROR )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_free)",
		  HOR_FATAL );
}

/*******************
*   void @hor_pseudo_inv_reset ( Hor_Assoc_Label label )
*
*   Resets the matrices associated with a pseudo-inverse calculation to zero,
*   equivalent to the situation just after calling hor_pseudo_inv_init().
********************/
void hor_pseudo_inv_reset ( Hor_Assoc_Label label )
{
   Pseudo_State *state;

   state = (Pseudo_State *) hor_assoc_find ( list, label );
   if ( state == NULL )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_reset)",
		  HOR_FATAL );

   hor_matq_zero ( state->ATA );
   hor_matq_zero ( state->ATB );
}

static void pseudo_inv_inc ( Pseudo_State *state, double *Arow, double *Brow )
{
   int      Xrows, Xcols, i, j;
   double **ATAm, **ATBm;

   Xrows = state->ATA->rows;
   Xcols = state->ATB->cols;
   ATAm = state->ATA->m;
   ATBm = state->ATB->m;

   /* increment ATA */
   for ( i = 0; i < Xrows; i++ )
      for ( j = 0; j < Xrows; j++ ) ATAm[i][j] += Arow[i]*Arow[j];

   /* increment ATB */
   for ( i = 0; i < Xrows; i++ )
      for ( j = 0; j < Xcols; j++ ) ATBm[i][j] += Arow[i]*Brow[j];
}

/*******************
*   void @hor_pseudo_inv_data ( Hor_Assoc_Label label,
*                              double *Arow, double *Brow )
*
*   Provides new rows of the matrices A and B.
********************/
void hor_pseudo_inv_data ( Hor_Assoc_Label label, double *Arow, double *Brow )
{
   Pseudo_State *state;

   state = (Pseudo_State *) hor_assoc_find ( list, label );
   if ( state == NULL )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_data)",
		  HOR_FATAL );

   pseudo_inv_inc ( state, Arow, Brow );
}

/*******************
*   void @hor_pseudo_inv_data_m ( Hor_Assoc_Label label,
*                                Hor_Matrix *Amat, Hor_Matrix *Bmat )
*
*   Provides new parts of matrices A and B. Amat and Bmat must have the same
*   number of rows.
********************/
void hor_pseudo_inv_data_m ( Hor_Assoc_Label label,
			     Hor_Matrix *Amat, Hor_Matrix *Bmat )
{
   Pseudo_State *state;
   int           Xrows, Xcols, j;

   state = (Pseudo_State *) hor_assoc_find ( list, label );
   if ( state == NULL )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_data_m)",
		  HOR_FATAL );

   Xrows = state->ATA->rows;
   Xcols = state->ATB->cols;

   if ( Amat->cols != Xrows )
      hor_error ( "illegal A-matrix size (hor_pseudo_inv_data_m)", HOR_FATAL );

   if ( Bmat->cols != Xcols )
      hor_error ( "illegal B-matrix size (hor_pseudo_inv_data_m)", HOR_FATAL );

   if ( Amat->rows != Bmat->rows )
      hor_error ( "incompatible A/B matrix sizes (hor_pseudo_inv_data_m)",
		  HOR_FATAL );

   for ( j = 0; j < Amat->rows; j++ )
      pseudo_inv_inc ( state, Amat->m[j], Bmat->m[j] );
}

/*******************
*   void @hor_pseudo_inv_data_v ( Hor_Assoc_Label label, ... )
*
*   The same as hor_pseudo_inv_data(), except that the data is provided in a
*   variable argument list, the elements of the A row being passed before
*   the B row.
********************/
void hor_pseudo_inv_data_v ( Hor_Assoc_Label label, ... )
{
   va_list       ap;
   Pseudo_State *state;
   int           Xrows, Xcols, i;
   double       *Arow, *Brow;

   state = (Pseudo_State *) hor_assoc_find ( list, label );
   if ( state == NULL )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_data_v)",
		  HOR_FATAL );

   Xrows = state->ATA->rows;
   Xcols = state->ATB->cols;
   Arow = hor_malloc_ntype ( double, Xrows );
   Brow = hor_malloc_ntype ( double, Xcols );

   va_start ( ap, label );
   for ( i = 0; i < Xrows; i++ ) Arow[i] = va_arg ( ap, double );
   for ( i = 0; i < Xcols; i++ ) Brow[i] = va_arg ( ap, double );
   va_end(ap);

   pseudo_inv_inc ( state, Arow, Brow );
   hor_free_multiple ( (void *) Brow, (void *) Arow, NULL );
}

/*******************
*   Hor_Matrix *@hor_pseudo_inv_solve ( Hor_Assoc_Label label )
*
*   Solves the equation A*X=B for X and returns a pointer to X. This should
*   only be called when enough rows of A and B have been passed using
*   hor_pseudo_inv_data()/hor_pseudo_inv_data_v(), the minimum number being
*   the number of rows in X (the Xrows argument to hor_pseudo_inv_init()).
*   The returned matrix should be freed using hor_mat_free() after use.
********************/
Hor_Matrix *hor_pseudo_inv_solve ( Hor_Assoc_Label label )
{
   Pseudo_State *state;

   state = (Pseudo_State *) hor_assoc_find ( list, label );
   if ( state == NULL )
      hor_error ( "illegal pseudo-inverse label (hor_pseudo_inv_solve)",
		  HOR_FATAL );

   if ( hor_matq_inv ( state->ATA, ATALUD, ATALUDinv, ATAinv ) == NULL )
      return NULL;

   return ( hor_mats_prod2 ( ATAinv, state->ATB ) );
}
