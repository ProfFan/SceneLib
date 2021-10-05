/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
              and Jason Merron (jasm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <stdarg.h>
#ifdef THREEL_C
#include "eyelib.h"
#endif /* THREEL_C */

#include "horatio/global.h"
#include "horatio/math.h"

#define EPSILON 1.0e-100

/*******************
*   Hor_Matrix *@hor_mat_alloc ( int rows, int cols )
*   void @hor_mat_free      ( Hor_Matrix *M )
*   void @hor_mat_free_list ( Hor_Matrix *M, ... )
*
*   hor_mat_alloc() returns a matrix with a given number of rows and columns.
*   hor_mat_free() frees data associated with a matrix.
*   hor_mat_free_list() frees all matrices in the NULL-terminated list.
********************/
Hor_Matrix *hor_mat_alloc ( int rows, int cols )
{
   Hor_Matrix *result;
   int     r;

   if ( rows <= 0 || cols <= 0 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   result = hor_malloc_type ( Hor_Matrix );
   if ( result == NULL ) {
      hor_errno = HOR_MATH_ALLOCATION_FAILED;
      return NULL;
   }

   result->rsize = result->rows = rows;
   result->csize = result->cols = cols;
   result->m     = hor_malloc_ntype ( double *, rows );
   if ( result->m == NULL ) {
      hor_errno = HOR_MATH_ALLOCATION_FAILED;
      hor_free ( (void *) result );
      return NULL;
   }

   result->m[0] = hor_malloc_ntype ( double, rows*cols );
   if ( result->m[0] == NULL ) {
      hor_errno = HOR_MATH_ALLOCATION_FAILED;
      hor_free ( (void *) result->m );
      hor_free ( (void *) result );
      return NULL;
   }

   for ( r = 1; r < rows; r++ )
      result->m[r] = result->m[r-1] + cols;

   return result;
}

void hor_mat_free ( Hor_Matrix *M )
{
   hor_free ( (void *) M->m[0] );
   hor_free ( (void *) M->m );
   hor_free ( (void *) M );
}

void hor_mat_free_list ( Hor_Matrix *M, ... )
{
   va_list ap;

   if ( M == NULL ) return;

   hor_mat_free ( M );
   va_start ( ap, M );
   for(;;)
   {
      M = va_arg ( ap, Hor_Matrix * );
      if ( M == NULL ) break;

      hor_mat_free ( M );
   }

   va_end(ap);
}

static void mat_fill ( double **M, int rows, int cols, va_list *aptr )
{
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 M[r][c] = va_arg ( *aptr, double );
}

static void mat_copy ( double **Mm, double **Nm, int rows, int cols,
		       int Mcols, int Ncols )
{
#ifdef EYELIB_H
   copyimagesf ( rows, cols, (float *) &Nm[0][0], (float *) &Mm[0][0],
		 Ncols, Mcols );
#else
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 Nm[r][c] = Mm[r][c];
#endif /* EYELIB_H */
}

static void mat_add2 ( double **A, double **B, double **C,
		       int rows, int cols, int Acols, int Bcols, int Ccols )
{
#ifdef EYELIB_H
   addimage3sf ( rows, cols, (float *) &C[0][0], (float *) &A[0][0],
		 (float *) &B[0][0], Ccols, Acols, Bcols );
#else
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 C[r][c] = A[r][c] + B[r][c];
#endif /* EYELIB_H */
}

static void mat_sub ( double **A, double **B, double **C,
		      int rows, int cols, int Acols, int Bcols, int Ccols )
{
#ifdef EYELIB_H
   subimage3sf ( rows, cols, (float *) &C[0][0], (float *) &A[0][0],
		 (float *) &B[0][0], Ccols, Acols, Bcols );
#else
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 C[r][c] = A[r][c] - B[r][c];
#endif /* EYELIB_H */
}

static void mat_prod2 ( double **A, double **B, double **C,
		        int rows, int cols, int common )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < common; i++ )
	    sum += A[r][i]*B[i][c];

	 C[r][c] = sum;
      }
}

static void mat_scale ( double **M, int rows, int cols, double factor,
		        int Mcols )
{
#ifdef EYELIB_H
   mulalphaimagesf ( rows, cols, (float) factor, (float *) &M[0][0], Mcols );
#else
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 M[r][c] *= factor;
#endif /* EYELIB_H */
}

static void mat_transpose ( double **A, double **AT, int rows, int cols )
{
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 AT[c][r] = A[r][c];
}

/* static Hor_Bool mat_lud()

      Performs matrix lower-upper decomposition M=LL^T assuming M is symmetric
      and positive definite. Both lower and upper triangular parts of M_LUD are
      filled in. */
static Hor_Bool mat_lud ( double **M,     /* square input matrix */
			  double **M_LUD, /* result lower-triangular matrix */
			  int      size ) /* size of M (square) */
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < size; r++ )
   {
      /* calculate off-diagonal elements on row r */
      for ( c = 0; c < r; c++ )
      {
	 sum = M[r][c];
	 for ( i = 0; i < c; i++ )
	    sum -= M_LUD[c][i]*M_LUD[r][i];

	 if ( fabs(M_LUD[c][c]) < EPSILON ) {
	    hor_errno = HOR_MATH_MATRIX_SINGULAR;
	    return HOR_FALSE;
	 }

	 M_LUD[r][c] = M_LUD[c][r] = sum/M_LUD[c][c];
      }

      /* calculate r'th diagonal element */
      sum = M[r][r];
      for ( i = 0; i < r; i++ )
	 sum -= M_LUD[r][i]*M_LUD[r][i];

      if ( sum < 0 ) {
	 hor_errno = HOR_MATH_MATRIX_NOT_POS_DEFINITE;
	 return HOR_FALSE;
      }

      M_LUD[r][r] = sqrt(sum);
   }

   return HOR_TRUE;
}

/* static Hor_Bool mat_lud_inv()

      Inverts lower-triangular matrix. Fills in lower and upper triangular
      parts. */
static Hor_Bool mat_lud_inv ( double **M_LUD, double **M_LUD_inv, int size )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < size; r++ )
   {
      if ( fabs(M_LUD[r][r]) < EPSILON ) {
	 hor_errno = HOR_MATH_MATRIX_SINGULAR;
	 return HOR_FALSE;
      }
      /* calculate r'th diagonal element */
      M_LUD_inv[r][r] = 1.0/M_LUD[r][r];

      /* calculate off-diagonal elements on row r */
      for ( c = r-1; c >= 0; c-- )
      {
	 sum = 0.0;
	 for ( i = c+1; i <= r; i++ )
	    sum -= M_LUD_inv[r][i]*M_LUD[i][c];

	 M_LUD_inv[r][c] = M_LUD_inv[c][r] = sum/M_LUD[c][c];
      }
   }

   return HOR_TRUE;
}

/* static void mat_prod_lu ( double **M_LUD, double **M_LUD_prod, int size )

      multiplies lower and upper triangles of matrix together. */
static void mat_prod_lu ( double **M_LUD, double **M_LUD_prod, int size )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < size; r++ )
      for ( c = 0; c <= r; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i <= hor_min2(c,r); i++ )
	    sum += M_LUD[r][i]*M_LUD[c][i];

	 M_LUD_prod[r][c] = M_LUD_prod[c][r] = sum;
      }
}

/* static void mat_prod_ul ( double **M_LUD, double **M_LUD_prod, int size )

      multiplies upper and lower triangles of matrix together. */
static void mat_prod_ul ( double **M_LUD, double **M_LUD_prod, int size )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < size; r++ )
      for ( c = r; c < size; c++ )
      {
	 sum = 0.0;
	 for ( i = hor_max2(c,r); i < size; i++ )
	    sum += M_LUD[i][r]*M_LUD[i][c];

	 M_LUD_prod[r][c] = M_LUD_prod[c][r] = sum;
      }
}

static void mat_zero ( double **M, int rows, int cols, int Mcols )
{
#ifdef EYELIB_H
   vfillimagesf ( rows, cols, 0.0F, (float *) &M[0][0], Mcols );
#else
   int r, c;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
	 M[r][c] = 0.0;
#endif /* EYELIB_H */
}

static void mat_identity ( double **M, int size, int Mcols )
{
   int r, c;

#ifdef EYELIB_H
   vfillimagesf ( size, size, 0.0F, (float *) &M[0][0], Mcols );
#else
   /* fill in off-diagonal entries in matrix with zeroes */
   for ( r = 0; r < size; r++ )
      for ( c = 0; c < r; c++ )
	 M[r][c] = M[c][r] = 0.0;
#endif /* EYELIB_H */

   /* fill in diagonal entries in matrix */
   for ( r = 0; r < size; r++ )
      M[r][r] = 1.0;
}

static void mat_zero_lower ( double **M, int size )
{
   int r, c;

   for ( r = 1; r < size; r++ )
      for ( c = 0; c < r; c++ )
	 M[r][c] = 0.0;
}

static void mat_zero_upper ( double **M, int size )
{
   int r, c;

   for ( r = 0; r < size-1; r++ )
      for ( c = r+1; c < size; c++ )
	 M[r][c] = 0.0;
}

static void mat_diagonal ( double **M, int size, va_list *aptr )
{
   int r, c;

#ifdef EYELIB_H
   vfillimagesf ( size, size, 0.0F, (float *) &M[0][0], Mcols );
#else
   /* fill in off-diagonal entries in matrix with zeroes */
   for ( r = 0; r < size; r++ )
      for ( c = 0; c < r; c++ )
	 M[r][c] = M[c][r] = 0.0;
#endif /* EYELIB_H */

   /* fill in diagonal entries in matrix */
   for ( r = 0; r < size; r++ )
      M[r][r] = va_arg ( *aptr, double );
}

/*******************
*   "Slow" matrix routines that allocate space where required.
*   If an error occurs (e.g. inverting a singular matrix), the global variable
*   hor_errno is set to one of the values found in global.h and NULL is
*   returned. If a matrix argument to a function is NULL, NULL is returned,
*   on the assumption that another error has previously happened.
*   The function hor_perror() can be called to print an error message.
********************/

/*******************
*   Hor_Matrix *@hor_mats_fill ( int rows, int cols, ... )
*
*   Creates a matrix with given dimensions and fills it with values passed in
*   variable argument list. Values are read in "raster scan" order, i.e.
*   columns first, then rows, from top-left to bottom right. The matrix is
*   returned. NOTE: cast all numerical arguments to double if necessary.
********************/
Hor_Matrix *hor_mats_fill ( int rows, int cols, ... )
{
   va_list  ap;
   Hor_Matrix  *M;

   M = hor_mat_alloc ( rows, cols );
   if ( M == NULL ) return NULL;

   va_start ( ap, cols );
   mat_fill ( M->m, rows, cols, &ap );
   va_end(ap);
   return M;
}

/*******************
*   Hor_Matrix *@hor_mats_copy ( Hor_Matrix *M )
*
*   Creates and returns a copy of a given matrix.
********************/
Hor_Matrix *hor_mats_copy ( Hor_Matrix *M )
{
   Hor_Matrix *N;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   N = hor_mat_alloc ( M->rows, M->cols );
   if ( M == NULL ) return NULL;

   mat_copy ( M->m, N->m, M->rows, M->cols, M->csize, N->csize );
   return N;
}

/*******************
*   Hor_Matrix *@hor_mats_add2 ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Creates a matrix C the same size as A & B and calculates C=A+B,
*   returning C.
********************/
Hor_Matrix *hor_mats_add2 ( Hor_Matrix *A, Hor_Matrix *B )
{
   Hor_Matrix *C;

   if ( A == NULL || B == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_same_size2 ( A, B ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   C = hor_mat_alloc ( A->rows, A->cols );
   if ( C == NULL ) return NULL;

   mat_add2 (A->m, B->m, C->m, A->rows, A->cols, A->csize, B->csize, C->csize);
   return C;
}

/*******************
*   Hor_Matrix *@hor_mats_sub ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Creates a matrix C the same size as A & B and calculates C=A-B,
*   returning C.
********************/
Hor_Matrix *hor_mats_sub ( Hor_Matrix *A, Hor_Matrix *B )
{
   Hor_Matrix *C;

   if ( A == NULL || B == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_same_size2 ( A, B ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   C = hor_mat_alloc ( A->rows, A->cols );
   if ( C == NULL ) return NULL;

   mat_sub ( A->m, B->m, C->m, A->rows, A->cols, A->csize, B->csize, C->csize);
   return C;
}

/*******************
*   Hor_Matrix *@hor_mats_prod2 ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Creates a matrix C and calculates C=AB, returning C.
********************/
Hor_Matrix *hor_mats_prod2 ( Hor_Matrix *A, Hor_Matrix *B )
{
   Hor_Matrix *C;

   if ( A == NULL || B == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( A->cols != B->rows ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   C = hor_mat_alloc ( A->rows, B->cols );
   if ( C == NULL ) return NULL;

   mat_prod2 ( A->m, B->m, C->m, A->rows, B->cols, A->cols );
   return C;
}

/*******************
*   Hor_Matrix *@hor_mats_prod3 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
*
*   Creates a matrix D and calculates D=ABC, returning D.
********************/
Hor_Matrix *hor_mats_prod3 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
{
   Hor_Matrix *WSP, *D;

   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_test_size ( B, A->cols, C->rows ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   D   = hor_mat_alloc ( A->rows, C->cols );
   if ( D == NULL ) return NULL;

   WSP = hor_mat_alloc ( A->rows, B->cols );
   if ( WSP == NULL ) { hor_mat_free ( D ); return NULL; }

   mat_prod2 ( A->m,   B->m, WSP->m, A->rows,   B->cols, A->cols );
   mat_prod2 ( WSP->m, C->m, D->m,   WSP->rows, C->cols, WSP->cols );
   hor_mat_free ( WSP );
   return D;
}

/*******************
*   Hor_Matrix *@hor_mats_scale ( Hor_Matrix *M, double factor )
*
*   Produces a copy of the input matrix M with each element multiplied by the
*   given scalar factor.
********************/
Hor_Matrix *hor_mats_scale ( Hor_Matrix *M, double factor )
{
   Hor_Matrix *MS;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   MS = hor_mats_copy ( M );
   mat_scale ( MS->m, MS->rows, MS->cols, factor, MS->csize );
   return MS;
}

/*******************
*   Hor_Matrix *@hor_mats_transpose ( Hor_Matrix *M )
*
*   Creates a matrix MT and calculates MT as the transpose of M, returning MT.
********************/
Hor_Matrix *hor_mats_transpose ( Hor_Matrix *M )
{
   Hor_Matrix *MT;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   MT = hor_mat_alloc ( M->cols, M->rows );
   if ( MT == NULL ) return NULL;

   mat_transpose ( M->m, MT->m, M->rows, M->cols );
   return MT;
}

/*******************
*   Hor_Matrix *@hor_mats_lud ( Hor_Matrix *M )
*
*   Creates a matrix M_LUD and calculates M_LUD as the matrix lower-upper
*   decomposition of M, filling in both lower and upper triangular.
*   M_LUD is returned.
********************/
Hor_Matrix *hor_mats_lud ( Hor_Matrix *M )
{
   Hor_Matrix *M_LUD;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M_LUD = hor_mat_alloc ( M->rows, M->cols );
   if ( M_LUD == NULL ) return NULL;

   if ( mat_lud ( M->m, M_LUD->m, M->rows ) )
      return M_LUD;
   else {
      hor_mat_free ( M_LUD );
      return NULL;
   }
}

/*******************
*   Hor_Matrix *@hor_mats_lud_inv ( Hor_Matrix *M_LUD )
*
*   Creates a matrix M_LUD_inv and calculates it as the inverse of M_LUD,
*   assuming M_LUD is in lower-upper decomposed form.
********************/
Hor_Matrix *hor_mats_lud_inv ( Hor_Matrix *M_LUD )
{
   Hor_Matrix *M_LUD_inv;

   if ( M_LUD == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M_LUD_inv = hor_mat_alloc ( M_LUD->rows, M_LUD->cols );
   if ( M_LUD_inv == NULL ) return NULL;

   if ( mat_lud_inv ( M_LUD->m, M_LUD_inv->m, M_LUD->rows ) )
      return M_LUD_inv;
   else {
      hor_mat_free ( M_LUD_inv );
      return NULL;
   }
}

/*******************
*   Hor_Matrix *@hor_mats_prod_lu ( Hor_Matrix *M_LUD )
*
*   Multiples lower and upper parts of LUD decomposed matrix M_LUD together
*   and returns the result.
********************/
Hor_Matrix *hor_mats_prod_lu ( Hor_Matrix *M_LUD )
{
   Hor_Matrix *M_LUD_lu;

   if ( M_LUD == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M_LUD_lu = hor_mat_alloc ( M_LUD->rows, M_LUD->cols );
   if ( M_LUD_lu == NULL ) return NULL;

   mat_prod_lu ( M_LUD->m, M_LUD_lu->m, M_LUD->rows );
   return M_LUD_lu;
}

/*******************
*   Hor_Matrix *@hor_mats_prod_ul ( Hor_Matrix *M_LUD )
*
*   Multiples upper and lower parts of LUD decomposed matrix M_LUD together
*   and returns the result.
********************/
Hor_Matrix *hor_mats_prod_ul ( Hor_Matrix *M_LUD )
{
   Hor_Matrix *M_LUD_ul;

   if ( M_LUD == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M_LUD_ul = hor_mat_alloc ( M_LUD->rows, M_LUD->cols );
   if ( M_LUD_ul == NULL ) return NULL;

   mat_prod_ul ( M_LUD->m, M_LUD_ul->m, M_LUD->rows );
   return M_LUD_ul;
}

/*******************
*   Hor_Matrix *@hor_mats_inv ( Hor_Matrix *M )
*
*   Creates matrix M_inv and calculates M_inv = M^-1 (inverse of M). M must be
*   square, symmetric and positive definite. M_inv is returned.
********************/
Hor_Matrix *hor_mats_inv ( Hor_Matrix *M )
{
   int         size;
   double    **Mm;
   Hor_Matrix *M_inv;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   size = M->rows;
   Mm   = M->m;
   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M_inv = hor_mat_alloc ( size, size );
   if ( M_inv == NULL ) return NULL;

   switch ( size )
   {
      case 1:
      if ( fabs(Mm[0][0]) < EPSILON ) {
	 hor_errno = HOR_MATH_MATRIX_SINGULAR;
	 hor_mat_free(M_inv);
	 return NULL;
      }

      M_inv->m[0][0] = 1.0/Mm[0][0];
      break;

      case 2:
      {
	 double det = Mm[0][0]*Mm[1][1] - Mm[0][1]*Mm[1][0];

	 if ( fabs(det) < EPSILON ) {
	    hor_errno = HOR_MATH_MATRIX_SINGULAR;
	    hor_mat_free(M_inv);
	    return NULL;
	 }

	 M_inv->m[0][0] = Mm[1][1]/det;
	 M_inv->m[0][1] = M_inv->m[1][0] = -Mm[0][1]/det;
	 M_inv->m[1][1] = Mm[0][0]/det;
      }
      break;

      case 3:
      {
	 double a1, a2, a3, det;

	 a1 = Mm[1][1]*Mm[2][2] - Mm[1][2]*Mm[2][1];
	 a2 = Mm[1][0]*Mm[2][2] - Mm[1][2]*Mm[2][0];
	 a3 = Mm[1][0]*Mm[2][1] - Mm[1][1]*Mm[2][0];
	 det = Mm[0][0]*a1 - Mm[0][1]*a2 + Mm[0][2]*a3;

	 if ( fabs(det) < EPSILON ) {
	    hor_errno = HOR_MATH_MATRIX_SINGULAR;
	    hor_mat_free(M_inv);
	    return NULL;
	 }

	 M_inv->m[0][0] = a1/det;
	 M_inv->m[0][1] = (Mm[0][2]*Mm[2][1] - Mm[0][1]*Mm[2][2])/det;
	 M_inv->m[0][2] = (Mm[0][1]*Mm[1][2] - Mm[0][2]*Mm[1][1])/det;
	 M_inv->m[1][0] = -a2/det;
	 M_inv->m[1][1] = (Mm[0][0]*Mm[2][2] - Mm[0][2]*Mm[2][0])/det;
	 M_inv->m[1][2] = (Mm[0][2]*Mm[1][0] - Mm[0][0]*Mm[1][2])/det;
	 M_inv->m[2][0] = a3/det;
	 M_inv->m[2][1] = (Mm[0][1]*Mm[2][0] - Mm[0][0]*Mm[2][1])/det;
	 M_inv->m[2][2] = (Mm[0][0]*Mm[1][1] - Mm[0][1]*Mm[1][0])/det;
      }
      break;

      default:
      {
	 int    *indx, i, j;
	 double *vv, d;
	 Hor_Matrix *M_LUD, *col;

	 indx = hor_malloc_ntype ( int,    size );
	 if ( indx == NULL ) return NULL;

	 vv   = hor_malloc_ntype ( double, size );
	 if ( vv == NULL ) { hor_free ( indx ); return NULL; }

	 col  = hor_mat_alloc ( size, 1 );
	 if ( vv == NULL ) { hor_free_multiple ( vv, indx, NULL );
			     return NULL; }

	 M_LUD = hor_mats_copy ( M );
	 if ( M_LUD == NULL ) { hor_free_multiple ( vv, indx, NULL );
				hor_mat_free ( col ); return NULL; }

	 hor_ludcmp ( M_LUD->m, M->rows, indx, &d, vv );
	 for ( j = 0; j < size; j++ )
	 {
	    for ( i = 0; i < size; i++ ) col->m[i][0] = 0.0;
	    col->m[j][0] = 1.0;
	    hor_lubksb ( M_LUD->m, size, indx, col->m );
	    for ( i = 0; i < size; i++ ) M_inv->m[i][j] = col->m[i][0];
	 }

	 hor_mat_free_list ( M_LUD, col, NULL );
	 hor_free_multiple ( (void *) vv, (void *) indx, NULL );
      }
#if 0
      {
	 double *W = hor_malloc_ntype ( double, size );
	 Hor_Matrix *U, *UT;
	 int     i, j;

	 U  = hor_mat_alloc ( size, size );
	 UT = hor_mat_alloc ( size, size );
	 hor_matq_copy ( M, U );
	 if ( !hor_matq_svdcmp ( U, W, M_inv ) ) return NULL;

	 /* V is in M_inv. Now compute V*W^-1 */
	 for ( j = 0; j < size; j++ )
	 {
	    if ( W[j] < EPSILON )
	    {
	       hor_errno = HOR_MATH_MATRIX_SINGULAR;
	       return NULL;
	    }

	    for ( i = 0; i < size; i++ ) M_inv->m[i][j] /= W[j];
	 }

	 /* compute M^-1 = V*W^-1*U^T */
	 hor_matq_transpose ( U, UT );
	 hor_matq_copy ( M_inv, U );
	 hor_matq_prod2 ( U, UT, M_inv );

	 hor_mat_free_list ( UT, U, NULL );
	 hor_free ( (void *) W );
      }
#endif
      break;
   }

   return M_inv;
}

/*******************
*   Hor_Matrix *@hor_mats_solve ( Hor_Matrix *A, Hor_Matrix *b )
*
*   Solves the matrix equation Ax=b for general non-singular matrix A and
*   vectors b and x, returning x.
********************/
Hor_Matrix *hor_mats_solve ( Hor_Matrix *A, Hor_Matrix *b )
{
   Hor_Matrix *LU, *x;
   int        *indx;
   double     *vv, d;

   if ( A == NULL || b == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( A ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }
   if ( !hor_mat_test_size ( b, A->rows, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   x  = hor_mats_copy ( b );
   LU = hor_mats_copy ( A );
   indx = hor_malloc_ntype ( int,    A->rows );
   if ( indx == NULL ) return NULL;

   vv   = hor_malloc_ntype ( double, A->rows );
   if ( vv == NULL ) { hor_free ( indx ); return NULL; }

   hor_ludcmp ( LU->m, A->rows, indx, &d, vv );
   hor_lubksb ( LU->m, A->rows, indx, x->m );

   hor_free ( (void *) vv );
   hor_free ( (void *) indx );
   hor_mat_free ( LU );
   return x;
}

/*******************
*   Hor_Matrix *@hor_mats_gen_solve ( Hor_Matrix *A, Hor_Matrix *b,
*                                    Hor_Matrix *x )
*
*   Solves the matrix equation Ax=b for a general mxn matrix A (m>=n) and
*   vectors b and x, returning x.
********************/
Hor_Matrix *hor_mats_gen_solve ( Hor_Matrix *A, Hor_Matrix *b, Hor_Matrix *x )
{
   Hor_Matrix *V, *U;
   double     *W;

   if (A == NULL || b == NULL) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if (!hor_mat_test_size (b, A->rows, 1) || 
       !hor_mat_big_enough (x, A->cols, 1)) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   U = hor_mats_copy (A);
   if ( U == NULL ) return NULL;

   V = hor_mat_alloc (A->cols, A->cols);
   if ( V == NULL ) { hor_mat_free ( U ); return NULL; }

   W = hor_malloc_ntype (double, A->rows);
   if ( W == NULL ) { hor_mat_free_list ( V, U, NULL ); return NULL; }

   hor_matq_copy(A, U);
   hor_matq_svdcmp(U, W, V);
   x->rows = A->cols;
   hor_svbksb(U->m, W, V->m, U->rows, U->cols, b->m, x->m);

   hor_free ( (void *) W );
   hor_mat_free_list ( V, U, NULL );
   return x;
}

/*******************
*   Hor_Matrix *@hor_mats_pseud_inv ( Hor_Matrix *A )
*
*   Computes and returns the pseudo-inverse of a matrix A as (A^T*A)^-1*A^T.
*   A must have at least as many rows as columns or else A^T*A will be
*   singular.
********************/
Hor_Matrix *hor_mats_pseud_inv ( Hor_Matrix *A )
{
   Hor_Matrix *AT, *ATA, *ATAinv, *Ap;

   if ( A == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }
  
   if ( A->cols > A->rows ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   AT = hor_mats_transpose (A);
   ATA = hor_mats_prod2 ( AT, A );
   ATAinv = hor_mats_inv ( ATA );
   Ap = hor_mats_prod2 ( ATAinv, AT );

   hor_mat_free_list ( ATAinv, ATA, AT, NULL );
   return Ap;
}

/*******************
*   Hor_Matrix *@hor_mats_identity ( int size )
*
*   Creates and returns identity matrix of given size.
********************/
Hor_Matrix *hor_mats_identity ( int size )
{
   Hor_Matrix *M;

   if ( size <= 0 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   M = hor_mat_alloc ( size, size );
   if ( M == NULL ) return NULL;

   mat_identity ( M->m, size, size );
   return M;
}

/*******************
*   Hor_Matrix *@hor_mats_zero ( int rows, int cols )
*
*   Creates and returns zero-filled matrix of given size.
********************/
Hor_Matrix *hor_mats_zero ( int rows, int cols )
{
   Hor_Matrix *M;

   if ( rows <= 0 || cols <= 0 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   M = hor_mat_alloc ( rows, cols );
   if ( M == NULL ) return NULL;

   mat_zero ( M->m, rows, cols, M->csize );
   return M;
}

/*******************
*   Hor_Matrix *@hor_mats_diagonal ( int size, ... )
*
*   Creates and returns diagonal matrix formed by the provided list of values.
*   NOTE: cast all numerical arguments to double if necessary.
********************/
Hor_Matrix *hor_mats_diagonal ( int size, ... )
{
   va_list  ap;
   Hor_Matrix  *M;

   if ( size <= 0 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   M = hor_mat_alloc ( size, size );
   if ( M == NULL ) return NULL;

   va_start ( ap, size );
   mat_diagonal ( M->m, size, &ap );
   va_end(ap);
   return M;
}

/*******************
*   Hor_Matrix *@hor_mats_extract ( Hor_Matrix *source,
*                                  int c0, int r0, int width, int height )
*
*   Returns a copy of the specified part of source matrix.
*   c0 and r0 specify the offsets of the region of source to be used,
*   width and height specify the size of the region.
********************/
Hor_Matrix *hor_mats_extract ( Hor_Matrix *source,
			       int c0, int r0, int width, int height )
{
   Hor_Matrix *dest;
   int         i, j;
   double    **sm = source->m, **dm;

   if ( c0 < 0 || c0 + width  > source->cols ||
        r0 < 0 || r0 + height > source->rows )
   { hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS; return NULL; }

   dest = hor_mat_alloc ( height, width );
   if ( dest == NULL ) return NULL;

   dm = dest->m;
   for ( i = 0; i < height; i++ )
      for ( j = 0; j < width; j++ )
	 dm[i][j] = sm[r0+i][c0+j];

   return dest;
}

/*******************
*   "Quick" matrix routines that require everything to be allocated beforehand.
*   If an error occurs (e.g. inverting a singular matrix), the global variable
*   hor_errno is set to one of the values found in global.h and NULL is
*   returned. If a matrix argument to a function is NULL, NULL is returned,
*   on the assumption that another error has previously happened.
*   The function hor_perror() can be called to print an error message.
********************/

/*******************
*   Hor_Matrix *@hor_matq_fill ( Hor_Matrix *A, ... )
*
*   Fills matrix A with values passed in variable argument list. Values are
*   read in "raster scan" order, i.e. columns first, then rows, from top-left
*   to bottom right. A is returned. NOTE: cast all numerical arguments to
*   double if necessary.
********************/
Hor_Matrix *hor_matq_fill ( Hor_Matrix *A, ... )
{
   va_list ap;

   if ( A == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   va_start ( ap, A );
   mat_fill ( A->m, A->rows, A->cols, &ap );
   va_end(ap);
   return A;
}

/*******************
*   Hor_Matrix *@hor_matq_copy ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Copies entries from matrix A into matrix B. returning B.
********************/
Hor_Matrix *hor_matq_copy ( Hor_Matrix *A, Hor_Matrix *B )
{
   if ( A == NULL || B == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_fits_in ( A, B ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_copy ( A->m, B->m, A->rows, A->cols, A->csize, B->csize );
   B->rows = A->rows;
   B->cols = A->cols;
   return B;
}

/*******************
*   Hor_Matrix *@hor_matq_add2 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
*
*   Calculates C=A+B and returns C.
********************/
Hor_Matrix *hor_matq_add2 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
{
   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_same_size2 ( A, B ) || !hor_mat_fits_in ( A, C ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_add2 (A->m, B->m, C->m, A->rows, A->cols, A->csize, B->csize, C->csize);
   C->rows = A->rows;
   C->cols = A->cols;
   return C;
}

/*******************
*   Hor_Matrix *@hor_matq_sub ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
*
*   Calculates C=A-B and returns C.
********************/
Hor_Matrix *hor_matq_sub ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
{
   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_same_size2 ( A, B ) || !hor_mat_fits_in ( A, C ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_sub (A->m, B->m, C->m, A->rows, A->cols, A->csize, B->csize, C->csize);
   C->rows = A->rows;
   C->cols = A->cols;
   return C;
}

/*******************
*   Hor_Matrix *@hor_matq_prod2 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
*
*   Calculates C=AB and returns C.
********************/
Hor_Matrix *hor_matq_prod2 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
{
   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( A->cols != B->rows || !hor_mat_big_enough ( C, A->rows, B->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prod2 ( A->m, B->m, C->m, A->rows, B->cols, A->cols );
   C->rows = A->rows;
   C->cols = B->cols;
   return C;
}

/*******************
*   Hor_Matrix *@hor_matq_prod3 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C,
*                                Hor_Matrix *WSP, Hor_Matrix *D )
*
*   Calculates matrix triple product D=ABC. Workspace matrix WSP is used to
*   calculate intermediate result WSP=AB, i.e. product is calculated as
*   D=(AB)C, and D is returned.
********************/
Hor_Matrix *hor_matq_prod3 ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C,
			     Hor_Matrix *WSP, Hor_Matrix *D )
{
   if ( A == NULL || B == NULL || C == NULL || WSP == NULL || D == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_test_size ( B, A->cols, C->rows ) ||
        !hor_mat_big_enough ( D, A->rows, C->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !hor_mat_big_enough ( WSP, A->rows, B->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prod2 ( A->m,   B->m, WSP->m, A->rows,   B->cols, A->cols );
   WSP->rows = A->rows;
   WSP->cols = B->cols;
   mat_prod2 ( WSP->m, C->m, D->m,   WSP->rows, C->cols, WSP->cols );
   D->rows = A->rows;
   D->cols = C->cols;
   return D;
}

/*******************
*   Hor_Matrix *@hor_matq_scale ( Hor_Matrix *M, double factor )
*
*   Multiplies each element of the input matrix M by the given scalar factor.
********************/
Hor_Matrix *hor_matq_scale ( Hor_Matrix *M, double factor )
{
   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   mat_scale ( M->m, M->rows, M->cols, factor, M->csize );
   return M;
}

/*******************
*   Hor_Matrix *@hor_matq_transpose ( Hor_Matrix *M, Hor_Matrix *MT )
*
*   Transposes matrix M and write result into MT, returning MT.
********************/
Hor_Matrix *hor_matq_transpose ( Hor_Matrix *M, Hor_Matrix *MT )
{
   if ( M == NULL || MT == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_big_enough ( MT, M->cols, M->rows ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_transpose ( M->m, MT->m, M->rows, M->cols );
   MT->rows = M->cols;
   MT->cols = M->rows;
   return MT;
}

/*******************
*   Hor_Matrix *@hor_matq_lud ( Hor_Matrix *M, Hor_Matrix *M_LUD )
*
*   Performs lower-upper decomposition of square, symmetric, positive-definite
*   matrix M, writing lower and upper parts into M_LUD, and returning M_LUD.
********************/
Hor_Matrix *hor_matq_lud ( Hor_Matrix *M, Hor_Matrix *M_LUD )
{
   if ( M == NULL || M_LUD == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD, M->rows, M->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !mat_lud ( M->m, M_LUD->m, M->rows ) ) return NULL;

   M_LUD->rows = M_LUD->cols = M->rows;
   return M_LUD;
}

/*******************
*   Hor_Matrix *@hor_matq_lud_inv ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_inv )
*
*   Inverts lower-upper decomposed matrix M_LUD, writing result into M_LUD_inv.
********************/
Hor_Matrix *hor_matq_lud_inv ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_inv )
{
   if ( M_LUD == NULL || M_LUD_inv == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD_inv, M_LUD->rows, M_LUD->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_lud_inv ( M_LUD->m, M_LUD_inv->m, M_LUD->rows );
   M_LUD_inv->rows = M_LUD_inv->cols = M_LUD->rows;
   return M_LUD_inv;
}

/*******************
*   Hor_Matrix *@hor_matq_prod_lu ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_lu )
*
*   Multiplies lower and upper parts of LUD decomposed matrix M_LUD and writes
*   result into M_LUD_lu, returning M_LUD_lu.
********************/
Hor_Matrix *hor_matq_prod_lu ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_lu )
{
   if ( M_LUD == NULL || M_LUD_lu == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD_lu, M_LUD->rows, M_LUD->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prod_lu ( M_LUD->m, M_LUD_lu->m, M_LUD->rows );
   M_LUD_lu->rows = M_LUD_lu->cols = M_LUD->rows;
   return M_LUD_lu;
}

/*******************
*   Hor_Matrix *@hor_matq_prod_ul ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_ul )
*
*   Multiplies upper and lower parts of LUD decomposed matrix M_LUD and writes
*   result into M_LUD_ul, returning M_LUD_ul.
********************/
Hor_Matrix *hor_matq_prod_ul ( Hor_Matrix *M_LUD, Hor_Matrix *M_LUD_ul )
{
   if ( M_LUD == NULL || M_LUD_ul == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M_LUD ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD_ul, M_LUD->rows, M_LUD->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prod_ul ( M_LUD->m, M_LUD_ul->m, M_LUD->rows );
   M_LUD_ul->rows = M_LUD_ul->cols = M_LUD->rows;
   return M_LUD_ul;
}

/*******************
*   Hor_Matrix *@hor_matq_solve_lower ( Hor_Matrix *L, Hor_Matrix *x,
*                                      Hor_Matrix *y )
*
*   Solves the linear equation system Lx=y for lower triangular matrix L and
*   vectors x and y, returning x.
********************/
Hor_Matrix *hor_matq_solve_lower ( Hor_Matrix *L, /* lower-triangular matrix */
				   Hor_Matrix *x, /* desired solution vector */
				   Hor_Matrix *y ) /* vector */
{
   int   i, j, size;
   double sum, **Lm, **xm, **ym;

   if ( L == NULL || x == NULL || y == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   size = L->rows; Lm = L->m; xm = x->m; ym = y->m;
   if ( !hor_mat_is_square ( L ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( x, size, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !hor_mat_test_size ( y, size, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   for ( i = 0; i < size; i++ )
   {
      sum = ym[i][0];
      for ( j = 0; j < i; j++ )
	 sum -= Lm[i][j]*xm[j][0];

      xm[i][0] = sum/Lm[i][i];
   }

   return x;
}

/*******************
*   Hor_Matrix *@hor_matq_solve_upper ( Hor_Matrix *U, Hor_Matrix *x,
*                                      Hor_Matrix *y )
*
*   Solves the linear equation system Ux=y for upper triangular matrix U and
*   vectors x and y, returning x.
********************/
Hor_Matrix *hor_matq_solve_upper ( Hor_Matrix *U, /* upper-triangular matrix */
				   Hor_Matrix *x, /* desired solution vector */
				   Hor_Matrix *y ) /* vector */
{
   int   i, j, size;
   double sum, **Um, **xm, **ym;

   if ( U == NULL || x == NULL || y == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   size = U->rows; Um = U->m; xm = x->m; ym = y->m;
   if ( !hor_mat_is_square ( U ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( x, size, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !hor_mat_test_size ( y, size, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   for ( i = size-1; i >= 0; i-- )
   {
      sum = ym[i][0];
      for ( j = size-1; j > i; j-- )
	 sum -= Um[i][j]*xm[j][0];

      xm[i][0] = sum/Um[i][i];
   }

   return x;
}

/*******************
*   Hor_Matrix *@hor_matq_inv ( Hor_Matrix *M, Hor_Matrix *M_LUD,
*                              Hor_Matrix *M_LUD_inv, Hor_Matrix *M_inv )
*
*   Inverts square, symmetric, positive definite matrix M, and writes result
*   into M_inv, returning M_inv. Matrices M_LUD and M_LUD_inv are used to store
*   intermediate results.
********************/
Hor_Matrix *hor_matq_inv ( Hor_Matrix *M,         Hor_Matrix *M_LUD,
			   Hor_Matrix *M_LUD_inv, Hor_Matrix *M_inv )
{
   int      size;
   double **Mm;

   size = M->rows; Mm = M->m;
   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD, size, size ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_LUD_inv, size, size ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   if ( !hor_mat_big_enough ( M_inv, size, size ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   switch ( size )
   {
      case 1:
      if ( fabs(Mm[0][0]) < EPSILON ) {
	 hor_errno = HOR_MATH_MATRIX_SINGULAR;
	 return NULL;
      }

      M_inv->m[0][0] = 1.0/Mm[0][0];
      break;

      case 2:
      {
	 double det = Mm[0][0]*Mm[1][1] - Mm[0][1]*Mm[1][0];

	 if ( fabs(det) < EPSILON ) {
	    hor_errno = HOR_MATH_MATRIX_SINGULAR;
	    return NULL;
	 }

	 M_inv->m[0][0] = Mm[1][1]/det;
	 M_inv->m[0][1] = M_inv->m[1][0] = -Mm[0][1]/det;
	 M_inv->m[1][1] = Mm[0][0]/det;
      }
      break;

      case 3:
      {
	 double a1, a2, a3, det;

	 a1 = Mm[1][1]*Mm[2][2] - Mm[1][2]*Mm[2][1];
	 a2 = Mm[1][0]*Mm[2][2] - Mm[1][2]*Mm[2][0];
	 a3 = Mm[1][0]*Mm[2][1] - Mm[1][1]*Mm[2][0];
	 det = Mm[0][0]*a1 - Mm[0][1]*a2 + Mm[0][2]*a3;

	 if ( fabs(det) < EPSILON ) {
	    hor_errno = HOR_MATH_MATRIX_SINGULAR;
	    return NULL;
	 }

	 M_inv->m[0][0] = a1/det;
	 M_inv->m[0][1] = (Mm[0][2]*Mm[2][1] - Mm[0][1]*Mm[2][2])/det;
	 M_inv->m[0][2] = (Mm[0][1]*Mm[1][2] - Mm[0][2]*Mm[1][1])/det;
	 M_inv->m[1][0] = -a2/det;
	 M_inv->m[1][1] = (Mm[0][0]*Mm[2][2] - Mm[0][2]*Mm[2][0])/det;
	 M_inv->m[1][2] = (Mm[0][2]*Mm[1][0] - Mm[0][0]*Mm[1][2])/det;
	 M_inv->m[2][0] = a3/det;
	 M_inv->m[2][1] = (Mm[0][1]*Mm[2][0] - Mm[0][0]*Mm[2][1])/det;
	 M_inv->m[2][2] = (Mm[0][0]*Mm[1][1] - Mm[0][1]*Mm[1][0])/det;
      }
      break;

      default:
      {
	 int        *indx, i, j;
	 double     *vv, d;
	 Hor_Matrix *col;

	 indx = hor_malloc_ntype ( int,    size );
	 if ( indx == NULL ) return NULL;

	 vv   = hor_malloc_ntype ( double, size );
	 if ( vv == NULL ) { hor_free ( indx ); return NULL; }

	 col  = hor_mat_alloc ( size, 1 );
	 if ( vv == NULL ) { hor_free_multiple ( vv, indx, NULL );
			     return NULL; }

	 hor_matq_copy ( M, M_LUD );
	 hor_ludcmp ( M_LUD->m, M->rows, indx, &d, vv );
	 for ( j = 0; j < size; j++ )
	 {
	    for ( i = 0; i < size; i++ ) col->m[i][0] = 0.0;
	    col->m[j][0] = 1.0;
	    hor_lubksb ( M_LUD->m, size, indx, col->m );
	    for ( i = 0; i < size; i++ ) M_inv->m[i][j] = col->m[i][0];
	 }

	 hor_mat_free ( col );
	 hor_free_multiple ( (void *) vv, (void *) indx, NULL );
      }
#if 0
      {
	 double *W = hor_malloc_ntype ( double, size );
	 int     i, j;

	 hor_matq_copy ( M, M_LUD );
	 if ( !hor_matq_svdcmp ( M_LUD, W, M_inv ) ) return NULL;

	 /* V is in M_inv. Now compute V*W^-1 */
	 for ( j = 0; j < size; j++ )
	 {
	    if ( W[j] < EPSILON )
	    {
	       hor_errno = HOR_MATH_MATRIX_SINGULAR;
	       return NULL;
	    }

	    for ( i = 0; i < size; i++ ) M_inv->m[i][j] /= W[j];
	 }

	 /* compute M^-1 = V*W^-1*U^T */
	 hor_matq_transpose ( M_LUD, M_LUD_inv );
	 hor_matq_copy ( M_inv, M_LUD );
	 hor_matq_prod2 ( M_LUD, M_LUD_inv, M_inv );

	 hor_free ( (void *) W );
      }
#endif
      break;
   }

   M_inv->rows = M_inv->cols = M_LUD->rows = M_LUD->cols = M_LUD_inv->rows =
   M_LUD_inv->cols = size;
   return M_inv;
}

/*******************
*   Hor_Matrix *@hor_matq_ludcmp ( Hor_Matrix *A, int *indx, double *d,
*                                 double *vv )
*
*   Performs an LU-decomposition of matrix A, returning the row permutations
*   in indx. d is set to 1 if the permutation is even, -1 if it is odd.
*   The result is written into A. The size of the indx and vv arrays should
*   be the same as A.
********************/
Hor_Matrix *hor_matq_ludcmp ( Hor_Matrix *A, int *indx, double *d, double *vv )
{
   if ( A == NULL || indx == NULL || d == NULL || vv == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( A ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   hor_ludcmp ( A->m, A->rows, indx, d, vv );
   return A;
}

/*******************
*   Hor_Matrix *@hor_matq_lubksb ( Hor_Matrix *A, int *indx, Hor_Matrix *b )
*
*   Solves the matrix equation Ax=b, where A is an LU-decomposed matrix made
*   by a call to hor_matq_ludcmp(). indx is also passed from the result of
*   hor_matq_ludcmp(). The result is written into b, which is returned.
********************/
Hor_Matrix *hor_matq_lubksb ( Hor_Matrix *A, int *indx, Hor_Matrix *b )
{
   if ( A == NULL || indx == NULL || b == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( A ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   if ( !hor_mat_test_size ( b, A->rows, 1 ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   hor_lubksb ( A->m, A->rows, indx, b->m );
   return b;
}

/*******************
*   Hor_Bool @hor_matq_tred2 ( Hor_Matrix *A, double *d, double *e )
*
*   Reduces matrix A to tridiagonal form by Householder reduction. The elements
*   of A are replaced by the orthogonal matrix (Q) effecting the similarity
*   transformation, the diagonal elements are written into d and the
*   off-diagonal elements into e, with e[0]=0.
********************/
Hor_Bool hor_matq_tred2 ( Hor_Matrix *A, double *d, double *e )
{
   if ( A == NULL || d == NULL || e == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( !hor_mat_is_square ( A ) || A->rows < 2 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return HOR_FALSE;
   }

   hor_tred2 ( A->m, A->rows, d, e );
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_matq_tqli ( double *d, double *e, Hor_Matrix *Z )
*
*   Finds the eigenvalues and eigenvectors of the input tridiagonal matrix
*   defined by the diagonal elements d and off-diagonal elements e. Z is input
*   as the output matrix of hor_matq_tred2() to find the eigenvectors of the
*   original matrix, or else as an indentity matrix of the same size.
*   On output, d returns the eigenvalues, Z the eigenvectors and e is
*   overwritten.
********************/
Hor_Bool hor_matq_tqli ( double *d, double *e, Hor_Matrix *Z )
{
   if ( d == NULL || e == NULL || Z == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( !hor_mat_is_square ( Z ) || Z->rows < 2 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return HOR_FALSE;
   }

   hor_tqli ( d, e, Z->rows, Z->m );
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_matq_eigsrt ( double *d, Hor_Matrix *V )
*   Sorts the given eigenvalue array d into descending order, interchanging
*   the columns of V (the eigenvectors) accordingly.
********************/
Hor_Bool hor_matq_eigsrt ( double *d, Hor_Matrix *V )
{
   if ( d == NULL || V == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   if ( !hor_mat_is_square ( V ) || V->rows < 2 ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return HOR_FALSE;
   }

   hor_eigsrt ( d, V->m, V->rows );
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_matq_svdcmp ( Hor_Matrix *A, double *W, Hor_Matrix *V )
*
*   Computes the singular-value-decomposition of matrix A: A = U*W*V^T.
*   U is written over A, being the same size. If A is of size m rows and n
*   columns, U is of size m*n, W and V of size n*n. W is diagonal and only a
*   1D array of values of at least size n needs to be passed.
********************/
Hor_Bool hor_matq_svdcmp ( Hor_Matrix *A, double *W, Hor_Matrix *V )
{
   if ( A->rows < A->cols )
   { hor_errno = HOR_MATH_MATRIX_EXTRA_ZEROES; return HOR_FALSE; }

   if ( !hor_mat_big_enough ( V, A->cols, A->cols ) )
   { hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS; return HOR_FALSE; }

   V->rows = V->cols = A->cols;
   return ( hor_svdcmp ( A->m, A->rows, A->cols, W, V->m ) );
}

/*******************
*   Hor_Bool @hor_matq_gaussj ( Hor_Matrix *A, Hor_Matrix *B )
*
*   Solves the linear matrix equations A*x=b for square matrix A and each
*   column b of matrix B using Gauss-Jordan elimination.
********************/
Hor_Bool hor_matq_gaussj ( Hor_Matrix *A, Hor_Matrix *B )
{
   if ( !hor_mat_is_square ( A ) )
   {
      hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
      return HOR_FALSE;
   }

   if ( A->rows != B->rows )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return HOR_FALSE;
   }

   return ( hor_gaussj ( A->m, A->rows, B->m, B->cols ) );
}

/*******************
*   Hor_Bool @hor_matq_mrqmin ( double x[], double y[], double sig[],
*                              int ndata, double a[], int ma, int lista[],
*                              int mfit, Hor_Matrix *covar, Hor_Matrix *alpha,
*                              double *chisq,
*                              void (*funcs)(double,double *,double *,
*                                            double *, int), double *alamda )
*
*   Performs Levenberg-Marquardt least-squares minimisation using provided
*   function funcs().
********************/
Hor_Bool hor_matq_mrqmin ( double x[], double y[], double sig[], int ndata,
			   double a[], int ma, int lista[], int mfit,
			   Hor_Matrix *covar, Hor_Matrix *alpha, double *chisq,
			   void (*funcs)(double, double *, double *,
					 double *, int), double *alamda )
{
   return ( hor_mrqmin ( x, y, sig, ndata, a, ma, lista, mfit, covar->m,
			 alpha->m, chisq, funcs, alamda ) );
}

/*******************
*   Hor_Matrix *@hor_matq_gen_solve ( Hor_Matrix *A, Hor_Matrix *b,
*                                    Hor_Matrix *U, Hor_Matrix *V, double *W,
*                                    Hor_Matrix *x )
*
*   Solves the matrix equation Ax=b for a general mxn matrix A (m>=n) and
*   vectors b and x, returning x. U, V and W are workspace. U must be at least
*   as big as A, V should be at least A->cols square and W should be an array
*   at least A->cols in size.
********************/
Hor_Matrix *hor_matq_gen_solve ( Hor_Matrix *A, Hor_Matrix *b,
				 Hor_Matrix *U, Hor_Matrix *V, double *W,
				 Hor_Matrix *x )
{
   if (A == NULL || b == NULL) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if (!hor_mat_test_size (b, A->rows, 1) || 
       !hor_mat_big_enough (U, A->rows, A->cols) ||
       !hor_mat_big_enough (V, A->cols, A->cols) ||
       !hor_mat_big_enough (x, A->cols, 1)) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   hor_matq_copy(A, U);
   hor_matq_svdcmp(U, W, V);
   x->rows = A->cols;
   hor_svbksb(U->m, W, V->m, U->rows, U->cols, b->m, x->m);

   return x;
}

/*******************
*   Hor_Matrix *@hor_matq_identity ( Hor_Matrix *M )
*
*   Sets the given matrix to the identity matrix.
********************/
Hor_Matrix *hor_matq_identity ( Hor_Matrix *M )
{
   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   mat_identity ( M->m, M->rows, M->csize );
   return M;
}

/*******************
*   Hor_Matrix *@hor_matq_zero ( Hor_Matrix *M )
*   Hor_Matrix *@hor_matq_zero_lower ( Hor_Matrix *M )
*   Hor_Matrix *@hor_matq_zero_upper ( Hor_Matrix *M )
*
*   hor_matq_zero() sets the given matrix M to zero. ..._lower and ..._upper
*   restrict their action to the lower and upper (below/above the diagonal)
*   parts of M respectively, exclusive of the diagonal itself, which is left
*   unchanged, and only work for square matrices.
********************/
Hor_Matrix *hor_matq_zero ( Hor_Matrix *M )
{
   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   mat_zero ( M->m, M->rows, M->cols, M->csize );
   return M;
}

Hor_Matrix *hor_matq_zero_lower ( Hor_Matrix *M )
{
   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   mat_zero_lower ( M->m, M->rows );
   return M;
}

Hor_Matrix *hor_matq_zero_upper ( Hor_Matrix *M )
{
   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   mat_zero_upper ( M->m, M->rows );
   return M;
}

/*******************
*   Hor_Matrix *@hor_matq_diagonal ( Hor_Matrix *M, ... )
*
*   Sets the given matrix as diagonal with provided list of values. NOTE: cast
*   all numerical arguments to double if necessary.
********************/
Hor_Matrix *hor_matq_diagonal ( Hor_Matrix *M, ... )
{
   va_list ap;

   if ( M == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( !hor_mat_is_square ( M ) ) {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   va_start ( ap, M );
   mat_diagonal ( M->m, M->rows, &ap );
   va_end(ap);
   return M;
}

/*******************
*   Hor_Matrix *@hor_matq_extract ( Hor_Matrix *source,
*                                  int c0, int r0, int width, int height,
*                                  Hor_Matrix *dest )
*
*   Copies specified part of source matrix to the destination matrix.
*   c0 and r0 specify the offsets of the region of source to be used,
*   width and height specify the size of the region. dest is returned.
********************/
Hor_Matrix *hor_matq_extract ( Hor_Matrix *source,
			       int c0, int r0, int width, int height,
			       Hor_Matrix *dest )
{
   int      i, j;
   double **sm = source->m, **dm = dest->m;

   if ( c0 < 0 || c0 + width  > source->cols || width > dest->csize ||
        r0 < 0 || r0 + height > source->rows || height > dest->rsize )
   { hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS; return NULL; }

   for ( i = 0; i < height; i++ )
      for ( j = 0; j < width; j++ )
	 dm[i][j] = sm[r0+i][c0+j];

   dest->cols = width;
   dest->rows = height;
   return dest;
}

/*******************
*   Hor_Matrix *@hor_mat_swap_rows ( Hor_Matrix *M, int row1, int row2 )
*   Hor_Matrix *@hor_mat_swap_cols ( Hor_Matrix *M, int col1, int col2 )
*
*   Swap all elements of given row/column of a matrix and return the
*   adjusted matrix.
********************/
Hor_Matrix *hor_mat_swap_rows ( Hor_Matrix *M, int row1, int row2 )
{
   int rows = M->rows, cols = M->cols;

   if ( row1 < 0 || row1 >= rows || row2 < 0 || row2 >= rows ) {
      hor_errno = HOR_MATH_ILLEGAL_VALUE;
      return NULL;
   }

   if ( row1 != row2 ) {
      double *ptr1 = M->m[row1], *ptr2 = M->m[row2], temp;
      int i;

      for ( i = 0; i < cols; i++ )
      { temp = ptr1[i]; ptr1[i] = ptr2[i]; ptr2[i] = temp; }
   }

   return M;
}

Hor_Matrix *hor_mat_swap_cols ( Hor_Matrix *M, int col1, int col2 )
{
   int rows = M->rows, cols = M->cols;

   if ( col1 < 0 || col1 >= cols || col2 < 0 || col2 >= cols ) {
      hor_errno = HOR_MATH_ILLEGAL_VALUE;
      return NULL;
   }

   if ( col1 != col2 ) {
      double **Mm = M->m, temp;
      int i;

      for ( i = 0; i < rows; i++ )
      { temp = Mm[i][col1]; Mm[i][col1] = Mm[i][col2]; Mm[i][col2] = temp; }
   }

   return M;
}

/*******************
*   Hor_Matrix *@hor_mat_insert ( Hor_Matrix *source, int cs, int rs,
*                                Hor_Matrix *dest,   int cd, int rd,
*                                int width, int height )
*
*   Copies specified part of source matrix to specified part of destination
*   matrix. cs, rs, cd, rd specify offsets of the region of source and dest to
*   be used, width and height specify the size of the region.
********************/
Hor_Matrix *hor_mat_insert ( Hor_Matrix *source, int cs, int rs,
			     Hor_Matrix *dest,   int cd, int rd,
			     int width, int height )
{
   int      srows = source->rows, scols = source->cols, i, j;
   int      drows = dest->rsize,  dcols = dest->csize;
   double **sm = source->m, **dm = dest->m;

   if ( cs < 0 || cs + width > scols || rs < 0 || rs + height > srows ||
        cd < 0 || cd + width > dcols || rd < 0 || rd + height > drows )
   { hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS; return NULL; }

   for ( i = 0; i < height; i++ )
      for ( j = 0; j < width; j++ )
	 dm[rd+i][cd+j] = sm[rs+i][cs+j];

   return dest;
}

/*******************
*   Hor_Matrix *@hor_mat_prod2_offset ( Hor_Matrix *M1, Hor_Matrix *M2,
*                                      Hor_Matrix *dest,
*                                      int offset_x, int offset_y )
*
*   Multiplies M1 and M2 and writes into matrix dest at given offsets.
*   dest is returned.
********************/
Hor_Matrix *hor_mat_prod2_offset ( Hor_Matrix *M1, Hor_Matrix *M2,
				   Hor_Matrix *dest,
				   int offset_x, int offset_y )
{
   double **M1m, **M2m, **destm;
   int      r, c, i;
   int      M1rows, M1cols, M2rows, M2cols;
   double   sum;

   if ( M1 == NULL || M2 == NULL || dest == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( M1->cols != M2->rows ||
        !hor_mat_big_enough ( dest, M1->rows, M2->cols ) ||
        offset_x < 0 || offset_x + M2->cols > dest->csize ||
        offset_y < 0 || offset_y + M1->rows > dest->rsize )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M1m   = M1->m;   M1cols = M1->cols;   M1rows = M1->rows;
   M2m   = M2->m;   M2cols = M2->cols;   M2rows = M2->rows;
   destm = dest->m;
   for ( r = 0; r < M1rows; r++ )
      for ( c = 0; c < M2cols; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < M1cols; i++ )
	    sum += M1m[r][i]*M2m[i][c];

	 destm[offset_y+r][offset_x+c] = sum;
      }

   return dest;
}

/*******************
*   Hor_Matrix *@hor_mat_prod2_part (  Hor_Matrix *M1, int c1, int r1, int h1,
*                                      Hor_Matrix *M2, int c2, int r2, int w2,
*                                      int common, Hor_Matrix *dest )
*
*   Multiplies specified parts of M1 and M2 (at given start offsets and given
*   width/height) and writes into matrix dest, which is returned.
*   common is the width used in M1, equal to the height used in M2.
********************/
Hor_Matrix *hor_mat_prod2_part ( Hor_Matrix *M1, int c1, int r1, int h1,
				 Hor_Matrix *M2, int c2, int r2, int w2,
				 int common, Hor_Matrix *dest )
{
   double **M1m, **M2m, **destm;
   int      r, c, i;
   double   sum;

   if ( M1 == NULL || M2 == NULL || dest == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( c1 < 0 || c1 + common > M1->cols || r1 < 0 || r1 + h1 > M1->rows ||
        c2 < 0 || c2 + w2 > M2->cols || r2 < 0 || r2 + common > M2->rows ||
        !hor_mat_big_enough ( dest, h1, w2 ) )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M1m = M1->m; M2m = M2->m; destm = dest->m;
   for ( r = 0; r < h1; r++ )
      for ( c = 0; c < w2; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < common; i++ )
	    sum += M1m[r1+r][c1+i]*M2m[r2+i][c2+c];

	 destm[r][c] = sum;
      }

   dest->rows = h1;
   dest->cols = w2;
   return dest;
}

/*******************
*   Hor_Matrix *@hor_mat_prod3_part (Hor_Matrix *M1, Hor_Matrix *M2,
*                                   Hor_Matrix *M3, int offset_x, int offset_y,
*                                   Hor_Matrix *work, Hor_Matrix *dest )
*
*   Multiplies M1, M2, M3, only using the part of M2 at given offsets and
*   of size given by M1 and M3.
********************/
Hor_Matrix *hor_mat_prod3_part (Hor_Matrix *M1, Hor_Matrix *M2, Hor_Matrix *M3,
				int offset_x, int offset_y,
				Hor_Matrix *work, Hor_Matrix *dest )
{
   double **M1m, **M2m, **M3m, **wm, **destm;
   int      r, c, i;
   int      M1rows, M1cols, M3rows, M3cols;
   double   sum;

   if ( M1 == NULL || M2 == NULL || M3 == NULL || work == NULL || dest == NULL)
   {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( offset_x < 0 || offset_x + M3->rows > M2->cols ||
        offset_y < 0 || offset_y + M1->cols > M2->rows ||
        !hor_mat_big_enough ( work, M1->rows, M3->rows ) ||
        !hor_mat_big_enough ( dest, M1->rows, M3->cols ) )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   M1m   = M1->m; M1cols = M1->cols; M1rows = M1->rows;
   M2m   = M2->m;
   M3m   = M3->m; M3cols = M3->cols; M3rows = M3->rows;
   destm = dest->m;
   wm = work->m;
   for ( r = 0; r < M1rows; r++ )
      for ( c = 0; c < M3rows; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < M1cols; i++ )
	    sum += M1m[r][i]*M2m[offset_x+i][offset_y+c];

	 wm[r][c] = sum;
      }

   for ( r = 0; r < M1cols; r++ )
      for ( c = 0; c < M3cols; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < M3rows; i++ )
	    sum += wm[r][i]*M3m[i][c];

	 destm[r][c] = sum;
      }

   dest->rows = M1rows;
   dest->cols = M3cols;
   return dest;
}

/*******************
*   Hor_Matrix *@hor_mat_inc_offset ( Hor_Matrix *dest, Hor_Matrix *source,
*                                    int offset_x, int offset_y )
*   Hor_Matrix *@hor_mat_dec_offset ( Hor_Matrix *dest, Hor_Matrix *source,
*                                    int offset_x, int offset_y )
*
*   increments/decrements given part of destination matrix with source matrix.
********************/
Hor_Matrix *hor_mat_inc_offset ( Hor_Matrix *dest, Hor_Matrix *source,
				 int offset_x, int offset_y )
{
   int r, c;
   double **dm, **sm;

   if ( dest == NULL || source == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( offset_x < 0 || offset_x + source->cols > dest->cols ||
        offset_y < 0 || offset_y + source->rows > dest->rows ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   sm = source->m;
   dm = dest->m;
   for ( r = 0; r < source->rows; r++ )
      for ( c = 0; c < source->cols; c++ )
	 dm[offset_y+r][offset_x+c] += sm[r][c];

   return dest;
}

Hor_Matrix *hor_mat_dec_offset ( Hor_Matrix *dest, Hor_Matrix *source,
				 int offset_x, int offset_y )
{
   int r, c;
   double **dm, **sm;

   if ( dest == NULL || source == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( offset_x < 0 || offset_x + source->cols > dest->cols ||
        offset_y < 0 || offset_y + source->rows > dest->rows )
   {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   sm = source->m;
   dm = dest->m;
   for ( r = 0; r < source->rows; r++ )
      for ( c = 0; c < source->cols; c++ )
	 dm[offset_y+r][offset_x+c] -= sm[r][c];

   return dest;
}

/*******************
*   double @hor_mat_det ( Hor_Matrix *M );
*
*   Returns the determinant of square matrix M.
********************/
double hor_mat_det ( Hor_Matrix *M )
{
   double      total, **Mm = M->m, *vv;
   int         size = M->rows, j, *indx;
   Hor_Matrix *M_LUD;

   if ( !hor_mat_is_square ( M ) )
      hor_error ( "matrix not square (hor_mat_determinant)", HOR_FATAL );

   if ( size == 1 ) return Mm[0][0];
   else if ( size == 2) return ( Mm[0][0]*Mm[1][1] - Mm[0][1]*Mm[1][0] );

   indx = hor_malloc_ntype ( int,    size );
   if ( indx == NULL ) return 0.0;

   vv   = hor_malloc_ntype ( double, size );
   if ( vv == NULL ) { hor_free ( indx ); return 0.0; }

   M_LUD = hor_mats_copy ( M );
   if ( M_LUD == NULL ) { hor_free_multiple ( vv, indx, NULL ); return 0.0; }

   hor_ludcmp ( M_LUD->m, M->rows, indx, &total, vv );
   for ( j = 0; j < size; j++ ) total *= M_LUD->m[j][j];
   
   hor_mat_free ( M_LUD );
   hor_free_multiple ( vv, indx, NULL );
   return total;
#if 0
   Mp = hor_mat_alloc ( size-1, size-1 );
   Mpm = Mp->m;
   for ( k = 0, total = 0.0; k < size; k++ )
   {
      for ( i = 0; i < size-1; i++ )
	 for ( j = 0; j < size-1; j++ )
	    Mpm[i][j] = Mm[i+1][(j+k+1) % size];

      total += Mm[0][k]*hor_mat_det ( Mp );
   }

   hor_mat_free ( Mp );
   return total;
#endif
}

/*******************
*   double @hor_mat_trace ( Hor_Matrix *M );
*
*   Returns the trace of square matrix M.
********************/
double hor_mat_trace ( Hor_Matrix *M )
{
   double **Mm = M->m, total;
   int      size = M->rows, i;

   if ( !hor_mat_is_square ( M ) )
      hor_error ( "matrix not square (hor_mat_determinant)", HOR_FATAL );

   for ( i = 0, total = 0.0; i < size; i++ ) total += Mm[i][i];

   return total;
}

/*******************
*   double @hor_mat_Fnorm ( Hor_Matrix *M );
*
*   Returns the Frobenius norm of matrix M, the square-root of the
*   sum-of-squares of its elements.
********************/
double hor_mat_Fnorm ( Hor_Matrix *M )
{
   double **Mm = M->m, total = 0.0;
   int      rows = M->rows, cols = M->cols, i, j;

   for ( i = 0; i < rows; i++ )
      for ( j = 0; j < cols; j++ ) total += hor_sqr(Mm[i][j]);

   return sqrt(total);
}

/*******************
*   void @hor_mat_read ( Hor_Matrix *M, ... )
*   void @hor_mat_readf ( Hor_Matrix *M, ... )
*   void @hor_mat_print ( Hor_Matrix *M, const char *string )
*
*   Miscellaneous matrix functions.
*
*   hor_mat_read() writes values from variable argument list into matrix in
*                  "raster scan" order.
*   hor_mat_readf() does the same, converting the matrix elements to float.
*   hor_mat_print() prints the contents of a matrix.
********************/
void hor_mat_read ( Hor_Matrix *M, ... )
{
   va_list  ap;
   double **Mm = M->m;
   int      r, c;

   va_start ( ap, M );
   for ( r = 0; r < M->rows; r++ )
      for ( c = 0; c < M->cols; c++ )
	 *(va_arg ( ap, double * )) = Mm[r][c];

   va_end(ap);
}

void hor_mat_readf ( Hor_Matrix *M, ... )
{
   va_list  ap;
   double **Mm = M->m;
   int      r, c;

   va_start ( ap, M );
   for ( r = 0; r < M->rows; r++ )
      for ( c = 0; c < M->cols; c++ )
	 *(va_arg ( ap, float * )) = (float) Mm[r][c];

   va_end(ap);
}

void hor_mat_print ( Hor_Matrix *M, const char *string )
{
   int r, c;

   hor_print ( "Matrix: %s\n", string );
   for ( r = 0; r < M->rows; r++ )
   {
      for ( c = 0; c < M->cols; c++ )
	 hor_print ( "%10.6f ", M->m[r][c] );

      hor_print ( "\n" );
   }
}

/*******************
*  Hor_Matrix *@hor_matq_set_dims ( Hor_Matrix *M, int rows, int cols )
*
*  Sets the "in use" dimensions of a matrix.  If the requested dimensions 
*  exceed the maximum allocation, the global variable hor_errno is set and 
*  NULL is returned.
********************/
Hor_Matrix *hor_matq_set_dims ( Hor_Matrix *M, int rows, int cols )
{
   if (!hor_mat_big_enough (M, rows, cols))
   {
      hor_errno = HOR_MATH_MATRIX_ILLEGAL_DIMENSIONS;
      return NULL;
   }

   M->rows = rows;
   M->cols = cols;
   return M;
}

