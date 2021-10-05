/*  Scene: software for sequential localisation and map-building

    bits.cc
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Many contributions from Joss Knight
    joss@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "general_headers.h"

/******************************hor_print_error********************************/

int hor_print_error ()
{
  if (hor_errno == HORATIO_OK) return HORATIO_OK;

  char prefix_string [60];
  snprintf (prefix_string, 60,
     "***ERROR!*** Horatio has encountered error code [%d]\n", hor_errno);
  hor_perror (prefix_string);
  return hor_errno;
}


/*******************************and_pi_range**********************************/

/* and_pi_range: a simple function which is passed a double
   representing an angle and then changes it so that it is in the
   range -pi -> pi */

void and_pi_range(double &angle)
{
  while(angle >= M_PI)
    angle -= 2 * M_PI;
  while(angle < -M_PI)
    angle += 2 * M_PI;

  return;
}

/******************Extensions to the Horatio matrix functions*****************/

/* Calculates the dot product of two 3 * 1 vectors */

double hor_matq_dot31(Hor_Matrix *A, Hor_Matrix *B)
{
  if ( A == NULL || B == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return 0.0;
  }

  if ( A->cols != 1 || A->rows != 3 || B->cols != 1 || B->rows != 3 ) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return 0.0;
  }

  return matel(A, 1, 1) * matel(B, 1, 1) +
         matel(A, 2, 1) * matel(B, 2, 1) +
         matel(A, 3, 1) * matel(B, 3, 1);
}

/*****************************************************************************/

/* Calculates the triple product C = A . B . A^T , where A and B are both
   3*3 matrices and B is symmetric. */

Hor_Matrix *hor_matq_ABAT3 (Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C)
{
  static double a11b11, a11b12, a11b13, 
                a21b11, a21b12, a21b13,
                a31b11, a31b12, a31b13,
                a12b12, a12b22, a12b23, 
                a22b12, a22b22, a22b23,
                a32b12, a32b22, a32b23,
                a13b13, a13b23, a13b33,
                a23b13, a23b23, a23b33,
                a33b13, a33b23, a33b33,
                a11b11_plus_a12b12_plus_a13b13,
                a11b12_plus_a12b22_plus_a13b23,
                a11b13_plus_a12b23_plus_a13b33,
                a21b11_plus_a22b12_plus_a23b13,
                a21b12_plus_a22b22_plus_a23b23,
                a21b13_plus_a22b23_plus_a23b33;
 
  if ( A == NULL || B == NULL || C == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != 3 || A->rows != 3 || B->cols != 3 || B->rows != 3 
      || C->cols != 3 || C->rows != 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }
    
  a11b11 = matel(A, 1, 1) * matel(B, 1, 1);
  a11b12 = matel(A, 1, 1) * matel(B, 1, 2);
  a11b13 = matel(A, 1, 1) * matel(B, 1, 3); 
  a21b11 = matel(A, 2, 1) * matel(B, 1, 1); 
  a21b12 = matel(A, 2, 1) * matel(B, 1, 2); 
  a21b13 = matel(A, 2, 1) * matel(B, 1, 3);
  a31b11 = matel(A, 3, 1) * matel(B, 1, 1); 
  a31b12 = matel(A, 3, 1) * matel(B, 1, 2); 
  a31b13 = matel(A, 3, 1) * matel(B, 1, 3);
  a12b12 = matel(A, 1, 2) * matel(B, 1, 2);
  a12b22 = matel(A, 1, 2) * matel(B, 2, 2); 
  a12b23 = matel(A, 1, 2) * matel(B, 2, 3);
  a22b12 = matel(A, 2, 2) * matel(B, 1, 2);
  a22b22 = matel(A, 2, 2) * matel(B, 2, 2); 
  a22b23 = matel(A, 2, 2) * matel(B, 2, 3);
  a32b12 = matel(A, 3, 2) * matel(B, 1, 2);
  a32b22 = matel(A, 3, 2) * matel(B, 2, 2);
  a32b23 = matel(A, 3, 2) * matel(B, 2, 3);
  a13b13 = matel(A, 1, 3) * matel(B, 1, 3);
  a13b23 = matel(A, 1, 3) * matel(B, 2, 3);
  a13b33 = matel(A, 1, 3) * matel(B, 3, 3);
  a23b13 = matel(A, 2, 3) * matel(B, 1, 3);
  a23b23 = matel(A, 2, 3) * matel(B, 2, 3);
  a23b33 = matel(A, 2, 3) * matel(B, 3, 3);
  a33b13 = matel(A, 3, 3) * matel(B, 1, 3);
  a33b23 = matel(A, 3, 3) * matel(B, 2, 3);
  a33b33 = matel(A, 3, 3) * matel(B, 3, 3);

  a11b11_plus_a12b12_plus_a13b13 = a11b11 + a12b12 + a13b13;
  a11b12_plus_a12b22_plus_a13b23 = a11b12 + a12b22 + a13b23;
  a11b13_plus_a12b23_plus_a13b33 = a11b13 + a12b23 + a13b33;
  a21b11_plus_a22b12_plus_a23b13 = a21b11 + a22b12 + a23b13;
  a21b12_plus_a22b22_plus_a23b23 = a21b12 + a22b22 + a23b23;
  a21b13_plus_a22b23_plus_a23b33 = a21b13 + a22b23 + a23b33;

  matel(C, 1, 1) = matel(A, 1, 1) * (a11b11_plus_a12b12_plus_a13b13) + 
                   matel(A, 1, 2) * (a11b12_plus_a12b22_plus_a13b23) + 
                   matel(A, 1, 3) * (a11b13_plus_a12b23_plus_a13b33);

  matel(C, 1, 2) = matel(A, 2, 1) * (a11b11_plus_a12b12_plus_a13b13) + 
                   matel(A, 2, 2) * (a11b12_plus_a12b22_plus_a13b23) + 
		   matel(A, 2, 3) * (a11b13_plus_a12b23_plus_a13b33);

  matel(C, 1, 3) = matel(A, 3, 1) * (a11b11_plus_a12b12_plus_a13b13) + 
                   matel(A, 3, 2) * (a11b12_plus_a12b22_plus_a13b23) + 
                   matel(A, 3, 3) * (a11b13_plus_a12b23_plus_a13b33);

  matel(C, 2, 1) = matel(C, 1, 2);

  matel(C, 2, 2) = matel(A, 2, 1) * (a21b11_plus_a22b12_plus_a23b13) + 
                   matel(A, 2, 2) * (a21b12_plus_a22b22_plus_a23b23) + 
                   matel(A, 2, 3) * (a21b13_plus_a22b23_plus_a23b33);

  matel(C, 2, 3) = matel(A, 3, 1) * (a21b11_plus_a22b12_plus_a23b13) + 
                   matel(A, 3, 2) * (a21b12_plus_a22b22_plus_a23b23) + 
                   matel(A, 3, 3) * (a21b13_plus_a22b23_plus_a23b33);

  matel(C, 3, 1) = matel(C, 1, 3);

  matel(C, 3, 2) = matel(C, 2, 3);

  matel(C, 3, 3) = matel(A, 3, 1) * (a31b11 + a32b12 + a33b13) + 
                   matel(A, 3, 2) * (a31b12 + a32b22 + a33b23) + 
                   matel(A, 3, 3) * (a31b13 + a32b23 + a33b33);

  return C;
}

/*****************************************************************************/

/* Calculates the triple product A^T * M * B, where A and B are 
   3 * 1 Hor_Matrix vectors, and M is a 3 * 3 Hor_Matrix.
   Returns a double answer. */

double hor_matq_ATMB3 (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B)
{
  if ( A == NULL || M == NULL || B == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1.0;
  }

  if (A->cols != 1 || A->rows != 3 || M->cols != 3 || M->rows != 3 
      || B->cols != 1 || B->rows != 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return -1.0;
  }

  return matel(A, 1, 1) * matel(M, 1, 1) * matel(B, 1, 1)
       + matel(A, 1, 1) * matel(M, 1, 2) * matel(B, 2, 1)
       + matel(A, 1, 1) * matel(M, 1, 3) * matel(B, 3, 1)
       + matel(A, 2, 1) * matel(M, 2, 1) * matel(B, 1, 1)
       + matel(A, 2, 1) * matel(M, 2, 2) * matel(B, 2, 1)
       + matel(A, 2, 1) * matel(M, 2, 3) * matel(B, 3, 1)
       + matel(A, 3, 1) * matel(M, 3, 1) * matel(B, 1, 1)
       + matel(A, 3, 1) * matel(M, 3, 2) * matel(B, 2, 1)
       + matel(A, 3, 1) * matel(M, 3, 3) * matel(B, 3, 1);
}

/*****************************************************************************/

/* Its close relation which calculates the triple product A * M * BT, where 
   A and B are 1 * 3 Hor_Matrix row vectors, and M is a 3 * 3 Hor_Matrix.
   Returns a double answer. */

double hor_matq_AMBT3 (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B)
{
  if ( A == NULL || M == NULL || B == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1.0;
  }

  if (A->cols != 3 || A->rows != 1 || M->cols != 3 || M->rows != 3 
      || B->cols != 3 || B->rows != 1) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return -1.0;
  }

  return matel(A, 1, 1) * matel(M, 1, 1) * matel(B, 1, 1)
       + matel(A, 1, 1) * matel(M, 1, 2) * matel(B, 1, 2)
       + matel(A, 1, 1) * matel(M, 1, 3) * matel(B, 1, 3)
       + matel(A, 1, 2) * matel(M, 2, 1) * matel(B, 1, 1)
       + matel(A, 1, 2) * matel(M, 2, 2) * matel(B, 1, 2)
       + matel(A, 1, 2) * matel(M, 2, 3) * matel(B, 1, 3)
       + matel(A, 1, 3) * matel(M, 3, 1) * matel(B, 1, 1)
       + matel(A, 1, 3) * matel(M, 3, 2) * matel(B, 1, 2)
       + matel(A, 1, 3) * matel(M, 3, 3) * matel(B, 1, 3);
}


/* General version of hor_matq_ATMB3, quick because the caller provides
 * the intermediate matrix
 */

Hor_Matrix *hor_matq_ATMB (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			   Hor_Matrix *W, Hor_Matrix *C)
{
  if (A == NULL || M == NULL || B == NULL || W == NULL || C == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->rows != M->rows || M->cols != B->rows ||
      W->rows != A->cols || W->cols != M->cols ||
      C->rows != A->cols || C->cols != B->cols) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  hor_matq_ATB (A, M, W);
  if (hor_errno != HORATIO_OK) return NULL;
  hor_matq_prod2 (W, B, C);
  if (hor_errno != HORATIO_OK) return NULL;

  return C;
}


/* General version of hor_matq_AMBT3, quick because the caller provides
 * the intermediate matrix
 */

Hor_Matrix *hor_matq_AMBT (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			   Hor_Matrix *W, Hor_Matrix *C)
{
  if (A == NULL || M == NULL || B == NULL || W == NULL || C == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != M->rows || M->cols != B->cols ||
      W->rows != A->rows || W->cols != M->cols ||
      C->rows != A->rows || C->cols != B->rows) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  hor_matq_prod2 (A, M, W);
  hor_matq_ABT (W, B, C);
  return C;
}

/* And again, this time it's A*M^T*B^T
 */

Hor_Matrix *hor_matq_AMTBT (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			    Hor_Matrix *W, Hor_Matrix *C)
{
  if (A == NULL || M == NULL || B == NULL || W == NULL || C == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != M->cols || M->rows != B->cols ||
      W->rows != A->rows || W->cols != M->rows ||
      C->rows != A->rows || C->cols != B->rows) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  hor_matq_ABT (A, M, W);
  hor_matq_ABT (W, B, C);
  return C;
}

/*****************************************************************************/

/* Calculates the product C = A . B^T in one step for general matrices */
static void mat_prod2T ( double **A, double **B, double **C,
		        int rows, int cols, int common )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < common; i++ )
	    sum += A[r][i]*B[c][i];

	 C[r][c] = sum;
      }
}

/* Calculates the product C = A^T . B in one step for general matrices */
static void mat_prodT2 ( double **A, double **B, double **C,
			 int rows, int cols, int common )
{
   int    r, c, i;
   double sum;

   for ( r = 0; r < rows; r++ )
      for ( c = 0; c < cols; c++ )
      {
	 sum = 0.0;
	 for ( i = 0; i < common; i++ )
	    sum += A[i][r]*B[i][c];

	 C[r][c] = sum;
      }
}

/* Simple C = A . B for general matrices */
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

Hor_Matrix *hor_matq_ABT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C )
{
   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( A->cols != B->cols || !hor_mat_big_enough ( C, A->rows, B->rows ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prod2T ( A->m, B->m, C->m, A->rows, B->rows, A->cols );
   C->rows = A->rows;
   C->cols = B->rows;
   return C;
}

/* Similarly for A^T . B
 */
Hor_Matrix *hor_matq_ATB (Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C)
{
   if ( A == NULL || B == NULL || C == NULL ) {
      hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
      return NULL;
   }

   if ( A->rows != B->rows || !hor_mat_big_enough ( C, A->cols, B->cols ) ) {
      hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
      return NULL;
   }

   mat_prodT2 ( A->m, B->m, C->m, A->cols, B->cols, A->rows );
   C->rows = A->cols;
   C->cols = B->cols;
   return C;
}

/* This function isn't quite "fast", meaning that it does no
   allocation: it will allocate the first time round, and then later
   if that isn't big enough it will re-allocate. */

Hor_Matrix *hor_matq_ABAT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C)
{
  static Hor_Matrix *AB;    // Space to store the first product, AB

  if ( A == NULL || B == NULL || C == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if ( A->cols != B->rows || B->cols != B->rows ||
       C->rows != A->rows || C->cols != C->rows) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  if (AB == NULL)
    AB = hor_mat_alloc (A->rows, B->cols);
  else if ( !hor_mat_big_enough ( AB, A->rows, B->cols ) ) 
  {
    hor_mat_free(AB);
    AB = hor_mat_alloc (A->rows, B->cols);
  }

  mat_prod2 (A->m, B->m, AB->m, A->rows, B->cols, A->cols);
  mat_prod2T (AB->m, A->m, C->m, A->rows, A->rows, A->cols);

  return C;
}

/* Whereas this is the genuinely fast version, the caller providing the
 * intermediate matrix W.
 * JGHK addition 17/9/00.
 */

Hor_Matrix *hor_matq_ABAT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *W, Hor_Matrix *C)
{
  if (A == NULL || B == NULL || W == NULL || C == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != B->rows || B->cols != B->rows ||
      W->rows != A->rows || W->cols != B->cols ||
      C->rows != A->rows || C->cols != C->rows) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  mat_prod2 (A->m, B->m, W->m, A->rows, B->cols, A->cols);
  mat_prod2T (W->m, A->m, C->m, A->rows, A->rows, A->cols);
  

  return C;
}

/*****************************************************************************/

/* Calculate A * A^T for a 3 * 1 vector. Answer B is 3 * 3 matrix */

Hor_Matrix *hor_matq_AAT3 (Hor_Matrix *A, Hor_Matrix *B)
{
  if ( A == NULL || B == NULL ) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if ( A->cols != 1 || A->rows != 3 || B->cols != 3 || B->rows != 3 ) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  static double B12, B13, B23;

  B12 =	matel(A, 1, 1) * matel(A, 2, 1);
  B13 = matel(A, 1, 1) * matel(A, 3, 1);
  B23 = matel(A, 2, 1) * matel(A, 3, 1);

  hor_matq_fill(B, 
		matel(A, 1, 1) * matel(A, 1, 1),
		B12,
		B13,
		B12,
		matel(A, 2, 1) * matel(A, 2, 1),
		B23,
		B13,
		B23,
		matel(A, 3, 1) * matel(A, 3, 1) );
  
  return B;
}

/*****************************************************************************/

/* Copy a 3*3 matrix into a block of a large matrix starting at row r
   and column c (meaning r = 0, c = 0 is the top-left corner block)*/

Hor_Matrix *hor_matq_insert_chunk33(Hor_Matrix *A, Hor_Matrix *B, int r, int c)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != 3 || A->rows != 3 || B->cols < c + 3 || B->rows < r + 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < 3; rr++)
    for (int cc = 0; cc < 3; cc++)
      B->m[rr + r][cc + c] = A->m[rr][cc];

  return B;
}

/* Same as above, but transpose A when copying
 */

Hor_Matrix *hor_matq_insert_chunkyxT(Hor_Matrix *A, Hor_Matrix *B, int r, int c)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  int x = A->cols, y = A->rows;

  if (B->cols < c + y || B->rows < r + x) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < x; rr++)
    for (int cc = 0; cc < y; cc++)
      B->m[rr + r][cc + c] = A->m[cc][rr];

  return B;
}

/*****************************************************************************/

/* Copy a 3*1 vector into a block of a large vector starting at row r
   (meaning r = 0 is the top block) */

Hor_Matrix *hor_matq_insert_chunk31(Hor_Matrix *A, Hor_Matrix *B, int r)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != 1 || A->rows != 3 || B->cols != 1 || B->rows < r + 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < 3; rr++)
    B->m[rr + r][0] = A->m[rr][0];

  return B;
}

/*****************************************************************************/

/* Copy a block of a large matrix starting at row r and column c 
   (meaning r = 0, c = 0 is the top-left corner block) into a 3*3 matrix */

Hor_Matrix *hor_matq_extract_chunk33(Hor_Matrix *B, Hor_Matrix *A, 
                                     int r, int c)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != 3 || A->rows != 3 || B->cols < c + 3 || B->rows < r + 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < 3; rr++)
    for (int cc = 0; cc < 3; cc++)
      A->m[rr][cc] = B->m[rr + r][cc + c];

  return A;
}

/*****************************************************************************/

/* Copy a block of a large vector starting at row r
   (meaning r = 0 is the top block) into a 3*1 vector */

Hor_Matrix *hor_matq_extract_chunk31(Hor_Matrix *B, Hor_Matrix *A, int r)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (A->cols != 1 || A->rows != 3 || B->cols != 1 || B->rows < r + 3) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < 3; rr++)
    A->m[rr][0] = B->m[rr + r][0];

  return A;
}

/*************************Generalised Insert / Extract************************/

/* Copy a y*x matrix A into a block of a large matrix B starting at row r
   and column c (meaning r = 0, c = 0 is the top-left corner block)*/

Hor_Matrix *hor_matq_insert_chunkyx(Hor_Matrix *A, Hor_Matrix *B, int r, int c)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  int x = A->cols, y = A->rows;

  if (B->cols < c + x || B->rows < r + y) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < y; rr++)
    for (int cc = 0; cc < x; cc++)
      B->m[rr + r][cc + c] = A->m[rr][cc];

  return B;
}

/*****************************************************************************/

/* Copy a y*1 vector A into a block of a large vector B starting at row r
   (meaning r = 0 is the top block) */

Hor_Matrix *hor_matq_insert_chunky1(Hor_Matrix *A, Hor_Matrix *B, int r)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  int y = A->rows;

  if (A->cols != 1 || B->cols != 1 || B->rows < r + y) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < y; rr++)
    B->m[rr + r][0] = A->m[rr][0];

  return B;
}

/*****************************************************************************/

/* Copy a block of a large matrix B starting at row r and column c
   (meaning r = 0, c = 0 is the top-left corner block) into a y*x
   matrix A */

Hor_Matrix *hor_matq_extract_chunkyx(Hor_Matrix *B, Hor_Matrix *A, 
                                     int r, int c)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  int x = A->cols, y = A->rows;

  if (B->cols < c + x || B->rows < r + y) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < y; rr++)
    for (int cc = 0; cc < x; cc++)
      A->m[rr][cc] = B->m[rr + r][cc + c];

  return A;
}

/*****************************************************************************/

/* Copy a block of a large vector B starting at row r
   (meaning r = 0 is the top block) into a y*1 vector A */

Hor_Matrix *hor_matq_extract_chunky1(Hor_Matrix *B, Hor_Matrix *A, int r)
{
  if ( A == NULL || B == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  int y = A->rows;

  if (A->cols != 1 || B->cols != 1 || B->rows < r + y) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int rr = 0; rr < y; rr++)
    A->m[rr][0] = B->m[rr + r][0];

  return A;
}

/*****************************************************************************/

/* Extract a row of matrix A into row vector R. r is the row number
   (with zero being the top row) */

Hor_Matrix *hor_matq_extract_row(Hor_Matrix *A, Hor_Matrix *R, int r)
{
  if ( A == NULL || R == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  if (R->cols != A->cols || R->rows != 1 || r > A->rows - 1) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int c = 0; c < A->cols; c++)
    R->m[0][c] = A->m[r][c];

  return R;
}

/*****************************************************************************/

/* A function which will calculate the best intersection of two lines as the
   midpoint of the line which perpendicularly bisects both.

   Arguments: pointers to 4 Hor_Matrix vectors. Two are points lying on the 
   lines, and two are direction vectors (probably unit, but they don't have to
   be). The fifth Hor_Matrix will be filled with the answer.

   One line is r = a + lambda b; other is r = c + mu d.
*/

Hor_Matrix *hor_matq_vector_intersect(Hor_Matrix *a, Hor_Matrix *b, 
				       Hor_Matrix *c, Hor_Matrix *d,
				       Hor_Matrix *r)
{
  assert (a != NULL && b != NULL && c != NULL && d != NULL && r != NULL);

  assert ( a->cols == 1 && a->rows == 3
      && b->cols == 1 && b->rows == 3
      && c->cols == 1 && c->rows == 3
      && d->cols == 1 && d->rows == 3
      && r->cols == 1 && r->rows == 3 );

  static Hor_Matrix *r1, *r2;         /* crap matrices for calculation */
  double A, B, C, D, E, F;   /* variables to use in the Kramer's soln. */
  double lambda, mu;         /* for the parameters when we find them   */
  double denom;

  if(r1 == NULL)
  {
    r1 = hor_mat_alloc(3, 1);
    r2 = hor_mat_alloc(3, 1);
  }

  A = -hor_matq_dot31(b, b);
  B = hor_matq_dot31(b, d);
  C = hor_matq_dot31(a, b) - hor_matq_dot31(b, c);
  D = -hor_matq_dot31(b, d);
  E = hor_matq_dot31(d, d);
  F = hor_matq_dot31(a, d) - hor_matq_dot31(c, d);
  denom = (A * E - D * B);
  lambda = (C * E - F * B) / denom;
  mu = (A * F - D * C) / denom;

  hor_matq_scale( b, lambda );
  hor_matq_scale( d, mu );

  hor_matq_add2( a, b, r1 );
  hor_matq_add2( c, d, r2 );

  hor_matq_add2( r1, r2, r);

  hor_matq_scale(r, 0.5);

  return r;
}

// Helper functions for hor_matq_vector_intersect_with_jacobians()
// Calculate generic parts of Jacobian (A generically represents a, b, c or d)
// HorI generically represents H or I! lambdaormu represents lambda or mu!
int dlambdaormu_by_dA(double HorI, double G, 
		      Hor_Matrix *dHorI_by_dA, Hor_Matrix *dG_by_dA, 
		      Hor_Matrix *dlambdaormu_by_dA)
{
  Hor_Matrix *temp1 = hor_mats_scale(dHorI_by_dA, (1.0 / G));
  Hor_Matrix *temp2 = hor_mats_scale(dG_by_dA, (-HorI / (G * G)));

  hor_matq_add2(temp1, temp2, dlambdaormu_by_dA);

  hor_mat_free_list(temp1, temp2, NULL);

  return 0;
}

int dlambda_by_dA(double H, double G, 
		  Hor_Matrix *dH_by_dA, Hor_Matrix *dG_by_dA, 
		  Hor_Matrix *dlambda_by_dA)
{
  dlambdaormu_by_dA(H, G, dH_by_dA, dG_by_dA, dlambda_by_dA);

  return 0;
}

int dmu_by_dA(double I, double G, 
		  Hor_Matrix *dI_by_dA, Hor_Matrix *dG_by_dA, 
		  Hor_Matrix *dmu_by_dA)
{
  dlambdaormu_by_dA(I, G, dI_by_dA, dG_by_dA, dmu_by_dA);

  return 0;
}

// Calculate the closest intersection of 2 lines but with Jacobians too
Hor_Matrix *hor_matq_vector_intersect_with_jacobians(
       Hor_Matrix *a, Hor_Matrix *b, Hor_Matrix *c, Hor_Matrix *d,
       Hor_Matrix *r,
       Hor_Matrix *dr_by_da, Hor_Matrix *dr_by_db, Hor_Matrix *dr_by_dc, 
       Hor_Matrix *dr_by_dd)
{
  assert (a != NULL && b != NULL && c != NULL && d != NULL && r != NULL);

  assert ( a->cols == 1 && a->rows == 3
	   && b->cols == 1 && b->rows == 3
	   && c->cols == 1 && c->rows == 3
	   && d->cols == 1 && d->rows == 3
	   && r->cols == 1 && r->rows == 3
	   && dr_by_da->rows == 3 && dr_by_da->cols == 3
	   && dr_by_db->rows == 3 && dr_by_db->cols == 3
	   && dr_by_dc->rows == 3 && dr_by_dc->cols == 3
	   && dr_by_dd->rows == 3 && dr_by_dd->cols == 3
	   );

  // Temp matrices for calculation
  // Let's allocate them every time and hope it's fast because of caching
  Hor_Matrix *r1, *r2;
  Hor_Matrix *dlambda_by_da, *dlambda_by_db, *dlambda_by_dc, *dlambda_by_dd,
    *dmu_by_da, *dmu_by_db, *dmu_by_dc, *dmu_by_dd,
    *dG_by_da, *dG_by_db, *dG_by_dc, *dG_by_dd, 
    *dH_by_da, *dH_by_db, *dH_by_dc, *dH_by_dd, 
    *dI_by_da, *dI_by_db, *dI_by_dc, *dI_by_dd,
    *temp1, *temp2, *temp3, *temp4,
    *temp331, *temp332, *temp333, *temp334;
  double A, B, C, D, E, F, G, H, I;  /* variables for the Kramer's soln. */
  double lambda, mu;                 /* for the parameters when we find them */

  // Allocate matrices
  r1 = hor_mat_alloc(3, 1);
  r2 = hor_mat_alloc(3, 1);
  dlambda_by_da = hor_mat_alloc(1, 3);
  dlambda_by_db = hor_mat_alloc(1, 3);
  dlambda_by_dc = hor_mat_alloc(1, 3);
  dlambda_by_dd = hor_mat_alloc(1, 3);
  dmu_by_da = hor_mat_alloc(1, 3);
  dmu_by_db = hor_mat_alloc(1, 3);
  dmu_by_dc = hor_mat_alloc(1, 3);
  dmu_by_dd = hor_mat_alloc(1, 3);
  dG_by_da = hor_mat_alloc(1, 3);
  dG_by_db = hor_mat_alloc(1, 3);
  dG_by_dc = hor_mat_alloc(1, 3);
  dG_by_dd = hor_mat_alloc(1, 3);
  dH_by_da = hor_mat_alloc(1, 3);
  dH_by_db = hor_mat_alloc(1, 3);
  dH_by_dc = hor_mat_alloc(1, 3);
  dH_by_dd = hor_mat_alloc(1, 3);
  dI_by_da = hor_mat_alloc(1, 3);
  dI_by_db = hor_mat_alloc(1, 3);
  dI_by_dc = hor_mat_alloc(1, 3);
  dI_by_dd = hor_mat_alloc(1, 3);
  temp1 = hor_mat_alloc(1, 3);
  temp2 = hor_mat_alloc(1, 3);
  temp3 = hor_mat_alloc(1, 3);
  temp4 = hor_mat_alloc(1, 3);
  temp331 = hor_mat_alloc(3, 3);
  temp332 = hor_mat_alloc(3, 3);
  temp333 = hor_mat_alloc(3, 3);
  temp334 = hor_mat_alloc(3, 3);

  // Calculate the output
  A = -hor_matq_dot31(b, b);
  B = hor_matq_dot31(b, d);
  C = hor_matq_dot31(a, b) - hor_matq_dot31(b, c);
  D = -hor_matq_dot31(b, d);
  E = hor_matq_dot31(d, d);
  F = hor_matq_dot31(a, d) - hor_matq_dot31(c, d);
  G = (A * E - D * B);
  H = (C * E - F * B);
  I = (A * F - D * C);

  lambda = H / G;
  mu = I / G;

  hor_matq_scale( b, lambda );
  hor_matq_scale( d, mu );

  hor_matq_add2( a, b, r1 );
  hor_matq_add2( c, d, r2 );

  hor_matq_add2( r1, r2, r);

  hor_matq_scale(r, 0.5);

  // And calculate the Jacobian --- hang in there
  hor_matq_zero(dG_by_da);

  hor_matq_transpose(b, temp1);
  hor_matq_scale(temp1, (-2.0 * E));
  hor_matq_transpose(d, temp2);
  hor_matq_scale(temp2, (B - D));
  hor_matq_add2(temp1, temp2, dG_by_db);

  hor_matq_zero(dG_by_dc);

  hor_matq_transpose(d, temp1);
  hor_matq_scale(temp1, (2.0 * A));
  hor_matq_transpose(b, temp2);
  hor_matq_scale(temp2, (B - D));
  hor_matq_add2(temp1, temp2, dG_by_dd);

  hor_matq_transpose(b, temp1);
  hor_matq_scale(temp1, E);
  hor_matq_transpose(d, temp2);
  hor_matq_scale(temp2, (-B));
  hor_matq_add2(temp1, temp2, dH_by_da);

  hor_matq_transpose(a, temp1);
  hor_matq_transpose(c, temp2);
  hor_matq_sub(temp1, temp2, temp3);
  hor_matq_scale(temp3, E);
  hor_matq_transpose(d, temp4);
  hor_matq_scale(temp4, (-F));
  hor_matq_add2(temp3, temp4, dH_by_db);

  hor_matq_transpose(b, temp1);
  hor_matq_scale(temp1, (-E));
  hor_matq_transpose(d, temp2);
  hor_matq_scale(temp2, B);
  hor_matq_add2(temp1, temp2, dH_by_dc);

  hor_matq_transpose(d, temp1);
  hor_matq_scale(temp1, (2.0 * C));
  hor_matq_transpose(b, temp2);
  hor_matq_scale(temp2, (-F));
  hor_matq_add2(temp1, temp2, temp3); // hold on to temp3
  hor_matq_transpose(a, temp1);
  hor_matq_transpose(c, temp2);
  hor_matq_sub(a, c, temp4);
  hor_matq_scale(temp4, (-B));
  hor_matq_add2(temp3, temp4, dH_by_dd);

  hor_matq_transpose(d, temp1);
  hor_matq_scale(temp1, A);
  hor_matq_transpose(b, temp2);
  hor_matq_scale(temp2, (-D));
  hor_matq_add2(temp1, temp2, dI_by_da);

  hor_matq_transpose(b, temp1);
  hor_matq_scale(temp1, (-2.0 * F));
  hor_matq_transpose(d, temp2);
  hor_matq_scale(temp2, C);
  hor_matq_add2(temp1, temp2, temp3); // hold on to temp3
  hor_matq_transpose(a, temp1);
  hor_matq_transpose(c, temp2);
  hor_matq_sub(temp1, temp2, temp4);
  hor_matq_scale(temp4, (-D));
  hor_matq_add2(temp3, temp4, dI_by_db);

  hor_matq_transpose(d, temp1);
  hor_matq_scale(temp1, (-A));
  hor_matq_transpose(b, temp2);
  hor_matq_scale(temp2, D);
  hor_matq_add2(temp1, temp2, dI_by_dc);

  hor_matq_transpose(a, temp1);
  hor_matq_transpose(c, temp2);
  hor_matq_sub(temp1, temp2, temp3);
  hor_matq_scale(temp3, A);
  hor_matq_transpose(b, temp4);
  hor_matq_scale(temp4, C);
  hor_matq_add2(temp3, temp4, dI_by_dd);

  // OK, next find Jacobians for parameters lambda and mu
  // Use helper functions above
  dlambda_by_dA(H, G, dH_by_da, dG_by_da, dlambda_by_da);
  dlambda_by_dA(H, G, dH_by_db, dG_by_db, dlambda_by_db);
  dlambda_by_dA(H, G, dH_by_dc, dG_by_dc, dlambda_by_dc);
  dlambda_by_dA(H, G, dH_by_dd, dG_by_dd, dlambda_by_dd);

  dmu_by_dA(I, G, dI_by_da, dG_by_da, dmu_by_da);
  dmu_by_dA(I, G, dI_by_db, dG_by_db, dmu_by_db);
  dmu_by_dA(I, G, dI_by_dc, dG_by_dc, dmu_by_dc);
  dmu_by_dA(I, G, dI_by_dd, dG_by_dd, dmu_by_dd);

  // Now finally calculate the answers...
  hor_matq_identity(temp331);
  hor_matq_prod2(b, dlambda_by_da, temp332);
  hor_matq_prod2(d, dmu_by_da, temp333);
  hor_matq_add2(temp331, temp332, temp334);
  hor_matq_add2(temp334, temp333, dr_by_da);
  hor_matq_scale(dr_by_da, 0.5);

  hor_matq_identity(temp331);
  hor_matq_scale(temp331, lambda);
  hor_matq_prod2(b, dlambda_by_db, temp332);
  hor_matq_prod2(d, dmu_by_db, temp333);
  hor_matq_add2(temp331, temp332, temp334);
  hor_matq_add2(temp334, temp333, dr_by_db);
  hor_matq_scale(dr_by_db, 0.5);

  hor_matq_prod2(b, dlambda_by_dc, temp331);
  hor_matq_identity(temp332);
  hor_matq_prod2(d, dmu_by_dc, temp333);
  hor_matq_add2(temp331, temp332, temp334);
  hor_matq_add2(temp334, temp333, dr_by_dc);
  hor_matq_scale(dr_by_dc, 0.5);

  hor_matq_prod2(b, dlambda_by_dd, temp331);
  hor_matq_identity(temp332);
  hor_matq_scale(temp332, mu);
  hor_matq_prod2(d, dmu_by_dd, temp333);
  hor_matq_add2(temp331, temp332, temp334);
  hor_matq_add2(temp334, temp333, dr_by_dd);
  hor_matq_scale(dr_by_dd, 0.5);

  hor_mat_free_list (r1, r2, 
    dlambda_by_da, dlambda_by_db, dlambda_by_dc, dlambda_by_dd,
    dmu_by_da, dmu_by_db, dmu_by_dc, dmu_by_dd,
    dG_by_da, dG_by_db, dG_by_dc, dG_by_dd, 
    dH_by_da, dH_by_db, dH_by_dc, dH_by_dd, 
    dI_by_da, dI_by_db, dI_by_dc, dI_by_dd,
    temp1, temp2, temp3, temp4,
    temp331, temp332, temp333, temp334, 
    NULL);

  return r;
}

/*****************************************************************************/

/* Allows Hor_Matrices to be output in the C++ way */

ostream& operator<<(ostream& os, Hor_Matrix *M)
{
  if (M == NULL)
    return os << "Attempt to print NULL matrix.\n";

  os << "\n";

  int p = os.precision();
  //  os.precision(3);

  int r, c;
  for ( r = 0; r < M->rows; r++ )
  {
    for ( c = 0; c < M->cols; c++ )
    {
      os.width(9);
      os << M->m[r][c] << " ";
    }
    os << "\n";
  }
  
  os.precision(p);

  return os;
}

/*****************************************************************************/

/* This cunning baby is a way of being able to have variable size matrices
 * without the overhead of constant reallocation.  It will only reallocate to
 * _increase_ the size of the matrix, otherwise it will just change the
 * "rows" and "cols" data members.
 */

Hor_Matrix *hor_mat_ensure_size (int rows, int cols, Hor_Matrix **pM)
{
  if (rows <= 0 || cols <= 0) {
    hor_errno = HOR_MATH_ILLEGAL_VALUE;
    return NULL;
  }

  Hor_Matrix *&M = *pM;

  // Define ALWAYS_REALLOCATE in Makefile if, on your machine, it is faster to
  // do so because matrices always remain in the cache rather than being placed
  // in main mem, which results in multiple page faults: ie. always reallocating
  // can be faster on some machines.
#ifdef ALWAYS_REALLOCATE
  if (M != NULL) hor_mat_free (M);
  M = hor_mat_alloc (rows, cols);
#else
  if (M == NULL) {
    M = hor_mat_alloc (rows, cols);
  }
  else if (M->rsize < rows && M->csize < cols) {
    hor_mat_free (M);
    M = hor_mat_alloc (rows, cols);
  }
  else if (M->rsize < rows) {
    int c = M->csize;
    hor_mat_free (M);
    M = hor_mat_alloc (rows, c);
  }
  else if (M->csize < cols) {
    int r = M->rsize;
    hor_mat_free (M);
    M = hor_mat_alloc (r, cols);
  }
  else {
    M->rows = rows;
    M->cols = cols;
  }
#endif

  return M;
}

/* List version of same
 */

int hor_mat_ensure_size_list (int rows, int cols ...)
{
  va_list ap;
  va_start (ap, cols);

  while (true) {
    Hor_Matrix **pM = va_arg (ap, Hor_Matrix **);
    if (pM == NULL) break;
    Hor_Matrix *out = hor_mat_ensure_size (rows, cols, pM);
    if (out == NULL) {
      va_end (ap);
      return hor_errno;
    }
  }

  va_end (ap);
  return 0;
}

/* Quick addition of four matrices
 */

Hor_Matrix *hor_matq_add4 (Hor_Matrix *M, Hor_Matrix *A, Hor_Matrix *B,
			   Hor_Matrix *C, Hor_Matrix *D)
{
  if (M == NULL || A == NULL || B == NULL || C == NULL || D == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }
  if (!hor_mat_same_size3 (M, A, B) || !hor_mat_same_size3 (M, C, D)) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int r = 0; r < M->rows; r++) {
    for (int c = 0; c < M->cols; c++) {
      M->m[r][c] = A->m[r][c] + B->m[r][c] + C->m[r][c] + D->m[r][c];
    }
  }

  return M;
}


/* Quick subtraction: M = M - (A + B + C + D)
 */

Hor_Matrix *hor_matq_sub4 (Hor_Matrix *M, Hor_Matrix *A, Hor_Matrix *B,
			   Hor_Matrix *C, Hor_Matrix *D)
{
  if (M == NULL || A == NULL || B == NULL || C == NULL || D == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }
  if (M->rows != A->rows || M->rows != B->rows || M->rows != C->rows ||
      M->rows != C->rows || M->cols != A->cols || M->cols != B->cols ||
      M->cols != C->cols || M->cols != D->cols) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  for (int r = 0; r < M->rows; r++) {
    for (int c = 0; c < M->cols; c++) {
      M->m[r][c] -= A->m[r][c] + B->m[r][c] + C->m[r][c] + D->m[r][c];
    }
  }

  return M;
}


/* Correct a covariance matrix that should be symmetric in a way that
 * doesn't result in it being an overconfident matrix
 */

Hor_Matrix *hor_mat_symmetrise (Hor_Matrix *M)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }
  if (!hor_mat_is_square (M)) {
    hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
    return NULL;
  }

  static Hor_Matrix *Mt = NULL;
  static Hor_Matrix *U  = NULL;
  static Hor_Matrix *W1 = NULL;
  static Hor_Matrix *W2 = NULL;
  static Hor_Matrix *V  = NULL;
  static Hor_Matrix *Vt  = NULL;
  static Hor_Matrix *R  = NULL;
  hor_mat_ensure_size_list (M->rows, M->cols, &Mt, &U, &W1, &V, &Vt, &R, NULL);
  hor_mat_ensure_size (M->rows, 1, &W2);
  hor_matq_zero (W1);

  hor_matq_copy (M, U);
  hor_matq_svdcmp (U, W1->m[0], V);

  hor_matq_transpose (M, Mt);
  hor_matq_add2 (M, Mt, U);
  hor_matq_scale (U, 0.5);
  hor_matq_svdcmp (U, W2->m[0], V);

  // Form diagonal matrix of original singular values
  for (int c = 1; c < M->cols; c++) {
    W1->m[c][c] = W1->m[0][c];
    W1->m[0][c] = 0.0;
  }

  hor_matq_transpose (V, Vt);
  hor_matq_prod3 (U, W1, Vt, V, R);

  //  cerr << "Symmetrising:\n" << M << endl;
  hor_matq_sub (M, R, U);
  //  cerr << "Removed asymmetric part:\n" << U << endl;
  //  cerr << "To give:\n" << R << endl;
  hor_matq_copy (R, M);

  return M;
}


/* Unlike the inversion horatio provides, this inverts ANY invertible
 * matrix, i.e. one which is square and non-singular.  Caller provides
 * four matrices of the same size as input matrix M for working.  R is result.
 */

Hor_Matrix *hor_matq_inv_any (Hor_Matrix *M, Hor_Matrix *U, Hor_Matrix *W,
			      Hor_Matrix *V, Hor_Matrix *Ut, Hor_Matrix *R)
{
  if (M == NULL || U == NULL  || W == NULL ||
      V == NULL || Ut == NULL || R == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }
  if (!hor_mat_is_square (M)) {
    hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
    return NULL;
  }
  if (!hor_mat_same_size3 (M, U, W) || !hor_mat_same_size3 (M, V, Ut) ||
      !hor_mat_same_size2 (M, R)) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  hor_matq_copy (M, U);
  hor_matq_zero (W);
  hor_matq_svdcmp (U, W->m[0], V);
  hor_matq_transpose (U, Ut);

  // Form inverse diagonal W
  for (int c = 0; c < M->cols; c++) {
    if (W->m[0][c] <= BITS_ZERO_SINGULAR_VALUE) {
      hor_errno = HOR_MATH_MATRIX_SINGULAR;
      return NULL;
    }
    W->m[c][c] = 1.0 / W->m[0][c];
    if (c != 0) W->m[0][c] = 0.0;
  }

  // Form inverse result
  hor_matq_prod3 (V, W, Ut, U, R);

  if (hor_errno != HORATIO_OK) return NULL;
  else return R;
}


/* Basically the same as above, but will take the pseudo-inverse if the
 * matrix is singular, in other words, non-zero singular values are
 * inverted, and zero ones stay as zero
 */

Hor_Matrix *hor_matq_psinv (Hor_Matrix *M, Hor_Matrix *U, Hor_Matrix *W,
			    Hor_Matrix *V, Hor_Matrix *Ut, Hor_Matrix *R)
{
  if (M == NULL || U == NULL  || W == NULL ||
      V == NULL || Ut == NULL || R == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }
  if (!hor_mat_is_square (M)) {
    hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
    return NULL;
  }
  if (!hor_mat_same_size3 (M, U, W) || !hor_mat_same_size3 (M, V, Ut) ||
      !hor_mat_same_size2 (M, R)) {
    hor_errno = HOR_MATH_MATRIX_INCOMPATIBLE;
    return NULL;
  }

  hor_matq_copy (M, U);
  hor_matq_svdcmp (U, W->m[0], V);
  hor_matq_transpose (U, Ut);

  // Form inverse diagonal W
  for (int c = 0; c < M->cols; c++) {
    if (W->m[0][c] <= BITS_ZERO_SINGULAR_VALUE) W->m[c][c] = 0.0;
    else W->m[c][c] = 1.0 / W->m[0][c];
    if (c != 0) W->m[0][c] = 0.0;
  }

  // Form inverse result
  hor_matq_prod3 (V, W, Ut, U, R);

  if (hor_errno != HORATIO_OK) return NULL;
  else return R;
}


/* For testing purposes, print an error message if the provided matrix is
 * not symmetric
 */

int hor_mat_test_for_symmetry (const char *message, Hor_Matrix *M)
{
  assert (M != NULL);
  assert (hor_mat_is_square (M));

  for (int r = 0; r < M->rows; r++) {
    for (int c = r + 1; c < M->cols; c++) {
      double diff = fabs (M->m[r][c] = M->m[c][r]);
      if (diff > 0.0) {
	cout << "****** " << message << " NOT SYMMETRIC by " << diff << "******\n";
	return 1;
      }
    }
  }

  return 0;
}


/* Handy:  Take the square root of the sum of the squares of all the matrix
 * elements
 */

double hor_mat_root_sum_square (Hor_Matrix *M)
{
  return hor_frobenius_norm (M);
}


/* Some ways of calculating scalar `norms' of square matrices to get an idea
 * of their size.  The first is the H_infinity norm, the maximum singular
 * value (works for non-square matrices).  The second assumes the input
 * matrix is a covariance matrix (so square), and calculates the volume of
 * the uncertainty ellipsoid at n standard deviations.  Must therefore also
 * be symmetric.
 * They are not quick calculations.
 */

double hor_hinfnorm (Hor_Matrix *M)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1;
  }

  double *W = new double [M->cols];
  Hor_Matrix *V = hor_mat_alloc (M->cols, M->cols);
  Hor_Matrix *U = hor_mats_copy (M);

  hor_matq_svdcmp (U, W, V);
  assert (hor_errno == HORATIO_OK);
  double result = -1;
  for (int i = 0; i < M->cols; i++) {
    if (W[i] > result) result = W[i];
  }
  hor_mat_free_list (U, V, 0);
  delete W;

  return result;
}


/* This calculates 4/3 * PI * n^(dim) * sqrt (product of eigenvalues)
 * The `volume' here is fairly meaningless unless the matrix is 3x3, but
 * presumably what is returned is still proportional to the hyper-volume
 * of a higher-dimensional hyper-ellipsoid
 */

double hor_uncertainty_volume_3s (Hor_Matrix *M)
{
  return hor_uncertainty_volume (M, 3.0);
}

double hor_uncertainty_volume (Hor_Matrix *M, double n)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1.0;
  }
  if (!hor_mat_is_square (M)) {
    hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
    return -1.0;
  }
  assert (M->rows > 0);

  int size = M->rows;

  Hor_Matrix *A = hor_mats_copy (M);
  double *d = new double [size];
  double *e = new double [size * size];
  hor_matq_tred2 (A, d, e);
  hor_matq_tqli (d, e, A);

  double product = d[0];
  for (int i = 1; i < size; i++) product *= fabs (d[i]);

  return (4.0/3.0) * M_PI * pow (n, size) * sqrt (product);
}


/* This is the root sum-squared matrix elements
 */

double hor_frobenius_norm (Hor_Matrix *M)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1.0;
  }

  double sum = 0;
  for (int r = 0; r < M->rows; r++) {
    for (int c = 0; c < M->cols; c++) {
      sum += M->m[r][c] * M->m[r][c];
    }
  }

  return sqrt (sum);
}


double hor_trace (Hor_Matrix *M)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return -1.0;
  }
  if (!hor_mat_is_square (M)) {
    hor_errno = HOR_MATH_MATRIX_NOT_SQUARE;
    return -1.0;
  }
  assert (M->rows > 0);

  double sum = M->m[0][0];
  for (int i = 1; i < M->rows; i++) {
    sum += M->m[i][i];
  }

  return sum;
}


/* Set every element of matrix M to val
 */

Hor_Matrix *hor_matq_set_all (Hor_Matrix *M, double val)
{
  if (M == NULL) {
    hor_errno = HOR_MATH_NULL_POINTER_ARGUMENT;
    return NULL;
  }

  for (int r = 0; r < M->rows; r++) {
    for (int c = 0; c < M->cols; c++) {
      M->m[r][c] = val;
    }
  }

  return M;
}

/******************************Quaternion Stuff*******************************/

// Invert quaternion
Hor_Matrix *invert_quaternion(Hor_Matrix *q, Hor_Matrix *qbar)
{
  assert (q != NULL && qbar != NULL);

  assert (q->rows == 4 && q->cols == 1 &&
	  qbar->rows == 4 && qbar->cols == 1);

  hor_matq_fill(qbar,
		vecel(q, 1),
		-vecel(q, 2),
		-vecel(q, 3),
		-vecel(q, 4));

  return qbar;
}

Hor_Matrix *dqbar_by_dq(Hor_Matrix *dqbar_by_dq)
{
  assert (dqbar_by_dq != NULL);

  assert (dqbar_by_dq->rows == 4 && dqbar_by_dq->cols == 4);

  hor_matq_fill(dqbar_by_dq,
		1.0, 0.0, 0.0, 0.0,
		0.0, -1.0, 0.0, 0.0,
		0.0, 0.0, -1.0, 0.0,
		0.0, 0.0, 0.0, -1.0);

  return dqbar_by_dq;
}

/* Product of two quaternions q3 = q2 x q1 */
Hor_Matrix *prodq2q1(Hor_Matrix *q2, Hor_Matrix *q1, Hor_Matrix *q3)
{
  assert ( q2 != NULL && q1 != NULL && q3 != NULL );

  assert ( q2->rows == 4 && q2->cols == 1 &&
	   q1->rows == 4 && q1->cols == 1 &&
	   q3->rows == 4 && q3->cols == 1 );

  // Aliases for clarity
  double& q20 = vecel(q2, 1); 
  double& q2x = vecel(q2, 2); 
  double& q2y = vecel(q2, 3); 
  double& q2z = vecel(q2, 4); 
  double& q10 = vecel(q1, 1); 
  double& q1x = vecel(q1, 2); 
  double& q1y = vecel(q1, 3); 
  double& q1z = vecel(q1, 4); 

  hor_matq_fill( q3,
		 q20 * q10 - (q2x * q1x + q2y * q1y + q2z * q1z),
		 q20 * q1x + q10 * q2x + q2y * q1z - q1y * q2z,
		 q20 * q1y + q10 * q2y + q2z * q1x - q1z * q2x,
		 q20 * q1z + q10 * q2z + q2x * q1y - q1x * q2y );

  return q3;
}

/* Jacobians for quaternion product */
Hor_Matrix *dq3_by_dq1(Hor_Matrix *q2, Hor_Matrix *dq3_by_dq1)
{
  assert ( q2 != NULL && dq3_by_dq1 != NULL );

  assert ( q2->rows == 4 && q2->cols == 1 &&
	   dq3_by_dq1->rows == 4 && dq3_by_dq1->cols == 4 );
  
  double& q20 = vecel(q2, 1); 
  double& q2x = vecel(q2, 2); 
  double& q2y = vecel(q2, 3); 
  double& q2z = vecel(q2, 4); 

  hor_matq_fill( dq3_by_dq1,
		 q20, -q2x, -q2y, -q2z,
		 q2x, q20, -q2z, q2y,
		 q2y, q2z, q20, -q2x,
		 q2z, -q2y, q2x, q20);

  return dq3_by_dq1;
}

Hor_Matrix *dq3_by_dq2(Hor_Matrix *q1, Hor_Matrix *dq3_by_dq2)
{
  assert ( q1 != NULL && dq3_by_dq2 != NULL );

  assert ( q1->rows == 4 && q1->cols == 1 &&
	   dq3_by_dq2->rows == 4 && dq3_by_dq2->cols == 4 );

  double& q10 = vecel(q1, 1); 
  double& q1x = vecel(q1, 2); 
  double& q1y = vecel(q1, 3); 
  double& q1z = vecel(q1, 4); 

  hor_matq_fill ( dq3_by_dq2,
		  q10, -q1x, -q1y, -q1z,
		  q1x, q10, q1z, -q1y,
		  q1y, -q1z, q10, q1x,
		  q1z, q1y, -q1x, q10);

  return dq3_by_dq2;
}

Hor_Matrix *R_from_q(Hor_Matrix *q, Hor_Matrix *R)
{
  assert ( q != NULL && R != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   R->rows == 3 && R->cols == 3 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  hor_matq_fill(R,
		q0*q0 + qx*qx - qy*qy - qz*qz,
		2 * (qx*qy - q0*qz),
		2 * (qx*qz + q0*qy),
		2 * (qx*qy + q0*qz),
		q0*q0 - qx*qx + qy*qy - qz*qz,
		2 * (qy*qz - q0*qx),
		2 * (qx*qz - q0*qy),
		2 * (qy*qz + q0*qx),
		q0*q0 - qx*qx - qy*qy + qz*qz);

  return R;
}

Hor_Matrix *dR_by_dq0(Hor_Matrix *q, Hor_Matrix *dR_by_dq0)
{
  assert ( q != NULL && dR_by_dq0 != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   dR_by_dq0->rows == 3 && dR_by_dq0->cols == 3 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  hor_matq_fill(dR_by_dq0,
		2*q0, -2*qz, 2*qy,
		2*qz, 2*q0, -2*qx,
		-2*qy, 2*qx, 2*q0);

  return dR_by_dq0;
}

Hor_Matrix *dR_by_dqx(Hor_Matrix *q, Hor_Matrix *dR_by_dqx)
{
  assert ( q != NULL && dR_by_dqx != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   dR_by_dqx->rows == 3 && dR_by_dqx->cols == 3 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  hor_matq_fill(dR_by_dqx,
		2*qx, 2*qy, 2*qz,
		2*qy, -2*qx, -2*q0,
		2*qz, 2*q0, -2*qx);

  return dR_by_dqx;
}

Hor_Matrix *dR_by_dqy(Hor_Matrix *q, Hor_Matrix *dR_by_dqy)
{
  assert ( q != NULL && dR_by_dqy != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   dR_by_dqy->rows == 3 && dR_by_dqy->cols == 3 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  hor_matq_fill(dR_by_dqy,
		-2*qy, 2*qx, 2*q0,
		2*qx, 2*qy, 2*qz,
		-2*q0, 2*qz, -2*qy);

  return dR_by_dqy;
}

Hor_Matrix *dR_by_dqz(Hor_Matrix *q, Hor_Matrix *dR_by_dqz)
{
  assert ( q != NULL && dR_by_dqz != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   dR_by_dqz->rows == 3 && dR_by_dqz->cols == 3 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  hor_matq_fill(dR_by_dqz,
		-2*qz, -2*q0, 2*qx,
		2*q0, -2*qz, 2*qy,
		2*qx, 2*qy, 2*qz);

  return dR_by_dqz;
}

// Simple quaternions and Jacobians
// Rotation about x axis
Hor_Matrix *q_from_thetax(double thetax, Hor_Matrix *q)
{
  assert(q != NULL);

  assert(q->rows == 4 && q->cols == 1);

  hor_matq_fill(q,
		cos(thetax / 2.0),
		sin(thetax / 2.0),
		0.0,
		0.0);

  return q;
}

Hor_Matrix *dq_by_dthetax_from_thetax(double thetax, Hor_Matrix *dq_by_dthetax)
{
  assert(dq_by_dthetax != NULL);

  assert(dq_by_dthetax->rows == 4 && dq_by_dthetax->cols == 1);

  hor_matq_fill(dq_by_dthetax,
		-0.5 * sin(thetax / 2.0),
		0.5 * cos(thetax / 2.0),
		0.0,
		0.0);

  return dq_by_dthetax;
}


// Simple quaternions and Jacobians
// Rotation about y axis
Hor_Matrix *q_from_thetay(double thetay, Hor_Matrix *q)
{
  assert(q != NULL);

  assert(q->rows == 4 && q->cols == 1);

  hor_matq_fill(q,
		cos(thetay / 2.0),
		0.0,
		sin(thetay / 2.0),
		0.0);

  return q;
}

Hor_Matrix *dq_by_dthetay_from_thetay(double thetay, Hor_Matrix *dq_by_dthetay)
{
  assert(dq_by_dthetay != NULL);

  assert(dq_by_dthetay->rows == 4 && dq_by_dthetay->cols == 1);

  hor_matq_fill(dq_by_dthetay,
		-0.5 * sin(thetay / 2.0),
		0.0,
		0.5 * cos(thetay / 2.0),
		0.0);

  return dq_by_dthetay;
}


// Simple quaternions and Jacobians
// Rotation about z axis
Hor_Matrix *q_from_thetaz(double thetaz, Hor_Matrix *q)
{
  assert(q != NULL);

  assert(q->rows == 4 && q->cols == 1);

  hor_matq_fill(q,
		cos(thetaz / 2.0),
		0.0,
		0.0,
		sin(thetaz / 2.0));

  return q;
}

Hor_Matrix *dq_by_dthetaz_from_thetaz(double thetaz, Hor_Matrix *dq_by_dthetaz)
{
  assert(dq_by_dthetaz != NULL);

  assert(dq_by_dthetaz->rows == 4 && dq_by_dthetaz->cols == 1);

  hor_matq_fill(dq_by_dthetaz,
		-0.5 * sin(thetaz / 2.0),
		0.0,
		0.0,
		0.5 * cos(thetaz / 2.0));

  return dq_by_dthetaz;
}


// Jacobian for rotation derived from quaternion multiplied by a vector
// This tricky because really we need a tensor to do it neatly!
// But do it here without
Hor_Matrix *dRq_times_a_by_dq (Hor_Matrix *q, Hor_Matrix *a, 
			       Hor_Matrix *dRq_times_a_by_dq)
{
  assert(q != NULL && a != NULL);

  assert(q->rows == 4 && q->cols == 1 &&
	 a->rows == 3 && a->cols == 1 &&
	 dRq_times_a_by_dq->rows == 3 && dRq_times_a_by_dq->cols == 4);

  Hor_Matrix *TempR = hor_mat_alloc(3, 3);
  Hor_Matrix *Temp31 = hor_mat_alloc(3, 1);

  // Make Jacobian by stacking four vectors together side by side
  dR_by_dq0(q, TempR);
  hor_matq_prod2(TempR, a, Temp31);
  hor_matq_insert_chunkyx(Temp31, dRq_times_a_by_dq, 0, 0);

  dR_by_dqx(q, TempR);
  hor_matq_prod2(TempR, a, Temp31);
  hor_matq_insert_chunkyx(Temp31, dRq_times_a_by_dq, 0, 1);

  dR_by_dqy(q, TempR);
  hor_matq_prod2(TempR, a, Temp31);
  hor_matq_insert_chunkyx(Temp31, dRq_times_a_by_dq, 0, 2);

  dR_by_dqz(q, TempR);
  hor_matq_prod2(TempR, a, Temp31);
  hor_matq_insert_chunkyx(Temp31, dRq_times_a_by_dq, 0, 3);

  hor_mat_free_list(TempR, Temp31, NULL);

  return dRq_times_a_by_dq;
}

// Normalise quaternion: make the sum of all the elements squared equal 1
Hor_Matrix *normalise_quaternion(Hor_Matrix *q, Hor_Matrix *qnorm)
{
  assert ( q != NULL && qnorm != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   qnorm->rows == 4 && qnorm->cols == 1 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  // qq is current magnitude of q
  double qq = sqrt(q0*q0 + qx*qx + qy*qy + qz*qz);

  hor_matq_fill(qnorm,
		q0 / qq,
		qx / qq,
		qy / qq,
		qz / qq);

  return qnorm;
}

// Auxiliary functions used by dqnorm_by_dq()
// Value of diagonal element of Jacobian
double dqi_by_dqi(double qi, double qq)
{
  return (1 - qi*qi / (qq*qq)) / qq;
}

// Value of off-diagonal element of Jacobian
double dqi_by_dqj(double qi, double qj, double qq)
{
  return -qi * qj / (qq*qq*qq);
}

// Work out Jacobian for normalised quaternion
Hor_Matrix *dqnorm_by_dq(Hor_Matrix *q, Hor_Matrix *dqnorm_by_dq)
{
  assert ( q != NULL && dqnorm_by_dq != NULL );

  assert ( q->rows == 4 && q->cols == 1 &&
	   dqnorm_by_dq->rows == 4 && dqnorm_by_dq->cols == 4 );

  double& q0 = vecel(q, 1); 
  double& qx = vecel(q, 2); 
  double& qy = vecel(q, 3); 
  double& qz = vecel(q, 4); 

  double qq = q0*q0 + qx*qx + qy*qy + qz*qz;

  hor_matq_fill(dqnorm_by_dq,
		dqi_by_dqi(q0, qq), dqi_by_dqj(q0, qx, qq), 
                               dqi_by_dqj(q0, qy, qq), dqi_by_dqj(q0, qz, qq), 
		dqi_by_dqj(qx, q0, qq), dqi_by_dqi(qx, qq), 
                               dqi_by_dqj(qx, qy, qq), dqi_by_dqj(qx, qz, qq), 
		dqi_by_dqj(qy, q0, qq), dqi_by_dqj(qy, qx, qq), 
                               dqi_by_dqi(qy, qq), dqi_by_dqj(qy, qz, qq), 
		dqi_by_dqj(qz, q0, qq), dqi_by_dqj(qz, qx, qq), 
                               dqi_by_dqj(qz, qy, qq), dqi_by_dqi(qz, qq));

  return dqnorm_by_dq;
}

// Form quaternion from angle-axis rotation (like omega * delta_t)
Hor_Matrix *q_from_aa(Hor_Matrix *aa, Hor_Matrix *q)
{
  assert ( aa != NULL && q != NULL );

  assert ( aa->rows == 3 && aa->cols == 1 &&
	   q->rows == 4 && q->cols == 1 );

  // Modulus of aa vector
  double mod = sqrt(vecel(aa, 1) * vecel(aa, 1) + 
		    vecel(aa, 2) * vecel(aa, 2) + 
		    vecel(aa, 3) * vecel(aa, 3));

  if (mod != 0.0) {
    double cm2 = cos(mod / 2.0);
    double sm2 = sin(mod / 2.0);

    hor_matq_fill(q,
		  cm2,
		  (vecel(aa, 1) / mod) * sm2,
		  (vecel(aa, 2) / mod) * sm2,
		  (vecel(aa, 3) / mod) * sm2 );
  }
  else
    hor_matq_fill(q,
		  1.0, 
		  0.0, 
		  0.0, 
		  0.0);

  return 0;
}


/*****************************************************************************/

#if 0
const double INCREMENT_SIZE = 0.000001;

// Testing Jacobians
// Here we test Jacobians against a small increment approximation

// We have a(b)
int jacobian_test()
{
  // a0 is standard value of a
  // b0 is standard value of b
  // a and b are locally adjusted versions
  // adiff and bdiff are vectors of differences
  // da0_by_db0 is originally calculated Jacobian
  // da_by_db is locally calculated Jacobian
  // func_ is function that can be called with arguments b, a to get new
  //   a from b

  func_(b0, a0);

  for (int bpos = 1; col <= b0->rows; bpos++) {
    hor_matq_copy(b0, b);
    vecel(b, col) += INCREMENT_SIZE;
    hor_matq_sub(b, b0, bdiff);
    func_(b, a);
    hor_matq_sub(a, a0, adiff);
    for (int apos = 0; apos <= a0->rows; apos++) {
      matel(da_by_db, apos, bpos) = vecel(adiff, apos) / vecel(bdiff, bpos);
    }
  }

  cout << "Explicit Jacobian:" << da0_by_db0 << endl;
  cout << "Approximated Jacobian:" << da_by_db << endl;

  return 0;
}
#endif
