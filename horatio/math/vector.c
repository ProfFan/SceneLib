/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/math.h"

/*******************
*   double @hor_vec_scalar_prod ( Hor_Matrix *x, Hor_Matrix *y )
*   double @hor_vec_max_coord   ( Hor_Matrix *x )
*
*   Miscellaneous vector functions using the Horatio matrix/vector type
*   Hor_Matrix.
*
*   hor_vec_scalar_prod() returns the scalar product of x and y.
*   hor_vec_max_coord() returns the largest absolute coordinate value of x.
********************/
double hor_vec_scalar_prod ( Hor_Matrix *x, Hor_Matrix *y )
{
   double total = 0.0, **xm = x->m, **ym = y->m;
   int    i, rows = x->rows;

   if ( x->cols != 1 || !hor_mat_same_size2 ( x, y ) )
      hor_error ("illegal vector dimensions %d %d %d %d (hor_vec_scalar_prod)",
		 HOR_FATAL, x->rows, x->cols, y->rows, y->cols );

   for ( i = 0; i < rows; i++ )
      total += xm[i][0]*ym[i][0];

   return total;
}

double hor_vec_max_coord ( Hor_Matrix *x )
{
   double max_abs = 0.0, **xm = x->m;
   int    i, rows = x->rows;

   if ( x->cols != 1 )
      hor_error ( "illegal vector dimensions %d %d (hor_vec_max_coord)",
		  HOR_FATAL, x->rows, x->cols );

   for ( i = 0; i < rows; i++ )
      if ( fabs(xm[i][0]) > max_abs ) max_abs = fabs(xm[i][0]);

   return max_abs;
}

static void vec_cross_prod ( double **xm, double **ym, double **zm )
{
   zm[0][0] = xm[1][0]*ym[2][0] - xm[2][0]*ym[1][0];
   zm[1][0] = xm[2][0]*ym[0][0] - xm[0][0]*ym[2][0];
   zm[2][0] = xm[0][0]*ym[1][0] - xm[1][0]*ym[0][0];
}

/*******************
*   Hor_Matrix *@hor_vecs_cross_prod ( Hor_Matrix *x, Hor_Matrix *y )
*   Hor_Matrix *@hor_vecs_unit       ( Hor_Matrix *x )
*
*   "Slow" vector routines.
*
*   hor_vecs_cross_prod() performs the vector cross product on the two input
*                         vectors x and y, allocating and returning the result.
*
*   hor_vecs_unit() creates and returns a unit vector in the direction of the
*                   given vector.
********************/
Hor_Matrix *hor_vecs_cross_prod ( Hor_Matrix *x, Hor_Matrix *y )
{
   Hor_Matrix *z;

   if ( x->cols != 1 || x->rows != 3 || !hor_mat_same_size2 ( x, y ) )
      hor_error ( "illegal vector size(s) %d %d %d %d (hor_vecs_cross_prod)",
		  HOR_FATAL, x->rows, x->cols, y->rows, y->cols );

   z = hor_mat_alloc ( x->rows, 1 );
   vec_cross_prod ( x->m, y->m, z->m );
   return z;
}

Hor_Matrix *hor_vecs_unit ( Hor_Matrix *x )
{
   Hor_Matrix  *y;
   double **ym, factor;
   int      i, rows = x->rows;

   if ( x->cols != 1 )
      hor_error ( "illegal vector dimensions %d %d (vec_unit)", HOR_FATAL,
		  x->rows, x->cols );

   factor = sqrt ( hor_vec_scalar_prod ( x, x ) );
   if ( factor == 0.0 )
      hor_error ( "zero vector provided (vec_unit)", HOR_FATAL );

   y = hor_mats_copy ( x );
   ym = y->m;
   for ( i = 0; i < rows; i++ )
      ym[i][0] /= factor;

   return y;
}

/*******************
*   Hor_Matrix *@hor_vecq_cross_prod ( Hor_Matrix *x, Hor_Matrix *y,
*                                     Hor_Matrix *z )
*   Hor_Matrix *@hor_vecq_unit       ( Hor_Matrix *x )
*
*   "Quick" vector routines.
*
*   hor_vecq_cross_prod() performs the vector cross product on the two input
*                         vectors x and y, writing the result into the third z.
*   hor_vecq_unit() scales x to be a unit vector.
********************/
Hor_Matrix *hor_vecq_cross_prod ( Hor_Matrix *x, Hor_Matrix *y, Hor_Matrix *z )
{
   if ( x->cols != 1 || x->rows != 3 || !hor_mat_same_size3 ( x, y, z ) )
      hor_error("illegal vector sizes %d %d %d %d %d %d (hor_vecq_cross_prod)",
		HOR_FATAL,
		x->rows, x->cols, y->rows, y->cols, z->rows, z->cols );

   vec_cross_prod ( x->m, y->m, z->m );
   return z;
}

Hor_Matrix *hor_vecq_unit ( Hor_Matrix *x )
{
   double **xm = x->m, factor;
   int      i, rows = x->rows;

   if ( x->cols != 1 )
      hor_error ( "illegal vector dimensions %d %d (vec_unit)", HOR_FATAL,
		  x->rows, x->cols );

   factor = sqrt ( hor_vec_scalar_prod ( x, x ) );
   if ( factor == 0.0 )
      hor_error ( "zero vector provided (vec_unit)", HOR_FATAL );

   for ( i = 0; i < rows; i++ ) xm[i][0] /= factor;

   return x;
}
