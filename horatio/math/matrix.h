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
