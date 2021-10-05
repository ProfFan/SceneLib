/*  Scene: software for sequential localisation and map-building

    bits.h
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

// bits.cc: assorted stand-alone maths functions augmenting Horatio

#ifndef _bits_h_
#define _bits_h_
// Constant representing the size of a matrix singular value at which
// it can be said to be zero (i.e. matrix is singular)
const double BITS_ZERO_SINGULAR_VALUE = 1.0e-15;

void and_pi_range(double &angle);

/* Macros: matel(M, i, j) gives the i, jth element of matrix M */
/*         vecel(V, i)    gives the i, 1st element of matrix (vector) V */
#define matel(M,i,j) ((M)->m[i-1][j-1])
#define vecel(V,i)   ((V)->m[i-1][0])


// For printing which error occurred based on horatio error code.
// (Slightly modified form of hor_perror())
int hor_print_error ();

// Various fast combined matrix multiplications
double hor_matq_dot31(Hor_Matrix *A, Hor_Matrix *B);
Hor_Matrix *hor_matq_ABAT3 (Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C);
double hor_matq_ATMB3 (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B);
double hor_matq_AMBT3 (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B);
Hor_Matrix *hor_matq_ATMB (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			   Hor_Matrix *W, Hor_Matrix *C);
Hor_Matrix *hor_matq_AMBT (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			   Hor_Matrix *W, Hor_Matrix *C);
Hor_Matrix *hor_matq_AMTBT (Hor_Matrix *A, Hor_Matrix *M, Hor_Matrix *B,
			    Hor_Matrix *W, Hor_Matrix *C);
Hor_Matrix *hor_matq_ABT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C);
Hor_Matrix *hor_matq_ATB (Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C);
Hor_Matrix *hor_matq_ABAT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C);
Hor_Matrix *hor_matq_ABAT ( Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *W, Hor_Matrix *C);
Hor_Matrix *hor_matq_AAT3 (Hor_Matrix *A, Hor_Matrix *B);

// C++ style output for Hor_Matrix
ostream& operator<<(ostream& os, Hor_Matrix *M);

// Inserting and extracting small matrices to and from large ones
// ...insert/extract_chunky1 are for vectors: 1 index defines where in
// the big vector to insert/extract the WHOLE small one
// ...insert/extract_chunkyx are for matrices with two indices to define
// where the WHOLE small matrix goes in the big one
// Indices for these functions start at zero --- i.e. r, c means
// insert starting at row and column r + 1, c + 1
// Special (faster?) versions for 3 vectors and 3*3 matrices
Hor_Matrix *hor_matq_insert_chunk33(Hor_Matrix *A, Hor_Matrix *B, 
                                    int r, int c);
Hor_Matrix *hor_matq_insert_chunk31(Hor_Matrix *A, Hor_Matrix *B, int r);
Hor_Matrix *hor_matq_extract_chunk33(Hor_Matrix *A, Hor_Matrix *B, 
                                     int r, int c);
Hor_Matrix *hor_matq_extract_chunk31(Hor_Matrix *A, Hor_Matrix *B, int r);

Hor_Matrix *hor_matq_insert_chunkyx(Hor_Matrix *A, Hor_Matrix *B, 
                                    int r, int c);
Hor_Matrix *hor_matq_insert_chunky1(Hor_Matrix *A, Hor_Matrix *B, int r);
Hor_Matrix *hor_matq_extract_chunkyx(Hor_Matrix *A, Hor_Matrix *B, 
                                     int r, int c);
Hor_Matrix *hor_matq_insert_chunkyxT(Hor_Matrix *A, Hor_Matrix *B, 
				     int r, int c);
Hor_Matrix *hor_matq_extract_chunky1(Hor_Matrix *A, Hor_Matrix *B, int r);


Hor_Matrix *hor_matq_extract_row(Hor_Matrix *A, Hor_Matrix *R, int r);


// Assorted
// Closest approach of 2 lines in 3D
Hor_Matrix *hor_matq_vector_intersect(Hor_Matrix *a, Hor_Matrix *b, 
				       Hor_Matrix *c, Hor_Matrix *d,
				       Hor_Matrix *r);

// Calculate the closest intersection of 2 lines but with Jacobians too
Hor_Matrix *hor_matq_vector_intersect_with_jacobians(
       Hor_Matrix *a, Hor_Matrix *b, Hor_Matrix *c, Hor_Matrix *d,
       Hor_Matrix *r,
       Hor_Matrix *dr_by_da, Hor_Matrix *dr_by_db, Hor_Matrix *dr_by_dc, 
       Hor_Matrix *dr_by_dd);

Hor_Matrix *hor_mat_ensure_size (int rows, int cols, Hor_Matrix **M);
int hor_mat_ensure_size_list (int rows, int cols ...);
Hor_Matrix *hor_matq_sub4 (Hor_Matrix *M, Hor_Matrix *A, Hor_Matrix *B,
			   Hor_Matrix *C, Hor_Matrix *D);
Hor_Matrix *hor_matq_add4 (Hor_Matrix *M, Hor_Matrix *A, Hor_Matrix *B,
			   Hor_Matrix *C, Hor_Matrix *D);
Hor_Matrix *hor_mat_symmetrise (Hor_Matrix *M);
Hor_Matrix *hor_matq_inv_any (Hor_Matrix *M, Hor_Matrix *U, Hor_Matrix *W,
			      Hor_Matrix *V, Hor_Matrix *Ut, Hor_Matrix *R);
Hor_Matrix *hor_matq_psinv (Hor_Matrix *M, Hor_Matrix *U, Hor_Matrix *W,
			    Hor_Matrix *V, Hor_Matrix *Ut, Hor_Matrix *R);
int hor_mat_test_for_symmetry (const char *message, Hor_Matrix *M);
double hor_mat_root_sum_square (Hor_Matrix *M);
double hor_hinfnorm (Hor_Matrix *M);
double hor_uncertainty_volume_3s (Hor_Matrix *M);
double hor_uncertainty_volume (Hor_Matrix *M, double n);
double hor_frobenius_norm (Hor_Matrix *M);
double hor_trace (Hor_Matrix *M);
Hor_Matrix *hor_matq_set_all (Hor_Matrix *M, double val);

/* Quaternion stuff */

// Invert quaternion
Hor_Matrix *invert_quaternion(Hor_Matrix *q, Hor_Matrix *qbar);
// Jacobian for inversion (is actually constant matrix)
Hor_Matrix *dqbar_by_dq(Hor_Matrix *dqbar_by_dq);

// Quaternion product
Hor_Matrix *prodq2q1(Hor_Matrix *q2, Hor_Matrix *q1, Hor_Matrix *q3);
// Jacobians of quaternion product
Hor_Matrix *dq3_by_dq1(Hor_Matrix *q2, Hor_Matrix *dq3_by_dq1);
Hor_Matrix *dq3_by_dq2(Hor_Matrix *q1, Hor_Matrix *dq3_by_dq2);

// Rotation matrix derived from quaternion
Hor_Matrix *R_from_q(Hor_Matrix *q, Hor_Matrix *R);
// Jacobians for rotation matrix
Hor_Matrix *dR_by_dq0(Hor_Matrix *q, Hor_Matrix *dR_by_dq0);
Hor_Matrix *dR_by_dqx(Hor_Matrix *q, Hor_Matrix *dR_by_dqx);
Hor_Matrix *dR_by_dqy(Hor_Matrix *q, Hor_Matrix *dR_by_dqy);
Hor_Matrix *dR_by_dqz(Hor_Matrix *q, Hor_Matrix *dR_by_dqz);

// Simple quaternions and Jacobians
// Rotation about x axis, y axis and z axis
Hor_Matrix *q_from_thetax(double thetax, Hor_Matrix *q);
Hor_Matrix *dq_by_dthetax_from_thetax(double thetax, 
				      Hor_Matrix *dq_by_dthetax);
Hor_Matrix *q_from_thetay(double thetay, Hor_Matrix *q);
Hor_Matrix *dq_by_dthetay_from_thetay(double thetay, 
				      Hor_Matrix *dq_by_dthetay);
Hor_Matrix *q_from_thetaz(double thetaz, Hor_Matrix *q);
Hor_Matrix *dq_by_dthetaz_from_thetaz(double thetaz, 
				      Hor_Matrix *dq_by_dthetaz);

// Jacobian for rotation derived from quaternion multiplied by a vector
// This tricky because really we need a tensor to do it neatly I think!
// But do it here without
Hor_Matrix *dRq_times_a_by_dq (Hor_Matrix *q, Hor_Matrix *a, 
			       Hor_Matrix *dRq_times_a_by_dq);


// Normalising a quaternion and Jacobian
Hor_Matrix *normalise_quaternion(Hor_Matrix *q, Hor_Matrix *qnorm);
Hor_Matrix *dqnorm_by_dq(Hor_Matrix *q, Hor_Matrix *dqnorm_by_dq);

// Form quaternion from angle-axis rotation (like omega * delta_t)
Hor_Matrix *q_from_aa(Hor_Matrix *aa, Hor_Matrix *q);

#endif
