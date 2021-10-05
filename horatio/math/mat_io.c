/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#ifdef HOR_TRANSPUTER
#include <iocntrl.h>
#include <channel.h>
#include <misc.h>
#else
#include <fcntl.h>
#ifdef HOR_MSDOS
#include <io.h>
#else
#include <unistd.h>
#endif
#endif

#include "horatio/global.h"
#include "horatio/math.h"

/* matrix channel I/O functions */

#ifdef HOR_TRANSPUTER

/*******************
*   Hor_Matrix *@HorChanInMatrix          ( Channel *c )
*   void        @HorChanInAllocatedMatrix ( Channel *c, Hor_Matrix *M )
*   void        @HorChanOutMatrix         ( Channel *c, Hor_Matrix *M )
*
*   Matrix channel I/O functions for transputers.
*
*   ChanInHor_Matrix() reads a matrix from an input channel and returns it.
*   ChanInAllocatedHor_Matrix() reads a previously allocated matrix from an
*                               input channel.
*   ChanOutHor_Matrix() outputs a matrix on a channel.
********************/
Hor_Matrix *HorChanInMatrix ( Channel *c )
{
   Hor_Matrix A, *B;

   ChanIn ( c, &A, sizeof(Hor_Matrix) );
   B = hor_mat_alloc ( A.rsize, A.csize );
   B->rows = A.rows;
   B->cols = A.cols;
   ChanIn ( c, B->m[0], A.rsize*A.csize*sizeof(double) );
   return B;
}

void HorChanInAllocatedMatrix ( Channel *c, Hor_Matrix *M )
{
   ChanIn ( c, M, sizeof(Hor_Matrix) );
   ChanIn ( c, M->m[0], M->rsize*M->csize*sizeof(double) );
}

void HorChanOutMatrix ( Channel *c, Hor_Matrix *M )
{
   ChanOut ( c, M, sizeof(Hor_Matrix) );
   ChanOut ( c, M->m[0], M->rsize*M->csize*sizeof(double) );
}

#endif

#ifndef HOR_REDUCED_LIB

/*******************
*   Hor_Matrix *@hor_read_matrix           ( int fd )
*   void        @hor_read_allocated_matrix ( int fd, Hor_Matrix *M )
*   void        @hor_write_matrix          ( int fd, Hor_Matrix *M )
*
*   Matrix stream I/O functions.
********************/
Hor_Matrix *hor_read_matrix ( int fd )
{
   Hor_Matrix A, *B;

   hor_pipe_read ( fd, (char *) &A, sizeof(Hor_Matrix) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) &A, sizeof(Hor_Matrix) );
#endif
   B = hor_mat_alloc ( A.rsize, A.csize );
   B->rows = A.rows;
   B->cols = A.cols;
   hor_pipe_read ( fd, (char *) B->m[0], A.rsize*A.csize*sizeof(double) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order8 ((char *) B->m[0], A.rsize*A.csize*sizeof(double));
#endif
   return B;
}

void hor_read_allocated_matrix ( int fd, Hor_Matrix *M )
{
   hor_pipe_read ( fd, (char *) M, sizeof(Hor_Matrix) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) M, sizeof(Hor_Matrix) );
#endif
   hor_pipe_read ( fd, (char *) M->m[0], M->rsize*M->csize*sizeof(double) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order8((char *) M->m[0], M->rsize*M->csize*sizeof(double));
#endif
}

void hor_write_matrix ( int fd, Hor_Matrix *M )
{
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4 ( (char *) M, sizeof(Hor_Matrix) );
#endif
   write ( fd, (char *) M, sizeof(Hor_Matrix) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order4((char *) M, sizeof(Hor_Matrix) );
   hor_reverse_byte_order8((char *) M->m[0], M->rsize*M->csize*sizeof(double));
#endif
   write ( fd, (char *) M->m[0], M->rsize*M->csize*sizeof(double) );
#ifndef HOR_TRANSPUTER
   hor_reverse_byte_order8((char *) M->m[0], M->rsize*M->csize*sizeof(double));
#endif
}

#endif /* HOR_REDUCED_LIB */
