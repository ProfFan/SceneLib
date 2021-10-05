/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/vector.h */

double      hor_vec_scalar_prod ( Hor_Matrix *x, Hor_Matrix *y );
double      hor_vec_max_coord   ( Hor_Matrix *x );
Hor_Matrix *hor_vecs_cross_prod ( Hor_Matrix *x, Hor_Matrix *y );
Hor_Matrix *hor_vecs_unit       ( Hor_Matrix *x );
Hor_Matrix *hor_vecq_cross_prod ( Hor_Matrix *x, Hor_Matrix *y, Hor_Matrix *z);
Hor_Matrix *hor_vecq_unit       ( Hor_Matrix *x );
