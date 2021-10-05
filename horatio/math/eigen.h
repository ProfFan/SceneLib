/* Copyright 1994 Philip McLauchlan (pm@robots.oxford.ac.uk) and
                  David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/eigen.h */

#ifdef _HORATIO_LIST_

void hor_eigen_init ( Hor_Assoc_Label label, int Awidth );
void hor_eigen_free ( Hor_Assoc_Label label );
void hor_eigen_reset ( Hor_Assoc_Label label );
void hor_eigen_data ( Hor_Assoc_Label label, double *Arow );
void hor_eigen_data_m ( Hor_Assoc_Label label, Hor_Matrix *Amat );
void hor_eigen_data_v ( Hor_Assoc_Label label, ... );
void hor_eigen_solve ( Hor_Assoc_Label label,
		       Hor_Matrix *eigenvectors, double *eigenvalues );
Hor_Matrix *hor_eigen_ATA ( Hor_Assoc_Label label );

void hor_scatter_init ( Hor_Assoc_Label label, int Scwidth );
void hor_scatter_free ( Hor_Assoc_Label label );
void hor_scatter_reset ( Hor_Assoc_Label label );
void hor_scatter_data ( Hor_Assoc_Label label, double *Scrow );
void hor_scatter_data_m ( Hor_Assoc_Label label, Hor_Matrix *Scmat );
void hor_scatter_data_v ( Hor_Assoc_Label label, ... );
void hor_scatter_solve ( Hor_Assoc_Label label,
		       Hor_Matrix *eigenvectors, 
			double *eigenvalues, 
			double *centroid );
int hor_scatter_dim ( Hor_Assoc_Label label );
int hor_scatter_get_n ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
