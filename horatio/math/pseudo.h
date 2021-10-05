/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/pseudo.h */

#ifdef _HORATIO_LIST_

void hor_pseudo_inv_init ( Hor_Assoc_Label label, int Xrows, int Xcols );
void hor_pseudo_inv_free ( Hor_Assoc_Label label );
void hor_pseudo_inv_reset ( Hor_Assoc_Label label );
void hor_pseudo_inv_data ( Hor_Assoc_Label label, double *Arow, double *Brow );
void hor_pseudo_inv_data_m ( Hor_Assoc_Label label,
			     Hor_Matrix *Amat, Hor_Matrix *Bmat );
void hor_pseudo_inv_data_v ( Hor_Assoc_Label label, ... );
Hor_Matrix *hor_pseudo_inv_solve ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
