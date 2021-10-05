/* Copyright 1994 Philip McLauchlan (pm@robots.oxford.ac.uk) and
                  David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/orth_reg.h */

#ifdef _HORATIO_LIST_

void hor_oreg_line_init ( Hor_Assoc_Label label, int dim );

#define hor_oreg_line_free(label) (hor_scatter_free(label))
#define hor_oreg_line_reset(label) (hor_scatter_reset(label))
#define hor_oreg_line_data(label,Scrow) (hor_scatter_data((label),(Scrow)))
#define hor_oreg_line_data_m(label,Scmat) (hor_scatter_data_m((label),(Scmat)))
#define hor_oreg_line_data_v hor_scatter_data_v

int hor_oreg_line_solve ( Hor_Assoc_Label label, 
			  float sigma, float conf_level,
			  int no_init_points,
			  double *v_direct, double *centroid );

#endif /* _HORATIO_LIST_ */
