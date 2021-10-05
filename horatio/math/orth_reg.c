/* Copyright 1993 David Djian (ddjian@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"

/*******************
*   void @hor_oreg_line_init ( Hor_Assoc_Label label, int dim )
*
*   Initialises a recursive orthogonal regression line-fitter, either 2D
*   (dim=2) or 3D (dim=3).
********************/
void hor_oreg_line_init ( Hor_Assoc_Label label, int dim )
{
   if ( dim != 2 && dim != 3 )
      hor_error ( "illegal line dimension %d (hor_oreg_line_init)", HOR_FATAL,
		  dim );

   hor_scatter_init ( label, dim );
}

/*******************
*   void @hor_oreg_line_free   ( Hor_Assoc_Label label )
*   void @hor_oreg_line_reset  ( Hor_Assoc_Label label )
*   void @hor_oreg_line_data   ( Hor_Assoc_Label label, double     *Scrow )
*   void @hor_oreg_line_data_m ( Hor_Assoc_Label label, Hor_Matrix *Scmat )
*   void @hor_oreg_line_data_v ( Hor_Assoc_Label label, ... )
*
*   Line-fitting orthogonal regression functions, implemented as macro calls
*   to the equivalent hor_scatter_... functions.
********************/

/*******************
*   Hor_Bool @hor_oreg_line_solve ( Hor_Assoc_Label label, 
*                                  float sigma,          (noise estimate)
*                                  float conf_level,     (for chi^2 test)
*                                  int   no_init_points, (minimum points)
*                                  double *v_direct, double *centroid )
*
*   Solves a recursive orthogonal regression line-fitter, filling in the
*   line parameters into v_direct and centroid.
********************/
int hor_oreg_line_solve ( Hor_Assoc_Label label, 
			  float sigma, float conf_level, 
			  int no_init_points, 
			  double *v_direct, double *centroid )
{
   Hor_Matrix *eigenvectors;
   double *eigenvalues, residual, confidence;
   int no_points, dof, dim, i;

   no_points = hor_scatter_get_n ( label );
   if ( no_points < no_init_points )
     /* set errno; don't do anything at the level above */
      return 2;

   dim = hor_scatter_dim ( label );
   eigenvectors = hor_mat_alloc ( dim, dim );
   eigenvalues = hor_malloc_ntype ( double, dim );
   hor_scatter_solve ( label, eigenvectors, eigenvalues, centroid );

   for ( i = 0; i < dim; i++ )
      v_direct [i] = eigenvectors->m[i][0];
   hor_mat_free ( eigenvectors );
   residual = 0.;
   for ( i = 1; i < dim; i++ )
     residual += eigenvalues [i];
   hor_free ( (void *) eigenvalues );

   residual *= ( no_points / (sigma*sigma) );
   dof = 2 * ( no_points - 2 );
   confidence = hor_chi_2_prob ( fabs(residual), dof );

   if ( confidence < conf_level ) return HOR_TRUE;
   else                           return HOR_FALSE;
}
