#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#include "fmatx.h"

void hor_fm_compute_epipoles (double f_matrix [3][3], HOR_FM_TRIVEC_DOUBLE *epipole1_trivec_ptr, HOR_FM_TRIVEC_DOUBLE *epipole2_trivec_ptr) /*
-------------------------------------------------------------------------------------------------------------------------------------------
use F to find the epipole e using Fe = 0.
*/

{
#define HOR_FM_UNKNOWNS_COUNT 3

    double
	nr_w_vector [3];

    int
	col_index,
	row_count,
	row_index;

    Hor_Matrix
	*nr_a_matrix,
	*nr_v_matrix;

    row_count = 3;

    /* allocate the numerical recipes SVD matrices.
     */

    nr_a_matrix = hor_mat_alloc (row_count, HOR_FM_UNKNOWNS_COUNT);
    nr_v_matrix = hor_mat_alloc (HOR_FM_UNKNOWNS_COUNT, HOR_FM_UNKNOWNS_COUNT);

    /*	get epipole in image 1.
     */

    for (row_index = 0; row_index < 3; row_index++)
    {
	for (col_index = 0; col_index < 3; col_index++)
	{
	    nr_a_matrix-> m [row_index][col_index] = f_matrix [row_index][col_index];
	}
    }

    hor_matq_svdcmp (nr_a_matrix, nr_w_vector, nr_v_matrix);
    hor_matq_eigsrt (nr_w_vector, nr_v_matrix);
 
    epipole1_trivec_ptr-> x = nr_v_matrix-> m [0][2];
    epipole1_trivec_ptr-> y = nr_v_matrix-> m [1][2];
    epipole1_trivec_ptr-> z = nr_v_matrix-> m [2][2];

    /*	use transpose of F matrix i.e. image 2 to image 1 transformation, in order to get epipole in image 2.
     */

    for (row_index = 0; row_index < 3; row_index++)
    {
	for (col_index = 0; col_index < 3; col_index++)
	{
	    nr_a_matrix-> m [row_index][col_index] = f_matrix [col_index][row_index];
	}
    }

    hor_matq_svdcmp (nr_a_matrix, nr_w_vector, nr_v_matrix);
    hor_matq_eigsrt (nr_w_vector, nr_v_matrix);

    epipole2_trivec_ptr-> x = nr_v_matrix-> m [0][2];
    epipole2_trivec_ptr-> y = nr_v_matrix-> m [1][2];
    epipole2_trivec_ptr-> z = nr_v_matrix-> m [2][2];

    hor_mat_free (nr_a_matrix);
    hor_mat_free (nr_v_matrix);

#undef HOR_FM_UNKNOWNS_COUNT
}

HOR_FM_BOOLEAN hor_fm_compute_fmatrix_linear (HOR_FM_FMATRIX_PARAMS_STR *fmatrix_params_ptr, HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, 
					      double f_matrix [3][3]) /*
---------------------------------------------------------------------
compute the fundamental matrix.
*/

{
#define HOR_FM_ITERATION_COUNT 5
#define HOR_FM_UNKNOWNS_COUNT 9

    double
	*a_matrix,
	*ata_matrix,
	inverse_matrix [3][3],
	*nr_w_vector;

    float
	debug1,
	debug2,
	distance,
	distance_squared,
	distance1_squared,
	distance2_squared,
	epipole_distance_squared,
	error,
	outlier_distance_squared,
	scale_factor,
	weighting;

    int
	col_index,
	corner_index,
	iteration_index,
	match_count,
	outlier_count,
	row_count,
	row_index,
	scale_index;

    HOR_FM_CORNER_STR
	*corner1_ptr,
	*corner2_ptr;

    HOR_FM_TRIVEC_DOUBLE
	epipole1_trivec,
	epipole2_trivec,
	epipolar_line_trivec;

    Hor_Matrix
	*nr_a_matrix,
	*nr_v_matrix;

    epipole_distance_squared = HOR_FM_SQUARE (fmatrix_params_ptr-> epipole_distance_threshold);
    outlier_distance_squared = HOR_FM_SQUARE (fmatrix_params_ptr-> epipolar_outlier_threshold);

    /* how many equations are there in the system?
     */

    row_count = 0;
    for (corner_index = 0; corner_index < corner_table1_ptr-> count; corner_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner_index];

	if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	    row_count++;
    }

    if (row_count < HOR_FM_UNKNOWNS_COUNT)
	return FALSE;

    /* allocate the numerical recipes SVD matrices.
     */

    nr_a_matrix = hor_mat_alloc (HOR_FM_UNKNOWNS_COUNT, HOR_FM_UNKNOWNS_COUNT);
    nr_v_matrix = hor_mat_alloc (HOR_FM_UNKNOWNS_COUNT, HOR_FM_UNKNOWNS_COUNT);
    nr_w_vector = malloc (HOR_FM_UNKNOWNS_COUNT * sizeof (double));

    /* linear system will be set up in a_matrix.  this will then be premultiplied by its transpose to give ata_matrix.  it is ata_matrix which is moved
       into nr_a_matrix for solution by svd.
       */
    
    a_matrix = (double *) malloc (row_count * HOR_FM_UNKNOWNS_COUNT * sizeof (double));
    ata_matrix = (double *) malloc (HOR_FM_UNKNOWNS_COUNT * HOR_FM_UNKNOWNS_COUNT * sizeof (double));
    if (a_matrix == NULL || ata_matrix == NULL)
	hor_error ("hor_fm_compute_fmatrix_linear:  memory failure", HOR_FATAL);

    /*	phase 1 - straight least-squares using all the matches.
     */

    row_index = 0;
    for (corner_index = 0; corner_index < corner_table1_ptr-> count; corner_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner_index];

	if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	{
	    corner2_ptr = &corner_table2_ptr-> array [corner1_ptr-> match_fore_index];

	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 0, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. x;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 1, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. x;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 2, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. x;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 3, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. y;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 4, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. y;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 5, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. y;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 6, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. z;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 7, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. z;
	    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 8, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. z;
	    row_index++;
	}
    }

    /* premultiply a_matrix by its transpose, then find the smallest eigenvector of the result to get the F matrix.
     */

    hor_fm_matx_premultiply_by_transpose (row_index, HOR_FM_UNKNOWNS_COUNT, a_matrix, ata_matrix);
    for (row_index = 0; row_index < HOR_FM_UNKNOWNS_COUNT; row_index++)
    {
	for (col_index = 0; col_index < HOR_FM_UNKNOWNS_COUNT; col_index++)
	{
	    nr_a_matrix-> m [row_index][col_index] = ata_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, col_index, HOR_FM_UNKNOWNS_COUNT)];
	}
    }
    hor_matq_svdcmp (nr_a_matrix, nr_w_vector, nr_v_matrix);
    hor_matq_eigsrt (nr_w_vector, nr_v_matrix);

    f_matrix [0][0] = nr_v_matrix-> m [0][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [0][1] = nr_v_matrix-> m [1][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [0][2] = nr_v_matrix-> m [2][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [1][0] = nr_v_matrix-> m [3][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [1][1] = nr_v_matrix-> m [4][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [1][2] = nr_v_matrix-> m [5][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [2][0] = nr_v_matrix-> m [6][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [2][1] = nr_v_matrix-> m [7][HOR_FM_UNKNOWNS_COUNT - 1];
    f_matrix [2][2] = nr_v_matrix-> m [8][HOR_FM_UNKNOWNS_COUNT - 1];

    hor_fm_matx_transpose (3, 3, (double *) f_matrix, (double *) inverse_matrix);

    /*	phase 2 - iterative computation similar to the one above but now 
	(a) discarding corners close to the epipoles, which can adversely affect the result,
	(b) weighting according to match/epipolar line distance, to reduce the contribution of outliers to the result.
     */

    for (iteration_index = 0; iteration_index < HOR_FM_ITERATION_COUNT; iteration_index++)
    {
	hor_fm_compute_epipoles (f_matrix, &epipole1_trivec, &epipole2_trivec);

	debug1 = 0;
	debug2 = 0;
	row_index = 0;
	for (corner_index = 0; corner_index < corner_table1_ptr-> count; corner_index++)
	{
	    corner1_ptr = &corner_table1_ptr-> array [corner_index];
	    if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	    {
		corner2_ptr = &corner_table2_ptr-> array [corner1_ptr-> match_fore_index];

		distance_squared = hor_fm_distance_squared_between_points (&epipole1_trivec, &corner1_ptr-> trivec);
		
		if (distance_squared > epipole_distance_squared)
		{
		    distance_squared = hor_fm_distance_squared_between_points (&epipole2_trivec, &corner2_ptr-> trivec);

		    if (distance_squared > epipole_distance_squared)
		    {
			hor_fm_matx_multiply (3, 3, (double *) f_matrix, 3, 1, (double *) &corner1_ptr-> trivec, (double *) &epipolar_line_trivec);
			distance1_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner2_ptr-> trivec);

			if (distance1_squared <= outlier_distance_squared)
			{
			    hor_fm_matx_multiply (3, 3, (double *) inverse_matrix, 3, 1, (double *) &corner2_ptr-> trivec, (double *) &epipolar_line_trivec);
			    distance2_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner1_ptr-> trivec);

			    if (distance2_squared <= outlier_distance_squared)
			    {
				debug1 += distance1_squared;
				debug2 += distance2_squared;

				distance = sqrt ((distance1_squared + distance2_squared) / 2.0);

				if (distance < fmatrix_params_ptr-> epipolar_weighting_min)
				    distance = fmatrix_params_ptr-> epipolar_weighting_min;

				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 0, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. x;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 1, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. x;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 2, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. x;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 3, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. y;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 4, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. y;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 5, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. y;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 6, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. x * corner2_ptr-> trivec. z;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 7, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. y * corner2_ptr-> trivec. z;
				a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, 8, HOR_FM_UNKNOWNS_COUNT)] = corner1_ptr-> trivec. z * corner2_ptr-> trivec. z;

				/* find the normalisation factor for this equation (so that sum of squared coefficients is 1).
				 */

				scale_factor = 0;
				for (scale_index = 0; scale_index < HOR_FM_UNKNOWNS_COUNT; scale_index++)
				    scale_factor += HOR_FM_SQUARE (a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, scale_index, HOR_FM_UNKNOWNS_COUNT)]);

				/* combine the normalisation and the match-epipolar distance to get the required weighting for the equation.
				 */

				weighting = distance * sqrt (scale_factor);
				for (scale_index = 0; scale_index < HOR_FM_UNKNOWNS_COUNT; scale_index++)
				    a_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, scale_index, HOR_FM_UNKNOWNS_COUNT)] /= weighting;
				
				row_index++;
			    }
			}
		    }
		}
	    }
	}

	/* force this to be the last iteration if errors are very low.
	 */

	error = sqrt ((debug1 + debug2) / ((double) row_index - 1.0));
	if (error < 0.4)
	    iteration_index = HOR_FM_ITERATION_COUNT - 1;

	/* terminate if insufficient matches have been accepted for the system.
	 */

	if (row_index < HOR_FM_UNKNOWNS_COUNT)
	{
	    hor_mat_free (nr_a_matrix);
	    hor_mat_free (nr_v_matrix);
	    free (nr_w_vector);
	    free (a_matrix);
	    free (ata_matrix);
	    return FALSE;
	}

	/* solve the linear system as previously.
	 */

	hor_fm_matx_premultiply_by_transpose (row_index, HOR_FM_UNKNOWNS_COUNT, a_matrix, ata_matrix);
	for (row_index = 0; row_index < HOR_FM_UNKNOWNS_COUNT; row_index++)
	{
	    for (col_index = 0; col_index < HOR_FM_UNKNOWNS_COUNT; col_index++)
	    {
		nr_a_matrix-> m [row_index][col_index] = ata_matrix [HOR_FM_MATX_INDEX_2DTO1D (row_index, col_index, HOR_FM_UNKNOWNS_COUNT)];
	    }
	}
	hor_matq_svdcmp (nr_a_matrix, nr_w_vector, nr_v_matrix);
	hor_matq_eigsrt (nr_w_vector, nr_v_matrix);

	f_matrix [0][0] = nr_v_matrix-> m [0][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [0][1] = nr_v_matrix-> m [1][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [0][2] = nr_v_matrix-> m [2][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [1][0] = nr_v_matrix-> m [3][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [1][1] = nr_v_matrix-> m [4][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [1][2] = nr_v_matrix-> m [5][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [2][0] = nr_v_matrix-> m [6][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [2][1] = nr_v_matrix-> m [7][HOR_FM_UNKNOWNS_COUNT - 1];
	f_matrix [2][2] = nr_v_matrix-> m [8][HOR_FM_UNKNOWNS_COUNT - 1];
	hor_fm_matx_transpose (3, 3, (double *) f_matrix, (double *) inverse_matrix);
    }

    /*	phase 3 - count up outliers and then decide what to do...
     */
    
    outlier_count = 0;
    match_count = 0;
    for (corner_index = 0; corner_index < corner_table1_ptr-> count; corner_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner_index];

	if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	{
	    corner2_ptr = &corner_table2_ptr-> array [corner1_ptr-> match_fore_index];

	    hor_fm_matx_multiply (3, 3, (double *) f_matrix, 3, 1, (double *) &corner1_ptr-> trivec, (double *) &epipolar_line_trivec);
	    distance_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner2_ptr-> trivec);
	    if (distance_squared > outlier_distance_squared)
		outlier_count++;

	    else
	    {
		hor_fm_matx_multiply (3, 3, (double *) inverse_matrix, 3, 1, (double *) &corner2_ptr-> trivec, (double *) &epipolar_line_trivec);
		distance_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner1_ptr-> trivec);
		if (distance_squared > outlier_distance_squared)
		    outlier_count++;
	    }

	    match_count++;
	}
    }

    /* if (a) too many outliers, or (b) insufficient matches remain, then assume the whole computation is a failure.
     */
    
    if ((float) outlier_count / (float) match_count > fmatrix_params_ptr-> eliminate_ratio_max
	|| match_count - outlier_count < HOR_FM_UNKNOWNS_COUNT)
    {
	hor_mat_free (nr_a_matrix);
	hor_mat_free (nr_v_matrix);
	free (nr_w_vector);
	free (a_matrix);
	free (ata_matrix);
	return FALSE;
    }

#if HOR_FM_DEBUG
    hor_message ("hor_fm_compute_fmatrix_linear:  eliminated %d outliers from %d matches\n", outlier_count, match_count);
#endif

    /* else mark the outliers as unmatched.
     */

    outlier_count = 0;
    for (corner_index = 0; corner_index < corner_table1_ptr-> count; corner_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner_index];

	if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	{
	    corner2_ptr = &corner_table2_ptr-> array [corner1_ptr-> match_fore_index];

	    hor_fm_matx_multiply (3, 3, (double *) f_matrix, 3, 1, (double *) &corner1_ptr-> trivec, (double *) &epipolar_line_trivec);
	    distance_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner2_ptr-> trivec);
	    if (distance_squared > outlier_distance_squared)
	    {
		corner1_ptr-> match_fore_index = HOR_FM_NO_MATCH;
		corner2_ptr-> match_back_index = HOR_FM_NO_MATCH;
	    }

	    else
	    {
		hor_fm_matx_multiply (3, 3, (double *) inverse_matrix, 3, 1, (double *) &corner2_ptr-> trivec, (double *) &epipolar_line_trivec);
		distance_squared = hor_fm_compute_perpdistance_squared (&epipolar_line_trivec, &corner1_ptr-> trivec);
		if (distance_squared > outlier_distance_squared)
		{
		    corner1_ptr-> match_fore_index = HOR_FM_NO_MATCH;
		    corner2_ptr-> match_back_index = HOR_FM_NO_MATCH;
		}
	    }
	}
    }

    hor_mat_free (nr_a_matrix);
    hor_mat_free (nr_v_matrix);
    free (nr_w_vector);
    free (a_matrix);
    free (ata_matrix);

    return TRUE;

#undef HOR_FM_ITERATION_COUNT
#undef HOR_FM_UNKNOWNS_COUNT
}

