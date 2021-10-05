#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#include "fmatx.h"

HOR_FM_TRIVEC_DOUBLE hor_fm_bivec_to_trivec (HOR_FM_BIVEC_DOUBLE *bivec_ptr) /*
----------------------------------------------------------------------------
convert non-homogeneous 2-vector to homogeneous 3-vector.
*/

{
    HOR_FM_TRIVEC_DOUBLE
	trivec;

    /* use HOR_FM_TRIVEC_HOMG_VALUE to keep all three components of the homogeneous vector in the same ball-park - for better conditioning.
     */

    trivec. x = bivec_ptr-> x;
    trivec. y = bivec_ptr-> y;
    trivec. z = HOR_FM_TRIVEC_HOMG_VALUE;

    return trivec;
}

float hor_fm_compute_perpdistance_squared (HOR_FM_TRIVEC_DOUBLE *line_trivec_ptr, HOR_FM_TRIVEC_DOUBLE *point_trivec_ptr) /*
-------------------------------------------------------------------------------------------------------------------------
perpendicular distance of point from line.
formula from faux and pratt - modified to take account of homogeneous vectors.
*/

{
    return 
	HOR_FM_SQUARE (HOR_FM_TRIVEC_HOMG_VALUE)
	    * HOR_FM_SQUARE ((line_trivec_ptr-> x * point_trivec_ptr-> x + line_trivec_ptr-> y * point_trivec_ptr-> y + line_trivec_ptr-> z * point_trivec_ptr-> z) / point_trivec_ptr-> z)
		/ (HOR_FM_SQUARE (line_trivec_ptr-> x) + HOR_FM_SQUARE (line_trivec_ptr-> y));
}

float hor_fm_distance_squared_between_points (HOR_FM_TRIVEC_DOUBLE *trivec1_ptr, HOR_FM_TRIVEC_DOUBLE *trivec2_ptr) /*
-------------------------------------------------------------------------------------------------------------------
return squared distance between two 2D points, which are represented by homogeneous vectors.
*/

{
    return 
	HOR_FM_SQUARE (HOR_FM_TRIVEC_HOMG_VALUE)
	    * (HOR_FM_SQUARE ((float) trivec1_ptr-> x / (float) trivec1_ptr-> z - (float) trivec2_ptr-> x / (float) trivec2_ptr-> z) 
	       + HOR_FM_SQUARE ((float) trivec1_ptr-> y / (float) trivec1_ptr-> z - (float) trivec2_ptr-> y / (float) trivec2_ptr-> z));
}

void hor_fm_matx_multiply (int row_count1, int col_count1, double *matrix1_ptr, int row_count2, int col_count2, double *matrix2_ptr, double *matrix_product_ptr) /*
----------------------------------------------------------------------------------------------------------------------------------------------------------------
multiply two matrices.
*/

{
    int
	col_index,
	matrix1_index,
	product_index,
	row_index,
	sum_index;

    if (col_count1 != row_count2)
	hor_error ("hor_fm_matx_multiply:  multiplying incompatible matrices", HOR_FATAL);

    if (matrix1_ptr == matrix_product_ptr || matrix2_ptr == matrix_product_ptr)
	hor_error ("hor_fm_matx_multiply:  illegal attempt to place matrix product in one of multiplier matrices", HOR_FATAL);

    product_index = 0;
    for (row_index = 0; row_index < row_count1; row_index++)
    {
	for (col_index = 0; col_index < col_count2; col_index++)
	{
	    matrix1_index = HOR_FM_MATX_INDEX_2DTO1D (row_index, 0, col_count1);
	    matrix_product_ptr [product_index] = matrix1_ptr [matrix1_index++] * matrix2_ptr [HOR_FM_MATX_INDEX_2DTO1D (0, col_index, col_count2)];
	    for (sum_index = 1; sum_index < col_count1; sum_index++)
	    {
		matrix_product_ptr [product_index] += matrix1_ptr [matrix1_index++] * matrix2_ptr [HOR_FM_MATX_INDEX_2DTO1D (sum_index, col_index, col_count2)];
	    }
	    product_index++;
	}
    }
}

void hor_fm_matx_premultiply_by_transpose (int row_count, int col_count, double *source_ptr, double *target_ptr) /*
----------------------------------------------------------------------------------------------------------------
source is a (row x col) matrix.  premultiply by transpose and put result in the (col x col) target matrix.
obscurely written but hopefully faster than explicitly doing the transpose and multiplying the matrices.
*/

{
    int
	diagonal_index,
	source_index,
	source1_index,
	source2_index,
	source_size,
	target_index,
	target_col_index,
	target_row_index,
	target1_index,
	target2_index;

    source_size = row_count * col_count;

    /* diagonal elements of target.
     */

    for (diagonal_index = 0; diagonal_index < col_count; diagonal_index++)
    {
	/* element (diagonal_index, diagonal_index) of target is formed by squaring column (diagonal_index) of source.
	 */

	target_index = HOR_FM_MATX_INDEX_2DTO1D (diagonal_index, diagonal_index, col_count);
	target_ptr [target_index] = 0;

	for (source_index = HOR_FM_MATX_INDEX_2DTO1D (0, diagonal_index, col_count); source_index < source_size; source_index += col_count)
	    target_ptr [target_index] += HOR_FM_SQUARE (source_ptr [source_index]);
    }

    /* off-diagonal elements of target.
     */

    for (target_row_index = 0; target_row_index < col_count; target_row_index++)
    {
	for (target_col_index = target_row_index + 1; target_col_index < col_count; target_col_index++)
	{
	    /* element (target_row_index, target_col_index), and its transpose, of target is formed by multiplying column (target_row_index) by column (target_col_index) of source.
	     */

	    target1_index = HOR_FM_MATX_INDEX_2DTO1D (target_row_index, target_col_index, col_count);
	    target2_index = HOR_FM_MATX_INDEX_2DTO1D (target_col_index, target_row_index, col_count);
	    target_ptr [target1_index] = 0;

	    source1_index = HOR_FM_MATX_INDEX_2DTO1D (0, target_row_index, col_count);
	    source2_index = HOR_FM_MATX_INDEX_2DTO1D (0, target_col_index, col_count);
	    while (source1_index < source_size)
	    {
		target_ptr [target1_index] += source_ptr [source1_index] * source_ptr [source2_index];

		source1_index += col_count; 
		source2_index += col_count; 
	    }

	    target_ptr [target2_index] = target_ptr [target1_index];
	}
    }
}

void hor_fm_matx_transpose (int row_count, int col_count, double *matrix_ptr, double *matrix_transpose_ptr) /*
-----------------------------------------------------------------------------------------------------------
transpose a matrix.
*/

{
    int	
	col_index,
	row_index,
	transpose_col_count;

    if (matrix_transpose_ptr == matrix_ptr)
	hor_error ("hor_fm_matx_transpose:  illegal attempt to place transpose in original matrix", HOR_FATAL);

    transpose_col_count = row_count;

    for (row_index = 0; row_index < row_count; row_index++)
    {
	for (col_index = 0; col_index < col_count; col_index++)
	{
	    matrix_transpose_ptr [HOR_FM_MATX_INDEX_2DTO1D (col_index, row_index, transpose_col_count)] = matrix_ptr [HOR_FM_MATX_INDEX_2DTO1D (row_index, col_index, col_count)];
	}
    }
}

HOR_FM_BIVEC_DOUBLE hor_fm_trivec_to_bivec (HOR_FM_TRIVEC_DOUBLE *trivec_ptr) /*
-----------------------------------------------------------------------------
convert homogeneous 3-vector to non-homogeneous 2-vector.
*/

{
    HOR_FM_BIVEC_DOUBLE
	bivec;

    if (trivec_ptr-> z == 0)
	hor_error ("hor_fm_trivec_to_bivec:  attempt to project point at infinity", HOR_FATAL);

    bivec. x = trivec_ptr-> x * HOR_FM_TRIVEC_HOMG_VALUE / trivec_ptr-> z;
    bivec. y = trivec_ptr-> y * HOR_FM_TRIVEC_HOMG_VALUE / trivec_ptr-> z;

    return bivec;
}

