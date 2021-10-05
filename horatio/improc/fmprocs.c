#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#include "fmatx.h"

static HOR_FM_BOOLEAN fm_match_windows (HOR_FM_CORNER_STR *corner1_ptr, HOR_FM_CORNER_STR *corner2_ptr, int *affinity_ptr);

void hor_fm_match_corners (HOR_FM_MATCH_PARAMS_STR *match_params_ptr, HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, 
			   HOR_FM_MATCH_TABLE_STR *match_table_ptr) /*
-------------------------------------------------------------------
match corners between corner_table1/table2.
*/

{
    int
	corner1_index,
	corner2_index,
	displace_x,
	displace_y,
	match_affinity,
	match_count;

    HOR_FM_BOOLEAN
	good_match_flag;

    HOR_FM_CORNER_STR
	*competing_corner1_ptr,
	*corner1_ptr,
	*corner2_ptr;
    
    HOR_FM_MATCH_STR
	*competing_match_ptr,
	*match_ptr;

    match_table_ptr-> count = corner_table1_ptr-> count;
    for (corner1_index = 0; corner1_index < corner_table1_ptr-> count; corner1_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner1_index];
	corner1_ptr-> match_fore_index = HOR_FM_NO_MATCH;

	match_ptr = &match_table_ptr-> array [corner1_index];
	match_ptr-> found_flag = FALSE;
    }

    for (corner2_index = 0; corner2_index < corner_table2_ptr-> count; corner2_index++)
    {
	corner2_ptr = &corner_table2_ptr-> array [corner2_index];
	corner2_ptr-> match_back_index = HOR_FM_NO_MATCH;
    }

    match_count = 0;
    for (corner1_index = 0; corner1_index < corner_table1_ptr-> count; corner1_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner1_index];
	match_ptr = &match_table_ptr-> array [corner1_index];

	/* initialise match_affinity - after first pass, it always contains the best affinity found so far for corner1.
	 */

	match_affinity = match_params_ptr-> affinity_threshold;

	for (corner2_index = 0; corner2_index < corner_table2_ptr-> count; corner2_index++)
	{
	    corner2_ptr = &corner_table2_ptr-> array [corner2_index];

	    /* search region test.
	     */

	    displace_x = abs (corner1_ptr-> i_bivec. x - corner2_ptr-> i_bivec. x);
	    if (displace_x < match_params_ptr-> distance_threshold)
	    {
		displace_y = abs (corner1_ptr-> i_bivec. y - corner2_ptr-> i_bivec. y);

		if (displace_y < match_params_ptr-> distance_threshold)
		{
		    /* try to match corner1 to corner2 - if the affinity is less (i.e. a stronger match) than the current match_affinity, this routine 
		       updates match_affinity to the new value.
		     */

		    good_match_flag = fm_match_windows (corner1_ptr, corner2_ptr, &match_affinity);

		    if (good_match_flag)
		    {
			match_ptr-> found_flag = TRUE;
			match_ptr-> match_index = corner2_index;
			match_ptr-> affinity = match_affinity;
		    }
		}
	    }
	}

	/* if a match has been found for corner1, check whether corner2 has already been matched elsewhere.
	 */

	if (match_ptr-> found_flag)
	{
	    corner2_ptr = &corner_table2_ptr-> array [match_ptr-> match_index];

	    /* no existing match - just accept it.
	     */

	    if (corner2_ptr-> match_back_index == HOR_FM_NO_MATCH)
	    {
		corner1_ptr-> match_fore_index = match_ptr-> match_index;
		corner2_ptr-> match_back_index = corner1_index;

		match_count++;
	    }

	    /* else corner2 has existing match - accept the winning match between the existing one and the new one.
	     */

	    else
	    {
		competing_corner1_ptr = &corner_table1_ptr-> array [corner2_ptr-> match_back_index];
		competing_match_ptr = &match_table_ptr-> array [corner2_ptr-> match_back_index];
		
		if (match_ptr-> affinity < competing_match_ptr-> affinity)
		{
		    corner1_ptr-> match_fore_index = match_ptr-> match_index;
		    corner2_ptr-> match_back_index = corner1_index;

		    competing_corner1_ptr-> match_fore_index = HOR_FM_NO_MATCH;
		    competing_match_ptr-> found_flag = FALSE;
		}

		else
		    match_ptr-> found_flag = FALSE;
	    }
	}
    }

#if HOR_FM_DEBUG
    hor_message ("hor_fm_match_corners:  trajectory count %d  corner count %d  found %d matches\n", corner_table1_ptr-> count, corner_table2_ptr-> count, match_count);
#endif

}

static HOR_FM_BOOLEAN fm_match_windows (HOR_FM_CORNER_STR *corner1_ptr, HOR_FM_CORNER_STR *corner2_ptr, int *affinity_ptr) /*
--------------------------------------------------------------------------------------------------------------------------
on entry, affinity_ptr contains the min acceptable affinity.  on exit, the computed affinity.  the BOOLEAN return value must be true for an acceptable match.
*/

{
    int
	best_mask2_col_offset,
	best_mask2_row_offset,
	mask1_col_index,
	mask2_col_offset,
	difference_total,
	mask1_row_index,
	mask2_row_offset;

    HOR_FM_BOOLEAN
	match_found_flag;

    /* take a fixed window in the centre of the corner 1 patch.
       step through a series of windows in the corner2 patch, looking for the best match.
       */

    match_found_flag = FALSE;
    for (mask2_row_offset = -HOR_FM_MATCH_SEARCH_OFFSET; mask2_row_offset <= HOR_FM_MATCH_SEARCH_OFFSET; mask2_row_offset++)
    {
	for (mask2_col_offset = -HOR_FM_MATCH_SEARCH_OFFSET; mask2_col_offset <= HOR_FM_MATCH_SEARCH_OFFSET; mask2_col_offset++)
	{
	    difference_total = 0;
	    for (mask1_row_index = HOR_FM_MATCH_MASK1_INDEX; mask1_row_index < HOR_FM_MATCH_MASK1_INDEX + HOR_FM_MATCH_MASK_SIZE; mask1_row_index++)
	    {
		for (mask1_col_index = HOR_FM_MATCH_MASK1_INDEX; mask1_col_index < HOR_FM_MATCH_MASK1_INDEX + HOR_FM_MATCH_MASK_SIZE; mask1_col_index++)
		{
		    difference_total += abs (corner1_ptr-> patch_ptr-> array. uc [mask1_row_index][mask1_col_index] 
					     - corner2_ptr-> patch_ptr-> array. uc [mask1_row_index + mask2_row_offset][mask1_col_index + mask2_col_offset]);
		}
			
		if (difference_total > *affinity_ptr)
		    break;
	    }

	    if (difference_total < *affinity_ptr)
	    {
		match_found_flag = TRUE;
		*affinity_ptr = difference_total;
		best_mask2_col_offset = mask2_col_offset;
		best_mask2_row_offset = mask2_row_offset;
	    }
	}
    }

    if (!match_found_flag)
	return FALSE;

    /* if match found, do a final test - is the detected match the best available, or is it possible to shift the window in the corner2 patch by 1 pixel to get a better match - if the
       latter, reject the match.
       */

    else if (abs (best_mask2_col_offset) < HOR_FM_MATCH_SEARCH_OFFSET && abs (best_mask2_row_offset) < HOR_FM_MATCH_SEARCH_OFFSET)
	return TRUE;

    /* else we are on the border of the search region, so check whether there are better matches outside the border.
     */

    else
    {
	for (mask2_row_offset = best_mask2_row_offset - 1; mask2_row_offset <= best_mask2_row_offset + 1; mask2_row_offset++)
	{
	    for (mask2_col_offset = best_mask2_col_offset - 1; mask2_col_offset <= best_mask2_col_offset + 1; mask2_col_offset++)
	    {
		/* only do the check if this mask position has not yet been checked.
		 */

		if (abs (mask2_col_offset) > HOR_FM_MATCH_SEARCH_OFFSET || abs (mask2_row_offset) > HOR_FM_MATCH_SEARCH_OFFSET)
		{
		    difference_total = 0;
		    for (mask1_row_index = HOR_FM_MATCH_MASK1_INDEX; mask1_row_index < HOR_FM_MATCH_MASK1_INDEX + HOR_FM_MATCH_MASK_SIZE; mask1_row_index++)
		    {
			for (mask1_col_index = HOR_FM_MATCH_MASK1_INDEX; mask1_col_index < HOR_FM_MATCH_MASK1_INDEX + HOR_FM_MATCH_MASK_SIZE; mask1_col_index++)
			{
			    difference_total += abs (corner1_ptr-> patch_ptr-> array. uc [mask1_row_index][mask1_col_index] 
						     - corner2_ptr-> patch_ptr-> array. uc [mask1_row_index + mask2_row_offset][mask1_col_index + mask2_col_offset]);
			}
		    }

		    /* if found a better match (lower affinity), then scrap this match altogether.
		     */

		    if (difference_total < *affinity_ptr)
			return FALSE;
		}
	    }
	}

	return TRUE;
    }
}

void hor_fm_match_corners_fmatrix (HOR_FM_MATCH_PARAMS_STR *match_params_ptr, double f_matrix [3][3], HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, 
				   HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, HOR_FM_MATCH_TABLE_STR *match_table_ptr) /*
-----------------------------------------------------------------------------------------------------------------------
match corner_tables 1 and 2, computing the F matrix from existing matches and using it to guide matching.
*/

{
    float
	epipolar_distance_squared,
	epipolar_distance_threshold_squared;

    int
	corner1_index,
	corner2_index,
	displace_x,
	displace_y,
	match_affinity,
	match_count;

    HOR_FM_BOOLEAN
	good_match_flag;

    HOR_FM_CORNER_STR
	*competing_corner1_ptr,
	*corner1_ptr,
	*corner2_ptr;

    HOR_FM_MATCH_STR
	*competing_match_ptr,
	*match_ptr;

    HOR_FM_TRIVEC_DOUBLE
	epipolar_trivec;

    for (corner1_index = 0; corner1_index < corner_table1_ptr-> count; corner1_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner1_index];
	if (corner1_ptr-> match_fore_index == HOR_FM_NO_MATCH)
	{
	    match_ptr = &match_table_ptr-> array [corner1_index];
      	    match_ptr-> found_flag = FALSE; /* in case outliers have been removed elsewhere so corner is unmatched but match entry is set. */
	}
    }

    epipolar_distance_threshold_squared = HOR_FM_SQUARE (match_params_ptr-> epipolar_distance_threshold);

    /* step through corner_table1 attempting to match all unmatched corners.
     */

    for (corner1_index = 0; corner1_index < corner_table1_ptr-> count; corner1_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner1_index];

	if (corner1_ptr-> match_fore_index == HOR_FM_NO_MATCH)
	{
	    match_ptr = &match_table_ptr-> array [corner1_index];

	    /* initialise match_affinity - after first pass, it always contains the best affinity found so far for corner1.
	       tighter search area in this routine so relax the threshold by increasing it.
	       */

	    match_affinity = (float) match_params_ptr-> affinity_threshold * 1.5;

	    /* get the epipolar line in image 2.
	     */

	    hor_fm_matx_multiply (3, 3, (double *) f_matrix, 3, 1, (double *) &corner1_ptr-> trivec, (double *) &epipolar_trivec);

	    for (corner2_index = 0; corner2_index < corner_table2_ptr-> count; corner2_index++)
	    {
		corner2_ptr = &corner_table2_ptr-> array [corner2_index];

		/* search region test.
		 */

		displace_x = abs (corner1_ptr-> i_bivec. x - corner2_ptr-> i_bivec. x);
		if (displace_x < match_params_ptr-> distance_threshold)
		{
		    displace_y = abs (corner1_ptr-> i_bivec. y - corner2_ptr-> i_bivec. y);
		    if (displace_y < match_params_ptr-> distance_threshold)
		    {

			/* proximity to epipolar line test.
			 */

			epipolar_distance_squared = hor_fm_compute_perpdistance_squared (&epipolar_trivec, &corner2_ptr-> trivec);
			if (epipolar_distance_squared <= epipolar_distance_threshold_squared)
			{

			    /* try to match corner1 to corner2 - if the affinity is less (i.e. a stronger match) than the current match_affinity, this routine 
			       updates match_affinity to the new value.
			       */

			    good_match_flag = fm_match_windows (corner1_ptr, corner2_ptr, &match_affinity);

			    if (good_match_flag)
			    {
				match_ptr-> found_flag = TRUE;
				match_ptr-> match_index = corner2_index;
				match_ptr-> affinity = match_affinity;
			    }
			}
		    }
		}
	    }

	    /* if a match has been found for corner1, check whether corner2 has already been matched elsewhere.
	     */

	    if (match_ptr-> found_flag)
	    {
		corner2_ptr = &corner_table2_ptr-> array [match_ptr-> match_index];

		/* no existing match - just accept it.
		 */

		if (corner2_ptr-> match_back_index == HOR_FM_NO_MATCH)
		{
		    corner1_ptr-> match_fore_index = match_ptr-> match_index;
		    corner2_ptr-> match_back_index = corner1_index;
		}

		/* else corner2 has existing match - accept the winning match between the existing one and the new one.
		 */

		else
		{
		    competing_corner1_ptr = &corner_table1_ptr-> array [corner2_ptr-> match_back_index];
		    competing_match_ptr = &match_table_ptr-> array [corner2_ptr-> match_back_index];

		    if (match_ptr-> affinity < competing_match_ptr-> affinity)
		    {
			corner1_ptr-> match_fore_index = match_ptr-> match_index;
			corner2_ptr-> match_back_index = corner1_index;
			competing_corner1_ptr-> match_fore_index = HOR_FM_NO_MATCH;
			competing_match_ptr-> found_flag = FALSE;
		    }

		    else
			match_ptr-> found_flag = FALSE;
		}
	    }
	}
    }

#if HOR_FM_DEBUG
    match_count = 0;
    for (corner1_index = 0; corner1_index < corner_table1_ptr-> count; corner1_index++)
    {
	corner1_ptr = &corner_table1_ptr-> array [corner1_index];
	if (corner1_ptr-> match_fore_index != HOR_FM_NO_MATCH)
	    match_count++;
    }

    hor_message ("hor_fm_match_corners_fmatrix:  trajectory count %d  corner count %d  found %d matches\n", corner_table1_ptr-> count, corner_table2_ptr-> count, match_count);
#endif
}

