#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#include "fmatx.h"

static void fmatch_interface (Hor_Trajectory_Map *map, Hor_Corner_Map *new_corner_map, int range, int pixel_diff_threshold);

/*******************
*   void @hor_fmatx_cm_traj_state_free ( void *state )
*
*   Fmatrix corner matcher trajectory state free function.
********************/
void hor_fmatx_cm_traj_state_free ( void *state )
{
   hor_free_image ( ((Hor_Fmatx_CM_Traj_State *) state)->patch );
   hor_free ( state );
}

static Hor_Bool fmatx_cm_test_traj ( Hor_Trajectory *traj, void *data )
{
   if ( traj->type != HOR_CM_FMATX || traj->status != HOR_UNMATCHED )
      return HOR_TRUE;

   return HOR_FALSE;
}
   
static void fmatx_init_new_corners ( Hor_Trajectory_Map *map,
				   Hor_Corner_Map     *new_corner_map )
{
   int c, r, width = new_corner_map->width, height = new_corner_map->height;
   Hor_Corner ***corners = new_corner_map->corners, *corner;
   Hor_Fmatx_CM_Traj_State *new_state;
   Hor_Traj_Point        *new_point;

   for ( r = 0; r < height; r++ )
      for ( c = 0; c < width; c++ )
	 if ( (corner = corners[r][c]) != NULL &&
	      corner->status == HOR_UNMATCHED )
	 {
	    new_point = hor_malloc_type(Hor_Traj_Point);
	    new_point->cf = corner->cf;
	    new_point->rf = corner->rf;
	    new_state = hor_malloc_type(Hor_Fmatx_CM_Traj_State);
	    new_state->patch = hor_copy_image ( &corner->patch-> image );
	    hor_add_new_trajectory ( map, HOR_CM_FMATX,
				     (void *) new_point, 100000,
				     (void *) new_state, hor_free_func,
				     hor_fmatx_cm_traj_state_free,
				     hor_cm_traj_display );
	 }
}

/*******************
*   void @hor_fmatx_match_corners (
*      Hor_Trajectory_Map *map,            (match map)
*      Hor_Corner_Map     *new_corner_map, (latest corner map)
*
*      (parameters:)
*      int   range,              (maximum corner motion between images)
*      int   pixel_diff_threshold)
*
*   Matches corners from two images using the fundamental matrix for guidance.
********************/
void hor_fmatx_match_corners (Hor_Trajectory_Map *map,
			       Hor_Corner_Map     *new_corner_map,

			     /* parameters */
			     int   range,
			     int   pixel_diff_threshold)
{
   if ( range < 0 || pixel_diff_threshold < 0 )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return;
   }

/*!!!!!!!!!!!!!!

   if ( map->type != HOR_CM_FMATX )
      hor_error ( "illegal corner match map type (hor_fmatch_match_corners)",
		  HOR_FATAL );
*/

   /* match new corners to old corner trajectories.
      if test to see if trajectories exist i.e. not the first image. */

   if (map-> traj_list != NULL)
       fmatch_interface (map, new_corner_map, range, pixel_diff_threshold);

   /* discard old corner trajectories */

   hor_delete_old_trajectories ( map, fmatx_cm_test_traj, NULL );

   /* create corner matches for new corner features */
   fmatx_init_new_corners ( map, new_corner_map );
}

static void fmatch_interface (Hor_Trajectory_Map *map, Hor_Corner_Map *new_corner_map, int range, int pixel_diff_threshold) /*
---------------------------------------------------------------------------------------------------------------------------
*/

{
    double
	f_matrix [3][3];

    int
	corner_count,
	corner_index,
	traj_count;

    HOR_FM_BOOLEAN
	fmatrix_flag;

    Hor_Corner 
	*horatio_corner;

    HOR_FM_CORNER_STR
	*corner1_ptr,
	*corner2_ptr;

    HOR_FM_CORNER_TABLE_STR
	*corner_table1_ptr,
	*corner_table2_ptr;

    HOR_FM_FMATRIX_PARAMS_STR
	fmatrix_params_str;

    HOR_FM_MATCH_PARAMS_STR
	match_params_str;

    HOR_FM_MATCH_TABLE_STR
	*match_table_ptr;

    Hor_Fmatx_CM_Traj_State 
	*new_state,
	*traj_state;

    Hor_List
	list;

    Hor_Traj_Point 
	*new_point,
	*traj_point;

    Hor_Trajectory 
	*traj;

    /* check the image patch size is correct.
     */

    for (list = map-> traj_list; list != NULL; list = list-> next)
    {
	traj = (Hor_Trajectory *) list-> contents;

	if (traj-> type == HOR_CM_FMATX)
	{
	    traj_state = (Hor_Fmatx_CM_Traj_State *) traj-> state;
	    if (traj_state-> patch-> width != 13)
	    {
		hor_error ("fmatch_interface:  patch width must equal 13 for the fmatch corner matcher", HOR_NON_FATAL);
		return;
	    }

	    /* else terminate the iteration and carry on processing.
	     */

	    else
		break;
	}
    }

    /* count the number of trajectories which were still visible in the last image.
     */

    traj_count = 0;
    for (list = map-> traj_list; list != NULL; list = list-> next)
    {
	traj = (Hor_Trajectory *) list-> contents;

	if (traj-> type == HOR_CM_FMATX && traj-> not_seen == 0)
	    traj_count++;
    }

    /* count the number of corners in the new image.
     */

    corner_count = 0;
    for (list = new_corner_map-> corner_list; list != NULL; list = list-> next)
	corner_count++;

    /* get memory for the local data structures.
     */

    corner_table1_ptr = (HOR_FM_CORNER_TABLE_STR *) malloc (sizeof (corner_table1_ptr-> count) + traj_count * sizeof (corner_table1_ptr-> array [0]));
    corner_table2_ptr = (HOR_FM_CORNER_TABLE_STR *) malloc (sizeof (corner_table2_ptr-> count) + corner_count * sizeof (corner_table2_ptr-> array [0]));
    match_table_ptr = (HOR_FM_MATCH_TABLE_STR *) malloc (sizeof (match_table_ptr-> count) + traj_count * sizeof (match_table_ptr-> array [0]));
    if (corner_table1_ptr == NULL || corner_table2_ptr == NULL || match_table_ptr == NULL)
    {
	hor_error ("fmatch_interface:  unable to proceed due to memory failure", HOR_NON_FATAL);
	if (corner_table1_ptr != NULL)
	    free (corner_table1_ptr);
	if (corner_table2_ptr != NULL)
	    free (corner_table2_ptr);
	if (match_table_ptr != NULL)
	    free (match_table_ptr);
	return;
    }

    /* copy the trajectory data into the local data structure.
     */

    corner_table1_ptr-> count = 0;
    for (list = map-> traj_list; list != NULL; list = list-> next)
    {
	traj = (Hor_Trajectory *) list-> contents;

	if (traj-> type == HOR_CM_FMATX && traj-> not_seen == 0)
	{
	    traj_point = (Hor_Traj_Point *) traj-> last-> contents;
	    traj_state = (Hor_Fmatx_CM_Traj_State *) traj-> state;

	    corner1_ptr = &corner_table1_ptr-> array [corner_table1_ptr-> count];
	    corner_table1_ptr-> count++;

	    corner1_ptr-> bivec. x = traj_point-> cf;
	    corner1_ptr-> bivec. y = traj_point-> rf;
	    corner1_ptr-> i_bivec. x = HOR_FM_ROUND (traj_point-> cf);
	    corner1_ptr-> i_bivec. y = HOR_FM_ROUND (traj_point-> rf);
	    corner1_ptr-> trivec = hor_fm_bivec_to_trivec (&corner1_ptr-> bivec);
	    corner1_ptr-> patch_ptr = traj_state-> patch;
	}
    }

    /* copy the corner data into the local data structure.
     */

    corner_table2_ptr-> count = 0;
    for (list = new_corner_map-> corner_list; list != NULL; list = list-> next)
    {
	horatio_corner = (Hor_Corner *) list-> contents;

	corner2_ptr = &corner_table2_ptr-> array [corner_table2_ptr-> count];
	corner_table2_ptr-> count++;

	corner2_ptr-> bivec. x = horatio_corner-> cf;
	corner2_ptr-> bivec. y = horatio_corner-> rf;
	corner2_ptr-> i_bivec. x = HOR_FM_ROUND (horatio_corner-> cf);
	corner2_ptr-> i_bivec. y = HOR_FM_ROUND (horatio_corner-> rf);
	corner2_ptr-> trivec = hor_fm_bivec_to_trivec (&corner2_ptr-> bivec);
	corner2_ptr-> patch_ptr = &horatio_corner-> patch-> image;
    }

    /* corner matching parameters.
     */

    match_params_str. distance_threshold = range;
    match_params_str. affinity_threshold
	= HOR_FM_MATCH_MASK_SIZE * HOR_FM_MATCH_MASK_SIZE * pixel_diff_threshold; /*  affinity measure is the total no of pixels in the mask times the pixel difference threshold. */
    match_params_str. epipolar_distance_threshold = HOR_FM_MATCH_EPIPOLAR_THRESHOLD;

    /* parameters for fmatrix computation.
     */

    fmatrix_params_str. epipole_distance_threshold = HOR_FM_FMATX_EPIPOLE_THRESHOLD;
    fmatrix_params_str. epipolar_outlier_threshold = HOR_FM_FMATX_OUTLIER_THRESHOLD;
    fmatrix_params_str. epipolar_weighting_min = HOR_FM_FMATX_WEIGHTING_MIN;
    fmatrix_params_str. eliminate_ratio_max = HOR_FM_FMATX_ELIMINATE_RATIO_MAX;

    /* do the matching.
     */

    hor_fm_match_corners (&match_params_str, corner_table1_ptr, corner_table2_ptr, match_table_ptr);
    fmatrix_flag = hor_fm_compute_fmatrix_linear (&fmatrix_params_str, corner_table1_ptr, corner_table2_ptr, f_matrix);
    if (fmatrix_flag)
	hor_fm_match_corners_fmatrix (&match_params_str, f_matrix, corner_table1_ptr, corner_table2_ptr, match_table_ptr);

    /* else one further try with a relaxed threshold.
     */

    else
    {
	hor_message ("fmatch_interface:  computation of fundamental matrix failed, things looking bad but trying again");
	match_params_str. affinity_threshold = 1.5 * (float) match_params_str. affinity_threshold;
	hor_fm_match_corners (&match_params_str, corner_table1_ptr, corner_table2_ptr, match_table_ptr);
	fmatrix_flag = hor_fm_compute_fmatrix_linear (&fmatrix_params_str, corner_table1_ptr, corner_table2_ptr, f_matrix);
	if (fmatrix_flag)
	    hor_fm_match_corners_fmatrix (&match_params_str, f_matrix, corner_table1_ptr, corner_table2_ptr, match_table_ptr);
	else
	    hor_message ("fmatch_interface:  computation of fundamental matrix failed again, matches may be rubbish");
    }

    /* update the existing trajectories.
     */

    corner_index = 0;
    for (list = map-> traj_list; list != NULL; list = list-> next)
    {
	traj = (Hor_Trajectory *) list-> contents;

	if (traj-> type == HOR_CM_FMATX && traj-> not_seen == 0)
	{
	    traj_state = (Hor_Fmatx_CM_Traj_State *) traj-> state;

	    corner1_ptr = &corner_table1_ptr-> array [corner_index];
	    corner_index++;

	    if (corner1_ptr-> match_fore_index == HOR_FM_NO_MATCH)
	    {
		traj-> status = HOR_MATCHED;
		hor_add_trajectory_element (traj, NULL);
	    }

	    else
	    {
		corner2_ptr = &corner_table2_ptr-> array [corner1_ptr-> match_fore_index];

		traj-> status = HOR_MATCHED;
		traj_point = hor_malloc_type (Hor_Traj_Point);
		traj_point-> cf = corner2_ptr-> bivec. x;
		traj_point-> rf = corner2_ptr-> bivec. y;
		hor_add_trajectory_element (traj, (void *) traj_point);
		hor_convert_image_data (corner2_ptr-> patch_ptr, traj_state-> patch);
	    }
	}
    }

    /* add new unmatched trajectories.
     */

    corner_index = 0;
    for (list = new_corner_map-> corner_list; list != NULL; list = list-> next)
    {
	horatio_corner = (Hor_Corner *) list-> contents;

	corner2_ptr = &corner_table2_ptr-> array [corner_index];
	corner_index++;

	if (corner2_ptr-> match_back_index != HOR_FM_NO_MATCH)
	    horatio_corner-> status = HOR_MATCHED;

	else
	{
	    new_point = hor_malloc_type (Hor_Traj_Point);
	    new_point-> cf = corner2_ptr-> bivec. x;
	    new_point-> rf = corner2_ptr-> bivec. y;
	    new_state = hor_malloc_type (Hor_Fmatx_CM_Traj_State);
	    new_state-> patch = hor_copy_image (corner2_ptr-> patch_ptr);
	    hor_add_new_trajectory (map, HOR_CM_FMATX,
				    (void *) new_point, 100000,
				    (void *) new_state, hor_free_func,
				    hor_fmatx_cm_traj_state_free,
				    hor_cm_traj_display);
	}
    }

    free (corner_table1_ptr);
    free (corner_table2_ptr);
    free (match_table_ptr);
}



