/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/improc.h"

Hor_Coarse_Feature_Map *hor_alloc_coarse_feature_map ( int c0,    int r0,
						       int cols,  int rows,
						       int csize, int rsize,
						       int max_features )
{
   Hor_Coarse_Feature_Map *feature_map;
   int i, j;
   Hor_Feature_Def **ptr;

   feature_map = hor_malloc_type(Hor_Coarse_Feature_Map);
   if ( feature_map == NULL )
   { hor_errno = HOR_IMPROC_ALLOCATION_FAILED; return NULL; }

   feature_map->feature = hor_malloc_ntype ( Hor_Feature_Def ***, rows );
   if ( feature_map->feature == NULL )
   {
      hor_free ( (void *) feature_map );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return NULL;
   }

   feature_map->feature[0] = hor_malloc_ntype ( Hor_Feature_Def **, rows*cols);
   if ( feature_map->feature[0] == NULL )
   {
      hor_free_multiple ( (void *) feature_map->feature,
			  (void *) feature_map, NULL );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return NULL;
   }

   for ( i = 1; i < rows; i++ )
      feature_map->feature[i] = feature_map->feature[i-1] + cols;

   ptr = hor_malloc_ntype ( Hor_Feature_Def *, rows*cols*max_features );
   if ( ptr == NULL )
   {
      hor_free_multiple ( (void *) feature_map->feature[0],
			  (void *) feature_map->feature,
			  (void *) feature_map, NULL );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return NULL;
   }

   for ( i = 0; i < rows; i++ )
      for ( j = 0; j < cols; j++, ptr += max_features )
	 feature_map->feature[i][j] = ptr;

   feature_map->no_features = hor_malloc_ntype ( int *, rows );
   if ( feature_map->no_features == NULL )
   {
      hor_free_multiple ( (void *) feature_map->feature[0][0],
			  (void *) feature_map->feature[0],
			  (void *) feature_map->feature,
			  (void *) feature_map, NULL );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return NULL;
   }

   feature_map->no_features[0] = hor_malloc_ntype ( int, rows*cols );
   if ( feature_map->no_features[0] == NULL )
   {
      hor_free_multiple ( (void *) feature_map->no_features,
			  (void *) feature_map->feature[0][0],
			  (void *) feature_map->feature[0],
			  (void *) feature_map->feature,
			  (void *) feature_map, NULL );
      hor_errno = HOR_IMPROC_ALLOCATION_FAILED;
      return NULL;
   }

   for ( i = 1; i < rows; i++ )
      feature_map->no_features[i] = feature_map->no_features[i-1] + cols;

   for ( i = 0; i < rows; i++ )
      for ( j = 0; j < cols; j++ )
	 feature_map->no_features[i][j] = 0;

   feature_map->c0 = c0;
   feature_map->r0 = r0;
   feature_map->cols = cols;
   feature_map->rows = rows;
   feature_map->csize = csize;
   feature_map->rsize = rsize;
   feature_map->feature_array = NULL;
   feature_map->farray_size   = 0;
   return feature_map;
}

void hor_free_coarse_feature_map ( Hor_Coarse_Feature_Map *feature_map )
{
   hor_free ( (void *) feature_map->no_features[0] );
   hor_free ( (void *) feature_map->no_features );
   hor_free ( (void *) feature_map->feature[0][0] );
   hor_free ( (void *) feature_map->feature[0] );
   if ( feature_map->feature_array != NULL )
   {
      int i;

      for ( i = 0; i < feature_map->farray_size; i++ )
      {
	 if ( feature_map->feature_array[i].z != NULL )
	    hor_mat_free ( feature_map->feature_array[i].z );

	 if ( feature_map->feature_array[i].Rinv != NULL )
	    hor_mat_free ( feature_map->feature_array[i].Rinv );
      }

      hor_free ( (void *) feature_map->feature_array );
   }

   hor_free ( (void *) feature_map->feature );
   hor_free ( (void *) feature_map );
}

Hor_Feature_Def *hor_make_feature_array ( Hor_List feature_list,
					  int      no_features )
{
   Hor_Feature_Def *feature_array = hor_malloc_ntype ( Hor_Feature_Def,
						       no_features );
   Hor_Feature_Def *feature;

   if ( feature_array == NULL ) return NULL;

   for ( feature = feature_array; feature_list != NULL;
	 feature++, feature_list = feature_list->next )
   {
      feature->status       = HOR_UNMATCHED;
      feature->feature      = feature_list->contents;
      feature->match        = NULL;
      feature->difference   = 0.0;
      feature->z            = NULL;
      feature->Rinv         = NULL;
      feature->z_type       = 0;
      feature->zdata        = NULL;
   }

   return feature_array;
}

Hor_Coarse_Feature_Map *hor_make_coarse_corner_map (Hor_Corner_Map *corner_map,
						    int csize,   int rsize,
						    int cfactor, int rfactor,
						    int max_features,
						    double offset_x,
						    double offset_y,
						    double xy_sigma)
{
   Hor_Coarse_Feature_Map *feature_map;
   int cols, rows, c, r, c0 = corner_map->c0, r0 = corner_map->r0, i;
   Hor_Feature_Def *feature;

   if ( (cfactor % 2 != 1) || (rfactor % 2 != 1) )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return NULL;
   }

   cols = (corner_map->width+csize-1)/csize;
   rows = (corner_map->height+rsize-1)/rsize;

   feature_map = hor_alloc_coarse_feature_map ( c0, r0, cols, rows,
					        csize, rsize, max_features );
   if ( feature_map == NULL ) return NULL;

   feature_map->farray_size = corner_map->ncorners;
   feature_map->feature_array =
      hor_make_feature_array ( corner_map->corner_list, corner_map->ncorners );

   if ( feature_map->feature_array == NULL )
   {
      hor_free_coarse_feature_map ( feature_map );
      return NULL;
   }

   for ( feature = feature_map->feature_array, i = 0; i < corner_map->ncorners;
	 feature++, i++ )
   {
      int cstart, rstart, cend, rend;
      Hor_Corner *corner = (Hor_Corner *) feature->feature;

      cstart = ((int) corner->cf - c0)/csize - cfactor/2;
      rstart = ((int) corner->rf - r0)/rsize - rfactor/2;
      cend = cstart + cfactor;
      rend = rstart + rfactor;
      if ( cstart < 0 ) cstart = 0;
      if ( rstart < 0 ) rstart = 0;
      if ( cend > cols ) cend = cols;
      if ( rend > rows ) rend = rows;
      for ( r = rstart; r < rend; r++ )
	 for ( c = cstart; c < cend; c++ )
	    feature_map->feature[r][c][feature_map->no_features[r][c]++] =
	       feature;

      /* measurement vector stuff */
      feature->z = hor_mats_fill ( 2, 1, corner->cf - offset_x,
				         corner->rf - offset_y );
      feature->Rinv = hor_mats_diagonal ( 2, 1.0/(xy_sigma*xy_sigma),
					     1.0/(xy_sigma*xy_sigma) );
      feature->z_type = HOR_POINT_XY;
      feature->zdata = NULL;
   }

   return feature_map;
}

Hor_Coarse_Feature_Map
   *hor_make_coarse_line_map ( Hor_Line_Segment_Map *line_map,
			       int csize,   int rsize,
			       int cfactor, int rfactor,
			       int max_features,
			       double offset_x, double offset_y, double zs,
			       double normal_sigma )
{
   Hor_Coarse_Feature_Map *feature_map;
   Hor_Feature_Def        *feature;
   int cols, rows, c, r, c0 = line_map->c0, r0 = line_map->r0, i;
   double x1, y1, x2, y2, xd, yd, length, s2 = normal_sigma*normal_sigma;

   if ( (cfactor % 2 != 1) || (rfactor % 2 != 1) )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return NULL;
   }

   cols = (line_map->width+csize-1)/csize;
   rows = (line_map->height+rsize-1)/rsize;

   feature_map = hor_alloc_coarse_feature_map ( c0, r0, cols, rows,
					        csize, rsize, max_features );
   if ( feature_map == NULL ) return NULL;

   feature_map->farray_size = line_map->nlines;
   feature_map->feature_array =
      hor_make_feature_array ( line_map->line_list, line_map->nlines );
   if ( feature_map->feature_array == NULL )
   {
      hor_free_coarse_feature_map ( feature_map );
      return NULL;
   }

   for ( feature = feature_map->feature_array, i = 0; i < line_map->nlines;
	 feature++, i++ )
   {
      int cstart, rstart, cend, rend;
      Hor_Line_Segment *segment = (Hor_Line_Segment *) feature->feature;
      Hor_Line_End *le;

      cstart = ((int) (0.5F*(segment->x1+segment->x2)) - c0)/csize - cfactor/2;
      rstart = ((int) (0.5F*(segment->y1+segment->y2)) - r0)/rsize - rfactor/2;
      cend = cstart + cfactor;
      rend = rstart + rfactor;
      if ( cstart < 0 ) cstart = 0;
      if ( rstart < 0 ) rstart = 0;
      if ( cend > cols ) cend = cols;
      if ( rend > rows ) rend = rows;
      for ( r = rstart; r < rend; r++ )
	 for ( c = cstart; c < cend; c++ )
	    feature_map->feature[r][c][feature_map->no_features[r][c]++] =
	       feature;

      /* measurement vector stuff */
      x1 = (double) segment->x1 - offset_x;
      y1 = (double) segment->y1 - offset_y;
      x2 = (double) segment->x2 - offset_x; xd = x2-x1;
      y2 = (double) segment->y2 - offset_y; yd = y2-y1;
      length = 2.0; /*sqrt(xd*xd+yd*yd);*/
      le = hor_malloc_type(Hor_Line_End);

      if ( fabs(xd) > fabs(yd) ) /* choose ax + y + cz = 0 */
      {
	 double x0 = (x1+x2)/2.0, d = xd/2.0, a = -yd/xd, c = -(a*x1+y1)/zs;

	 feature->z    = hor_mats_fill ( 2, 1, a, c );
	 feature->Rinv = hor_mats_fill ( 2, 2, x0*x0+2.0*d*d/3.0, x0*zs,
					       x0*zs,             zs*zs );
	 hor_matq_scale ( feature->Rinv, length/s2/(1.0+a*a) );
	 feature->z_type = HOR_LINE_AX_Y_CZ;
	 le->xy1 = x1; le->xy2 = x2;
      }
      else /* choose x + by + cz = 0 */
      {
	 double y0 = (y1+y2)/2.0, d = yd/2.0, b = -xd/yd, c = -(b*y1+x1)/zs;

	 feature->z    = hor_mats_fill ( 2, 1, b, c );
	 feature->Rinv = hor_mats_fill ( 2, 2, y0*y0+2.0*d*d/3.0, y0*zs,
					       y0*zs,             zs*zs );
	 hor_matq_scale ( feature->Rinv, length/s2/(1.0+b*b) );
	 feature->z_type = HOR_LINE_X_BY_CZ;
	 le->xy1 = y1; le->xy2 = y2;
      }

      feature->zdata = (void *) le;
   }

   return feature_map;
}

void hor_traj_coarse_position ( int traj_type, void *traj_last,
			        int *c, int *r )
{
   switch ( traj_type )
   {
      case HOR_CM_BOG_STANDARD:
      case HOR_CM_BRAIN_DEAD:
      case HOR_CM_FMATX:
      {
	 Hor_Traj_Point *point = (Hor_Traj_Point *) traj_last;

	 *c = (int) point->cf;
	 *r = (int) point->rf;
      }
      break;

      case HOR_LM_BOG_STANDARD:
      {
	 Hor_Traj_Line *line = (Hor_Traj_Line *) traj_last;

	 *c = (int) (0.5F*(line->c1f+line->c2f));
	 *r = (int) (0.5F*(line->r1f+line->r2f));
      }
      break;

      default:
      hor_error ( "illegal trajectory type (hor_traj_coarse_position)",
		  HOR_FATAL );
      break;
   }
}

void hor_traj_compatibility ( Hor_Trajectory_Map *traj_map,
			      int traj_type,
			      Hor_Coarse_Feature_Map *feature_map,
			      void (*pos_func)(int type, void *last,
					       int *c, int *r),
			      double (*difference_func)(Hor_Trajectory *traj,
							void *feature,
							void *params),
			      double difference_thres, void *params,
			      Hor_Pot_Match *pot_match, int *start,
			      int max_pot_matches )
{
   Hor_List list;
   int c0    = feature_map->c0,    r0    = feature_map->r0, c, r, i, n;
   int csize = feature_map->csize, rsize = feature_map->rsize;
   int cols  = feature_map->cols,  rows  = feature_map->rows, total = *start;
   Hor_Feature_Def **ptr;
   double difference;

   for ( list = traj_map->traj_list; list != NULL; list = list->next )
   {
      Hor_Trajectory *traj = (Hor_Trajectory *) list->contents;

      if ( traj->type != traj_type ) continue;

      /* generate feature position for coarse coding */
      pos_func ( traj_type, traj->last->contents, &c, &r );
      c = (c - c0)/csize; r = (r - r0)/rsize;

      if ( c < 0 || r < 0 || c >= cols || r >= rows ) continue;
      n = feature_map->no_features[r][c];
      for ( i = 0, ptr = feature_map->feature[r][c]; i < n; i++, ptr++ )
	 if ( (difference = difference_func ( traj, (*ptr)->feature, params ))
	      < difference_thres )
	 {
	    if ( total >= max_pot_matches )
	    {
	       hor_warning ( "too many potential matches" );
	       *start = total;
	       return;
	    }

	    pot_match[total].traj         = traj;
	    pot_match[total].feature      = *ptr;
	    pot_match[total].difference   = difference;
	    total++;
	 }
   }

   *start = total;
}

void hor_select_unique ( Hor_Pot_Match *pot_match, int no_pot_matches,
			 int iterations )
{
   Hor_Pot_Match   *pm;
   Hor_Trajectory  *traj;
   int              i, it;
   double           min_difference;
   Hor_Feature_Def *best_feature;

   /* set status of trajectories and features to HOR_UNMATCHED */
   for ( i = 0, pm = pot_match; i < no_pot_matches; i++, pm++ )
   {
      pm->traj->status = HOR_UNMATCHED;
      pm->feature->status = HOR_UNMATCHED;
   }

   /* here assume that all the potential matches for each trajectory are
      contiguous */
   for ( it = 1; it <= iterations; it++ )
   {
      for ( i = 0, pm = pot_match; i < no_pot_matches; i++, pm++ )
      {
	 traj = pm->traj;
	 if ( traj->status == HOR_MATCHED ) continue;

	 min_difference = HUGE_VAL;
	 best_feature = NULL;
	 for ( ; pm->traj == traj && i < no_pot_matches; i++, pm++ )
	    if ( pm->feature->status != HOR_MATCHED &&
		 pm->difference < min_difference )
	    {
	       best_feature = pm->feature;
	       min_difference = pm->difference;
	    }

	 if ( best_feature != NULL )
	    if ( best_feature->match == NULL )
	    {
	       best_feature->status     = HOR_PROVISIONAL;
	       best_feature->difference = min_difference;
	       best_feature->match      = traj;
	    }
	    else
	       if ( min_difference > best_feature->difference )
	       /* new match is better */
	       {
		  /* reset status of previous best match */
		  best_feature->match->status = HOR_UNMATCHED;

		  traj->status = HOR_PROVISIONAL;
		  best_feature->difference = min_difference;
		  best_feature->match      = traj;
	       }
      }

      /* convert provisional matches to confirmed matches */
      for ( i = 0, pm = pot_match; i < no_pot_matches; i++, pot_match++ )
      {
	 if ( pm->traj->status == HOR_PROVISIONAL )
	    pm->traj->status = HOR_UNMATCHED;

	 if ( pm->feature->status == HOR_PROVISIONAL )
	    pm->feature->status = HOR_UNMATCHED;
      }
   }
}
