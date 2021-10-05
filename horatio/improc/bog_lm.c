/* Copyright 1993 Ian D. Reid (ian@robots.oxford.ac.uk)
              and Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

#define MAX_PERCENT 20.0
#define SQR(X)   ((X)*(X))

static Hor_Bool bog_lm_test_traj ( Hor_Trajectory *traj, void *data )
{
   if ( traj->type != HOR_LM_BOG_STANDARD || traj->status != HOR_UNMATCHED )
      return HOR_TRUE;

   return HOR_FALSE;
}

/*******************
*   void @hor_bog_match_lines (
*      Hor_Trajectory_Map   *map,       (match map)
*      Hor_Line_Segment_Map *new_lines, (latest corner map)
*
*      (parameters:)
*      int   dist_thresh,  (distance threshold between two matched lines)
*      float cos_thresh,   (orientation cosine threshold between two lines)
*      float size_thresh,  (threshold on line length ratio)
*      int   iterations )  (number of winner-take-all matching iterations)
*
*   Matches line segments from two images using a bog-standard algorithm.
********************/
void hor_bog_match_lines ( Hor_Trajectory_Map   *map,
			   Hor_Line_Segment_Map *new_lines,

			   /* parameters */
			   int   dist_thresh,
			   float cos_thresh,
			   float size_thresh,
			   int   iterations )
{
    int                    i, best_i = -1, n, it;
    Hor_Trajectory        *traj, *new_traj, **trajectories;
    Hor_Traj_Line         *line;
    Hor_Bog_LM_Traj_State *state, *new_state;
    Hor_Line_Segment      *segment, *best_segment;
    Hor_List               tlist, llist;
    double                 mismatch_strength, mismatch_strength_min;
    double                 sx1, sy1, sx2, sy2, lx1, ly1, lx2, ly2,
                           tmp, a1, a2, b1, b2, c, s1, s2, max_dist;
    Hor_List               new_line_list = new_lines->line_list;

    n = new_lines->nlines;
    trajectories = hor_malloc_ntype ( Hor_Trajectory *, n );
    for ( i = 0; i < n; i++ ) trajectories[i] = NULL;

    for ( tlist = map->traj_list, i = 0; tlist != NULL;
	  tlist = tlist->next, i++) {
       traj = (Hor_Trajectory *) tlist->contents;
       if ( traj->type != HOR_LM_BOG_STANDARD ) continue;

       traj->status = HOR_UNMATCHED;
    }

    for ( it = 1; it <= iterations; it++ )
    {
       for ( tlist = map->traj_list; tlist != NULL; tlist = tlist->next )
       {
	  traj = (Hor_Trajectory *) tlist->contents;
	  if ( traj->type != HOR_LM_BOG_STANDARD ) continue;

	  if ( traj->status != HOR_UNMATCHED ) continue;

	  line = (Hor_Traj_Line *) traj->last->contents;
	  state = (Hor_Bog_LM_Traj_State *) traj->state;

	  lx1 = (double) line->c1f; ly1 = (double) line->r1f;
	  lx2 = (double) line->c2f; ly2 = (double) line->r2f;

	  mismatch_strength_min = 1.0e+50;
	  best_segment = NULL;
	  for ( llist = new_line_list, i = 0; llist != NULL;
	        llist = llist->next, i++ ) {
	     segment = (Hor_Line_Segment *) llist->contents;
	     if ( segment->status == HOR_MATCHED ) continue;

	     sx1 = segment->x1; sy1 = segment->y1;
	     sx2 = segment->x2; sy2 = segment->y2;
	     if ( segment->dir == -1 ) { tmp=sx1; sx1=sx2; sx2=tmp;
					 tmp=sy1; sy1=sy2; sy2=tmp; }

	     a1 = sqrt(SQR(lx2-lx1) + SQR(ly2-ly1));
	     a2 = sqrt(SQR(sx2-sx1) + SQR(sy2-sy1));
	     max_dist = hor_max3 ( dist_thresh, a1*MAX_PERCENT/100.0,
				                a2*MAX_PERCENT/100.0 );

	     /* simple bounds check to eliminate lots of potential candidates*/
	     if ( hor_max2(lx1,lx2)+max_dist < hor_min2(sx1,sx2) ||
		  hor_min2(lx1,lx2)-max_dist > hor_max2(sx1,sx2) ||
		  hor_max2(ly1,ly2)+max_dist < hor_min2(sy1,sy2) ||
		  hor_min2(ly1,ly2)-max_dist > hor_max2(sy1,sy2))
		continue;

	     /* test perpendicular distance between lines */
	     if (fabs(((lx2 - lx1)*sy1 + (ly1 - ly2)*sx1 -
		       (lx2 - lx1)*ly1 - (ly1 - ly2)*lx1)/a1) > max_dist)
	        continue;
	     if (fabs(((sx2 - sx1)*ly1 + (sy1 - sy2)*lx1 -
		       (sx2 - sx1)*sy1 - (sy1 - sy2)*sx1)/a1) > max_dist)
	        continue;

	     /* test angle between lines */
	     if ( ((sx2-sx1)*(lx2-lx1) + (sy2-sy1)*(ly2-ly1))/
		   sqrt(SQR(sx2-sx1)+SQR(sy2-sy1))/
		   sqrt(SQR(lx2-lx1)+SQR(ly2-ly1)) <= cos_thresh ) continue;

	     /* test relative lengths */
	     tmp = sqrt(SQR(lx2 - lx1) + SQR(ly2 - ly1)) /
		   sqrt(SQR(sx2 - sx1) + SQR(sy2 - sy1));
	     if (tmp>1) tmp = 1/tmp;
	     if (tmp < size_thresh/100.0) continue;

	     b1 = sqrt(SQR(sx1-lx1)+SQR(sy1-ly1));
	     b2 = sqrt(SQR(sx2-lx2)+SQR(sy2-ly2));
	     c = sqrt(SQR(sx1-lx2)+SQR(sy1-ly2));
	     s1 = 0.5*(a1+b1+c);
	     s2 = 0.5*(a2+b2+c);
	     mismatch_strength = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c)) +
		                 sqrt(s2*(s2-a2)*(s2-b2)*(s2-c));
	     if ( mismatch_strength < mismatch_strength_min )
	     {
		mismatch_strength_min = mismatch_strength;
		best_segment = segment;
		best_i = i;
	     }
	  }

	  if ( best_segment != NULL )
	     if ( best_segment->status == HOR_UNMATCHED )
	     {
		traj->status = HOR_PROVISIONAL;
		state->mismatch_strength = mismatch_strength_min;
		trajectories[best_i] = traj;
		best_segment->status = HOR_PROVISIONAL;
	     }
	     else /* best_segment->status == HOR_PROVISIONAL */
	     {
		new_traj = trajectories[best_i];
		new_state = (Hor_Bog_LM_Traj_State *) new_traj->state;
		if ( mismatch_strength_min < new_state->mismatch_strength )
		/* new match is better */
		{
		   new_traj->status = HOR_UNMATCHED;
		   traj->status = HOR_PROVISIONAL;
		   state->mismatch_strength = mismatch_strength_min;
		   trajectories[best_i] = traj;
		}
	     }
       }

       for ( tlist = map->traj_list; tlist != NULL; tlist = tlist->next )
       {
	  traj = (Hor_Trajectory *) tlist->contents;
	  if ( traj->status == HOR_PROVISIONAL ) traj->status = HOR_MATCHED;
       }

       for ( llist = new_line_list, i = 0; llist != NULL;
	     llist = llist->next, i++ ) {
	  segment = (Hor_Line_Segment *) llist->contents;
	  if ( segment->status == HOR_PROVISIONAL )
	  {
	     traj = trajectories[i];
	     state = (Hor_Bog_LM_Traj_State *) traj->state;
	     line = hor_malloc_type(Hor_Traj_Line);
	     if ( segment->dir == 1 )
	     {
		line->c1f = segment->x1; line->r1f = segment->y1;
		line->c2f = segment->x2; line->r2f = segment->y2;
	     }
	     else
	     {
		line->c1f = segment->x2; line->r1f = segment->y2;
		line->c2f = segment->x1; line->r2f = segment->y1;
	     }

	     hor_add_trajectory_element ( traj, (void *) line );
	     segment->status = HOR_MATCHED;
	  }
       }
    }

    /* delete trajectories of unmatched line segments */
    hor_delete_old_trajectories ( map, bog_lm_test_traj, NULL );

    /* create trajectories for new line segments */
    for ( llist = new_line_list, i = 0; llist != NULL;
	  llist = llist->next, i++ ) {
       if ( (segment = (Hor_Line_Segment *) llist->contents) != NULL &&
	    segment->status == HOR_UNMATCHED )
       {
	  line = hor_malloc_type(Hor_Traj_Line);
	  if ( segment->dir == 1 )
	  {
	     line->c1f = segment->x1; line->r1f = segment->y1;
	     line->c2f = segment->x2; line->r2f = segment->y2;
	  }
	  else
	  {
	     line->c1f = segment->x2; line->r1f = segment->y2;
	     line->c2f = segment->x1; line->r2f = segment->y1;
	  }

	  new_state = hor_malloc_type(Hor_Bog_LM_Traj_State);
	  new_state->mismatch_strength = 1.0e+50;
	  hor_add_new_trajectory ( map, HOR_LM_BOG_STANDARD,
				   (void *) line, 100000,
				   (void *) new_state, hor_free_func,
				   hor_free_func, hor_lm_traj_display );
       }
    }

    hor_free ( (void *) trajectories );
}
