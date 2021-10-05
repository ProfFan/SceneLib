/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

static float image_diff_ratio ( Hor_Image *image1, int c1, int r1,
			        Hor_Image *image2, int r2, int c2,
			        int patch_size )
{
   int i, j, half_size = patch_size/2; /* patch_size assumed to be odd */

   c1 += image1->width/2; r1 += image1->height/2;
   c2 += image2->width/2; r2 += image2->height/2;
   if ( c1-half_size < 0 || c1+half_size >= image1->width ||
        r1-half_size < 0 || r1+half_size >= image1->height ||
        c2-half_size < 0 || c2+half_size >= image2->width ||
        r2-half_size < 0 || r2+half_size >= image2->height )
   {
      hor_warning ( "illegal parameters (image_diff_ratio)" );
      return 100000000.0F;
   }

   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char **imarr1 = image1->array.uc;
	 u_char **imarr2 = image2->array.uc;
	 int      total1 = 0, total2 = 0, total_sqr = 0, total_diff = 0;
	 int      n = patch_size*patch_size, pix1, pix2;

	 for ( i = -half_size; i < half_size; i++ )
	    for ( j = -half_size; j < half_size; j++ )
	    {
	       total1 += (pix1 = imarr1[r1+i][c1+j]);
	       total2 += (pix2 = imarr2[r2+i][c2+j]);
	       total_sqr += pix1*pix1 + pix2*pix2;
	       total_diff += abs ( pix2-pix1 );
	    }

	 return ( (float) total_diff /
		  sqrt((float) (n*total_sqr - total1*total1 - total2*total2)));
      }
      break;

      case HOR_INT:
      {
	 int **imarr1 = image1->array.i;
	 int **imarr2 = image2->array.i;
	 int   total1 = 0, total2 = 0, total_sqr = 0, total_diff = 0;
	 int   n = patch_size*patch_size, pix1, pix2;

	 for ( i = -half_size; i < half_size; i++ )
	    for ( j = -half_size; j < half_size; j++ )
	    {
	       total1 += (pix1 = imarr1[r1+i][c1+j]);
	       total2 += (pix2 = imarr2[r2+i][c2+j]);
	       total_sqr += pix1*pix1 + pix2*pix2;
	       total_diff += abs ( pix2-pix1 );
	    }

	 return ( (float) total_diff /
		  sqrt((float) (n*total_sqr - total1*total1 - total2*total2)));
      }
      break;

      case HOR_FLOAT:
      {
	 float **imarr1 = image1->array.f, **imarr2 = image2->array.f;
	 float   total1=0.0F, total2=0.0F, total_sqr=0.0F,total_diff=0.0;
	 float   n = (float) (patch_size*patch_size), pix1, pix2;

	 for ( i = -half_size; i < half_size; i++ )
	    for ( j = -half_size; j < half_size; j++ )
	    {
	       total1 += (pix1 = imarr1[r1+i][c1+j]);
	       total2 += (pix2 = imarr2[r2+i][c2+j]);
	       total_sqr += pix1*pix1 + pix2*pix2;
	       total_diff += fabs ( pix2-pix1 );
	    }

	 return (total_diff/sqrt(n*total_sqr - total1*total1 - total2*total2));
      }
      break;

      default:
      hor_error ( "illegal image type (image_diff)", HOR_FATAL );
      break;
   }

   return -1.0F;
}

/*******************
*   void @hor_bog_cm_traj_state_free ( void *state )
*
*   Bog-standard corner matcher trajectory state free function.
********************/
void hor_bog_cm_traj_state_free ( void *state )
{
   hor_free_sub_image ( ((Hor_Bog_CM_Traj_State *) state)->patch );
   hor_free ( state );
}

static void bog_match_corners ( Hor_Trajectory_Map *map,
			        Hor_Corner_Map     *new_corner_map,

			        /* parameters */
			        int   range,
			        float imdiff_ratio_thres,
			        int   iterations )
{
   int c0 = new_corner_map->c0, width  = new_corner_map->width;
   int r0 = new_corner_map->r0, height = new_corner_map->height;
   int it, c, r;
   Hor_Trajectory *traj, *new_traj;
   Hor_Bog_CM_Traj_State *state, *new_state;
   Hor_Traj_Point *point;
   Hor_Corner ***corners = new_corner_map->corners, *corner;
   Hor_List      list;
   Hor_Image        *timage;
   Hor_Trajectory ***trajectories;
   Hor_Impixel       null_pixel;

   /* allocate 2D array of new corner trajectory pointers */
   timage = hor_alloc_image ( new_corner_map->width, new_corner_map->height,
			      HOR_POINTER, NULL );
   null_pixel.p = NULL;
   hor_fill_image_with_constant ( timage, null_pixel );
   trajectories = (Hor_Trajectory ***) timage->array.p;

   /* reset status of every trajectory state to HOR_UNMATCHED */
   for ( list = map->traj_list; list != NULL; list = list->next )
   {
      traj = (Hor_Trajectory *) list->contents;
      if ( traj->type != HOR_CM_BOG_STANDARD ) continue;

      traj->status = HOR_UNMATCHED;
   }

   for ( it = 1; it <= iterations; it++ )
   {
      for ( list = map->traj_list; list != NULL; list = list->next )
      {
	 traj = (Hor_Trajectory *) list->contents;
	 if ( traj->type != HOR_CM_BOG_STANDARD ) continue;

	 if ( traj->status == HOR_UNMATCHED )
	 {
	    float best_ratio = imdiff_ratio_thres + 1.0F;
	    int p, q, u, v, best_c = -1, best_r = -1;

	    point = (Hor_Traj_Point *) traj->last->contents;
	    state = (Hor_Bog_CM_Traj_State *) traj->state;

	    c = (int) point->cf;
	    r = (int) point->rf;

	    /* search for a match in the vicinity of the old match */
	    for ( p = -range; p <= range; p++ )
	       for ( q = -range; q <= range; q++ )
	       {
		  /* calculate position relative to new corner map */
		  u = r-r0+p; v = c-c0+q;

		  if ( u >= 0 && u < height && v >= 0 && v < width )
		     if ( corners[u][v] != NULL &&
			  corners[u][v]->status != HOR_MATCHED )
		     {
			float imdiff_ratio;

			corner = corners[u][v];
			if ((imdiff_ratio =
			     image_diff_ratio (
				  &(state->patch->image),  0, 0,
				  &(corner->patch->image), 0, 0,
				  corner->patch->image.width ) ) < best_ratio )
			{
			   best_ratio = imdiff_ratio;
			   best_c = c+q;
			   best_r = r+p; /* position in new image frame */
			}
		     }
	       }

	    if ( best_ratio < imdiff_ratio_thres )
	       if ( corners[best_r-r0][best_c-c0]->status == HOR_UNMATCHED )
	       {
		  traj->status = HOR_PROVISIONAL;
		  state->imdiff_ratio = best_ratio;
		  trajectories[best_r-r0][best_c-c0] = traj;
		  corners[best_r-r0][best_c-c0]->status = HOR_PROVISIONAL;
	       }
	       else /*corners[best_r-r0][best_c-c0]->status ==HOR_PROVISIONAL*/
	       {
		  new_traj = trajectories[best_r-r0][best_c-c0];
		  new_state = (Hor_Bog_CM_Traj_State *) new_traj->state;

		  if ( best_ratio < new_state->imdiff_ratio )
		  /* new match is better */
		  {
		     new_traj->status = HOR_UNMATCHED;
		     traj->status = HOR_PROVISIONAL;
		     state->imdiff_ratio = best_ratio;
		     trajectories[best_r-r0][best_c-c0] = traj;
		  }
	       }
	 }
      }

      for ( list = map->traj_list; list != NULL; list = list->next )
      {
	 traj = (Hor_Trajectory *) list->contents;
	 if ( traj->status == HOR_PROVISIONAL ) traj->status = HOR_MATCHED;
      }

      for ( r = 0; r < height; r++ )
	 for ( c = 0; c < width; c++ )
	    if ( (corner = corners[r][c]) != NULL &&
		 corner->status == HOR_PROVISIONAL )
	    {
	       traj = trajectories[r][c];
	       state = (Hor_Bog_CM_Traj_State *) traj->state;
	       point = hor_malloc_type(Hor_Traj_Point);
	       point->cf = corner->cf;
	       point->rf = corner->rf;
	       hor_add_trajectory_element ( traj, (void *) point );
	       hor_convert_image_data ( &corner->patch->image,
				        &state->patch->image );
	       corner->status = HOR_MATCHED;
	    }
   }

   hor_free_image ( timage );
}

static void bog_init_new_corners ( Hor_Trajectory_Map *map,
				   Hor_Corner_Map     *new_corner_map )
{
   int c, r, width = new_corner_map->width, height = new_corner_map->height;
   Hor_Corner ***corners = new_corner_map->corners, *corner;
   Hor_Bog_CM_Traj_State *new_state;
   Hor_Traj_Point        *new_point;

   for ( r = 0; r < height; r++ )
      for ( c = 0; c < width; c++ )
	 if ( (corner = corners[r][c]) != NULL &&
	      corner->status == HOR_UNMATCHED )
	 {
	    new_point = hor_malloc_type(Hor_Traj_Point);
	    new_point->cf = corner->cf;
	    new_point->rf = corner->rf;
	    new_state = hor_malloc_type(Hor_Bog_CM_Traj_State);
	    new_state->imdiff_ratio = -1.0F;
	    new_state->patch = hor_copy_sub_image ( corner->patch );
	    hor_add_new_trajectory ( map, HOR_CM_BOG_STANDARD,
				     (void *) new_point, 100000,
				     (void *) new_state, hor_free_func,
				     hor_bog_cm_traj_state_free,
				     hor_cm_traj_display );
	 }
}

static Hor_Bool bog_cm_test_traj ( Hor_Trajectory *traj, void *data )
{
   if ( traj->type != HOR_CM_BOG_STANDARD || traj->status != HOR_UNMATCHED )
      return HOR_TRUE;

   return HOR_FALSE;
}
   
/*******************
*   void @hor_bog_match_corners (
*      Hor_Trajectory_Map *map,            (match map)
*      Hor_Corner_Map     *new_corner_map, (latest corner map)
*
*      (parameters:)
*      int   range,              (maximum corner motion between images)
*      float imdiff_ratio_thres, (threshold on image difference ratio
*                                 between windows around potentially matched
*                                 corners)
*      int   iterations )        (no. of winner-take-all iterations)
*
*   Matches corners from two images using a bog-standard algorithm.
********************/
void hor_bog_match_corners ( Hor_Trajectory_Map *map,
			     Hor_Corner_Map     *new_corner_map,

			     /* parameters */
			     int   range,
			     float imdiff_ratio_thres,
			     int   iterations )
{
   if ( range < 0 || imdiff_ratio_thres < 0 )
   {
      hor_errno = HOR_IMPROC_ILLEGAL_PARAMETERS;
      return;
   }

   /* match new corners to old corner trajectories */
   bog_match_corners ( map, new_corner_map,
		       range, imdiff_ratio_thres, iterations );

   /* discard old corner trajectories */
   hor_delete_old_trajectories ( map, bog_cm_test_traj, NULL );

   /* create corner matches for new corner features */
   bog_init_new_corners ( map, new_corner_map );
}
