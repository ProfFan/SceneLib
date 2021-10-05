/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   void @hor_cm_traj_display ( Hor_Trajectory *traj, void *params )
*
*   Corner trajectory display functions.
********************/
void hor_cm_traj_display ( Hor_Trajectory *traj, void *params )
{
   Hor_CM_Output_Params *cm_params = (Hor_CM_Output_Params *) params;
   Hor_Traj_Point *point;
   Hor_DList       tlist;
   float           prev_c, prev_r;
   int             tcount;

   hor_display_set_line_width ( 0 );
   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( cm_params->trajectory_colour );

   /* only display trajectory if it has more than one point in it */
   if ( traj->length < 2 ) return;

   point = (Hor_Traj_Point *) traj->last->contents;
   prev_c = point->cf;
   prev_r = point->rf;
   for ( tlist = traj->last->prev, tcount = 1;
	 tlist != NULL && tcount < cm_params->trajectory_length;
	 tlist = tlist->prev, tcount++ )
   {
      point = (Hor_Traj_Point *) tlist->contents;
      if ( point != NULL )
      {
	 hor_display_line ( prev_c, prev_r, point->cf, point->rf );
	 prev_c = point->cf;
	 prev_r = point->rf;
      }
   }

   if ( traj->last->contents != NULL )
   {
      hor_display_set_colour ( cm_params->dot_colour );
      point = (Hor_Traj_Point *) traj->last->contents;
      if ( point != NULL )
	 hor_display_fill_circle_actual_size( (int) point->cf,
					      (int) point->rf, 2);
   }
}
