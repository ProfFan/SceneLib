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
*   void @hor_lm_traj_display ( Hor_Trajectory *traj, void *params )
*
*   Line segment trajectory display function. params should be a pointer to
*   a Hor_LM_Output_Params structure.
********************/
void hor_lm_traj_display ( Hor_Trajectory *traj, void *params )
{
   Hor_LM_Output_Params *lm_params = (Hor_LM_Output_Params *) params;
   Hor_Traj_Line *last, *prev;

   /* only display trajectory if it has more than one line in it */
   if ( traj->length < 2 ) return;

   last = (Hor_Traj_Line *) traj->last->contents;
   prev = (Hor_Traj_Line *) traj->last->prev->contents;
   if ( last == NULL || prev == NULL ) return;

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_line_width(2);
   hor_display_set_colour(lm_params->prev_colour);
   hor_display_line ( prev->c1f, prev->r1f, prev->c2f, prev->r2f );
   hor_display_set_colour(lm_params->join_colour);
   hor_display_line ( 0.5F*(prev->c1f+prev->c2f), 0.5F*(prev->r1f+prev->r2f),
		      0.5F*(last->c1f+last->c2f), 0.5F*(last->r1f+last->r2f) );
   hor_display_set_colour(lm_params->last_colour);
   hor_display_line ( last->c1f, last->r1f, last->c2f, last->r2f );

   hor_display_set_line_width(0);
   hor_display_flush();
}
