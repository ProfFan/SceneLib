/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   void @hor_display_trajectory_map (
*      Hor_Trajectory_Map *map,     (trajectories to be displayed)
*      int                 type,    (type of trajectories to be displayed)
*      void               *params ) (display parameters)
*
*   Trajectory map display function. The type argument specifies which type
*   of trajectory is displayed. If type is < 0, all trajectories are displayed.
********************/
void hor_display_trajectory_map ( Hor_Trajectory_Map *map,
				  int type, void *params )
{
   Hor_List        list;
   Hor_Trajectory *traj;

   for ( list = map->traj_list; list != NULL; list = list->next )
   {
      traj = (Hor_Trajectory *) list->contents;
      if ( type < 0 || traj->type == type )
	 if ( traj->display_func != NULL ) traj->display_func ( traj, params );
   }

   hor_display_flush();
}
