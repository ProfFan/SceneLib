/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/improc.h"

static Hor_Assoc_Label next_label = HOR_ASSOC_START;

/*******************
*   Hor_Trajectory *@hor_add_new_trajectory (
*      Hor_Trajectory_Map *map,
*      int                 type,          (type of new trajectory, e.g.
*                                          HOR_CM_BOG_STANDARD for the
*                                          bog-standard corner matcher)
*      void               *first_element, (first element to go into the
*                                          trajectory)
*      int                 max_length,    (maximum length of the trajectory)
*      void               *state,         (user-defined state variables)
*      void (*element_free_func)(void *element),
*      void (*state_free_func)(void *state),
*      void (*display_func)(Hor_Trajectory *traj, void *params) )
*
*   Allocates a single trajectory structure, initialising it with the provided
*   arguments, and adds it to the traj_list field of the trajectory map.
*   If first_element is NULL, the trajectory list is set to NULL.
*   A unique label for the trajectory is provided automatically.
*   If max_length is < 0, the whole trajectory will be retained, otherwise
*   the oldest element of the trajectory is removed when
*   hor_add_trajectory_element() is called enough times to exceed the
*   max_length limit. The new trajectory is returned.
*
*   Function argument meanings:
*
*   element_free_func(): frees an element of the trajectory list.
*   state_free_func():   frees the user-defined state associated with
*                        a trajectory.
*   display_func():      displays a single trajectory.
********************/
Hor_Trajectory *hor_add_new_trajectory (
		     Hor_Trajectory_Map *map,
		     int                 type,
		     void               *first_element,
		     int                 max_length,
		     void               *state,
		     void (*element_free_func)(void *element),
		     void (*state_free_func)(void *state),
		     void (*display_func)(Hor_Trajectory *traj, void *params) )
{
   Hor_Trajectory *traj = hor_malloc_type(Hor_Trajectory);

   traj->label = next_label++;
   traj->type  = type;
   traj->status = HOR_UNMATCHED;
   if ( first_element == NULL )
   {
      traj->first = traj->last = NULL;
      traj->length = 0;
      traj->times_seen = 0;
   }
   else
   {
      traj->first = traj->last = hor_dmake_straight ( first_element );
      traj->length = 1;
      traj->times_seen = 1;
   }

   traj->max_length        = max_length;
   traj->not_seen          = 0;
   traj->state             = state;
   traj->element_free_func = element_free_func;
   traj->state_free_func   = state_free_func;
   traj->display_func      = display_func;
   map->traj_list = hor_insert ( map->traj_list, (void *) traj );
   map->ntrajs++;
   return traj;
}

static void free_trajectory ( void *data )
{
   Hor_Trajectory *traj = (Hor_Trajectory *) data;

   hor_dfree_list ( traj->first, traj->element_free_func );
   if ( traj->state_free_func != NULL )
      traj->state_free_func ( traj->state );

   hor_free ( data );
}

/*******************
*   void @hor_delete_old_trajectories (
*      Hor_Trajectory_Map *map,
*      Hor_Bool (*test_func)(Hor_Trajectory *traj, void *data),
*      void *data )
*
*   Checks each trajectory using the provided boolean-valued function
*   test_func(). If it returns HOR_TRUE, the trajectory is retained in the
*   trajectory map, otherwise it is removed. The data pointer allows user
*   data to be passed to test_func().
********************/
void hor_delete_old_trajectories ( Hor_Trajectory_Map *map,
				   Hor_Bool (*test_func)(Hor_Trajectory *traj,
							 void *data),
				   void *data )
{
   Hor_Trajectory *traj;
   Hor_List       *list_ptr;

   /* transfer unmatched trajectories to the old_list field of map */
   list_ptr = &map->traj_list;
   while ( *list_ptr != NULL )
   {
      traj = (Hor_Trajectory *) (*list_ptr)->contents;
      if ( test_func ( traj, data ) ) /* retain this trajectory */
	 list_ptr = &(*list_ptr)->next;
      else /* HOR_FALSE returned: delete this trajectory */
      {
	 hor_delete_first ( list_ptr, free_trajectory );
	 map->ntrajs--;
      }
   }
}

/*******************
*   void @hor_add_trajectory_element ( Hor_Trajectory *traj, void *element )
*
*   Adds a new element to the trajectory's element list. A NULL value for
*   element corresponds to the feature not being observed, and the not_seen
*   counter is incremented.
********************/
void hor_add_trajectory_element ( Hor_Trajectory *traj, void *element )
{
   int excess, i;

   if ( traj->last == NULL )
      traj->first = traj->last = hor_dmake_straight ( element );
   else
      traj->last = hor_dinsert_after ( traj->last, element );

   traj->length++;
   if ( element == NULL ) traj->not_seen++;
   else {
      traj->not_seen = 0;
      traj->times_seen++;
   }

   if ( traj->max_length < 0 ) return;

   /* limit length of trajectory to maximum length */
   excess = traj->length - traj->max_length;

   if ( excess <= 0 || traj->first == NULL ) return;

   for ( i = 0; i < excess; i++ ) traj->first = traj->first->next;

   if ( traj->first == NULL ) /* remove whole trajectory, max_length = 0 */
   {
      hor_dfree_list ( traj->last, traj->element_free_func );
      traj->last = NULL;
   }
   else
      hor_dfree_list_before ( traj->first, traj->element_free_func );

   traj->length = traj->max_length;
}
   
/*******************
*   Hor_Trajectory_Map *@hor_alloc_trajectory_map (
*      int    type,       (type of state variables)
*      void  *state,      (initial state variables)
*      void (*state_free_func)(int type, void *state),
*      void (*traj_element_free_func)(int type, void *element),
*      void (*traj_state_free_func)(int type, void *state),
*      void (*traj_display_func)(Hor_Trajectory *traj,
*                                void *params) )
*
*   void @hor_free_trajectory ( void *trajectory )
*   void @hor_free_trajectory_map ( Hor_Trajectory_Map *map )
*
*   Allocate/free trajectory map structure. The type argument to
*   hor_alloc_trajectory_map() can be HOR_TRAJ_MAP_NO_STATE, in which case
*   state and state_free_func_free_func should be passed as NULL.
*   hor_free_trajectory() frees a
*   single trajectory (Hor_Trajectory *) casted to a (void *) so that it can
*   be used as a argument to hor_free_list(), and other list node free
*   functions.
*
*   Function arguments to hor_alloc_trajectory_map:
*
*   state_free_func():        frees the user-defined trajectory map state
*                             variables.
********************/
Hor_Trajectory_Map *hor_alloc_trajectory_map (
			 int    type,
			 void  *state,
			 void (*state_free_func)(void *state) )
{
   Hor_Trajectory_Map *map = hor_malloc_type(Hor_Trajectory_Map);

   map->traj_list = NULL;
   map->ntrajs    = 0;

   /* set user-defined fields */
   map->type            = type;
   map->state           = state;
   map->state_free_func = state_free_func;
   return map;
}

void hor_free_trajectory_map ( Hor_Trajectory_Map *map )
{
   hor_free_list ( map->traj_list, free_trajectory );
   if ( map->state_free_func != NULL ) map->state_free_func ( map->state );

   hor_free ( (void *) map );
}

/*******************
*   Hor_Trajectory *@hor_find_trajectory ( Hor_Trajectory_Map *map,
*                                         Hor_Assoc_Label label )
*
*   Returns the trajectory with the given label.
********************/
Hor_Trajectory *hor_find_trajectory ( Hor_Trajectory_Map *map,
				      Hor_Assoc_Label label )
{
   Hor_List        list;
   Hor_Trajectory *traj;

   for ( list = map->traj_list; list != NULL; list = list->next )
   {
      traj = (Hor_Trajectory *) list->contents;
      if ( traj->label == label ) return traj;
   }

   return NULL;
}

/*******************
*   int @hor_count_trajectories ( Hor_Trajectory_Map *map, int type )
*
*   Count the number of trajectories in trajectory map with given trajectory
*   type.
********************/
int hor_count_trajectories ( Hor_Trajectory_Map *map, int type )
{
   Hor_List        list;
   Hor_Trajectory *traj;
   int             count = 0;

   for ( list = map->traj_list; list != NULL; list = list->next )
   {
      traj = (Hor_Trajectory *) list->contents;
      if ( traj->type == type ) count++;
   }

   return count;
}

/*******************
*   void @hor_reset_trajectory_status ( Hor_Trajectory_Map *map,
*                                      int type, int status )
*
*   Resets the status fields of the trajectories of given type to the
*   given value.
********************/
void hor_reset_trajectory_status ( Hor_Trajectory_Map *map,
				   int type, int status )
{
   Hor_List        list;
   Hor_Trajectory *traj;

   for ( list = map->traj_list; list != NULL; list = list->next )
   {
      traj = (Hor_Trajectory *) list->contents;
      if ( traj->type == type ) traj->status = status;
   }
}
