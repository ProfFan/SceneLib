/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/traject.h */

typedef struct
{
   float cf, rf; /* position relative to the top-left corner of the image */
} Hor_Traj_Point;

typedef struct
{
   float c1f, r1f, c2f, r2f; /* positions of the endpoints of line segments
				relative to the top-left corner of the image */
} Hor_Traj_Line;

/* standard Horatio feature matcher trajectory types */
#define HOR_CM_BOG_STANDARD 0 /* bog-standard corner matcher */
#define HOR_CM_BRAIN_DEAD   1 /* Modified Wang/Brady corner matcher */
#define HOR_CM_FMATX        2 /* Fundamental matrix corner matching */
#define HOR_LM_BOG_STANDARD 3 /* bog-standard line segment matcher */

typedef struct _Hor_Trajectory
{
   Hor_Assoc_Label label; /* identifier for this feature trajectory */
   int             type; /* in case multiple trajectory types are present in
			    the same trajectory map. Current standard
			    Horatio trajectory types are HOR_CM_BOG_STANDARD,
			    HOR_CM_BRAIN_DEAD and HOR_LM_BOG_STANDARD. */
   int             status; /* standard values are HOR_UNMATCHED,
			      HOR_PROVISIONAL and HOR_MATCHED. */
   Hor_DList       first; /* first (earliest) element in a list of
			     Hor_Traj_Point's, Hor_Traj_Line's
			     or other types of feature location */
   Hor_DList       last; /* last element in the same list (latest one to be
			    added) */
   int             length; /* length of the above list */
   int             times_seen; /* number of times feature seen == # non-null
				  elements of list */
   int             max_length; /* maximum length of the above list */
   int             not_seen; /* number of frames since this feature was last
				seen, i.e. number of NULL elements since the
				last non-NULL element */

   /* user-defined fields */
   void  *state; /* trajectory state */
   void (*element_free_func)(void *element);
   void (*state_free_func)(void *state);
   void (*display_func)(struct _Hor_Trajectory *traj, void *params);
} Hor_Trajectory;

void hor_add_trajectory_element ( Hor_Trajectory *traj, void *element );

/* standard Horatio trajectory map state types */
#define HOR_TRAJ_MAP_NO_STATE 0

typedef struct
{
   Hor_List traj_list; /* list of trajectories (Hor_Trajectory *`s) */
   int      ntrajs;    /* length of traj_list, i.e. no. of trajectories */

   /* user-defined fields */
   int    type; /* e.g. HOR_TRAJ_MAP_NO_STATE */
   void  *state;
   void (*state_free_func)(void *state);
} Hor_Trajectory_Map;

/* standard feature attribute types */
#define HOR_NO_ATTRIB 0 /* no attribute, attrib field set to NULL */

Hor_Trajectory_Map *hor_alloc_trajectory_map (
		       int    type,
		       void  *state,
		       void (*state_free_func)(void *state) );

void hor_free_trajectory_map ( Hor_Trajectory_Map *map );

Hor_Trajectory *hor_add_new_trajectory (
			      Hor_Trajectory_Map *map,
			      int    type,
			      void  *first_element,
			      int    max_length,
			      void  *state,
			      void (*element_free_func)(void *element),
			      void (*state_free_func)(void *state),
			      void (*display_func)(Hor_Trajectory *traj,
						   void *params) );
void hor_delete_old_trajectories ( Hor_Trajectory_Map *map,
				   Hor_Bool (*test_func)(Hor_Trajectory *traj,
							 void *data),
				   void *data );
int hor_count_trajectories ( Hor_Trajectory_Map *trajectory_map, int type );
Hor_Trajectory *hor_find_trajectory ( Hor_Trajectory_Map *map,
				      Hor_Assoc_Label label );
void hor_reset_trajectory_status ( Hor_Trajectory_Map *map,
				   int type, int status );

/* standard Horatio feature status values */
#define HOR_UNMATCHED   0 /* for an unmatched feature */
#define HOR_PROVISIONAL 1 /* for a provisionally matched feature */
#define HOR_MATCHED     2 /* for a matched feature */
