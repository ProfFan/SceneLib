/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bog_lm.h */

typedef struct
{
   /* temporary variables used by the matcher */
   double mismatch_strength;
} Hor_Bog_LM_Traj_State;

int  hor_bog_lm_traj_count ( Hor_Trajectory_Map *map );
void hor_bog_match_lines ( Hor_Trajectory_Map   *map,
			   Hor_Line_Segment_Map *new_lines,

			   /* parameters */
			   int   max_dist,
			   float cos_thresh,
			   float size_thresh,
			   int   iterations );

