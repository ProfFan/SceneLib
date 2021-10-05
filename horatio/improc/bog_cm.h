/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bog_cm.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   /* temporary variables used by the matcher */
   float imdiff_ratio;

   /* small image patch around the position of the latest corner feature */
   Hor_Sub_Image *patch;
} Hor_Bog_CM_Traj_State;

void hor_bog_cm_traj_state_free ( void *state );
void hor_bog_match_corners ( Hor_Trajectory_Map *map,
			     Hor_Corner_Map     *new_corner_map,

			     /* parameters */
			     int   range,
			     float imdiff_ratio_thres,
			     int   iterations );
#endif /* _HORATIO_IMAGE_ */
