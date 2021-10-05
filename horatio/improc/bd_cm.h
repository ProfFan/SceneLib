/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bd_cm.h */

#ifdef _HORATIO_IMAGE_

#define HOR_BD_CM_MAX_HISTORY 3

typedef struct
{
   int   n;
   float x[HOR_BD_CM_MAX_HISTORY],
         y[HOR_BD_CM_MAX_HISTORY];
   float corr; /* correlation value between images patches around corners */
   float innovation;
   float covar[4][4];
   float state[4];  
   float speed;
   int   age;

   /* small image patch around the position of the latest corner feature */
   Hor_Sub_Image *patch;
} Hor_BD_CM_Traj_State;

void hor_bd_cm_traj_state_free ( void *state );
void hor_bd_match_corners ( Hor_Trajectory_Map *map,
			    Hor_Corner_Map     *new_corner_map,

			    /* parameters */
			    int   match_window,
			    float corr_thresh );
#endif /* _HORATIO_IMAGE_ */
