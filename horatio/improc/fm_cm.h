/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/fm_cm.h */

typedef struct
{
   /* small image patch around the position of the latest corner feature */
   Hor_Image *patch;
} Hor_Fmatx_CM_Traj_State;

/*******************
*   typedef struct
*   {
*      int   range  ;              (maximum corner displacement between frames)
*      float pixel_diff_threshold; (threshold on image differences)
*   } @Hor_FCM_Process_Params;
*
*   Fmatrix-standard corner matching process parameter structure definitions.
********************/
typedef struct
{
   int   range;                /* maximum corner displacement between frames */
   float pixel_diff_threshold; /* threshold on image differences */
} Hor_FCM_Process_Params;

void hor_fmatx_match_corners (Hor_Trajectory_Map *map,
			      Hor_Corner_Map     *new_corner_map,

			      /* parameters */
			      int   range,
			      int   pixel_diff_threshold);
