/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/co_match.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   int    trajectory_length;
   u_long trajectory_colour;
   u_long dot_colour;
} Hor_CM_Output_Params;

void hor_cm_traj_display ( Hor_Trajectory *traj, void *params );
/* params should be a pointer to a Hor_CM_Display_Params structure */

#endif /* _HORATIO_IMAGE_ */
