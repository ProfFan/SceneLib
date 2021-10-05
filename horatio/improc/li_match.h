/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/li_match.h */

typedef struct
{
   int    trajectory_length;
   u_long last_colour, prev_colour, join_colour;
} Hor_LM_Output_Params;

void hor_lm_traj_display ( Hor_Trajectory *traj, void *params );
/* params should be a pointer to a Hor_LM_Display_Params structure */
