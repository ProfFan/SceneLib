/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/bcm_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_bog_corner_match_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_bog_corner_match_colours ( u_long trajectory_colour,
				        u_long dot_colour,
				        u_long old_traj_colour );

#ifdef _HORATIO_IMPROC_
#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_bog_corner_match_defaults ( Hor_BCM_Process_Params,
					     int trajectory_length );
Hor_Bool hor_get_bog_corner_match_params ( Hor_BCM_Process_Params *,
					   Hor_CM_Output_Params * );
#endif
#endif
