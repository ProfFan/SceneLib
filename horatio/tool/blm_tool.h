/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/blm_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_bog_line_match_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_bog_line_match_colours ( u_long last_colour, u_long prev_colour,
				      u_long join_colour );

#ifdef _HORATIO_IMPROC_
#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_bog_line_match_defaults ( Hor_BLM_Process_Params,
					   int trajectory_length );
Hor_Bool hor_get_bog_line_match_params ( Hor_BLM_Process_Params *,
					 Hor_LM_Output_Params * );
#endif
#endif
