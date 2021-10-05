/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/fl_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_image_flow_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_image_flow_colours ( u_long start_colour, u_long line_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_image_flow_defaults ( Hor_FL_Process_Params,
				       float scale, int increment );
Hor_Bool hor_get_image_flow_params ( Hor_FL_Process_Params *,
				     Hor_FL_Output_Params * );
#endif
