/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/is_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_image_segment_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_image_segment_colours ( u_long display_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_image_segment_defaults ( Hor_IS_Process_Params );
Hor_Bool hor_get_image_segment_params   ( Hor_IS_Process_Params * );
#endif
