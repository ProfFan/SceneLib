/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/cl_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_correlation_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_correlation_colours ( u_long thres_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_correlation_defaults ( Hor_CL_Process_Params, float scale );
Hor_Bool hor_get_correlation_params   ( Hor_CL_Process_Params * );
#endif
