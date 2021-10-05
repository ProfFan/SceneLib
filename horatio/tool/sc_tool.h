/* Copyright 1993 HOR_CHARles Wiles (csw@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/sc_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_smith_corner_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_smith_corner_colours ( u_long corner_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_smith_corner_defaults ( Hor_SC_Process_Params );
Hor_Bool hor_get_smith_corner_params   ( Hor_SC_Process_Params * );
#ifdef _HORATIO_IMPROC_
void     hor_get_smith_corner_colours ( Hor_CO_Output_Params  * );
#endif
#endif
