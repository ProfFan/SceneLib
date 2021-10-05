/* Copyright 1994 David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/lf_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_line_fit_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_line_fit_colours ( u_long string_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_line_fit_defaults ( Hor_LF_Process_Params );
Hor_Bool hor_get_line_fit_params   ( Hor_LF_Process_Params * );
#ifdef _HORATIO_IMPROC_
void     hor_get_line_fit_colours ( Hor_LI_Output_Params  * );
#endif
#endif
