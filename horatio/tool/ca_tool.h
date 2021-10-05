/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/ca_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_canny_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_canny_colours (u_long string_colour,    u_long discard_colour,
			    u_long left_term_colour, u_long right_term_colour);

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_canny_defaults ( Hor_CA_Process_Params );
Hor_Bool hor_get_canny_params   ( Hor_CA_Process_Params * );
#ifdef _HORATIO_IMPROC_
void     hor_get_canny_colours ( Hor_ED_Output_Params  * );
#endif
#endif
