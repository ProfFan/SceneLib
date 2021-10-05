/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/velocity.h */

#ifdef _XtIntrinsic_h
Widget hor_fill_velocity_panel ( Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_velocity_defaults(void);
int  hor_get_upper_left_c_velocity(void);
int  hor_get_upper_left_r_velocity(void);
int  hor_get_lower_right_c_velocity(void);
int  hor_get_lower_right_r_velocity(void);
