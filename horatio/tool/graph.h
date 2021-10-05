/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/graph.h */

#ifdef _HORATIO_LIST_

#ifdef _XtIntrinsic_h
Hor_Assoc_Label hor_create_graph ( Widget parent,
				   Hor_Assoc_Label horiz_label,
				   Hor_Assoc_Label vert_label,
				   int width, int height,
				   u_long background_colour );
void hor_register_graphs ( Display *display );
Hor_Bool hor_popup_graph ( Widget button, Hor_Assoc_Label graph_label,
			   float t_range, float f_low, float f_high, ... );
#endif /* _XtIntrinsic_h */

Hor_Bool hor_popdown_graph ( Hor_Assoc_Label graph_label );
Hor_Bool hor_graph_point ( Hor_Assoc_Label graph_label,
			   Hor_Assoc_Label trace_label, float t, float f,
			   u_long colour );
Hor_Bool hor_reset_graph ( Hor_Assoc_Label graph_label );
void hor_graph_reset_time ( Hor_Assoc_Label graph_label, float t_low );
Hor_Bool hor_graph_in_use    ( Hor_Assoc_Label graph_label );
Hor_Bool hor_redisplay_graph ( Hor_Assoc_Label graph_label );
Hor_Bool hor_write_graph ( Hor_Assoc_Label graph_label, const char *file_name);

#endif /* _HORATIO_LIST_ */
