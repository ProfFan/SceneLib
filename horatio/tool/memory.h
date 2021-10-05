/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/memory.h */

#ifdef _XtIntrinsic_h
#ifdef _HORATIO_LIST_

Hor_Assoc_Label hor_create_memory_panel ( XtAppContext app_con, Widget parent);
Hor_Bool hor_popup_memory_panel ( Widget button, Hor_Assoc_Label panel_label,
				  Hor_List list,
				  void (*display_func)(void *),
				  void (*select_func)(void *),
				  void (*wrap_forwards_func)(void *),
				  void (*wrap_backwards_func)(void *),
				  void (*free_func)(Hor_List),
				  double time_interval );
Hor_Bool hor_popdown_memory_panel   ( Hor_Assoc_Label panel_label );
Hor_Bool hor_memory_panel_in_use    ( Hor_Assoc_Label panel_label );
Hor_Bool hor_redisplay_memory_panel ( Hor_Assoc_Label panel_label );

#endif /* _HORATIO_LIST_ */
#endif /* _XtIntrinsic_h */
