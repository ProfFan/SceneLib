/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_TOOL_
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/ls_tool.h */

void hor_set_line_segment_colours ( u_long line_colour );

#ifdef _HORATIO_IMPROC_
Hor_Bool hor_get_line_segment_params ( Hor_LI_Output_Params * );
#endif
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/pc_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_plessey_corner_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_plessey_corner_colours ( u_long corner_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_plessey_corner_defaults ( Hor_PC_Process_Params );
Hor_Bool hor_get_plessey_corner_params   ( Hor_PC_Process_Params * );
#ifdef _HORATIO_IMPROC_
void     hor_get_plessey_corner_colours ( Hor_CO_Output_Params  * );
#endif
#endif

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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/cow_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_wang_corner_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_wang_corner_colours ( u_long corner_colour );

#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_wang_corner_defaults ( Hor_COW_Process_Params );
Hor_Bool hor_get_wang_corner_params   ( Hor_COW_Process_Params * );
#ifdef _HORATIO_IMPROC_
void hor_get_wang_corner_colours ( Hor_CO_Output_Params * );
#endif
#endif

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/bcm_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_bog_corner_match_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_bog_corner_match_colours ( u_long trajectory_colour,
				        u_long dot_colour,
				        u_long old_traj_colour );

#ifdef _HORATIO_IMPROC_
#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_bog_corner_match_defaults ( Hor_BCM_Process_Params,
					     int trajectory_length );
Hor_Bool hor_get_bog_corner_match_params ( Hor_BCM_Process_Params *,
					   Hor_CM_Output_Params * );
#endif
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/dcm_tool.h */

#ifdef _XtIntrinsic_h
Widget hor_create_bd_corner_match_popup ( String name, Widget parent, ... );
#endif /* _XtIntrinsic_h */

void hor_set_bd_corner_match_colours ( u_long trajectory_colour,
				       u_long dot_colour,
				       u_long old_traj_colour );

#ifdef _HORATIO_IMPROC_
#ifdef _HORATIO_PROCESS_
Hor_Bool hor_set_bd_corner_match_defaults ( Hor_DCM_Process_Params,
					    int trajectory_length );
Hor_Bool hor_get_bd_corner_match_params ( Hor_DCM_Process_Params *,
					  Hor_CM_Output_Params * );
#endif
#endif
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/popup.h */

#include <stdarg.h>

#ifdef _XtIntrinsic_h

int hor_convert_X_args ( va_list *aptr, Arg **arg );

/*******************
*   typedef struct
*   {
*      Widget   popup_frame; (frame for popup window)
*      Position x, y;        (display position relative to parent)
*   } Hor_Popup_Data;
*
*   Structure containing popup information.
********************/
typedef struct
{
   Widget   popup_frame;
   Position x, y;
} Hor_Popup_Data;

void hor_show_popup ( Widget widget,
		      XtPointer client_data, XtPointer call_data );

/*******************
*   void  @hor_set_widget_value ( Widget *widget, const char *string )
*   char *@hor_get_widget_value ( Widget *widget )
*
*   hor_set_widget_value() sets the string value of the widget to the string.
*   hor_get_widget_value() returns the string value of the widget.
*   Both are implemented as macros.
********************/
#define hor_set_widget_value(wid,string) XtVaSetValues(wid, XtNvalue, string,0)
/*typedef char * (*XowFuncCast)(Widget); needed because of fussy g++ */
#define hor_get_widget_value(wid) XowPanelTextGetValueString(wid)

/*******************
*   Widget @hor_create_text ( Widget popup_panel, Widget last,
*                            const char *name )
*
*   Returns a panel-text widget with the given popup panel as parent.
*   New widget is placed below the given "last" widget and assigned the
*   given name.
********************/
#define hor_create_text( popup_panel, last, name ) \
          XtVaCreateManagedWidget ( name, panelTextWidgetClass, popup_panel, \
				    XtNfromVert, last, 0 )

void hor_hide_popup ( Widget widget,
		      XtPointer client_data, XtPointer call_data );
void hor_create_done_button ( Widget popup_panel, Widget last );
void hor_create_reset_done_buttons ( Widget popup_panel, Widget last,
				     void (*reset_func) (void) );

#endif /* _XtIntrinsic_h */

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
/* Copyright 1993 Paul A. Beardsley (pab@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/threed.h */

#ifdef _XtIntrinsic_h
#ifdef _HORATIO_MATH_

typedef struct {Hor_Matrix *p;}       Hor_Item_3D_point; /* p is a 3x1 matrix */
typedef struct {Hor_Matrix *p1, *p2;} Hor_Item_3D_line;  /* p1 & p2 are 3x1 matrices */
typedef struct
{
   Hor_Matrix *p1,    *p2,    *p3;
   int         c1, r1, c2, r2, c3, r3;
   Hor_Image  *image;
} Hor_Item_3D_facet;

typedef struct
{
   Hor_Item_3D_point point;
   Hor_Assoc_Label   font;
   const char       *label;
} Hor_Item_3D_label;

typedef struct
{
   enum {HOR_LINE_3D, HOR_POINT_3D, HOR_FACET_3D, HOR_LABEL_3D} type;
   union {Hor_Item_3D_point point;
	  Hor_Item_3D_line  line;
          Hor_Item_3D_facet facet;
          Hor_Item_3D_label label;} u;
} Hor_Item_3D;

typedef void (*Hor_3D_Item_Func) (Hor_Assoc_Label, Hor_Item_3D);

#ifdef _HORATIO_LIST_

typedef void (*Hor_3D_Item_List_Func) (Hor_Assoc_List assoc_list);

Hor_Assoc_Label hor_create_3D ( Widget parent, int size,
			        u_long background_colour,
			        u_long selected_colour );
void hor_register_3D ( Display *display );
Hor_Bool hor_popup_3D ( Widget button, Hor_Assoc_Label label_3D,
		        Hor_3D_Item_Func      oneselect_func,
		        Hor_3D_Item_List_Func procselected_func,
		        Hor_3D_Item_List_Func allclear_func,
		        Hor_3D_Item_Func      onedelete_func,
		        Hor_3D_Item_List_Func allrestore_func,
		        Hor_3D_Item_List_Func finish_func );
Hor_Bool hor_popdown_3D ( Hor_Assoc_Label label_3D );
Hor_Bool hor_3D_item ( Hor_Assoc_Label label_3D,
		       Hor_Assoc_Label item_label,
		       Hor_Item_3D item_ptr, u_long colour );
Hor_Bool hor_init_3D ( Hor_Assoc_Label label_3D,
		       Hor_Bool subtract_centroid );
Hor_Bool hor_colour_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label, u_long colour );
Hor_Bool hor_select_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label );
Hor_Bool hor_delete_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label );
Hor_Bool hor_reset_3D_item ( Hor_Assoc_Label label_3D,
			     Hor_Assoc_Label item_label );
Hor_Bool hor_3D_clear ( Hor_Assoc_Label label_3D );
Hor_Bool hor_3D_in_use    ( Hor_Assoc_Label label_3D );
Hor_Bool hor_redisplay_3D ( Hor_Assoc_Label label_3D );
Hor_Bool hor_write_3D_text ( Hor_Assoc_Label label_3D, const char *base_name );
Hor_Bool hor_write_3D_image ( Hor_Assoc_Label label_3D, const char *base_name,
			      int image_size );

Hor_Item_3D *hor_alloc_item_3D ( int item_type );
Hor_Item_3D *hor_fill_item_3D ( Hor_Item_3D *item, ... );
Hor_Item_3D *hor_make_item_3D ( int item_type, ... );
void         hor_free_item_3D ( Hor_Item_3D *item );

#endif /* _HORATIO_LIST_ */
#endif /* _HORATIO_MATH_ */
#endif /* _XtIntrinsic_h */
/*
 * toggle group module:
 * to create a button with an associated popup containing a set of 
 * toggles and a done button
 *
 * Ian Reid, 9.9.93
 *
 * Horatio changes: Philip McLauchlan 30.9.93
 */

/* included from tool/toggle.h */

typedef int (*Hor_TG_Callback) (int bit_field, int toggle_mask, void *data);

#ifdef _XtIntrinsic_h

Widget hor_create_togglegroup_widget(String name,             /* widget name */
				     Widget parent,           /* parent widget */
				     String *choices,         /* list of strings of toggles */
				     int no_choices,          /* number in list */
				     Position x,              /* default popup position rel've */
				     Position y,              /*  to button */
				     Hor_TG_Callback on_callback, /* callback procedures */
				     Hor_TG_Callback off_callback, /* for toggle switching */
				     void *data, /* data to be passed to callbacks */
				     ...);      /* NULL-terminated list of resource,value pairs */
int  hor_get_togglegroup ( Widget button );
void hor_set_togglegroup ( Widget button, int bit_field );

#endif /* _XtIntrinsic_h */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/region.h */

void hor_region_delete_last_selected (void);
void hor_region_delete_selected      (void);
void hor_region_set_function         ( void (*)(int c1, int r1, int c2, int r2,
						void *data) );
void hor_region_start                ( int c, int r, void *data );
void hor_region_moving               ( int c, int r, void *data );
void hor_region_finish               ( int c, int r, void *data );
void hor_region_cancel               ( int c, int r, void *data );
void hor_region_select               ( int c, int r, void *data );
void hor_region_clear                (void);
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/read_prm.h */

/* maximum length of editable parameter string */
#define HOR_EDIT_STRING_SIZE 20

Hor_Bool hor_read_double_param(char        read_string[HOR_EDIT_STRING_SIZE],
			       const char *param_name,
			       Hor_Bool  (*test_function)(double, const char *,
							  Hor_Error_Type),
			       Hor_Error_Type error_type,
			       double *dp );
Hor_Bool hor_read_float_param ( char        read_string[HOR_EDIT_STRING_SIZE],
			        const char *param_name,
			        Hor_Bool  (*test_function)(float, const char *,
							   Hor_Error_Type),
			        Hor_Error_Type error_type,
			        float *fp );
Hor_Bool hor_read_int_param   ( char        read_string[HOR_EDIT_STRING_SIZE],
			        const char *param_name,
			        Hor_Bool  (*test_function)(int, const char *,
							   Hor_Error_Type),
			        Hor_Error_Type error_type,
			        int *ip );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/test_prm.h */

Hor_Bool hor_int_null         ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_pos          ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos      ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos_even ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_int_abs_pos_odd  ( int   param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_null       ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_pos        ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_abs_pos    ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_float_01_inc     ( float param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_null      ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_pos       ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_abs_pos   ( double param, const char *param_name,
			        Hor_Error_Type error_type );
Hor_Bool hor_double_01_inc    ( double param, const char *param_name,
			        Hor_Error_Type error_type );
