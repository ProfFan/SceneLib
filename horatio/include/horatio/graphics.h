/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_GRAPHICS_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/print.h */

#ifdef _XtIntrinsic_h
Hor_Assoc_Label hor_set_text_window ( Widget wid );
#endif

void hor_reset_text_window  ( Hor_Assoc_Label );
void hor_delete_text_window ( Hor_Assoc_Label );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/colour.h */

/*******************
*   typedef union
*   {
*      u_char  *uc;
*      u_short *us;
*   } @Hor_X_to_Image_ColourMap;
*
*   Definition of colour-map structure to map X colour identifiers into
*   internal image format, whether u_char (8 bits) or u_short (16 bits).
********************/
typedef union
{
   u_char  *uc;
   u_int *us;
} Hor_X_to_Image_ColourMap;

#ifdef _XtIntrinsic_h
void hor_colourmap_setup ( Display *display,
			   u_int min_grey_bits, u_int max_grey_bits, ... );
#endif

u_long hor_colour_alloc ( u_short red, u_short green, u_short blue );
void hor_free_colourmap(void);

Hor_Bool hor_get_colour_name_value ( const char *name, u_long *colour );

#ifdef _HORATIO_IMAGE_
Hor_Image_Type hor_get_image_display_format(void);
#endif /* _HORATIO_IMAGE_ */
u_int          hor_get_image_display_depth(void);

/*******************
*   extern u_long *@Hor_Grey;
*
*   Grey-level array (0..255) indexing into X grey-level colourmap.
*   Hor_Grey[0] is black, Hor_Grey[255] is white.
********************/
extern u_long *Hor_Grey;

/*******************
*   #define @HOR_NEUTRAL_COLOUR 0xffffffff
*
*   Can be used to turn off drawing commands (must be detected
*   explicitly to work)
********************/
#define HOR_NEUTRAL_COLOUR 0xffffffff
/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/display.h */

/*******************
*   typedef void (*@hor_button_func)(int c, int r, void *data);
*   typedef void (*@hor_window_func)(void *data);
*
*   Horatio mouse event callback function types.
*   c & r are the canvas coordinates of the mouse position.
*   data is the pointer provided by hor_display_set_mouse_functions().
********************/
typedef void (*hor_button_func)(int c, int r, void *data);
typedef void (*hor_window_func)(void *data);

#ifdef _XtIntrinsic_h
Hor_Assoc_Label hor_display_set_string ( Widget parent, ... );
#endif

void hor_display_delete_string ( Hor_Assoc_Label string_label );

typedef struct {
	int c;
	int r;
} Hor_2D_Vertex;

/*******************
*   typedef enum { @HOR_DISPLAY_COPY, @HOR_DISPLAY_XOR } @Hor_Display_Function;
*
*   Types of graphics drawing available:
*      HOR_DISPLAY_COPY: overlay on existing display.
*      HOR_DISPLAY_XOR:  combine current colour in exclusive-or fashion with
*                        displayed colour.
********************/
typedef enum { HOR_DISPLAY_COPY, HOR_DISPLAY_XOR } Hor_Display_Function;

#ifdef _XtIntrinsic_h
void hor_display_initialise ( Display *, GC,
			      u_long image_background_colour,
			      u_long image_border_colour,
			      u_long region_border_colour,
			      u_long below_threshold_colour,
			      u_long above_threshold_colour );
/* Added by davison@etl.go.jp */
void hor_display_initialise_xforms ( Display *, GC,
			      u_long image_background_colour,
			      u_long image_border_colour,
			      u_long region_border_colour,
			      u_long below_threshold_colour,
			      u_long above_threshold_colour );

Hor_Assoc_Label hor_display_set_window ( Window, Widget, u_int cursor_shape,
					 Hor_Assoc_Label string_label );
#endif

void            hor_display_reset_window  ( Hor_Assoc_Label );
Hor_Assoc_Label hor_display_get_window(void);
void            hor_display_delete_window ( Hor_Assoc_Label );

void hor_display_set_mouse_functions ( Hor_Assoc_Label canvas_label,
			     hor_button_func, hor_button_func, const char *,
			     hor_button_func, hor_button_func, const char *,
			     hor_button_func, hor_button_func, const char *,
			     hor_button_func, hor_window_func, hor_window_func,
			     void *data );
void hor_display_remove_mouse_functions ( Hor_Assoc_Label canvas_label );
			        
Hor_Bool hor_display_set_params ( int width, int height );
Hor_Bool hor_display_canvas_set_params ( Hor_Assoc_Label canvas_label,
					 int width, int height );
Hor_Bool hor_display_get_params ( int *top_left_c,     int *top_left_r,
				  int *bottom_right_c, int *bottom_right_r );
Hor_Bool hor_display_get_dims   ( int *width_ptr, int *height_ptr );

void hor_display_set_cursor ( u_int cursor );

void            hor_display_set_function   ( Hor_Display_Function );
void            hor_display_set_colour     ( u_long );
void            hor_display_set_line_width ( u_int );
Hor_Assoc_Label hor_display_load_font      ( const char *font_name );
void            hor_display_set_font       ( Hor_Assoc_Label font_label );

#ifdef _XtIntrinsic_h
XFontStruct *hor_display_get_fontstruct ( Hor_Assoc_Label font_label );
#endif
void     hor_display_text           ( int c, int r, const char *string );
void     hor_display_canvas_text    ( int c, int r, const char *string );

void hor_display_clear ( u_long );
void hor_display_flush ( void );

void hor_display_point_convert         ( int  canvas_c, int  canvas_r,
					 int *c_ptr,    int *r_ptr );
void hor_display_region_convert        ( int  c1,     int  r1,
					 int  c2,     int  r2,
					 int *c1_ptr, int *r1_ptr,
					 int *c2_ptr, int *r2_ptr );
Hor_Bool hor_display_within_image      ( int c, int r );
void hor_display_draw_canvas_rectangle ( int c, int r, int width, int height );
void hor_display_fill_canvas_rectangle ( int c, int r, int width, int height );
void hor_display_highlight_region ( int c0, int r0, int width, int height );

void hor_display_draw_rectangle ( int c, int r, int width, int height );
void hor_display_fill_rectangle ( int c, int r, int width, int height );
void hor_display_fill_diamond             ( int c, int r, float half_size );
void hor_display_draw_circle              ( int c, int r, int radius );
void hor_display_draw_circle_actual_size  ( int c, int r, int radius );
void hor_display_fill_circle              ( int c, int r, int radius );
void hor_display_fill_circle_actual_size  ( int c, int r, int radius );
void hor_display_draw_ellipse ( int c, int r, float axis1, float axis2,
			        float angle );
void hor_display_draw_ellipse_actual_size ( int c, int r,
					    float axis1, float axis2,
					    float angle );
void hor_display_plot ( int c, int r );
void hor_display_line ( float c1, float r1, float c2, float r2 );

u_long hor_display_get_pixel ( int c, int r );

#ifdef _HORATIO_IMAGE_
Hor_Bool hor_display_image     (Hor_Image *,     int hor_subsample_ratio, ...);
Hor_Bool hor_display_sub_image (Hor_Sub_Image *, int hor_subsample_ratio, ...);
Hor_Bool hor_display_subsampled_image ( int c0, int r0, int hor_subsample_c,
				                        int hor_subsample_r,
				        Hor_Image *image, ... );
#endif /* _HORATIO_IMAGE_ */

u_long hor_display_get_xor_colour ( void );

Hor_Assoc_Label hor_display_store_state(void);
void            hor_display_recall_state  ( Hor_Assoc_Label );
void            hor_display_destroy_state ( Hor_Assoc_Label );

#ifdef _HORATIO_IMAGE_
Hor_Image *hor_display_get_image(void);
#endif /* _HORATIO_IMAGE_ */

Hor_Bool   hor_display_write_to_file  ( const char *base_name );
Hor_Bool   hor_display_read_from_file ( const char *base_name );

Hor_List hor_display_make_movie ( const char *root_name, ... );
Hor_List hor_make_movie_from_images ( Hor_List image_sequence );
#ifdef _XtIntrinsic_h
#ifdef _HORATIO_IMAGE_
XImage *hor_display_make_movie_image ( Hor_Image *image, ... );
#endif /* _HORATIO_IMAGE_ */
void hor_display_show_movie_image ( XImage *ximage );
#endif /* _XtIntrinsic_h */
void hor_display_destroy_movie ( Hor_List movie );

/* Copyright 1994 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/state.h */

void hor_recall_states (Hor_List states);
void hor_destroy_states ( Hor_List states );
void hor_destroy_states_movie ( Hor_List movie );
