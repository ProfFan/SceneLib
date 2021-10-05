/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

#ifdef _XtIntrinsic_h

void fill_canvas_panel ( Widget panel, int canvas_size, int no_canvases,
			 int    canvases_to_a_side );
void register_canvases ( Display *display,
			 u_long   image_background_colour,
			 u_long   image_border_colour,
			 u_long   region_border_colour,
			 u_long   below_threshold_colour,
			 u_long   above_threshold_colour );

#endif /* _XtIntrinsic_h */

void change_canvas(void);
