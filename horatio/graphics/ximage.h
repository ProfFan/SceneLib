/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

void hor_set_threshold_colours ( u_long below_threshold_colour,
				 u_long above_threshold_colour );
XImage *hor_convert_image_to_X_format ( Hor_Image *, Display *, int scrnum,
				        int depth, int hor_subsample_c,
					           int hor_subsample_r,
				        int pixel_size_factor,
				        va_list *aptr );
void    hor_display_X_format_image    ( XImage *, Display *, Window, GC,
				        int top_left_c, int top_left_r );
