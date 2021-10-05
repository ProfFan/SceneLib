/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/fl_disp.h */

#ifdef _HORATIO_IMAGE_
void hor_display_image_flow (
   Hor_Sub_Image *flow_c,
   Hor_Sub_Image *flow_r,   /* x & y components of normal flow vector field */
   Hor_Sub_Image *flow_flag,/* specifies which vectors to believe */
   u_long     start_colour, /* colour of dot plotted at start of vector */
   u_long     line_colour,  /* colour of lines to represent vectors */
   float      scale,        /* factor by which to multiply length of lines to
			       improve visibility */
   int        increment );  /* pixel distance between displayed samples of
			       vector field */
#endif /* _HORATIO_IMAGE_ */
