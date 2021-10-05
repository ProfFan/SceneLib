/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   void @hor_display_image_flow (
*           Hor_Sub_Image *flow_c,
*           Hor_Sub_Image *flow_r,    (components of normal flow vector field)
*           Hor_Sub_Image *flow_flag, (specifies which vectors to believe)
*           u_long     start_colour, (colour of dot plotted at start of vector)
*           u_long     line_colour,  (colour of lines to represent vectors)
*           float      scale,        (factor by which to multiply length of
*                                     lines to improve visibility)
*           int        increment )   (pixel distance between displayed samples
*                                     of vector field) 
*
*   Image flow display function.
********************/
void hor_display_image_flow (
   Hor_Sub_Image *flow_c,
   Hor_Sub_Image *flow_r,   /* x & y components of normal flow vector field */
   Hor_Sub_Image *flow_flag,/* specifies which vectors to believe */
   u_long     start_colour, /* colour of dot plotted at start of vector */
   u_long     line_colour,  /* colour of lines to represent vectors */
   float      scale,        /* factor by which to multiply length of lines to
			       improve visibility */
   int        increment )   /* pixel distance between displayed samples of
			       vector field */
{
   int c0 = flow_c->c0, width  = flow_c->image.width;
   int r0 = flow_c->r0, height = flow_c->image.height;
   int r, c;

   char  **arr_f = flow_flag->image.array.c;
   float **arr_c = flow_c->image.array.f;
   float **arr_r = flow_r->image.array.f;

   if ( !hor_same_dims_sub_images ( flow_c, flow_r, flow_flag, NULL ) )
      hor_error ( "vectors & flags incompatible (hor_display_image_flow)",
		  HOR_FATAL );

   if ( !hor_type_sub_image ( flow_c,    HOR_FLOAT ) ||
        !hor_type_sub_image ( flow_r,    HOR_FLOAT ) ||
        !hor_type_sub_image ( flow_flag, HOR_CHAR ) )
      hor_error ( "images wrong type (hor_display_image_flow)", HOR_FATAL );

   hor_display_highlight_region ( flow_c->c0,          flow_r->r0,
				  flow_c->image.width, flow_c->image.height );
   hor_display_set_function ( HOR_DISPLAY_COPY );

   hor_display_set_colour ( start_colour );
   for ( r = 0; r < height; r += increment )
      for ( c = 0; c < width; c += increment )
	 if ( arr_f[r][c] )
	    hor_display_fill_circle_actual_size ( c0+c, r0+r, 2 );

   hor_display_set_colour ( line_colour );
   for ( r = 0; r < height; r += increment )
      for ( c = 0; c < width; c += increment )
	 if ( arr_f[r][c] )
	    hor_display_line ( (float)(c0+c) + 0.5, (float) (r0+r) + 0.5,
			       (float)(c0+c) + 0.5 + scale*arr_c[r][c],
			       (float)(r0+r) + 0.5 + scale*arr_r[r][c] );

   hor_display_flush();
}
