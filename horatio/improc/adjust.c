/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   void @hor_adjust_region_for_border(int  width,          int  height,
*                                     int  lower_c_border, int  upper_c_border,
*                                     int  lower_r_border, int  upper_r_border,
*                                     int *c1p,            int *r1p,
*                                     int *c2p,            int *r2p )
*
*   Coordinates of top-left and bottom-right corners of region are given
*   initially by (*c1p,*r1p) and (*c2p,*r2p) respectively. These coordinates
*   are adjusted to fit inside the rectangle with corners
*   (lower_c_border, lower_r_border) and
*   (width - upper_c_border, height - upper_r_border).
********************/
void hor_adjust_region_for_border ( int  width,          int  height,
				    int  lower_c_border, int  upper_c_border,
				    int  lower_r_border, int  upper_r_border,
				    int *c1p,            int *r1p,
				    int *c2p,            int *r2p )
{
   int upper_c = width  - upper_c_border;
   int upper_r = height - upper_r_border;

   if ( width < 0 || height < 0 ||
        *c1p > *c2p || *r1p > *r2p ||
        lower_c_border < 0 || upper_c_border < 0 ||
        lower_r_border < 0 || upper_r_border < 0 ||
        upper_c <= lower_c_border || upper_r <= lower_r_border )
   {
      /* destroy region by setting dimensions to zero */
      *c2p = *c1p;
      *r2p = *r1p;
      return;
   }
        
   if      ( *c1p < lower_c_border ) *c1p = lower_c_border;
   else if ( *c1p > upper_c )        *c1p = upper_c;

   if      ( *r1p < lower_r_border ) *r1p = lower_r_border;
   else if ( *r1p > upper_r )        *r1p = upper_r;

   if      ( *c2p < lower_c_border ) *c2p = lower_c_border;
   else if ( *c2p > upper_c )        *c2p = upper_c;

   if      ( *r2p < lower_r_border ) *r2p = lower_r_border;
   else if ( *r2p > upper_r )        *r2p = upper_r;
}
