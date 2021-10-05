/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/co_disp.h */

#ifdef _HORATIO_IMAGE_

/*******************
*   typedef struct
*   {
*      u_long corner_colour;
*   } @Hor_CO_Output_Params;
*
*   Standard corner map output parameter structure definition, to be used in
*   conjunction with hor_display_corner().
********************/
typedef struct
{
   u_long corner_colour;
} Hor_CO_Output_Params;

void hor_display_corner ( Hor_Corner *corner, void *params );
void hor_display_corner_map ( Hor_Corner_Map *, int type, void *params );

#endif /* _HORATIO_IMAGE_ */
