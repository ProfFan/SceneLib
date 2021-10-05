/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   void @hor_display_corner ( Hor_Corner *corner, void *params )
*   void @hor_display_corner_map ( Hor_Corner_Map *map, int type, void *params)
*
*   Corner/corner map display functions.
*   The type argument to hor_display_corner_map() specifies which type
*   of corner is displayed. If type is < 0, all corners are displayed.
********************/
void hor_display_corner ( Hor_Corner *corner, void *params )
{
   Hor_CO_Output_Params *co_params = (Hor_CO_Output_Params *) params;

   hor_display_set_function ( HOR_DISPLAY_COPY );
   hor_display_set_colour ( co_params->corner_colour );
   hor_display_plot ( (int) corner->cf, (int) corner->rf );
}

void hor_display_corner_map ( Hor_Corner_Map *map, int type, void *params )
{
   Hor_List    list;
   Hor_Corner *cptr;

   if ( map == NULL )
   {
      hor_errno = HOR_IMPROC_NULL_POINTER_ARGUMENT;
      return;
   }

   hor_display_highlight_region ( map->c0, map->r0, map->width, map->height );
   for ( list = map->corner_list; list != NULL; list = list->next )
   {
      cptr = (Hor_Corner *) list->contents;
      if ( type < 0 || cptr->type == type )
	 if ( cptr->display_func != NULL ) cptr->display_func ( cptr, params );
   }

   hor_display_flush();
}
