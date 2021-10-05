#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/*******************
*   void @hor_display_line_segment ( Hor_Line_Segment *line, void *params )
*   void @hor_display_line_segment_map ( Hor_Line_Segment_Map *map,
*                                       int   type,
*                                       void *params )
*
*   Line segment/line segment list display functions.
*   The type argument to hor_display_line_segment_map() specifies which type
*   of line segment is displayed. If type is < 0, all lines are displayed.
*   Line segments of different types should be displayed by consecutive calls
*   to hor_display_line_segment_map().
********************/
void hor_display_line_segment ( Hor_Line_Segment *line, void *params )
{
   Hor_LI_Output_Params *ls_params = (Hor_LI_Output_Params *) params;

   hor_display_set_function(HOR_DISPLAY_COPY);
   hor_display_set_colour(ls_params->line_colour);
   hor_display_set_line_width(2);
   hor_display_line ( line->x1, line->y1, line->x2, line->y2 );
   hor_display_set_line_width(0);
}

void hor_display_line_segments ( Hor_Line_Segment_Map *map, int type,
				 void *params )
{
    Hor_List list;
    Hor_Line_Segment *line;

    for ( list = map->line_list; list != NULL; list = list->next )
    {
       line = (Hor_Line_Segment *) list->contents;
       if ( type < 0 || line->type == type )
	  if ( line->display_func != NULL )
	     line->display_func ( line, params );
    }

    hor_display_flush();
}
