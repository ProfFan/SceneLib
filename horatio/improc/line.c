/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   Hor_Line_Segment_Map *@hor_alloc_line_segment_map (
*      int   type,   (type of corner map, e.g. HOR_NO_ATTRIB)
*      void *attrib, (user-defined corner map attributes)
*      void (*attrib_free_func)(void *attrib) )
*
*   void @hor_free_line_segment_map ( Hor_Line_Segment_Map *map )
*
*   Line segment map allocatio/free functions.
********************/
Hor_Line_Segment_Map *hor_alloc_line_segment_map ( int type, void *attrib,
				      void (*attrib_free_func)(void *attrib) )
{
   Hor_Line_Segment_Map *map;

   map = hor_malloc_type(Hor_Line_Segment_Map);
   map->line_list        = NULL;
   map->nlines           = 0;
   map->type             = type;
   map->attrib           = attrib;
   map->attrib_free_func = attrib_free_func;
   return map;
}

static void free_line_segment ( void *data )
{
   Hor_Line_Segment *line = (Hor_Line_Segment *) data;

   if ( line->attrib_free_func != NULL )
      line->attrib_free_func ( line->attrib );

   hor_free ( data );
}

void hor_free_line_segment_map ( Hor_Line_Segment_Map *map )
{
   if ( map == NULL ) return;

   hor_free_list ( map->line_list, free_line_segment );
   if ( map->attrib_free_func != NULL ) map->attrib_free_func ( map->attrib );

   hor_free ( (void *) map );
}

/*******************
*   Hor_Line_Segment *@hor_add_line_segment (
*            Hor_Line_Segment_Map *map,
*            double x1, double y1,
*            double x2, double y2, (endpoints of line segment)
*            double angle,         (line segment orientation)
*            int    dir,           (direction, -1 or +1)
*            Hor_Bool truncated1,  (start of line (x1, y1) is truncated)
*            Hor_Bool truncated2,  (end of line (x2, y2) is truncated)
*            int type, (type of line segment, e.g. HOR_NO_ATTRIB)
*            void *attrib, (user attributes associated with line segment)
*            void (*attrib_free_func)(void *attrib),
*            void (*display_func)(Hor_Line_Segment *line, void *params) )
*
*   Adds a line segment with given fields to a line segment map.
********************/
Hor_Line_Segment *hor_add_line_segment ( Hor_Line_Segment_Map *map,
			    double x1, double y1, double x2, double y2,
			    double angle, int dir,
			    Hor_Bool truncated1, Hor_Bool truncated2,
			    int type, void *attrib,
			    void (*attrib_free_func)(void *attrib),
			    void (*display_func)(Hor_Line_Segment *line,
						 void *params) )
{
   Hor_Line_Segment *line = hor_malloc_type(Hor_Line_Segment);


   line->status = HOR_UNMATCHED;
   line->x1 = x1; line->y1 = y1;
   line->x2 = x2; line->y2 = y2;
   line->angle  = angle;
   line->dir    = dir;
   line->truncated1 = truncated1;
   line->truncated2 = truncated2;
   line->type   = type;
   line->attrib = attrib;
   line->attrib_free_func = attrib_free_func;
   line->display_func     = display_func;
   map->line_list = hor_insert ( map->line_list, line );
   map->nlines++;
   return line;
}
