/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/line.h */

typedef struct _Hor_Line_Segment {
     int    status;         /* for use by a line matcher, for instance */
     double x1, y1, x2, y2; /* endpoint positions */
     double angle;          /* angle of line in degrees, clockwise
			       from positive x-direction */
     int    dir;            /* +1 or -1, the contrast sign. If (x1,y1) to
			       (x2,y2) points in same direction as the original
			       edge string, i.e. in the same direction as the
			       leftmost to rightmost fields of the string,
			       then dir is +1, otherwise it is -1 */
     Hor_Bool truncated1; /* start of line (x1, y1) is truncated */
     Hor_Bool truncated2; /* end of line (x2, y2) is truncated */

     /* user-defined attributes */
     int    type; /* e.g. HOR_MAP_NO_ATTRIB */
     void  *attrib;
     void (*attrib_free_func)(void *attrib);
     void (*display_func)(struct _Hor_Line_Segment *line, void *params);
} Hor_Line_Segment;

typedef struct
{
   int c0, r0, width, height;
   Hor_List line_list;
   int      nlines;

   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
} Hor_Line_Segment_Map;

Hor_Line_Segment_Map *hor_alloc_line_segment_map ( int type, void *attrib,
				      void (*attrib_free_func)(void *attrib) );
void hor_free_line_segment_map ( Hor_Line_Segment_Map *map );
Hor_Line_Segment *hor_add_line_segment (
			    Hor_Line_Segment_Map *map,
			    double x1, double y1, double x2, double y2,
			    double angle, int dir,
			    Hor_Bool truncated1, Hor_Bool truncated2,
			    int type, void *attrib,
			    void (*attrib_free_func)(void *attrib),
			    void (*display_func)(Hor_Line_Segment *line,
						 void *params) );
		      
