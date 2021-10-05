/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/li_disp.h */

typedef struct
{
   int line_colour;
} Hor_LI_Output_Params;

void hor_display_line_segment ( Hor_Line_Segment *line, void *params );
void hor_display_line_segments ( Hor_Line_Segment_Map *map,
				 int type, void *params );
