/* Copyright 1994 David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/line_fit.h */

Hor_Line_Segment_Map *hor_find_lines ( Hor_Edge_Map *edge_map, 
				       int no_init_points,
				       float sigma, float thres );
Hor_Line_Segment_Map *hor_find_lines_rec ( Hor_Line_Segment_Map* line_map,
					   int no_init_points,
					   float sigma, float thres );
