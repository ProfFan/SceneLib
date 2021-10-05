/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from feature.h */

/* feature measurement types */
#define HOR_POINT_XY     0
#define HOR_LINE_AX_Y_CZ 1
#define HOR_LINE_X_BY_CZ 2

typedef struct
{
   double xy1, xy2;
   double l1,  l2;
} Hor_Line_End;

typedef enum { HOR_POSSIBLE, HOR_CHOSEN, HOR_DISCARDED } Hor_Match_Type;

#ifdef _HORATIO_MATH_

typedef struct
{
   int             status;
   void           *feature;
   Hor_Trajectory *match;
   double          difference;
   Hor_Matrix     *z, *Rinv; /* to be freed */
   int             z_type;
   void           *zdata;
} Hor_Feature_Def;

typedef struct
{
   Hor_Trajectory  *traj;
   Hor_Feature_Def *feature;
   double           difference;
   Hor_Matrix      *y; /* typically from VSDF: not to be freed */
   int              y_type;
   Hor_Match_Type   chosen;
   Hor_Match_Type   chosen_best;
} Hor_Pot_Match;

typedef struct
{
   int c0, r0, cols, rows, csize, rsize;
   Hor_Feature_Def    *feature_array;
   int                 farray_size;
   Hor_Feature_Def ****feature;
   int               **no_features;
} Hor_Coarse_Feature_Map;

Hor_Coarse_Feature_Map *hor_alloc_coarse_feature_map ( int c0,    int r0,
						       int cols,  int rows,
						       int csize, int rsize,
						       int max_features );
void hor_free_coarse_feature_map ( Hor_Coarse_Feature_Map *feature_map );
Hor_Feature_Def *hor_make_feature_array ( Hor_List feature_list,
					  int      no_features );
#ifdef _HORATIO_IMAGE_
Hor_Coarse_Feature_Map *hor_make_coarse_corner_map (Hor_Corner_Map *corner_map,
						    int csize,   int rsize,
						    int cfactor, int rfactor,
						    int max_features,
						    double offset_x,
						    double offset_y,
						    double xy_sigma);
#endif /* _HORATIO_IMAGE_ */
Hor_Coarse_Feature_Map
   *hor_make_coarse_line_map ( Hor_Line_Segment_Map *line_map,
			       int csize,   int rsize,
			       int cfactor, int rfactor,
			       int max_features,
			       double offset_x, double offset_y, double zs,
			       double normal_sigma );
void hor_traj_coarse_position ( int traj_type, void *traj_last,
			        int *c, int *r );
void hor_traj_compatibility ( Hor_Trajectory_Map *traj_map,
			      int traj_type,
			      Hor_Coarse_Feature_Map *feature_map,
			      void (*pos_func)(int type, void *last,
					       int *c, int *r),
			      double (*difference_func)(Hor_Trajectory *traj,
							void *feature,
							void *params),
			      double difference_thres, void *params,
			      Hor_Pot_Match *pot_match, int *start,
			      int max_pot_matches );
void hor_select_unique ( Hor_Pot_Match *pot_match, int no_pot_matches,
			 int iterations );

#endif /* _HORATIO_MATH_ */
