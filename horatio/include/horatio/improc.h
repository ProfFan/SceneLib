/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_IMPROC_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/adjust.h */

void hor_adjust_region_for_border ( int  width,
				    int  height,
				    int  lower_c_border, int  upper_c_border,
				    int  lower_r_border, int  upper_r_border,
				    int *c1p,            int *r1p,
				    int *c2p,            int *r2p );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/convolve.h */

#ifdef _HORATIO_IMAGE_
Hor_Bool hor_convolve_image ( Hor_Image *imptr, float *cmask, int csize,
			                        float *rmask, int rsize,
			      Hor_Sub_Image *result );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/gaussian.h */

float *hor_make_gaussian_mask ( float sigma, int gauss_size );
void   hor_free_gaussian_mask ( float *mask, int gauss_size );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/traject.h */

typedef struct
{
   float cf, rf; /* position relative to the top-left corner of the image */
} Hor_Traj_Point;

typedef struct
{
   float c1f, r1f, c2f, r2f; /* positions of the endpoints of line segments
				relative to the top-left corner of the image */
} Hor_Traj_Line;

/* standard Horatio feature matcher trajectory types */
#define HOR_CM_BOG_STANDARD 0 /* bog-standard corner matcher */
#define HOR_CM_BRAIN_DEAD   1 /* Modified Wang/Brady corner matcher */
#define HOR_CM_FMATX        2 /* Fundamental matrix corner matching */
#define HOR_LM_BOG_STANDARD 3 /* bog-standard line segment matcher */

typedef struct _Hor_Trajectory
{
   Hor_Assoc_Label label; /* identifier for this feature trajectory */
   int             type; /* in case multiple trajectory types are present in
			    the same trajectory map. Current standard
			    Horatio trajectory types are HOR_CM_BOG_STANDARD,
			    HOR_CM_BRAIN_DEAD and HOR_LM_BOG_STANDARD. */
   int             status; /* standard values are HOR_UNMATCHED,
			      HOR_PROVISIONAL and HOR_MATCHED. */
   Hor_DList       first; /* first (earliest) element in a list of
			     Hor_Traj_Point's, Hor_Traj_Line's
			     or other types of feature location */
   Hor_DList       last; /* last element in the same list (latest one to be
			    added) */
   int             length; /* length of the above list */
   int             times_seen; /* number of times feature seen == # non-null
				  elements of list */
   int             max_length; /* maximum length of the above list */
   int             not_seen; /* number of frames since this feature was last
				seen, i.e. number of NULL elements since the
				last non-NULL element */

   /* user-defined fields */
   void  *state; /* trajectory state */
   void (*element_free_func)(void *element);
   void (*state_free_func)(void *state);
   void (*display_func)(struct _Hor_Trajectory *traj, void *params);
} Hor_Trajectory;

void hor_add_trajectory_element ( Hor_Trajectory *traj, void *element );

/* standard Horatio trajectory map state types */
#define HOR_TRAJ_MAP_NO_STATE 0

typedef struct
{
   Hor_List traj_list; /* list of trajectories (Hor_Trajectory *`s) */
   int      ntrajs;    /* length of traj_list, i.e. no. of trajectories */

   /* user-defined fields */
   int    type; /* e.g. HOR_TRAJ_MAP_NO_STATE */
   void  *state;
   void (*state_free_func)(void *state);
} Hor_Trajectory_Map;

/* standard feature attribute types */
#define HOR_NO_ATTRIB 0 /* no attribute, attrib field set to NULL */

Hor_Trajectory_Map *hor_alloc_trajectory_map (
		       int    type,
		       void  *state,
		       void (*state_free_func)(void *state) );

void hor_free_trajectory_map ( Hor_Trajectory_Map *map );

Hor_Trajectory *hor_add_new_trajectory (
			      Hor_Trajectory_Map *map,
			      int    type,
			      void  *first_element,
			      int    max_length,
			      void  *state,
			      void (*element_free_func)(void *element),
			      void (*state_free_func)(void *state),
			      void (*display_func)(Hor_Trajectory *traj,
						   void *params) );
void hor_delete_old_trajectories ( Hor_Trajectory_Map *map,
				   Hor_Bool (*test_func)(Hor_Trajectory *traj,
							 void *data),
				   void *data );
int hor_count_trajectories ( Hor_Trajectory_Map *trajectory_map, int type );
Hor_Trajectory *hor_find_trajectory ( Hor_Trajectory_Map *map,
				      Hor_Assoc_Label label );
void hor_reset_trajectory_status ( Hor_Trajectory_Map *map,
				   int type, int status );

/* standard Horatio feature status values */
#define HOR_UNMATCHED   0 /* for an unmatched feature */
#define HOR_PROVISIONAL 1 /* for a provisionally matched feature */
#define HOR_MATCHED     2 /* for a matched feature */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/tr_disp.h */

void hor_display_trajectory_map ( Hor_Trajectory_Map *map,
				  int type, void *params );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/edge.h */

/* Strings are a list of (Hor_Estring *)'s */
typedef struct _Hor_Estring
{
   Hor_DList leftmost, rightmost; /* string termination points */
   int       length;              /* number of edges in string */

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
   void (*display_func)(struct _Hor_Estring *string, void *params);
} Hor_Estring;

typedef struct _Hor_Edgel
{
   int   status;   /* for temporary use by string finder */
   float strength; /* typically the square of the image gradient at the edge */
   int   r,  c;    /* physical row and col w.r.t. c0, r0 of edge map */
   float rf, cf;   /* sub pixel accuracy location w.r.t. image array */
   float angle;    /* edge orientation in radians anticlockwise from "east" */
   Hor_Estring *string; /* string containing this edge, if any. If no string,
			   set this to NULL */

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
   void (*display_func)(struct _Hor_Edgel *edge, void *params);
} Hor_Edgel;

/* edge status field values */
#define HOR_ISOLATED  0
#define HOR_IN_STRING 1

typedef struct
{
   int c0,    r0;      /* topleft position relative to parent image */
   int width, height;  /* width, height of rectangular ROI */
   float max_strength; /* the maximum edge strength */
   Hor_Edgel ***edges; /* array of pointers to edgels */
   Hor_List edge_list;   int nedges;   /* list of and number of edges */
   Hor_List string_list; int nstrings; /* list of and number of edge strings */

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
} Hor_Edge_Map;

Hor_Edge_Map *hor_alloc_edge_map ( int c0, int r0, int width, int height,
				   int type, void *attrib,
				   void (*attrib_free_func)(void *attrib) );
void          hor_free_edge_map  ( Hor_Edge_Map * );
Hor_List      hor_delete_estring ( Hor_Edge_Map *, Hor_List *slist_ptr );

Hor_Edgel *hor_add_edge ( Hor_Edge_Map *map, float strength,
			  int c, int r, float cf, float rf, float angle,
			  int type, void *attrib,
			  void (*attrib_free_func)(void *attrib),
			  void (*display_func)(Hor_Edgel *edge, void *params));
Hor_Estring *hor_add_estring ( Hor_Edge_Map *map, Hor_Edgel *edge,
			       int type, void *attrib,
			       void (*attrib_free_func)(void *attrib),
			       void (*display_func)(Hor_Estring *edge,
						    void *params) );

/*******************
*   typedef struct
*   {
*      u_long string_colour;     (colour for Canny edge strings)
*      u_long discard_colour;    (colour for discarded edges)
*      u_long left_term_colour;  (colour for string left-termination points)
*      u_long right_term_colour; (colour for string right-termination points)
*   } @Hor_ED_Output_Params;
*
*   Standard edge map output parameter structure definition, to be used in
*   conjunction with hor_display_edge_map().
********************/
typedef struct
{
   u_long string_colour, discard_colour, left_term_colour, right_term_colour;
} Hor_ED_Output_Params;

void hor_display_edge ( Hor_Edgel *edge, void *params );
void hor_display_estring ( Hor_Estring *string, void *params );
void hor_display_edge_map ( Hor_Edge_Map *edge_map, int edge_type,
			    int string_type, void *params );


/* Canny edge orientation attribute code: orientation in 10 degree units
   anticlockwise from "west". Alternative edge attribute code is
   HOR_NO_ATTRIB */
#define HOR_ORIENT_ATTRIB 1
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/edg_file.h */

Hor_Bool      hor_write_edge_map ( Hor_Edge_Map *edge_map,
				   const char *base_name );
Hor_Edge_Map *hor_read_edge_map  ( const char *base_name );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/edge.h */

#ifdef _HORATIO_IMAGE_
Hor_Edge_Map *hor_canny ( Hor_Image *imptr, float *gauss_mask, int gauss_size,
			  float low_thres, float high_thres,
			  int   string_length_thres, int c1, int r1,
			                             int c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
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
/* Copyright 1994 David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/line_fit.h */

Hor_Line_Segment_Map *hor_find_lines ( Hor_Edge_Map *edge_map, 
				       int no_init_points,
				       float sigma, float thres );
Hor_Line_Segment_Map *hor_find_lines_rec ( Hor_Line_Segment_Map* line_map,
					   int no_init_points,
					   float sigma, float thres );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/line_seg.h */

Hor_Line_Segment_Map *hor_fit_line_segments ( Hor_Edge_Map *edge_map, double lambda_lim );

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/li_match.h */

typedef struct
{
   int    trajectory_length;
   u_long last_colour, prev_colour, join_colour;
} Hor_LM_Output_Params;

void hor_lm_traj_display ( Hor_Trajectory *traj, void *params );
/* params should be a pointer to a Hor_LM_Display_Params structure */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bog_lm.h */

typedef struct
{
   /* temporary variables used by the matcher */
   double mismatch_strength;
} Hor_Bog_LM_Traj_State;

int  hor_bog_lm_traj_count ( Hor_Trajectory_Map *map );
void hor_bog_match_lines ( Hor_Trajectory_Map   *map,
			   Hor_Line_Segment_Map *new_lines,

			   /* parameters */
			   int   max_dist,
			   float cos_thresh,
			   float size_thresh,
			   int   iterations );

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/corner.h */

#ifdef _HORATIO_IMAGE_
typedef struct _Hor_Corner
{
   int   status; /* for use by corner matcher. Standard values are
		    HOR_UNMATCHED, HOR_PROVISIONAL and HOR_MATCHED. */
   float strength; /* corner strength */
   float cf, rf;   /* corner position to sub-pixel precision relative to the
		      top-left hand corner of the image */
   Hor_Sub_Image *patch; /* image patch around corner */

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
   void (*display_func)(struct _Hor_Corner *corner, void *params);
} Hor_Corner;

typedef struct
{
   int  c0,    r0;     /* the top-left position relative to parent image */
   int  width, height; /* the width, height of this rectangle */
   float max_strength; /* maximum corner strength */
   Hor_Corner ***corners;
   Hor_List      corner_list;
   int           ncorners;

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
} Hor_Corner_Map;

Hor_Corner_Map *hor_alloc_corner_map (int c0, int r0, int width, int height,
				      int type, void *attrib,
				      void (*attrib_free_func)(void *attrib));
void hor_free_corner_map ( Hor_Corner_Map *corner_map );

Hor_Corner *hor_add_corner ( Hor_Corner_Map *map, float strength,
			     float cf, float rf,
			     Hor_Image *image, int    c0, int     r0,
			                       int width, int height,
			     int type, void *attrib,
			     void (*attrib_free_func)(void *attrib),
			     void (*display_func)(Hor_Corner *corner,
						  void *params) );
#endif /* _HORATIO_IMAGE_ */
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/cor_file.h */

#ifdef _HORATIO_IMAGE_
Hor_Bool        hor_write_corner_map ( Hor_Corner_Map *,
				       const char *base_name );
Hor_Corner_Map *hor_read_corner_map  ( const char *base_name );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/plessey.h */

#ifdef _HORATIO_IMAGE_
Hor_Corner_Map *hor_plessey_corners ( Hor_Image *image,
				      float *gauss_mask, int gauss_size,
				      float  thres,      int patch_size,
				      int c1, int r1, int c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/smith.h */

#ifdef _HORATIO_IMAGE_
Hor_Corner_Map *hor_smith_corners ( Hor_Image *image,
				    int diff_thres, int geom_thres,
				    int patch_size,
				    int c1, int r1, int c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/wang.h */

#ifdef _HORATIO_IMAGE_
Hor_Corner_Map *hor_wang_corners ( Hor_Image *imptr,
				   float  edge_thresh,
				   float  corner_thresh,
				   int    patch_size,
				   int    c1, int r1,
				   int    c2, int r2 );
#endif /* _HORATIO_IMAGE_ */
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/co_match.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   int    trajectory_length;
   u_long trajectory_colour;
   u_long dot_colour;
} Hor_CM_Output_Params;

void hor_cm_traj_display ( Hor_Trajectory *traj, void *params );
/* params should be a pointer to a Hor_CM_Display_Params structure */

#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bog_cm.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   /* temporary variables used by the matcher */
   float imdiff_ratio;

   /* small image patch around the position of the latest corner feature */
   Hor_Sub_Image *patch;
} Hor_Bog_CM_Traj_State;

void hor_bog_cm_traj_state_free ( void *state );
void hor_bog_match_corners ( Hor_Trajectory_Map *map,
			     Hor_Corner_Map     *new_corner_map,

			     /* parameters */
			     int   range,
			     float imdiff_ratio_thres,
			     int   iterations );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/bd_cm.h */

#ifdef _HORATIO_IMAGE_

#define HOR_BD_CM_MAX_HISTORY 3

typedef struct
{
   int   n;
   float x[HOR_BD_CM_MAX_HISTORY],
         y[HOR_BD_CM_MAX_HISTORY];
   float corr; /* correlation value between images patches around corners */
   float innovation;
   float covar[4][4];
   float state[4];  
   float speed;
   int   age;

   /* small image patch around the position of the latest corner feature */
   Hor_Sub_Image *patch;
} Hor_BD_CM_Traj_State;

void hor_bd_cm_traj_state_free ( void *state );
void hor_bd_match_corners ( Hor_Trajectory_Map *map,
			    Hor_Corner_Map     *new_corner_map,

			    /* parameters */
			    int   match_window,
			    float corr_thresh );
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/flow.h */

#ifdef _HORATIO_IMAGE_
#ifndef HOR_TRANSPUTER

/*******************
*   typedef struct
*   {
*      Hor_Sub_Image *old_image;
*      Hor_Sub_Image *flow_flag;
*      Hor_Sub_Image *flow_c;
*      Hor_Sub_Image *flow_r;
*   } @Hor_Image_Flow;
*
*   Image flow result structure definition.
********************/
typedef struct
{
   Hor_Sub_Image *old_image, *flow_flag, *flow_c, *flow_r;
} Hor_Image_Flow;

Hor_Image_Flow *hor_image_flow ( Hor_Image     *imptr,
				 Hor_Sub_Image *old_image,
				 float     *gauss_mask, int gauss_size,
				 float      rc_gradient_threshold,
				 float      t_gradient_threshold,
				 int        c1, int r1,
				 int        c2, int r2 );

void hor_free_image_flow ( Hor_Image_Flow *result );

#endif /* !HOR_TRANSPUTER */
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/fl_disp.h */

#ifdef _HORATIO_IMAGE_
void hor_display_image_flow (
   Hor_Sub_Image *flow_c,
   Hor_Sub_Image *flow_r,   /* x & y components of normal flow vector field */
   Hor_Sub_Image *flow_flag,/* specifies which vectors to believe */
   u_long     start_colour, /* colour of dot plotted at start of vector */
   u_long     line_colour,  /* colour of lines to represent vectors */
   float      scale,        /* factor by which to multiply length of lines to
			       improve visibility */
   int        increment );  /* pixel distance between displayed samples of
			       vector field */
#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/segment.h */

#ifdef _HORATIO_IMAGE_

typedef struct
{
   int dummy;
} Hor_Image_Segment;

Hor_Image_Segment *hor_image_segment ( Hor_Image *image,
				       int    c1, int r1, int c2, int r2,
				       int    patch_size, int patch_density,
				       float  low_threshold,
				       float  threshold_step,
				       float  high_threshold,
				       float  sd_scale,
				       u_long thres_colour );

#endif /* _HORATIO_IMAGE_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/correl.h */

#ifdef _HORATIO_IMAGE_

/*******************
*   typedef struct
*   {
*      Hor_Sub_Image *image;
*   } @Hor_Correlation;
*
*   Correlation result structure definition.
********************/
typedef struct
{
   Hor_Sub_Image *image;
} Hor_Correlation;

Hor_Correlation *hor_correlation (Hor_Image *image, Hor_Sub_Image *old_image,
				  int    c1, int r1, int c2, int r2,
				  int    patch_size, int patch_density,
				  int    max_motion,
				  float  display_scale, u_long display_colour);

#endif /* _HORATIO_IMAGE_ */
