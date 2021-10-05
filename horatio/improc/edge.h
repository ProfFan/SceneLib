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
