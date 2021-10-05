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
*   Hor_Edge_Map *@hor_alloc_edge_map ( int c0, int r0, int width, int height,
*                                      int type, void *attrib,
*                                      void (*attrib_free_func)(void *attrib) )
*
*   void @hor_free_edge_map ( Hor_Edge_Map *edge_map )
*   Hor_List @hor_delete_string ( Hor_Edge_Map *edge_map, Hor_List *slist_ptr )
*
*   Canny edge-map allocation/free functions. The attrib and attrib_free_func
*   arguments to hor_alloc_edge_map should be NULL if type is set to
*   HOR_NO_ATTRIB.
*
*   hor_delete_string() deletes the string node pointed to by slist_ptr and
*   returns the next string node. It also deletes the string counter in
*   the edge map.
********************/
Hor_Edge_Map *hor_alloc_edge_map ( int c0, int r0, int width, int height,
				   int type, void *attrib,
				   void (*attrib_free_func)(void *attrib) )
{
   int       r;
   Hor_Edge_Map *edge_map;
   Hor_Edgel  ***edges, **eptr, **epend;

   edge_map = hor_malloc_type ( Hor_Edge_Map );
   edge_map->c0     = c0;
   edge_map->r0     = r0;
   edge_map->width  = width;
   edge_map->height = height;
   edge_map->max_strength = 0.0F;

   edges = hor_malloc_ntype ( Hor_Edgel **, height+2 ) + 1;
   edges[-1] = hor_malloc_ntype ( Hor_Edgel *, (width+2)*(height+2) ) + 1;
   for ( r = 0; r < height+1; r++ )
      edges[r] = edges[r-1] + width + 2;

   /* set all pointers to NULL */
   for ( eptr = edges[-1]-1, epend = eptr + (width+2)*(height+2);
	 eptr != epend; eptr++ )
      *eptr = NULL;

   edge_map->edges = edges;

   edge_map->edge_list   = NULL; edge_map->nedges = 0;
   edge_map->string_list = NULL; edge_map->nstrings = 0;

   /* set user-defined attribute and associated functions */
   edge_map->type             = type;
   edge_map->attrib           = attrib;
   edge_map->attrib_free_func = attrib_free_func;
   return edge_map;
}

static void reset_edge ( void *ptr, void *data )
{
   Hor_Edgel *edge = (Hor_Edgel *) ptr;
   edge->status = HOR_ISOLATED;
   edge->string = NULL;
}

static void free_string ( void *data )
{
   Hor_Estring *string = (Hor_Estring *) data;

   hor_dlist_action ( string->leftmost, reset_edge, NULL );
   hor_dfree_nodes ( string->leftmost );
   if ( string->attrib_free_func != NULL )
      string->attrib_free_func ( string->attrib );

   hor_free ( data );
}

Hor_List hor_delete_estring ( Hor_Edge_Map *edge_map, Hor_List *slist_ptr )
{
   edge_map->nstrings--;
   return ( hor_delete_first ( slist_ptr, (void (*)(void *)) free_string ) );
}

static void free_edge ( void *data )
{
   Hor_Edgel *edge = (Hor_Edgel *) data;

   if ( edge->attrib_free_func != NULL )
      edge->attrib_free_func ( edge->attrib );

   hor_free ( data );
}

void hor_free_edge_map ( Hor_Edge_Map *edge_map )
{
   if ( edge_map == NULL ) return;

   hor_free_list ( edge_map->string_list, free_string );
   hor_free_list ( edge_map->edge_list,   free_edge );
   hor_free ( (void *) (edge_map->edges[-1]-1) );
   hor_free ( (void *) (edge_map->edges-1) );
   if ( edge_map->attrib_free_func != NULL )
      edge_map->attrib_free_func ( edge_map->attrib );

   hor_free ( (void *) edge_map );
}

/*******************
*   Hor_Edgel *@hor_add_edge ( Hor_Edge_Map *map,
*           float strength, (edge strength)
*           int   c,  r;    (edge position w.r.t. the top-left corner
*                            of the edge map)
*           float cf, rf;   (edge position to sub-pixel precision relative
*                            to the top-left hand corner of the image)
*           float angle;    (angle in radians anticlockwise from "east")
*           int   type,     (type of edge, e.g. HOR_NO_ATTRIB)
*           void *attrib,   (user attributes associated with edge)
*           void (*attrib_free_func)   (void *attrib),
*           void (*display_func)(Hor_Edgel *edge, void *params) )
*
*   Adds a edge with given fields to a edge map. The type argument can be
*   HOR_NO_ATTRIB, in which case attrib, attrib_free_func, and display_func
*   should be passed as NULL.
********************/
Hor_Edgel *hor_add_edge ( Hor_Edge_Map *map, float strength,
		    int c, int r, float cf, float rf, float angle,
		    int type, void *attrib,
		    void (*attrib_free_func)(void *attrib),
		    void (*display_func)(Hor_Edgel *edge, void *params) )
{
   Hor_Edgel *edge = hor_malloc_type(Hor_Edgel);
 
   if ( c < 0 || c >= map->width || r < 0 || r >= map->height )
   {
      hor_warning ( "edge position outside edge map (hor_add_edge)" );
      return NULL;
   }

   edge->status = HOR_ISOLATED;
   edge->strength = strength;
   edge->c = c; edge->cf = cf;
   edge->r = r; edge->rf = rf;
   edge->angle = angle;
   edge->type   = type;
   edge->string = NULL;
   edge->attrib = attrib;
   edge->attrib_free_func = attrib_free_func;
   edge->display_func     = display_func;
   map->edge_list = hor_insert ( map->edge_list, edge );
   map->edges[r][c] = edge;
   map->nedges++;
   if ( strength > map->max_strength ) map->max_strength = strength;
   return edge;
}

/*******************
*   Hor_Estring *@hor_add_estring ( Hor_Edge_Map *edge_map,
*           Hor_Edgel *edge,
*           int   type,     (type of edge, e.g. HOR_NO_ATTRIB)
*           void *attrib,   (user attributes associated with edge)
*           void (*attrib_free_func)   (void *attrib),
*           void (*display_func)(Hor_Edgel *edge, void *params) )
*
*   Adds a string with given fields to a edge map. The provided edge is
*   the first edge in the string. The type argument can be
*   HOR_NO_ATTRIB, in which case attrib, attrib_free_func, and display_func
*   should be passed as NULL.
********************/
Hor_Estring *hor_add_estring ( Hor_Edge_Map *edge_map, Hor_Edgel *edge,
			       int type, void *attrib,
			       void (*attrib_free_func)(void *attrib),
			       void (*display_func)(Hor_Estring *edge,
						    void *params) )
{
   Hor_Estring *string = hor_malloc_type(Hor_Estring);

   string->leftmost = string->rightmost = hor_dmake_straight(edge);
   string->length = 1;
   edge_map->string_list = hor_insert ( edge_map->string_list,
				        (void *) string );
   edge_map->nstrings++;

   string->type   = type;
   string->attrib = attrib;
   string->attrib_free_func = attrib_free_func;
   string->display_func     = display_func;
   edge->status = HOR_IN_STRING;
   edge->string = string;
   return string;
}

/*******************
*   void @hor_display_edge ( Hor_Edgel *edge, void *params )
*   void @hor_display_estring ( Hor_Estring *string, void *params )
*   void @hor_display_edge_map ( Hor_Edge_Map *edge_map, int edge_type,
*                               int string_type, void *params )
*
*   Edge/edge map display function. The edge_type and string_type arguments
*   to hor_display_edge_map() specify which types of edge/string are displayed.
*   If either is < 0, all edges/strings are displayed. Edges/strings of
*   different types should be displayed by consecutive calls
*   to hor_display_edge_map().
********************/
void hor_display_edge ( Hor_Edgel *edge, void *params )
{
   hor_display_plot ( edge->cf, edge->rf );
}

static void display_edgel ( void *item, void *data  )
{
   Hor_Edgel *edge = (Hor_Edgel *) item;

   hor_display_plot ( edge->cf, edge->rf );
}

void hor_display_estring ( Hor_Estring *string, void *params )
{
   Hor_ED_Output_Params *ed_params = (Hor_ED_Output_Params *) params;
   Hor_Edgel *edge;

   hor_display_set_colour ( ed_params->string_colour );
   hor_dlist_action ( string->leftmost, display_edgel, NULL );

   if ( string->leftmost != string->rightmost ) /* i.e. not a closed contour */
   {
      hor_display_set_colour ( ed_params->left_term_colour );
      edge = (Hor_Edgel *) string->leftmost->contents;
      hor_display_plot ( edge->cf, edge->rf );

      hor_display_set_colour ( ed_params->right_term_colour );
      edge = (Hor_Edgel *) string->rightmost->contents;
      hor_display_plot ( edge->cf, edge->rf );
   }
}

void hor_display_edge_map ( Hor_Edge_Map *edge_map, int edge_type,
			    int string_type, void *params )
{
   Hor_ED_Output_Params *ed_params = (Hor_ED_Output_Params *) params;

   if ( edge_map == NULL )
   {
      hor_errno = HOR_IMPROC_NULL_POINTER_ARGUMENT;
      return;
   }

   hor_display_highlight_region ( edge_map->c0,    edge_map->r0,
				  edge_map->width, edge_map->height );
   hor_display_set_function ( HOR_DISPLAY_COPY );

   hor_display_set_colour ( ed_params->discard_colour );
   hor_list_action ( edge_map->edge_list, display_edgel, NULL );
   hor_list_action ( edge_map->string_list,
		     (void (*)(void*,void*)) hor_display_estring, params );
}
