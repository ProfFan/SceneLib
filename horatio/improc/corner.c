/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   Hor_Corner_Map *@hor_alloc_corner_map (
*      int    c0,    int r0,     (offsets of the corner map from the top-left
*                                 corner of the image)
*      int    width, int height, (dimensions of the corner map)
*      int    type,              (type of corner map, e.g. HOR_NO_ATTRIB)
*      void  *attrib,            (user-defined corner map attributes)
*      void (*attrib_free_func)(void *attrib) )
*
*   void @hor_free_corner_map  ( Hor_Corner_Map *corner_map )
*
*   Corner map allocation/free functions. The type argument to
*   hor_alloc_corner_map() can be HOR_NO_ATTRIB, in which case attrib and
*   attrib_free_func should be passed as NULL.
********************/
Hor_Corner_Map *hor_alloc_corner_map ( int c0, int r0, int width, int height,
				       int type, void *attrib,
				       void (*attrib_free_func)(void *attrib) )
{
   Hor_Corner_Map *map;
   Hor_Corner   ***corners, **cptr, **cpend;
   int             r;

   map = hor_malloc_type ( Hor_Corner_Map );
   map->c0           = c0;
   map->r0           = r0;
   map->width        = width;
   map->height       = height;
   map->max_strength = 0.0F;
   map->corner_list  = NULL;
   map->ncorners     = 0;

   corners = hor_malloc_ntype ( Hor_Corner **, height );
   corners[0] = hor_malloc_ntype ( Hor_Corner *, width*height );
   for ( r = 1; r < height; r++ )
      corners[r] = corners[r-1] + width;

   /* set all pointers to NULL */
   for ( cptr = corners[0], cpend = cptr + width*height; cptr != cpend; cptr++)
      *cptr = NULL;

   map->corners = corners;

   /* set user-defined attribute and associated functions */
   map->type             = type;
   map->attrib           = attrib;
   map->attrib_free_func = attrib_free_func;
   return map;
}

static void free_corner ( void *data )
{
   Hor_Corner *corner = (Hor_Corner *) data;

   if ( corner->attrib_free_func != NULL )
      corner->attrib_free_func ( corner->attrib );

   hor_free_sub_image ( corner->patch );
   hor_free ( data );
}

void hor_free_corner_map ( Hor_Corner_Map *map )
{
   if ( map == NULL ) return;

   hor_free ( (void *) map->corners[0] );
   hor_free ( (void *) map->corners );
   hor_free_list ( map->corner_list, free_corner );
   if ( map->attrib_free_func != NULL ) map->attrib_free_func ( map->attrib );

   hor_free ( (void *) map );
}

/*******************
*   Hor_Corner *@hor_add_corner ( Hor_Corner_Map *map,
*           float strength,   (corner strength)
*           float cf, rf;     (corner position to sub-pixel precision relative
*                              to the top-left hand corner of the image)
*           Hor_Image *image, (image from which patch around corner is to
*                              be read)
*           int c0,    int r0,     (offset and dimensions of rectangular
*           int width, int height,  section of image around corner)
*           int   type,   (type of corner, e.g. HOR_NO_ATTRIB)
*           void *attrib, (user attributes associated with corner)
*           void (*attrib_free_func)   (void *attrib),
*           void (*display_func)(Hor_Corner *corner, void *params) )
*
*   Adds a corner with given fields to a corner map. The type argument
*   can be HOR_NO_ATTRIB, in which case attrib, attrib_free_func and
*   display_func should be passed as NULL.
********************/
Hor_Corner *hor_add_corner ( Hor_Corner_Map *map, float strength,
			     float cf, float rf,
			     Hor_Image *image, int c0,    int r0,
			                       int width, int height,
			     int type, void *attrib,
			     void (*attrib_free_func)   (void *attrib),
			     void (*display_func)(Hor_Corner *corner,
						  void *params) )
{
   Hor_Corner *corner = hor_malloc_type(Hor_Corner);
   int c = (int) cf - map->c0, r = (int) rf - map->r0;

   if ( c < 0 || c >= map->width || r < 0 || r >= map->height )
   {
      hor_warning ( "corner position outside corner map (hor_add_corner)" );
      return NULL;
   }

   corner->status = HOR_UNMATCHED;
   corner->strength = strength;
   corner->cf = cf;
   corner->rf = rf;
   if ( image == NULL ) corner->patch = NULL;
   else
   {
      corner->patch = hor_extract_from_image ( image, c0, r0, width, height );
      if ( corner->patch == NULL ) /* probably means that the patch goes
				      outside the image: abort */
      {
	 hor_free ( (void *) corner );
	 return NULL;
      }
   }

   corner->type   = type;
   corner->attrib = attrib;
   corner->attrib_free_func = attrib_free_func;
   corner->display_func     = display_func;
   map->corner_list = hor_insert ( map->corner_list, corner );
   map->corners[r][c] = corner;
   map->ncorners++;
   if ( strength > map->max_strength ) map->max_strength = strength;
   return corner;
}
