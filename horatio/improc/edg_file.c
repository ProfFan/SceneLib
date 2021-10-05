/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

static void write_edgel ( void *ptr, void *data )
{
   Hor_Edgel *eptr = (Hor_Edgel *) ptr;
   FILE *fp = (FILE *) data;

   fprintf ( fp, "%d %d %f %f %f %f\n", eptr->c, eptr->r,
	     eptr->cf, eptr->rf, eptr->angle, eptr->strength );
}

static void write_string ( void *ptr, void *data )
{
   Hor_Estring *sptr = (Hor_Estring *) ptr;
   FILE *fp = (FILE *) data;

   fprintf ( fp, "%d %c\n", sptr->length,
	     (sptr->leftmost == sptr->rightmost) ? 'c' : 'o' );
   hor_dlist_action ( sptr->leftmost, write_edgel, (void *) fp );
}

static Hor_Edgel *read_edgel ( Hor_Edge_Map *edge_map, FILE *fp )
{
   float strength, cf, rf, angle;
   int   c, r;
   Hor_Edgel *edge;

   if ( fscanf ( fp, "%d %d %f %f %f %f\n", &c, &r, &cf, &rf, &angle,
		                            &strength ) != 6 )
      return NULL;

   edge = hor_add_edge ( edge_map, strength, c, r, cf, rf, angle,
			 HOR_NO_ATTRIB, NULL, NULL, NULL );
   edge->status = HOR_IN_STRING;
   return edge;
}

/**********************
*   Hor_Bool      @hor_write_edge_map ( Hor_Edge_Map *edge_map,
*                                      const char *base_name )
*   Hor_Edge_Map *@hor_read_edge_map ( const char *base_name )
*
*   Edge map file I/O functions.
**********************/
Hor_Bool hor_write_edge_map ( Hor_Edge_Map *edge_map, const char *base_name )
{
   char file_name[300];
   FILE *fp;

   if ( edge_map == NULL )
   {
      hor_errno = HOR_IMPROC_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   sprintf ( file_name, "%s.edg", base_name );
   if ( (fp = fopen ( file_name, "w" ) ) == NULL )
   {
      hor_errno = HOR_IMPROC_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   fprintf ( fp, "%d %d %d %d %f %d\n", edge_map->c0, edge_map->r0,
	     edge_map->width, edge_map->height,
	     edge_map->max_strength, edge_map->nstrings );
   hor_list_action ( edge_map->string_list, write_string, (void *) fp );

   fclose ( fp );
   return HOR_TRUE;
}

Hor_Edge_Map *hor_read_edge_map ( const char *base_name )
{
   Hor_Edge_Map *edge_map;
   char          file_name[300];
   Hor_Estring  *string;
   int           i, j, c0, r0, width, height, nstrings, length;
   u_short       max_strength;
   char          closed_flag;
   Hor_Edgel    *edge;
   FILE         *fp;

   sprintf ( file_name, "%s.edg", base_name );
   if ( (fp = fopen ( file_name, "r" ) ) == NULL )
   {
      hor_errno = HOR_IMPROC_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   if ( fscanf ( fp, "%d %d %d %d %hu %d\n", &c0, &r0, &width, &height,
		     &max_strength, &nstrings )
        != 6 )
   {
      hor_errno = HOR_IMPROC_READ_FAILED;
      fclose ( fp );
      return NULL;
   }

   edge_map = hor_alloc_edge_map ( c0, r0, width, height,
				   HOR_NO_ATTRIB, NULL, NULL );
   for ( i = 0; i < nstrings; i++ )
   {
      if ( fscanf ( fp, "%d %c\n", &length, &closed_flag ) != 2 )
      {
	 hor_errno = HOR_IMPROC_READ_FAILED;
	 hor_free_edge_map ( edge_map );
	 fclose ( fp );
	 return NULL;
      }


      if ( (edge = read_edgel ( edge_map, fp )) == NULL )
      {
	 hor_free_edge_map ( edge_map );
	 fclose ( fp );
	 return NULL;
      }

      string = hor_add_estring ( edge_map, edge,
				 HOR_NO_ATTRIB, NULL, NULL, NULL );
      for ( j = 1; j < length; j++ )
      {
	 if ( (edge = read_edgel ( edge_map, fp )) == NULL )
	 {
	    hor_free_edge_map ( edge_map );
	    fclose ( fp );
	    return NULL;
	 }

	 string->rightmost = hor_dinsert_after ( string->rightmost,
						 (void *) edge );
      }

      if ( closed_flag == 'c' ) /* closed contour */
      {
	 string->rightmost->next = string->leftmost;
	 string->leftmost->prev = string->rightmost;
	 string->rightmost = string->leftmost;
      }

      string->length = length;
   }

   fclose ( fp );

   edge_map->string_list = hor_reverse(edge_map->string_list);
   return edge_map;
}
