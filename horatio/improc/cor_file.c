/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"

/**********************
*   Hor_Bool @hor_write_corner_map ( Hor_Corner_Map *corner_map,
*                                   const char *base_name)
*   Hor_Corner_Map *@hor_read_corner_map ( const char *base_name )
*
*   Corner map file I/O functions.
**********************/
Hor_Bool hor_write_corner_map ( Hor_Corner_Map *corner_map,
			        const char *base_name )
{
   Hor_List    list;
   Hor_Corner *cptr;
   FILE       *fp;
   char        file_name[300];

   if ( corner_map == NULL )
   {
      hor_errno = HOR_IMPROC_NULL_POINTER_ARGUMENT;
      return HOR_FALSE;
   }

   sprintf ( file_name, "%s.cor", base_name );
   if ( (fp = fopen ( file_name, "w" ) ) == NULL )
   {
      hor_errno = HOR_IMPROC_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   fprintf ( fp, "%d %d %d %d %d\n", corner_map->c0, corner_map->r0,
	     corner_map->width, corner_map->height, corner_map->ncorners );
   for ( list = corner_map->corner_list; list != NULL; list = list->next )
   {
      cptr = (Hor_Corner *) list->contents;
      fprintf ( fp, "%f %f %f\n", cptr->cf, cptr->rf, cptr->strength );
   }

   fclose ( fp );
   return HOR_TRUE;
}

Hor_Corner_Map *hor_read_corner_map ( const char *base_name )
{
   Hor_Corner_Map *map;
   Hor_Image      *image;
   FILE           *fp;
   char            file_name[300];
   int             c0, r0, width, height, ncorners, i, c, r;
   float           strength, cf, rf;

   sprintf ( file_name, "%s.cor", base_name );
   if ( (fp = fopen ( file_name, "r" ) ) == NULL )
   {
      hor_errno = HOR_IMPROC_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   if ( fscanf ( fp, "%d %d %d %d %d\n", &c0, &r0, &width, &height, &ncorners )
        != 5 )
   {
      hor_errno = HOR_IMPROC_READ_FAILED;
      fclose ( fp );
      return NULL;
   }

   map = hor_alloc_corner_map ( c0, r0, width, height,
			        HOR_NO_ATTRIB, NULL, NULL );

   /* construct an artificial black image to sample */
   image = hor_alloc_image ( 1, 1, HOR_U_CHAR, NULL );
   image->array.uc[0][0] = 0;

   for ( i = 0; i < ncorners; i++ )
   {
      if ( fscanf ( fp, "%f %f %f\n", &cf, &rf, &strength ) != 3 )
      {
	 hor_errno = HOR_IMPROC_READ_FAILED;
	 hor_free_corner_map ( map );
	 fclose ( fp );
	 return NULL;
      }

      c = (int) cf - c0;
      r = (int) rf - r0;
      if ( c < 0 || c >= width || r < 0 || r > height )
      {
	 hor_errno = HOR_IMPROC_ILLEGAL_CORNER_POSITION;
         hor_free_corner_map ( map );
         fclose ( fp );
         return NULL;
      }

      hor_add_corner ( map, strength, cf, rf, image, 0, 0, 1, 1,
		       HOR_NO_ATTRIB, NULL, NULL, hor_display_corner );
   }

   fclose ( fp );
   return map;
}
