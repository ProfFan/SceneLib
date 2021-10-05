/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stddef.h>
#ifdef HOR_TRANSPUTER
#include <channel.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"

#ifdef HOR_TRANSPUTER

/*******************
*   Hor_List @HorChanInList  ( Channel *in, int no_nodes,
*                             void * (*input_func)(Channel *),
*                             void   (*free_func)(void *) )
*   void @HorChanOutList ( Channel *out, Hor_List list,
*                         void (*output_func)(Channel *,void *data) )
*
*   Functions to input/output list of objects. For transputers only.
********************/
Hor_List HorChanInList ( Channel *in, int no_nodes,
			 void * (*input_func)(Channel *),
			 void (*free_func)(void *) )
{
   Hor_List list = NULL;
   void    *data;
   int      count;

   for ( count = 0; count < no_nodes; count++ )
   {
      data = input_func ( in );
      if ( data == NULL )
      {
	 hor_free_list ( list, free_func );
	 return NULL;
      }

      list = hor_insert ( list, data );
   }

   return list;
}

void HorChanOutList ( Channel *out, Hor_List list,
		      void (*output_func)(Channel *,void *data) )
{
   for ( ; hor_list_non_null(list); list = hor_next_node(list) )
      output_func ( out, hor_node_contents(list) );
}

#endif /* HOR_TRANSPUTER */

#ifndef HOR_REDUCED_LIB

/*******************
*   Hor_List @hor_read_list ( int fd, int no_nodes, void * (*input_func)(int),
*                            void (*free_func)(void *) )
*   void @hor_write_list ( int fd, Hor_List list,
*                         void (*output_func)(int,void *data) )
*
*   List stream I/O functions.
********************/
Hor_List hor_read_list ( int fd, int no_nodes, void * (*input_func)(int),
			 void (*free_func)(void *) )
{
   Hor_List list = NULL;
   void    *data;
   int      count;

   for ( count = 0; count < no_nodes; count++ )
   {
      data = input_func ( fd );
      if ( data == NULL )
      {
	 hor_free_list ( list, free_func );
	 return NULL;
      }

      list = hor_insert ( list, data );
   }

   return list;
}

void hor_write_list ( int fd, Hor_List list,
		      void (*output_func)(int,void *data) )
{
   for ( ; hor_list_non_null(list); list = hor_next_node(list) )
      output_func ( fd, list->contents );
}

#endif /* HOR_REDUCED_LIB */
