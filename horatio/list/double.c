/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* double_list.c: module for manipulating doubly linked lists */
#include <stdlib.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/list.h"

/*******************
*   Hor_DList @hor_dmake_straight ( void *data )
*   Hor_DList @hor_dmake_circular ( void *data )
*   Hor_DList @hor_dinsert_before ( Hor_DList list, void *data )
*   Hor_DList @hor_dinsert_after  ( Hor_DList list, void *data )
*   void      @hor_dfree_list     ( Hor_DList list,
*                                  void (*free_func)(void *) )
*   void      @hor_dfree_list_before ( Hor_DList list,
*                                     void (*free_func)(void *) )
*   void      @hor_dfree_list_after  ( Hor_DList list,
*                                     void (*free_func)(void *) )
*   void      @hor_dfree_nodes  ( Hor_DList list )
*   void      @Hor_DList_action ( Hor_DList list,
*                                 void (*act_func)(void *item, void *data),
*                                 void * )
*
*   Routines to manipulate doubly-linked lists.
*
*   hor_dmake_straight() initialises a doubly linked list with two endpoints.
*   hor_dmake_circular() initialises doubly linked list with no endpoints,
*                        i.e. effectively a circular list.
*
*   hor_dinsert_before() inserts a new node into list before a specified node.
*   hor_dinsert_after() inserts a new node into list after a specified node.
*
*   hor_dfree_list() frees all the data and nodes associated with a list.
*                    Specified node can be anywhere in the list.
*
*   hor_dfree_list_before() frees the data and nodes before the given node.
*   hor_dfree_list_after()  frees the data and nodes after the given node.
*
*   hor_dfree_nodes() frees all the nodes in a list, but not the data.
*                     Specified node can be anywhere in the list.
*
*   hor_dlist_action() performs action specified by act_func() on all data in
*                      list. The user pointer "data" is also passed
*                      to act_func().
********************/
Hor_DList hor_dmake_straight ( void *data )
{
   Hor_DList new_node;

   new_node = hor_malloc_type(Hor_DNode);
   new_node->contents = data;
   new_node->prev = new_node->next = NULL;
   return new_node;
}

Hor_DList hor_dmake_circular ( void *data )
{
   Hor_DList new_node;

   new_node = hor_malloc_type(Hor_DNode);
   new_node->contents = data;
   new_node->prev = new_node->next = new_node;
   return new_node;
}

Hor_DList hor_dinsert_before ( Hor_DList list, void *data )
{
   Hor_DList new_node;

   if ( list == NULL ) return NULL;

   new_node = hor_malloc_type(Hor_DNode);
   new_node->contents = data;
   new_node->prev     = list->prev;
   if ( list->prev != NULL ) list->prev->next = new_node;

   new_node->next = list;
   list->prev     = new_node;
   return new_node;
}

Hor_DList hor_dinsert_after ( Hor_DList list, void *data )
{
   Hor_DList new_node;

   if ( list == NULL ) return NULL;

   new_node = hor_malloc_type(Hor_DNode);
   new_node->contents = data;
   new_node->prev     = list;
   if ( list->next != NULL ) list->next->prev = new_node;

   new_node->next = list->next;
   list->next     = new_node;
   return new_node;
}

/*   hor_dfree_list_backwards()

        frees the data and nodes in a list found by traversing the list
	backwards from a specified node. */
static Hor_DList hor_dfree_list_backwards ( Hor_DList list,
					    Hor_DList stop_node,
					    void (*free_func)(void *) )
{
   Hor_DList result;

   if ( list == NULL || list == stop_node ) return list;

   if ( free_func != NULL ) free_func ( list->contents );

   result = hor_dfree_list_backwards ( list->prev, stop_node, free_func );
   hor_free ( (void *) list );
   return result;
}

/*   hor_dfree_list_forwards()

        frees the data and nodes in a list found by traversing the list
	forwards from a specified node. */
static Hor_DList hor_dfree_list_forwards ( Hor_DList list, Hor_DList stop_node,
					   void (*free_func)(void *) )
{
   Hor_DList result;

   if ( list == NULL || list == stop_node ) return list;

   if ( free_func != NULL ) free_func ( list->contents );

   result = hor_dfree_list_forwards ( list->next, stop_node, free_func );
   hor_free ( (void *) list );
   return result;
}

void hor_dfree_list ( Hor_DList list, void (*free_func)(void *) )
{
   if ( list == NULL ) return;

   if ( free_func != NULL ) free_func ( list->contents );

   if ( hor_dfree_list_backwards ( list->prev,list, free_func ) == NULL )
      hor_dfree_list_forwards ( list->next, list, free_func );

   hor_free ( (void *) list );
}

void hor_dfree_list_before ( Hor_DList list, void (*free_func)(void *) )
{
   if ( list == NULL ) return;

   if ( hor_dfree_list_backwards ( list->prev, list, free_func ) == NULL )
      list->prev = NULL; /* terminated list */
   else
   { list->prev = list->next = NULL; } /* circular list */
}

void hor_dfree_list_after ( Hor_DList list, void (*free_func)(void *) )
{
   if ( list == NULL ) return;

   if ( hor_dfree_list_forwards ( list->next, list, free_func ) == NULL )
      list->next = NULL; /* terminated list */
   else
   { list->prev = list->next = NULL; } /* circular list */
}

/* hor_dlist_action_backwards() hor_dlist_action_forwards()

      Perform action specified by act_func() on all the data found traversing a
      list backwards/forwards from the specified node. Returns the node at
      which the function terminates, either NULL or the specified stop node. */
static Hor_DList hor_dlist_action_backwards ( Hor_DList  list,
					      Hor_DList  stop_node,
					      void (*act_func)(void *, void *),
					      void *data )
{
   if ( list == NULL || list == stop_node ) return list;

   act_func ( list->contents, data );
   return ( hor_dlist_action_backwards ( list->prev, stop_node, act_func,
					 data ) );
}

static Hor_DList hor_dlist_action_forwards ( Hor_DList list,
					     Hor_DList stop_node,
					     void (*act_func)(void *, void *),
					     void *data )
{
   if ( list == NULL || list == stop_node ) return list;

   act_func ( list->contents, data );
   return ( hor_dlist_action_forwards ( list->next, stop_node, act_func,
				        data ) );
}

void hor_dlist_action ( Hor_DList list, void (*act_func)(void *, void *),
		        void *data )
{
   if ( list == NULL ) return;

   act_func ( list->contents, data );
   if ( hor_dlist_action_backwards (list->prev, list, act_func, data) == NULL )
      hor_dlist_action_forwards ( list->next, list, act_func, data );
}

/*******************
*   Hor_DList @hor_dmake_straight_from_list ( Hor_List list )
*   Hor_DList @hor_dmake_circular_from_list ( Hor_List list )
*
*   Converts a singly-linked list into a doubly-linked list, respectively
*   straight (terminated) and circular (unterminated).
********************/
Hor_DList hor_dmake_straight_from_list ( Hor_List list )
{
   Hor_DList result, dlist;

   if ( list == NULL ) return NULL;

   result = hor_dmake_straight ( list->contents );
   for ( list = list->next, dlist = result; list != NULL; list = list->next )
      dlist = hor_dinsert_after ( dlist, list->contents );

   return result;
}

Hor_DList hor_dmake_circular_from_list ( Hor_List list )
{
   Hor_DList result, dlist;

   if ( list == NULL ) return NULL;

   result = hor_dmake_circular ( list->contents );
   for ( list = list->next, dlist = result; list != NULL; list = list->next )
      dlist = hor_dinsert_after ( dlist, list->contents );

   return result;
}
