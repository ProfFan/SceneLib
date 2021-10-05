/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* single_list.c: module for manipulation of general singly linked lists */
#include <stdlib.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/list.h"

/* hor_append_list()

      Appends list to list pointed to by list_ptr. */
static void hor_append_list ( Hor_List *list_ptr, Hor_List list )
{
   if ( *list_ptr == NULL ) *list_ptr = list;
   else
      hor_append_list ( &(*list_ptr)->next, list );
}

/*******************
*   Hor_List @hor_insert          ( Hor_List list, void *data )
*   Hor_List @hor_append          ( Hor_List list, void *data )
*   Hor_List @hor_concat          ( Hor_List list1, Hor_List list2 )
*   void     @hor_transfer        ( Hor_List *from, Hor_List *to )
*   void     @hor_free_list       ( Hor_List list, void (*free_func)(void *) )
*   void     @hor_free_nodes      ( Hor_List list )
*   Hor_List @hor_reverse         ( Hor_List list )
*   Hor_List @hor_reverse_no_free ( Hor_List list )
*   void     @hor_list_action     ( Hor_List list,
*                                   void (*act_func)(void *item, void *data),
*                                   void *data )
*   Hor_List @hor_delete_first    ( Hor_List *nodepp,
*                                  void (*free_func)(void *) )
*   Hor_List @hor_delete_next     ( Hor_List *nodepp,
*                                  void (*free_func)(void *) )
*   int  @hor_list_size           ( Hor_List list )
*
*   Routines to manipulate singly-linked lists.
*   hor_insert() adds new data item to the start of a list and returns the
*                extended list.
*   hor_append() adds new data item to the end of a list and returns the
*                extended list. Less efficient than hor_insert().
*   hor_concat() concatenates list2 to the end of list1 and returns the
*                whole list list1.
*   hor_transfer() transfers the first element of the "from" list to the
*                  start of the "to" list. No data is allocated or freed.
*   hor_free_list() frees the data and nodes associated with a list, using the
*                   function free_func() to free the node contents.
*   hor_free_nodes() frees just the nodes (not the data) associated with a
*                    list.
*   hor_reverse() returns the reverse of a list, and frees the nodes associated
*                 with the old list.
*   hor_reverse_no_free() does the same as hor_reverse(), but doesn't free the
*                         nodes of the old list.
*   hor_list_action() performs an action on all the members of a list,
*                     the action being defined by the function act_func().
*                     The user pointer "data" is also passed to act_func().
*   hor_delete_first() deletes the first element of a list and returns the
*                      remainder.
*   hor_delete_next() deletes the second element of the given list and joins
*                     the first element to the third.
*
*   hor_list_size() returns the number of elements in a list.
********************/
Hor_List hor_insert ( Hor_List list, void *data )
{
   Hor_List new_node = hor_malloc_type(Hor_Node);

   new_node->contents = data;
   new_node->next = list;
   return new_node;
}

Hor_List hor_append ( Hor_List list, void *data )
{
   Hor_List new_node = hor_malloc_type(Hor_Node);

   new_node->contents = data;
   new_node->next = NULL;
   hor_append_list ( &list, new_node );
   return list;
}

Hor_List hor_concat ( Hor_List list1, Hor_List list2 )
{
   hor_append_list ( &list1, list2 );
   return list1;
}

void hor_transfer ( Hor_List *from, Hor_List *to )
{
   Hor_List to_old = *to;

   *to = *from;
   *from = (*from)->next;
   (*to)->next = to_old;
}

void hor_free_list ( Hor_List list, void (*free_func)(void *) )
{
   Hor_List next;

   while ( list != NULL )
   {
      if ( free_func != NULL ) free_func ( list->contents );

      next = list->next;
      hor_free ( (void *) list );
      list = next;
   }
}

static Hor_List add_list_node_to_start ( Hor_List list, Hor_List tail )
{
   Hor_List new_node;

   if ( hor_list_null(list) ) return tail;

   new_node = hor_malloc_type(Hor_Node);
   new_node->contents = list->contents;
   new_node->next = tail;
   return ( add_list_node_to_start ( list->next, new_node ) );
}

Hor_List hor_reverse ( Hor_List list )
{
   Hor_List result;

   result = add_list_node_to_start ( list, NULL );
   hor_free_nodes ( list );
   return result;
}

Hor_List hor_reverse_no_free ( Hor_List list )
{
   Hor_List result;

   result = add_list_node_to_start ( list, NULL );
   return result;
}

void hor_list_action ( Hor_List list, void (*act_func)(void *, void *),
		       void *data )
{
   for ( ; list != NULL; list = list->next ) act_func ( list->contents, data );
}

Hor_List hor_delete_first ( Hor_List *nodepp, void (*free_func)(void *) )
{
   Hor_List temp;

   temp = *nodepp;
   if ( temp == NULL ) return NULL;

   if ( free_func != NULL ) free_func ( temp->contents );

   *nodepp = temp->next;
   hor_free ( (void *) temp );
   return ( *nodepp );
}

void hor_delete_next ( Hor_List list, void (*free_func)(void *) )
{
   Hor_List temp;

   if ( list == NULL )
      hor_error ( "NULL passed to hor_delete_next()", HOR_FATAL );

   if ( list->next == NULL ) return;

   if ( free_func != NULL ) free_func ( list->next->contents );
   temp = list->next;
   list->next = list->next->next;
   hor_free ( (void *) temp );
}

int hor_list_size ( Hor_List list )
{
   if ( list == NULL ) return 0;
   else return ( 1 + hor_list_size ( list->next ) );
}

/*******************
*   void     @hor_list_to_array ( Hor_List list, void ***array_ptr, int *n_ptr)
*   Hor_List @hor_array_to_list ( void **array, int n )
*
*   Hor_List <-> array conversion functions.
*
*   hor_list_to_array() converts the data associated with a list into an array.
*   hor_array_to_list() does the reverse.
********************/
void hor_list_to_array ( Hor_List list, void ***array_ptr, int *n_ptr )
{
   int    i, n;
   void **array;

   *n_ptr     = n     = hor_list_size ( list );
   *array_ptr = array = hor_malloc_ntype ( void *, n );
   for ( i = 0; i < n; i++, list = list->next )
      array[i] = list->contents;
}

Hor_List hor_array_to_list ( void **array, int n )
{
   int      i;
   Hor_List result = NULL;

   for ( i = n-1; i >= 0; i-- )
      result = hor_insert ( result, array[i] );

   return result;
}

/* The following functions Copyright Jason Merron, August 1994 */

/*******************
*   Hor_List @hor_duplicate_list (Hor_List list)
*
*   Return a separate copy of a singly-linked list.
********************/
Hor_List hor_duplicate_list (Hor_List list)
{
   Hor_List newlist;

   for (newlist = NULL; list != NULL; list = list->next)
      newlist = hor_insert (newlist, (void *) list->contents);

  return hor_reverse (newlist);
}

/*******************
*   Hor_List @hor_insert_after_first (Hor_List list, void *data)
*
*   Same as hor_insert, but inserts new record after the head of the list.
********************/
Hor_List hor_insert_after_first (Hor_List list, void *data)
{
  Hor_List *second_node;

  if ( list == NULL )
      hor_error ( "NULL passed to hor_insert_after_first()", HOR_FATAL );

  second_node = &(list->next);
  *second_node = hor_insert (*second_node, data);
  return list;
}
