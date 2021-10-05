/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* association_list.c: module for manipulating association lists, by which
                       labels are associated with specific data items */
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/list.h"

/*******************
*   Hor_Assoc_List  @hor_assoc_insert     ( Hor_Assoc_List list,
*                                          Hor_Assoc_Label label, void *data)
*   Hor_Assoc_List  @hor_assoc_append     ( Hor_Assoc_List list,
*                                          Hor_Assoc_Label label, void *data)
*   Hor_Assoc_List  @hor_assoc_concat     ( Hor_Assoc_List list1,
*                                          Hor_Assoc_List list2 )
*   int             @hor_assoc_list_size  ( Hor_Assoc_List );
*   void           *@hor_assoc_find       ( Hor_Assoc_List list,
*                                          Hor_Assoc_Label label )
*   Hor_Assoc_Label @hor_assoc_remove     ( Hor_Assoc_List *list_ptr,
*                                          Hor_Assoc_Label label,
*                                          void          (*free_func)(void *) )
*   void            @hor_assoc_free       ( Hor_Assoc_List list,
*                                          void         (*free_func)(void *))
*   Hor_Assoc_List  @hor_assoc_delete_first ( Hor_Assoc_List *nodepp,
*                                            void (*free_func)(void *) )
*   void            @hor_assoc_delete_next ( Hor_Assoc_List list,
*                                           void (*free_func)(void *) )
*   void            @hor_assoc_free_nodes ( Hor_Assoc_List list )
*
*   Hor_List @hor_make_assoc_label_list ( Hor_Assoc_Label label1, ... )
*   Hor_List @hor_copy_assoc_label_list ( Hor_List label_list )
*   void     @hor_free_assoc_label_list ( Hor_List label_list )
*
*   Routines for manipulating association lists, by which labels are
*   associated with specific data items.
*
*   hor_assoc_insert() adds a new node to the start of an association list,
*                      with specified label and data item (data) and returns
*                      the extended list.
*   hor_assoc_append() adds the new node to the end of the list.
*   hor_assoc_concat() hor_concatenates two association lists and returns the
*                      extended list list1.
*   hor_assoc_list_size() returns the number of items in the given list.
*   hor_assoc_find() returns the data item associated with the given label.
*   hor_assoc_remove() removes node from association list with specific label
*                      and frees associated data using function free_func(),
*                      modifying the list if the required node was the first.
*                      The value returned is the label if the node is removed
*                      successfully, HOR_ASSOC_ERROR otherwise.
*   hor_assoc_free() frees an association list and the associated data using
*                    the function free_func().
*   hor_assoc_free_nodes() frees just the nodes of an association list.
*   hor_assoc_delete_first() deletes the first element of a list and returns
*                            the remainder.
*   hor_assoc_delete_next() deletes the second element of the given list and
*                           joins the first element to the third.
*   hor_make_assoc_label_list() makes a list of association labels from
*                               variable argument list of labels terminated by
*                               HOR_ASSOC_END.
*   hor_copy_assoc_label_list() creates and returns a copy of an label list.
*   hor_free_assoc_label_list() frees the memory associated with a list of
*                               association labels.
********************/

/* assoc_append():

      Appends two association lists. */
static void assoc_append ( Hor_Assoc_List *list_ptr, Hor_Assoc_List list )
{
   if ( *list_ptr == NULL )
      *list_ptr = list;
   else
      assoc_append ( &(*list_ptr)->next, list );
}

Hor_Assoc_List hor_assoc_insert ( Hor_Assoc_List list, Hor_Assoc_Label label,
				  void *data )
{
   Hor_Assoc_List new_node = hor_malloc_type(Hor_Assoc_Node);

   new_node->label = label;
   new_node->data  = data;
   new_node->next  = list;
   return new_node;
}

Hor_Assoc_List hor_assoc_append ( Hor_Assoc_List list, Hor_Assoc_Label label,
				  void *data )
{
   Hor_Assoc_List new_node = hor_malloc_type(Hor_Assoc_Node);

   new_node->label = label;
   new_node->data  = data;
   new_node->next  = NULL;
   assoc_append ( &list, new_node );
   return list;
}

Hor_Assoc_List hor_assoc_concat ( Hor_Assoc_List list1, Hor_Assoc_List list2 )
{
   assoc_append ( &list1, list2 );
   return list1;
}

int hor_assoc_list_size  ( Hor_Assoc_List list )
{
   if ( list == NULL ) return 0;
   else return ( 1 + hor_assoc_list_size ( list->next ) );
}

void *hor_assoc_find ( Hor_Assoc_List list, Hor_Assoc_Label label )
{
   if ( list == NULL ) return NULL;

   if ( list->label == label )
      return list->data;
   else
      return ( hor_assoc_find ( list->next, label ) );
}

static Hor_Assoc_Label hor_assoc_remove_from_list ( Hor_Assoc_List  previous,
						    Hor_Assoc_List  current,
						    Hor_Assoc_Label label,
						    void (*free_func)(void *) )
{
   if ( current == NULL ) return HOR_ASSOC_ERROR;

   if ( current->label == label )
   {
      if ( free_func != NULL )
	 free_func ( current->data );

      previous->next = current->next;
      hor_free ( (void *) current );
      return label;
   }
   else
      return ( hor_assoc_remove_from_list ( current, current->next,
					    label, free_func ) );
}

Hor_Assoc_Label hor_assoc_remove ( Hor_Assoc_List *list_ptr,
				   Hor_Assoc_Label label,
				   void          (*free_func)(void *) )
{
   if ( *list_ptr == NULL ) return HOR_ASSOC_ERROR;

   if ( (*list_ptr)->label == label )
   {
      Hor_Assoc_List list;

      free_func ( (*list_ptr)->data );
      list = (*list_ptr)->next;
      hor_free ( (void *) *list_ptr );
      *list_ptr = list;
      return label;
   }
   else
      return ( hor_assoc_remove_from_list ( *list_ptr, (*list_ptr)->next,
					    label, free_func ) );
}

void hor_assoc_free ( Hor_Assoc_List list, void (*free_func)(void *) )
{
   if ( list == NULL ) return;

   if ( free_func != NULL ) free_func ( list->data );

   hor_assoc_free ( list->next, free_func );
   hor_free ( (void *) list );
}

Hor_Assoc_List hor_assoc_delete_first ( Hor_Assoc_List *nodepp,
				        void (*free_func)(void *) )
{
   Hor_Assoc_List temp;

   temp = *nodepp;
   if ( temp == NULL ) return NULL;

   if ( free_func != NULL ) free_func ( temp->data );

   *nodepp = temp->next;
   hor_free ( (void *) temp );
   return ( *nodepp );
}

void hor_assoc_delete_next ( Hor_Assoc_List list, void (*free_func)(void *) )
{
   Hor_Assoc_List temp;

   if ( list == NULL )
      hor_error ( "NULL passed to hor_assoc_delete_next()", HOR_FATAL );

   if ( list->next == NULL ) return;

   if ( free_func != NULL ) free_func ( list->next->data );
   temp = list->next;
   list->next = list->next->next;
   hor_free ( (void *) temp );
}

Hor_List hor_make_assoc_label_list ( Hor_Assoc_Label label1, ... )
{
   va_list         ap;
   Hor_Assoc_Label label, *label_array;
   Hor_List        label_list;
   int             label_count = 1;

   if ( label1 == HOR_ASSOC_END ) return NULL;

   va_start ( ap, label1 );
   while(1)
   {
      label = va_arg ( ap, Hor_Assoc_Label );
      if ( label == HOR_ASSOC_END ) break;

      label_count++;
   }

   va_end(ap);

   label_array = hor_malloc_ntype ( Hor_Assoc_Label, label_count );
   label_array[0] = label1;
   label_list = hor_insert ( NULL, label_array );
   va_start ( ap, label1 );
   label_count = 1;
   while(1)
   {
      label = va_arg ( ap, Hor_Assoc_Label );
      if ( label == HOR_ASSOC_END ) break;

      label_array[label_count] = label;
      label_list = hor_insert ( label_list, label_array + label_count );
      label_count++;
   }

   va_end(ap);

   return ( hor_reverse ( label_list ) );
}

Hor_List hor_copy_assoc_label_list ( Hor_List label_list )
{
   int              label_count = 0;
   Hor_List         list, new_list = NULL;
   Hor_Assoc_Label *label_array;

   for ( list = label_list; list != NULL; list = list->next )
      label_count++;

   if ( label_count == 0 ) return NULL;
   label_array = hor_malloc_ntype ( Hor_Assoc_Label, label_count );
   label_count = 0;
   for ( list = label_list; list != NULL; list = list->next )
   {
      label_array[label_count] = hor_assoc_label_contents(list);
      new_list = hor_insert ( new_list, label_array + label_count );
      label_count++;
   }

   return ( hor_reverse ( new_list ) );
}

void hor_free_assoc_label_list ( Hor_List label_list )
{
   if ( label_list == NULL ) return;

   hor_free ( (void *) label_list->contents );
   hor_free_nodes ( label_list );
}
