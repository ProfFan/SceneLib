/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from list/single.h */

/* Definitions of singly-linked list types. */
typedef struct _Hor_Node
{
   void             *contents;
   struct _Hor_Node *next;
} Hor_Node, *Hor_List;

/*******************
*   void     *@hor_node_contents ( Hor_List list )
*   Hor_List  @hor_next_node     ( Hor_List list )
*
*   Function to access the nodes of a list.
*
*   hor_node_contents() returns the contents of the first node in a list.
*   hor_next_node() returns the remainder of the list after the first node.
*
*   Both are implemented as macros.
********************/
#define hor_node_contents(list)  ((list)->contents)
#define hor_next_node(list)      ((list)->next)

/*******************
*   Hor_Bool @hor_list_null     ( Hor_List list )
*   Hor_Bool @hor_list_non_null ( Hor_List list )
*
*   Functions to test whether a list is empty or not.
*
*   hor_list_null() returns HOR_TRUE if list is equal to NULL, HOR_FALSE
*   otherwise. hor_list_non_null() does the reverse.
*
*   Both are implemented as macros.
********************/
#define hor_list_null(list)      ((list) == NULL)
#define hor_list_non_null(list)  ((list) != NULL)

Hor_List hor_insert    ( Hor_List list, void *data );
Hor_List hor_append    ( Hor_List list, void *data );
Hor_List hor_concat    ( Hor_List list1, Hor_List list2 );
void     hor_transfer  ( Hor_List *from, Hor_List *to );
void     hor_free_list ( Hor_List list, void (*free_func)(void *) );
#define hor_free_nodes(list) hor_free_list(list,NULL)

Hor_List hor_reverse         ( Hor_List );
Hor_List hor_reverse_no_free ( Hor_List );
void     hor_list_action     ( Hor_List list, void (*act_func)(void *, void *),
			       void *data );
Hor_List hor_delete_first    ( Hor_List *nodepp, void (*free_func)(void *) );
void     hor_delete_next     ( Hor_List list, void (*free_func)(void *) );
int      hor_list_size       ( Hor_List );
void     hor_list_to_array   ( Hor_List list, void ***array_ptr, int *n_ptr );
Hor_List hor_array_to_list   ( void **array, int n );
Hor_List hor_duplicate_list (Hor_List list);
Hor_List hor_insert_after_first (Hor_List list, void *data);
