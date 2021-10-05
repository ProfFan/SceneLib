/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from list/double.h */

/* Doubly-linked list types. */
typedef struct _Hor_DNode
{
   void              *contents;
   struct _Hor_DNode *prev;
   struct _Hor_DNode *next;
} Hor_DNode, *Hor_DList;

/*******************
*   void     *@hor_dnode_contents ( Hor_DList list )
*   Hor_DList @hor_prev_dnode     ( Hor_DList list )
*   Hor_DList @hor_next_dnode ( Hor_DList list )
*
*   Function to access the nodes of a list.
*
*   hor_dnode_contents() returns the contents of the first node in a list.
*   hor_prev_dnode() returns the list starting at the node before the given
*                    one.
*   hor_next_dnode() returns the remainder of the list after the first node.
*
*   All are implemented as macros.
********************/
#define hor_dnode_contents hor_node_contents
#define hor_prev_dnode(list) ((list)->prev)
#define hor_next_dnode hor_next_node

Hor_DList hor_dmake_straight ( void *data );
Hor_DList hor_dmake_circular ( void *data );
Hor_DList hor_dinsert_before ( Hor_DList list, void *data );
Hor_DList hor_dinsert_after  ( Hor_DList list, void *data );
void hor_dfree_list          ( Hor_DList list, void (*free_func)(void *) );
void hor_dfree_list_before   ( Hor_DList list, void (*free_func)(void *) );
void hor_dfree_list_after    ( Hor_DList list, void (*free_func)(void *) );
#define hor_dfree_nodes(list) hor_dfree_list(list,NULL)

void  hor_dlist_action ( Hor_DList list, void (*act_func)(void *, void *),
			 void *data );

Hor_DList hor_dmake_straight_from_list ( Hor_List list );
Hor_DList hor_dmake_circular_from_list ( Hor_List list );
