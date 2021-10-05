/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_LIST_
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from list/assoc.h */

/* Label type for associating labels with data objects. */
typedef int Hor_Assoc_Label;

/*******************
*   @HOR_ASSOC_END    (marks end of association label list)
*   @HOR_ASSOC_ERROR, (denotes illegal association label)
*   @HOR_ASSOC_START  (start value for production of new association labels)
*
*   Special label values.
********************/
#define HOR_ASSOC_END   -2
#define HOR_ASSOC_ERROR -1
#define HOR_ASSOC_START  0

/*******************
*   Hor_Assoc_Label @hor_assoc_label_contents ( Hor_List list )
*
*   Returns the first association label in a singly-linked list of
*   association labels.
********************/
#define hor_assoc_label_contents(list) (*((Hor_Assoc_Label *) hor_node_contents(list)))

/* Association list type definitions. */
typedef struct hor_assoc_node
{
   void                  *data;  /* data object */
   Hor_Assoc_Label        label; /* associated label */
   struct hor_assoc_node *next;
} Hor_Assoc_Node, *Hor_Assoc_List;


/*******************
*   Hor_Assoc_Label @hor_assoc_label ( Hor_Assoc_List list )
*   void           *@hor_assoc_data  ( Hor_Assoc_List list )
*   Hor_Assoc_List  @hor_assoc_next  ( Hor_Assoc_List list )
*
*   Functions to access the nodes of an association list.
*
*   hor_assoc_label() returns the label of the given node of the association
*                     list.
*   hor_assoc_data()  returns the data associated with the given node.
*   hor_assoc_next()  returns the remainder of the association list after the
*                     first node.
*
*   All are implemented as macros.
********************/
#define hor_assoc_label(list)     ((list)->label)
#define hor_assoc_data(list)      ((list)->data)
#define hor_assoc_next(list)      ((list)->next)

/*******************
*   Hor_Bool @hor_assoc_null      ( Hor_Assoc_List list )
*   Hor_Bool @hor_assoc_non_null  ( Hor_Assoc_List list )
*   Hor_Bool @hor_assoc_null_data ( void          *data )
*
*   Functions to test whether an association list or associated data is NULL
*   or not.
*
*   hor_assoc_null()      returns HOR_TRUE if association list is equal to NULL.
*   hor_assoc_non_null()  does the reverse.
*   hor_assoc_null_data() tests the association data for equality to NULL.
*
*   All are implemented as macros.
********************/
#define hor_assoc_null(list)      ((list) == NULL)
#define hor_assoc_non_null(list)  ((list) != NULL)
#define hor_assoc_null_data(data) (data == NULL)

Hor_Assoc_List  hor_assoc_insert ( Hor_Assoc_List, Hor_Assoc_Label, void * );
Hor_Assoc_List  hor_assoc_append ( Hor_Assoc_List, Hor_Assoc_Label, void * );
Hor_Assoc_List  hor_assoc_concat ( Hor_Assoc_List, Hor_Assoc_List );
int             hor_assoc_list_size ( Hor_Assoc_List );
void           *hor_assoc_find      ( Hor_Assoc_List, Hor_Assoc_Label );
Hor_Assoc_Label hor_assoc_remove    ( Hor_Assoc_List *list_ptr,
				      Hor_Assoc_Label label,
				      void          (*free_func)(void *) );
void            hor_assoc_free      ( Hor_Assoc_List list,
				      void (*free_func)(void *) );
Hor_Assoc_List hor_assoc_delete_first ( Hor_Assoc_List *nodepp,
				        void (*free_func)(void *) );
void hor_assoc_delete_next ( Hor_Assoc_List list, void (*free_func)(void *) );
#define         hor_assoc_free_nodes(list) hor_assoc_free(list,NULL)

Hor_List hor_make_assoc_label_list ( Hor_Assoc_Label, ... );
Hor_List hor_copy_assoc_label_list ( Hor_List );
void     hor_free_assoc_label_list ( Hor_List );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* include from list/list_io.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h
Hor_List HorChanInList ( Channel *in, int no_nodes,
			 void * (*input_func)(Channel *),
			 void (*free_func)(void *) );
void HorChanOutList ( Channel *out, Hor_List list,
		      void (*output_func)(Channel *,void *) );
#endif
#endif

#ifndef HOR_REDUCED_LIB
Hor_List hor_read_list ( int fd, int no_nodes, void * (*input_func)(int),
			 void (*free_func)(void *) );
void hor_write_list ( int fd, Hor_List list,
		      void (*output_func)(int,void *) );
#endif /* HOR_REDUCED_LIB */
