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
