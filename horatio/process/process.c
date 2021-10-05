/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"

typedef struct
{
   Hor_Output_Result_Func output_result_func;
   Hor_Free_Result_Func   free_result_func;
   Hor_Free_Data_Func     free_data_func;
} Result_Type_Def;

typedef struct
{
   Hor_List             input_types; /* list of result type labels */
   Hor_Assoc_Label      result_type_label;
   Hor_Execute_Func     execute_func;
   Hor_Update_Data_Func update_data_func;
   Hor_Free_Data_Func   free_data_func;
} Process_Type_Def;
 
typedef struct
{
   Hor_Assoc_Label proc_type_label;
   void           *proc_result;
   Hor_List        input_procs; /* list of input process labels */
   void           *proc_data;
   void           *output_data;
} Process_Def;

/* association list of registered process result types */
static Hor_Assoc_List proc_type_list = NULL;

/* association list of registered process types */
static Hor_Assoc_List result_type_list  = NULL;

/* processes that actually get called by hor_execute_processes() */
static Hor_Assoc_List active_proc_list = NULL;

/* processes waiting to be put on active list. when processes are called,
   they are put on the passive list hor_execute_processes() is called with the
   active list as null, in which case all processes are made active and
   called. */
static Hor_Assoc_List passive_proc_list = NULL;

/* get_proc_type_def(): returns the process type definition structure
                        for a given process type label */
static Process_Type_Def *get_proc_type_def (Hor_Assoc_Label proc_type_labelbel)
{
   void *result = hor_assoc_find ( proc_type_list, proc_type_labelbel );

   return ( (Process_Type_Def *) result );
}

/* get_result_type_def(): returns the result type definition structure
                          for a given result type label */
static Result_Type_Def *get_result_type_def (Hor_Assoc_Label result_type_label)
{
   void *result = hor_assoc_find ( result_type_list, result_type_label );

   return ( (Result_Type_Def *) result );
}

/* get_proc(): returns the process definition structure
               for a given process label */
static Process_Def *get_proc ( Hor_Assoc_Label proc_label )
{
   void *result;

   result = hor_assoc_find ( passive_proc_list, proc_label );
   if ( result == NULL )
      result = hor_assoc_find ( active_proc_list, proc_label );

   return ( (Process_Def *) result );
}

/* check_result_type_label_list(): checks a list of result type labels to see
                                   if they have all been registered */
static Hor_Bool check_result_type_label_list (Hor_List result_type_label_list)
{
   Hor_Assoc_Label result_type_label;

   for ( ; result_type_label_list != NULL;
	 result_type_label_list = result_type_label_list->next )
   {
      result_type_label = hor_assoc_label_contents(result_type_label_list);
      if ( get_result_type_def(result_type_label) == NULL )
      {
	 hor_errno = HOR_PROCESS_UNDEFINED_RESULT_TYPE;
	 return HOR_FALSE;
      }
   }

   return HOR_TRUE;
}

/* check_input_proc_types(): checks a list of processes against a list
                             of result type labels to check that the result
			     types of the processes match the type labels */
static Hor_Bool check_input_proc_types ( Hor_List input_procs,
				         Hor_List input_types )
{
   Hor_Assoc_Label   proc_label;
   Hor_Assoc_Label   result_type_label;
   Process_Def      *proc;
   Process_Type_Def *proc_type_def;

   for ( ; input_procs != NULL && input_types != NULL;
	 input_procs = hor_next_node(input_procs),
	 input_types = hor_next_node(input_types) )
   {
      proc_label        = hor_assoc_label_contents(input_procs);
      result_type_label = hor_assoc_label_contents(input_types);
      if ( (proc = get_proc ( proc_label )) == NULL )
      {
	 hor_errno = HOR_PROCESS_UNDEFINED_PROCESS;
	 return HOR_FALSE;
      }

      if ( (proc_type_def = get_proc_type_def(proc->proc_type_label))
	   == NULL )
      {
	 hor_errno = HOR_PROCESS_UNDEFINED_PROCESS_TYPE;
	 return HOR_FALSE;
      }

      if ( proc_type_def->result_type_label != result_type_label )
      {
	 hor_errno = HOR_PROCESS_INCOMPATIBLE_RESULT_TYPES;
	 return HOR_FALSE;
      }
   }

   if ( input_procs != NULL || input_types != NULL )
   {
      hor_errno = HOR_PROCESS_DIFFERENT_LENGTH_RESULT_LISTS;
      return HOR_FALSE;
   }

   return HOR_TRUE;
}

/* free_proc(): frees all data associated with a process */
static void free_proc ( void *data ) /* pointer to a process definition */
{
   Process_Def      *proc = (Process_Def *) data;
   Process_Type_Def *proc_type_def;
   Result_Type_Def  *res_type_def;

   /* free list of input processes */
   hor_free_assoc_label_list ( proc->input_procs );

   /* generate definition of process type */
   proc_type_def = get_proc_type_def ( proc->proc_type_label );

   /* generate definition of result type */
   res_type_def  = get_result_type_def ( proc_type_def->result_type_label );

   /* free process data */
   if ( res_type_def->free_data_func != NULL )
      res_type_def->free_data_func ( proc->output_data );

   if ( proc_type_def->free_data_func != NULL )
      proc_type_def->free_data_func ( proc->proc_data );

   /* free result */
   if ( res_type_def->free_result_func != NULL )
      res_type_def->free_result_func ( proc->proc_result );

   hor_free ( (void *) proc );
}

/* counters for registration labels */
static Hor_Assoc_Label next_result_type_label  = HOR_ASSOC_START;
static Hor_Assoc_Label next_proc_type_label = HOR_ASSOC_START;
static Hor_Assoc_Label next_proc_label      = HOR_ASSOC_START;

/*******************
*   Hor_Assoc_Label @hor_add_result_type (
*      Hor_Output_Result_Func output_result_func, (function called to output a
*                                                  result of the new type,
*                                                  usually either to the
*                                                  display it or write it to a
*                                                  file)
*      Hor_Free_Result_Func   free_result_func, (function called to free a
*                                                result of the new type)
*      Hor_Free_Data_Func     free_data_func ) (function called free the data
*                                               associated with the outputting
*                                               of a result of the new type)
*
*   Registers a new process result type and returns a label for it.
********************/
Hor_Assoc_Label hor_add_result_type (Hor_Output_Result_Func output_result_func,
				     Hor_Free_Result_Func   free_result_func,
				     Hor_Free_Data_Func     free_data_func )
{
   Result_Type_Def *result_type;
   Hor_Assoc_List   type_list;

   if ( free_result_func == NULL )
   {
      hor_errno = HOR_PROCESS_NULL_POINTER_ARGUMENT;
      return HOR_ASSOC_ERROR;
   }

   /* check for duplicated type */
   for ( type_list = result_type_list; type_list != NULL;
	 type_list = type_list->next )
   {
      result_type = (Result_Type_Def *) type_list->data;
      if ( result_type->free_result_func == free_result_func )
      {
	 hor_errno = HOR_PROCESS_DUPLICATED_RESULT_TYPES;
	 return HOR_ASSOC_ERROR;
      }
   }

   result_type = hor_malloc_type ( Result_Type_Def );
   result_type->output_result_func = output_result_func;
   result_type->free_result_func   = free_result_func;
   result_type->free_data_func     = free_data_func;
   result_type_list = hor_assoc_insert ( result_type_list,
					 next_result_type_label, result_type );
   return ( next_result_type_label++ );
}

/*******************
*   Hor_Assoc_Label @hor_add_process_type (
*      Hor_List             input_types, (list of result type labels of input
*                                         processes, to be checked against
*                                         result type labels of processes
*                                         actually input to processes of the
*                                         new type)
*      Hor_Assoc_Label      result_type_label, (label of the type of result
*                                               that executing a process of the
*                                               new type will return)
*      Hor_Execute_Func     execute_func, (function called to execute a
*                                          process of the new type)
*      Hor_Update_Data_Func update_data_func, (function called to update data
*                                              associated with a process of the
*                                              new type at each step of the
*                                              sequence)
*      Hor_Free_Data_Func   free_data_func ) (function called to free data
*                                             associated with a process of the
*                                             new type)
*
*   Registers a new process type and return a label for it.
********************/
Hor_Assoc_Label hor_add_process_type ( Hor_List             input_types,
				       Hor_Assoc_Label      result_type_label,
				       Hor_Execute_Func     execute_func,
				       Hor_Update_Data_Func update_data_func,
				       Hor_Free_Data_Func   free_data_func )
{
   Process_Type_Def *proc_type;
   Hor_Assoc_List    type_list;

   /* check for duplicated type */
   for ( type_list = proc_type_list; type_list != NULL;
         type_list = type_list->next )
   {
      proc_type = (Process_Type_Def *) type_list->data;
      if ( proc_type->execute_func == execute_func )
      {
	 hor_errno = HOR_PROCESS_DUPLICATED_PROCESS_TYPES;
	 return HOR_ASSOC_ERROR;
      }
   }

   if ( !check_result_type_label_list ( input_types ) ) return HOR_ASSOC_ERROR;

   if ( get_result_type_def ( result_type_label ) == NULL )
   {
      hor_errno = HOR_PROCESS_UNDEFINED_RESULT_TYPE;
      return HOR_ASSOC_ERROR;
   }

   proc_type = hor_malloc_type ( Process_Type_Def );
   proc_type->input_types           = hor_copy_assoc_label_list (input_types);
   proc_type->result_type_label     = result_type_label;
   proc_type->execute_func          = execute_func;
   proc_type->update_data_func      = update_data_func;
   proc_type->free_data_func        = free_data_func;
   proc_type_list = hor_assoc_insert ( proc_type_list, next_proc_type_label,
				       proc_type );
   return ( next_proc_type_label++ );
}

/*******************
*   Hor_Assoc_Label @hor_add_process (
*      Hor_Assoc_Label proc_type_label, (label of the process's type)
*      Hor_List        input_procs, (list of input process labels)
*      void           *proc_data, (pointer to structure containing
*                                  parameters used by the process)
*      void           *output_data ) (pointer to structure containing
*                                     parameters used by the process for
*                                     output purposes)
*
*   Adds a new process to the process list, returning a label for it.
********************/
Hor_Assoc_Label hor_add_process ( Hor_Assoc_Label proc_type_label,
				  Hor_List input_procs,
				  void *proc_data, void *output_data )
{
   Process_Def      *new_proc;
   Process_Type_Def *proc_type_def;
   Result_Type_Def  *res_type_def;

   /* check that process type label is actually registered */
   if ( (proc_type_def = get_proc_type_def ( proc_type_label )) == NULL )
   {
      hor_errno = HOR_PROCESS_UNDEFINED_PROCESS_TYPE;
      return HOR_ASSOC_ERROR;
   }

   if ( proc_type_def->free_data_func == NULL && proc_data != NULL)
   {
      hor_errno = HOR_PROCESS_ILLEGAL_PROCESS_DATA;
      return HOR_ASSOC_ERROR;
   }

   /* check that result types of input process list and result types registered
      for process types match */
   if ( !check_input_proc_types ( input_procs, proc_type_def->input_types ) )
      return HOR_ASSOC_ERROR;

   /* generate definition of process result type */
   res_type_def  = get_result_type_def ( proc_type_def->result_type_label );

   if ( res_type_def->free_data_func == NULL && output_data != NULL )
   {
      hor_errno = HOR_PROCESS_ILLEGAL_OUTPUT_DATA;
      return HOR_ASSOC_ERROR;
   }

   /* create new process */
   new_proc = hor_malloc_type ( Process_Def );
   new_proc->input_procs     = input_procs;
   new_proc->proc_type_label = proc_type_label;
   new_proc->proc_result     = NULL;
   new_proc->proc_data       = proc_data;
   new_proc->output_data     = output_data;

   /* add new process to end of process list */
   active_proc_list = hor_assoc_append ( active_proc_list, next_proc_label,
					 new_proc );
   return ( next_proc_label++ );
}

/* make_result(): returns pointer to structure containing latest result of
                  given process along with its result type label */
static Hor_Process_Input *make_result ( Hor_Assoc_Label proc_label )
{
   Process_Def       *proc;
   Process_Type_Def  *proc_type_def;
   Hor_Process_Input *result;

   proc = get_proc ( proc_label );
   proc_type_def = get_proc_type_def ( proc->proc_type_label );

   result = hor_malloc_type ( Hor_Process_Input );
   result->process_label     = proc_label;
   result->result_type_label = proc_type_def->result_type_label;
   result->process_result    = proc->proc_result;
   return result;
}

/* make_input_list(): takes list of process labels and forms corresponding
                      list of process results */
static Hor_List make_input_list ( Hor_List input_procs )
{
   if ( input_procs == NULL ) return NULL;
   else
      return (hor_insert(make_input_list ( input_procs->next ),
			 make_result (hor_assoc_label_contents(input_procs))));
}

/*******************
*   void @hor_execute_processes(void)
*
*   Executes any processes added to process list since last call.
*   If there aren't any execute the whole process list.
********************/
void hor_execute_processes(void)
{
   Hor_Assoc_List    node;
   Process_Def      *proc;
   Process_Type_Def *proc_type_def;
   Result_Type_Def  *res_type_def;
   void             *new_result;

   if ( hor_assoc_null(active_proc_list) ) /* no new processes to execute, so
					      execute all processes */
   {
      active_proc_list = hor_assoc_concat(passive_proc_list, active_proc_list);
      passive_proc_list = NULL;
   }

   for ( node = active_proc_list; node != NULL; node = node->next )
   {
      Hor_List input_list;

      /* generate definition of process */
      proc = (Process_Def *) hor_assoc_data(node);

      /* generate definition of process type */
      proc_type_def = get_proc_type_def ( proc->proc_type_label );

      /* generate definition of result type */
      res_type_def  = get_result_type_def ( proc_type_def->result_type_label );

      /* generate list of input process results */
      input_list = make_input_list ( proc->input_procs );

      /* execute process */
      if ( proc_type_def->free_data_func != NULL )
	 new_result = proc_type_def->execute_func ( input_list,
						    proc->proc_result,
						    proc->proc_data );
      else /* no process parameters */
	 new_result = proc_type_def->execute_func ( input_list,
						    proc->proc_result,
						    NULL );

      hor_free_list ( input_list, hor_free_func );

      /* output result */
      if ( res_type_def->output_result_func != NULL )
	 if ( res_type_def->free_data_func != NULL )
	    res_type_def->output_result_func ( new_result,
					       proc->output_data );
         else /* no output parameters */
	    res_type_def->output_result_func ( new_result, NULL );

      /* free old result */
      if ( res_type_def->free_result_func != NULL )
	 res_type_def->free_result_func ( proc->proc_result );

      /* update process result */
      proc->proc_result = new_result;

      /* update process data */
      if ( proc_type_def->update_data_func != NULL )
	 proc_type_def->update_data_func ( proc->proc_data );
   }

   /* all active processes have been executed, so put them all
      on passive list */
   passive_proc_list = hor_assoc_concat ( passive_proc_list, active_proc_list);
   active_proc_list = NULL;
}

/*******************
*   void @hor_clear_processes(void)
*
*   Removes all processes.
********************/
void hor_clear_processes(void)
{
   hor_assoc_free ( active_proc_list, free_proc );
   hor_assoc_free ( passive_proc_list, free_proc );
   active_proc_list  = NULL;
   passive_proc_list = NULL;
   next_proc_label = HOR_ASSOC_START;
}

/*******************
*   void *@hor_get_process_data ( Hor_Assoc_Label proc_label )
*
*   Returns the parameter data pointer associated with the specified process.
********************/
void *hor_get_process_data ( Hor_Assoc_Label proc_label )
{
   Process_Def *proc;

   if ( (proc = get_proc ( proc_label )) == NULL )
   {
      hor_errno = HOR_PROCESS_UNDEFINED_PROCESS;
      return NULL;
   }

   return proc->proc_data;
}
