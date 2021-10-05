/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/process.h */

/*******************
*   typedef void   (*@Hor_Output_Result_Func) (void *process_result,
*                                         void *output_data);
*   typedef void   (*@Hor_Free_Result_Func)   (void *process_result);
*   typedef void   (*@Hor_Free_Data_Func)     (void *data);
*   typedef void * (*@Hor_Execute_Func)       (Hor_List, void *process_result,
*                                                       void *process_data);
*   typedef void   (*@Hor_Update_Data_Func)   (void *process_data);
*
*   Virtual process pipeline control function type definitions.
********************/
typedef void   (*Hor_Output_Result_Func) (void *process_result,
					  void *output_data);
typedef void   (*Hor_Free_Result_Func)   (void *process_result);
typedef void   (*Hor_Free_Data_Func)     (void *data);

#ifdef _HORATIO_LIST_
typedef void * (*Hor_Execute_Func)       (Hor_List, void *process_result,
					            void *process_data);
#endif

typedef void   (*Hor_Update_Data_Func)   (void *process_data);

#ifdef _HORATIO_LIST_

Hor_Assoc_Label hor_add_result_type (
   Hor_Output_Result_Func output_result_func,   /* function called to output a
						   result of the new type,
						   usually either to the
						   display it or write it to a
						   file */
   Hor_Free_Result_Func   free_result_func,     /* function called to free a
						   result of the new type */
   Hor_Free_Data_Func     free_data_func );     /* function called to free the
						   data associated with the
						   outputting of a result of
						   the new type */

Hor_Assoc_Label hor_add_process_type (
   Hor_List             input_types,       /* list of result type labels of
					      input processes, to be checked
					      against result type labels of
					      processes actually input to
					      processes of the new type */
   Hor_Assoc_Label      result_type_label, /* label of the type of result that
					      executing a process of the new
					      type will return */
   Hor_Execute_Func     execute_func,      /* function called to execute a
					      process of the new type */
   Hor_Update_Data_Func update_data_func,  /* function called to update data
					      associated with a process of the
					      new type at each step of the
					      sequence */
   Hor_Free_Data_Func   free_data_func );  /* function called to free the data
					      associated with a process of the
					      new type */

Hor_Assoc_Label hor_add_process (
   Hor_Assoc_Label process_type_label, /* label of the process's type */
   Hor_List        input_processes,    /* list of input process labels */
   void       *process_data,       /* pointer to structure containing
				      parameters used by the process */
   void       *output_data );   /* pointer to structure containing
				   parameters used by the process for
				   output purposes */

#endif /* _HORATIO_LIST_ */

void hor_execute_processes(void);
void hor_clear_processes(void);

#ifdef _HORATIO_LIST_

void *hor_get_process_data ( Hor_Assoc_Label proc_label );

typedef struct
{
   Hor_Assoc_Label process_label, result_type_label;
   void           *process_result;
} Hor_Process_Input;

#endif /* _HORATIO_LIST_ */
