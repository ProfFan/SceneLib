/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/gb_proc.h */

/*******************
*   typedef struct
*   {
*      char root_name[200];
*      int  image_count;
*   } @Hor_GB_Process_Data;
*
*   Grab process parameter structure definition.
********************/
typedef struct
{
   char root_name[200];
   int  image_count;
} Hor_GB_Process_Data;

#ifdef _HORATIO_LIST_

void hor_set_gb_result_type_label  ( Hor_Assoc_Label );
void hor_set_gb_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_gb_result_type_label(void);
Hor_Assoc_Label hor_get_gb_process_type_label(void);
Hor_Assoc_Label hor_get_gb_process_label(void);

#endif /* _HORATIO_LIST_ */

Hor_GB_Process_Data *hor_gb_make_process_data ( const char *root_name );
void                 hor_gb_add_process       ( const char *root_name );

#ifdef _HORATIO_LIST_
void *hor_gb_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif

void  hor_gb_output              ( void *process_result, void *output_data );
void  hor_gb_update_process_data ( void *process_data   );
void  hor_gb_free_process_data   ( void *process_data   );
void  hor_gb_free_result         ( void *process_result );
