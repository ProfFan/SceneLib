/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/ls_proc.h */

#ifdef _HORATIO_LIST_

void            hor_set_ls_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_ls_process_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_ls_add_process ( Hor_LI_Output_Params, Hor_Assoc_Label );

#endif /* _HORATIO_IMPROC_ */
void *hor_ls_execute ( Hor_List input_list, void *old_result,
		                            void *process_data );
#endif /* _HORATIO_LIST_ */
