/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/li_proc.h */

#ifdef _HORATIO_LIST_

void            hor_set_li_result_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_li_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_LI_Output_Params *hor_li_make_output_data ( Hor_LI_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void  hor_li_output      ( void *process_result, void *output_data );
void  hor_li_free_result ( void *process_result );
