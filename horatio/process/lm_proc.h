/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/lm_proc.h */

#ifdef _HORATIO_LIST_

void hor_set_lm_result_type_label  ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_lm_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_LM_Output_Params *hor_lm_make_output_data ( Hor_LM_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_lm_output      ( void *process_result, void *output_data );
void hor_lm_free_result ( void *process_result );
