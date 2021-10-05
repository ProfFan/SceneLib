/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/cm_proc.h */

#ifdef _HORATIO_LIST_

void hor_set_cm_result_type_label  ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_cm_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_CM_Output_Params *hor_cm_make_output_data ( Hor_CM_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_cm_output      ( void *process_result, void *output_data );
void hor_cm_free_result ( void *process_result );
