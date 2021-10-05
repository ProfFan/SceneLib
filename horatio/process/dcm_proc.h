/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/dcm_proc.h */

/*******************
*   typedef struct
*   {
*      int   match_window; (maximum pixel distance between predicted
*                           corner position and actual position)
*      float corr_thres;   (threshold on image patch correlation)
*   } @Hor_DCM_Process_Params;
*
*   Brain-dead corner matching process parameter structure definitions.
********************/
typedef struct
{
   int   match_window; /* maximum pixel distance between predicted
			  corner position and actual position */
   float corr_thres;   /* threshold on image patch correlation */
} Hor_DCM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_dcm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_dcm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_DCM_Process_Params *hor_dcm_make_process_data ( Hor_DCM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_dcm_add_process ( Hor_DCM_Process_Params, Hor_CM_Output_Params,
			   Hor_Assoc_Label );

void *hor_dcm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
