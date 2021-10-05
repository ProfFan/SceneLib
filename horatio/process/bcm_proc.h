/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/bcm_proc.h */

/*******************
*   typedef struct
*   {
*      int   range;              (maximum corner displacement between frames)
*      float imdiff_ratio_thres; (threshold on image differences)
*      int   iterations;         (number of passes through corner maps)
*   } @Hor_BCM_Process_Params;
*
*   Bog-standard corner matching process parameter structure definitions.
********************/
typedef struct
{
   int   range;              /* maximum corner displacement between frames */
   float imdiff_ratio_thres; /* threshold on image differences */
   int   iterations;         /* (number of passes through corner maps */
} Hor_BCM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_bcm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_bcm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_BCM_Process_Params *hor_bcm_make_process_data ( Hor_BCM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_bcm_add_process ( Hor_BCM_Process_Params, Hor_CM_Output_Params,
			   Hor_Assoc_Label );

void *hor_bcm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
