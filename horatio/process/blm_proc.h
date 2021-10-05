/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/blm_proc.h */

/*******************
*   typedef struct
*   {
*      int   dist_thresh;   (distance threshold between two matched lines)
*      float cos_thresh;    (orientation cosine threshold between two lines)
*      float size_thresh;   (threshold on line length ratio)
*      int   iterations;    (number of winner-take-all matching iterations)
*   } @Hor_BLM_Process_Params;
*
*   Bog-standard line matching process parameter structure definitions.
********************/
typedef struct
{
   int   dist_thresh;   /* distance threshold between two matched lines */
   float cos_thresh;    /* orientation cosine threshold between two lines */
   float size_thresh;   /* threshold on line length ratio */
   int   iterations;    /* number of winner-take-all matching iterations */
} Hor_BLM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_blm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_blm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_BLM_Process_Params *hor_blm_make_process_data ( Hor_BLM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_blm_add_process ( Hor_BLM_Process_Params, Hor_LM_Output_Params,
			   Hor_Assoc_Label );

void *hor_blm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
