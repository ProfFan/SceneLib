/* Copyright 1994 David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/lf_proc.h */

typedef struct
{
   int   no_points;      /* number of points used to fit a line */
   float sigma;          /* standard deviation of edge position error */
   float fit_thres;      /* line fitting threshold */
} Hor_LF_Process_Params;

#ifdef _HORATIO_LIST_

void            hor_set_lf_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_lf_process_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_lf_add_process ( Hor_LF_Process_Params,
				     Hor_LI_Output_Params, Hor_Assoc_Label );

#endif /* _HORATIO_IMPROC_ */
void *hor_lf_execute ( Hor_List input_list, void *old_result,
		                            void *process_data );
#endif /* _HORATIO_LIST_ */
