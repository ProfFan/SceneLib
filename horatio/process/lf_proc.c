/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"

static Hor_Assoc_Label lf_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_lf_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_lf_process_type_label(void)
*   Hor_Assoc_Label @hor_lf_add_process ( Hor_LF_Process_Params proc_prms,
*                                        Hor_LI_Output_Params  out_prms,
*                                        Hor_Assoc_Label       canny_process )
*   void *@hor_lf_execute             ( Hor_List  input_list,
*                                      void *lf_old_result, void *lf_data )
*
*   Orthogonal regression line segment fitter process definition functions.
********************/
void hor_set_lf_process_type_label ( Hor_Assoc_Label process_type_label )
{
   lf_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_lf_process_type_label ( void )
{
   return lf_process_type_label;
}

Hor_Assoc_Label hor_lf_add_process ( Hor_LF_Process_Params proc_prms,
				     Hor_LI_Output_Params  out_prms,
				     Hor_Assoc_Label       canny_process )
{
   Hor_LF_Process_Params *prms = hor_malloc_type(Hor_LF_Process_Params);

   if ( lf_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "Line segment process type not set (hor_lf_add_process)",
		  HOR_FATAL );

   *prms = proc_prms;
   return ( hor_add_process ( lf_process_type_label,
			      hor_make_assoc_label_list ( canny_process,
							  HOR_ASSOC_END ),
			      prms, hor_li_make_output_data ( out_prms ) ) );
}

void *hor_lf_execute (Hor_List input_list, void *lf_old_result, void *lf_data)
{
   Hor_LF_Process_Params *prms = (Hor_LF_Process_Params *) lf_data;

   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      input_list->contents;
   Hor_Edge_Map      *edge_map;

   if ( process_input->result_type_label != hor_get_ed_result_type_label() )
      hor_error ( "corrupted process input list (hor_lf_execute)", HOR_FATAL );

   edge_map = (Hor_Edge_Map *) process_input->process_result;
   if ( edge_map == NULL ) return NULL;

   return ( (void *) hor_find_lines ( edge_map, prms->no_points,
				      prms->sigma, prms->fit_thres ) );
}
