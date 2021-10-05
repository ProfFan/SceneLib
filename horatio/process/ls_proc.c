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

static Hor_Assoc_Label ls_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_ls_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_ls_process_type_label(void)
*   Hor_Assoc_Label @hor_ls_add_process ( Hor_LI_Output_Params  out_prms,
*                                        Hor_Assoc_Label canny_process )
*   void *@hor_ls_execute             ( Hor_List  input_list,
*                                      void *ls_old_result, void *ls_data )
*
*   Will Dickson's line segment fitter process definition functions.
********************/
void hor_set_ls_process_type_label ( Hor_Assoc_Label process_type_label )
{
   ls_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_ls_process_type_label ( void )
{
   return ls_process_type_label;
}

Hor_Assoc_Label hor_ls_add_process ( Hor_LI_Output_Params out_prms,
				     Hor_Assoc_Label canny_process )
{
   if ( ls_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "Line segment process type not set (hor_ls_add_process)",
		  HOR_FATAL );

   return ( hor_add_process ( ls_process_type_label,
			      hor_make_assoc_label_list ( canny_process,
							  HOR_ASSOC_END ),
			      NULL, hor_li_make_output_data ( out_prms ) ) );
}

void *hor_ls_execute (Hor_List input_list, void *ls_old_result, void *ls_data)
{
   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      input_list->contents;
   Hor_Edge_Map      *edge_map;

   if ( process_input->result_type_label != hor_get_ed_result_type_label() )
      hor_error ( "corrupted process input list (hor_ls_execute)", HOR_FATAL );

   edge_map = (Hor_Edge_Map *) process_input->process_result;
   if ( edge_map == NULL ) return NULL;

   return ( (void *) hor_fit_line_segments ( edge_map, 0.05 ) );
}
