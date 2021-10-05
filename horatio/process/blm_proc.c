/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"

static Hor_Assoc_Label blm_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_blm_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_blm_process_type_label(void)
*   Hor_BLM_Process_Data *@hor_blm_make_process_data(Hor_BLM_Process_Params)
*   void  @hor_blm_add_process ( Hor_BLM_Process_Params blm_process_params,
*                               Hor_LM_Output_Params  lm_output_params,
*                               Hor_Assoc_Label       line_process )
*   void *@hor_blm_execute ( Hor_List input_list,
*                           void *blm_old_result, void *blm_data )
*
*   Bog-standard line matcher process definition functions.
********************/
void hor_set_blm_process_type_label ( Hor_Assoc_Label process_type_label )
{
   blm_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_blm_process_type_label(void)
{
   return blm_process_type_label;
}

Hor_BLM_Process_Params *hor_blm_make_process_data
                                       ( Hor_BLM_Process_Params params )
{
   Hor_BLM_Process_Params *data;

   data = hor_malloc_type ( Hor_BLM_Process_Params );
   *data = params;
   return data;
}

void hor_blm_add_process ( Hor_BLM_Process_Params blm_process_params,
			   Hor_LM_Output_Params   lm_output_params,
			   Hor_Assoc_Label        line_process )
{
   if ( blm_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "line matcher process type not set (hor_blm_add_process)",
		  HOR_FATAL);

   hor_add_process ( blm_process_type_label,
		     hor_make_assoc_label_list ( line_process,HOR_ASSOC_END),
		     hor_blm_make_process_data ( blm_process_params ),
		     hor_lm_make_output_data ( lm_output_params ) );
}

void *hor_blm_execute ( Hor_List input_list,
		        void *blm_old_result, void *blm_data )
{
   Hor_Trajectory_Map     *old_result = (Hor_Trajectory_Map *) blm_old_result;
   Hor_BLM_Process_Params *data = (Hor_BLM_Process_Params *) blm_data;
   Hor_Line_Segment_Map   *lines;
   Hor_Process_Input      *process_input;
   Hor_Trajectory_Map     *new_result;

   if ( input_list == NULL  )
      hor_error ( "process input list too short (hor_blm_execute)", HOR_FATAL);

   process_input = (Hor_Process_Input *) input_list->contents;
   if ( process_input->result_type_label != hor_get_li_result_type_label() )
      hor_error ( "corrupted process input list (hor_blm_execute)", HOR_FATAL);

   lines = (Hor_Line_Segment_Map *) process_input->process_result;
   if ( lines == NULL ) return NULL;

   if ( old_result == NULL )
      new_result = hor_alloc_trajectory_map ( HOR_TRAJ_MAP_NO_STATE,
					      NULL, NULL );
   else /* copy fields of old result into new result */
   {
      new_result = hor_malloc_type(Hor_Trajectory_Map);
      *new_result = *old_result;
      old_result->traj_list = NULL;
   }

   hor_bog_match_lines ( new_result, lines, data->dist_thresh,
			 data->cos_thresh, data->size_thresh,
			 data->iterations );
   return ( (void *) new_result );
}
