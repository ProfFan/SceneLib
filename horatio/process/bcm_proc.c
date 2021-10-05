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

static Hor_Assoc_Label bcm_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_bcm_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_bcm_process_type_label(void)
*   Hor_BCM_Process_Data *@hor_bcm_make_process_data(Hor_BCM_Process_Params)
*   void  @hor_bcm_add_process ( Hor_BCM_Process_Params bcm_process_params,
*                               Hor_CM_Output_Params  cm_output_params,
*                               Hor_Assoc_Label       corner_process )
*   void *@hor_bcm_execute ( Hor_List input_list,
*                           void *bcm_old_result, void *bcm_data )
*
*   Bog-standard corner matcher process definition functions.
********************/
void hor_set_bcm_process_type_label ( Hor_Assoc_Label process_type_label )
{
   bcm_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_bcm_process_type_label(void)
{
   return bcm_process_type_label;
}

Hor_BCM_Process_Params *hor_bcm_make_process_data
                                       ( Hor_BCM_Process_Params params )
{
   Hor_BCM_Process_Params *data;

   data = hor_malloc_type ( Hor_BCM_Process_Params );
   *data = params;
   return data;
}

void hor_bcm_add_process ( Hor_BCM_Process_Params bcm_process_params,
			   Hor_CM_Output_Params   cm_output_params,
			   Hor_Assoc_Label        corner_process )
{
   if ( bcm_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "corner matcher process type not set (hor_bcm_add_process)",
		  HOR_FATAL);

   hor_add_process ( bcm_process_type_label,
		     hor_make_assoc_label_list ( corner_process,HOR_ASSOC_END),
		     hor_bcm_make_process_data ( bcm_process_params ),
		     hor_cm_make_output_data ( cm_output_params ) );
}

void *hor_bcm_execute ( Hor_List input_list,
		        void *bcm_old_result, void *bcm_data )
{
   Hor_Trajectory_Map *old_result = (Hor_Trajectory_Map *) bcm_old_result;
   Hor_BCM_Process_Params *data = (Hor_BCM_Process_Params *) bcm_data;
   Hor_Corner_Map         *corner_map;
   Hor_Process_Input      *process_input;
   Hor_Trajectory_Map *new_result;

   if ( input_list == NULL  )
      hor_error ( "process input list too short (hor_bcm_execute)", HOR_FATAL);

   process_input = (Hor_Process_Input *) input_list->contents;
   if ( process_input->result_type_label != hor_get_co_result_type_label() )
      hor_error ( "corrupted process input list (hor_bcm_execute)", HOR_FATAL);

   corner_map = (Hor_Corner_Map *) process_input->process_result;
   if ( corner_map == NULL ) return NULL;

   if ( old_result == NULL )
      new_result = hor_alloc_trajectory_map ( HOR_TRAJ_MAP_NO_STATE,
					      NULL, NULL );
   else /* copy fields of old result into new result */
   {
      new_result = hor_malloc_type(Hor_Trajectory_Map);
      *new_result = *old_result;
      old_result->traj_list = NULL;
   }

   hor_bog_match_corners ( new_result, corner_map, data->range,
			   data->imdiff_ratio_thres, data->iterations );
   return ( (void *) new_result );
}
