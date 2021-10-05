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

static Hor_Assoc_Label dcm_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_dcm_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_dcm_process_type_label(void)
*   Hor_DCM_Process_Data *@hor_dcm_make_process_data(Hor_DCM_Process_Params)
*   void  @hor_dcm_add_process ( Hor_DCM_Process_Params dcm_process_params,
*                               Hor_CM_Output_Params  cm_output_params,
*                               Hor_Assoc_Label       corner_process )
*   void *@hor_dcm_execute ( Hor_List input_list,
*                           void *dcm_old_result, void *dcm_data )
*
*   Brain dead corner matcher process definition functions.
********************/
void hor_set_dcm_process_type_label ( Hor_Assoc_Label process_type_label )
{
   dcm_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_dcm_process_type_label(void)
{
   return dcm_process_type_label;
}

Hor_DCM_Process_Params *hor_dcm_make_process_data
                                       ( Hor_DCM_Process_Params params )
{
   Hor_DCM_Process_Params *data;

   data = hor_malloc_type ( Hor_DCM_Process_Params );
   *data = params;
   return data;
}

void hor_dcm_add_process ( Hor_DCM_Process_Params dcm_process_params,
			   Hor_CM_Output_Params   cm_output_params,
			   Hor_Assoc_Label        corner_process )
{
   if ( dcm_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "corner matcher process type not set (hor_dcm_add_process)",
		  HOR_FATAL);

   hor_add_process ( dcm_process_type_label,
		     hor_make_assoc_label_list ( corner_process,HOR_ASSOC_END),
		     hor_dcm_make_process_data ( dcm_process_params ),
		     hor_cm_make_output_data ( cm_output_params ) );
}

void *hor_dcm_execute ( Hor_List input_list,
		        void *dcm_old_result, void *dcm_data )
{
   Hor_Trajectory_Map *old_result = (Hor_Trajectory_Map *) dcm_old_result;
   Hor_DCM_Process_Params *data = (Hor_DCM_Process_Params *) dcm_data;
   Hor_Corner_Map         *corner_map;
   Hor_Process_Input      *process_input;
   Hor_Trajectory_Map *new_result;

   if ( input_list == NULL  )
      hor_error ( "process input list too short (hor_dcm_execute)", HOR_FATAL);

   process_input = (Hor_Process_Input *) input_list->contents;
   if ( process_input->result_type_label != hor_get_co_result_type_label() )
      hor_error ( "corrupted process input list (hor_dcm_execute)", HOR_FATAL);

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

   hor_bd_match_corners ( new_result, corner_map,
			  data->match_window, data->corr_thres );
   return ( (void *) new_result );
}
