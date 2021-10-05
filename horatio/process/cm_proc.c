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

static Hor_Assoc_Label cm_result_type_label  = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_cm_result_type_label ( Hor_Assoc_Label result_type_label )
*   Hor_Assoc_Label @hor_get_cm_result_type_label(void)
*   Hor_CM_Output_Params *@hor_cm_make_output_data  ( Hor_CM_Output_Params )
*   void  @hor_cm_output      ( void *cm_result, void *cm_data )
*   void  @hor_cm_free_result ( void *cm_result )
*
*   Corner matcher process definition functions.
********************/
void hor_set_cm_result_type_label ( Hor_Assoc_Label result_type_label )
{
   cm_result_type_label = result_type_label;
}

Hor_Assoc_Label hor_get_cm_result_type_label(void)
{
   return cm_result_type_label;
}

Hor_CM_Output_Params *hor_cm_make_output_data ( Hor_CM_Output_Params params )
{
   Hor_CM_Output_Params *data;

   data = hor_malloc_type ( Hor_CM_Output_Params );
   *data = params;
   return data;
}

void hor_cm_output ( void *cm_result, void *cm_data )
{
   Hor_Trajectory_Map   *result = (Hor_Trajectory_Map   *) cm_result;
   Hor_CM_Output_Params *data   = (Hor_CM_Output_Params *) cm_data;

   if ( result == NULL ) return;

   if ( result->traj_list == NULL )
      hor_message ( "no corner matches to display" );
   else
      hor_display_trajectory_map ( result, -1, data );
}

void hor_cm_free_result ( void *cm_result )
{
   if ( cm_result != NULL )
      hor_free_trajectory_map ( (Hor_Trajectory_Map *) cm_result );
}
