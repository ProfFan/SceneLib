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

static Hor_Assoc_Label lm_result_type_label  = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_lm_result_type_label ( Hor_Assoc_Label result_type_label )
*   Hor_Assoc_Label @hor_get_lm_result_type_label(void)
*   Hor_LM_Output_Params *@hor_lm_make_output_data  ( Hor_LM_Output_Params )
*   void  @hor_lm_output      ( void *lm_result, void *lm_data )
*   void  @hor_lm_free_result ( void *lm_result )
*
*   Line matcher process definition functions.
********************/
void hor_set_lm_result_type_label ( Hor_Assoc_Label result_type_label )
{
   lm_result_type_label = result_type_label;
}

Hor_Assoc_Label hor_get_lm_result_type_label(void)
{
   return lm_result_type_label;
}

Hor_LM_Output_Params *hor_lm_make_output_data ( Hor_LM_Output_Params params )
{
   Hor_LM_Output_Params *data;

   data = hor_malloc_type ( Hor_LM_Output_Params );
   *data = params;
   return data;
}

void hor_lm_output ( void *lm_result, void *lm_data )
{
   Hor_Trajectory_Map   *result = (Hor_Trajectory_Map   *) lm_result;
   Hor_LM_Output_Params *data   = (Hor_LM_Output_Params *) lm_data;

   if ( result == NULL ) return;

   if ( result->traj_list == NULL )
      hor_message ( "no line matches to display" );
   else
      hor_display_trajectory_map ( result, HOR_LM_BOG_STANDARD, data );
}

void hor_lm_free_result ( void *lm_result )
{
   if ( lm_result != NULL )
      hor_free_trajectory_map ( (Hor_Trajectory_Map *) lm_result );
}
