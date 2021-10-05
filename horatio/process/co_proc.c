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

static Hor_Assoc_Label co_result_type_label  = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_co_result_type_label ( Hor_Assoc_Label result_type_label )
*   Hor_Assoc_Label @hor_get_co_result_type_label(void)
*   Hor_CO_Output_Params *hor_co_make_output_data ( Hor_CO_Output_Params )
*   void  @hor_co_output      ( void *co_result, void *co_data )
*   void  @hor_co_free_result ( void *co_result )
*
*   General corner detector process definition functions.
********************/
void hor_set_co_result_type_label ( Hor_Assoc_Label result_type_label )
{
   co_result_type_label = result_type_label;
}

Hor_Assoc_Label hor_get_co_result_type_label ( void )
{
   return co_result_type_label;
}

Hor_CO_Output_Params *hor_co_make_output_data ( Hor_CO_Output_Params out_prms )
{
   Hor_CO_Output_Params *data;

   data = hor_malloc_type ( Hor_CO_Output_Params );
   *data = out_prms;
   return data;
}

void hor_co_output ( void *co_result, void *co_data )
{
   Hor_Corner_Map *result = (Hor_Corner_Map *) co_result;
   
   if ( result == NULL ) return;

   hor_display_corner_map ( result, HOR_NO_ATTRIB, co_data );
}

void hor_co_free_result ( void *result )
{
   hor_free_corner_map ( (Hor_Corner_Map *) result );
}
