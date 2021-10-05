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

static Hor_Assoc_Label ed_result_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_ed_result_type_label ( Hor_Assoc_Label result_type_label )
*   Hor_Assoc_Label @hor_get_ed_result_type_label(void)
*   Hor_ED_Output_Params *@hor_ed_make_output_data ( Hor_ED_Output_Params );
*   void @hor_ed_output      ( void *ed_result, void *ed_data )
*   void @hor_ed_free_result ( void *ed_result )
*
*   Edge detector operator process definition functions.
********************/
void hor_set_ed_result_type_label ( Hor_Assoc_Label result_type_label )
{
   ed_result_type_label = result_type_label;
}

Hor_Assoc_Label hor_get_ed_result_type_label(void)
{
   return ed_result_type_label;
}

Hor_ED_Output_Params *hor_ed_make_output_data ( Hor_ED_Output_Params hor_prms )
{
   Hor_ED_Output_Params *data;

   data = hor_malloc_type ( Hor_ED_Output_Params );
   *data = hor_prms;
   return data;
}

void hor_ed_output ( void *ed_result, void *ed_data )
{
   Hor_Edge_Map *result = (Hor_Edge_Map *) ed_result;

   if ( result == NULL ) return;
   hor_display_edge_map ( result, HOR_NO_ATTRIB, HOR_NO_ATTRIB, ed_data );
}

void hor_ed_free_result ( void *result )
{
   hor_free_edge_map ( (Hor_Edge_Map *) result );
}
