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

static Hor_Assoc_Label li_result_type_label  = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_li_result_type_label ( Hor_Assoc_Label result_type_label )
*   Hor_Assoc_Label @hor_get_li_result_type_label(void)
*   Hor_LI_Output_Params *@hor_li_make_output_data ( Hor_LI_Output_Params );
*   void  @hor_li_output              ( void *li_result, void *li_data )
*   void  @hor_li_free_result         ( void *li_result )
*
*   General line segment fitter process definition functions.
********************/
void hor_set_li_result_type_label ( Hor_Assoc_Label result_type_label )
{
   li_result_type_label = result_type_label;
}

Hor_Assoc_Label hor_get_li_result_type_label ( void )
{
   return li_result_type_label;
}

Hor_LI_Output_Params *hor_li_make_output_data ( Hor_LI_Output_Params hor_prms )
{
   Hor_LI_Output_Params *data;

   data = hor_malloc_type ( Hor_LI_Output_Params );
   *data = hor_prms;
   return data;
}

void hor_li_output ( void *li_result, void *li_data )
{
   Hor_Line_Segment_Map *result = (Hor_Line_Segment_Map *) li_result;

   if ( result == NULL ) return;

   hor_display_line_segments ( result, HOR_NO_ATTRIB, li_data );
}

void hor_li_free_result ( void *result )
{
   hor_free_line_segment_map ( (Hor_Line_Segment_Map *) result );
}
