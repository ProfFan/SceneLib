/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"

static Hor_Assoc_Label is_result_type_label  = HOR_ASSOC_ERROR;
static Hor_Assoc_Label is_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_is_result_type_label ( Hor_Assoc_Label result_type_label )
*   void @hor_set_is_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_is_result_type_label(void)
*   Hor_Assoc_Label @hor_get_is_process_type_label(void)
*   Hor_IS_Process_Data *@hor_is_make_process_data(Hor_IS_Process_Params prms,
*                                               int c1, int r1, int c2, int r2)
*   Hor_IS_Output_Params *@hor_is_make_output_data ( float sd_scale )
*   void  @hor_is_add_process ( Hor_IS_Process_Params prms,
*                              int c1, int r1, int c2, int r2 )
*   void *@hor_is_execute     ( Hor_List input_list,
*                              void *is_old_result, void *is_data )
*   void  @hor_is_output      ( void *is_result, void *is_data )
*   void  @hor_is_free_result ( void *is_result )
*
*   Region-based image segmentation process definition functions.
********************/
void hor_set_is_result_type_label ( Hor_Assoc_Label result_type_label )
{
   is_result_type_label = result_type_label;
}

void hor_set_is_process_type_label ( Hor_Assoc_Label process_type_label )
{
   is_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_is_result_type_label ( void )
{
   return is_result_type_label;
}

Hor_Assoc_Label hor_get_is_process_type_label ( void )
{
   return is_process_type_label;
}

Hor_IS_Process_Data *hor_is_make_process_data ( Hor_IS_Process_Params prms,
					        int c1, int r1, int c2, int r2)
{
   Hor_IS_Process_Data *data;

   data = hor_malloc_type ( Hor_IS_Process_Data );
   data->params = prms;
   data->c1     = c1;
   data->r1     = r1;
   data->c2     = c2;
   data->r2     = r2;
   return data;
}

Hor_IS_Output_Params *hor_is_make_output_data ( float sd_scale )
{
   Hor_IS_Output_Params *data;

   data = hor_malloc_type ( Hor_IS_Output_Params );
   data->sd_scale = sd_scale;
   return data;
}

void hor_is_add_process ( Hor_IS_Process_Params prms,
		      int c1, int r1, int c2, int r2 )
{
   if ( is_process_type_label == HOR_ASSOC_ERROR )
      hor_error ("hor_image_segment process type not set (hor_is_add_process)",
		 HOR_FATAL );

   hor_add_process ( is_process_type_label,
		     hor_make_assoc_label_list ( hor_get_gb_process_label(),
						 HOR_ASSOC_END ),
		     hor_is_make_process_data ( prms, c1, r1, c2, r2 ),
		     hor_is_make_output_data ( prms.sd_scale ) );
}

void *hor_is_execute ( Hor_List input_list, void *is_old_result, void *is_data)
{
   Hor_IS_Process_Data *data        = (Hor_IS_Process_Data *) is_data;

   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      input_list->contents;
   Hor_Image         *image;

   if ( process_input->process_label     != hor_get_gb_process_label() ||
        process_input->result_type_label != hor_get_gb_result_type_label() )
      hor_error ( "corrupted process input list (hor_is_execute)", HOR_FATAL );

   image = (Hor_Image *) process_input->process_result;
   if ( image == NULL ) return NULL;

   return ( (void *) hor_image_segment (image,
					data->c1, data->r1, data->c2, data->r2,
					data->params.patch_size,
					data->params.patch_density,
					data->params.low_threshold,
					data->params.threshold_step,
					data->params.high_threshold,
					data->params.sd_scale,
					data->params.thres_colour ) );
}

void hor_is_output ( void *is_result, void *is_data )
{
   Hor_Image_Segment  *result = (Hor_Image_Segment  *) is_result;

   if ( result == NULL ) return;
/*
   hor_display_highlight_region ( result->c0, result->r0,
                                  result->image.width, result->image.height );
   hor_display_sub_image ( result, 1, 0.0, 256.0/data->sd_scale );
*/
}

void hor_is_free_result ( void *result )
{
   hor_free ( (void *) result );
}
