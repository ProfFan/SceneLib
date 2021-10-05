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

static Hor_Assoc_Label cl_result_type_label  = HOR_ASSOC_ERROR;
static Hor_Assoc_Label cl_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_cl_result_type_label ( Hor_Assoc_Label result_type_label )
*   void @hor_set_cl_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_cl_result_type_label(void)
*   Hor_Assoc_Label @hor_get_cl_process_type_label(void)
*   Hor_CL_Process_Data *@hor_cl_make_process_data(Hor_CL_Process_Params prms,
*                                               int c1, int r1, int c2, int r2)
*   Hor_CL_Output_Params *@hor_cl_make_output_data ( float display_scale )
*   void  @hor_cl_add_process ( Hor_CL_Process_Params prms,
*                             int c1, int r1, int c2, int r2 )
*   void *@hor_cl_execute     ( Hor_List  input_list,
*                              void *cl_old_result, void *  cl_data )
*   void  @hor_cl_output      ( void *cl_result, void *cl_data )
*   void  @hor_cl_free_result ( void *cl_result )
*
*   Hor_Correlation process definition functions.
********************/
void hor_set_cl_result_type_label ( Hor_Assoc_Label result_type_label )
{
   cl_result_type_label = result_type_label;
}

void hor_set_cl_process_type_label ( Hor_Assoc_Label process_type_label )
{
   cl_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_cl_result_type_label ( void )
{
   return cl_result_type_label;
}

Hor_Assoc_Label hor_get_cl_process_type_label ( void )
{
   return cl_process_type_label;
}

Hor_CL_Process_Data *hor_cl_make_process_data ( Hor_CL_Process_Params prms,
					        int c1, int r1, int c2, int r2)
{
   Hor_CL_Process_Data *data;

   data = hor_malloc_type ( Hor_CL_Process_Data );
   data->params = prms;
   data->c1     = c1;
   data->r1     = r1;
   data->c2     = c2;
   data->r2     = r2;
   return data;
}

Hor_CL_Output_Params *hor_cl_make_output_data ( float display_scale )
{
   Hor_CL_Output_Params *data;

   data = hor_malloc_type ( Hor_CL_Output_Params );
   data->display_scale = display_scale;
   return data;
}

void hor_cl_add_process ( Hor_CL_Process_Params prms,
			  int c1, int r1, int c2, int r2 )
{
   if ( cl_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "hor_correlation process type not set (hor_cl_add_process)",
		  HOR_FATAL );

   hor_add_process ( cl_process_type_label,
		     hor_make_assoc_label_list ( hor_get_gb_process_label(),
						 HOR_ASSOC_END ),
		     hor_cl_make_process_data ( prms, c1, r1, c2, r2 ),
		     hor_cl_make_output_data ( prms.display_scale ) );
}

void *hor_cl_execute ( Hor_List input_list, void *cl_old_result, void *cl_data)
{
   Hor_Correlation     *old_result = (Hor_Correlation     *) cl_old_result;
   Hor_CL_Process_Data *data       = (Hor_CL_Process_Data *) cl_data;

   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      input_list->contents;
   Hor_Image         *image;
   Hor_Sub_Image     *old_image;

   if ( process_input->process_label     != hor_get_gb_process_label() ||
        process_input->result_type_label != hor_get_gb_result_type_label() )
      hor_error ( "corrupted process input list (hor_cl_execute)", HOR_FATAL );

   if ( old_result == NULL ) old_image = NULL;
   else                      old_image = old_result->image;

   image = (Hor_Image *) process_input->process_result;
   if ( image == NULL ) return NULL;

   return ( (void *)
	    hor_correlation ( image, old_image,
			      data->c1, data->r1, data->c2, data->r2,
			      data->params.patch_size,
			      data->params.patch_density,
			      data->params.max_motion,
			      data->params.display_scale,
			      data->params.display_colour ) );
}

void hor_cl_output ( void *cl_result, void *cl_data )
{
   Hor_Correlation  *result = (Hor_Correlation  *) cl_result;

   if ( result == NULL ) return;
/*
   hor_display_highlight_region ( result->c0, result->r0,
			          result->image.width, result->image.height );
   hor_display_sub_image ( result, 1, 0.0, 256.0/data->display_scale );
*/
}

void hor_cl_free_result ( void *cl_result )
{
   Hor_Correlation *result = (Hor_Correlation *) cl_result;

   if ( result == NULL ) return;

   hor_free_sub_image ( result->image );
   hor_free ( (void *) result );
}
