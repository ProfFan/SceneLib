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

static Hor_Assoc_Label fl_result_type_label  = HOR_ASSOC_ERROR;
static Hor_Assoc_Label fl_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_fl_result_type_label ( Hor_Assoc_Label result_type_label )
*   void @hor_set_fl_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_fl_result_type_label(void)
*   Hor_Assoc_Label @hor_get_fl_process_type_label(void)
*   Hor_FL_Process_Data *@hor_fl_make_process_data(Hor_FL_Process_Params prms,
*                                              int c1, int r1, int c2, int r2 )
*   Hor_FL_Output_Params *@hor_fl_make_output_data  ( Hor_FL_Output_Params )
*   void  @hor_fl_add_process ( Hor_FL_Process_Params prms,
*                              Hor_FL_Output_Params  out_prms,
*                              int  c1, int  r1, int  c2, int  r2 )
*   void *@hor_fl_execute             ( Hor_List input_list,
*                                      void *fl_old_result, void *fl_data )
*   void  @hor_fl_output              ( void *fl_result, void *fl_data )
*   void  @hor_fl_free_process_data   ( void *data )
*   void  @hor_fl_free_result         ( void *fl_result )
*
*   Image flow process definition functions.
********************/
void hor_set_fl_result_type_label ( Hor_Assoc_Label result_type_label )
{
   fl_result_type_label = result_type_label;
}

void hor_set_fl_process_type_label ( Hor_Assoc_Label process_type_label )
{
   fl_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_fl_result_type_label ( void )
{
   return fl_result_type_label;
}

Hor_Assoc_Label hor_get_fl_process_type_label ( void )
{
   return fl_process_type_label;
}

Hor_FL_Process_Data *hor_fl_make_process_data ( Hor_FL_Process_Params prms,
					        int c1, int r1, int c2, int r2)
{
   Hor_FL_Process_Data *data;

   data = hor_malloc_type ( Hor_FL_Process_Data );
   data->params = prms;
   data->gauss_mask = hor_make_gaussian_mask ( prms.sigma, prms.gauss_size );
   data->c1 = c1;
   data->r1 = r1;
   data->c2 = c2;
   data->r2 = r2;

   return data;
}

Hor_FL_Output_Params *hor_fl_make_output_data ( Hor_FL_Output_Params out_prms )
{
   Hor_FL_Output_Params *data;

   data = hor_malloc_type ( Hor_FL_Output_Params );
   *data = out_prms;
   return data;
}

void hor_fl_add_process ( Hor_FL_Process_Params proc_prms,
			  Hor_FL_Output_Params  out_prms,
			  int c1, int r1, int c2, int r2 )
{
   if ( fl_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "image flow process type not set (hor_fl_add_process)",
		  HOR_FATAL );

   hor_add_process ( fl_process_type_label,
		     hor_make_assoc_label_list ( hor_get_gb_process_label(),
						 HOR_ASSOC_END ),
		     hor_fl_make_process_data ( proc_prms, c1, r1, c2, r2 ),
		     hor_fl_make_output_data ( out_prms ) );
}

void *hor_fl_execute ( Hor_List input_list, void *fl_old_result, void *fl_data)
{
   Hor_Image_Flow      *old_result = (Hor_Image_Flow      *) fl_old_result;
   Hor_FL_Process_Data *data       = (Hor_FL_Process_Data *) fl_data;

   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      hor_node_contents(input_list);
   Hor_Image         *image;
   Hor_Sub_Image     *old_image;

   if ( process_input->process_label     != hor_get_gb_process_label() ||
        process_input->result_type_label != hor_get_gb_result_type_label() )
      hor_error ( "corrupted process input list (hor_fl_execute)", HOR_FATAL );

   if ( old_result == NULL ) old_image = NULL;
   else                      old_image = old_result->old_image;

   image = (Hor_Image *) process_input->process_result;
   if ( image == NULL ) return NULL;

   return ( (void *) hor_image_flow (image, old_image,
				     data->gauss_mask, data->params.gauss_size,
				     data->params.rc_deriv_threshold,
				     data->params.t_deriv_threshold,
				     data->c1, data->r1, data->c2, data->r2) );
}

void hor_fl_output ( void *fl_result, void *fl_data )
{
   Hor_Image_Flow       *result = (Hor_Image_Flow       *) fl_result;
   Hor_FL_Output_Params *data   = (Hor_FL_Output_Params *) fl_data;

   if ( result == NULL ) return;

   if ( result->flow_c == NULL )
      hor_message ( "no image flow results to display yet" );
   else
      hor_display_image_flow ( result->flow_c, result->flow_r,
			       result->flow_flag,
			       data->start_colour, data->line_colour,
			       data->scale, data->increment );
}

void hor_fl_free_process_data ( void *fl_data )
{
   Hor_FL_Process_Data *data = (Hor_FL_Process_Data *) fl_data;

   hor_free_gaussian_mask ( data->gauss_mask, data->params.gauss_size );
   hor_free ( (void *) data );
}

void hor_fl_free_result ( void *fl_result )
{
   Hor_Image_Flow *result = (Hor_Image_Flow *) fl_result;

   hor_free_image_flow ( result );
}
