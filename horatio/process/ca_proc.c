/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"

static Hor_Assoc_Label ca_process_type_label = HOR_ASSOC_ERROR;

/*******************
*   void @hor_set_ca_process_type_label ( Hor_Assoc_Label process_type_label )
*   Hor_Assoc_Label @hor_get_ca_process_type_label(void)
*   Hor_CA_Process_Data *@hor_ca_make_process_data(Hor_CA_Process_Params prms,
*                                         int cv1, int rv1, int cv2, int rv2,
*                                         int  c1, int  r1, int  c2, int  r2 )
*   Hor_Assoc_Label @hor_ca_add_process ( Hor_CA_Process_Params proc_prms,
*                                        Hor_ED_Output_Params  out_prms,
*                                        int cv1, int rv1, int cv2, int rv2,
*                                        int  c1, int  r1, int  c2, int  r2 )
*   void *@hor_ca_execute             ( Hor_List  input_list,
*                                      void *ca_old_result, void *ca_data )
*   void  @hor_ca_update_process_data ( void *data )
*   void  @hor_ca_free_process_data   ( void *data )
*
*   Canny operator process definition functions.
********************/
void hor_set_ca_process_type_label ( Hor_Assoc_Label process_type_label )
{
   ca_process_type_label = process_type_label;
}

Hor_Assoc_Label hor_get_ca_process_type_label(void)
{
   return ca_process_type_label;
}

Hor_CA_Process_Data *hor_ca_make_process_data ( Hor_CA_Process_Params prms,
				        int cv1, int rv1, int cv2, int rv2,
				        int  c1, int  r1, int  c2, int  r2 )
{
   Hor_CA_Process_Data *data;

   data = hor_malloc_type ( Hor_CA_Process_Data );
   data->fixed.params = prms;
   data->fixed.gauss_mask = hor_make_gaussian_mask(prms.sigma,prms.gauss_size);
   data->fixed.cv1    = cv1;
   data->fixed.rv1    = rv1;
   data->fixed.cv2    = cv2;
   data->fixed.rv2    = rv2;
   data->writeable.c1 = c1;
   data->writeable.r1 = r1;
   data->writeable.c2 = c2;
   data->writeable.r2 = r2;
   return data;
}

Hor_Assoc_Label hor_ca_add_process ( Hor_CA_Process_Params proc_prms,
				     Hor_ED_Output_Params  out_prms,
				     int cv1, int rv1, int cv2, int rv2,
				     int  c1, int  r1, int  c2, int  r2 )
{
   if ( ca_process_type_label == HOR_ASSOC_ERROR )
      hor_error ( "Canny process type not set (hor_ca_add_process)",
		  HOR_FATAL );

   return ( hor_add_process (
	       ca_process_type_label,
	       hor_make_assoc_label_list ( hor_get_gb_process_label(),
					   HOR_ASSOC_END ),
	       hor_ca_make_process_data ( proc_prms,
					  cv1, rv1, cv2, rv2, c1, r1, c2, r2 ),
	       hor_ed_make_output_data ( out_prms ) ) );
}

void *hor_ca_execute (Hor_List input_list, void *ca_old_result, void *ca_data)
{
   Hor_CA_Process_Data *data = (Hor_CA_Process_Data *) ca_data;

   Hor_Process_Input *process_input = (Hor_Process_Input *)
                                      input_list->contents;
   Hor_Image         *image;

   if ( process_input->process_label     != hor_get_gb_process_label() ||
        process_input->result_type_label != hor_get_gb_result_type_label() )
      hor_error ( "corrupted process input list (hor_ca_execute)", HOR_FATAL );

   image = (Hor_Image *) process_input->process_result;
   if ( image == NULL ) return NULL;

   return ( (void *) hor_canny ( image,
				 data->fixed.gauss_mask,
				 data->fixed.params.gauss_size,
				 data->fixed.params.low_thres,
				 data->fixed.params.high_thres,
				 data->fixed.params.length_thres,
				 data->writeable.c1, data->writeable.r1,
				 data->writeable.c2, data->writeable.r2 ) );
}

void hor_ca_update_process_data ( void *ca_data )
{
   Hor_CA_Process_Data *data = (Hor_CA_Process_Data *) ca_data;

   data->writeable.c1 += data->fixed.cv1;
   data->writeable.r1 += data->fixed.rv1;
   data->writeable.c2 += data->fixed.cv2;
   data->writeable.r2 += data->fixed.rv2;
}

void hor_ca_free_process_data ( void *ca_data )
{
   Hor_CA_Process_Data *data = (Hor_CA_Process_Data *) ca_data;

   hor_free_gaussian_mask ( data->fixed.gauss_mask,
			    data->fixed.params.gauss_size);
   hor_free ( (void *) data );
}
