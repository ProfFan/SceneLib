/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"

/*******************
*   void @hor_init_process_stuff(void)
*
*   Create all Horatio process and result types.
********************/
void hor_init_process_stuff(void)
{
   Hor_List result_type_list = NULL;

   /* register grab process result type */
   hor_set_gb_result_type_label ( hor_add_result_type ( hor_gb_output,
						        hor_gb_free_result,
						        NULL ) );

   /* register process type to grab */
   hor_set_gb_process_type_label ( hor_add_process_type ( NULL,
						hor_get_gb_result_type_label(),
					        hor_gb_execute,
						hor_gb_update_process_data,
						hor_gb_free_process_data ) );

   /* create list with grab result type (image) as only member for input
      to Canny, corner and other processes */
   result_type_list = hor_make_assoc_label_list(hor_get_gb_result_type_label(),
						HOR_ASSOC_END );

   /* register result and process types for Canny edge detector processes */
   hor_set_ed_result_type_label ( hor_add_result_type ( hor_ed_output,
						        hor_ed_free_result,
						        hor_free_func ) );

   hor_set_ca_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_ed_result_type_label(),
						hor_ca_execute,
						hor_ca_update_process_data,
						hor_ca_free_process_data ) );

   /* register result and process types for corner processes */
   hor_set_co_result_type_label ( hor_add_result_type ( hor_co_output,
						        hor_co_free_result,
						        hor_free_func ) );

   /* Plessey corner detector process type */
   hor_set_pc_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_co_result_type_label(),
						hor_pc_execute,
						hor_pc_update_process_data,
						hor_pc_free_process_data ) );

   /* Smith corner detector process type */
   hor_set_sc_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_co_result_type_label(),
						hor_sc_execute,
						hor_sc_update_process_data,
						hor_free_func ) );

   /* Wang/Brady corner detector process type */
   hor_set_cow_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_co_result_type_label(),
					       hor_cow_execute,
					       hor_cow_update_process_data,
					       hor_free_func ) );

   /* register result and process types for image flow processes */
   hor_set_fl_result_type_label ( hor_add_result_type ( hor_fl_output,
						        hor_fl_free_result,
						        hor_free_func ) );
   hor_set_fl_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_fl_result_type_label(),
						hor_fl_execute, NULL,
						hor_fl_free_process_data ) );

   /* register result and process types for image segment processes */
   hor_set_is_result_type_label ( hor_add_result_type ( hor_is_output,
						        hor_is_free_result,
						        hor_free_func ) );
   hor_set_is_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_is_result_type_label(),
						hor_is_execute, NULL,
						hor_free_func ) );

   /* register result and process types for hor_correlation processes */
   hor_set_cl_result_type_label ( hor_add_result_type ( hor_cl_output,
						        hor_cl_free_result,
						        hor_free_func ) );
   hor_set_cl_process_type_label ( hor_add_process_type ( result_type_list,
						hor_get_cl_result_type_label(),
						hor_cl_execute, NULL,
						hor_free_func ) );

   hor_free_assoc_label_list ( result_type_list );

   /* create list with corner map result type as only member
      for input to corner matching process */
   result_type_list = hor_make_assoc_label_list(hor_get_co_result_type_label(),
						HOR_ASSOC_END );
   /* register result and process types for corner matching processes */
   hor_set_cm_result_type_label ( hor_add_result_type ( hor_cm_output,
						        hor_cm_free_result,
						        hor_free_func ) );
   hor_set_bcm_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_cm_result_type_label(),
					       hor_bcm_execute, NULL,
					       hor_free_func ) );
   hor_set_dcm_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_cm_result_type_label(),
					       hor_dcm_execute, NULL,
					       hor_free_func ) );
   hor_free_assoc_label_list ( result_type_list );

   /* create list with canny result type as only member
      for input to line segment fitting process */
   result_type_list = hor_make_assoc_label_list(hor_get_ed_result_type_label(),
						HOR_ASSOC_END );
   /* register result type for line segment fitting processes */
   hor_set_li_result_type_label ( hor_add_result_type ( hor_li_output,
						        hor_li_free_result,
						        hor_free_func ) );

   /* register process types for line segment fitting algorithms */
   hor_set_lf_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_li_result_type_label(),
					       hor_lf_execute, NULL,
					       hor_free_func ) );
   hor_set_ls_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_li_result_type_label(),
					       hor_ls_execute, NULL, NULL ) );
   hor_free_assoc_label_list ( result_type_list );

   /* create list with line segment list result type as only member
      for input to line matching process */
   result_type_list = hor_make_assoc_label_list(hor_get_li_result_type_label(),
						HOR_ASSOC_END );
   /* register result and process types for line matching processes */
   hor_set_lm_result_type_label ( hor_add_result_type ( hor_lm_output,
						        hor_lm_free_result,
						        hor_free_func ) );
   hor_set_blm_process_type_label ( hor_add_process_type ( result_type_list,
					       hor_get_lm_result_type_label(),
					       hor_blm_execute, NULL,
					       hor_free_func ) );
   hor_free_assoc_label_list ( result_type_list );
}
