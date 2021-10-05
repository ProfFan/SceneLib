/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

#include "ops_prms.h"

/*******************
*   void @select_improc_in_region ( int c1, int r1, int c2, int r2, void *data)
*
*   This is called when a region has been selected by the user. The process
*   type(s) whose HORATIO toggle(s) is on causes a new process to be created
*   of the relevant type in the selected window. This function creates the
*   new process(es).
********************/
void select_improc_in_region ( int c1, int r1, int c2, int r2, void *data )
{
   int process_code = get_ops_toggle_state();

   switch ( process_code )
   {
      case 0:
      hor_warning ( "no operation specified" );
      hor_region_delete_last_selected();
      break;

      /* special codes here */

      case IMAGE_FLOW_CODE:
      {
	 Hor_FL_Process_Params fl_process_params;
	 Hor_FL_Output_Params  fl_output_params;

	 if ( !hor_get_image_flow_params ( &fl_process_params,
					   &fl_output_params ) )
	 { hor_region_delete_last_selected(); return; }

	 hor_fl_add_process ( fl_process_params, fl_output_params,
			      c1, r1, c2, r2 );
	 hor_message ( "image flow process selected" );
      }
      break;

      case IMAGE_SEGMENT_CODE:
      {
	 Hor_IS_Process_Params is_process_params;

	 if ( !hor_get_image_segment_params ( &is_process_params ) )
	 { hor_region_delete_last_selected(); return; }

	 hor_is_add_process ( is_process_params, c1, r1, c2, r2 );
	 hor_message ( "image segment process selected" );
      }
      break;

      case CORRELATION_CODE:
      {
	 Hor_CL_Process_Params cl_process_params;

	 if ( !hor_get_correlation_params ( &cl_process_params ) )
	 { hor_region_delete_last_selected(); return; }

	 hor_cl_add_process ( cl_process_params, c1, r1, c2, r2 );
	 hor_message ( "correlation process selected" );
      }
      break;

      /* individual codes here */

      default:
      {
	 Hor_Assoc_Label corner_proc, edge_proc, line_proc;

	 corner_proc = edge_proc = line_proc = HOR_ASSOC_ERROR;

	 if ( process_code & WANG_CORNER_CODE )
	 {
	    Hor_COW_Process_Params bd_process_params;
	    Hor_CO_Output_Params   bd_output_params;

	    if ( !hor_get_wang_corner_params ( &bd_process_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_get_wang_corner_colours ( &bd_output_params );
	    corner_proc = hor_cow_add_process(bd_process_params,
					      bd_output_params,
					      hor_get_upper_left_c_velocity(),
					      hor_get_upper_left_r_velocity(),
					      hor_get_lower_right_c_velocity(),
					      hor_get_lower_right_r_velocity(),
					      c1, r1, c2, r2);
	    hor_message ( "Wang/Brady corner matcher process selected");
	 }

	 if ( process_code & SMITH_CORNER_CODE )
	 {
	    Hor_SC_Process_Params sc_process_params;
	    Hor_CO_Output_Params  sc_output_params;

	    if ( !hor_get_smith_corner_params ( &sc_process_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_get_smith_corner_colours ( &sc_output_params );
	    corner_proc = hor_sc_add_process (sc_process_params,
					      sc_output_params,
					      hor_get_upper_left_c_velocity(),
					      hor_get_upper_left_r_velocity(),
					      hor_get_lower_right_c_velocity(),
					      hor_get_lower_right_r_velocity(),
					      c1, r1, c2, r2);
	    hor_message ( "Smith corner matcher process selected");
	 }

	 if ( process_code & PLESSEY_CORNER_CODE )
	 {
	    Hor_PC_Process_Params pc_process_params;
	    Hor_CO_Output_Params  pc_output_params;

	    if ( !hor_get_plessey_corner_params ( &pc_process_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_get_plessey_corner_colours ( &pc_output_params );
	    corner_proc = hor_pc_add_process (pc_process_params,
					      pc_output_params,
					      hor_get_upper_left_c_velocity(),
					      hor_get_upper_left_r_velocity(),
					      hor_get_lower_right_c_velocity(),
					      hor_get_lower_right_r_velocity(),
					      c1, r1, c2, r2);
	    hor_message ( "Plessey corner matcher process selected");
	 }

	 if ( process_code & BOG_CORNER_MATCH_CODE )
	 {
	    Hor_BCM_Process_Params cm_process_params;
	    Hor_CM_Output_Params   cm_output_params;

	    if ( corner_proc == HOR_ASSOC_ERROR ) {
	       hor_error ("corner matcher with no corners (select_improc_in_region)", HOR_NON_FATAL);
	       return;
	    }

	    if ( !hor_get_bog_corner_match_params ( &cm_process_params,
						    &cm_output_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_bcm_add_process ( cm_process_params, cm_output_params,
				  corner_proc );
	    hor_message ( "Bog-standard corner matcher process selected");
	 }

	 if ( process_code & BD_CORNER_MATCH_CODE )
	 {
	    Hor_DCM_Process_Params cm_process_params;
	    Hor_CM_Output_Params   cm_output_params;

	    if ( corner_proc == HOR_ASSOC_ERROR ) {
	       hor_error ("corner matcher with no corners (select_improc_in_region)", HOR_NON_FATAL);
	       return;
	    }

	    if ( !hor_get_bd_corner_match_params ( &cm_process_params,
						   &cm_output_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_dcm_add_process ( cm_process_params, cm_output_params,
				  corner_proc );
	    hor_message ( "Brain-dead corner matcher process selected");
	 }

	 if ( process_code & CANNY_CODE )
	 {
	    Hor_CA_Process_Params ca_process_params;
	    Hor_ED_Output_Params  ca_output_params;

	    if ( !hor_get_canny_params ( &ca_process_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_get_canny_colours ( &ca_output_params );
	    edge_proc = hor_ca_add_process ( ca_process_params,
					     ca_output_params,
					     hor_get_upper_left_c_velocity(),
					     hor_get_upper_left_r_velocity(),
					     hor_get_lower_right_c_velocity(),
					     hor_get_lower_right_r_velocity(),
					     c1, r1, c2, r2 );
	    hor_message ( "Canny process selected" );
	 }

	 if ( process_code & LINE_FIT_CODE )
	 {
	    Hor_LF_Process_Params lf_process_params;
	    Hor_LI_Output_Params lf_output_params;

	    if ( edge_proc == HOR_ASSOC_ERROR ) {
	       hor_error ( "line fitter with no edges (select_improc_in_region)", HOR_NON_FATAL );
	       return;
	    }

	    if ( !hor_get_line_fit_params ( &lf_process_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_get_line_fit_colours ( &lf_output_params );
	    line_proc = hor_lf_add_process ( lf_process_params,
					     lf_output_params, edge_proc );
	    hor_message("orthogonal regression line fitting process selected");
	 }

	 if ( process_code & LINE_SEGMENT_CODE )
	 {
	    Hor_LI_Output_Params ls_output_params;

	    if ( edge_proc == HOR_ASSOC_ERROR ) {
	       hor_error ( "line fitter with no edges (select_improc_in_region)", HOR_NON_FATAL );
	       return;
	    }

	    hor_get_line_segment_params ( &ls_output_params );
	    line_proc = hor_ls_add_process ( ls_output_params, edge_proc );
	    hor_message ( "Will Dickson line segment process selected" );
	 }

	 if ( process_code & LINE_MATCH_CODE )
	 {
	    Hor_BLM_Process_Params lm_process_params;
	    Hor_LM_Output_Params   lm_output_params;

	    if ( line_proc == HOR_ASSOC_ERROR ) {
	       hor_error ( "line matcher with no lines (select_improc_in_region)", HOR_NON_FATAL );
	       return;
	    }

	    if ( !hor_get_bog_line_match_params ( &lm_process_params,
						  &lm_output_params ) )
	    { hor_region_delete_last_selected(); return; }

	    hor_blm_add_process ( lm_process_params, lm_output_params,
				  line_proc );
	    hor_message ("Bog-standard line segment matcher process selected");
	 }
      }

      break;
   }
}
