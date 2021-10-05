/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formanalyse.h"

FD_formanalyse *create_form_formanalyse(void)
{
  FL_OBJECT *obj;
  FD_formanalyse *fdui = (FD_formanalyse *) fl_calloc(1, sizeof(*fdui));

  fdui->formanalyse = fl_bgn_form(FL_NO_BOX, 340, 400);
  obj = fl_add_box(FL_UP_BOX,0,0,340,400,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
  fdui->button_quit = obj = fl_add_button(FL_NORMAL_BUTTON,10,360,320,30,"Quit");
    fl_set_object_callback(obj,quit,0);
  obj = fl_add_text(FL_NORMAL_TEXT,90,10,150,30,"Navigation Data Analysis");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->counter_set_step = obj = fl_add_counter(FL_NORMAL_COUNTER,110,50,120,20,"Step Number");
    fl_set_object_callback(obj,set_step,0);
    fl_set_counter_precision(obj, 0);
    fl_set_counter_bounds(obj, 0, 1000000);
    fl_set_counter_step(obj, 1, 10);
  fdui->button_input_viewing_direction = obj = fl_add_button(FL_NORMAL_BUTTON,10,250,160,30,"Input Viewing Direction");
    fl_set_object_callback(obj,input_viewing_direction,0);
  fdui->button_input_postscript_parameters = obj = fl_add_button(FL_NORMAL_BUTTON,10,280,160,30,"Input Postscript Parameters");
    fl_set_object_callback(obj,input_postscript_parameters,0);
  fdui->button_input_postscript_axis_limits = obj = fl_add_button(FL_NORMAL_BUTTON,10,310,160,30,"Input Postscript Axis Limits");
    fl_set_object_callback(obj,input_postscript_axis_limits,0);
  fdui->button_output_postscript = obj = fl_add_button(FL_NORMAL_BUTTON,170,250,160,30,"Output Postscript");
    fl_set_object_callback(obj,output_postscript,0);
  fdui->button_output_ground_truth_postscript = obj = fl_add_button(FL_NORMAL_BUTTON,170,280,160,30,"Output Ground Truth Postscript");
    fl_set_object_callback(obj,output_ground_truth_postscript,0);
  fdui->button_output_trajectory_picture = obj = fl_add_button(FL_NORMAL_BUTTON,170,310,160,30,"Output Trajectory Picture");
    fl_set_object_callback(obj,output_trajectory_picture,0);
  fdui->button_print_robot_state = obj = fl_add_button(FL_NORMAL_BUTTON,10,140,160,30,"Print Robot State");
    fl_set_object_callback(obj,print_robot_state,0);
  obj = fl_add_text(FL_NORMAL_TEXT,120,100,110,30,"Numerical Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  obj = fl_add_text(FL_NORMAL_TEXT,120,210,110,30,"Postscript Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_print_whole_state = obj = fl_add_button(FL_NORMAL_BUTTON,170,140,160,30,"Print Whole State");
    fl_set_object_callback(obj,print_whole_state,0);
  fdui->button_print_true_state = obj = fl_add_button(FL_NORMAL_BUTTON,170,170,160,30,"Print True State");
    fl_set_object_callback(obj,print_true_state,0);
  fdui->button_print_feature_state = obj = fl_add_button(FL_NORMAL_BUTTON,10,170,160,30,"Print Feature State");
    fl_set_object_callback(obj,print_feature_state,0);
  fl_end_form();

  fdui->formanalyse->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

