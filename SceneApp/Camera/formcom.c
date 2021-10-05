/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formcom.h"

FD_formcom *create_form_formcom(void)
{
  FL_OBJECT *obj;
  FD_formcom *fdui = (FD_formcom *) fl_calloc(1, sizeof(*fdui));

  fdui->formcom = fl_bgn_form(FL_NO_BOX, 500, 220);
  obj = fl_add_box(FL_UP_BOX,0,0,500,220,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_BLUE);
  obj = fl_add_text(FL_NORMAL_TEXT,330,10,130,20,"Feature Manipulation");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_delete_feature = obj = fl_add_button(FL_NORMAL_BUTTON,290,60,200,30,"Delete Feature");
    fl_set_object_callback(obj,delete_feature,0);
  obj = fl_add_text(FL_NORMAL_TEXT,120,120,30,20,"Go");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_go_step_by_step = obj = fl_add_button(FL_NORMAL_BUTTON,10,140,130,30,"Go Step by Step");
    fl_set_object_callback(obj,go_step_by_step,0);
  obj = fl_add_text(FL_NORMAL_TEXT,370,100,50,20,"Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_print_robot_state = obj = fl_add_button(FL_NORMAL_BUTTON,290,120,100,30,"Print Robot State");
    fl_set_object_callback(obj,print_robot_state,0);
  fdui->button_quit = obj = fl_add_button(FL_NORMAL_BUTTON,10,180,260,30,"Quit");
    fl_set_object_callback(obj,quit,0);
  fdui->button_output_state = obj = fl_add_button(FL_NORMAL_BUTTON,390,150,100,30,"Output State to File");
    fl_set_object_callback(obj,output_state,0);
  fdui->slider_set_steps = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,20,210,20,"Number of steps");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_steps,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 1, 100);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  obj = fl_add_button(FL_NORMAL_BUTTON,220,20,50,20,"One");
    fl_set_object_callback(obj,one_steps,0);
  fdui->button_auto_select_feature = obj = fl_add_button(FL_NORMAL_BUTTON,290,30,200,30,"Auto-Select Feature");
    fl_set_object_callback(obj,auto_select_feature,0);
  fdui->button_print_feature_state = obj = fl_add_button(FL_NORMAL_BUTTON,390,120,100,30,"Print Feature State");
    fl_set_object_callback(obj,print_feature_state,0);
  fdui->button_print_whole_state = obj = fl_add_button(FL_NORMAL_BUTTON,290,150,100,30,"Print Whole State");
    fl_set_object_callback(obj,print_whole_state,0);
  obj = fl_add_text(FL_NORMAL_TEXT,60,60,150,20,"Additional Control Forms");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_show_filter = obj = fl_add_button(FL_NORMAL_BUTTON,140,80,130,30,"Filtering Control");
    fl_set_object_callback(obj,show_filter_form,0);
  fdui->button_show_inspect = obj = fl_add_button(FL_NORMAL_BUTTON,10,80,130,30,"Scene Inspector");
    fl_set_object_callback(obj,show_inspect_form,0);
  fdui->button_zero_axes = obj = fl_add_button(FL_NORMAL_BUTTON,400,180,90,30,"Zero Axes");
    fl_set_object_callback(obj,zero_axes,0);
  fdui->button_output_postscript = obj = fl_add_button(FL_NORMAL_BUTTON,290,180,100,30,"Output Postscript");
    fl_set_object_callback(obj,output_postscript,0);
  fdui->button_go_auto_select = obj = fl_add_button(FL_NORMAL_BUTTON,140,140,130,30,"Go With Auto-Select");
    fl_set_object_callback(obj,go_auto_select,0);
  fl_end_form();

  fdui->formcom->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

