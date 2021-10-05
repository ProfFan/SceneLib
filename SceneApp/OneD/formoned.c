/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formoned.h"

FD_formoned *create_form_formoned(void)
{
  FL_OBJECT *obj;
  FD_formoned *fdui = (FD_formoned *) fl_calloc(1, sizeof(*fdui));

  fdui->formoned = fl_bgn_form(FL_NO_BOX, 540, 320);
  obj = fl_add_box(FL_UP_BOX,0,0,540,320,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_BLUE);
  fdui->slider_set_v = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,40,200,20,"Velocity(m/s)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_v,0);
    fl_set_slider_bounds(obj, -0.5, 0.5);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 0.01, 0.1);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_zero_v = obj = fl_add_button(FL_NORMAL_BUTTON,220,40,50,20,"Zero");
    fl_set_object_callback(obj,zero_v,0);
  obj = fl_add_text(FL_NORMAL_TEXT,50,10,160,20,"Robot Control Parameters");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->slider_set_steps = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,130,200,20,"Number of steps");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_steps,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 1, 100);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_one_steps = obj = fl_add_button(FL_NORMAL_BUTTON,220,130,50,20,"One");
    fl_set_object_callback(obj,one_steps,0);
  obj = fl_add_text(FL_NORMAL_TEXT,340,10,130,20,"Feature Manipulation");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_delete_feature = obj = fl_add_button(FL_NORMAL_BUTTON,300,40,210,30,"Delete Feature");
    fl_set_object_callback(obj,delete_feature,0);
  obj = fl_add_text(FL_NORMAL_TEXT,390,220,30,20,"Go");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_go_vehicle = obj = fl_add_button(FL_NORMAL_BUTTON,300,250,210,30,"Go");
    fl_set_object_callback(obj,go_vehicle,0);
  obj = fl_add_text(FL_NORMAL_TEXT,380,90,50,20,"Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_print_state = obj = fl_add_button(FL_NORMAL_BUTTON,300,120,210,30,"Print State");
    fl_set_object_callback(obj,print_state,0);
  fdui->button_quit = obj = fl_add_button(FL_NORMAL_BUTTON,20,280,250,30,"Quit");
    fl_set_object_callback(obj,quit,0);
  fdui->button_output_state = obj = fl_add_button(FL_NORMAL_BUTTON,300,150,210,30,"Output State to File");
    fl_set_object_callback(obj,output_state,0);
  fdui->button_print_true_state = obj = fl_add_button(FL_NORMAL_BUTTON,300,180,210,30,"Print True State");
    fl_set_object_callback(obj,print_true_state,0);
  fdui->button_one_second = obj = fl_add_button(FL_NORMAL_BUTTON,220,90,50,20,"One");
    fl_set_object_callback(obj,one_second,0);
  fdui->slider_set_delta_t = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,90,200,20,"Time step(s)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_delta_t,0);
    fl_set_slider_precision(obj, 1);
    fl_set_slider_bounds(obj, 0.2, 5);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 0.1);
    fl_set_slider_increment(obj, 0.1, 1);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_navigate_to_next_waypoint = obj = fl_add_button(FL_NORMAL_BUTTON,300,280,210,30,"Navigate to Next Waypoint");
    fl_set_object_callback(obj,navigate_to_next_waypoint,0);
  obj = fl_add_text(FL_NORMAL_TEXT,60,170,150,20,"Additional Control Forms");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_show_inspect = obj = fl_add_button(FL_NORMAL_BUTTON,20,200,120,30,"Scene Inspector");
    fl_set_object_callback(obj,show_inspect_form,0);
  fdui->button_show_filter = obj = fl_add_button(FL_NORMAL_BUTTON,150,200,120,30,"Filtering Control");
    fl_set_object_callback(obj,show_filter_form,0);
  fdui->button_zero_axes = obj = fl_add_button(FL_NORMAL_BUTTON,20,240,250,30,"Zero Axes");
    fl_set_object_callback(obj,zero_axes,0);
  fl_end_form();

  fdui->formoned->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

