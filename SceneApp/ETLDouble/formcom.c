/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formcom.h"

FD_formcom *create_form_formcom(void)
{
  FL_OBJECT *obj;
  FD_formcom *fdui = (FD_formcom *) fl_calloc(1, sizeof(*fdui));

  fdui->formcom = fl_bgn_form(FL_NO_BOX, 760, 340);
  obj = fl_add_box(FL_UP_BOX,0,0,760,340,"");
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
  fdui->slider_set_S = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,90,200,20,"Steering increment (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_S,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 180, -180);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->slider_set_T = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,130,200,20,"Turret increment (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_T,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 180, -180);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_zero_v = obj = fl_add_button(FL_NORMAL_BUTTON,220,40,50,20,"Zero");
    fl_set_object_callback(obj,zero_v,0);
  fdui->button_zero_S = obj = fl_add_button(FL_NORMAL_BUTTON,220,90,50,20,"Zero");
    fl_set_object_callback(obj,zero_S,0);
  fdui->button_zero_T = obj = fl_add_button(FL_NORMAL_BUTTON,220,130,50,20,"Zero");
    fl_set_object_callback(obj,zero_T,0);
  obj = fl_add_text(FL_NORMAL_TEXT,40,10,170,20,"Robot 1 Control Parameters");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->slider_set_delta_t = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,180,200,20,"Time step (s)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_delta_t,0);
    fl_set_slider_precision(obj, 1);
    fl_set_slider_bounds(obj, 0.2, 5);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 0.1);
    fl_set_slider_increment(obj, 0.1, 1);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_one_second = obj = fl_add_button(FL_NORMAL_BUTTON,220,180,50,20,"One");
    fl_set_object_callback(obj,one_second,0);
  obj = fl_add_text(FL_NORMAL_TEXT,590,10,130,20,"Feature Manipulation");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_delete_feature = obj = fl_add_button(FL_NORMAL_BUTTON,550,70,200,30,"Delete Feature");
    fl_set_object_callback(obj,delete_feature,0);
  obj = fl_add_text(FL_NORMAL_TEXT,640,210,30,20,"Go");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_go_using_set_parameters = obj = fl_add_button(FL_NORMAL_BUTTON,550,240,200,30,"Go Using Robot Control Parameters");
    fl_set_object_callback(obj,go_using_set_parameters,0);
  obj = fl_add_text(FL_NORMAL_TEXT,630,110,50,20,"Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_print_robot_state = obj = fl_add_button(FL_NORMAL_BUTTON,550,140,100,30,"Print Robot State");
    fl_set_object_callback(obj,print_robot_state,0);
  fdui->button_quit = obj = fl_add_button(FL_NORMAL_BUTTON,550,300,200,30,"Quit");
    fl_set_object_callback(obj,quit,0);
  fdui->slider_set_steps = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,220,200,20,"Number of steps");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_steps,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 1, 100);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  obj = fl_add_button(FL_NORMAL_BUTTON,220,220,50,20,"One");
    fl_set_object_callback(obj,one_steps,0);
  fdui->button_auto_select_feature = obj = fl_add_button(FL_NORMAL_BUTTON,550,40,200,30,"Auto-Select Feature");
    fl_set_object_callback(obj,auto_select_feature,0);
  fdui->button_print_feature_state = obj = fl_add_button(FL_NORMAL_BUTTON,650,140,100,30,"Print Feature State");
    fl_set_object_callback(obj,print_feature_state,0);
  fdui->button_print_whole_state = obj = fl_add_button(FL_NORMAL_BUTTON,550,170,100,30,"Print Whole State");
    fl_set_object_callback(obj,print_whole_state,0);
  fdui->button_navigate_to_next_waypoint = obj = fl_add_button(FL_NORMAL_BUTTON,550,270,200,30,"Navigate to Next Waypoint");
    fl_set_object_callback(obj,navigate_to_next_waypoint,0);
  fdui->slider_set_v2 = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,280,40,200,20,"Velocity(m/s)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_v2,0);
    fl_set_slider_bounds(obj, -0.5, 0.5);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 0.01, 0.1);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->slider_set_S2 = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,280,90,200,20,"Steering increment (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_S2,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 180, -180);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->slider_set_T2 = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,280,130,200,20,"Turret increment (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_T2,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 180, -180);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.11);
    fl_set_slider_step(obj, 0.01);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_zero_v2 = obj = fl_add_button(FL_NORMAL_BUTTON,490,40,50,20,"Zero");
    fl_set_object_callback(obj,zero_v2,0);
  fdui->button_zero_S2 = obj = fl_add_button(FL_NORMAL_BUTTON,490,90,50,20,"Zero");
    fl_set_object_callback(obj,zero_S2,0);
  fdui->button_zero_T2 = obj = fl_add_button(FL_NORMAL_BUTTON,490,130,50,20,"Zero");
    fl_set_object_callback(obj,zero_T2,0);
  obj = fl_add_text(FL_NORMAL_TEXT,310,10,170,20,"Robot 2 Control Parameters");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_observe_second_robot = obj = fl_add_button(FL_NORMAL_BUTTON,280,210,260,30,"Make Observation of Second Robot");
    fl_set_object_callback(obj,observe_second_robot,0);
  obj = fl_add_text(FL_NORMAL_TEXT,340,180,110,20,"Robot Interaction");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_waypoint_check_correct = obj = fl_add_button(FL_NORMAL_BUTTON,280,240,260,30,"Go to Waypoint, Check, Correct");
    fl_set_object_callback(obj,waypoint_check_correct,0);
  fdui->button_output_state = obj = fl_add_button(FL_NORMAL_BUTTON,650,170,100,30,"Output State to File");
    fl_set_object_callback(obj,output_state,0);
  fdui->button_lock_steering_turret = obj = fl_add_lightbutton(FL_PUSH_BUTTON,220,110,50,20,"Lock");
    fl_set_object_callback(obj,lock_steering_turret,0);
    fl_set_button(obj, 1);
  fdui->button_lock_steering_turret2 = obj = fl_add_lightbutton(FL_PUSH_BUTTON,490,110,50,20,"Lock");
    fl_set_object_callback(obj,lock_steering_turret2,0);
    fl_set_button(obj, 1);
  fdui->button_zero_axes = obj = fl_add_button(FL_NORMAL_BUTTON,280,300,260,30,"Zero Axes at Robot 1 Position");
    fl_set_object_callback(obj,zero_axes,0);
  obj = fl_add_text(FL_NORMAL_TEXT,50,260,150,20,"Additional Control Forms");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_show_inspect = obj = fl_add_button(FL_NORMAL_BUTTON,10,290,120,30,"Scene Inspector");
    fl_set_object_callback(obj,show_inspect_form,0);
  fdui->button_show_filter = obj = fl_add_button(FL_NORMAL_BUTTON,140,290,120,30,"Filtering Control");
    fl_set_object_callback(obj,show_filter_form,0);
  fl_end_form();

  fdui->formcom->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

