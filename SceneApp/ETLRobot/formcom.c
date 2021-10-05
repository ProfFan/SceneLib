/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formcom.h"

FD_formcom *create_form_formcom(void)
{
  FL_OBJECT *obj;
  FD_formcom *fdui = (FD_formcom *) fl_calloc(1, sizeof(*fdui));

  fdui->formcom = fl_bgn_form(FL_NO_BOX, 500, 390);
  obj = fl_add_box(FL_UP_BOX,0,0,500,390,"");
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
  obj = fl_add_text(FL_NORMAL_TEXT,50,10,160,20,"Robot Control Parameters");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->slider_set_delta_t = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,190,200,20,"Time step (s)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_delta_t,0);
    fl_set_slider_precision(obj, 1);
    fl_set_slider_bounds(obj, 0.2, 5);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 0.1);
    fl_set_slider_increment(obj, 0.1, 1);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_one_second = obj = fl_add_button(FL_NORMAL_BUTTON,220,190,50,20,"One");
    fl_set_object_callback(obj,one_second,0);
  obj = fl_add_text(FL_NORMAL_TEXT,330,10,130,20,"Feature Manipulation");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_delete_feature = obj = fl_add_button(FL_NORMAL_BUTTON,290,70,200,30,"Delete Feature");
    fl_set_object_callback(obj,delete_feature,0);
  obj = fl_add_text(FL_NORMAL_TEXT,380,230,30,20,"Go");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_go_using_set_parameters = obj = fl_add_button(FL_NORMAL_BUTTON,290,260,200,30,"Go Using Robot Control Parameters");
    fl_set_object_callback(obj,go_using_set_parameters,0);
  obj = fl_add_text(FL_NORMAL_TEXT,370,110,50,20,"Output");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_print_robot_state = obj = fl_add_button(FL_NORMAL_BUTTON,290,140,100,30,"Print Robot State");
    fl_set_object_callback(obj,print_robot_state,0);
  fdui->button_quit = obj = fl_add_button(FL_NORMAL_BUTTON,10,350,250,30,"Quit");
    fl_set_object_callback(obj,quit,0);
  fdui->button_output_state = obj = fl_add_button(FL_NORMAL_BUTTON,390,170,100,30,"Output State to File");
    fl_set_object_callback(obj,output_state,0);
  fdui->slider_set_steps = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,230,200,20,"Number of steps");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_steps,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 1, 100);
    fl_set_slider_value(obj, 1);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 10);
     fl_set_slider_return(obj, FL_RETURN_END);
  obj = fl_add_button(FL_NORMAL_BUTTON,220,230,50,20,"One");
    fl_set_object_callback(obj,one_steps,0);
  fdui->button_auto_select_feature = obj = fl_add_button(FL_NORMAL_BUTTON,290,40,200,30,"Auto-Select Feature");
    fl_set_object_callback(obj,auto_select_feature,0);
  fdui->button_print_feature_state = obj = fl_add_button(FL_NORMAL_BUTTON,390,140,100,30,"Print Feature State");
    fl_set_object_callback(obj,print_feature_state,0);
  fdui->button_print_whole_state = obj = fl_add_button(FL_NORMAL_BUTTON,290,170,100,30,"Print Whole State");
    fl_set_object_callback(obj,print_whole_state,0);
  fdui->button_navigate_to_next_waypoint = obj = fl_add_button(FL_NORMAL_BUTTON,290,320,200,30,"Navigate to Next Waypoint");
    fl_set_object_callback(obj,navigate_to_next_waypoint,0);
  fdui->button_lock_steering_turret = obj = fl_add_lightbutton(FL_PUSH_BUTTON,220,110,50,20,"Lock");
    fl_set_object_callback(obj,lock_steering_turret,0);
    fl_set_button(obj, 1);
  obj = fl_add_text(FL_NORMAL_TEXT,60,270,150,20,"Additional Control Forms");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_show_filter = obj = fl_add_button(FL_NORMAL_BUTTON,140,300,120,30,"Filtering Control");
    fl_set_object_callback(obj,show_filter_form,0);
  fdui->button_show_inspect = obj = fl_add_button(FL_NORMAL_BUTTON,10,300,120,30,"Scene Inspector");
    fl_set_object_callback(obj,show_inspect_form,0);
  fdui->button_zero_axes = obj = fl_add_button(FL_NORMAL_BUTTON,400,200,90,30,"Zero Axes");
    fl_set_object_callback(obj,zero_axes,0);
  fdui->button_measure_roll_pitch = obj = fl_add_button(FL_NORMAL_BUTTON,290,290,100,30,"Measure Roll/Pitch");
    fl_set_object_callback(obj,measure_roll_pitch,0);
  fdui->button_go_vehicle_roll_pitch = obj = fl_add_button(FL_NORMAL_BUTTON,390,290,100,30,"Go Using Roll/Pitch");
    fl_set_object_callback(obj,go_vehicle_roll_pitch,0);
  fdui->button_output_postscript = obj = fl_add_button(FL_NORMAL_BUTTON,290,200,100,30,"Output Postscript");
    fl_set_object_callback(obj,output_postscript,0);
  fdui->button_navigate_to_next_waypoint_using_rollpitch = obj = fl_add_button(FL_NORMAL_BUTTON,290,350,200,30,"Nav. to Next Waypoint using Roll/Pitch");
    fl_set_object_callback(obj,navigate_to_next_waypoint_using_rollpitch,0);
  fl_end_form();

  fdui->formcom->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

