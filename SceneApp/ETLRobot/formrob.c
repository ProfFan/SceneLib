/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formrob.h"

FD_formrob *create_form_formrob(void)
{
  FL_OBJECT *obj;
  FD_formrob *fdui = (FD_formrob *) fl_calloc(1, sizeof(*fdui));

  fdui->formrob = fl_bgn_form(FL_NO_BOX, 960, 280);
  obj = fl_add_box(FL_UP_BOX,0,0,960,280,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
  obj = fl_add_text(FL_NORMAL_TEXT,90,10,90,20,"Head Control");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_grab_images = obj = fl_add_button(FL_NORMAL_BUTTON,10,110,70,30,"Grab Images");
    fl_set_object_callback(obj,grab_images,0);
  fdui->slider_set_pan = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,30,230,20,"Pan (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_pan,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, 100, -100);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 0);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_zero_pan = obj = fl_add_button(FL_NORMAL_BUTTON,250,30,40,20,"Zero");
    fl_set_object_callback(obj,zero_pan,0);
  fdui->slider_set_elevation = obj = fl_add_valslider(FL_HOR_BROWSER_SLIDER,10,70,230,20,"Elevation (degrees)");
    fl_set_object_lsize(obj,FL_DEFAULT_SIZE);
    fl_set_object_callback(obj,set_elevation,0);
    fl_set_slider_precision(obj, 0);
    fl_set_slider_bounds(obj, -45, 45);
    fl_set_slider_value(obj, 0);
    fl_set_slider_size(obj, 0.10);
    fl_set_slider_step(obj, 1);
    fl_set_slider_increment(obj, 1, 0);
     fl_set_slider_return(obj, FL_RETURN_END);
  fdui->button_zero_elevation = obj = fl_add_button(FL_NORMAL_BUTTON,250,70,40,20,"Zero");
    fl_set_object_callback(obj,zero_elevation,0);
  obj = fl_add_text(FL_NORMAL_TEXT,80,180,120,20,"Feature Acquisition");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_init_man_selected_point_feature = obj = fl_add_button(FL_NORMAL_BUTTON,10,210,280,30,"Initialise Manually-Selected Point Feature");
    fl_set_object_callback(obj,init_man_selected_point_feature,0);
  fdui->button_init_auto_selected_point_feature = obj = fl_add_button(FL_NORMAL_BUTTON,10,240,280,30,"Initialise Automatically-Selected Point Feature");
    fl_set_object_callback(obj,init_auto_selected_point_feature,0);
  fdui->canvasL = obj = fl_add_canvas(FL_NORMAL_CANVAS,300,20,320,240,"");
  fdui->canvasR = obj = fl_add_canvas(FL_NORMAL_CANVAS,630,20,320,240,"");
  fdui->button_write_images = obj = fl_add_button(FL_NORMAL_BUTTON,150,110,70,30,"Write Images");
    fl_set_object_callback(obj,write_images,0);
  fdui->button_read_images = obj = fl_add_button(FL_NORMAL_BUTTON,80,110,70,30,"Read Images");
    fl_set_object_callback(obj,read_images,0);
  fdui->button_write_patch = obj = fl_add_button(FL_NORMAL_BUTTON,220,110,70,30,"Write Patch");
    fl_set_object_callback(obj,write_patch,0);
  fdui->button_show_head_odometry = obj = fl_add_button(FL_NORMAL_BUTTON,10,140,140,30,"Show Head Odometry");
    fl_set_object_callback(obj,show_head_odometry,0);
  fdui->button_show_robot_odometry = obj = fl_add_button(FL_NORMAL_BUTTON,150,140,140,30,"Show Robot Odometry");
    fl_set_object_callback(obj,show_robot_odometry,0);
  fl_end_form();

  fdui->formrob->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

