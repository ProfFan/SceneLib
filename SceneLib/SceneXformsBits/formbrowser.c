/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formbrowser.h"

FD_formbrowser *create_form_formbrowser(void)
{
  FL_OBJECT *obj;
  FD_formbrowser *fdui = (FD_formbrowser *) fl_calloc(1, sizeof(*fdui));

  fdui->formbrowser = fl_bgn_form(FL_NO_BOX, 210, 270);
  obj = fl_add_box(FL_UP_BOX,0,0,210,270,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_BLUE);
  fdui->waypoint_browser = obj = fl_add_browser(FL_HOLD_BROWSER,10,40,190,140,"");
    fl_set_object_boxtype(obj,FL_UP_BOX);
    fl_set_object_lstyle(obj,FL_FIXED_STYLE);
    fl_set_object_callback(obj,waypoint_selected,0);
  obj = fl_add_text(FL_NORMAL_TEXT,70,10,70,20,"Waypoints");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->button_delete_waypoint = obj = fl_add_button(FL_NORMAL_BUTTON,150,190,50,30,"Delete");
    fl_set_object_callback(obj,delete_waypoint,0);
  fdui->button_add_waypoint_before = obj = fl_add_button(FL_NORMAL_BUTTON,10,190,70,30,"Add Before");
    fl_set_object_callback(obj,add_waypoint_before,0);
  fdui->input_waypoint = obj = fl_add_input(FL_NORMAL_INPUT,50,230,150,30,"To add:");
  fdui->button_add_waypoint_after = obj = fl_add_button(FL_NORMAL_BUTTON,80,190,70,30,"Add After");
    fl_set_object_callback(obj,add_waypoint_after,0);
  fl_end_form();

  fdui->formbrowser->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

