/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formrobmode.h"

FD_formrobmode *create_form_formrobmode(void)
{
  FL_OBJECT *obj;
  FD_formrobmode *fdui = (FD_formrobmode *) fl_calloc(1, sizeof(*fdui));

  fdui->formrobmode = fl_bgn_form(FL_NO_BOX, 250, 250);
  obj = fl_add_box(FL_UP_BOX,0,0,250,250,"");
  obj = fl_add_text(FL_NORMAL_TEXT,80,10,90,30,"Robot Modes");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  obj = fl_add_text(FL_NORMAL_TEXT,20,80,220,30,"(Don't select if wires are attached to robot.)");
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->button_zero_axes = obj = fl_add_lightbutton(FL_PUSH_BUTTON,20,50,210,30,"Zero turret and steering axes.");
  fdui->button_reduced_image = obj = fl_add_lightbutton(FL_PUSH_BUTTON,20,120,210,30,"Reduced image display (radio ethernet).");
  fdui->button_done = obj = fl_add_button(FL_NORMAL_BUTTON,20,210,210,30,"Done");
  fl_end_form();

  fdui->formrobmode->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

