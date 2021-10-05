/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formrob.h"

FD_formrob *create_form_formrob(void)
{
  FL_OBJECT *obj;
  FD_formrob *fdui = (FD_formrob *) fl_calloc(1, sizeof(*fdui));

  fdui->formrob = fl_bgn_form(FL_NO_BOX, 470, 260);
  obj = fl_add_box(FL_UP_BOX,0,0,470,260,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
  fdui->canvas = obj = fl_add_canvas(FL_NORMAL_CANVAS,140,10,320,240,"");
  fdui->button_read_next_image = obj = fl_add_button(FL_NORMAL_BUTTON,20,50,90,30,"Read Next Image");
    fl_set_object_callback(obj,read_next_image,0);
  fdui->button_write_patch = obj = fl_add_button(FL_NORMAL_BUTTON,30,80,70,30,"Write Patch");
    fl_set_object_callback(obj,write_patch,0);
  obj = fl_add_text(FL_NORMAL_TEXT,10,20,120,20,"Image Manipulation");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fl_end_form();

  fdui->formrob->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

