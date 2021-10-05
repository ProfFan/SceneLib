/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formsim.h"

FD_formsim *create_form_formsim(void)
{
  FL_OBJECT *obj;
  FD_formsim *fdui = (FD_formsim *) fl_calloc(1, sizeof(*fdui));

  fdui->formsim = fl_bgn_form(FL_NO_BOX, 270, 80);
  obj = fl_add_box(FL_UP_BOX,0,0,270,80,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
  fdui->button_print_true_state = obj = fl_add_button(FL_NORMAL_BUTTON,30,40,210,30,"Print True State");
    fl_set_object_callback(obj,print_true_state,0);
  obj = fl_add_text(FL_NORMAL_TEXT,50,10,180,20,"Simulation-Specific Controls");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fl_end_form();

  fdui->formsim->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

