/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "forminspect.h"

FD_forminspect *create_form_forminspect(void)
{
  FL_OBJECT *obj;
  FD_forminspect *fdui = (FD_forminspect *) fl_calloc(1, sizeof(*fdui));

  fdui->forminspect = fl_bgn_form(FL_NO_BOX, 301, 391);
  obj = fl_add_box(FL_UP_BOX,0,0,301,391,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_BLUE);
  fdui->text_selected_covariance = obj = fl_add_text(FL_NORMAL_TEXT,140,320,140,40,"");
    fl_set_object_color(obj,FL_LEFT_BCOL,FL_MCOL);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  obj = fl_add_text(FL_NORMAL_TEXT,40,10,210,20," Covariance Matrix (Norms x 1000)");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->frame_covariance = obj = fl_add_frame(FL_ENGRAVED_FRAME,20,40,250,250,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
  fdui->counter_cell_size = obj = fl_add_counter(FL_SIMPLE_COUNTER,70,320,60,10,"Cell size");
    fl_set_object_color(obj,FL_COL1,FL_RED);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_callback(obj,change_cell_size,0);
    fl_set_counter_precision(obj, 0);
    fl_set_counter_bounds(obj, 10, 1000000);
    fl_set_counter_value(obj, 20);
    fl_set_counter_step(obj, 1, 1);
  fdui->scrollbar_covariance_horizontal = obj = fl_add_scrollbar(FL_HOR_THIN_SCROLLBAR,20,290,250,10,"");
    fl_set_object_boxtype(obj,FL_EMBOSSED_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
    fl_set_object_callback(obj,scroll_covariance,1);
    fl_set_scrollbar_value(obj, 0);
    fl_set_scrollbar_step(obj, 1);
    fl_set_scrollbar_increment(obj, 1, 1);
     fl_set_scrollbar_return(obj, FL_RETURN_ALWAYS);
  fdui->scrollbar_covariance_vertical = obj = fl_add_scrollbar(FL_VERT_THIN_SCROLLBAR,270,50,10,240,"");
    fl_set_object_boxtype(obj,FL_EMBOSSED_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_COL1);
    fl_set_object_callback(obj,scroll_covariance,0);
    fl_set_scrollbar_value(obj, 0);
    fl_set_scrollbar_step(obj, 1);
    fl_set_scrollbar_increment(obj, 1, 1);
     fl_set_scrollbar_return(obj, FL_RETURN_ALWAYS);
  fdui->button_center = obj = fl_add_button(FL_NORMAL_BUTTON,20,300,10,10,"@circle");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_callback(obj,center_on_selected,0);
  obj = fl_add_frame(FL_ENGRAVED_FRAME,140,320,140,40,"");
    fl_set_object_color(obj,FL_BLACK,FL_DARKCYAN);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lstyle(obj,FL_FIXED_STYLE);
  fdui->button_diagonal_scroll = obj = fl_add_button(FL_TOUCH_BUTTON,270,290,10,10,"@3>>");
    fl_set_object_boxtype(obj,FL_EMBOSSED_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_DARKCYAN);
    fl_set_object_lcolor(obj,FL_COL1);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_callback(obj,scroll_diagonally,1);
  obj = fl_add_text(FL_NORMAL_TEXT,165,300,90,20,"Selected covariance");
    fl_set_object_boxtype(obj,FL_NO_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT|FL_ALIGN_INSIDE);
  fdui->counter_precision = obj = fl_add_counter(FL_SIMPLE_COUNTER,70,330,60,10,"Precision");
    fl_set_object_color(obj,FL_COL1,FL_RED);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_callback(obj,change_precision,0);
    fl_set_counter_precision(obj, 0);
    fl_set_counter_bounds(obj, 1, 1000000);
    fl_set_counter_value(obj, 3);
    fl_set_counter_step(obj, 1, 1);
  obj = fl_add_button(FL_TOUCH_BUTTON,270,40,10,10,"@7>>");
    fl_set_object_boxtype(obj,FL_EMBOSSED_BOX);
    fl_set_object_color(obj,FL_DARKCYAN,FL_DARKCYAN);
    fl_set_object_lcolor(obj,FL_COL1);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_callback(obj,scroll_diagonally,0);
  fdui->button_redraw_form = obj = fl_add_button(FL_NORMAL_BUTTON,10,10,20,20,"R");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lstyle(obj,FL_TIMESBOLDITALIC_STYLE);
    fl_set_object_callback(obj,redraw_form,0);
  fdui->button_close = obj = fl_add_button(FL_NORMAL_BUTTON,270,10,20,20,"@square");
    fl_set_object_callback(obj,hide_form,0);
  fdui->button_norm_volume = obj = fl_add_roundbutton(FL_PUSH_BUTTON,70,350,20,20,"Vol");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_BOTTOM);
    fl_set_object_callback(obj,switch_norm_type,1);
  fdui->button_norm_frobenius = obj = fl_add_roundbutton(FL_PUSH_BUTTON,90,350,20,20,"Frob");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_BOTTOM);
    fl_set_object_callback(obj,switch_norm_type,2);
  fdui->button_norm_trace = obj = fl_add_roundbutton(FL_PUSH_BUTTON,110,350,20,20,"Trace");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_BOTTOM);
    fl_set_object_callback(obj,switch_norm_type,3);
  obj = fl_add_text(FL_NORMAL_TEXT,10,355,50,10,"Norm type   ");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->button_norm_hinf = obj = fl_add_roundbutton(FL_PUSH_BUTTON,50,350,20,20,"Hinf");
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_BOTTOM);
    fl_set_object_callback(obj,switch_norm_type,0);
    fl_set_button(obj, 1);
  fl_end_form();

  fdui->forminspect->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

