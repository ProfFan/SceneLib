/* Form definition file generated with fdesign. */

#include "forms.h"
#include <stdlib.h>
#include "formkalman.h"

static FL_PUP_ENTRY fdchoice_choice_filter_type1_0[] =
{ 
    /*  itemtext   callback  shortcut   mode */
    { "SLOW",	0,	"",	 FL_PUP_NONE},
    { "FAST",	0,	"",	 FL_PUP_NONE},
    { "JGHK",	0,	"",	 FL_PUP_NONE},
    { "NEBOT",	0,	"",	 FL_PUP_NONE},
    {0}
};

static FL_PUP_ENTRY fdchoice_choice_filter_type2_1[] =
{ 
    /*  itemtext   callback  shortcut   mode */
    { "SLOW",	0,	"",	 FL_PUP_NONE},
    { "FAST",	0,	"",	 FL_PUP_NONE},
    { "JGHK",	0,	"",	 FL_PUP_NONE},
    { "NEBOT",	0,	"",	 FL_PUP_NONE},
    { "NONE",	0,	"",	 FL_PUP_NONE},
    {0}
};

FD_formkalman *create_form_formkalman(void)
{
  FL_OBJECT *obj;
  FD_formkalman *fdui = (FD_formkalman *) fl_calloc(1, sizeof(*fdui));

  fdui->formkalman = fl_bgn_form(FL_NO_BOX, 270, 160);
  obj = fl_add_box(FL_UP_BOX,0,0,270,160,"");
    fl_set_object_color(obj,FL_DARKCYAN,FL_BLUE);
  fdui->choice_filter_type1 = obj = fl_add_choice(FL_NORMAL_CHOICE2,60,15,70,20,"Filter 1");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
    fl_set_object_callback(obj,change_filter_type,0);
    fl_set_choice_entries(obj, fdchoice_choice_filter_type1_0);
    fl_set_choice(obj,1);
  fdui->choice_filter_type2 = obj = fl_add_choice(FL_NORMAL_CHOICE2,60,45,70,20,"Filter 2");
    fl_set_object_lsize(obj,FL_NORMAL_SIZE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
    fl_set_object_callback(obj,change_filter_type,1);
    fl_set_choice_entries(obj, fdchoice_choice_filter_type2_1);
    fl_set_choice(obj,5);
  obj = fl_add_text(FL_NORMAL_TEXT,10,70,150,20,"Params (selected filter)");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_NORMAL_STYLE+FL_EMBOSSED_STYLE);
  fdui->counter_jghk_age_limit = obj = fl_add_counter(FL_SIMPLE_COUNTER,90,90,60,20,"JGHK age limit");
    fl_set_object_color(obj,FL_COL1,FL_RED);
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_callback(obj,set_filter_params,0);
    fl_set_counter_precision(obj, 0);
    fl_set_counter_bounds(obj, 1, 1000000);
    fl_set_counter_value(obj, 10);
    fl_set_counter_step(obj, 1, 1);
  fdui->button_display_filter2_state = obj = fl_add_round3dbutton(FL_RADIO_BUTTON,140,40,30,30,"");
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_callback(obj,switch_to_saved_state,1);
  fdui->button_display_filter1_state = obj = fl_add_round3dbutton(FL_RADIO_BUTTON,140,10,30,30,"");
    fl_set_object_lalign(obj,FL_ALIGN_LEFT);
    fl_set_object_callback(obj,switch_to_saved_state,0);
    fl_set_button(obj, 1);
  obj = fl_add_text(FL_NORMAL_TEXT,200,30,30,20,"Time");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_BOLD_STYLE);
  obj = fl_add_text(FL_NORMAL_TEXT,230,30,30,20,"Total");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_BOLD_STYLE);
  obj = fl_add_text(FL_NORMAL_TEXT,160,30,40,20,"Features");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
    fl_set_object_lstyle(obj,FL_BOLD_STYLE);
  fdui->text_features_retained1 = obj = fl_add_text(FL_NORMAL_TEXT,165,15,30,20,"10");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->text_filter_time1 = obj = fl_add_text(FL_NORMAL_TEXT,200,15,30,20,"0.01");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->text_total_time1 = obj = fl_add_text(FL_NORMAL_TEXT,230,15,30,20,"0.001");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->text_features_retained2 = obj = fl_add_text(FL_NORMAL_TEXT,165,45,30,20,"10");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->text_filter_time2 = obj = fl_add_text(FL_NORMAL_TEXT,200,45,30,20,"0.01");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->text_total_time2 = obj = fl_add_text(FL_NORMAL_TEXT,230,45,30,20,"0.001");
    fl_set_object_color(obj,FL_DARKCYAN,FL_MCOL);
    fl_set_object_lsize(obj,FL_TINY_SIZE);
    fl_set_object_lalign(obj,FL_ALIGN_CENTER|FL_ALIGN_INSIDE);
  fdui->input_nebot_c1 = obj = fl_add_input(FL_FLOAT_INPUT,90,110,60,20,"Nebot const 1");
    fl_set_object_callback(obj,set_filter_params,1);
  fdui->input_nebot_c2 = obj = fl_add_input(FL_FLOAT_INPUT,90,130,60,20,"Nebot const 2");
    fl_set_object_callback(obj,set_filter_params,2);
  fl_end_form();

  fdui->formkalman->fdui = fdui;

  return fdui;
}
/*---------------------------------------*/

