/*  Scene: software for sequential localisation and map-building

    kalman_form.cc
    Copyright (C) 2000 Joss Knight
    joss@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

// Source file for the module containing callbacks for the kalman parameter
// form.
// The form is just a way of interfacing to filter options in the kalman class

// Included files
#include <general_headers.h>
#include "forms.h"

#include "models_base.h"
#include "feature.h"
#include "scene_base.h"
#include "3d.h"
#include "sim_or_rob.h"
#include "simul.h"
#include "forminspect.h"
#include "scene_inspect.h"
#include "kalman.h"
#include "waypoints.h"

#include "formkalman.h"
#include "kalman_form.h"
#include "control_general.h"

// To keep these variables static, we need the pass_<x>_ptr functions
namespace {
  Kalman *kalman = NULL;
  Scene *scene = NULL;
  Three_D_Display *three_d = NULL;
  Sim_Or_Rob *sim_or_rob = NULL;
  Scene_Inspect *inspector = NULL;
  FD_formkalman *form = NULL;
}


// Prototypes
void set_filter_params (FL_OBJECT *obj, long data);
void switch_to_saved_state (FL_OBJECT *obj, long data);
void update_filter_params ();


// Constants
const int FIELD_MAX = 50;

void pass_all_ptrs (Kalman *k, Scene *s, Three_D_Display *td, Sim_Or_Rob *sr,
		    Scene_Inspect *si, FD_formkalman *fd)
{
  assert (k != NULL && s != NULL && td != NULL && sr != NULL &&
	  si != NULL && fd != NULL);

  kalman = k;
  scene = s;
  three_d = td;
  sim_or_rob = sr;
  inspector = si;
  form = fd;

  fl_freeze_form (form->formkalman);

  // Enforce filter method as per form default (set in form design)
  change_filter_type (form->choice_filter_type1, 0);
  change_filter_type (form->choice_filter_type2, 1);
  fl_activate_object (form->button_display_filter1_state);
  fl_activate_object (form->button_display_filter2_state);

  fl_set_button (form->button_display_filter1_state, 0);
  fl_set_button (form->button_display_filter2_state, 1);
  update_filter_params ();
  fl_set_button (form->button_display_filter1_state, 1);
  fl_set_button (form->button_display_filter2_state, 0);
  update_filter_params ();
  if (kalman->query_maintain_separate_state ()) {
    fl_activate_object (form->button_display_filter1_state);
    fl_activate_object (form->button_display_filter2_state);
  }
  else {
    fl_deactivate_object (form->button_display_filter1_state);
    fl_deactivate_object (form->button_display_filter2_state);
  }

  fl_unfreeze_form (form->formkalman);

  update_performance_info ();
}


void change_filter_type (FL_OBJECT *obj, long data)
{
  assert (kalman != NULL);
  assert (data == 0 || data == 1);

  int method = Kalman::filter_method (fl_get_choice (obj) - 1);
  if (data == 1) {
    if (strcmp (fl_get_choice_text (obj), "NONE") == 0) {
      if (kalman->query_maintain_separate_state ()) {
	fl_set_button (form->button_display_filter1_state, 1);
	switch_to_saved_state (form->button_display_filter1_state, 0);
	kalman->set_whether_to_maintain_separate_state (false, scene);
	fl_deactivate_object (form->button_display_filter1_state);
	fl_deactivate_object (form->button_display_filter2_state);
      }
    }
    else {
      if (!kalman->query_maintain_separate_state ()) {
	kalman->set_whether_to_maintain_separate_state (true, scene);
	fl_activate_object (form->button_display_filter1_state);
	fl_activate_object (form->button_display_filter2_state);
      }
      kalman->set_filter_method (1, Kalman::filter_method (method));
    }
  }
  else kalman->set_filter_method (0, Kalman::filter_method (method));
}

  
void switch_to_saved_state (FL_OBJECT *obj, long data)
{
  assert (kalman != NULL);
  assert (data == 0 || data == 1);

  if (data == 0 && !fl_get_button (form->button_display_filter1_state)) {
    fl_set_button (form->button_display_filter1_state, 1);
    return;
  }
  if (data == 1 && !fl_get_button (form->button_display_filter2_state)) {
    fl_set_button (form->button_display_filter2_state, 1);
    return;
  }

  /*
  if (data == 0) fl_set_button (form->button_display_filter2_state, 0);
  else fl_set_button (form->button_display_filter1_state, 0);
  */

  kalman->switch_to_saved_state (scene);
  update_filter_params ();
  
  // Redraw 3d display
  three_d->draw_axes();

  if (strcmp (sim_or_rob->simulation_or_robot_type, "SIMULATION") == 0)
  {
    display_true_vehicle ((Simulation *) sim_or_rob, three_d);
    display_true_features ((Simulation *) sim_or_rob, three_d);
  }

  display_estimated_features (scene, three_d);
  display_estimated_vehicle (scene, three_d);

  // Recalculate for scene inspector form
  inspector->update_form ();
}


void set_filter_params (FL_OBJECT *obj, long data)
{
  assert (obj != NULL);

  bool which_filter = fl_get_button (form->button_display_filter2_state);

  int jghk_age_limit;
  double nebot_c1, nebot_c2;

  switch (data) {
  case 0:
    jghk_age_limit = int (fl_get_counter_value (form->counter_jghk_age_limit));
    kalman->set_filter_params (which_filter, jghk_age_limit, -1.0, -1.0);
    break;

  case 1:
    sscanf (fl_get_input (obj), "%lf", &nebot_c1);
    if (nebot_c1 < 0.0) update_filter_params ();
    else kalman->set_filter_params (which_filter, -1, nebot_c1, -1.0);
    break;

  case 2:
    sscanf (fl_get_input (obj), "%lf", &nebot_c2);
    if (nebot_c2 < 0.0) update_filter_params ();
    else kalman->set_filter_params (which_filter, -1, -1.0, nebot_c2);
    break;

  default:
    bool SHOULDNT_REACH_HERE = false;
    assert (SHOULDNT_REACH_HERE);
  }
}


void update_filter_params ()
{
  bool which_filter = fl_get_button (form->button_display_filter2_state);

  Kalman::filter_params fp = kalman->get_filter_params (which_filter);

  fl_set_counter_value (form->counter_jghk_age_limit, double (fp.jghk_age_limit));

  char str [FIELD_MAX];
  sprintf (str, "%0.3g", fp.nebot_c1);
  fl_set_input (form->input_nebot_c1, str);
  sprintf (str, "%0.3g", fp.nebot_c2);
  fl_set_input (form->input_nebot_c2, str);
}


void update_performance_info ()
{
  Kalman::filter_params fp = kalman->get_filter_params (0);

  //  fl_freeze_form (form->formkalman);

  char str [FIELD_MAX];
  snprintf (str, FIELD_MAX, "%d", fp.features_retained);
  fl_set_object_label (form->text_features_retained1, const_cast<char *> (str));
  fl_set_object_color (form->text_features_retained1, FL_DARKCYAN, FL_MCOL);

  snprintf (str, FIELD_MAX, "%0.3g", fp.filter_time);
  fl_set_object_label (form->text_filter_time1, const_cast<char *> (str));
  fl_set_object_color (form->text_filter_time1, FL_DARKCYAN, FL_MCOL);

  snprintf (str, FIELD_MAX, "%0.3g", fp.total_filter_time);
  fl_set_object_label (form->text_total_time1, const_cast<char *> (str));
  fl_set_object_color (form->text_total_time1, FL_DARKCYAN, FL_MCOL);

  fp = kalman->get_filter_params (1);

  snprintf (str, FIELD_MAX, "%d", fp.features_retained);
  fl_set_object_label (form->text_features_retained2, const_cast<char *> (str));
  fl_set_object_color (form->text_features_retained2, FL_DARKCYAN, FL_MCOL);

  snprintf (str, FIELD_MAX, "%0.3g", fp.filter_time);
  fl_set_object_label (form->text_filter_time2, const_cast<char *> (str));
  fl_set_object_color (form->text_filter_time2, FL_DARKCYAN, FL_MCOL);

  snprintf (str, FIELD_MAX, "%0.3g", fp.total_filter_time);
  fl_set_object_label (form->text_total_time2, const_cast<char *> (str));
  fl_set_object_color (form->text_total_time2, FL_DARKCYAN, FL_MCOL);

  //  fl_unfreeze_form (form->formkalman);
}
