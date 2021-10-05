/*  Scene: software for sequential localisation and map-building

    scene_inspect.cc
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

/******************************************************************************
 * scene_inspect.cc
 *
 * Callback functions for the inspect form (forminspect) and associated
 * classes Scene_Inspect and Matrix_Browser.  Matrix_Browser may be useful
 * outside the context of this particular form but probably not, so it's
 * defined in this compilation unit.
 *
 * Original code by Joss Knight
 ******************************************************************************/

/******************** Included Files ******************************************/

#include <general_headers.h>
#include "models_base.h"
#include "feature.h"
#include "scene_base.h"

extern "C" {
#include "forms.h"
}

#include <cassert>
#include <cmath>
#include <string>
#include <cstring>
#include <list>
#include <algorithm>

#include "forminspect.h"
#include "scene_inspect.h"


/******************** Global variables ****************************************/


extern Scene_Inspect *inspector;  // Must be declared elsewhere, the callbacks use it


/******************** Utility functions for this file *************************/

namespace Scene_Inspect_utilities {
  template <class T> inline int round (const T& val)
  {
    return int (val + (val < 0 ? -1 : 1) * 0.5);
  }

  template <class T> inline bool in (list<T> l, const T& compare_to)
  {
    return find (l.begin (), l.end (), compare_to) != l.end ();
  }

  template <class T> inline T bound_inclusive (T val, T min, T max)
  {
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
  }
};


/******************** forminspect callbacks ***********************************/

void scroll_covariance (FL_OBJECT *obj, long horiz_vert)
{
  assert (obj != 0);
  assert (horiz_vert == 0 || horiz_vert == 1);

  inspector->scroll_covariance_browser ();
}


void change_cell_size (FL_OBJECT *obj, long data)
{
  assert (obj != 0);
  data = data;

  inspector->change_browser_cellsize ((int) fl_get_counter_value (obj));
}


void select_cell (FL_OBJECT *obj, long data)
{
  assert (obj != 0);

  inspector->select_cell (data, obj->u_ldata);
  inspector->update_form ();
}


void center_on_selected (FL_OBJECT *obj, long data)
{
  assert (obj != 0);

  inspector->center_on_selected ();
}


void scroll_diagonally (FL_OBJECT *obj, long data)
{
  assert (obj != 0);
  assert (data == 0 || data == 1);

  inspector->scroll_browser_diagonally (data);
}


void change_precision (FL_OBJECT *obj, long data)
{
  inspector->set_precision ();
  inspector->update_form ();
}


void redraw_form (FL_OBJECT *obj, long data)
{
  assert (obj != 0);

  inspector->redraw_form ();

  data = data;
}


void hide_form (FL_OBJECT *obj, long data)
{
  inspector->hide_form ();

  obj = obj;  data = data;
}


void switch_norm_type (FL_OBJECT *obj, long data)
{
  assert (data >= Scene_Inspect::HINFNORM && data < Scene_Inspect::BAD_NORM_TYPE);
  inspector->switch_norm_type (Scene_Inspect::Norm_Type (data));

  obj = obj;
}


/******************** Class Scene_Inspect *************************************/

// This is the workhorse of the inspect form, responsible for extracting data
// from a Scene object passed to it and displaying it on the form.


// Constructor

Scene_Inspect::Scene_Inspect (FD_forminspect *form_in, Scene *scene_in) :
  _form (form_in),
  _scene (scene_in),
  _covariance_browser (new Matrix_Browser
		      (form_in->forminspect,
		       _DEFAULT_CELL_SIZE, _DEFAULT_CELL_SIZE / 2,
		       form_in->frame_covariance)),
  _norm_type (_DEFAULT_NORM_TYPE)
{
  _norm_matrix = hor_mat_alloc (1, 1);
  _selected_covariance = hor_mat_alloc (_MAX_STATE_DIMENSION,
					_MAX_STATE_DIMENSION);
  fl_set_counter_value (_form->counter_cell_size, _DEFAULT_CELL_SIZE);
  set_precision ();

  // This, a modification to some form object attributes,
  // necessary because of bugs in fdesign, should go in forminspect.c.
  // However, it's too boring to redo it every time I change the form,
  // so, for the moment, I do it here.
  // *** CHANGE FOR FINAL VERSION ***
  fl_set_counter_step (_form->counter_cell_size, 1.0, 1.0);
  fl_set_counter_step (_form->counter_precision, 1.0, 1.0);
  fl_set_object_lsize (_form->text_selected_covariance, FL_TINY_SIZE);
  fl_set_object_lstyle (_form->text_selected_covariance, FL_FIXED_STYLE);
}


// Hide the form (controlling program will have to reshow)

void Scene_Inspect::hide_form ()
{
  fl_hide_form (_form->forminspect);
}


// Redraw whole form (say text display goes weird)

void Scene_Inspect::redraw_form ()
{
  fl_redraw_form (_form->forminspect);
}


// Update all info on form

int Scene_Inspect::update_form ()
{
  update_covariance_browser ();

  return 0;
}


// Update info just for the covariance browser

void Scene_Inspect::update_covariance_browser ()
{
  calc_norm_matrix ();
  hor_matq_scale (_norm_matrix, 1000);
  scroll_covariance_browser ();
  update_selected_covariance ();
}


// Update info in the text box which gives us the full covariance matrix
// of the selected cell

void Scene_Inspect::update_selected_covariance ()
{
  calc_selected_covariance ();

  // Form _selected_covariance matrix into a string
  char covariance_string [_MAXIMUM_COVARIANCE_CHARS];
  int precision = _covariance_browser->get_precision ();
  Hor_Matrix *M = _selected_covariance;
  hor_matq_scale (M, 1000);

  char *str_section = covariance_string;
  for (int r = 0; r < M->rows; r++) {
    for (int c = 0; c < M->cols; c++) {

      str_section += sprintf (str_section, "%*.*g ",
			      c == M->cols - 1 ? 0 : -max (precision + 3, 6),
			      precision, M->m [r][c]);
    }
    *str_section = '\n';
    str_section++;
  }
  if (!(M->rows == 0 && M->cols == 0)) --str_section;
  *str_section = '\0';

  // Put it in the text box
  fl_set_object_label (_form->text_selected_covariance, covariance_string);
}


// Scroll covariance browser.  That is, update with the same matrix

void Scene_Inspect::scroll_covariance_browser ()
{
  correct_scrollbars ();
  bool ok = !_covariance_browser->display
    (_norm_matrix,
     (int) (fl_get_scrollbar_value (_form->scrollbar_covariance_vertical) - 1),
     (int) (fl_get_scrollbar_value 
                    (_form->scrollbar_covariance_horizontal) - 1));
  assert (ok);
}


// Increment or decrement each scrollbar position by one and redraw

void Scene_Inspect::scroll_browser_diagonally (bool downright)
{
  int addon = downright ? 1 : -1;
  int updown = Scene_Inspect_utilities::round
    (fl_get_scrollbar_value (_form->scrollbar_covariance_vertical)) +  addon;
  int leftright = Scene_Inspect_utilities::round
    (fl_get_scrollbar_value (_form->scrollbar_covariance_horizontal)) + addon;

  fl_set_scrollbar_value (_form->scrollbar_covariance_vertical, updown);
  fl_set_scrollbar_value (_form->scrollbar_covariance_horizontal, leftright);
  scroll_covariance_browser ();
}

// Delete and redraw the covariance browser (if it is desired to wipe it clean)

void Scene_Inspect::redraw_covariance_browser ()
{
  delete _covariance_browser;
  int cell_size = Scene_Inspect_utilities::round
    (fl_get_counter_value (_form->counter_cell_size));

  _covariance_browser = new Matrix_Browser (_form->forminspect,
					    cell_size, cell_size / 2,
					    _form->frame_covariance);

  update_covariance_browser ();
}


// Redraw covariance browser at new cell size

void Scene_Inspect::change_browser_cellsize (int cellsize)
{
  double min, max;
  fl_get_counter_bounds (_form->counter_cell_size, &min, &max);

  // Check validity of cellsize, modify form if necessary
  if (cellsize < min) {
    cellsize = Scene_Inspect_utilities::round (min);
    fl_set_counter_value (_form->counter_cell_size, cellsize);
  }
  else if (cellsize > max) {
    cellsize = Scene_Inspect_utilities::round (max);
    fl_set_counter_value (_form->counter_cell_size, cellsize);
  }

  _covariance_browser->change_cell_sizes (cellsize, 
                                 (int) (cellsize * _LABEL_SIZE_RATIO));
  update_covariance_browser ();
}


// This cell has been selected by the user.  The coordinates refer to
// matrix row and column

void Scene_Inspect::select_cell (int row, int col, bool rowcol_only = false)
{
  if (_covariance_browser->_important_cells.size () > 1) {
    _covariance_browser->_important_cells.pop_front ();
  }
  if (!rowcol_only) _covariance_browser->_important_cells.push_back
		     (Matrix_Browser::Coord (row, col));

  if (_covariance_browser->_important_rows.size () > 1) {
    _covariance_browser->_important_rows.pop_front ();
  }
  _covariance_browser->_important_rows.push_back (row);

  if (_covariance_browser->_important_cols.size () > 1) {
    _covariance_browser->_important_cols.pop_front ();
  }
  _covariance_browser->_important_cols.push_back (col);
}


// Center the covariance browser (by altering scrollbars) on the currently
// selected cell

void Scene_Inspect::center_on_selected ()
{
  if (_covariance_browser->_important_cells.size () == 0) {
    return;
  }

  // Calculate some sizes and things
  FL_Coord cell_size =
    FL_Coord (Scene_Inspect_utilities::round (fl_get_counter_value
					      (_form->counter_cell_size)));
  static FL_Coord frame_width, frame_height;
  static bool first_time = true;
  if (first_time) {
    frame_width = _form->frame_covariance->w;
    frame_height = _form->frame_covariance->h;
    frame_width -= FL_Coord (cell_size * _LABEL_SIZE_RATIO);
    frame_height -= FL_Coord (cell_size * _LABEL_SIZE_RATIO);
    first_time = false;
  }
  int num_cells_updown = int (frame_height / cell_size);
  int num_cells_leftright = int (frame_width / cell_size);

  Matrix_Browser::Coord selected =
    _covariance_browser->_important_cells.back ();
  fl_set_scrollbar_value (_form->scrollbar_covariance_vertical,
			  selected.row + 1 - (num_cells_updown / 2));
  fl_set_scrollbar_value (_form->scrollbar_covariance_horizontal,
			  selected.col + 1 - (num_cells_leftright / 2));
  scroll_covariance_browser ();
}


// Change the precision of numbers in the covariance browser and selected
// covariance box

void Scene_Inspect::set_precision ()
{
  int precision = Scene_Inspect_utilities::round
    (fl_get_counter_value (_form->counter_precision));
  assert (precision > 0);

  _covariance_browser->set_precision (precision);
}


// Change the method by which norms of matrix blocks are calculated for
// display

void Scene_Inspect::switch_norm_type (Scene_Inspect::Norm_Type nt)
{
  assert (nt >= HINFNORM && nt < BAD_NORM_TYPE);

  /*
  fl_set_button (_form->button_norm_hinf, nt == Scene_Inspect::HINFNORM);
  fl_set_button (_form->button_norm_volume, nt == Scene_Inspect::VOLUME);
  fl_set_button (_form->button_norm_frobenius, nt == Scene_Inspect::FROBENIUS);
  fl_set_button (_form->button_norm_trace, nt == Scene_Inspect::TRACE);
  */

  _norm_type = nt;
  this->update_covariance_browser ();
}


// Calculate the matrix of covariance norms from the scene object

void Scene_Inspect::calc_norm_matrix ()
{
  hor_mat_free (_norm_matrix);
  int state_size = 1 + _scene->get_no_features ();
  _norm_matrix = hor_mat_alloc (state_size, state_size);
  Hor_Matrix *M = _norm_matrix;

  // Insert top left value Pxx
  M->m[0][0] = norm (_scene->get_Pxx ());

  // Loop through features
  int k = 1;
  for (Feature *f = _scene->get_first_feature_ptr (); f != 0; f = f->get_next ()) {
    // Insert Pxk
    M->m[0][k] = norm (f->get_Pxy ());

    // Loop through each feature observed before this one
    int j = 1;
    for (Matrix_Block *mbptr = f->get_first_mbptr (); mbptr != 0;
	 mbptr = mbptr->next) {
      assert (j < k);

      // Insert Pjk
      M->m[j++][k] = norm (mbptr->m);
    }
    // Finally insert norm of Pyy
    M->m[k][k] = norm (f->get_Pyy ());

    // Take this opportunity to mark the most recently selected feature
    // for highlight
    if (f->get_selected_flag () && !f->get_next_selected ()) {
      select_cell (k, k, true);
    }

    k++;
  }
  assert (k == state_size);

  // That filled in the top-right triangle of the matrix.
  // Copy and transpose to the bottom-left.

  for (int r = 0; r < M->rows; r++) {
    for (int c = r + 1; c < M->cols; c++) {
      M->m[c][r] = M->m[r][c];
    }
  }

}


// Calculate the selected covariance matrix.  Runs along the same lines as
// calc_norm_matrix except we're not actually forming _norm_matrix.

void Scene_Inspect::calc_selected_covariance ()
{
  // Set _selected_covariance to zero matrix if nothing selected
  if (_covariance_browser->_important_cells.size () == 0) {
    _selected_covariance->rows = 0;
    _selected_covariance->cols = 0;
    return;
  }
  Matrix_Browser::Coord selected = _covariance_browser->_important_cells.back ();

  // Check Pxx
  if (selected == Matrix_Browser::Coord (0, 0)) {
    hor_matq_copy (_scene->get_Pxx (), _selected_covariance);
    return;
  }

  // Loop through features
  Matrix_Block *mbptr;
  int k = 1;
  for (Feature *f = _scene->get_first_feature_ptr (); f != 0;
       f = f->get_next (), k++) {

    if (selected.col != k) continue;

    // Check Pxk
    if (selected.row == 0) {
      hor_matq_copy (f->get_Pxy (), _selected_covariance);
      return;
    }

    // Loop through each feature observed before this one
    int j = 1;
    for (mbptr = f->get_first_mbptr (); mbptr != 0; mbptr = mbptr->next, j++) {

      // Check Pjk
      if (selected.row == j) {
	hor_matq_copy (mbptr->m, _selected_covariance);
	return;
      }
    }
    // Must be Pyy
    hor_matq_copy (f->get_Pyy (), _selected_covariance);
    return;
  }
}


// Resize scrollbars depending on state size, and ensure they're within bounds

void Scene_Inspect::correct_scrollbars ()
{
  fl_freeze_form (_form->forminspect);

  // Calculate some sizes and things
  int state_size = 1 + _scene->get_no_features ();
  FL_Coord cell_size =
    FL_Coord (Scene_Inspect_utilities::round (fl_get_counter_value
					      (_form->counter_cell_size)));
  static FL_Coord frame_width, frame_height;
  static bool first_time = true;
  if (first_time) {
    frame_width = _form->frame_covariance->w;
    frame_height = _form->frame_covariance->h;
    frame_width -= FL_Coord (cell_size * _LABEL_SIZE_RATIO);
    frame_height -= FL_Coord (cell_size * _LABEL_SIZE_RATIO);
    first_time = false;
  }

  // Now calculate how big the scrollbars should be and set
  int num_cells_updown = int (frame_height / cell_size);
  int num_cells_leftright = int (frame_width / cell_size);
  double scrollbar_height = num_cells_updown / double (state_size);
  double scrollbar_width = num_cells_leftright / double (state_size);
  if (scrollbar_height > 1.0) scrollbar_height = 1.0;
  if (scrollbar_width > 1.0) scrollbar_width = 1.0;
  fl_set_scrollbar_size (_form->scrollbar_covariance_vertical, scrollbar_height);
  fl_set_scrollbar_size (_form->scrollbar_covariance_horizontal, scrollbar_width);

  // Set scrollbar position bounds and force scrollbar position to within them
  int vert_max = max (state_size - num_cells_updown + 1, 1);
  int horiz_max = max (state_size - num_cells_leftright + 1, 1);
  double vert_pos = fl_get_scrollbar_value (_form->scrollbar_covariance_vertical);
  double horiz_pos = fl_get_scrollbar_value (_form->scrollbar_covariance_horizontal);
  fl_set_scrollbar_bounds (_form->scrollbar_covariance_vertical, 1, vert_max);
  fl_set_scrollbar_bounds (_form->scrollbar_covariance_horizontal, 1, horiz_max);
  if (vert_pos < 1)
    fl_set_scrollbar_value (_form->scrollbar_covariance_vertical, 1);
  else if (vert_pos > vert_max)
    fl_set_scrollbar_value (_form->scrollbar_covariance_vertical, vert_max);
  if (horiz_pos < 1)
    fl_set_scrollbar_value (_form->scrollbar_covariance_horizontal, 1);
  else if (horiz_pos > horiz_max)
    fl_set_scrollbar_value (_form->scrollbar_covariance_horizontal, horiz_max);

  fl_unfreeze_form (_form->forminspect);
}


double Scene_Inspect::norm (Hor_Matrix *M)
{
  assert (M != NULL);

  double out;

  switch (_norm_type) {
  case HINFNORM:
    out = hor_hinfnorm (M);
    break;

  case VOLUME:
    out = hor_uncertainty_volume (M, 3.0);
    break;

  case FROBENIUS:
    out = hor_frobenius_norm (M);
    break;

  case TRACE:
    out = hor_trace (M);
    break;

  default:
    bool SHOULDNT_REACH_HERE = false;
    assert (SHOULDNT_REACH_HERE);
  }

  assert (hor_errno == HORATIO_OK);
  return out;
}


/******************** Class Matrix_Browser ************************************/

// Draws a chunk of a matrix on a form as an
// array of text boxes.  It will expand the array if the matrix expands.  If
// the required matrix block to be displayed changes, the new values are
// written into the same boxes.


// Constructor

Matrix_Browser::Matrix_Browser (FL_FORM *f, FL_Coord cell_size_in, 
				FL_Coord label_size_in, FL_OBJECT *frame_obj) :
  _form (f),
  _cell_size (cell_size_in),
  _label_size (label_size_in),
  _precision (_DEFAULT_PRECISION)
{
  assert (f != 0);
  assert (cell_size_in > 0);
  assert (label_size_in > 0);
  assert (frame_obj != 0);

  // Set some data
  fl_get_object_geometry (frame_obj, &_frame_left, &_frame_top,
			  &_frame_width, &_frame_height);
  _cells_wide = (_frame_width - _label_size) / _cell_size;
  _cells_high = (_frame_height - _label_size) / _cell_size;

  // Create the array of text boxes (which are actually buttons, but don't worry)
  create_cell_array ();
}


// Destructor, delete the cells from the form

Matrix_Browser::~Matrix_Browser ()
{
  delete_cell_array ();
}


// Change the size of the cells in the matrix array

int Matrix_Browser::change_cell_size (int cell_size)
{
  // Maintain same ratio of label size to cell size
  int label_size = cell_size * _label_size / _cell_size;
  return change_cell_sizes (cell_size, label_size);
}


int Matrix_Browser::change_cell_sizes (int cell_size, int label_size)
{
  if (cell_size <= 0) return 1;
  if (label_size <= 0) return 2;

  _cell_size = cell_size;
  _label_size = label_size;

  delete_cell_array ();
  _cells_wide = (_frame_width - _label_size) / _cell_size;
  _cells_high = (_frame_height - _label_size) / _cell_size;
  create_cell_array ();

  return 0;
}


// Display matrix values in cells.  Note this takes matrix rows and columns,
// ie. starting at 0.

int Matrix_Browser::display (const Hor_Matrix *M, int start_row, int start_col)
{
  assert (M != 0);

  fl_freeze_form (_form);

  // Fill in the row and column numbers in the first row/column of the array
  // (I've been calling them `labels')
  int row = start_row;
  for (int r = 0; r < _cells_high; r++, row++) {

    FL_OBJECT *this_cell = _row_labels (r, 0);
    if (row >= 0 && row < M->rows) {

      put_int_in_label (this_cell, row);
      if (!this_cell->visible) fl_show_object (this_cell);
    }
    else if (this_cell->visible) fl_hide_object (this_cell);
  }

  int col = start_col;
  for (int c = 0; c < _cells_wide; c++, col++) {

    FL_OBJECT *this_cell = _col_labels (0, c);
    if (col >= 0 && col < M->cols) {

      put_int_in_label (this_cell, col);
      if (!this_cell->visible) fl_show_object (this_cell);
    }
    else if (this_cell->visible) fl_hide_object (this_cell);
  }

  // Run through each cell in turn and put a value in it if the relevant
  // element of the matrix (starting at (start_row, start_col)) exists.
  // If it doesn't exist, hide the cell.
  row = start_row;
  for (int r = 0; r < _cells_high; r++, row++) {
    col = start_col;
    for (int c = 0; c < _cells_wide; c++, col++) {

      FL_OBJECT *this_cell = _cells (r, c);
      if (row >= 0 && row < M->rows &&
	  col >= 0 && col < M->cols) {

	put_value_in_cell (M->m [row][col], r, c, row, col);
	if (!this_cell->visible) fl_show_object (this_cell);
      }
      else if (this_cell->visible) fl_hide_object (this_cell);
    }
  }

  // If we're showing this cell, we want it to know which matrix
  // element it is for when it's clicked on.  We use the data passed
  // to the callback function, and the FL_OBJECT member u_ldata
  row = start_row;
  for (int r = 0; r < _cells_high; r++, row++) {
    col = start_col;
    for (int c = 0; c < _cells_wide; c++, col++) {

      FL_OBJECT *this_cell = _cells (r, c);
      if (this_cell->visible) {
	fl_set_object_callback (this_cell, select_cell, row);
	this_cell->u_ldata = col;
      }
    }
  }

  fl_unfreeze_form (_form);

  return 0;
}


// Create the array of matrix cells and their labels

void Matrix_Browser::create_cell_array ()
{
  _cells.alloc (_cells_wide, _cells_high);
  _row_labels.alloc (1, _cells_high);
  _col_labels.alloc (_cells_wide, 1);

  fl_freeze_form (_form);
  fl_addto_form (_form);

  // Create the row labels
  _row_labels.group = fl_bgn_group ();
  for (int r = 0; r < _cells_high; r++) {

    create_cell (_row_labels.get_ptr (r, 0),
		 _frame_left, _frame_top + _label_size + r * _cell_size,
		 _label_size, _cell_size);
  }
  fl_end_group ();

  // Create the column labels
  _col_labels.group = fl_bgn_group ();
  for (int c = 0; c < _cells_high; c++) {

    create_cell (_col_labels.get_ptr (0, c),
		 _frame_left + _label_size + c * _cell_size, _frame_top,
		 _cell_size, _label_size);
  }
  fl_end_group ();

  // Create the cells
  _cells.group = fl_bgn_group ();
  for (int r = 0; r < _cells_high; r++) {
    for (int c = 0; c < _cells_wide; c++) {

      create_cell (_cells.get_ptr (r, c),
		   _frame_left + _label_size + c * _cell_size,
		   _frame_top + _label_size + r * _cell_size,
		   _cell_size, _cell_size);
    }
  }
  fl_end_group ();

  fl_end_form ();
  fl_unfreeze_form (_form);
}


// Delete the array of matrix cells and their labels

void Matrix_Browser::delete_cell_array ()
{
  fl_freeze_form (_form);

  // Delete row labels;
  for (int r = 0; r < _cells_high; r++) {

    fl_delete_object (_row_labels (r, 0));
    fl_free_object (_row_labels (r, 0));
  }
  fl_delete_object (_row_labels.group);
  fl_free_object (_row_labels.group);

  // Delete column labels;
  for (int c = 0; c < _cells_wide; c++) {

    fl_delete_object (_col_labels (0, c));
    fl_free_object (_col_labels (0, c));
  }
  fl_delete_object (_col_labels.group);
  fl_free_object (_col_labels.group);

  // Delete cell array
  for (int r = 0; r < _cells_high; r++) {
    for (int c = 0; c < _cells_wide; c++) {

      fl_delete_object (_cells (r, c));
      fl_free_object (_cells (r, c));
    }
  }
  fl_delete_object (_cells.group);
  fl_free_object (_cells.group);

  fl_unfreeze_form (_form);
}


// Create the button object associated with a cell, in default form

void Matrix_Browser::create_cell (FL_OBJECT **cell_ptr, FL_Coord left, FL_Coord top,
				  FL_Coord width, FL_Coord height)
{
  assert (cell_ptr != 0);
  assert (left >= _frame_left && top >= _frame_top);
  assert (left <= _frame_left + _frame_width - _cell_size);
  assert (top <= _frame_top + _frame_height - _cell_size);
  assert (width > 0 && height > 0);

  *cell_ptr = fl_add_button (FL_NORMAL_BUTTON, left, top,
			     width, height, "");

  fl_set_object_boxtype (*cell_ptr, FL_BORDER_BOX);
  fl_set_object_color (*cell_ptr, _CELL_BACKGROUND_COLOUR, FL_RED);
  fl_set_object_lsize(*cell_ptr, FL_TINY_SIZE);
  fl_set_object_bw (*cell_ptr, _CELL_BORDER_WIDTH);

  // If width != height this is a label cell.  Set callback for main
  // cells only
  if (width == height) fl_set_object_callback (*cell_ptr, select_cell, 0);

  // Hide cell on creation
  fl_hide_object (*cell_ptr);
}


// Write a floating point value into a cell, and give it any special properties
// (colour etc) warranted by its position in the matrix

void Matrix_Browser::put_value_in_cell (double val, int cell_row, int cell_col,
					int mat_row, int mat_col)
{
  assert (cell_row >= 0 && cell_row < _cells_high);
  assert (cell_col >= 0 && cell_col < _cells_wide);

  FL_OBJECT *this_cell = _cells (cell_row, cell_col);
  char thelabel [_MAXIMUM_LABEL_CHARS];
  snprintf (thelabel, _MAXIMUM_LABEL_CHARS, "%0.*g", _precision, val);
  fl_set_object_label (this_cell, thelabel);

  // Use the long value passed to the cell's callback function, and
  // the button object's u_ldata member to tell the cell which matrix
  // cell it is
  fl_set_object_callback (this_cell, select_cell, mat_row);
  this_cell->u_ldata = mat_col;

  // Give different background to the diagonal
  if (mat_row == mat_col) {
    fl_set_object_color (this_cell, _CELL_DIAGONAL_BACKCOLOUR, FL_BLACK);
  }
  else {
    fl_set_object_color (this_cell, _CELL_BACKGROUND_COLOUR, FL_BLACK);
  }

  // Give special colours to important cells, rows, and columns
  using Scene_Inspect_utilities::in;
  if (in (_important_cells, Coord (mat_row, mat_col))) {
    fl_set_object_color (this_cell, _IMPORTANT_CELL_COLOUR, FL_BLACK);
  }
  else if (in (_important_rows, mat_row)) {
    fl_set_object_color (this_cell, _IMPORTANT_ROW_COLOUR, FL_BLACK);
  }
  else if (in (_important_cols, mat_col)) {
    fl_set_object_color (this_cell, _IMPORTANT_COL_COLOUR, FL_BLACK);
  }
}


// Write an integer value into a row/column label, and give it any
// special properties warranted by its position

void Matrix_Browser::put_int_in_label (FL_OBJECT *label_obj, int label)
{
  assert (label_obj != 0);

  char thelabel [_MAXIMUM_LABEL_CHARS];
  snprintf (thelabel, _MAXIMUM_LABEL_CHARS, "%d", label);
  fl_set_object_label (label_obj, thelabel);
}
