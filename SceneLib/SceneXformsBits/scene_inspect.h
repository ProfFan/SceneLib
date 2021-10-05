/*  Scene: software for sequential localisation and map-building

    scene_inspect.h
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

#ifndef _scene_inspect_h_
#define _scene_inspect_h_
/******************************************************************************
 * scene_inspect.h
 *
 * Callback functions for the inspect form (forminspect) and
 * associated classes Scene_Inspect and Matrix_Browser.
 * Matrix_Browser may be useful outside the context of this particular
 * form but probably not, so it's defined in this compilation unit.
 *
 * Original code by Joss Knight
 ******************************************************************************/

/******************** Included Files ******************************************/

/******************** Matrix_Browser class ************************************/

// Draws a chunk of a matrix on a form as an array of text boxes.  It
// will expand the array if the matrix expands.  If the required
// matrix block to be displayed changes, the new values are written
// into the same boxes.  The boxes are actually buttons so they can be
// selected.

class Matrix_Browser
{
  // Constants
  static const int _DEFAULT_PRECISION = 1;
  static const FL_COLOR _CELL_BACKGROUND_COLOUR = FL_LEFT_BCOL;
  static const FL_COLOR _CELL_DIAGONAL_BACKCOLOUR = FL_TOMATO;
  static const FL_COLOR _IMPORTANT_CELL_COLOUR = FL_BLUE;
  static const FL_COLOR _IMPORTANT_ROW_COLOUR = FL_SLATEBLUE;
  static const FL_COLOR _IMPORTANT_COL_COLOUR = FL_SLATEBLUE;
  static const FL_Coord _CELL_BORDER_WIDTH = 2;
  static const int _MAXIMUM_LABEL_CHARS = 20;

  // Types
  class Cells
  {
  public:
    Cells () : cell (0), group (0) {}
    Cells (int w, int h) :
      width (w), height (h), cell (new FL_OBJECT * [w*h]), group (0) {}
    ~Cells () {delete[] cell;}

  private:
    int width, height;
    FL_OBJECT **cell;

  public:
    FL_OBJECT *group;

  public:
    void alloc (int w, int h) {
      width = w;
      height = h;
      delete cell;
      cell = new FL_OBJECT * [w*h];
    }
    FL_OBJECT *operator() (int r, int c) {
      if (r < 0 || c < 0 || r >= height || c >= width) return 0;
      return (cell [r * width + c]);
    }
    FL_OBJECT **get_ptr (int r, int c) {
      if (r < 0 || c < 0 || r >= height || c >= width) return 0;
      return (cell + (r * width + c));
    }
  };

 public:
  struct Coord {
    Coord () {}
    Coord (int r, int c) {row = r; col = c;}

    int row;
    int col;

    bool operator== (const Coord &that) const {
      return (row == that.row && col == that.col);
    }
  };

  // Constructors, destructors
 public:
  Matrix_Browser (FL_FORM *f, FL_Coord cell_size_in, FL_Coord label_size_in, 
		  FL_OBJECT *frame_obj);
  ~Matrix_Browser ();

  // Allow public access to vectors telling which cells to highlight
 public:
  list<Coord> _important_cells;
  list<int> _important_rows;
  list<int> _important_cols;

  // Key data
 protected:
  FL_FORM *_form;        // We're drawing on this form
  FL_Coord _cell_size;   // With this size of box per value
  FL_Coord _label_size;  // This size of row/column label
  FL_Coord _frame_top, _frame_left, _frame_width, _frame_height; // Bounding frame
  Cells _cells;          // Array of pointers to cell buttons
  Cells _row_labels;     // Similarly for the row/column labels
  Cells _col_labels;

  // General data
 protected:
  int _precision;        // Display values to this precision
  int _cells_wide, _cells_high;

  // Access methods
 public:
  int set_precision (int precision_in) {
    if (precision_in > 0) {
      _precision = precision_in;
      return 0;
    }
    else return 1;
  }
  int get_precision () {return _precision;}
  int change_cell_size (FL_Coord cell_size);
  int change_cell_sizes (FL_Coord cell_size, FL_Coord label_size);

  // Display
 public:
  int display (const Hor_Matrix *matrix, int start_row, int start_col);

  // Utilities
 protected:
  void create_cell_array ();
  void delete_cell_array ();
  void create_cell (FL_OBJECT **cell_ptr, FL_Coord left, FL_Coord top,
		    FL_Coord width, FL_Coord height);
  void put_value_in_cell (double val, int cell_row, int cell_col,
			  int mat_row, int mat_col);
  void put_int_in_label (FL_OBJECT *label_obj, int label);
};


/******************** Scene_Inspect Class *************************************/

// This is the workhorse of the inspect form, responsible for extracting data
// from a Scene object passed to it and displaying it on the form.

class Scene_Inspect
{
 public:
  // Types
  enum Norm_Type {HINFNORM, VOLUME, FROBENIUS, TRACE, BAD_NORM_TYPE};

  // Constants
 protected:
  static const int _DEFAULT_CELL_SIZE = 29;
  static const float _LABEL_SIZE_RATIO = 0.5;
  static const int _MAXIMUM_COVARIANCE_CHARS = 1000;
  static const int _MAX_STATE_DIMENSION = 10;
  static const Norm_Type _DEFAULT_NORM_TYPE = HINFNORM;

  // Constructors, destructors
 public:
  Scene_Inspect(FD_forminspect* form_in, Scene *scene_in);
  ~Scene_Inspect () {
    delete _covariance_browser;
    hor_mat_free_list (_norm_matrix, _selected_covariance, 0);
  }

  // Key data
 protected:
  FD_forminspect *_form;               // Output is on this form
  Scene *_scene;                       // We're inspecting this
  Matrix_Browser *_covariance_browser; // Displays all or part of covariance data

  // General data
 protected:
  Hor_Matrix *_norm_matrix;             // Matrix of covariance norms, saved
  Hor_Matrix *_selected_covariance;     // A single covariance matrix block
  Norm_Type _norm_type;

  // Updating methods
 public:
  void hide_form ();
  void redraw_form ();
  int update_form ();
  void update_covariance_browser ();
  void update_selected_covariance ();
  void scroll_covariance_browser ();
  void scroll_browser_diagonally (bool);
  void redraw_covariance_browser ();
  void change_browser_cellsize (int cell_size);
  void select_cell (int row, int col, bool = false);
  void center_on_selected ();

  // Access
 public:
  void set_precision ();
  void switch_norm_type (Norm_Type nt);

  // Utilities
 protected:
  void calc_norm_matrix ();
  void calc_selected_covariance ();
  void correct_scrollbars ();
  double norm (Hor_Matrix *M);
};

#endif
/******************** Don't add anything after this line **********************/
