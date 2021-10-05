/*  Scene: software for sequential localisation and map-building

    feature.h
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Additions from Joss Knight
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

class Feature;
class Scene_Single;

// Feature and Matrix_Block classes

// Scene, Feature and Matrix_Block classes
typedef void * Identifier;

/****************************Matrix_Block Class*******************************/

/* Matrix_Block is a linkable wrapper for a Hor_Matrix which is used to 
   store the cross-covariance between two features */

class Matrix_Block {
  friend class Feature;
  friend class Scene_Single;
  friend class Scene_Inspect;     // JK addition 21/8
  friend class Kalman;            // JK addition 19/9/00
protected:
  Matrix_Block(Hor_Matrix *mat, int px, int py);
  ~Matrix_Block();

  // Functions for adjusting the position after a feature has been deleted
  void decrement_posx() {
    posx--;
  }
  void decrement_posx_posy() {
    posx--;
    posy--;
  }

  Matrix_Block *next;
  Hor_Matrix *m;
  Hor_Matrix *m_new;
  double jghk_m_weight;  // Temporary for testing -- JGHK 24/10/00
  int posx, posy;        // This block's position in the matrix array
  int &sizex, &sizey;    // The size of the block
};

/******************************Feature Class**********************************/

class Feature {
  friend class Scene_Single; 
  friend class Kalman;

  // Most stuff protected because only accessed through friend classes
  // Scene and Kalman

  // Public access to some things though
 public:
  Feature_Measurement_Model *get_feature_measurement_model() 
                                       {return feature_measurement_model;}
  Hor_Matrix *get_h() {return h;}
  Hor_Matrix *get_z() {return z;}
  Hor_Matrix *get_nu() {return nu;}  
  Hor_Matrix *get_y() {return y;}
  Hor_Matrix *get_Pyy() {return Pyy;}
  Hor_Matrix *get_Pxy() {return Pxy;}
  Hor_Matrix *get_S() {return S;}
  Feature *get_next() {return next;}
  Feature *get_next_selected() {return next_selected;}
  int get_label() {return label;}
  Identifier get_identifier() {return identifier;}
  int get_selected_flag() {return selected_flag;}
  Matrix_Block *get_first_mbptr () {return first_mbptr;}  // JK addition 21/8
  int get_age () {return age;}
  void increment_age () {age++;}
  void zero_age () {age = 0;}

  // Copy matrix_news to originals, and the other way round
  int bring_covariances_up_to_date ();
  int copy_covariances_to_new ();

 protected:
  // Constructors: two sorts for unknown and known features
  // Differentiated by number of arguments

  // Unknown feature: initialised with a measurement
  Feature(Identifier id, int lab, int list_pos, 
	  Scene_Single *scene,  Hor_Matrix *h, Feature_Measurement_Model *f_m_m);
  // Known feature: initialised with state
  Feature(Identifier id, int lab, int list_pos, 
	  Scene_Single *scene, Hor_Matrix *y_known, Hor_Matrix *xp_o, 
	  Feature_Measurement_Model *f_m_m, int k_f_l);
  int feature_constructor_bookeeping();
  int reinitialise (Scene_Single *scene);


  // Destructor
  ~Feature();              
  int scheduled_for_termination_flag;
                           /* Set if we want shortly to delete this feature
			      but can't do it right now */

  // Function which removes matrix blocks when another feature is deleted
  int feature_is_removed(int list_pos);
  
 protected:
  // Measurement model pointer
  Feature_Measurement_Model *feature_measurement_model;

 protected:
  // Important bookkeeping data
  int selected_flag;       // Set if feature is selected
  int position_in_list;    /* The current position of this feature in
			      the list of features */
  int position_in_total_state_vector;
                           /* With general feature state sizes, need this
			      to identify starting position of this feature 
			      in total state vector */
  Identifier identifier;   /* Identifier is a pointer to something in the 
			      real world which identifies this feature
			      uniquely */
  int label;               // Label for this feature within scene class
  Feature *next;           // Link to the next one in the list
  Feature *next_selected;  /* Link to next selected feature
			      This is NULL if it is not selected or
			      is the last in the selected list */
  int age;                 // Gets set to zero when feature is measured


  // Important geometrical data
  Hor_Matrix *y;           // Contains the estimate of feature position
  Hor_Matrix *Pyy;         // Contains the covariance of this
  Hor_Matrix *Pxy;         // The covariance between the vehicle estimate and y
  Hor_Matrix *Pyy_new;     // New versions of these, to replace the old
  Hor_Matrix *Pxy_new;
  Hor_Matrix *xp_orig;       /* The vehicle position we initialised
			        the feature from. */
  Matrix_Block *first_mbptr;    /* Points to the first in a list of block
				   parts of the big covariance matrix */
  Matrix_Block *last_mbptr;     // Points to the last in the list

  // Places to store predicted (h) and actual (z) measurements and
  // Jacobians
  Hor_Matrix *h;
  Hor_Matrix *hinit;
  Hor_Matrix *z;
  Hor_Matrix *nu; // Innovation
  Hor_Matrix *dh_by_dxv;
  Hor_Matrix *dh_by_dy;
  Hor_Matrix *R;
  Hor_Matrix *S;

  // Keep track of the attempted and successful measurements made of a 
  // feature over time 
  int successful_measurement_flag;
  int attempted_measurements_of_feature;
  int successful_measurements_of_feature;

  // Set to a label >= 0 if this is a known feature; otherwise -1
  int known_feature_label;

  // Public flags: flags can stay public because it's not possible to cock
  // up changing them -- they can only be true or false
 public:
  bool nebot_ignore;
  bool jghk_ignore;
  double jghk_Pxy_weight;  // Temporary for testing -- JGHK 24/10/00
};

