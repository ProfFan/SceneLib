/*  Scene: software for sequential localisation and map-building

    scene_single.cc
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

#include <general_headers.h>
#include "models_base.h"
#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"

/**************************Scene_Single Constructor***************************/

Scene_Single::Scene_Single(Hor_Matrix *initial_xv, 
	     Hor_Matrix *initial_Pxx,
	     Motion_Model *m_m)
  : motion_model(m_m)
{
  scene_constructor_bookkeeping(initial_xv, initial_Pxx);
}

// Special constructor when using internal measurement model
Scene_Single::Scene_Single(Hor_Matrix *initial_xv, Hor_Matrix *initial_Pxx,
	     Motion_Model *m_m, Internal_Measurement_Model *i_m_m)
  : motion_model(m_m),
    internal_measurement_model(i_m_m)
{
  // Call normal constructor to do the normal stuff
  scene_constructor_bookkeeping(initial_xv, initial_Pxx);

  internal_measurement_model = i_m_m;

  hv = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE, 1);
  zv = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE, 1);
  nuv = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE, 1);
  dhv_by_dxv = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE, 
			     motion_model->STATE_SIZE);
  Rv  = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE,
		      internal_measurement_model->MEASUREMENT_SIZE);
  Sv  = hor_mat_alloc(internal_measurement_model->MEASUREMENT_SIZE,
		      internal_measurement_model->MEASUREMENT_SIZE);
}

/* Generalised: now pass initial state
   and the functions describing state behaviour */

int Scene_Single::scene_constructor_bookkeeping(Hor_Matrix *initial_xv, 
	     Hor_Matrix *initial_Pxx)
{
  assert(initial_xv->rows == motion_model->STATE_SIZE &&
	 initial_Pxx->rows == motion_model->STATE_SIZE &&
	 initial_Pxx->cols == motion_model->STATE_SIZE);

  xv = hor_mats_copy(initial_xv);
  Pxx = hor_mats_copy(initial_Pxx);

  // Bookkeeping initialisation
  first_feature_ptr = NULL;
  first_selected_ptr = NULL;
  last_feature_ptr = NULL;
  last_selected_ptr = NULL;
  no_features = 0;
  no_selected = 0;
  next_free_label = 0;

  marked_feature_label = -1;

  Matrix_Array = NULL; 
  Matrix_Array_dimension = 0;
  total_state_size = motion_model->STATE_SIZE;

  output_counter = 0;

  return 0;
}

/*********************** Access **********************************************/

int Scene_Single::get_Pyjyk (int j, int k, Hor_Matrix *Pyjyk)
{
  assert (j > 0 && k > 0);
  assert (Pyjyk != NULL);

  if (k >= j) {
    hor_matq_copy (Matrix_Array[k-1][j-1], Pyjyk);
    assert (hor_errno == HORATIO_OK);
  }
  else {
    hor_matq_transpose (Matrix_Array[j-1][k-1], Pyjyk);
    assert (hor_errno == HORATIO_OK);
  }

  return 0;
}


Hor_Matrix *Scene_Single::get_Pyjyk_ptr (int j, int k)
{
  assert (j > 0 && k > 0);
  if (j > k) return NULL;

  return Matrix_Array[k-1][j-1];
}


/************************Scene_Single Member Functions************************/

/* Function to unite the bookeeping stuff for the two functions below */
int Scene_Single::add_new_feature_bookeeping(Feature *nf)
{
  if (first_feature_ptr == NULL)
    first_feature_ptr = nf;
  else
    last_feature_ptr->next = nf;

  if (first_selected_ptr == NULL)
    first_selected_ptr = nf;
  else
    last_selected_ptr->next_selected = nf;

  last_feature_ptr = nf;
  last_selected_ptr = nf;
  
  no_selected++;
  no_features++;
  next_free_label++; // Potential millenium-style bug when this overloads
                     // (hello if you're reading this in the year 3000)

  fill_matrix_pointer_array();

  mark_feature_by_lab(nf->label);   // Mark this feature

  return 0;
}

int Scene_Single::add_new_feature(Identifier id, Hor_Matrix *h, 
			   Feature_Measurement_Model *f_m_m)
{
  Feature *nf = new Feature(id, next_free_label, no_features, 
  			    this, h, f_m_m);

  add_new_feature_bookeeping(nf);

  return nf->label;
}

/* Special add_new_feature for known features */

int Scene_Single::add_new_known_feature(Identifier id, Hor_Matrix *y_known, 
			   Hor_Matrix *xp_o, Feature_Measurement_Model *f_m_m,
				 int known_feature_label)
{
  Feature *nf = new Feature(id, next_free_label, no_features, 
		       this, y_known, xp_o, f_m_m, known_feature_label);
  add_new_feature_bookeeping(nf);

  // Want known features deselected as default
  deselect_feature(nf);
  
  return nf->label;
}

/*********Pass feature identifier as argument and get Feature pointer*********/

Feature *Scene_Single::find_feature(Identifier id)
{
  for (Feature *f = first_feature_ptr; f; f = f->next)
    if (f->identifier == id)
      return f;

  // If we've got here the identifier hasn't been found 
  return NULL;
}

/***********Pass feature label as argument and get Feature pointer************/

Feature *Scene_Single::find_feature_lab(int lab)
{
  for (Feature *f = first_feature_ptr; f; f = f->next)
    if (f->label == lab)
      return f;

  // If we've got here the identifier hasn't been found 
  return NULL;
}

/**********Pass feature pointer to add feature to the selected list***********/

int Scene_Single::select_feature(Feature *fp)
{
  if (fp->selected_flag == 1)
  {
    cout << "Feature with label " << fp->label << " is already selected." 
         << endl;
    return 1;
  }

  fp->selected_flag = 1;
  no_selected++;

  if (first_selected_ptr == NULL)
    first_selected_ptr = fp;
  else
    last_selected_ptr->next_selected = fp;

  last_selected_ptr = fp;

  return 1;
}

/*********Pass feature label to remove feature from the selected list*********/

int Scene_Single::deselect_feature(Feature *fp)
{ 
  if (fp->selected_flag == 0)
  { 
    cout << "Feature with label " << fp->label << " is already deselected."
         << endl;
    return 1;
  }

  // Special case that we want to deselect the first feature in the list
  if (fp == first_selected_ptr)
  {
    first_selected_ptr = fp->next_selected;
    fp->next_selected = NULL;
    fp->selected_flag = 0;
    no_selected--;
    if (fp == last_selected_ptr)
      last_selected_ptr = NULL;

    return 1;
  }

  for (Feature *tfp = first_selected_ptr; tfp; tfp = tfp->next_selected)
  {
    if (tfp->next_selected == fp)
    {
      tfp->next_selected = fp->next_selected;
      fp->next_selected = NULL;
      fp->selected_flag = 0;
      no_selected--;
      if (fp == last_selected_ptr)
	last_selected_ptr = tfp;

      return 1;
    }
  }

  cerr << "Error: selected feature not found in list." << endl;
  return 0;
}

/************Toggle the selectedness of feature with identifier id************/

int Scene_Single::toggle_feature(Identifier id)
{
  Feature *fp;

  if (!(fp = find_feature(id)))
  {
    cout << "Feature with identifier " << id << " not found." << endl;
    return 0;
  }

  if (fp->selected_flag)
    return deselect_feature(fp);
  else
    return select_feature(fp);
}

/**************Toggle the selectedness of feature with label lab**************/

int Scene_Single::toggle_feature_lab(int lab)
{
  Feature *fp;

  if (!(fp = find_feature_lab(lab)))
  {
    cout << "Feature with label " << lab << " not found." << endl;
    return 0;
  }

  if (fp->selected_flag)
    return deselect_feature(fp);
  else
    return select_feature(fp);
}

/***********************Automatically select feature**************************/

int Scene_Single::auto_select_feature()
{
  int cant_see_flag;     /* Flag which we will set to 0 if a feature is 
			    visible and various other values if it isn't */

  double max_score = 0.0;
  int visible_features = 0;

  Feature *fp_selected = NULL;
  Feature *fp = NULL;

  for (fp = first_feature_ptr; fp; fp = fp->next)
  {
    predict_single_feature_measurements(fp);
    /* See if the feature is visible */
    cant_see_flag = fp->feature_measurement_model->visibility_test(
                             motion_model->xpRES, fp->y, fp->xp_orig, fp->h);
    
    /* Feature has passed visibility tests */ 
    if (cant_see_flag == 0)
    {
      visible_features++;
      double score = fp->feature_measurement_model->selection_score
	(fp->feature_measurement_model->SiRES);
      if (score > max_score)
      { 
	max_score = score;
	fp_selected = fp;
      }
    }
  }

  // Deselect all features
  for (fp = first_selected_ptr; fp; fp = first_selected_ptr)
    deselect_feature(fp);

  cout << "Best score found: " << max_score << ".\n";

  // If we've found a suitable feature, select it
  if (max_score == 0.0)
  {
    cout << "No visible features: find some new ones." << endl;
    return -1;
  }
  else 
  {
    select_feature(fp_selected);
    cout << "Auto-selected feature with label " << fp_selected->label << ".\n";
    return 0;
  }
}

/**********************Mark the last selected feature*************************/

int Scene_Single::mark_feature_by_lab(int lab)
{
  marked_feature_label = lab;
  
  // cout << "Marked feature with label " << marked_feature_label << endl;

  return 0;
}

// Mark the first feature in the features list
int Scene_Single::mark_first_feature()
{
  if (no_features == 0)
    return -1;

  mark_feature_by_lab(first_feature_ptr->label);
  return 0;
}

/*****************************Delete Feature**********************************/

/* Function to delete feature with label lab from the list we are keeping
   information about.

   As well as deleting the Feature object, we also need to tidy up the links
   to it from the previous feature. 

   And in all the subsequent Feature objects, we need to remove the matrix
   elements relating to the deleted feature. */

/// This function should not be called from within scene.cc!
/// Does not free up identifier

int Scene_Single::delete_feature()
{
  int deleted_label; // To return (for deleting from a 3D display for instance)

  if (marked_feature_label == -1)
  {
    cerr << "No feature marked to delete." << endl;
    return -1;
  }

  Feature *f_ptr, *prev_ptr = NULL;
  
  for(f_ptr = first_feature_ptr; f_ptr; f_ptr = f_ptr->next)
  {
    if (f_ptr->label == marked_feature_label)
      break;

    prev_ptr = f_ptr;
  }

  if (f_ptr == NULL)
  {
    cerr << "Error: marked feature not found in list." << endl;
    return -1;
  }

  deleted_label = f_ptr->label;

  if (f_ptr == first_feature_ptr)
  {
    /* Special case where we have the first feature in the list */
    first_feature_ptr = f_ptr->next;
  }
  else
  {
    /* Remove link to the feature from the previous one */
    prev_ptr->next = f_ptr->next;
  }

  if (f_ptr == last_feature_ptr)
  {
    /* Special case where we have the last feature in the list */
    last_feature_ptr = prev_ptr;
  }
  else
  {
    /* Remove the covariance elements relating to this feature from the 
       subsequent features */
    for (Feature *f = f_ptr->next; f; f = f->next)
      f->feature_is_removed(f_ptr->position_in_list);
  }

  if (f_ptr->selected_flag)
    deselect_feature(f_ptr);

  delete f_ptr;

  no_features--;
  marked_feature_label = -1;  

  fill_matrix_pointer_array();

  return deleted_label;
}

/****************************Exterminate Features*****************************/

/* Delete all features with scheduled_for_termination_flag set */
// Note that when calling this, go through first and eliminate
// scheduled features from 3D display
int Scene_Single::exterminate_features()
{
  Feature *feature_to_delete;

  for (Feature *fp = first_feature_ptr; fp; )
  {
    if (fp->scheduled_for_termination_flag)
    {
      feature_to_delete = fp;
      fp = fp->next;
      mark_feature_by_lab(feature_to_delete->label);
      delete_feature();
    }
    else
      fp = fp->next;
  }

  return 0;
}

/*****************************************************************************/

int Scene_Single::print_current_features()
{
  cout << "\n" << no_features << " feature" << (no_features == 1 ? "" : "s") 
       << " currently known about:" << endl;

  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    cout << "Feature identifier: " << f->identifier << ". Label: " 
         << f->label << endl;
    cout << "State estimate: " << f->y;
  }  
  
  return 0;
}

int Scene_Single::print_selected_features()
{
  cout << "\n" << no_selected << " feature" << (no_selected == 1 ? "" : "s")
       << " currently selected:" << endl;

  for (Feature *f = first_selected_ptr; f; f = f->next_selected)
    cout << "Feature identifier: " << f->identifier << ". Label: "
         << f->label << ". List position: "
	 << f->position_in_list << "." << endl;

  return 0;
}

int Scene_Single::print_covariances()
{
  Matrix_Block *mbptr;
  cout << "Covariance matrix blocks:" << endl;
  cout << "(V, V): " << Pxx;

  int y_feature_no;

  int x_feature_no = 0;
  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    cout << "(V, " << x_feature_no << "): " << f->Pxy;
    
    y_feature_no = 0;
    for (mbptr = f->first_mbptr; mbptr; mbptr = mbptr->next)
    {
      if (y_feature_no >= x_feature_no)
      {
	cerr << "Error with print_covariances." << endl;
	return -1;
      }

      cout << "(" << mbptr->posy << ", " << mbptr->posx << "): " 
           << mbptr->m;
      y_feature_no++;
    }
    cout << "(" << x_feature_no << ", " << x_feature_no << "): " << f->Pyy;

    x_feature_no++;
  }

  return 0;
}

/*********************For manipulating the whole state************************/

int Scene_Single::construct_total_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M)
{
  construct_total_state(V);
  construct_total_covariance(M);

  return 0;
}

int Scene_Single::fill_state_and_covariance(Hor_Matrix *V, Hor_Matrix *M)
{
  fill_states(V);
  fill_covariances(M);

  return 0;
}

/******Function which produces full state vector for vehicle and features*****/

int Scene_Single::construct_total_state(Hor_Matrix *V)
{
  assert (V->rows == total_state_size && V->cols == 1);

  int y_position = 0;

  hor_matq_insert_chunky1(xv, V, y_position);
  y_position += motion_model->STATE_SIZE;

  int y_feature_no = 0;

  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    hor_matq_insert_chunky1(f->y, V, y_position);
    y_feature_no++;
    y_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (y_feature_no == no_features && y_position == total_state_size);

  return 0;
}

/**************Function which produces a full covariance matrix***************/

int Scene_Single::construct_total_covariance(Hor_Matrix *M)
{
  assert (M->rows == total_state_size && M->cols == M->rows);

  hor_matq_insert_chunkyx(Pxx, M, 0, 0);

  int x_position = motion_model->STATE_SIZE;

  Matrix_Block *mbptr;

  int x_feature_no = 0;
  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    int y_feature_no = 0;
    int y_position = 0;

    hor_matq_insert_chunkyx(f->Pxy, M, y_position, x_position);
    hor_matq_insert_chunkyxT(f->Pxy, M, x_position, y_position);
    y_position += motion_model->STATE_SIZE;

    for (mbptr = f->first_mbptr; mbptr; mbptr = mbptr->next)
    {
      assert (y_feature_no < x_feature_no);

      hor_matq_insert_chunkyx(mbptr->m, M, 
			      y_position, x_position);
      hor_matq_insert_chunkyxT(mbptr->m, M, 
			       x_position, y_position);
      y_position += mbptr->sizey;
      y_feature_no++;
    }
    hor_matq_insert_chunkyx(f->Pyy, M, 
			    y_position, x_position);

    x_feature_no++;
    x_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (x_position == total_state_size);

  /* OK: all the top-right elements are right now. Let's mirror them
     to the bottom-left */
  /* Don't need to do this any more because of hor_matq_insert_chunkyxT
  for (int r = 0; r < M->rows; r++)
    for (int c = r + 1; c < M->cols; c++)
      M->m[c][r] = M->m[r][c];
  */

  return 0;
}

/*************Fill the state structures from a total state vector*************/

// JK modification 4/10/00 -- can now take V smaller than full state size, and
// will only fill until V `runs out'

int Scene_Single::fill_states(Hor_Matrix *V)
{
  assert (V->rows <= total_state_size && V->cols == 1 && V->rows > 0);

  int y_position = 0;

  hor_matq_extract_chunky1(V, xv, y_position);
  y_position += motion_model->STATE_SIZE;

  int y_feature_no = 0;

  for (Feature *f = first_feature_ptr; f && y_position < V->rows; f = f->next)
  {
    hor_matq_extract_chunky1(V, f->y, y_position);
    y_feature_no++;
    y_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (y_feature_no <= no_features && y_position <= total_state_size);

  return 0;
}

/********Fill the covariance structures from a total covariance matrix********/

// JK modification 4/10/00 -- can now take M smaller than full state size, and
// will only fill until M `runs out'

int Scene_Single::fill_covariances(Hor_Matrix *M)
{
  assert (M->rows <= total_state_size && M->cols == M->rows && M->rows > 0);

  hor_matq_extract_chunkyx(M, Pxx, 0, 0);

  int x_position = motion_model->STATE_SIZE;

  Matrix_Block *mbptr;

  int x_feature_no = 0;
  for (Feature *f = first_feature_ptr; f && x_position < M->cols; f = f->next)
  {
    int y_feature_no = 0;
    int y_position = 0;

    hor_matq_extract_chunkyx(M, f->Pxy, y_position, x_position);
    y_position += motion_model->STATE_SIZE;

    for (mbptr = f->first_mbptr; mbptr; mbptr = mbptr->next)
    {
      assert (y_feature_no < x_feature_no);

      hor_matq_extract_chunkyx(M, mbptr->m, 
			       y_position, x_position);
      y_position += mbptr->sizey;
      y_feature_no++;
    }
    hor_matq_extract_chunkyx(M, f->Pyy, 
			     y_position, x_position);

    x_feature_no++;
    x_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  assert (x_position <= total_state_size);

  return 0;
}

/********************Fill array of pointers to matrix blocks******************/

int Scene_Single::fill_matrix_pointer_array()
{
  // Allocate the array of pointers to matrix elements
  // First deallocate if we need to
  if (Matrix_Array != NULL)
  {
    for(int i=0; i < Matrix_Array_dimension; i++)
      delete Matrix_Array[i];
    delete Matrix_Array;
  }
      
  // Allocate for new size
  Matrix_Array = new Hor_Matrix**[no_features];
   for(int i=0; i < no_features; i++)
      Matrix_Array[i] = new Hor_Matrix*[no_features];

  Matrix_Array_dimension = no_features;

  Hor_Matrix **y_ptr;
  Matrix_Block *mbptr;

  total_state_size = motion_model->STATE_SIZE; // Count up total state size too

  Hor_Matrix ***x_ptr = Matrix_Array;
  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    f->position_in_total_state_vector = total_state_size;

    y_ptr = *x_ptr;
    for (mbptr = f->first_mbptr; mbptr; mbptr = mbptr->next)
    {
      *y_ptr = mbptr->m;
      y_ptr++;
    }
    *y_ptr = f->Pyy;
    x_ptr++;

    total_state_size += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }
  
  return 0;
}

/*******************Bookkeeping when measurements are made********************/

int Scene_Single::starting_measurements()
{
  successful_measurement_vector_size = 0;

  return 0;
}

int Scene_Single::failed_measurement_of_feature(Feature *sfp)
{
  cout << "Measurement failed of feature with label " << sfp->label << endl; 
  sfp->successful_measurement_flag = 0;
  sfp->attempted_measurements_of_feature++;    

  return 0;
}

int Scene_Single::successful_measurement_of_feature(Feature *sfp)
{
  sfp->successful_measurement_flag = 1;
  successful_measurement_vector_size += 
	                sfp->feature_measurement_model->MEASUREMENT_SIZE;

  sfp->feature_measurement_model->func_nui(sfp->h, sfp->z);
  hor_matq_copy(sfp->feature_measurement_model->nuiRES, sfp->nu);

  sfp->successful_measurements_of_feature++;
  sfp->attempted_measurements_of_feature++;

  // Age other features and zero the age of this one, if necessary
  if (sfp->get_age () != 0) {
    age_all_features (sfp);
    sfp->zero_age ();
  }

  return 0;
}


/* Increment the age of all features younger than given feature
 */

void Scene_Single::age_all_features (Feature *fend)
{
  int stop_age = fend->get_age ();
  if (stop_age == 0) return;

  // If there are other features the same age, we can happily age all features
  for (Feature *f = first_feature_ptr; f; f = f->next) {
    if (f != fend && f->get_age () == stop_age) {
      stop_age = -1;
      break;
    }
  }

  // Now age features to fill gap left by one feature's age going to zero
  // If there are other features the same age, no gap was left, so age all
  for (Feature *f = first_feature_ptr; f; f = f->next) {
    if (f->get_age () == -1) f->zero_age ();
    if (f->get_age () < stop_age || stop_age == -1) f->increment_age ();
  }
}


/***********Fill passed matrices with stuff needed to do the update***********/

int Scene_Single::construct_total_measurement_stuff(Hor_Matrix *nu_tot, 
					     Hor_Matrix *dh_by_dx_tot,
					     Hor_Matrix *R_tot)
{
  int size = successful_measurement_vector_size;

  assert (nu_tot->rows == size && nu_tot->cols == 1 &&
	  dh_by_dx_tot->rows == size && 
	  dh_by_dx_tot->cols == total_state_size &&
	  R_tot->rows == size && R_tot->cols == size);

  hor_matq_zero(nu_tot);
  hor_matq_zero(dh_by_dx_tot);
  hor_matq_zero(R_tot);

  int vector_position = 0;

  for (Feature *sfp = first_selected_ptr; sfp; sfp = sfp->next_selected)
  {
    if (sfp->successful_measurement_flag)
    {
      hor_matq_insert_chunky1(sfp->nu, nu_tot, vector_position);

      hor_matq_insert_chunkyx(sfp->dh_by_dxv, dh_by_dx_tot, 
			      vector_position, 0);
      hor_matq_insert_chunkyx(sfp->dh_by_dy, dh_by_dx_tot,
			      vector_position, 
			      sfp->position_in_total_state_vector);

      hor_matq_insert_chunkyx(sfp->R, R_tot, vector_position, vector_position);

      vector_position += sfp->feature_measurement_model->MEASUREMENT_SIZE;
    }
  }

  return 0;
}

/**************************Print Out Current State****************************/

int Scene_Single::print_robot_state()
{
  cout << "Robot state" << xv;
  cout << "Robot covariance" << Pxx;

  return 0;
}

int Scene_Single::print_marked_feature_state()
{
  if (marked_feature_label == -1)
  {
    cerr << "No feature marked to delete." << endl;
    return -1;
  }

  Feature *f_ptr, *prev_ptr = NULL;
  
  for(f_ptr = first_feature_ptr; f_ptr; f_ptr = f_ptr->next)
  {
    if (f_ptr->label == marked_feature_label)
      break;

    prev_ptr = f_ptr;
  }

  if (f_ptr == NULL)
  {
    cerr << "Error: marked feature not found in list." << endl;
    return -1;
  }

  cout << "Feature identifier: " << f_ptr->identifier << ". Label: "
         << f_ptr->label << ". List position: "
	 << f_ptr->position_in_list << "." << endl;

  cout << "yi" << f_ptr->y;
  cout << "Pyiyi" << f_ptr->Pyy;

  return 0;
}

int Scene_Single::print_whole_state()
{
  cout << "Robot state" << xv;

  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    cout << "Feature identifier: " << f->identifier << ". Label: "
         << f->label << ". List position: "
	 << f->position_in_list << "." << endl;

    cout << "yi" << f->y;
  }

  print_covariances();

  return 0;
}

/************************Output Current State to File*************************/

/* Format: output_counter (label)
           no_features
	   state vector
	   covariance matrix */

/* Most general function is called by overloaded functions below with
   fewer arguments. */
int Scene_Single::output_state_to_file(int counter, char *filename)
{
  ofstream outfile(filename, ios::app);

  Hor_Matrix *x, *P;

  int size = total_state_size;
  x = hor_mat_alloc(size, 1);
  P = hor_mat_alloc(size, size);

  construct_total_state_and_covariance(x, P);

  outfile << counter << endl
          << no_features << endl
          << x << P;

  cout << "State for step " << counter 
       << " output to file " << filename << "." << endl;

  hor_mat_free_list(x, P, NULL);

  return 0;
}

int Scene_Single::output_state_to_file(int counter)
{
  // Make the output filename depend on what features are selected
  // Can't quite remember why I wanted to do this...?
  char name[100];
  char extra[2];
  strcpy (name, "state_");
  
  extra[1] = '\0';

  for (Feature *sfp = first_selected_ptr; sfp; sfp = sfp->next_selected)
  {
    if (sfp->position_in_list > 9)
    {
      cout << "Problem with filename: too many features." << endl;
      break;
    }
    extra[0] = '0' + sfp->position_in_list;
    strcat(name, extra);
  }

  output_state_to_file(counter, name);

  return 0;
}

/* A call with no arguments uses the internal output counter. */

int Scene_Single::output_state_to_file()
{
  output_state_to_file(output_counter, "state");

  output_counter++;

  return 0;
}

/************************Predict Feature Measurements*************************/

/* For a selected feature, work out:

   h: the predicted measurement 
   dh_by_dx
   dh_by_dy : the components of the Jacobian with respect to the vehicle
              position and the feature's position
*/

int Scene_Single::predict_single_feature_measurements(Feature *sfp)
{
  motion_model->func_xp(xv);
  sfp->feature_measurement_model->func_hi_and_dhi_by_dxp_and_dhi_by_dyi(sfp->y,
						       	motion_model->xpRES);  

  hor_matq_copy(sfp->feature_measurement_model->hiRES, sfp->h);
  hor_matq_copy(sfp->feature_measurement_model->dhi_by_dyiRES, sfp->dh_by_dy);

  motion_model->func_dxp_by_dxv(xv);
  
  hor_matq_prod2(sfp->feature_measurement_model->dhi_by_dxpRES,
		 motion_model->dxp_by_dxvRES, 
		 sfp->dh_by_dxv);

  sfp->feature_measurement_model->func_Ri(sfp->h);  
  hor_matq_copy(sfp->feature_measurement_model->RiRES, sfp->R);

  sfp->feature_measurement_model->func_Si(Pxx, sfp->Pxy, sfp->Pyy, 
					sfp->dh_by_dxv, sfp->dh_by_dy, sfp->R);
  hor_matq_copy(sfp->feature_measurement_model->SiRES, sfp->S);

  return 0;
}

// Similar to above but places various results in provided matrices.
// Must be provided only with feature position/state (y).
// Cannot calculate predicted innovation covariance S.

int Scene_Single::predict_single_feature_measurements (Feature_Measurement_Model *model,
						       Hor_Matrix *y,
						       Hor_Matrix *xp, Hor_Matrix *h,
						       Hor_Matrix *dh_by_dy,
						       Hor_Matrix *dh_by_dxv,
						       Hor_Matrix *R)
{
  assert (model != NULL && y != NULL && xp != NULL && h != NULL);
  assert (dh_by_dy != NULL && dh_by_dxv != NULL && R != NULL);

  motion_model->func_xp (xv);
  hor_matq_copy (motion_model->xpRES, xp);
  assert (hor_errno == HORATIO_OK);   // These asserts check input mat sizes correct

  model->func_hi_and_dhi_by_dxp_and_dhi_by_dyi (y, xp);
  hor_matq_copy(model->hiRES, h);
  hor_matq_copy(model->dhi_by_dyiRES, dh_by_dy);

  motion_model->func_dxp_by_dxv (xv);
  hor_matq_prod2 (model->dhi_by_dxpRES, motion_model->dxp_by_dxvRES, dh_by_dxv);
  assert (hor_errno == HORATIO_OK);

  model->func_Ri (h);  
  hor_matq_copy (model->RiRES, R);
  assert (hor_errno == HORATIO_OK);

  return 0;
}

/****************************Predict Measurements*****************************/

int Scene_Single::predict_measurements()
{
  for (Feature *sfp = first_selected_ptr; sfp; sfp = sfp->next_selected)
    predict_single_feature_measurements(sfp);

  return 0;
}

/***********************Test a feature for measurability**********************/

int Scene_Single::test_for_visibility(Feature *fp)
{
  predict_single_feature_measurements(fp);
  motion_model->func_xp(xv);
  
  return fp->feature_measurement_model->visibility_test(motion_model->xpRES, 
							fp->y, 
							fp->xp_orig, fp->h);
}

/****************************Internal Measurements****************************/

int Scene_Single::predict_internal_measurement()
{
  assert(internal_measurement_model != NULL);

  internal_measurement_model->func_hv_and_dhv_by_dxv(xv);

  hor_matq_copy(internal_measurement_model->hvRES, hv);
  hor_matq_copy(internal_measurement_model->dhv_by_dxvRES,
		dhv_by_dxv);

  internal_measurement_model->func_Rv(hv);
  hor_matq_copy(internal_measurement_model->RvRES, Rv);

  internal_measurement_model->func_Sv(Pxx, dhv_by_dxv, Rv);
  hor_matq_copy(internal_measurement_model->SvRES, Sv);

  // cout << "hv" << hv << "Rv" << Rv << "Sv" << Sv;
  
  return 0;
}


int Scene_Single::construct_total_internal_measurement_stuff(Hor_Matrix *nu_tot, 
					       Hor_Matrix *dh_by_dx_tot,
					       Hor_Matrix *R_tot)
{
  int size = internal_measurement_model->MEASUREMENT_SIZE;

  assert (nu_tot->rows == size && nu_tot->cols == 1 &&
	  dh_by_dx_tot->rows == size && 
	  dh_by_dx_tot->cols == total_state_size &&
	  R_tot->rows == size && R_tot->cols == size);

  if (!successful_internal_measurement_flag)
  {
    cerr << "Error with internal measurements: updating after failed attempt."
	 << endl;
    return -1;
  }

  hor_matq_zero(nu_tot);
  hor_matq_zero(dh_by_dx_tot);
  hor_matq_zero(R_tot);

  hor_matq_insert_chunky1(nuv, nu_tot, 0);

  hor_matq_insert_chunkyx(dhv_by_dxv, dh_by_dx_tot, 0, 0);

  hor_matq_insert_chunkyx(Rv, R_tot, 0, 0);

  return 0;
}

int Scene_Single::successful_internal_measurement()
{
  successful_internal_measurement_flag = 1;

  internal_measurement_model->func_nuv(hv, zv);
  hor_matq_copy(internal_measurement_model->nuvRES, nuv);

  return 0;
}

int Scene_Single::failed_internal_measurement()
{
  successful_internal_measurement_flag = 0;

  return 0;
}

int Scene_Single::zero_axes()
{
  int size = get_total_state_size();
  Hor_Matrix *x = hor_mats_zero(size, 1);
  Hor_Matrix *dxnew_by_dxold = hor_mats_zero(size, size);

  // We form the new state and Jacobian of the new state w.r.t. the old state
  int state_position = 0;
  
  // Robot part
  motion_model->func_zeroedxv_and_dzeroedxv_by_dxv(get_xv());
  hor_matq_insert_chunky1(motion_model->zeroedxvRES, x, 0);
  hor_matq_insert_chunkyx(motion_model->dzeroedxv_by_dxvRES, dxnew_by_dxold,
			  0, 0);

  state_position += motion_model->STATE_SIZE;

  // Each feature in turn
  int feature_no = 0;

  // Copy these to be on the safe side
  motion_model->func_xp(xv);
  Hor_Matrix *local_xp = hor_mats_copy(motion_model->xpRES);
  motion_model->func_dxp_by_dxv(xv);
  Hor_Matrix *local_dxp_by_dxv = hor_mats_copy(motion_model->dxp_by_dxvRES);
  

  for (Feature *f = first_feature_ptr; f; f = f->next)
  {
    f->feature_measurement_model->
      func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(f->get_y(), 
                                                    local_xp);
    hor_matq_insert_chunky1(f->feature_measurement_model->zeroedyiRES,
			    x, state_position);

    // Calculate dzeroedyi_by_dxv in Temp_FS1
    hor_matq_prod2(f->feature_measurement_model->dzeroedyi_by_dxpRES,
		   local_dxp_by_dxv, 
		   f->feature_measurement_model->Temp_FS1);

    hor_matq_insert_chunkyx(f->feature_measurement_model->Temp_FS1,
			    dxnew_by_dxold, state_position, 0);
    hor_matq_insert_chunkyx(f->feature_measurement_model->dzeroedyi_by_dyiRES,
			    dxnew_by_dxold, state_position, state_position);
   
    feature_no++;
    state_position += f->feature_measurement_model->FEATURE_STATE_SIZE;
  }

  // Check we've counted properly
  assert (feature_no == no_features && state_position == total_state_size);

  // Do a Jacobian transform to get the new covariance
  Hor_Matrix *P = hor_mat_alloc(size, size);
  construct_total_covariance(P);

  Hor_Matrix *Tempss = hor_mat_alloc(size, size);

  Hor_Matrix *dxnew_by_dxoldT = hor_mat_alloc(size, size);
  hor_matq_transpose(dxnew_by_dxold, dxnew_by_dxoldT);
  hor_matq_prod3(dxnew_by_dxold, P, dxnew_by_dxoldT, Tempss, P);

  // Finally load the scene data structures with the new state and covariance
  fill_states(x);
  fill_covariances(P);

  hor_mat_free_list(x, P, Tempss, dxnew_by_dxold, dxnew_by_dxoldT, 
		    local_xp, local_dxp_by_dxv, NULL);

  return 0;
}

int Scene_Single::normalise_state()
{
  cout << "Normalising state." << endl;

  /* Normalising state:

     This deals with the case where the robot state needs normalising
     (e.g. if it contains a quaternion)

     We assume the feature states do not need normalising
  */

  int size = get_total_state_size();

  // First form original total state and covariance
  Hor_Matrix *x = hor_mat_alloc(size, 1);
  Hor_Matrix *P = hor_mat_alloc(size, size);
  construct_total_state_and_covariance(x, P);


  // Make model calculations: store results in RES matrices
  get_motion_model()->func_xvnorm_and_dxvnorm_by_dxv(get_xv());

  // Find new state xnorm
  Hor_Matrix *xnorm = hor_mat_alloc(size, 1);

  /* Feature elements of xnorm are the same as x */
  hor_matq_copy(x, xnorm);
  hor_matq_insert_chunky1(get_motion_model()->xvnormRES, xnorm, 0);


  // Find new P
  Hor_Matrix *Pnorm = hor_mat_alloc(size, size);

  /* Since most elements of dxnorm_by_dx are zero... */
  Hor_Matrix *dxnorm_by_dx = hor_mats_zero(size, size);

  // Fill the rest of the elements of dxnorm_by_dx: 1 on diagonal for features
  hor_matq_identity(dxnorm_by_dx);

  hor_matq_insert_chunkyx(get_motion_model()->dxvnorm_by_dxvRES, 
			  dxnorm_by_dx, 0, 0);

  hor_matq_ABAT(dxnorm_by_dx, P, Pnorm);

  // cerr << "xnorm:" << f;
  // cerr << "Pnorm:" << Pnorm;

  fill_state_and_covariance(xnorm, Pnorm);
  vehicle_state_has_been_changed();

  hor_mat_free_list(x, P, xnorm, Pnorm, dxnorm_by_dx, NULL);

  return 0;
}
