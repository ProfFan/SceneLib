/*  Scene: software for sequential localisation and map-building

    feature.cc
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

/************************Matrix_Block Constructor*****************************/

Matrix_Block::Matrix_Block(Hor_Matrix *mat, int px, int py)
  :  sizex(mat->cols), sizey(mat->rows)
{
  m = mat;
  posx = px;
  posy = py;

  m_new = hor_mat_alloc (sizey, sizex);

  // cout << "Creating block at x = " << posx << ", y = " << posy << 
  //     ", sizex = " << sizex << ", sizey = " << sizey << ".\n";
}

/************************Matrix_Block Destructor******************************/

Matrix_Block::~Matrix_Block()
{
  // cout << "Removing block at x = " << posx << ", y = " << posy << ".\n";

  hor_mat_free (m);
  hor_mat_free (m_new);
}

/****************************Feature Constructors*****************************/

/* Function which unites common stuff for the two constructors
   below */
int Feature::feature_constructor_bookeeping()
{
  next = NULL;
  next_selected = NULL;
  selected_flag = 1;        // Feature is selected when first detected
  scheduled_for_termination_flag = 0;
  attempted_measurements_of_feature = 0;
  successful_measurements_of_feature = 0;
  age = -1;    // -1 means not yet measured whereas 0 means just measured
  jghk_ignore = false;

  // Allocate matrices for storing predicted and actual measurements
  h = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 1);
  hinit = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 1);
  z = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 1);
  nu = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 1);
  dh_by_dxv = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 
		    feature_measurement_model->motion_model->STATE_SIZE);
  dh_by_dy = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE, 
			   feature_measurement_model->FEATURE_STATE_SIZE);
  R = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE,
		    feature_measurement_model->MEASUREMENT_SIZE);
  S = hor_mat_alloc(feature_measurement_model->MEASUREMENT_SIZE,
		    feature_measurement_model->MEASUREMENT_SIZE);
  Pyy_new = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
			  feature_measurement_model->FEATURE_STATE_SIZE);

  return 0;
}

/* For adding a normal, uncertain feature */
Feature::Feature(Identifier id, int lab, int list_pos, 
		Scene_Single *scene,  Hor_Matrix *h, Feature_Measurement_Model *f_m_m)
  : feature_measurement_model(f_m_m)
{
  feature_constructor_bookeeping();
  hor_matq_copy (h, hinit);
  assert (hor_errno == HORATIO_OK);

  identifier = id;
  label = lab;
  position_in_list = list_pos;   // Position of new feature in list
  position_in_total_state_vector = -1; /* This should be set in 
					  fill_matrix_pointer_array 
					  when feature is added */

  // If this is the first feature in the map
  if (position_in_list == 0)
  {
    first_mbptr = NULL;
    last_mbptr = NULL;
  }

  /* Save the vehicle position where this feature was acquired */
  scene->get_motion_model()->func_xp(scene->get_xv());
  xp_orig = hor_mats_copy(scene->get_motion_model()->xpRES);

  // Call model functions to calculate feature state, measurement noise
  // and associated Jacobians. Results are stored in RES matrices 

  // First calculate "position state" and Jacobian
  scene->get_motion_model()->func_xp(scene->get_xv());
  scene->get_motion_model()->func_dxp_by_dxv(scene->get_xv());

  // Now calculate meat + 2 veg from model
  feature_measurement_model->func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(h, 
			       scene->get_motion_model()->xpRES);

  // State y
  y = hor_mats_copy(feature_measurement_model->yiRES);

  // Temp_FS1 will store dyi_by_dxv
  hor_matq_prod2(feature_measurement_model->dyi_by_dxpRES, 
		 scene->get_motion_model()->dxp_by_dxvRES,
		 feature_measurement_model->Temp_FS1);

  // Pxy
  Pxy = hor_mat_alloc(scene->get_motion_model()->STATE_SIZE, 
		      feature_measurement_model->FEATURE_STATE_SIZE);
  hor_matq_ABT(scene->get_Pxx(), 
	       feature_measurement_model->Temp_FS1, Pxy);
  Pxy_new = hor_mat_alloc(scene->get_motion_model()->STATE_SIZE, 
			  feature_measurement_model->FEATURE_STATE_SIZE);

  // Pyy
  Pyy = hor_mat_alloc(feature_measurement_model->FEATURE_STATE_SIZE,
		      feature_measurement_model->FEATURE_STATE_SIZE);
  hor_matq_ABAT(feature_measurement_model->Temp_FS1, 
		scene->get_Pxx(), 
		feature_measurement_model->Temp_FF1);
  hor_matq_ABAT(feature_measurement_model->dyi_by_dhiRES, 
		feature_measurement_model->RiRES, 
		feature_measurement_model->Temp_FF2);
  hor_matq_add2(feature_measurement_model->Temp_FF1, 
		feature_measurement_model->Temp_FF2,
		Pyy);

  // Covariances of this feature and others
  Hor_Matrix *hmp;
  Matrix_Block *mbp;
  Feature *fp = scene->get_first_feature_ptr();

  for (int i = 0; i < position_in_list; i++)
  {
    // dyi_by_dxv . Pxyj -> Temp_FF1
    hor_matq_prod2 (feature_measurement_model->Temp_FS1, fp->get_Pxy(), 
		    feature_measurement_model->Temp_FF1);
    // hmp now points to Pyjx . dyi_by_dxvT
    hmp = hor_mats_transpose (feature_measurement_model->Temp_FF1);

    mbp = new Matrix_Block (hmp, position_in_list, i);
    if (i == 0)
      first_mbptr = mbp;
    else
      last_mbptr->next = mbp;
    last_mbptr = mbp;
    last_mbptr->next = NULL;

    fp = fp->next;
  }  

  known_feature_label = -1;
}

/* Alternative constructor for known features: different no. of 
   arguments differentiates it */
Feature::Feature(Identifier id, int lab, int list_pos, 
	           Scene_Single *scene, Hor_Matrix *y_known, Hor_Matrix *xp_o, 
                   Feature_Measurement_Model *f_m_m, int k_f_l)
  : feature_measurement_model(f_m_m)
{
  feature_constructor_bookeeping();

  identifier = id;
  label = lab;
  position_in_list = list_pos;   // Position of new feature in list

  // If this is the first feature in the map
  if (position_in_list == 0)
  {
    first_mbptr = NULL;
    last_mbptr = NULL;
  }

  /* Save the vehicle position where this feature was acquired */
  xp_orig = hor_mats_copy(xp_o);

  // Straighforward initialisation of state and covariances
  y = hor_mats_copy(y_known);
  Pxy = hor_mats_zero(scene->get_motion_model()->STATE_SIZE, 
		      feature_measurement_model->FEATURE_STATE_SIZE);
  Pxy_new = hor_mats_zero(scene->get_motion_model()->STATE_SIZE, 
			  feature_measurement_model->FEATURE_STATE_SIZE);
  Pyy = hor_mats_zero(feature_measurement_model->FEATURE_STATE_SIZE,
		      feature_measurement_model->FEATURE_STATE_SIZE);

  Hor_Matrix *hmp;
  Matrix_Block *mbp;
  Feature *fp = scene->get_first_feature_ptr();

  for (int i = 0; i < position_in_list; i++)
  {
    hmp = hor_mats_zero(feature_measurement_model->FEATURE_STATE_SIZE,
			feature_measurement_model->FEATURE_STATE_SIZE);
    mbp = new Matrix_Block(hmp, position_in_list, i);
    if (i == 0)
      first_mbptr = mbp;
    else
      last_mbptr->next = mbp;
    last_mbptr = mbp;
    last_mbptr->next = NULL;

    fp = fp->next;
  }    

  known_feature_label = k_f_l;
}


/* This is a special function for use when swapping different state and
 * covariances in and out of the Scene object.  If, while one state and
 * covariance was in situ, this feature was added, then it was initialised
 * with values that may be wrong for the new state and covariance.
 * Reinitialise assuming z has not been changed and recalculates initial
 * values for the feature state and covariance.  JGHK 10/00
 */

int Feature::reinitialise (Scene_Single *scene)
{
  // Set measurement h as first measurement hinit;
  Hor_Matrix *h = hinit;

  /* Save the vehicle position where this feature was acquired */
  scene->get_motion_model()->func_xp (scene->get_xv());
  hor_matq_copy (scene->get_motion_model()->xpRES, xp_orig);

  // Call model functions to calculate feature state, measurement noise
  // and associated Jacobians. Results are stored in RES matrices 

  // First calculate "position state" and Jacobian
  scene->get_motion_model()->func_xp (scene->get_xv());
  scene->get_motion_model()->func_dxp_by_dxv (scene->get_xv());

  // Now calculate meat + 2 veg from model
  feature_measurement_model->func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri
    (h, scene->get_motion_model()->xpRES);

  // State y
  hor_matq_copy (feature_measurement_model->yiRES, y);

  // Temp_FS1 will store dyi_by_dxv
  hor_matq_prod2 (feature_measurement_model->dyi_by_dxpRES, 
		  scene->get_motion_model()->dxp_by_dxvRES,
		  feature_measurement_model->Temp_FS1);

  // Pxy
  hor_matq_ABT (scene->get_Pxx(), 
		feature_measurement_model->Temp_FS1, Pxy);

  // Pyy
  hor_matq_ABAT (feature_measurement_model->Temp_FS1, 
		 scene->get_Pxx(), 
		 feature_measurement_model->Temp_FF1);
  hor_matq_ABAT (feature_measurement_model->dyi_by_dhiRES, 
		 feature_measurement_model->RiRES, 
		 feature_measurement_model->Temp_FF2);
  hor_matq_add2 (feature_measurement_model->Temp_FF1, 
		 feature_measurement_model->Temp_FF2,
		 Pyy);

  // Covariances of this feature and others
  Feature *fp = scene->get_first_feature_ptr();
  for (Matrix_Block *mbp = first_mbptr; mbp; mbp = mbp->next, fp = fp->next) {

    // dyi_by_dxv . Pxyj -> Temp_FF1
    hor_matq_prod2 (feature_measurement_model->Temp_FS1, fp->get_Pxy(), 
		    feature_measurement_model->Temp_FF1);
    // mbp->m now points to Pyjx . dyi_by_dxvT
    hor_matq_transpose (feature_measurement_model->Temp_FF1, mbp->m);
  }

  return 0;
}

/*****************************Feature Destructor******************************/

Feature::~Feature()
{
  Matrix_Block *mbptr, *next_mbptr;   /* Need to do this in a slightly weird
					 way because we can't get next
					 from a block which has been deleted */

  for (mbptr = first_mbptr; mbptr; mbptr = next_mbptr)
  {
    next_mbptr = mbptr->next;
    delete mbptr;
  }
  if (mbptr != NULL)
    cerr << "Problems deleting matrix blocks." << endl;

  hor_mat_free_list(y, Pyy, Pxy, Pyy_new, Pxy_new, xp_orig, h, z, nu,
		    dh_by_dxv, dh_by_dy, NULL);
}

/*******************Remove matrix blocks for deleted feature******************/

int Feature::feature_is_removed(int list_pos)
{
  Matrix_Block *mbptr, *prev_mbptr = NULL, *next_mbptr = NULL;

  position_in_list--;

  for (mbptr = first_mbptr; mbptr; mbptr = mbptr->next)
  {
    if (mbptr->posy == list_pos)
    {
      if (mbptr == first_mbptr)
      {
	first_mbptr = mbptr->next;
      }
      else
      {
	prev_mbptr->next = mbptr->next;
      }
      
      if (mbptr == last_mbptr)
      {
	last_mbptr = prev_mbptr;
      }

      next_mbptr = mbptr->next;
      delete mbptr;

      /* Adjust the posx, posy numbers of the remaining blocks */
      for (mbptr = next_mbptr; mbptr; mbptr = mbptr->next)
	mbptr->decrement_posx_posy();

      return 0;
    }

    mbptr->decrement_posx();
    prev_mbptr = mbptr;
  }

  cout << "Problem in feature_is_removed()." << endl;

  return -1;
}


/* Copy mbptr->m_new, Pyy_new and Pxy_new to mbptr->m, Pyy and Pxy
 */

int Feature::bring_covariances_up_to_date ()
{
  for (Matrix_Block *mbptr = first_mbptr; mbptr; mbptr = mbptr->next) {
    hor_matq_copy (mbptr->m_new, mbptr->m);
  }
  hor_matq_copy (Pyy_new, Pyy);
  hor_matq_copy (Pxy_new, Pxy);
  assert (hor_errno == HORATIO_OK);

  return 0;
}


/* Similarly the other way round
 */

int Feature::copy_covariances_to_new ()
{
  for (Matrix_Block *mbptr = first_mbptr; mbptr; mbptr = mbptr->next) {
    hor_matq_copy (mbptr->m, mbptr->m_new);
  }
  hor_matq_copy (Pyy, Pyy_new);
  hor_matq_copy (Pxy, Pxy_new);
  assert (hor_errno == HORATIO_OK);

  return 0;
}
