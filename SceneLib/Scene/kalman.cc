/*  Scene: software for sequential localisation and map-building

    kalman.cc
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Substantial additions from Joss Knight
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

#include "sim_or_rob.h"
#include "3d.h"

#include "kalman.h"

/*****************************Kalman Constructor******************************/

Kalman::Kalman () :
  filter1 (SLOW),
  filter2 (SLOW),
  maintain_separate_state (false),
  switch_filters1and2 (false),
  x_saved (NULL),
  P_saved (NULL)
{
}


Kalman::Kalman (bool maintain_separate_state_in, Scene *start_scene) :
  filter1 (SLOW),
  filter2 (SLOW),
  maintain_separate_state (maintain_separate_state_in),
  switch_filters1and2 (false),
  x_saved (NULL),
  P_saved (NULL)
{
  assert (start_scene != NULL);

  initialise_separate_state (start_scene);
}


/*************************Predict New Vehicle Position************************/

int Kalman::predict_filter (Scene *scene, Hor_Matrix *u, double delta_t)
{
  assert (scene != NULL && u != NULL && delta_t > 0.0);

  int out1 = predict_filter_choose (scene, u, delta_t, filter1, filter1_params);
  int out2 = 0;
  if (maintain_separate_state) {
    switch_saved_for_current_state (scene);
    out2 = predict_filter_choose (scene, u, delta_t, filter2, filter2_params);
    switch_saved_for_current_state (scene);
  }

  return out1 != 0 ? out1 : out2;
}


int Kalman::predict_filter_choose (Scene *scene, Hor_Matrix *u, double delta_t,
				   filter_method how_to_filter, filter_params& params)
{
  assert (scene != NULL && u != NULL && delta_t > 0.0);

  // Timer
  clock_t time = clock ();

  int out;
  switch (how_to_filter) {
  case SLOW:
    out = predict_filter_slow (scene, u, delta_t);
    break;

  case FAST_FULL:
  case JGHK:
  case NEBOT:
    out = predict_filter_fast (scene, u, delta_t, how_to_filter, params);
    break;

  default:
    bool SHOULDNT_REACH_HERE = false;
    assert (SHOULDNT_REACH_HERE);
    return 1;
  }

  params.filter_time = double (clock () - time) / double (CLOCKS_PER_SEC);
  params.total_filter_time += params.filter_time;

  return out;
}
    
/* Simple overall prediction */
int Kalman::predict_filter_slow (Scene *scene, Hor_Matrix *u, double delta_t)
{
  cerr << "*** SLOW PREDICTION ***\n";

  /* What we need to do for the prediction:
     
     Calculate f and grad_f_x
     Calculate Q
     Form x(k+1|k) and P(k+1|k)
  */

  int size = scene->get_total_state_size();

  // First form original total state and covariance
  Hor_Matrix *x = hor_mat_alloc(size, 1);
  Hor_Matrix *P = hor_mat_alloc(size, size);
  scene->construct_total_state_and_covariance(x, P);


  // Make model calculations: store results in RES matrices
  // scene->get_motion_model()->jacobians_test(scene->get_xv(), u, delta_t);
  scene->get_motion_model()->func_fv_and_dfv_by_dxv(scene->get_xv(), 
						    u, delta_t);
  scene->get_motion_model()->func_Q(scene->get_xv(), u, delta_t);


  // Find new state f
  Hor_Matrix *f = hor_mat_alloc(size, 1);

  /* Feature elements of f are the same as x */
  hor_matq_copy(x, f);
  hor_matq_insert_chunky1(scene->get_motion_model()->fvRES, f, 0);


  // Find new P
  Hor_Matrix *Pnew = hor_mat_alloc(size, size);

  /* Since most elements of df_by_dx are zero... */
  Hor_Matrix *df_by_dx = hor_mats_zero(size, size);

  // Fill the rest of the elements of df_by_dx: 1 on diagonal for features
  for (int i = scene->get_motion_model()->STATE_SIZE; i < df_by_dx->rows; i++)
    df_by_dx->m[i][i] = 1.0;

  hor_matq_insert_chunkyx(scene->get_motion_model()->dfv_by_dxvRES, 
			  df_by_dx, 0, 0);

  // Calculate the process noise
  Hor_Matrix *Q = hor_mats_zero(size, size);
  hor_matq_insert_chunkyx(scene->get_motion_model()->QxRES, Q, 0, 0);

  hor_matq_ABAT(df_by_dx, P, Pnew);

  hor_matq_add2(Q, Pnew, Pnew);

  // cerr << "f:" << f;
  // cerr << "Pnew:" << Pnew;

  scene->fill_state_and_covariance(f, Pnew);
  scene->vehicle_state_has_been_changed();

  hor_mat_free_list(x, P, f, Pnew, df_by_dx, Q, NULL);

  return 0;
}


/*************** Fast version of predict filter *******************/

/* Doesn't mess around with big covariance matrices.  Does exactly the
 * same thing though.
 * Added by jghk 17/9/00, generalised from old ajd code.
 */

int Kalman::predict_filter_fast(Scene *scene, Hor_Matrix *u, double delta_t,
				filter_method how_to_filter, filter_params& params)
{
  if (how_to_filter == FAST_FULL) cerr << "*** FAST PREDICTION ***\n";
  else if (how_to_filter == JGHK) cerr << "*** JGHK PREDICTION ***\n";
  else if (how_to_filter == NEBOT) cerr << "*** NEBOT PREDICTION ***\n";
  //  scene->print_whole_state ();

  // Set up static matrices
  static Hor_Matrix *Temp_vbyvA = NULL;
  static Hor_Matrix *Temp_vbyvB = NULL;
  static Hor_Matrix *Temp_vbyf = NULL;
  int vstate_size = scene->get_motion_model()->STATE_SIZE;
  hor_mat_ensure_size (vstate_size, vstate_size, &Temp_vbyvA);
  hor_mat_ensure_size (vstate_size, vstate_size, &Temp_vbyvB);

  // Make model calculations: store results in RES matrices
  scene->get_motion_model()->func_fv_and_dfv_by_dxv(scene->get_xv(), 
						    u, delta_t);
  scene->get_motion_model()->func_Q(scene->get_xv(), u, delta_t);

  hor_matq_ABAT (scene->get_motion_model()->dfv_by_dxvRES, scene->get_Pxx (),
		 Temp_vbyvA, Temp_vbyvB);
  assert (hor_print_error () == HORATIO_OK);

  /* Change the state vector */
  hor_matq_copy (scene->get_motion_model()->fvRES, scene->get_xv ());

  /* Change the vehicle state covariance */
  hor_matq_add2 (Temp_vbyvB, scene->get_motion_model()->QxRES, scene->get_Pxx ());
  
  /* Change the covariances between vehicle state and feature states */
  for (Feature *f = scene->get_first_feature_ptr (); f; f = f->next)
  {
    //    if (how_to_filter == JGHK) {
    //      if (f->get_age () > params.jghk_age_limit) continue;
    //    }

    int fsize = f->get_feature_measurement_model()->FEATURE_STATE_SIZE;
    hor_mat_ensure_size (vstate_size, fsize, &Temp_vbyf);

    hor_matq_prod2 (scene->get_motion_model()->dfv_by_dxvRES, f->Pxy, Temp_vbyf);
    hor_matq_copy (Temp_vbyf, f->Pxy);
  }

  scene->vehicle_state_has_been_changed ();

  //  scene->print_whole_state ();

  return 0;
}


/********************************Update Filter********************************/

int Kalman::total_update_filter (Scene *scene)
{
  assert (scene != NULL);

  int out1 = total_update_filter_choose (scene, filter1, filter1_params);
  int out2 = 0;
  if (maintain_separate_state) {
    switch_saved_for_current_state (scene);
    out2 = total_update_filter_choose (scene, filter2, filter2_params);
    switch_saved_for_current_state (scene);
  }

  return out1 != 0 ? out1 : out2;
}


int Kalman::total_update_filter_choose (Scene *scene, filter_method how_to_filter,
					filter_params& params)
{
  assert (scene != NULL);

  // Performance info stuff
  clock_t time = clock ();
  params.features_retained = scene->get_no_features ();

  int out;
  switch (how_to_filter) {
  case SLOW:
    out = total_update_filter_slow (scene);
    break;

  case FAST_FULL:
  case NEBOT:
    out = total_update_filter_fast (scene, how_to_filter, params);
    break;

  case JGHK:
    reincorporate_features (scene, params);
    out = total_update_filter_fast (scene, how_to_filter, params);
    break;

  default:
    bool SHOULDNT_REACH_HERE = false;
    assert (SHOULDNT_REACH_HERE);
    return 1;
  }

  double update_time = double (clock () - time) / double (CLOCKS_PER_SEC);
  params.filter_time += update_time;
  params.total_filter_time += update_time;

  return out;
}


/* Update the filter in a simple overall way */
int Kalman::total_update_filter_slow(Scene *scene)
{
  Hor_Matrix *x = NULL;
  Hor_Matrix *P = NULL;

  update_filter_slow_separate (scene, &x, &P);

  scene->fill_state_and_covariance(x, P);
  scene->vehicle_state_has_been_changed();

  // cout << "x after update:" << x;
  // cout << "P after update:" << P;

  hor_mat_free_list (x, P, NULL);

  return 0;
}

/************************Fast Version of Update Filter************************/

// Reimplemented in generalised form from old AJD code by JGHK 18/9/00
/*
   What we need to do:

   Loop over the measurement elements {
     Work out S and S^-1
     Calculate A to D
     Update vehicle state xv
     Update vehicle covariance Pxx
     Loop over each feature k {
       Update feature state yk
       Update vehicle to feature covariance Pxyk
       Update feature covariance Pykyk
       Loop over each previous features j {
         Update feature to feature cross-covariance Pyjyk
       }
     }
   }

   */

int Kalman::total_update_filter_fast (Scene *scene, 
				      filter_method how_to_filter,
				      filter_params &params)
{
  assert (hor_print_error () == HORATIO_OK);

  if (how_to_filter == FAST_FULL) cerr << "*** FAST UPDATE ***\n";
  else if (how_to_filter == JGHK) cerr << "*** JGHK UPDATE ***\n";
  else if (how_to_filter == NEBOT) cerr << "*** NEBOT UPDATE ***\n";

  // Do slow update to locate bugs
  /*
  Hor_Matrix *true_x = NULL;
  Hor_Matrix *true_P = NULL;
  update_filter_slow_separate (scene, &true_x, &true_P);
  */


  // Initialisations
  int vsize = scene->get_motion_model()->STATE_SIZE;

  // In general, it's not a good idea to assume that the temporary `new'
  // covariance matrix is the same as the old one at the start (it certainly
  // won't be if we're swapping between two separate updates).  This can be
  // removed when `streamlining' less general code.
  copy_covariances_to_new (scene);

  /* Loop over selected features */
  for (Feature *sf = scene->get_first_selected_ptr (); sf; sf = sf->next_selected) {
    if (sf->successful_measurement_flag) {

      //      scene->print_whole_state ();

      // If there are more than one observed feature, we need to re-predict the
      // measurement after each separate update
      if (sf != scene->get_first_selected_ptr ()) {
	scene->predict_single_feature_measurements (sf);
      }

      // Initialisations for this feature
      int msize = sf->get_feature_measurement_model()->MEASUREMENT_SIZE;
      int sfsize = sf->get_feature_measurement_model()->FEATURE_STATE_SIZE;

      // Set up static matrices
      static Hor_Matrix *S = NULL;
      static Hor_Matrix *Sinv = NULL;
      static Hor_Matrix *Sinvt = NULL;
      static Hor_Matrix *Temp_mbyv = NULL;
      static Hor_Matrix *Temp_mbysf = NULL;
      static Hor_Matrix *Temp_vbym = NULL;
      static Hor_Matrix *Temp_sfbym = NULL;
      static Hor_Matrix *Temp_mbymA = NULL;
      static Hor_Matrix *Temp_mbymB = NULL;
      static Hor_Matrix *Temp_mbymC = NULL;
      static Hor_Matrix *Temp_mbymD = NULL;
      static Hor_Matrix *A = NULL;
      static Hor_Matrix *B = NULL;
      static Hor_Matrix *C = NULL;
      static Hor_Matrix *D = NULL;
      static Hor_Matrix *Pxx_new = NULL;

      hor_mat_ensure_size_list (msize, msize, &S, &Sinv, &Sinvt, &Temp_mbymA,
				&Temp_mbymB, &Temp_mbymC, &Temp_mbymD, NULL);
      hor_mat_ensure_size (msize, vsize, &Temp_mbyv);
      hor_mat_ensure_size (msize, sfsize, &Temp_mbysf);
      hor_mat_ensure_size (vsize, msize, &Temp_vbym);
      hor_mat_ensure_size (sfsize, msize, &Temp_sfbym);
      hor_mat_ensure_size (vsize, vsize, &A);
      hor_mat_ensure_size (vsize, sfsize, &B);
      hor_mat_ensure_size (sfsize, vsize, &C);
      hor_mat_ensure_size (sfsize, sfsize, &D);
      hor_mat_ensure_size (vsize, vsize, &Pxx_new);

      //*** Calculate S and Sinv
      // Is this really necessary?  S is calculated when measurement is predicted...
      hor_matq_ABAT (sf->dh_by_dxv, scene->get_Pxx (), Temp_mbyv, Temp_mbymA);
      hor_matq_AMTBT (sf->dh_by_dy, sf->Pxy, sf->dh_by_dxv, Temp_mbyv, Temp_mbymB);
      hor_matq_AMBT (sf->dh_by_dxv, sf->Pxy, sf->dh_by_dy, Temp_mbysf, Temp_mbymC);
      hor_matq_ABAT (sf->dh_by_dy, sf->Pyy, Temp_mbysf, Temp_mbymD);
      
      // cerr << "A =\n" << Temp_mbymA << "\nB =\n" << Temp_mbymB;
      // cerr << "\nC =\n" << Temp_mbymC << "\nD =\n" << Temp_mbymD << endl;
      assert (hor_print_error () == HORATIO_OK);

      //   S = TempA + TempB + TempC + TempD + R
      for (int r = 0; r < msize; r++) {
	for (int c = 0; c < msize; c++) {
	  S->m[r][c] = Temp_mbymA->m[r][c] + Temp_mbymB->m[r][c] +
	    Temp_mbymC->m[r][c] + Temp_mbymD->m[r][c] + sf->R->m[r][c];
	}
      }
      //      hor_mat_symmetrise (S);
      //      hor_matq_inv (S, Temp_mbymA, Temp_mbymB, Sinv);
      hor_matq_inv_any (S, Temp_mbymA, Temp_mbymB, Temp_mbymC, Temp_mbymD, Sinv);
      if (hor_errno == HOR_MATH_MATRIX_SINGULAR) {
	cerr << "Got `singular' S =\n" << S << "\nwith R =\n" << sf->R << endl;
	hor_wait_for_keyboard ();
	hor_errno = HORATIO_OK;
	hor_matq_scale (S, 1e6);
	hor_matq_inv_any (S, Temp_mbymA, Temp_mbymB, Temp_mbymC, Temp_mbymD, Sinv);
	assert (hor_print_error () == HORATIO_OK);
	hor_matq_scale (S, 1e-6);
	hor_matq_scale (Sinv, 1e6);
      }
      assert (hor_print_error () == HORATIO_OK);
      hor_matq_transpose (Sinv, Sinvt);
      assert (hor_print_error () == HORATIO_OK);

      //*** Calculate A to D
      //      cerr << "S = [\n" << S << "]\n";
      hor_matq_ATMB (sf->dh_by_dxv, Sinvt, sf->dh_by_dxv, Temp_vbym, A);
      hor_matq_ATMB (sf->dh_by_dxv, Sinvt, sf->dh_by_dy, Temp_vbym, B);
      hor_matq_ATMB (sf->dh_by_dy, Sinvt, sf->dh_by_dxv, Temp_sfbym, C);
      hor_matq_ATMB (sf->dh_by_dy, Sinvt, sf->dh_by_dy, Temp_sfbym, D);
      assert (hor_print_error () == HORATIO_OK);

      //*** Update vehicle state and covariance
      update_state (scene->get_xv (), scene->get_Pxx (), sf->Pxy,
		    sf->dh_by_dxv, sf->dh_by_dy, Sinv, sf->nu);
      update_covariance (scene->get_Pxx (), scene->get_Pxx (),
			 scene->get_Pxx (), sf->Pxy, sf->Pxy,
			 A, B, C, D, Pxx_new);
      //      hor_mat_symmetrise (Pxx_new);

      //*** Now loop over each feature for the rest
      int i = sf->position_in_list + 1;  // What row is selected feature on?
      int k = 1;                         // Holds column of current feature
      for (Feature *f = scene->get_first_feature_ptr (); f; f = f->next, k++) {

	if (how_to_filter == JGHK) {
	  if (f->get_age () > params.jghk_age_limit) continue;
	}

	// Size of this feature
	int ksize = f->y->rows;

	// Get covariance between this and observed feature
       	static Hor_Matrix *Pykyi = NULL;
	static Hor_Matrix *Pyiyk = NULL;
	hor_mat_ensure_size (ksize, sfsize, &Pykyi);
	hor_mat_ensure_size (sfsize, ksize, &Pyiyk);
	scene->get_Pyjyk (i, k, Pyiyk);
	hor_matq_transpose (Pyiyk, Pykyi);

	//*** Update covariance between feature and vehicle
	update_covariance (f->Pxy, scene->get_Pxx (), f->Pxy, Pykyi,
			   sf->Pxy, A, B, C, D, f->Pxy_new);

	//*** Update covariance Pykyk
	update_covariance (f->Pyy, f->Pxy, f->Pxy, Pyiyk, Pykyi,
			   A, B, C, D, f->Pyy_new, true, false);
	/* Test when doing two filter types at once
	if (how_to_filter == FAST_FULL) {
	  if (f->position_in_list == 0)
	    hor_matq_fill (f->Pyy_new, 10.0, 0.0, 0.0,  0.0, 10.0, 0.0, 0.0, 0.0, 10.0);
	}
       */
	if (how_to_filter == NEBOT) {
	  double (*norm) (Hor_Matrix *) = hor_uncertainty_volume_3s;
	  f->nebot_ignore = (fabs (norm (f->Pyy) - norm (f->Pyy_new)) <
			      params.nebot_c1 * norm (f->Pyy)) ||
			     (norm (f->Pyy) < params.nebot_c2);
	  if (f->nebot_ignore) {
	    params.features_retained--;
	    // Is it OK to leave Pyy changed?  It looks like not, change back.
	    hor_matq_copy (f->Pyy, f->Pyy_new);
	  }
	}
	//	hor_mat_symmetrise (f->Pyy_new);
	assert (hor_print_error () == HORATIO_OK);

	//*** Now update feature state.  This does observed feature too
	if (how_to_filter != NEBOT || !f->nebot_ignore) {
	  update_state (f->y, f->Pxy, Pykyi, sf->dh_by_dxv, sf->dh_by_dy,
			Sinv, sf->nu, true);
	}

      }        // Next k feature

      // Doing this loop twice isn't the most efficient way to do it, but it
      // does mean we can do the NEBOT method along with the others
      i = sf->position_in_list + 1;  // What row is selected feature on?
      k = 1;                         // Holds column of current feature
      for (Feature *f = scene->get_first_feature_ptr (); f; f = f->next, k++) {

	// Have to do all this stuff again
	if (how_to_filter == JGHK) {
	  if (f->get_age () > params.jghk_age_limit) {
	    params.features_retained--;
	    continue;
	  }
	  else f->jghk_ignore = false;
	}

	// Size of this feature
	int ksize = f->y->rows;

	// Get covariance between this and observed feature
       	static Hor_Matrix *Pykyi = NULL;
	static Hor_Matrix *Pyiyk = NULL;
	hor_mat_ensure_size (ksize, sfsize, &Pykyi);
	hor_mat_ensure_size (sfsize, ksize, &Pyiyk);
	scene->get_Pyjyk (i, k, Pyiyk);
	hor_matq_transpose (Pyiyk, Pykyi);

	//*** Now run through to update the cross-covariances
	int j = 1;                    // Row feature
	Feature *jf = scene->get_first_feature_ptr ();
	for (Matrix_Block *mbptr = f->first_mbptr; mbptr;
	     mbptr = mbptr->next, j++, jf = jf->next) {
	  assert (j < k);

	  if (how_to_filter == JGHK) {
	    if (jf->get_age () > params.jghk_age_limit) continue;
	  }
	  else if (how_to_filter == NEBOT) {
	    if (f->nebot_ignore && jf->nebot_ignore) continue;
	  }

	  // Size of this feature
	  int jsize = jf->feature_measurement_model->FEATURE_STATE_SIZE;

	  // Get covariance Pyjyi
	  static Hor_Matrix *Pyjyi = NULL;
	  hor_mat_ensure_size (jsize, sfsize, &Pyjyi);
	  scene->get_Pyjyk (j, i, Pyjyi);

	  //*** Update cross-covariance Pyjyk
	  update_covariance (mbptr->m, jf->Pxy, f->Pxy, Pyiyk, Pyjyi,
			     A, B, C, D, mbptr->m_new, true, false);
	}
	assert (hor_print_error () == HORATIO_OK);

      }       // Next k feature

      // We postponed actually changing covariances because the old versions
      // were often still needed.  Now actually change covariances to new ones
      change_covariances (scene, Pxx_new);

      //      scene->print_whole_state ();

    }       // End of if (sf->successful_measurement_flag)

  }       // Loop to next selected feature (observed feature)

  // That's it!
  scene->vehicle_state_has_been_changed ();

  // Test result
  /*
  int size2 = scene->get_total_state_size();      // Size of state vector
  Hor_Matrix *x = hor_mat_alloc (size2, 1);
  Hor_Matrix *P = hor_mat_alloc (size2, size2);
  scene->construct_total_state_and_covariance (x, P);
  //  Hor_Matrix *Diff = hor_mats_sub (true_P, P);
   //  cerr << "true P - calc'd P = \n" << Diff << endl;
  */

  return 0;
}


int Kalman::update_filter_internal_measurement (Scene *scene,
						Internal_Measurement_Model *i_m_m)
{
  assert (scene != NULL);

  int out1 = update_filter_internal_measurement_choose (scene, i_m_m, filter1,
							filter1_params);
  int out2 = 0;
  if (maintain_separate_state) {
    switch_saved_for_current_state (scene);
    out2 = update_filter_internal_measurement_choose (scene, i_m_m, filter2,
						      filter2_params);
    switch_saved_for_current_state (scene);
  }

  return out1 != 0 ? out1 : out2;
}


int Kalman::update_filter_internal_measurement_choose (Scene *scene,
						       Internal_Measurement_Model *i_m_m,
						       filter_method how_to_filter,
						       filter_params& params)
{
  switch (how_to_filter) {
  case SLOW:
    return update_filter_internal_measurement_slow (scene,
						    i_m_m);

  case FAST_FULL:
  case JGHK:
    return update_filter_internal_measurement_slow (scene,
						    i_m_m);

  default:
    bool SHOULDNT_REACH_HERE = false;
    assert (SHOULDNT_REACH_HERE);
    return 1;
  }
}


int Kalman::update_filter_internal_measurement_slow (Scene *scene, 
                        Internal_Measurement_Model *internal_measurement_model)
{
  int size = internal_measurement_model->MEASUREMENT_SIZE;
                                                  // Size of measurement vector
  int size2 = scene->get_total_state_size();      // Size of state vector

  Hor_Matrix *x = hor_mat_alloc(size2, 1);
  Hor_Matrix *P = hor_mat_alloc(size2, size2);

  scene->construct_total_state_and_covariance(x, P);

  // cout << "x:" << x;
  // cout << "P:" << P;

  /* 1. Form nu and dh_by_dx */
  Hor_Matrix *nu_tot = hor_mat_alloc(size, 1);
  Hor_Matrix *dh_by_dx_tot = hor_mat_alloc(size, size2);
  Hor_Matrix *dh_by_dx_totT = hor_mat_alloc(size2, size);
  Hor_Matrix *R_tot = hor_mat_alloc(size, size);

  scene->construct_total_internal_measurement_stuff(nu_tot, dh_by_dx_tot, 
						    R_tot);
  hor_matq_transpose(dh_by_dx_tot, dh_by_dx_totT);
  // cout << "nu_tot:" << nu_tot;

  /* 2. Calculate S(k+1) */

  Hor_Matrix *S = hor_mat_alloc(size, size);

  Hor_Matrix *Tempss2 = hor_mat_alloc(size, size2);
  Hor_Matrix *Temps2s = hor_mat_alloc(size2, size);

  hor_matq_prod3(dh_by_dx_tot, P, dh_by_dx_totT, Tempss2, S);
  hor_matq_add2(S, R_tot, S);

  // cout << "R_tot:" << R_tot;
  //  cout << "S:" << S;
  // cout << "dh_by_dx_tot:" << dh_by_dx_tot;
  // cout << "dh_by_dx_totT:" << dh_by_dx_totT;

  /* 3. Calculate W(k+1) */

  Hor_Matrix *SInv = hor_mats_inv(S);

  // cout << "SInv:" << SInv;

  Hor_Matrix *W = hor_mat_alloc(size2, size);
  hor_matq_prod3(P, dh_by_dx_totT, SInv, Temps2s, W);

  // cout << "W:" << W;

  /* 4. Calculate x(k+1|k+1) */
  Hor_Matrix *xdisp = hor_mat_alloc(size2, 1);
  hor_matq_prod2(W, nu_tot, xdisp);

  hor_matq_add2(x, xdisp, x);

  /* 5. Calculate P(k+1|k+1) */
  Hor_Matrix *WT = hor_mat_alloc(size, size2);
  hor_matq_transpose(W, WT);

  Hor_Matrix *Pdisp = hor_mat_alloc(size2, size2);
  hor_matq_prod3(W, S, WT, Temps2s, Pdisp);

  hor_matq_sub(P, Pdisp, P);

  scene->fill_state_and_covariance(x, P);
  scene->vehicle_state_has_been_changed();

  // cout << "x after update:" << x;
  // cout << "P after update:" << P;

  hor_mat_free_list(x, P, dh_by_dx_tot, dh_by_dx_totT, 
		    R_tot, S, Tempss2, Temps2s, SInv, W, nu_tot, 
		    xdisp, WT, Pdisp, NULL);

  return 0;
}


/* Access
 */

int Kalman::set_filter_method (bool which_filter, filter_method new_method)
{
  // true_which_filter = which_filter XOR switch_filters1and2
  filter_method *pfm;
  pfm = (which_filter || switch_filters1and2) &&
    !(which_filter && switch_filters1and2) ? &filter2 : &filter1;

  if (new_method < SLOW || new_method >= BAD_FILTER_METHOD) {
    if (which_filter == 0) filter1 = FAST_FULL;
    else filter2 = FAST_FULL;
    return 1;
  }
  else {
    if (which_filter == 0) filter1 = new_method;
    else filter2 = new_method;
    return 0;
  }
}


int Kalman::switch_to_saved_state (Scene *scene)
{
  if (!maintain_separate_state) {
    cerr << "***ERROR in switch to saved state : ";
    cerr << "not maintaining separate state and covariance\n";
    return 1;
  }
  switch_filters1and2 = !switch_filters1and2;
  filter_method temp_fm = filter1;
  filter1 = filter2;
  filter2 = temp_fm;
  filter_params temp_fp = filter1_params;
  filter1_params = filter2_params;
  filter2_params = temp_fp;
  switch_saved_for_current_state (scene);

  return 0;
}


/* Set filter parameters.  Input negative value to leave unchanged
 */

int Kalman::set_filter_params (bool which_filter, int jghk_age_limit,
			       double nebot_c1, double nebot_c2)
{
  // true_which_filter = which_filter XOR switch_filters1and2
  filter_params *pparams;
  pparams = (which_filter || switch_filters1and2) &&
    !(which_filter && switch_filters1and2) ? &filter2_params : &filter1_params;

  if (jghk_age_limit > 0) {
    pparams->jghk_age_limit = jghk_age_limit;
    return 0;
  }
  if (nebot_c1 >= 0.0) {
    pparams->nebot_c1 = nebot_c1;
  }
  if (nebot_c2 >= 0.0) {
    pparams->nebot_c2 = nebot_c2;
  }

  return 0;
}


Kalman::filter_params Kalman::get_filter_params (bool which_filter)
{
  // true_which_filter = which_filter XOR switch_filters1and2
  return (which_filter || switch_filters1and2) &&
    !(which_filter && switch_filters1and2) ? filter2_params : filter1_params;
}


/* The following are tools for updates
 */

int Kalman::update_filter_slow_separate (Scene *scene, Hor_Matrix **px, Hor_Matrix **pP)
{
  cout << "*** SLOW UPDATE ***\n";

  /* Steps to update the total filter:
     1. Form h and dh_by_dx and R(k+1) and z
     2. Calculate S(k+1)
     3. Calculate W(k+1)
     4. Calculate x(k+1|k+1)
     5. Calculate P(k+1|k+1)
  */

  int size = scene->get_successful_measurement_vector_size();
                                                  // Size of measurement vector
  int size2 = scene->get_total_state_size();      // Size of state vector

  *px = hor_mat_alloc(size2, 1);
  *pP = hor_mat_alloc(size2, size2);

  Hor_Matrix *&x = *px;
  Hor_Matrix *&P = *pP;

  scene->construct_total_state_and_covariance(x, P);

  // cout << "x:" << x;
  // cout << "P:" << P;

  /* 1. Form nu and dh_by_dx */
  Hor_Matrix *nu_tot = hor_mat_alloc(size, 1);
  Hor_Matrix *dh_by_dx_tot = hor_mat_alloc(size, size2);
  Hor_Matrix *dh_by_dx_totT = hor_mat_alloc(size2, size);
  Hor_Matrix *R_tot = hor_mat_alloc(size, size);

  scene->construct_total_measurement_stuff(nu_tot, dh_by_dx_tot, R_tot);
  hor_matq_transpose(dh_by_dx_tot, dh_by_dx_totT);
  // cout << "nu_tot:" << nu_tot;

  /* 2. Calculate S(k+1) */

  Hor_Matrix *S = hor_mat_alloc(size, size);

  Hor_Matrix *Tempss2 = hor_mat_alloc(size, size2);
  Hor_Matrix *Temps2s = hor_mat_alloc(size2, size);

  hor_matq_prod3(dh_by_dx_tot, P, dh_by_dx_totT, Tempss2, S);
  hor_matq_add2(S, R_tot, S);

  // cout << "R_tot:" << R_tot;
  //  cout << "S = [\n" << S << "]\n";
  // cout << "dh_by_dx_tot:" << dh_by_dx_tot;
  // cout << "dh_by_dx_totT:" << dh_by_dx_totT;

  /* 3. Calculate W(k+1) */

  Hor_Matrix *TempssA = hor_mat_alloc (size, size);
  Hor_Matrix *TempssB = hor_mat_alloc (size, size);
  Hor_Matrix *TempssC = hor_mat_alloc (size, size);
  Hor_Matrix *TempssD = hor_mat_alloc (size, size);
  Hor_Matrix *SInv = hor_mat_alloc (size, size);
  hor_matq_inv (S, TempssA, TempssB, SInv);
  assert (hor_print_error () == HORATIO_OK);

  Hor_Matrix *W = hor_mat_alloc(size2, size);
  hor_matq_prod3(P, dh_by_dx_totT, SInv, Temps2s, W);

  // cout << "W:" << W;

  /* 4. Calculate x(k+1|k+1) */
  Hor_Matrix *xdisp = hor_mat_alloc(size2, 1);
  hor_matq_prod2(W, nu_tot, xdisp);

  hor_matq_add2(x, xdisp, x);

  /* 5. Calculate P(k+1|k+1) */
  Hor_Matrix *WT = hor_mat_alloc(size, size2);
  hor_matq_transpose(W, WT);

  Hor_Matrix *Pdisp = hor_mat_alloc(size2, size2);
  hor_matq_prod3(W, S, WT, Temps2s, Pdisp);

  hor_matq_sub(P, Pdisp, P);

  // cout << "x after update:" << x;
  // cout << "P after update:" << P;

  hor_mat_free_list(dh_by_dx_tot, dh_by_dx_totT, 
		    R_tot, S, Tempss2, Temps2s, TempssA, TempssB,
		    TempssC, TempssD, SInv, W, nu_tot, 
		    xdisp, WT, Pdisp, NULL);

  return 0;
}


Hor_Matrix *Kalman::update_state (Hor_Matrix *state, Hor_Matrix *P1,
				  Hor_Matrix *P2, Hor_Matrix *dh_by_dxv,
				  Hor_Matrix *dh_by_dy, Hor_Matrix *Sinv,
				  Hor_Matrix *nu, bool transpose_P1 = false)
{
  assert (state != NULL && P1 != NULL && P2 != NULL && dh_by_dxv != NULL &&
	  dh_by_dy != NULL && Sinv != NULL && nu != NULL);
  if (transpose_P1) assert (P1->cols == state->rows && P1->rows == dh_by_dxv->cols);
  else assert (P1->rows == state->rows && dh_by_dxv->cols == P1->cols);
  assert (Sinv->rows == dh_by_dxv->rows && nu->rows == Sinv->cols &&
	  P2->rows == state->rows && dh_by_dy->cols == P2->cols &&
	  Sinv->rows == dh_by_dy->rows && nu->cols == state->cols);

  static Hor_Matrix *Temp_sbymA = NULL;
  static Hor_Matrix *Temp_sbymB = NULL;
  static Hor_Matrix *Temp_sby1A = NULL;
  static Hor_Matrix *Temp_sby1B = NULL;
  static Hor_Matrix *Temp_mbys  = NULL;
  int ssize  = state->rows;
  int msize  = dh_by_dxv->rows;
  hor_mat_ensure_size_list (ssize, msize, &Temp_sbymA, &Temp_sbymB, NULL);
  hor_mat_ensure_size_list (ssize, 1, &Temp_sby1A, &Temp_sby1B, NULL);
  hor_mat_ensure_size (msize, ssize, &Temp_mbys);

  if (!transpose_P1) {
    hor_matq_ABT (P1, dh_by_dxv, Temp_sbymA);
    hor_matq_prod3 (Temp_sbymA, Sinv, nu, Temp_sbymB, Temp_sby1B);
  }
  else {
    hor_matq_prod2 (dh_by_dxv, P1, Temp_mbys);
    hor_matq_ATMB (Temp_mbys, Sinv, nu, Temp_sbymA, Temp_sby1B);
  }
  hor_matq_add2 (state, Temp_sby1B, Temp_sby1A);

  hor_matq_ABT (P2, dh_by_dy, Temp_sbymA);
  hor_matq_prod3 (Temp_sbymA, Sinv, nu, Temp_sbymB, Temp_sby1B);

  hor_matq_add2 (Temp_sby1A, Temp_sby1B, state);

  assert (hor_print_error () == HORATIO_OK);
  return state;
}


Hor_Matrix *Kalman::update_covariance (Hor_Matrix *Px, Hor_Matrix *P1,
				       Hor_Matrix *P2, Hor_Matrix *P3,
				       Hor_Matrix *P4, Hor_Matrix *A,
				       Hor_Matrix *B,  Hor_Matrix *C,
				       Hor_Matrix *D, Hor_Matrix *R,
				       bool transpose_P1 = false,
				       bool transpose_P3 = true)
{
  assert (Px != NULL && P1 != NULL && P2 != NULL && P3 != NULL
	  && P4 != NULL && R != NULL);
  if (transpose_P1) assert (P1->cols == Px->rows && P1->rows == A->rows);
  else assert (P1->rows == Px->rows && P1->cols == A->rows);
  assert (A->cols == P2->rows && P4->rows == Px->rows &&
	  P4->cols == C->rows && C->cols == P2->rows);
  if (transpose_P3) assert (B->cols == P3->cols && P3->rows == Px->cols);
  else assert (B->cols == P3->rows && P3->cols == Px->cols);
  assert (hor_mat_same_size2 (Px, R));

  int vsize  = A->rows;
  int sfsize = B->cols;
  int rs     = Px->rows;
  int cs     = Px->cols;
  static Hor_Matrix *Temp_rsbyv   = NULL;
  static Hor_Matrix *Temp_rsbysf  = NULL;
  static Hor_Matrix *Temp_rsbycsA = NULL;
  static Hor_Matrix *Temp_rsbycsB = NULL;
  static Hor_Matrix *Temp_rsbycsC = NULL;
  static Hor_Matrix *Temp_rsbycsD = NULL;
  hor_mat_ensure_size (rs, vsize, &Temp_rsbyv);
  hor_mat_ensure_size (rs, sfsize, &Temp_rsbysf);
  hor_mat_ensure_size_list (rs, cs, &Temp_rsbycsA, &Temp_rsbycsB,
			    &Temp_rsbycsC, &Temp_rsbycsD, NULL);

  if (transpose_P1) hor_matq_ATMB (P1, A, P2, Temp_rsbyv, Temp_rsbycsA);
  else hor_matq_prod3 (P1, A, P2, Temp_rsbyv, Temp_rsbycsA);

  if (transpose_P1) hor_matq_ATB (P1, B, Temp_rsbysf);
  else hor_matq_prod2 (P1, B, Temp_rsbysf);
  if (transpose_P3) hor_matq_ABT (Temp_rsbysf, P3, Temp_rsbycsB);
  else hor_matq_prod2 (Temp_rsbysf, P3, Temp_rsbycsB);

  hor_matq_prod3 (P4, C, P2, Temp_rsbyv, Temp_rsbycsC);

  if (transpose_P3) hor_matq_AMBT (P4, D, P3, Temp_rsbysf, Temp_rsbycsD);
  else hor_matq_prod3 (P4, D, P3, Temp_rsbysf, Temp_rsbycsD);

  // Final subtraction
  hor_matq_copy (Px, R);
  hor_matq_sub4 (R, Temp_rsbycsA, Temp_rsbycsB, Temp_rsbycsC, Temp_rsbycsD);

  assert (hor_print_error () == HORATIO_OK);
  return R;
}


/* This baby is the same as the above for Pyy covariances only, and will
 * only update Pyy if it passed the NEBOT test.  It returns true if it
 * passed the nebot test.
 * I'm not using this because it only saves one subtraction in terms
 * of processing and I think calculating the uncertainty volume of
 * Delta P may be invalid.
 */

bool Kalman::update_Pyy_nebot (Hor_Matrix *Pyy, Hor_Matrix *Pxy,
			       Hor_Matrix *Pyiyk, Hor_Matrix *Pykyi,
			       Hor_Matrix *A, Hor_Matrix *B, Hor_Matrix *C,
			       Hor_Matrix *D, Hor_Matrix *R,
			       double c1, double c2)
{
  assert (Pyy != NULL && Pxy != NULL && Pyiyk != NULL
	  && Pykyi != NULL && A != NULL && B != NULL
	  && C != NULL && D != NULL && R != NULL);
  assert (Pxy->cols == Pyy->rows && Pxy->rows == A->rows);
  assert (A->cols == Pxy->rows && Pykyi->rows == Pyy->rows &&
	  Pykyi->cols == C->rows && C->cols == Pxy->rows);
  assert (B->cols == Pyiyk->rows && Pyiyk->cols == Pyy->cols);
  assert (hor_mat_same_size2 (Pyy, R));

  int vsize  = A->rows;
  int sfsize = B->cols;
  int rs     = Pyy->rows;
  int cs     = Pyy->cols;
  static Hor_Matrix *Temp_rsbyv   = NULL;
  static Hor_Matrix *Temp_rsbysf  = NULL;
  static Hor_Matrix *Temp_rsbycsA = NULL;
  static Hor_Matrix *Temp_rsbycsB = NULL;
  static Hor_Matrix *Temp_rsbycsC = NULL;
  static Hor_Matrix *Temp_rsbycsD = NULL;
  static Hor_Matrix *Temp_rsbycsSum = NULL;
  hor_mat_ensure_size (rs, vsize, &Temp_rsbyv);
  hor_mat_ensure_size (rs, sfsize, &Temp_rsbysf);
  hor_mat_ensure_size_list (rs, cs, &Temp_rsbycsA, &Temp_rsbycsB,
			    &Temp_rsbycsC, &Temp_rsbycsD,
			    &Temp_rsbycsSum, NULL);

  hor_matq_ATMB (Pxy, A, Pxy, Temp_rsbyv, Temp_rsbycsA);
  hor_matq_ATMB (Pxy, B, Pyiyk, Temp_rsbysf, Temp_rsbycsB);
  hor_matq_prod3 (Pykyi, C, Pxy, Temp_rsbyv, Temp_rsbycsC);
  hor_matq_prod3 (Pykyi, D, Pyiyk, Temp_rsbysf, Temp_rsbycsD);

  // Final subtraction
  hor_matq_add4 (Temp_rsbycsSum, Temp_rsbycsA, Temp_rsbycsB,
		 Temp_rsbycsC, Temp_rsbycsD);
  hor_matq_copy (Pyy, Temp_rsbycsA);
  hor_matq_scale (Temp_rsbycsA, c1);

  assert (hor_print_error () == HORATIO_OK);

  double (*norm) (Hor_Matrix *) = hor_uncertainty_volume_3s;
  if (norm (Temp_rsbycsSum) < norm (Temp_rsbycsA) || norm (Pyy) < c2) {
    hor_matq_copy (Pyy, R);
    return false;
  }
  hor_matq_sub (Pyy, Temp_rsbycsSum, R);
  return true;

  return R;
}


/* In Feature there is an old and a new version of every matrix.  Shift the
 * new to the old to bring each feature up to date
 */

int Kalman::change_covariances (Scene *scene, Hor_Matrix *Pxx_new)
{
  assert (scene != NULL && Pxx_new != NULL);
  assert (hor_mat_same_size2 (scene->get_Pxx (), Pxx_new));

  hor_matq_copy (Pxx_new, scene->get_Pxx ());
  for (Feature *f = scene->get_first_feature_ptr (); f; f = f->get_next ()) {
    f->bring_covariances_up_to_date ();
  }

  return 0;
}

/* On the other hand, we may need to make sure all the `new' versions are the
 * same as the current versions, using this method
 */

int Kalman::copy_covariances_to_new (Scene *scene)
{
  assert (scene != NULL);

  for (Feature *f = scene->get_first_feature_ptr (); f; f = f->get_next ()) {
    f->copy_covariances_to_new ();
  }

  return 0;
}


/* For the JGHK filter method: To ensure valid cross-covariances between
 * features that we wish to reincorporate into the group of features
 * being updated following a period outside that group.
 * At the moment, this means setting everything to zero (all the other
 * calculation is a hang-over from before but I'm not going to delete
 * it yet).
 */

void Kalman::reincorporate_features (Scene *scene, Kalman::filter_params params)
{
  assert (scene != NULL);

  int vsize = scene->get_motion_model()->STATE_SIZE;
  for (Feature *f = scene->get_first_feature_ptr (); f; f = f->next) {
    if (f->get_age () > params.jghk_age_limit) {
      f->jghk_ignore = true;
    }
    else if (f->jghk_ignore) {
      f->jghk_ignore = false;

      // Must calculate dy_by_dxv
      Feature_Measurement_Model *fmm = f->feature_measurement_model;
      Motion_Model *mm = scene->get_motion_model ();
      static Hor_Matrix *dy_by_dxv = NULL;
      static Hor_Matrix *dxv_by_dy = NULL;
      static Hor_Matrix *U = NULL;
      static Hor_Matrix *W = NULL;
      static Hor_Matrix *V = NULL;
      static Hor_Matrix *Ut = NULL;
      hor_mat_ensure_size (fmm->FEATURE_STATE_SIZE, vsize, &dy_by_dxv);
      hor_mat_ensure_size_list (vsize, fmm->FEATURE_STATE_SIZE, &dxv_by_dy, &U,
				&W, &V, &Ut, NULL);

      scene->get_motion_model()->func_xp (scene->get_xv ());
      scene->get_motion_model()->func_dxp_by_dxv (scene->get_xv ());
      fmm->func_hi_and_dhi_by_dxp_and_dhi_by_dyi (f->y, mm->xpRES);
      fmm->func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri (fmm->hiRES, mm->xpRES);
      hor_matq_prod2 (fmm->dyi_by_dxpRES, mm->dxp_by_dxvRES, dy_by_dxv);
      hor_matq_psinv (dy_by_dxv, U, W, V, Ut, dxv_by_dy);
      assert (hor_print_error () == HORATIO_OK);

      // Covariance between feature and vehicle Pxyi
      //      hor_matq_ABT (scene->get_Pxx (), dy_by_dxv, f->Pxy);
      hor_matq_zero (f->Pxy);
      //      hor_matq_prod2 (dxv_by_dy, f->Pyy, f->Pxy);

      // Run through other features and update cross-covariances
      int i = f->position_in_list + 1;
      int j = 1;                         // Other feature
      for (Feature *jf = scene->get_first_feature_ptr (); jf; j++, jf = jf->next) {

	// Don't need to bother with this if I'm not updating this feature
	if (jf->get_age () > params.jghk_age_limit) continue;
	if (j == i) continue;

	// Now adjust Pyjyi
	if (j < i) {
	  Hor_Matrix *Pyjyi = scene->get_Pyjyk_ptr (j, i);
	  static Hor_Matrix *Pyiyj = NULL;
	  hor_mat_ensure_size (Pyjyi->cols, Pyjyi->rows, &Pyiyj);
	  hor_matq_prod2 (dy_by_dxv, jf->Pxy, Pyiyj);
	  hor_matq_transpose (Pyiyj, Pyjyi);
	  hor_matq_zero (Pyjyi);
	}
	else {
	  Hor_Matrix *Pyiyj = scene->get_Pyjyk_ptr (i, j);
	  hor_matq_prod2 (dy_by_dxv, jf->Pxy, Pyiyj);
	  hor_matq_zero (Pyiyj);
	}
      }
      assert (hor_print_error () == HORATIO_OK);

    }
  }
}


/* If we're maintaining a separate state then when it's first used it must
 * have been initialised.
 */

void Kalman::initialise_separate_state (Scene *scene)
{
  assert (scene != NULL);
  assert (maintain_separate_state);

  int size = scene->get_total_state_size ();
  hor_mat_ensure_size (size, 1, &x_saved);
  hor_mat_ensure_size (size, size, &P_saved);

  scene->construct_total_state_and_covariance (x_saved, P_saved);
}


/* For testing: switch the state and covariance recorded in the features in
 * scene with that saved within the Kalman class.  This is so that we can
 * update both in different ways and compare the result.
 * NOTE: THIS COCKS UP IF YOU DELETE FEATURES -- it can't handle that yet
 * It doesn't work with any Scene type other than Scene_Single yet either.
 */

int Kalman::switch_saved_for_current_state (Scene *scene)
{
  assert (scene != NULL);
  assert (maintain_separate_state);

  static Hor_Matrix *x_temp = NULL;
  static Hor_Matrix *P_temp = NULL;

  int size = scene->get_total_state_size();      // Size of state vector
  hor_mat_ensure_size (size, 1, &x_temp);
  hor_mat_ensure_size (size, size, &P_temp);

  scene->construct_total_state_and_covariance (x_temp, P_temp);
  scene->fill_state_and_covariance (x_saved, P_saved);

  // Reinitialise new points
  for (Feature *f = scene->get_first_feature_ptr (); f; f = f->get_next ()) {
    if (f->position_in_total_state_vector < x_saved->rows) continue;
    f->reinitialise ((Scene_Single *) (scene));
  }

  hor_mat_ensure_size (size, 1, &x_saved);
  hor_mat_ensure_size (size, size, &P_saved);
  hor_matq_copy (x_temp, x_saved);
  hor_matq_copy (P_temp, P_saved);

  return 0;
}
