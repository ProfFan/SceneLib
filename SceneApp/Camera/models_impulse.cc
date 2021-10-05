/*  SceneApp: applications for sequential localisation and map-building

    Camera/models_impulse.cc
    Copyright (C) 2001 Andrew Davison
    ajd@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "general_headers.h"
#include "models_base.h"
#include "models_impulse.h"

Impulse_ThreeD_Motion_Model::Impulse_ThreeD_Motion_Model()
  : ThreeD_Motion_Model(13, 3, "IMPULSE_THREED")
{
  rold = hor_mat_alloc(3, 1);
  qold = hor_mat_alloc(4, 1);
  vold = hor_mat_alloc(3, 1);
  omegaold = hor_mat_alloc(3, 1);
  rnew = hor_mat_alloc(3, 1);
  qnew = hor_mat_alloc(4, 1);
  vnew = hor_mat_alloc(3, 1);
  omeganew = hor_mat_alloc(3, 1);
  Temp31A = hor_mat_alloc(3, 1);
  Temp31B = hor_mat_alloc(3, 1);
  TempqA = hor_mat_alloc(4, 1);
  TempqB = hor_mat_alloc(4, 1);
  qwt = hor_mat_alloc(4, 1);
  Temp33A = hor_mat_alloc(3, 3);
  Temp33B = hor_mat_alloc(3, 3);
  Temp44A = hor_mat_alloc(4, 4);
  Temp44B = hor_mat_alloc(4, 4);
  Temp43A = hor_mat_alloc(4, 3);
  Temp43B = hor_mat_alloc(4, 3);
  Pnn = hor_mat_alloc(6, 6);
  dxnew_by_dn = hor_mat_alloc(STATE_SIZE, 6);
  dxnew_by_dnT = hor_mat_alloc(6, STATE_SIZE);
  Temp_STATE_NOISE = hor_mat_alloc(STATE_SIZE, 6);
  xv_noisy = hor_mat_alloc(STATE_SIZE, 1);
}

Impulse_ThreeD_Motion_Model::~Impulse_ThreeD_Motion_Model()
{
  hor_mat_free_list(rold, qold, vold, omegaold,
		    rnew, qnew, vnew, omeganew, 
		    Temp31A, Temp31B, TempqA, TempqB, qwt, 
		    Temp33A, Temp33B, Temp44A, Temp44B, Temp43A, Temp43B, 
		    Pnn, dxnew_by_dn, dxnew_by_dnT, Temp_STATE_NOISE,
		    xv_noisy,
		    NULL);
}

int Impulse_ThreeD_Motion_Model::func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
					   Hor_Matrix *u, double delta_t)
{
  assert (xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Separate things out to make it clearer
  extract_r_q_v_omega(xv, rold, qold, vold, omegaold);

  // rnew = r + v * delta_t
  hor_matq_copy(vold, Temp31A);
  hor_matq_scale(Temp31A, delta_t);
  hor_matq_add2(rold, Temp31A, rnew);

  // qnew = q x q(omega * delta_t)
  hor_matq_copy(omegaold, Temp31A);
  hor_matq_scale(Temp31A, delta_t);
  // Form quaternion from angle-axis vector
  q_from_aa(Temp31A, qwt); // Keep qwt (q(omega * delta_t)) for later use

  prodq2q1(qold, qwt, qnew); 

  // vnew = v
  hor_matq_copy(vold, vnew);
  
  // omeganew = omega
  hor_matq_copy(omegaold, omeganew);

  // Put it all together
  compose_xv(rnew, qnew, vnew, omeganew, fvRES);

  // cout << "rold qold vold omegaold" << rold << qold 
  //      << vold << omegaold;
  // cout << "rnew qnew vnew omeganew" << rnew << qnew 
  //      << vnew << omeganew;

  // Now on to the Jacobian...
  // Identity is a good place to start since overall structure is like this
  // I       0             I * delta_t   0
  // 0       dqnew_by_dq   0             dqnew_by_domega
  // 0       0             I             0
  // 0       0             0             I
  hor_matq_identity(dfv_by_dxvRES);

  // Fill in dxnew_by_dv = I * delta_t
  hor_matq_identity(Temp33A);
  hor_matq_scale(Temp33A, delta_t);
  hor_matq_insert_chunkyx(Temp33A, dfv_by_dxvRES, 0, 7);

  // Fill in dqnew_by_dq
  // qnew = qold x qwt  = q2 x q1 in bits.cc language
  dq3_by_dq2(qwt, Temp44A);
  hor_matq_insert_chunkyx(Temp44A, dfv_by_dxvRES, 3, 3);

  // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
  dq3_by_dq1(qold, Temp44A); // Temp44A is d(q x qwt) by dqwt
  // Use function below for dqwt_by_domega
  dqomegadt_by_domega(omegaold, delta_t, Temp43A);
  // Multiply them together
  hor_matq_prod2(Temp44A, Temp43A, Temp43B);
  // And plug it in
  hor_matq_insert_chunkyx(Temp43B, dfv_by_dxvRES, 3, 10);

  // cout << "dfv_by_dxvRES" << dfv_by_dxvRES;

  return 0;
}

int Impulse_ThreeD_Motion_Model::func_Q(Hor_Matrix *xv, 
					Hor_Matrix *u, double delta_t)
{
  assert(xv != NULL && u != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  u->rows == CONTROL_SIZE && u->cols == 1);

  // Fill noise covariance matrix Pnn: this is the covariance of 
  // the noise vector (V)
  //                  (Omega)
  // that gets added to the state. 
  // Form of this could change later, but for now assume that 
  // V and Omega are independent, and that each of their components is
  // independent... 
  double linear_velocity_noise_covariance = 
     SD_A_component_filter * SD_A_component_filter * delta_t * delta_t;
  double angular_velocity_noise_covariance =
     SD_alpha_component_filter * SD_alpha_component_filter * delta_t * delta_t;

  hor_matq_zero(Pnn);
  matel(Pnn, 1, 1) = linear_velocity_noise_covariance;
  matel(Pnn, 2, 2) = linear_velocity_noise_covariance;
  matel(Pnn, 3, 3) = linear_velocity_noise_covariance;
  matel(Pnn, 4, 4) = angular_velocity_noise_covariance;
  matel(Pnn, 5, 5) = angular_velocity_noise_covariance;
  matel(Pnn, 6, 6) = angular_velocity_noise_covariance;

  // Form Jacobian dxnew_by_dn
  // Is like this:
  // I * delta_t     0
  // 0               dqnew_by_dOmega
  // I               0
  // 0               I

  // Start by zeroing
  hor_matq_zero(dxnew_by_dn);

  // Fill in easy bits first
  hor_matq_identity(Temp33A);
  hor_matq_insert_chunkyx(Temp33A, dxnew_by_dn, 7, 0);
  hor_matq_insert_chunkyx(Temp33A, dxnew_by_dn, 10, 3);
  hor_matq_scale(Temp33A, delta_t);
  hor_matq_insert_chunkyx(Temp33A, dxnew_by_dn, 0, 0);

  // Tricky bit is dqnew_by_dOmega
  // Is actually the same calculation as in func_fv...
  // Since omega and Omega are additive...?
  extract_r_q_v_omega(xv, rold, qold, vold, omegaold); // overkill but easy
  // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
  dq3_by_dq1(qold, Temp44A); // Temp44A is d(q x qwt) by dqwt
  // Use function below for dqwt_by_domega
  dqomegadt_by_domega(omegaold, delta_t, Temp43A);
  // Multiply them together
  hor_matq_prod2(Temp44A, Temp43A, Temp43B);
  // And then plug into Jacobian
  hor_matq_insert_chunkyx(Temp43B, dxnew_by_dn, 3, 3);

  // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
  hor_matq_transpose(dxnew_by_dn, dxnew_by_dnT);

  hor_matq_prod2(dxnew_by_dn, Pnn, Temp_STATE_NOISE);
  hor_matq_prod2(Temp_STATE_NOISE, dxnew_by_dnT, QxRES);

  //  cout << "QxRES" << QxRES;

  return 0;
}

int Impulse_ThreeD_Motion_Model::func_xp(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_fill(xpRES,
		vecel(xv, 1),
		vecel(xv, 2),
		vecel(xv, 3),
		vecel(xv, 4),
		vecel(xv, 5),
		vecel(xv, 6),
		vecel(xv, 7));

  return 0;
}

int Impulse_ThreeD_Motion_Model::func_dxp_by_dxv(Hor_Matrix *xv)
{
  assert(xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  hor_matq_zero(dxp_by_dxvRES);

  matel(dxp_by_dxvRES, 1, 1) = 1.0;
  matel(dxp_by_dxvRES, 2, 2) = 1.0;
  matel(dxp_by_dxvRES, 3, 3) = 1.0;
  matel(dxp_by_dxvRES, 4, 4) = 1.0;
  matel(dxp_by_dxvRES, 5, 5) = 1.0;
  matel(dxp_by_dxvRES, 6, 6) = 1.0;
  matel(dxp_by_dxvRES, 7, 7) = 1.0;

  return 0;
}

// Noisy process equation for simulation
// Simply perturb xv with Gaussian noise and send it through func_fv
int Impulse_ThreeD_Motion_Model::func_fv_noisy(Hor_Matrix *xv_true, 
				       Hor_Matrix *u_true, double delta_t)
{
  assert (xv_true != NULL && u_true != NULL);

  assert (xv_true->rows == STATE_SIZE && xv_true->cols == 1 &&
	  u_true->rows == CONTROL_SIZE && u_true->cols == 1);

  hor_matq_copy(xv_true, xv_noisy);
  
  // Linear velocity
  for (int row = 8; row <= 10; row++)
    vecel(xv_noisy, row) 
         = hor_gauss_rand(vecel(xv_true, row), SD_A_component * delta_t);
  // Angular velocity
  for (int row = 11; row <= 13; row++)
    vecel(xv_noisy, row) 
         = hor_gauss_rand(vecel(xv_true, row), SD_alpha_component * delta_t);

  // Now send through normal process equaion
  func_fv_and_dfv_by_dxv(xv_noisy, u_true, delta_t);

  // And copy result
  hor_matq_copy(fvRES, fv_noisyRES);

  return 0;
}

int Impulse_ThreeD_Motion_Model::
      func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(Hor_Matrix *xv, 
							 Hor_Matrix *xpdef)
{
  assert (xv != NULL && xpdef != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  xpdef->rows == POSITION_STATE_SIZE && xpdef->cols == 1);

  // When we redefine axes:
  // r and q change as normal
  // v and omega are vectors so they change in the same way as r

  cerr << "func_xvredef... not yet implemented for Impulse_ThreeD_Motion_Model"
       << endl;
  assert(0);

  return 0;
}

int Impulse_ThreeD_Motion_Model::
      func_xvnorm_and_dxvnorm_by_dxv(Hor_Matrix *xv)
{
  assert (xv != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1);

  // Normalise the state vector: since quaternion is redundant we sometimes
  // need to enforce that it stays with size 1

  // Most parts of the state vector don't change so copy as starting point
  hor_matq_copy(xv, xvnormRES);

  // Most parts of Jacobian are identity
  hor_matq_identity(dxvnorm_by_dxvRES);

  // Extract quaternion  
  func_xp(xv);
  func_q(xpRES);

  hor_matq_copy(qRES, Tempqa);

  normalise_quaternion(Tempqa, Tempqb);
  dqnorm_by_dq(Tempqa, Temp44a);

  hor_matq_insert_chunky1(Tempqb, xvnormRES, 3);
  hor_matq_insert_chunkyx(Temp44a, dxvnorm_by_dxvRES, 3, 3);

  return 0;
}

// Easy access to state blocks: fill matrices r, q, v, omega with
// values based on state xv
int Impulse_ThreeD_Motion_Model::extract_r_q_v_omega(Hor_Matrix *xv, 
         Hor_Matrix *r, Hor_Matrix *q, Hor_Matrix *v, Hor_Matrix *omega)
{
  assert (xv != NULL && r != NULL && q != NULL && v != NULL && omega != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  r->rows == 3 && r->cols == 1 &&
	  q->rows == 4 && q->cols == 1 &&
	  v->rows == 3 && v->cols == 1 &&
	  omega->rows == 3 && omega->cols == 1 );

  int row = 0;

  hor_matq_extract_chunky1(xv, r, row);
  row += r->rows;
  hor_matq_extract_chunky1(xv, q, row);
  row += q->rows;
  hor_matq_extract_chunky1(xv, v, row);
  row += v->rows;
  hor_matq_extract_chunky1(xv, omega, row);
  row += omega->rows;

  return 0;
}
// The opposite: put r, q, v, omega back into vector xv
int Impulse_ThreeD_Motion_Model::compose_xv(Hor_Matrix *r, Hor_Matrix *q,
		 Hor_Matrix *v, Hor_Matrix *omega, Hor_Matrix *xv)
{
  assert (xv != NULL && r != NULL && q != NULL && v != NULL && omega != NULL);

  assert (xv->rows == STATE_SIZE && xv->cols == 1 &&
	  r->rows == 3 && r->cols == 1 &&
	  q->rows == 4 && q->cols == 1 &&
	  v->rows == 3 && v->cols == 1 &&
	  omega->rows == 3 && omega->cols == 1 );

  int row = 0;

  hor_matq_insert_chunky1(r, xv, row);
  row += r->rows;
  hor_matq_insert_chunky1(q, xv, row);
  row += q->rows;
  hor_matq_insert_chunky1(v, xv, row);
  row += v->rows;
  hor_matq_insert_chunky1(omega, xv, row);
  row += omega->rows;

  return 0;
}

// Calculate commonly used Jacobian part dq(omega * delta_t) by domega
int Impulse_ThreeD_Motion_Model::dqomegadt_by_domega(Hor_Matrix *omega, 
                              double delta_t, Hor_Matrix *dqomegadt_by_domega)
{
  assert (omega != NULL && dqomegadt_by_domega != NULL);

  assert (omega->rows == 3 && omega->cols == 1 &&
	  dqomegadt_by_domega->rows == 4 && dqomegadt_by_domega->cols == 3);

  // Make reference labels for clarity
  double &omegax = vecel(omega, 1);
  double &omegay = vecel(omega, 2);
  double &omegaz = vecel(omega, 3);

  double &dq0_by_domegax = matel(dqomegadt_by_domega, 1, 1);
  double &dq0_by_domegay = matel(dqomegadt_by_domega, 1, 2);
  double &dq0_by_domegaz = matel(dqomegadt_by_domega, 1, 3);
  double &dqx_by_domegax = matel(dqomegadt_by_domega, 2, 1);
  double &dqx_by_domegay = matel(dqomegadt_by_domega, 2, 2);
  double &dqx_by_domegaz = matel(dqomegadt_by_domega, 2, 3);
  double &dqy_by_domegax = matel(dqomegadt_by_domega, 3, 1);
  double &dqy_by_domegay = matel(dqomegadt_by_domega, 3, 2);
  double &dqy_by_domegaz = matel(dqomegadt_by_domega, 3, 3);
  double &dqz_by_domegax = matel(dqomegadt_by_domega, 4, 1);
  double &dqz_by_domegay = matel(dqomegadt_by_domega, 4, 2);
  double &dqz_by_domegaz = matel(dqomegadt_by_domega, 4, 3);

  // Modulus
  double omegamod = sqrt(omegax * omegax + omegay * omegay + omegaz * omegaz);

  // Use generic ancillary functions to calculate components of Jacobian
  dq0_by_domegax = dq0_by_domegaA(omegax, omegamod, delta_t);
  dq0_by_domegay = dq0_by_domegaA(omegay, omegamod, delta_t);
  dq0_by_domegaz = dq0_by_domegaA(omegaz, omegamod, delta_t);
  dqx_by_domegax = dqA_by_domegaA(omegax, omegamod, delta_t);
  dqx_by_domegay = dqA_by_domegaB(omegax, omegay, omegamod, delta_t);
  dqx_by_domegaz = dqA_by_domegaB(omegax, omegaz, omegamod, delta_t);
  dqy_by_domegax = dqA_by_domegaB(omegay, omegax, omegamod, delta_t);
  dqy_by_domegay = dqA_by_domegaA(omegay, omegamod, delta_t);
  dqy_by_domegaz = dqA_by_domegaB(omegay, omegaz, omegamod, delta_t);
  dqz_by_domegax = dqA_by_domegaB(omegaz, omegax, omegamod, delta_t);
  dqz_by_domegay = dqA_by_domegaB(omegaz, omegay, omegamod, delta_t);
  dqz_by_domegaz = dqA_by_domegaA(omegaz, omegamod, delta_t);
  
  return 0;
}


// Ancillary functions: calculate parts of Jacobian dq_by_domega
// which are repeatable due to symmetry.
// Here omegaA is one of omegax, omegay, omegaz
// omegaB, omegaC are the other two
// And similarly with qA, qB, qC

double Impulse_ThreeD_Motion_Model::dq0_by_domegaA(double omegaA, 
                                         double omega, double delta_t)
{
  return (-delta_t / 2.0) * (omegaA / omega) * sin(omega * delta_t / 2.0);
}

double Impulse_ThreeD_Motion_Model::dqA_by_domegaA(double omegaA, 
                                         double omega, double delta_t)
{
  return (delta_t / 2.0) * omegaA * omegaA / (omega * omega) 
    * cos(omega * delta_t / 2.0)
    + (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega))
    * sin(omega * delta_t / 2.0);
}

double Impulse_ThreeD_Motion_Model::dqA_by_domegaB(double omegaA, 
                         double omegaB, double omega, double delta_t)
{
  return (omegaA * omegaB / (omega * omega)) * 
    ( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
      - (1.0 / omega) * sin(omega * delta_t / 2.0) );
}
