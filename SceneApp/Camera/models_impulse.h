/*  SceneApp: applications for sequential localisation and map-building

    Camera/models_impulse.h
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

// General motion model in 3D
// Assumes a random impulse changes the velocity at each time step
// State vector: 13 elements
//                 x 
//  r              y 
//                 z 
//  -              - 
//                 q0 
//  q              qx
//                 qy
//                 qz
//  -      =       - 
//                 vx
//  v              vy
//                 vz
//  -              - 
//                 omegax
//  omega          omegay
//                 omegaz
// Control vector has no elements

// Noise vector n = V
//                  Omega
// Update:
// rnew     =   r + (v + V) delta_t
// qnew     =   q x q((omega + Omega) delta_t)
// vnew     =   v + V
// omeganew =   omega + Omega

class Impulse_ThreeD_Motion_Model : public ThreeD_Motion_Model
{
 public:
  Impulse_ThreeD_Motion_Model();
  ~Impulse_ThreeD_Motion_Model();

  // Constants
  // May need a lot of tweaking!
  // Standard deviation of linear acceleration
  static const double SD_A_component_filter = 2.0; // m s^-2
  static const double SD_A_component = SD_A_component_filter; // for simulation
  // Standard deviation of angular acceleration
  static const double SD_alpha_component_filter = 3.0;  // rad s^-2
  static const double SD_alpha_component = SD_alpha_component_filter; //for sim

  // Redefined virtual functions
  int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t);

  int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  int func_xp(Hor_Matrix *xv);

  int func_dxp_by_dxv(Hor_Matrix *xv);

  int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, double delta_t);

  int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef);

  int func_xvnorm_and_dxvnorm_by_dxv(Hor_Matrix *xv);

 protected:

  // Matrices for calculations
  Hor_Matrix *rold, *qold, *vold, *omegaold;
  Hor_Matrix *rnew, *qnew, *vnew, *omeganew;
  Hor_Matrix *Temp31A, *Temp31B;
  Hor_Matrix *TempqA, *TempqB;
  Hor_Matrix *qwt;
  Hor_Matrix *Temp33A, *Temp33B;
  Hor_Matrix *Temp44A, *Temp44B;
  Hor_Matrix *Temp43A, *Temp43B;
  Hor_Matrix *Pnn;
  Hor_Matrix *dxnew_by_dn, *dxnew_by_dnT;
  Hor_Matrix *Temp_STATE_NOISE;
  Hor_Matrix *xv_noisy;
  
  // Easy access to state blocks: fill matrices r, q, v, omega with
  // values based on state xv
  int extract_r_q_v_omega(Hor_Matrix *xv, Hor_Matrix *r, Hor_Matrix *q,
			  Hor_Matrix *v, Hor_Matrix *omega);
  // The opposite: put r, q, v, omega back into vector xv
  int compose_xv(Hor_Matrix *r, Hor_Matrix *q,
		 Hor_Matrix *v, Hor_Matrix *omega, Hor_Matrix *xvnew);

  // Calculate commonly used Jacobian part dq(omega * delta_t) by domega
  int dqomegadt_by_domega(Hor_Matrix *omega, double delta_t,
			  Hor_Matrix *dqomegadt_by_domega);

  // Ancillary functions: calculate parts of Jacobian dq_by_domega
  // which are repeatable due to symmetry.
  double dq0_by_domegaA(double omegaA, double omega, double delta_t);
  double dqA_by_domegaA(double omegaA, double omega, double delta_t);
  double dqA_by_domegaB(double omegaA, double omegaB, 
			double omega, double delta_t);
};
