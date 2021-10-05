/*  SceneApp: applications for sequential localisation and map-building
    Copyright (C) 2000 Andrew Davison
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

// A simple 1D motion model where process noise is proportional to
// distance travelled

class Simple_OneD_Motion_Model : public OneD_Motion_Model
{
 public:
  Simple_OneD_Motion_Model();
  ~Simple_OneD_Motion_Model();

  // Redefined virtual functions
  int func_fv_and_dfv_by_dxv(Hor_Matrix *xv, 
			     Hor_Matrix *u, double delta_t);

  int func_Q(Hor_Matrix *xv, Hor_Matrix *u, double delta_t);

  int func_xp(Hor_Matrix *xv);

  int func_dxp_by_dxv(Hor_Matrix *xv);

  int func_fv_noisy(Hor_Matrix *xv_true, Hor_Matrix *u_true, double delta_t);

  int func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
    (Hor_Matrix *xv, Hor_Matrix *xpdef);

  int navigate_to_waypoint(Hor_Matrix *xv, Hor_Matrix *xv_goal, 
			   Hor_Matrix *u, double delta_t);

 protected:
  // Constants
  const double SD_frac_v_filter;
  const double SD_frac_v;

  // Calculation space matrices
  Hor_Matrix *u_noisy;
};


// Point feature for 1D motion, constant measurement noise model
class OneD_Point_Feature_Measurement_Model : 
                                        public Point_Feature_Measurement_Model 
{
 public:
  // Constructor
  OneD_Point_Feature_Measurement_Model(Motion_Model *m_m);
  ~OneD_Point_Feature_Measurement_Model();

  // Redefined virtual functions
  int func_yipose_and_Pyiyipose(Hor_Matrix *yi, Hor_Matrix *Pyiyi);

  int func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(Hor_Matrix *hi, 
						   Hor_Matrix *xp);

  int func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Hor_Matrix *yi, 
					   Hor_Matrix *xp);

  int func_Ri(Hor_Matrix *hi);

  int func_nui(Hor_Matrix *hi, Hor_Matrix *zi);

  int func_hi_noisy(Hor_Matrix *yi_true, Hor_Matrix *xp_true);

  int func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
		    Hor_Matrix *yi, Hor_Matrix *xp);
  int visibility_test(Hor_Matrix *xp, Hor_Matrix *yi, 
			      Hor_Matrix *xp_true, 
			      Hor_Matrix *hi);

  double selection_score(Hor_Matrix *Si);

  // Constants
  const double SD_h, SD_h_filter;

 protected:
};
