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

/** Model class for Escher head: just sets constants for general head model */

class Escher_Head_Point_Feature_Measurement_Model 
#ifdef _THREED_
  : public ThreeD_Head_Point_Feature_Measurement_Model
#else
  : public TwoD_Head_Point_Feature_Measurement_Model
#endif
{
 public:
  /// Constructor
  Escher_Head_Point_Feature_Measurement_Model(Motion_Model *m_m)
#ifdef _THREED_
    : ThreeD_Head_Point_Feature_Measurement_Model(
#else
    : TwoD_Head_Point_Feature_Measurement_Model(
#endif
      m_m,
      /* Ii */ 0.180,
      /* Hh */ 1.305,
      /* PAN_RANGE_LIMIT */ M_PI * 90.0 / 180.0,
      /* ELEVATION_RANGE_LIMIT */ M_PI * 40.0 / 180.0,
      /* VERGENCE_RANGE_LIMIT */ M_PI * 45.0 / 180.0,
      /* MAXIMUM_LENGTH_RATIO */ 1.4,
      /* MAXIMUM_ANGLE_DIFFERENCE */ M_PI * 45.0 / 180.0,
      /* SD_alpha */ 0.002,
      /* SD_e */ 0.01,
      /* SD_gamma */ 0.002,
      /* SD_alpha_filter */ 0.002,
      /* SD_e_filter */ 0.01,
      /* SD_gamma_filter */ 0.002)
    {}
};
