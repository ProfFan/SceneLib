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

/// Specific case of the Head class for Escher
class Head_Escher : public Head {
 public:
  /// Constructor
  Head_Escher(double Ii, double Hh)
    : Head(
	   /// Ii and Hh come from outside (defined in head_model)
	   Ii, Hh,
	   /* Pp */ 0.0,
	   /* Cc */ 0.0,
	   /* Nn */ 0.0,
	   /* FkuL */ 600.0,
	   /* FkvL */ 600.0,
	   /* U0L */ 159.0,
	   /* V0L */ 119.0,
	   /* FkuR */ 600.0,
	   /* FkvR */ 600.0,
	   /* U0R */ 159.0,
	   /* V0R */ 119.0)
    {}
};
