/*  Scene: software for sequential localisation and map-building

    kalman_form.h
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

// Header file for the module containing callbacks for the kalman parameter
// form.
// The form is just a way of interfacing to filter options in the kalman class

void pass_all_ptrs (Kalman *, Scene *, Three_D_Display *, Sim_Or_Rob *,
		    Scene_Inspect *, FD_formkalman *);

void update_performance_info ();
