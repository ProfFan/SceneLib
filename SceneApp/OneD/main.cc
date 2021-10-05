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

#include "forms.h"
#include "formoned.h"

#include "control.h"

int main(int argc, char *argv[])
{
   FD_formoned *fd_formoned;

   fl_initialize(&argc, argv, 0, 0, 0);
   fd_formoned = create_form_formoned();

   pass_fd_pointer_oned(fd_formoned);
   fl_show_form(fd_formoned->formoned,FL_PLACE_CENTERFREE,FL_FULLBORDER,
		"formoned");

   // Initialise control.cc
   set_up_control();

   // Enter main control loop in control.cc
   main_control_loop();

   return 0;
}
