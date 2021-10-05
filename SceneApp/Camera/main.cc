/*  SceneApp: applications for sequential localisation and map-building

    Camera/main.cc
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

#include "forms.h"
#include "formcom.h"

#ifdef _ROBOT_
#include "formrob.h"
#else
#include "formsim.h"
#endif

#include "control.h"

int main(int argc, char *argv[])
{
   FD_formcom *fd_formcom;

#ifdef _ROBOT_
   FD_formrob *fd_formrob;
#else
   FD_formsim *fd_formsim;
#endif

   fl_initialize(&argc, argv, 0, 0, 0);

   fd_formcom = create_form_formcom();
#ifdef _ROBOT_
   fd_formrob = create_form_formrob();
#else
   fd_formsim = create_form_formsim();
#endif


   pass_fd_pointer_com(fd_formcom);
#ifdef _ROBOT_
   pass_fd_pointer_rob(fd_formrob);
#else
   pass_fd_pointer_sim(fd_formsim);
#endif

   // Show the main forms
   fl_show_form(fd_formcom->formcom,FL_PLACE_CENTERFREE,FL_FULLBORDER,
		"formcom");
#ifdef _ROBOT_
   fl_show_form(fd_formrob->formrob,FL_PLACE_CENTERFREE,FL_FULLBORDER,
		"formrob");
#else
   fl_show_form(fd_formsim->formsim,FL_PLACE_CENTERFREE,FL_FULLBORDER,
		"formsim");
#endif

   // Initialise control.cc
   set_up_control(argc, argv);

   // Enter main control loop in control.cc
   main_control_loop();

   return 0;
}
