/*  Scene: software for sequential localisation and map-building

    waypoint_browser.cc
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
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
#include "models_double.h"
#include "waypoints.h"
#include "forms.h"
#include "formbrowser.h"
#include "waypoint_browser.h"

/*******************************Waypoint Browser******************************/

// Form for editing waypoints
FD_formbrowser *fd_formbrowser;

char waypoint_string[1000];
char temp_string[100];

static Waypoints *waypoints;

int display_waypoints()
{
  fl_clear_browser(fd_formbrowser->waypoint_browser);

  for (int i = 0; i < waypoints->get_no_of_waypoints(); i++)
  {
    sprintf(waypoint_string, "%3d ", i);
    for (int r = 0; r < waypoints->get_motion_model()->STATE_SIZE; r++)
    {
      sprintf(temp_string, "%5.2f ", matel((*waypoints)[i], r + 1, 1));
      strcat(waypoint_string, temp_string);
    }

    fl_addto_browser(fd_formbrowser->waypoint_browser, waypoint_string);
  }

  return 0;
}

int initialise_waypoint_browser(Waypoints *wp)

{
  waypoints = wp;

  // Form for editing waypoints
  fd_formbrowser = create_form_formbrowser();
  fl_show_form(fd_formbrowser->formbrowser,
	       FL_PLACE_CENTERFREE,FL_FULLBORDER,"formbrowser");
  
  fl_set_browser_fontsize(fd_formbrowser->waypoint_browser, FL_SMALL_SIZE);
  fl_set_browser_fontstyle(fd_formbrowser->waypoint_browser, FL_FIXED_STYLE);

  display_waypoints();
  highlight_waypoint_browser_line();

  return 0;
}

int highlight_waypoint_browser_line()
{
  fl_select_browser_line(fd_formbrowser->waypoint_browser, 
			 waypoints->get_current_waypoint() + 1);

  return 0;
}

/* callbacks and freeobj handles for form browser */
void waypoint_selected(FL_OBJECT *ob, long data)
{
  waypoints->set_current_waypoint(
		     fl_get_browser(fd_formbrowser->waypoint_browser) - 1);
}

void delete_waypoint(FL_OBJECT *ob, long data)
{
  if (waypoints->get_no_of_waypoints() == 0)
  {
    cout << "No waypoints to delete." << endl;
    return;
  }


  waypoints->delete_current_waypoint();

  display_waypoints();
  highlight_waypoint_browser_line();
}

int read_waypoint_string(Hor_Matrix *&new_waypoint)
{
  strcpy(waypoint_string, fl_get_input(fd_formbrowser->input_waypoint));

  char *cptr = waypoint_string;
  int nnn;  // Counts characters read for each number
  int r = 0;

  new_waypoint = hor_mat_alloc(waypoints->get_motion_model()->STATE_SIZE, 1);

  while(r < waypoints->get_motion_model()->STATE_SIZE && 
	sscanf(cptr, "%lf %n", &(matel(new_waypoint, r + 1, 1)), &nnn) > 0)
  {
    cptr += nnn;
    r++;
  }

  if (r != waypoints->get_motion_model()->STATE_SIZE)
  {
    cerr << "Wrong input format: waypoint not added." << endl;
    hor_mat_free(new_waypoint);
    return 0;
  }

  return 1;
}

void add_waypoint_before(FL_OBJECT *ob, long data)
{
  Hor_Matrix *new_waypoint;
  if (!read_waypoint_string(new_waypoint))
    return;

  waypoints->add_waypoint_before_current(new_waypoint);

  display_waypoints();
  highlight_waypoint_browser_line();
}

void add_waypoint_after(FL_OBJECT *ob, long data)
{
  Hor_Matrix *new_waypoint;
  if (!read_waypoint_string(new_waypoint))
    return;

  waypoints->add_waypoint_after_current(new_waypoint);

  display_waypoints();
  highlight_waypoint_browser_line();
}

