/*  Scene: software for sequential localisation and map-building

    3d.h
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

// Class wrapper for Horatio 3D tool

// These functions must exist elsewhere to be called back from here.
int true_feature_selected(int i);
int estimated_feature_selected(int i);

const int MAX_FEATURES = 10000000;   /* The maximum number of features we can 
					keep information on at a time 
					(only a limit because of the labelling
					in the 3D tool) */
const double NO_SIGMA_SHOW = 3.0;  /* The number of standard deviations 
				      we want to represent in the 
				      representations of covariance ellipses 
				      we draw in the 3D tool */
const double ROB_HEIGHT = 0.5;     /* Height of representation of robot */
extern u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, 
       thistle, Cyan; 
enum {HOR_LINE_3D, HOR_POINT_3D, HOR_FACET_3D, HOR_LABEL_3D};

const double AXLEN = 4.0;      /* constant denoting extent of xyz axes drawn */

// We allow two true vehicles and two estimated ones
/* The first 20 available labels are used for the axes */
const int ITEMS_FOR_VEHICLE = 15;
const int FIRST_TRUE_VEHICLE1_ITEM = 20; 
                                 /* First 3D Tool item for the first true 
				    vehicle */
const int FIRST_ESTIMATED_VEHICLE1_ITEM = FIRST_TRUE_VEHICLE1_ITEM + 
                                          ITEMS_FOR_VEHICLE;
const int FIRST_TRUE_VEHICLE2_ITEM = FIRST_ESTIMATED_VEHICLE1_ITEM + 
                                     ITEMS_FOR_VEHICLE;
const int FIRST_ESTIMATED_VEHICLE2_ITEM = FIRST_TRUE_VEHICLE2_ITEM + 
                                          ITEMS_FOR_VEHICLE;

const int FIRST_TRUE_FEATURE_ITEM = FIRST_ESTIMATED_VEHICLE2_ITEM + 
                  ITEMS_FOR_VEHICLE;  /* First item for true scene points */
const int FIRST_ESTIMATED_FEATURE_ITEM 
                      = (FIRST_TRUE_FEATURE_ITEM + MAX_FEATURES); 
                               /* First item for estimated scene points */
const int FIRST_COVARIANCE_ITEM = 
                           (FIRST_ESTIMATED_FEATURE_ITEM + MAX_FEATURES);
                               /* Need three lines per feature to represent
				  the uncertainty */
const int MAX_OBJECTS_IN_COVARIANCE = 3;  /* This may change with new 
					     feature types */

class Three_D_Display
{
public:
  Three_D_Display(Widget button, Display *display);    // constructor

  int draw_axes();

  // Functions for robot moving in 2D
  int draw_true_point(Hor_Matrix *yipose, int label);
  int draw_estimated_point(Hor_Matrix *yipose, int label);
  int draw_estimated_selected_point(Hor_Matrix *yipose, int label);

  int draw_true_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose);
  int draw_estimated_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose);
  int draw_true_robot2(Hor_Matrix *xpose, Hor_Matrix *Rpose);
  int draw_estimated_robot2(Hor_Matrix *xpose, Hor_Matrix *Rpose);

  int draw_estimated_point_covariance(Hor_Matrix *yipose, 
				      Hor_Matrix *Pyiyipose, int label);
  int draw_estimated_selected_point_covariance(Hor_Matrix *yipose, 
				      Hor_Matrix *Pyiyipose, int label);
  int remove_estimated_point(int label);
  int remove_point(int label);
  int remove_estimated_point_covariance(int label);

 private:
  int draw_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose, 
		 int start_label, u_long colour);
  int draw_point(Hor_Matrix *yipose, int label, u_long colour);
  int remove_point_covariance(int start_label);
  int draw_point_covariance(Hor_Matrix *yipose, Hor_Matrix *Pyiyipose, 
                      int start_label, u_long colour);

private:
  Hor_Assoc_Label threed_label;

  // Store these things here so we just have one version per 3D object
  Hor_Matrix *vehicle_vertex_ptrs[8];
  Hor_Matrix *lab_vertex_ptrs[8];

  Hor_Assoc_Label object_label;
  Hor_Item_3D object_item;

  // Stuff for drawing point covariances
  int j;
  double axis_length;

  Hor_Matrix *PP;
  Hor_Matrix *Temp31A, *Temp31B;
  Hor_Matrix *Temp3Dpos;
  Hor_Matrix *Temp33A;

  double eigenvalues[3];
  double spare_array[3];

  Hor_Assoc_Label  covariance_label;
};

// Nonmember functions
void oneselect_proc (Hor_Assoc_Label label, Hor_Item_3D item );
void procselected_proc ( Hor_Assoc_List item_list );
void allclear_proc ( Hor_Assoc_List item_list );
void onedelete_proc ( Hor_Assoc_Label label, Hor_Item_3D item );
void allrestore_proc ( Hor_Assoc_List item_list );



