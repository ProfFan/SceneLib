/*  Scene: software for sequential localisation and map-building

    3d.cc
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

// Class for storing 3D tool stuff
#include <general_headers.h>
#include "3d.h"

// Public functions

Three_D_Display::Three_D_Display(Widget button, Display *display) // constructor
{
  threed_label = hor_create_3D ( button, 400, Hor_Grey[0], Red );
  hor_register_3D( display );

  object_label = HOR_ASSOC_START;

  hor_popup_3D ( NULL, threed_label, 
                 oneselect_proc, 
                 procselected_proc,
		 allclear_proc, 
		 onedelete_proc, 
		 allrestore_proc, 
		 NULL );

  // Temporary storage matrices
  Temp31A = hor_mat_alloc(3, 1);
  Temp31B = hor_mat_alloc(3, 1);
  Temp3Dpos = hor_mat_alloc(3, 1);
  Temp33A = hor_mat_alloc(3, 3);


  draw_axes();
  hor_init_3D ( threed_label, HOR_TRUE );

  // Other initialisations
  vehicle_vertex_ptrs[1] = hor_mats_fill(3, 1, -0.3, ROB_HEIGHT, -0.6);
  vehicle_vertex_ptrs[2] = hor_mats_fill(3, 1, 0.3, ROB_HEIGHT, -0.6);
  vehicle_vertex_ptrs[3] = hor_mats_fill(3, 1, -0.3, 0.0, -0.6);
  vehicle_vertex_ptrs[4] = hor_mats_fill(3, 1, 0.3, 0.0, -0.6);
  vehicle_vertex_ptrs[5] = hor_mats_fill(3, 1, -0.3, ROB_HEIGHT, 0.0);
  vehicle_vertex_ptrs[6] = hor_mats_fill(3, 1, 0.3, ROB_HEIGHT, 0.0);
  vehicle_vertex_ptrs[7] = hor_mats_fill(3, 1, -0.3, 0.0, 0.2);
  vehicle_vertex_ptrs[0] = hor_mats_fill(3, 1, 0.3, 0.0, 0.2);

  for(int i=0; i < 8; i++)
    lab_vertex_ptrs[i] = hor_mat_alloc(3, 1);
    
  PP = hor_mat_alloc(3, 3);
}

int Three_D_Display::draw_axes()
{
  object_label = HOR_ASSOC_START;

  // Create axes
  object_item.type = Hor_Item_3D::HOR_LINE_3D;

  object_item.u.line.p1 = hor_mat_alloc (3, 1);
  object_item.u.line.p2 = hor_mat_alloc (3, 1);
 
  hor_matq_fill ( object_item.u.line.p1, -AXLEN, 0.0, 0.0 );    /* X axis */
  hor_matq_fill ( object_item.u.line.p2, AXLEN, 0.0, 0.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, 0.0, -AXLEN, 0.0 );    /* Y axis */
  hor_matq_fill ( object_item.u.line.p2, 0.0, AXLEN, 0.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, 0.0, 0.0, -AXLEN );    /* Z axis */
  hor_matq_fill ( object_item.u.line.p2, 0.0, 0.0, AXLEN );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  /* axis labels */

  /* X */
  hor_matq_fill ( object_item.u.line.p1, AXLEN + 3.0, 0.0, -1.0 );
  hor_matq_fill ( object_item.u.line.p2, AXLEN + 1.0, 0.0, 1.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, AXLEN + 1.0, 0.0, -1.0 );
  hor_matq_fill ( object_item.u.line.p2, AXLEN + 3.0, 0.0, 1.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  /* Y */
  hor_matq_fill ( object_item.u.line.p1, 0.0, AXLEN + 1.0, 0.0 );
  hor_matq_fill ( object_item.u.line.p2, 0.0, AXLEN + 2.0, 0.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, -1.0, AXLEN + 3.0, 0.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, 1.0, AXLEN + 3.0, 0.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  /* Z */
  hor_matq_fill ( object_item.u.line.p1, 1.0, 0.0, AXLEN + 3.0 );
  hor_matq_fill ( object_item.u.line.p2, -1.0, 0.0, AXLEN + 3.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p1, 1.0, 0.0, AXLEN + 1.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_matq_fill ( object_item.u.line.p2, -1.0, 0.0, AXLEN + 1.0 );
  hor_3D_item ( threed_label, object_label++, object_item, LightSeaGreen );

  hor_mat_free ( object_item.u.line.p2 );
  hor_mat_free ( object_item.u.line.p1 );

  return 0;
}

int Three_D_Display::draw_true_point(Hor_Matrix *yipose, int label)
{
  draw_point(yipose, label + FIRST_TRUE_FEATURE_ITEM, Yellow);

  return 0;
}

int Three_D_Display::draw_estimated_point(Hor_Matrix *yipose, int label)
{
  draw_point(yipose, label + FIRST_ESTIMATED_FEATURE_ITEM, Green);

  return 0;
}

int Three_D_Display::draw_estimated_selected_point(Hor_Matrix *yipose, int label)
{
  draw_point(yipose, label + FIRST_ESTIMATED_FEATURE_ITEM, Red);

  return 0;
}

int Three_D_Display::draw_point(Hor_Matrix *yipose, int label, u_long colour)
{
  object_item.type = Hor_Item_3D::HOR_POINT_3D;  
  object_label = (Hor_Assoc_Label) label;
  object_item.u.point.p = yipose;

  hor_3D_item (threed_label, object_label, object_item, colour);

  return 0;
}

int Three_D_Display::draw_true_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose)
{
  draw_robot(xpose, Rpose, FIRST_TRUE_VEHICLE1_ITEM, Yellow);

  return 0;
}

int Three_D_Display::draw_estimated_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose)
{
  draw_robot(xpose, Rpose, FIRST_ESTIMATED_VEHICLE1_ITEM, Green);

  return 0;
}

int Three_D_Display::draw_true_robot2(Hor_Matrix *xpose, Hor_Matrix *Rpose)
{
  draw_robot(xpose, Rpose, FIRST_TRUE_VEHICLE2_ITEM, Yellow);

  return 0;
}

int Three_D_Display::draw_estimated_robot2(Hor_Matrix *xpose, Hor_Matrix *Rpose)
{
  draw_robot(xpose, Rpose, FIRST_ESTIMATED_VEHICLE2_ITEM, Green);

  return 0;
}

int Three_D_Display::draw_robot(Hor_Matrix *xpose, Hor_Matrix *Rpose, 
				int start_label, u_long colour)
{
  Hor_Assoc_Label use_label = start_label;

  for(int i=0; i < 8; i++) 
  {
    hor_matq_prod2( Rpose, vehicle_vertex_ptrs[i], lab_vertex_ptrs[i] );
    hor_matq_add2( xpose, lab_vertex_ptrs[i], lab_vertex_ptrs[i] );
  }

  object_item.type = Hor_Item_3D::HOR_LINE_3D;    

  /* 12 lines in vehicle */

  object_item.u.line.p1 = lab_vertex_ptrs[1];
  object_item.u.line.p2 = lab_vertex_ptrs[2];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[2];
  object_item.u.line.p2 = lab_vertex_ptrs[4];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[4];
  object_item.u.line.p2 = lab_vertex_ptrs[3];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[3];
  object_item.u.line.p2 = lab_vertex_ptrs[1];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[2];
  object_item.u.line.p2 = lab_vertex_ptrs[6];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[4];
  object_item.u.line.p2 = lab_vertex_ptrs[0];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[3];
  object_item.u.line.p2 = lab_vertex_ptrs[7];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[1];
  object_item.u.line.p2 = lab_vertex_ptrs[5];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[5];
  object_item.u.line.p2 = lab_vertex_ptrs[6];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[6];
  object_item.u.line.p2 = lab_vertex_ptrs[0];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[0];
  object_item.u.line.p2 = lab_vertex_ptrs[7];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  object_item.u.line.p1 = lab_vertex_ptrs[7];
  object_item.u.line.p2 = lab_vertex_ptrs[5];
  hor_3D_item ( threed_label, use_label++, object_item, colour);

  return 0;
}

int Three_D_Display::draw_estimated_point_covariance(Hor_Matrix *yipose, 
						     Hor_Matrix *Pyiyipose, 
						     int label)
{
  draw_point_covariance(yipose, Pyiyipose, 
	     FIRST_COVARIANCE_ITEM + MAX_OBJECTS_IN_COVARIANCE * label, Green);

  return 0;
}

int Three_D_Display::draw_estimated_selected_point_covariance(Hor_Matrix *yipose, 
						     Hor_Matrix *Pyiyipose, 
						     int label)
{
  draw_point_covariance(yipose, Pyiyipose, 
     FIRST_COVARIANCE_ITEM + MAX_OBJECTS_IN_COVARIANCE * label, Red);

  return 0;
}

int Three_D_Display::draw_point_covariance(Hor_Matrix *yipose, 
				     Hor_Matrix *Pyiyipose, 
				     int start_label, u_long colour)
{  
  object_item.type = Hor_Item_3D::HOR_LINE_3D;

  hor_matq_copy(Pyiyipose, PP);

  hor_matq_tred2(PP, eigenvalues, spare_array);
  hor_matq_tqli(eigenvalues, spare_array, PP);
    
  hor_matq_eigsrt(eigenvalues, PP);

  object_item.u.line.p1 = Temp31A;
  object_item.u.line.p2 = Temp31B;

  /* Draw lines for the three axes */
  for (j = 0; j < 3; j++)    /* j counts from 0 to 2 between the 
				different eigenvalues */
  {
    axis_length = NO_SIGMA_SHOW * sqrt(eigenvalues[j]);


    covariance_label = (Hor_Assoc_Label) start_label + j;

    hor_matq_fill (Temp31A, 
		   matel(yipose, 1, 1) + axis_length * matel(PP, 1, j + 1),
		   matel(yipose, 2, 1) + axis_length * matel(PP, 2, j + 1),
		   matel(yipose, 3, 1) + axis_length * matel(PP, 3, j + 1));
    hor_matq_fill (Temp31B, 
		   matel(yipose, 1, 1) - axis_length * matel(PP, 1, j + 1),
		   matel(yipose, 2, 1) - axis_length * matel(PP, 2, j + 1),
		   matel(yipose, 3, 1) - axis_length * matel(PP, 3, j + 1));

    hor_3D_item (threed_label, covariance_label, object_item, 
		 colour);
  }

  return 0;
}

int Three_D_Display::remove_estimated_point(int label)
{
  remove_point(label + FIRST_ESTIMATED_FEATURE_ITEM);

  return 0;
}

int Three_D_Display::remove_point(int label)
{
  object_label = (Hor_Assoc_Label) label;
  hor_delete_3D_item (threed_label, object_label);

  return 0;
}

int Three_D_Display::remove_estimated_point_covariance(int label)
{
  remove_point_covariance(FIRST_COVARIANCE_ITEM + 
                                          MAX_OBJECTS_IN_COVARIANCE * label);

  return 0;
}

int Three_D_Display::remove_point_covariance(int start_label)
{
  for (j = 0; j < 3; j++)    /* j counts from 0 to 2 between the 
				different eigenvalues */
  {
    covariance_label = (Hor_Assoc_Label) start_label + j;

    hor_delete_3D_item (threed_label, covariance_label);
  }

  return 0;
}

/**************************Non-Member Functions*******************************/

void oneselect_proc (Hor_Assoc_Label label, Hor_Item_3D item )
{
  int i = MAX_FEATURES; /* Will hold the feature number if we find one 
			   Initialise to MAX_FEATURES because this will
			   never correspond to a real feature */

  switch ( item.type )
  {
  case HOR_LINE_3D:
  {
    Hor_Matrix *p1 = item.u.line.p1, *p2 = item.u.line.p2;

    cout << "Selected line: "
         << p1->m[0][0] << ", " << p1->m[1][0] << ", " << p1->m[2][0] << " -> "
         << p2->m[0][0] << ", " << p2->m[1][0] << ", " << p2->m[2][0] << endl;

    if (label < FIRST_COVARIANCE_ITEM || 
	label > FIRST_COVARIANCE_ITEM + 
                                     MAX_OBJECTS_IN_COVARIANCE * MAX_FEATURES)
    {
      cout << "Not a feature line." << endl;
      break;
    }

    cout << "Estimated feature " 
         << (i = (label - FIRST_COVARIANCE_ITEM) 
                                        / MAX_OBJECTS_IN_COVARIANCE) << endl;
    estimated_feature_selected(i);

    break;
  }
      
  case HOR_POINT_3D:
  {
    Hor_Matrix *p = item.u.point.p;

    cout << "Selected point: "
         << p->m[0][0] << ", " << p->m[1][0] << ", " << p->m[2][0] << endl
         << "Label is " << label << "." << endl;

    if (label >= FIRST_ESTIMATED_FEATURE_ITEM &&
        label < FIRST_ESTIMATED_FEATURE_ITEM + MAX_FEATURES)
    {
      cout << "Estimated feature " 
	   << (i = label - FIRST_ESTIMATED_FEATURE_ITEM) << endl;
      estimated_feature_selected(i);
    }
    else if (label >= FIRST_TRUE_FEATURE_ITEM && 
             label < FIRST_TRUE_FEATURE_ITEM + MAX_FEATURES)
    {
      cout << "True feature " << (i = label - FIRST_TRUE_FEATURE_ITEM) 
           << endl;
      true_feature_selected(i);
    }
    else
    {
      cout << "Not a feature point." << endl;
      break;
    } 

    break;
  }
    
  case HOR_FACET_3D:
    cout << "Facet selected." << endl;
    break;

  case HOR_LABEL_3D:
    break;
  }
}

/*************************These functions do nothing**************************/

void procselected_proc ( Hor_Assoc_List )
{
  cout << "Nothing to see here." << endl;
}
         
void allclear_proc ( Hor_Assoc_List )
{
  cout << "Nothing to see here." << endl;
}

void onedelete_proc (Hor_Assoc_Label, Hor_Item_3D )
{
  cout << "Nothing to see here." << endl;
}

void allrestore_proc ( Hor_Assoc_List )
{
  cout << "Nothing to see here." << endl;
}

