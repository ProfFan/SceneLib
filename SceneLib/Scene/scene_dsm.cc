/*  Scene: software for sequential localisation and map-building
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
#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"
#include "scene_dsm.h"

Scene_DSM_Submap::Scene_DSM_Submap(Hor_Matrix *initial_xv,  Hor_Matrix *initial_Pxx,
				   Motion_Model *m_m,
       			   Hor_Matrix *b_box_xp1, Hor_Matrix *b_box_xp2,
				   int sm_number)
  : Scene_Single(initial_xv, initial_Pxx, m_m)
{
  constructor_bookkeeping(b_box_xp1, b_box_xp2, sm_number);
}

Scene_DSM_Submap::Scene_DSM_Submap(Hor_Matrix *initial_xv,  Hor_Matrix *initial_Pxx,
				   Motion_Model *m_m,
       			   Hor_Matrix *b_box_xp1, Hor_Matrix *b_box_xp2,
				   int sm_number,
				   Internal_Measurement_Model *i_m_m)
  : Scene_Single(initial_xv, initial_Pxx, m_m, i_m_m)
{
  constructor_bookkeeping(b_box_xp1, b_box_xp2, sm_number);
}

int Scene_DSM_Submap::constructor_bookkeeping(Hor_Matrix *b_box_xp1, 
					      Hor_Matrix *b_box_xp2,
					      int sm_number)
{
  submap_number = sm_number;
  next_submap_pointer = NULL;

  bounding_box_xp1 = hor_mats_copy(b_box_xp1);
  bounding_box_xp2 = hor_mats_copy(b_box_xp2);  

  return 0;
}

Scene_DSM::Scene_DSM(Hor_Matrix *initial_xv, 
		     Hor_Matrix *initial_Pxx,
		     Motion_Model *m_m,
		     Hor_Matrix *b_b_size)
{
  constructor_bookkeeping(initial_xv, initial_Pxx, m_m, b_b_size);
}

Scene_DSM::Scene_DSM(Hor_Matrix *initial_xv, 
		     Hor_Matrix *initial_Pxx,
		     Motion_Model *m_m, Hor_Matrix *b_b_size,
		     Internal_Measurement_Model *i_m_m)
{
  internal_measurement_model = i_m_m;

  constructor_bookkeeping(initial_xv, initial_Pxx, m_m, b_b_size);
}

int Scene_DSM::constructor_bookkeeping(Hor_Matrix *initial_xv, 
		     Hor_Matrix *initial_Pxx,
		     Motion_Model *m_m,
		     Hor_Matrix *b_b_size)
{
  motion_model = m_m;

  bounding_box_size = hor_mat_alloc(motion_model->POSITION_STATE_SIZE, 1);
  hor_matq_copy(b_b_size, bounding_box_size);
  motion_model->func_xp(initial_xv);
  initial_xp = hor_mats_copy(motion_model->xpRES);

  no_of_submaps = 0;

  // Start first submap
  add_new_submap(initial_xv, initial_Pxx);

  first_submap_pointer = current_submap_pointer;
  last_submap_pointer = current_submap_pointer;

  return 0;
}

int Scene_DSM::add_new_submap(Hor_Matrix *initial_xv, 
			  Hor_Matrix *initial_Pxx)
{
  Hor_Matrix *bounding_box_xp1 = 
                      hor_mat_alloc(motion_model->POSITION_STATE_SIZE, 1);
  Hor_Matrix *bounding_box_xp2 = 
                      hor_mat_alloc(motion_model->POSITION_STATE_SIZE, 1);

  motion_model->func_xp(initial_xv);
  find_bounding_box_of_position(motion_model->xpRES, bounding_box_xp1,
				bounding_box_xp2);

  Scene_DSM_Submap *new_submap;

  if (internal_measurement_model == NULL)
    new_submap = new Scene_DSM_Submap(initial_xv, initial_Pxx,
				      motion_model, bounding_box_xp1,
				      bounding_box_xp2, no_of_submaps);
  else
    new_submap = new Scene_DSM_Submap(initial_xv, initial_Pxx,
				      motion_model, bounding_box_xp1,
				      bounding_box_xp2, no_of_submaps,
				      internal_measurement_model);

  if (last_submap_pointer != NULL)
    last_submap_pointer->next_submap_pointer = new_submap;
  current_submap_pointer = new_submap;

  last_submap_pointer = new_submap;

  cout << "Adding new submap " << no_of_submaps << ". Bounding box parameters:"
       << endl << "Bottom-left:" << bounding_box_xp1 << "Top-right:"
       << bounding_box_xp2 << endl;

  no_of_submaps++;

  hor_mat_free_list(bounding_box_xp1, bounding_box_xp2, NULL);

  return 0;
}

int Scene_DSM::find_bounding_box_of_position(Hor_Matrix *xp, 
				    Hor_Matrix *bounding_box_xp1,
				    Hor_Matrix *bounding_box_xp2)
{
  //  cout << "xp" << xp << "initial_xp" << initial_xp 
  //       << "bounding_box_size" << bounding_box_size << endl;

  for(int r = 1; r <= motion_model->POSITION_STATE_SIZE; r++)
  {
    double x = fabs(matel(xp, r, 1) - matel(initial_xp, r, 1))
      + matel(bounding_box_size, r, 1) / 2.0;

    int t = (int) (x / matel(bounding_box_size, r, 1));

    double xmin = t * matel(bounding_box_size, r, 1) 
                    - matel(bounding_box_size, r, 1) / 2.0;
    double xmax = xmin + matel(bounding_box_size, r, 1);

    if (matel(xp, r, 1) < 0)
    {
      double temp = xmin;
      xmin = -xmax;
      xmax = -temp;
    }

    matel(bounding_box_xp1, r, 1) = xmin + matel(initial_xp, r, 1);
    matel(bounding_box_xp2, r, 1) = xmax + matel(initial_xp, r, 1);
  }

  //  cout << "bounding_box_xp1" << bounding_box_xp1 
  //       << "bounding_box_xp2" << bounding_box_xp2;

  return 0;
}

int Scene_DSM::check_submap()
{
  Hor_Matrix *xptemp;
  motion_model->func_xp(get_xv());
  xptemp = hor_mats_copy(motion_model->xpRES);

  if (!motion_model->bounding_box_test(
				  current_submap_pointer->bounding_box_xp1, 
				  current_submap_pointer->bounding_box_xp2,
				  xptemp))
  // We are out of the current bounding box
  {
    Scene_DSM_Submap *submap_pointer = find_known_submap(xptemp);

    if (submap_pointer == NULL) 
    {
      // Need to add new submap
      add_new_submap(get_xv(), get_Pxx());
    }
    else
    {
      cout << "Re-entering submap " << submap_pointer->submap_number << endl;

      if (submap_pointer->submap_number > 
	  current_submap_pointer->submap_number)
	cross_map_updating(current_submap_pointer, submap_pointer);
      else
	cross_map_relocation(current_submap_pointer, submap_pointer);

      current_submap_pointer = submap_pointer;
    }

  }

  hor_mat_free(xptemp);

  return 0;
}

Scene_DSM_Submap *Scene_DSM::find_known_submap(Hor_Matrix *xp)
{
  Scene_DSM_Submap *submap;

  for (submap = first_submap_pointer; submap; 
       submap = submap->next_submap_pointer)
  {
    //    cout << "Testing submap number " << submap->submap_number << endl;
    if (motion_model->bounding_box_test(submap->bounding_box_xp1, 
					submap->bounding_box_xp2,
					xp))
      return submap;
  }

  // Return NULL if not found
  return NULL;

}

int Scene_DSM::vehicle_state_has_been_changed()
{
  check_submap();

  return 0;
}

int Scene_DSM::cross_map_relocation(Scene_DSM_Submap *submapA, 
				    Scene_DSM_Submap *submapB)
{
  cout << "Cross-map relocation for transfer from submap " 
       << submapA->submap_number << " to submap " << submapB->submap_number 
       << endl;

  int sizeB = submapB->get_total_state_size();
  Hor_Matrix *xB = hor_mat_alloc(sizeB, 1);
  Hor_Matrix *PB = hor_mat_alloc(sizeB, sizeB);

  submapB->construct_total_state_and_covariance(xB, PB);

  hor_matq_insert_chunky1(submapA->get_xv(), xB, 0);

  Hor_Matrix *PxxAplusPxxB = hor_mats_add2(submapA->get_Pxx(),
					   submapB->get_Pxx());

  hor_matq_insert_chunkyx(PxxAplusPxxB, PB, 0, 0);

  submapB->fill_state_and_covariance(xB, PB);

  hor_mat_free_list(xB, PB, PxxAplusPxxB, NULL);

  return 0;
}



int Scene_DSM::cross_map_updating(Scene_DSM_Submap *submapA, 
				  Scene_DSM_Submap *submapB)
{
  cout << "Cross-map updating for transfer from submap " 
       << submapA->submap_number << " to submap " << submapB->submap_number 
       << endl;

  cerr << "Not yet implemented: using cross-map relocation." 
       << endl;
  cross_map_relocation(submapA, submapB);

  return 0;
}

