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

#include <general_headers.h>

// View simulator Related

#include "forms.h"
#include "formrob.h"
#include "grab.h"

#include <OB/CORBA.h>
#include <OB/Util.h>
#include <OB/CosNaming.h>
#include <OB/CosEventChannelAdmin.h>
#include <OB/CosEventComm_skel.h>
#include <OB/Reactor.h>
#include "VisionSimulator.h"
#include "FishEye.h"
#include "modsim.h"
#include "common.h"





#include "module.h"
#include "head.h"
#include "head_escher.h"

#include "improc.h"
#include "correlate.h"





#include "models_base.h"
#include "models_nomad.h"
#include "models_head.h"

#include "sim_or_rob.h"
#include "robot.h"

/*******************************Robot Class***********************************/

// Constructor

Robot::Robot(FD_formrob *fd_fr, int zero_axes_flag, int reduced_im_flag,
	     Head_Point_Feature_Measurement_Model 
	     *h_p_f_m_m,
	     const double Ii, const double Hh,
	     int argc, char **argv)
  : Sim_Or_Rob("ROBOT"),
    fd_formrob(fd_fr),
    head_point_feature_measurement_model(h_p_f_m_m),
    reduced_image_flag(reduced_im_flag)
{
  // Initialize View Simulator
  connect_simulator(argc, argv);
  init_simulator();
  //  set_view_angle((CORBA_Float)32.6, (CORBA_Float)32.6);
  set_view_angle((CORBA_Float)30.0, (CORBA_Float)30.0);

  // Initialise Head class
  // Head_Escher is specific to Escher, but we can point to it as if it is
  // its generic base class Head
  Head_Escher *head_escher = new Head_Escher(Ii, Hh);
  head = head_escher;

  // Init Meteor / image display (in grab.cc)
  init_grabbing(fd_formrob, image, &uL, &vL, &uR, &vR, BOXSIZE);

  // Connect to head
  system("/data/cas1/Nomad/TestRobot/escher");  /* Run script to launch 
						   escher windows */
  init_escher();

  // Connect to robot
  init_nomad(CASNOMAD1_IP); // IP address for casnomad1
  if (zero_axes_flag)
  {
    cout << "Zeroing Nomad's turret and steering axes: please wait." << endl;
    zero_nomad_axes();

    // usleep(5000000);
    // cout << "Centering turret." << endl;

    // Move so that Escher faces forwards
    // increment_vehicle_turret_angle(TURRET_OFFSET);
  }

#ifdef _THREED_
  // Initialise roll/pitch sensor
  init_gyro();
  reset_gyro();
#endif

  // Initialise head location parameters
  alpha = 0.0; e = 0.0; gammaL = 0.0; gammaR = 0.0;
  alpha_note = 0.0; e_note = 0.0; gammaL_note = 0.0; gammaR_note = 0.0;

  // Initialise image markers to impossible values
  uL = -1; vL = -1; uR = -1; vR = -1;
  
  cout << "Escher initialised to:" << endl;
  print_head_angles();

  // Allocate image patches
  left_patch = hor_alloc_image(BOXSIZE, BOXSIZE, HOR_U_CHAR, NULL);
  right_patch = hor_alloc_image(BOXSIZE, BOXSIZE, HOR_U_CHAR, NULL);
 
  // Vector for point feature position
  hL = hor_mat_alloc(3, 1);

  // Ellipse matrix for searching when we should be locked on
  PuInvLock = hor_mats_fill(2, 2,
			 1.0 / (LOCKSEARCHSIGMA * LOCKSEARCHSIGMA), 0.0,
			 0.0, 1.0 / (LOCKSEARCHSIGMA * LOCKSEARCHSIGMA));

  PuInvL = hor_mat_alloc(2, 2);
  PuInvR = hor_mat_alloc(2, 2);

  timing_OK_counter = 0;

  Temp31 = hor_mat_alloc(3, 1);
  MC0G = hor_mat_alloc(3, 3);
  MC01 = hor_mat_alloc(3, 3);
  MC1G = hor_mat_alloc(3, 3);
  unit = hor_mat_alloc(3, 1);
}

Robot::~Robot()
{
  // Close down communications with hardware
#ifdef _THREED_
  close_gyro();
#endif
  close_simulator();
  close_escher();
  close_nomad();
  close_grabbing();
}

char *known_point_patch_stem = "known_patch";

// Initialise known feature
void *Robot::initialise_known_feature(Feature_Measurement_Model *f_m_m,
			       Hor_Matrix *yi,
			       int known_feature_label)
{
  // Get identifying feature patch from file
  // So far only handle points
  if(strcmp("HEAD_POINT", f_m_m->feature_type) != 0)
  {
    cerr << "Known point not a HEAD_POINT: passing." << endl;
    return NULL;
  }

  char name[100];
  char index[10];

  strcpy(name, known_point_patch_stem);
  sprintf(index, "%d", known_feature_label);

  strcat(name, index);

  return (void *) hor_read_image(name);
}

int Robot::make_initial_measurement_of_feature(Hor_Matrix *z, void *&id, 
					 Feature_Measurement_Model *f_m_m)
{
  if (strcmp(f_m_m->feature_type, "HEAD_POINT") == 0)
  {
    id = (void *) initialise_point_feature(z);
    if (id != NULL)
      return 0;
    else
      return -1;
  }
  // .... insert other feature cases here
  else
  {
    cerr << "Do not know how to measure feature type " << f_m_m->feature_type
	 << endl;
    return -1;
  }

}

#ifdef _THREED_

// Average "zero" values from roll-pitch sensor: subtract these from 
// all readings
const float roll_zero_degrees = -0.9;
const float pitch_zero_degrees = -1.2;

// Make measurement from pan/tilt sensor
int Robot::make_internal_measurement(Internal_Measurement_Model 
             *internal_measurement_model, Hor_Matrix *zv, Hor_Matrix *hv, 
	     Hor_Matrix *Sv)
{
  assert(strcmp(internal_measurement_model->internal_type, "ROLL_PITCH") == 0);
  
  float roll, pitch;
  get_gyro(&roll, &pitch);

  roll -= roll_zero_degrees;
  pitch -= pitch_zero_degrees;

  // Convert degrees from sensor to radians
  hor_matq_fill(zv,
		(double) (DEG2RAD(roll)),
		(double) (DEG2RAD(pitch)));

  cout << "Predicted roll/pitch measurement:" << hv
       << "Actual roll/pitch measurement:" << zv;

  return 0;
}
#endif

// Little service function to copy image region centred at u, v into patch
void copy_into_patch(Hor_Image *im, Hor_Image *patch, int u, int v)
{
  for(int r = 0; r < BOXSIZE; r++)        
    for(int c = 0; c < BOXSIZE; c++)
      patch->array.uc[r][c] = 
	    im->array.uc[r + v - (BOXSIZE - 1) / 2][c + u - (BOXSIZE - 1) / 2];
}

/*****************************Head Control Functions**************************/

int Robot::print_head_angles()
{
  cout << "Pan = " << alpha * 180.0 / M_PI 
       << " degrees, Elevation = " << e * 180.0 / M_PI 
       << " degrees." << endl;
  return 0;
}

int Robot::get_head_angles(double *alpha_ptr, double *e_ptr, 
			   double *gammaL_ptr, double *gammaR_ptr)
{
  *alpha_ptr = alpha;
  *e_ptr = e;
  *gammaL_ptr = gammaL;
  *gammaR_ptr = gammaR;

  return 0;
}

// How much we are going to overmove the elevation axis by to try to get
// it more accurate
const double ELEVATION_ADJUST_POSITIVE = 1.055;
const double ELEVATION_ADJUST_NEGATIVE = 1.02;
const double ELEVATION_OFFSET = DEG2RAD(1.0);

// Change elevation commands slightly to account for head errors
double elevation_adjust(double e_unadjusted)
{
  return (e_unadjusted >= 0 ? 
	  e_unadjusted * ELEVATION_ADJUST_POSITIVE + ELEVATION_OFFSET : 
	  e_unadjusted * ELEVATION_ADJUST_NEGATIVE + ELEVATION_OFFSET);
}

double elevation_unadjust(double e_adjusted)
{
  return (e_adjusted >= 0 ? 
	  (e_adjusted - ELEVATION_OFFSET) / ELEVATION_ADJUST_POSITIVE : 
	  (e_adjusted - ELEVATION_OFFSET) / ELEVATION_ADJUST_NEGATIVE);  
}

/* Read the current odometry and save it into the head angles
   Should need to use this much because the head will go where it is sent,
   and this is stored in the head angle parameters */
int Robot::read_head_odometry()
{
  static float alpha_deg, e_deg, gammaL_deg, gammaR_deg;

  whereis_escher(&alpha_deg, &e_deg, &gammaL_deg, &gammaR_deg);

  alpha = (double) alpha_deg * M_PI / 180.0;
  e = (double) e_deg * M_PI / 180.0;
  e = elevation_unadjust(e);
  gammaL = (double) gammaL_deg * M_PI / 180.0;
  gammaR = (double) gammaR_deg * M_PI / 180.0;

  return 0;
}

int Robot::move_head()
{
  if (alpha > head_point_feature_measurement_model->PAN_RANGE_LIMIT)
  {
    alpha = head_point_feature_measurement_model->PAN_RANGE_LIMIT;
    cerr << "Alpha at range limit " << alpha << endl;
  }
  else if (alpha < -head_point_feature_measurement_model->PAN_RANGE_LIMIT)
  {
    alpha = -head_point_feature_measurement_model->PAN_RANGE_LIMIT;
    cerr << "Alpha at range limit " << alpha << endl;
  }
  if (e > head_point_feature_measurement_model->ELEVATION_RANGE_LIMIT)
  {
    e = head_point_feature_measurement_model->ELEVATION_RANGE_LIMIT;
    cerr << "E at range limit " << e << endl;
  }
  else if (e < -head_point_feature_measurement_model->ELEVATION_RANGE_LIMIT)
  {
    e = -head_point_feature_measurement_model->ELEVATION_RANGE_LIMIT;
    cerr << "E at range limit " << e << endl;
  }
  if (gammaL > 0.0)
  {
    gammaL = 0.0;
    cerr << "VL at range limit " << gammaL << endl;
  }
  else if(gammaL < -head_point_feature_measurement_model->VERGENCE_RANGE_LIMIT)
  {
    gammaL = -head_point_feature_measurement_model->VERGENCE_RANGE_LIMIT;
    cerr << "VL at range limit " << gammaL << endl;
  }
  if (gammaR < 0.0)
  {
    gammaR = 0.0;
    cerr << "VR at range limit " << gammaR << endl;
  }
  else if (gammaR > head_point_feature_measurement_model->VERGENCE_RANGE_LIMIT)
  {
    gammaR = head_point_feature_measurement_model->VERGENCE_RANGE_LIMIT;
    cerr << "VR at range limit " << gammaR << endl;
  }

  escher_demand((float) (alpha * 180.0 / M_PI), 
		(float) (elevation_adjust(e) * 180.0 / M_PI), 
		(float) (gammaL * 180.0 / M_PI), 
		(float) (gammaR * 180.0 / M_PI));
  sim_escher_demand((float) (alpha * 180.0 / M_PI), 
		    //		(float) (elevation_adjust(e) * 180.0 / M_PI), 
		(float) (e * 180.0 / M_PI), 
		(float) (gammaL * 180.0 / M_PI), 
		(float) (gammaR * 180.0 / M_PI));

  return 0;
}

int Robot::wait_for_head()
{
  wait_for_escher((float) (alpha * 180.0 / M_PI), 
		  (float) (elevation_adjust(e) * 180.0 / M_PI), 
		  (float) (gammaL * 180.0 / M_PI), 
		  (float) (gammaR * 180.0 / M_PI));

  return 0;
}

int Robot::set_pan(double new_alpha)
{
  alpha = new_alpha;
  gammaL = gammaR = 0.0;

  move_head();
  wait_for_head();
  print_head_angles();
  grab_and_display_images();

  return 0;
}

int Robot::set_elevation(double new_e)
{
  e = new_e;
  gammaL = gammaR = 0.0;

  move_head();
  wait_for_head();
  print_head_angles();
  grab_and_display_images();

  return 0;
}

int Robot::note_current_head_position()
{
  alpha_note = alpha;
  e_note = e;
  gammaL_note = gammaL;
  gammaR_note = gammaR;

  return 0;
}

int Robot::return_to_noted_head_position()
{
  alpha = alpha_note;
  e = e_note;
  gammaL = gammaL_note;
  gammaR = gammaR_note;
  
  move_head();
  wait_for_head();
  print_head_angles();

  g_grab_images();
  display_images();

  return 0;
}

/* Function to move the head to the i'th position to search for new features
   (following a simple law dividing the viewing range up).

   First try just a constant elevation value; divide pan up evenly. */

int Robot::move_head_to_new_point_feature_search_position(int i)
{
  double alpha_step = 
             2 * head_point_feature_measurement_model->PAN_RANGE_LIMIT 
             / ( (double) NEW_FEATURES + 1.0 );

  alpha = -head_point_feature_measurement_model->PAN_RANGE_LIMIT 
          + (i + 1) * alpha_step;
  e = 10.0 * M_PI / 180.0;

  move_head();
  print_head_angles();
  wait_for_head();
  g_grab_images();
  display_images();

  return 0;
}

/********************************Grab Images**********************************/

// Explicitly grab and display images
int Robot::grab_and_display_images()
{
  g_grab_images(); // calls routine in grab.cc
  g_display_images();

  return 0;
}

/*******************************Display Images********************************/

int Robot::display_images()
{
  if (!reduced_image_flag)
    g_display_images(); // calls routine in grab.cc

  return 0;
}

/***************************Write Images to Disk******************************/

int Robot::write_images()
{
  g_write_images();

  return 0;
}

/**************************Write Image Patch to Disk**************************/

int Robot::write_patch()
{
  Hor_Image *hip = hor_alloc_image(BOXSIZE, BOXSIZE, HOR_U_CHAR, NULL);

  if (uL > 0 && uR < 0) // Patch selected in left image
  {
    /* Copy the selected patch to the save space patch */
    copy_into_patch(image[LEFT], hip, uL, vL);
    hor_write_image("patchL", hip);
    cout << "Written patch patchL.mit" << endl;
  }
  else if (uR > 0 && uL < 0) // Patch selected in right image
  {
    /* Copy the selected patch to the save space patch */
    copy_into_patch(image[RIGHT], hip, uR, vR);
    hor_write_image("patchR", hip);
    cout << "Written patch patchR.mit" << endl;
  }
  else
  {
    cout << "No patch selected." << endl;
  }

  hor_free_image(hip);
  return 0;
}

/****************************Read Images from Disk****************************/

int Robot::read_images()
{
  g_read_images();

  return 0;
}

/****************************Select Image Regions*****************************/

int Robot::set_left_image_selection(int uL_, int vL_)
{
  uL = uL_; vL = vL_; uR = -1; vR = -1;

  return 0;
}

int Robot::set_right_image_selection(int uR_, int vR_)
{
  uL = -1; vL = -1; uR = uR_; vR = vR_;

  return 0;
}

/*******************************Lock On***************************************/

/* This function will just provide some extra accuracy if we need it in 
   slow situations. We expect the head to be already aligned with the feature. 
   left_patch and right_patch should already be saved.
   uLp, vLp, uRp and vRp are used for output --- they will contain the exact
   place where the feature is found. */

int Robot::lock_on_to_point(int *uLp, int *vLp, int *uRp, int *vRp)
{
  int steps = 0;

  // Repeat this loop until fixation is good enough 
  for(;;)
  {
    // Display the search regions
    xor_elliptical_search_region(image[LEFT], PuInvLock, 
				 head->U0L, head->V0L, BOXSIZE);
    xor_elliptical_search_region(image[RIGHT], PuInvLock, 
				 head->U0R, head->V0R, BOXSIZE);
    display_images();
    usleep(500000);
    xor_elliptical_search_region(image[LEFT], PuInvLock, 
				 head->U0L, head->V0L, BOXSIZE);
    xor_elliptical_search_region(image[RIGHT], PuInvLock, 
				 head->U0R, head->V0R, BOXSIZE);

    // Search images for patches
    if (elliptical_search(image[LEFT], left_patch, PuInvLock, uLp, vLp, 
				head->U0L, head->V0L, BOXSIZE) != 0 ||
	elliptical_search(image[RIGHT], right_patch, PuInvLock, uRp, vRp,
				head->U0R, head->V0R, BOXSIZE) != 0)
    {
      cout << "Match not found." << endl;
      return -1;
    }

    // Display the patches
    // Display found patch in left image
    drawboxL( *uLp - (BOXSIZE - 1) / 2, *vLp - (BOXSIZE - 1) / 2,
	      BOXSIZE, BOXSIZE, Red);
    // Display centred box in left image
    drawboxL( (int) head->U0L - (BOXSIZE - 1) / 2, 
	      (int) head->V0L - (BOXSIZE - 1) / 2,
	      BOXSIZE, BOXSIZE, Green );

    // Display found patch in right image
    drawboxR( *uRp - (BOXSIZE - 1) / 2, *vRp - (BOXSIZE - 1) / 2,
	      BOXSIZE, BOXSIZE, Red);
    // Display centred box in right image
    drawboxR( (int) head->U0R - (BOXSIZE - 1) / 2, 
	      (int) head->V0R - (BOXSIZE - 1) / 2,
	      BOXSIZE, BOXSIZE, Green );
    
    // Break if fixation is good enough or steps is too large
    if ( 
        ((*uLp - head->U0L)*(*uLp - head->U0L) + 
	 (*vLp - head->V0L)*(*vLp - head->V0L) 
        <= SATISFIED * SATISFIED) && 
        ((*uRp - head->U0R)*(*uRp - head->U0R) + 
	 (*vRp - head->V0R)*(*vRp - head->V0R) 
        <= SATISFIED * SATISFIED)
        )
      // We are close enough
      return 0;
    else if (steps > LOCK_ON_ITERATIONS)
      return -1;

    // Find 3D feature position
    head->find_3D_position(alpha, e, gammaL, gammaR,
			   (double) *uLp, (double) *vLp, 
                           (double) *uRp, (double) *vRp,
			   hL);

    // Calculate head angles: angles passed by reference
    double doub;  // dummy double
    head->find_stereo_angles(hL, &doub, alpha, e, gammaL, gammaR);

    // Move head
    move_head();
    wait_for_head();
    g_grab_images();
    display_images();

    steps++;
  }
}

/*************************Initialise a Point Feature**************************/

/* Function returns pointer to image patch and fills z with its measurement */
Hor_Image *Robot::initialise_point_feature(Hor_Matrix *z)
{
  // Calculate where to search in the right image
  double A, B;       // Line parameters

  if (uL > 0 && uR < 0) // Patch selected in left image
  {
    /* Copy the selected patch to the save space patch */
    copy_into_patch(image[LEFT], left_patch, uL, vL);

    head->find_epipolar_lineLR(uL, vL, &A, &B,
			     gammaL, gammaR);
    cout << "Epipolar line: vR = " << A << " uR + " << B << endl;

    // Do epipolar search for patch in right image
    if(epipolar_search(image[RIGHT], left_patch, A, B, &uR, &vR, BOXSIZE) == 0)
      cout << "Match found at " << uR << ", " << vR 
           << " in right image." << endl;
    else
    {
      cout << "No good match found." << endl;
      return NULL;
    }

    // Save the patch we find in the right image
    copy_into_patch(image[RIGHT], right_patch, uR, vR);
  }
  else if (uR > 0 && uL < 0) // Patch selected in right image
  {
    /* Copy the selected patch to the save space patch */
    copy_into_patch(image[RIGHT], right_patch, uR, vR);

    head->find_epipolar_lineRL(uR, vR, &A, &B,
			     gammaL, gammaR);
    cout << "Epipolar line: vL = " << A << " uL + " << B << endl;

    // Do epipolar search for patch in left image
    if(epipolar_search(image[LEFT], right_patch, A, B, &uL, &vL, BOXSIZE) == 0)
      cout << "Match found at " << uL << ", " << vL 
           << " in left image." << endl;
    else
    {
      cout << "No good match found." << endl;
      return NULL;
    }

    // Save the patch we find in the left image
    copy_into_patch(image[LEFT], left_patch, uL, vL);
  }
  else
  {
    cout << "No patch selected." << endl;
    return NULL;
  }

  // Display patch in left image
  drawboxL( uL - (BOXSIZE - 1) / 2, vL - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Red );
  // Display patch in right image
  drawboxR( uR - (BOXSIZE - 1) / 2, vR - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Red );

  // Find 3D position of feature
  head->find_3D_position(alpha, e, gammaL, gammaR,
			 (double) uL, (double) vL, (double) uR, (double) vR,
			 hL);
  cout << "Initial cartesian measurement of feature in vehicle frame:" 
       << hL;

  // Calculate head angles: angles passed by reference
  double new_alpha, new_e, new_gammaL, new_gammaR;
  double doub;  // dummy double
  head->find_stereo_angles(hL, &doub, 
                           new_alpha, new_e, new_gammaL, new_gammaR);
  cout << "New angles: pan = " << new_alpha << ", elev = " << new_e 
       << ", LV = " << new_gammaL << ", RV = " << new_gammaR << endl;

  /* Test that we've got a sensible 3D position --- sometimes the
     find_stereo_angles routine gives positions behind the cameras. */
  double alpha_test = alpha - new_alpha;
  and_pi_range(alpha_test);
  alpha_test = (alpha_test >= 0 ? alpha_test : -alpha_test);
  if (alpha_test > M_PI / 2.0)   // Problems
  {
    cout << "Sensible 3D position not calculated: feature not initialised."
         << endl;
    return NULL;
  }

  alpha = new_alpha;
  e = new_e;
  gammaL = new_gammaL;
  gammaR = new_gammaR;

  // Move head
  move_head();
  wait_for_head();
  g_grab_images();
  display_images();

  // Lock onto feature
  if (lock_on_to_point(&uL, &vL, &uR, &vR) == 0)
  {
    cout << "Lock on achieved." << endl;
  }
  else
  {
    cout << "Lock on not successful." << endl;
    return NULL;
  }

  // Final determination of z
  head->find_3D_position(alpha, e, gammaL, gammaR,
			 (double) uL, (double) vL, (double) uR, (double) vR,
			 hL);
  // Convert hL to angular form
  head->convert_linear_measurement_to_angular(hL, z);

  cout << "Final measurement:" << z << endl;
  cout << "Angles: pan = " << alpha << ", elev = " << e 
       << ", LV = " << gammaL << ", RV = " << gammaR << endl;

  // Go and initialise it in scene + 3D  
  Hor_Image *hip = hor_alloc_image(BOXSIZE, BOXSIZE, HOR_U_CHAR, NULL);

  // Final save of fixated image patch
  copy_into_patch(image[LEFT], hip, uL, vL);

  // Reset patch selection to impossible values
  uL = vL = uR = vR = -1;

  return hip;
}

/*****************************Measure Feature*********************************/

// Of course this only deals with POINTS at the moment
int Robot::measure_feature(void *id, Hor_Matrix *z, 
			   Hor_Matrix *h, Hor_Matrix *S)
{
  // First need to convert h and S into cartesian coordinates
  Hor_Matrix *hL = hor_mat_alloc(3, 1);
  Hor_Matrix *ShL = hor_mat_alloc(3, 3);
  Hor_Matrix *dhL_by_dh = hor_mat_alloc(3, 3);

  double se = sin(matel(h, 2, 1));
  double ce = cos(matel(h, 2, 1));
  double salpha = sin(matel(h, 1, 1));
  double calpha = cos(matel(h, 1, 1));
  double sgamma = sin(matel(h, 3, 1));
  double dh_dgamma = -head->Ii / (2.0 * sgamma * sgamma);
  double h_len = head->Ii / (2.0 * tan(matel(h, 3, 1)));

  hor_matq_fill(hL,
		h_len * ce * salpha,
		h_len * se,
		h_len * ce * calpha);

  hor_matq_fill(dhL_by_dh,	
	h_len * ce * calpha, -h_len * se * salpha, dh_dgamma * ce * salpha,
	0.0,                  h_len * ce,          dh_dgamma * se,
       -h_len * ce * salpha, -h_len * se * calpha, dh_dgamma * ce * calpha);

  hor_matq_ABAT3(dhL_by_dh, S, ShL);

  // Move head
  head->find_stereo_angles(hL, &h_len, alpha, e, gammaL, gammaR);
  move_head();

  //  print_head_angles();

  // Calculate search region
  head->calculate_image_search_ellipses(ShL, h_len, alpha, e, gammaL, gammaR,
					PuInvL, PuInvR);

  wait_for_head();
  g_grab_images();
  display_images();

  // Display the search regions
  xor_elliptical_search_region(image[LEFT], PuInvL, 
			       head->U0L, head->V0L, BOXSIZE);
  xor_elliptical_search_region(image[RIGHT], PuInvR, 
			       head->U0R, head->V0R, BOXSIZE);
  display_images();
  xor_elliptical_search_region(image[LEFT], PuInvL, 
			       head->U0L, head->V0L, BOXSIZE);
  xor_elliptical_search_region(image[RIGHT], PuInvR, 
			       head->U0R, head->V0R, BOXSIZE);
  
  // Do the searches
  int uL, vL, uR, vR;
  if (elliptical_search(image[LEFT], (Hor_Image *) id, 
			PuInvL, &uL, &vL, head->U0L, head->V0L, BOXSIZE) != 0
      || elliptical_search(image[RIGHT], (Hor_Image *) id, 
			 PuInvR, &uR, &vR, head->U0R, head->V0R, BOXSIZE) != 0)
  {
    cout << "Feature not successfully matched." << endl;
    return -1;
  }

  // Display patch in left image
  drawboxL( uL - (BOXSIZE - 1) / 2, vL - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Red );
  // Display patch in right image
  drawboxR( uR - (BOXSIZE - 1) / 2, vR - (BOXSIZE - 1) / 2,
	    BOXSIZE, BOXSIZE, Red );

  // Final determination of z
  head->find_3D_position(alpha, e, gammaL, gammaR,
			 (double) uL, (double) vL, (double) uR, (double) vR,
			 hL);

  //  cerr << hL;

  // Convert hL to angular form
  head->convert_linear_measurement_to_angular(hL, z);
  
  hor_mat_free_list(hL, ShL, NULL);

  if(head->check_miss_measurement_angular(h, z, S, NO_SIGMA)) {
     cout 
        << "Measurement failed because of mismatch (innovation test failed)." 
	<< endl;
     return -1;
  }

  return 0;
}

/*****************************************************************************/

// Start the vehicle moving
int Robot::set_control(Hor_Matrix *u, double delta_t)
{
  v_control = matel(u, 1, 1);                     // In metres
  dS_control = matel(u, 2, 1) * 180.0 / M_PI;     // In degrees
  dT_control = matel(u, 3, 1) * 180.0 / M_PI;     // In degrees

  zero_nomadic_state_vector();
  required_distance = fabs(matel(u, 1, 1) * delta_t);

  increment_vehicle_steering_angle(dS_control);
  increment_vehicle_turret_angle(dT_control);
  set_vehicle_velocity(v_control);

  return 0;
}

// Continue a motion --- make sure we don't build up odometry errors
int Robot::continue_control(Hor_Matrix *u, double delta_t)
{
  v_control = matel(u, 1, 1);                     // In metres
  dS_control = matel(u, 2, 1) * 180.0 / M_PI;     // In degrees
  dT_control = matel(u, 3, 1) * 180.0 / M_PI;     // In degrees

  required_distance += fabs(matel(u, 1, 1) * delta_t);

  increment_vehicle_steering_angle(dS_control);
  increment_vehicle_turret_angle(dT_control);
  set_vehicle_velocity(v_control);

  return 0;
}

/* Wait for end of motion: loops in this function until the robot is in
   the position we want. If when this function is called the robot has 
   already gone past that position, it returns the amount of distance we've
   gone over by. Returns 0.0 if we're OK. */
double Robot::wait_for_end_of_motion(Hor_Matrix *u)
{
  current_distance = get_distance_from_odometry();

  if (current_distance > required_distance)
    return current_distance - required_distance;
  do
  {
    current_distance = get_distance_from_odometry();
    cout << "current_distance " << current_distance << ", required_distance "
	 << required_distance << endl;
    //    cerr << "current_distance: " << current_distance 
    //    	 << " required_distance " << required_distance << endl;
  }
  while (current_distance < required_distance);

  return 0.0;
}

int Robot::stop_vehicle()
{
  set_vehicle_velocity(0.0);

  // Also wait until steering and turret have stopped
  wait_for_nomad_stop(); 

  return 0;
}

int Robot::print_robot_odometry()
{
  show_nomadic_state_vector();

  return 0;
}

/* Find and display best n patches in left image. Arguments ubest, vbest,
   evbest are arrays of size n. */
int Robot::rob_find_best_n_patches(int n, int *ubest, int *vbest, double *evbest)
{
  find_best_n_patches(image[LEFT], n, ubest, vbest, evbest, BOXSIZE, 
		      (int) head->U0L, (int) head->V0L);

  // Display patches in left image
  for (int i = 0; i < n; i++)
  {
    drawboxL( ubest[i] - (BOXSIZE - 1) / 2, vbest[i] - (BOXSIZE - 1) / 2,
	       BOXSIZE, BOXSIZE, Red );

    // cout << "Patch of value " << evbest[i] 
    // 	    << " found at u = " << ubest[i] << ", v = " << vbest[i] << endl;
  }

  return 0;
}

// Speak
int Robot::say(char *speech_string)
{
  return speak(speech_string);
}
