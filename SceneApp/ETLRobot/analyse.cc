// Analyse results: note that this program is specific to the 
// Nomad robot 3D case with point features

// Main file where everything is controlled from

#include <general_headers.h>
#include "models_base.h"
#include "models_nomad.h"
#include "models_head.h"
#include "models_head_escher.h"
#ifdef _THREED_
#include "models_rollpitch.h"
#endif

#include "3d.h"

#include "feature.h"
#include "scene_base.h"
#include "scene_single.h"

#include "sim_or_rob.h"
#include "simul.h"
#include "postscript.h"
#include "waypoints.h"
#include "kalman.h"
#include "control_general.h"

#include "forms.h"
#include "formanalyse.h"

#include "analyse.h"


/**********************Main class pointers defined here***********************/

// Define one generic type motion model, but potentially several 
// specific feature models
Turn_Move_TwoD_Motion_Model *turn_move_twod_motion_modelA;

#ifdef _THREED_
// For 3D mode where the robot runs on a non-flat surface
// We use measurements from a roll-pitch sensor to help
Turn_Move_ThreeD_Motion_Model *turn_move_threed_motion_modelA;
Roll_Pitch_Sensor_Internal_Measurement_Model 
       *roll_pitch_sensor_internal_measurement_modelA;
Internal_Measurement_Model *internal_measurement_modelA;
#endif

Motion_Model *motion_modelA;

#ifdef _THREED_
ThreeD_Head_Point_Feature_Measurement_Model 
#else
TwoD_Head_Point_Feature_Measurement_Model 
#endif
  *head_point_feature_measurement_modelA;

int number_of_feature_measurement_models = 1;
Feature_Measurement_Model **feature_measurement_model_array;

Simulation *simulationA;

Three_D_Display *three_d_dispA;
Scene *sceneA;


Hor_Matrix **true_points, **true_positions, 
           **estimated_states, **estimated_covariances;

Postscript *postscriptA;

int current_step = 0;
int ground_truth_flag = 0; /* Flag is set if we read in ground truth data
			      as well as estimates. */
int this_step_has_ground_truth_flag = 0;
/* Things to read from the command line */
int no_features;
int no_steps;
int first_step = 0;
char true_point_file[100];
char true_vehicle_file[100];
char estimate_file[100];




/*****************************Display Parameters******************************/

// Global display pointer
Display *display;

/* Horatio colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;



void print_analyse_args ( void )
{
   fprintf ( stderr, "analyse no_features no_steps true_point_file true_vehicle_file estimate_file\n" );
   fprintf ( stderr, "or      no_steps estimate_file\n" );
}

// Scan Command Line
void scan_command_line ( int    argc, char **argv )
{
   for ( ; argc > 0; argv++, argc-- )
   {
      if ( (*argv)[0] != '-' ) break;

      switch ( (*argv)[1] )
      {
         case 'h':
	 print_analyse_args();
	 exit(0);
	 break;

         default:
	 hor_error ( "illegal argument -%c", HOR_FATAL, (*argv)[1] );
	 break;
      }
   }
   
   /* Need 5 arguments for case where we read in ground truth + estimates: 
      Number of features, number of steps, file with
      true points in, file with true vehicle positions in, 
      file with estimates in. 

      For cases where we just have estimates: just two arguments:
      number of steps, file with estimates in. */
   if (argc < 2)
   {
     print_analyse_args();
     exit(0);
   }

   if (argc >= 5)  // Full set including ground truth
   {
     hor_read_int_param (*argv++, "no_features", hor_int_abs_pos,
			 HOR_FATAL, &no_features);
     // A hack: put 1000 for 0
     if (no_features == 1000)
       no_features = 0;

     hor_read_int_param (*argv++, "no_steps", hor_int_abs_pos,
			 HOR_FATAL, &no_steps);
     strcpy(true_point_file, *argv++);
     strcpy(true_vehicle_file, *argv++);
     strcpy(estimate_file, *argv++);
     ground_truth_flag = 1;
   }
   else  // Just estimates
   {
     hor_read_int_param (*argv++, "no_steps", hor_int_abs_pos,
			 HOR_FATAL, &no_steps);
     strcpy(estimate_file, *argv++);
     ground_truth_flag = 0;
   }
}

// Read In Files
int read_in_files()
{
   int i;

   if (ground_truth_flag)
   {
     /* Allocate memory for true states. */
     true_points = new Hor_Matrix * [no_features];
     for (i = 0; i < no_features; i++)
       true_points[i] = hor_mat_alloc(
                     head_point_feature_measurement_modelA->FEATURE_STATE_SIZE, 1);

     true_positions = new Hor_Matrix * [no_steps];
     for (i = 0; i < no_steps; i++)
       true_positions[i] = hor_mat_alloc(motion_modelA->STATE_SIZE, 1);
   }

   /* Don't allocate the estimate arrays until we read in the data. */
   estimated_states = new Hor_Matrix * [no_steps];
   estimated_covariances = new Hor_Matrix * [no_steps];

   int intin;

   if (ground_truth_flag)
   {
     if (no_features != 0)
     {
       /* Read in true points from file */
       ifstream tpf (true_point_file, ios::in);
       if (!tpf)
       {
	 cerr << "Error: can't open file " << true_point_file << endl;
	 exit(0);
       }
       for (i = 0; tpf && i < no_features; i++)
       {
	 for (int row = 1; row <= true_points[i]->rows; row++) {
	   tpf >> vecel(true_points[i], row);
	 }
       }
       cout << "Read in " << i << " true features." << endl;
       // Check
       // for (i = 0; i < no_features; i++)
       //   cout << "Point " << i << ": " << true_points[i];
     }

     /* Read in true positions from file */
     ifstream tvf (true_vehicle_file, ios::in);
     if (!tvf)
     {
       cerr << "Error: can't open file " << true_vehicle_file << endl;
       exit(0);
     }
     /* This file may be sparse so only load some entries */
     int next_true_pos_in_file;
     tvf >> next_true_pos_in_file;
     for (i = 0; i < no_steps; i++)
     {
       if (next_true_pos_in_file != i)
       {
	 true_positions[i] = NULL;
       }
       else
       {
	 for (int row = 1; row <= true_positions[i]->rows; row++) {
	   tvf >> vecel(true_positions[i], row);
	 }
	 if (tvf)
	   tvf >> next_true_pos_in_file;
	 else
	   break;
       }
     }
     cout << "Read in " << i << " true positions." << endl;
     // Check
     // for (i = 0; i < no_steps; i++)
     //   cout << "Position " << i << ": " << true_positions[i];
   }


   /* Read in estimated states and covariances */
   int local_no_features, local_size;
   ifstream ef (estimate_file, ios::in);
   if (!ef)
   {
     cerr << "Error: can't open file " << estimate_file << endl;
     exit(0);
   }

   ef >> i;  // Start counter at first state in file
   first_step = i;
   current_step = first_step;
   ef.seekg(0); // Reset to beginning of file

   for (; ef && i < no_steps; i++)
   {
     ef >> intin >> local_no_features;
     if (intin != i)
     {
       cerr << "Problem reading estimated positions." << endl;
       exit(0);
     }
     local_size = motion_modelA->STATE_SIZE 
      + head_point_feature_measurement_modelA->FEATURE_STATE_SIZE * (local_no_features);

     estimated_states[i] = hor_mat_alloc(local_size, 1);
     estimated_covariances[i] = hor_mat_alloc(local_size, local_size);

     int r, c;
     for (r = 0; r < local_size; r++)
     {
       ef >> estimated_states[i]->m[r][0];
     }
     for (r = 0; r < local_size; r++)
       for (c = 0; c < local_size; c++)
       {
	 ef >> estimated_covariances[i]->m[r][c];
       }
   }
   cout << "Read in " << i << " estimated states and covariances." << endl;
   // Check
   // for (i = 0; i < no_steps; i++)
   // {
   //   cout << "State " << i << ": " << estimated_states[i];
   //   cout << "Covariance " << i << ": " << estimated_covariances[i];
   // }
     
   return 0;
}

// Set Step
int set_step(int step)
{
  if (step < first_step || step >= no_steps)
  {
    cerr << "Invalid step requested." << endl;
    return -1;
  }

  cout << "Now showing step " << step << ".\n";
  Hor_Matrix *dummy = hor_mats_fill(
         head_point_feature_measurement_modelA->FEATURE_STATE_SIZE, 1, 
                             1.0, 1.0, 1.0);
  Hor_Matrix *V = NULL;

  if (ground_truth_flag)
  {
    /* Make sure we've got the right number of true features */
    for (int i = simulationA->get_no_true_features(); i < no_features; i++)
      simulationA->add_new_true_feature(head_point_feature_measurement_modelA,
					dummy);

    if (true_positions[step] != NULL)
    {
      /* Load true features */
      V = hor_mat_alloc(motion_modelA->STATE_SIZE + 
        head_point_feature_measurement_modelA->FEATURE_STATE_SIZE * no_features,
			1);
      hor_matq_insert_chunky1(true_positions[step], V, 0);
      for (int i = 0; i < no_features; i++)
	hor_matq_insert_chunk31(true_points[i], V, 
                   motion_modelA->STATE_SIZE + 
                 head_point_feature_measurement_modelA->FEATURE_STATE_SIZE * i);
      simulationA->fill_true_state(V);

      this_step_has_ground_truth_flag = 1;
    }
    else
      this_step_has_ground_truth_flag = 0;
  }

  /* Work out the number of features we're currently maintaining states for */
  int local_no_features = 
           (estimated_states[step]->rows - motion_modelA->STATE_SIZE)
                  / head_point_feature_measurement_modelA->FEATURE_STATE_SIZE;

  /* Make sure we've got the right number of features in scene */
  for (int i = sceneA->get_no_features(); i < local_no_features; i++)
  {
    void *id = NULL;
    sceneA->add_new_feature(id, dummy, head_point_feature_measurement_modelA);
  }
  for (int i = sceneA->get_no_features(); i > local_no_features; i--)
  {
    sceneA->mark_first_feature();
    sceneA->delete_feature();
  }    

  /* Load scene estimates */
  sceneA->fill_states(estimated_states[step]);
  sceneA->fill_covariances(estimated_covariances[step]);

  if (ground_truth_flag && this_step_has_ground_truth_flag)
  {
    display_true_vehicle(simulationA, three_d_dispA);
    display_true_features(simulationA, three_d_dispA);
  }

  display_estimated_vehicle(sceneA, three_d_dispA);
  display_estimated_features(sceneA, three_d_dispA);

  hor_mat_free_list(dummy, V, NULL);

  return 0;
}

void set_step(FL_OBJECT *ob, long data)
{
  current_step = (int) fl_get_counter_value(ob);
  set_step(current_step);
}



void input_viewing_direction(FL_OBJECT *ob, long data)
{
  double alpha, e;

  cout << 
       "Input pan and elevation angles (in degrees) to view postscript from:" 
       << endl;
  cin >> alpha >> e;
  alpha *= M_PI / 180.0;
  e *= M_PI / 180.0;

  postscriptA->set_projection(alpha, e);
}

void input_postscript_parameters(FL_OBJECT *ob, long data)
{
  double xoffset, yoffset, scale;
  int xsize, ysize;

  cout << "Input xsize, ysize, scale, xoffset, yoffset:" << endl;
  postscriptA->print_current_parameters();
  cin >> xsize >> ysize >> scale >> xoffset >> yoffset;

  postscriptA->set_postscript_parameters(xoffset, yoffset, 
                                         scale, xsize, ysize);
}

void input_postscript_axis_limits(FL_OBJECT *ob, long data)
{
  double x_neg, x_pos, y_neg, y_pos, z_neg, z_pos;

  cout << "Input x_neg, x_pos, y_neg, y_pos, z_neg, z_pos:" << endl;
  postscriptA->print_current_axis_limits();
  cin >> x_neg >> x_pos >> y_neg >> y_pos >> z_neg >> z_pos;

  postscriptA->set_axis_limits(x_neg, x_pos, y_neg, y_pos, z_neg, z_pos);
}

void output_postscript(FL_OBJECT *ob, long data)
{
  if (ground_truth_flag && this_step_has_ground_truth_flag)
    postscriptA->output_postscript(sceneA, simulationA);
  else
    postscriptA->output_postscript(sceneA);

  cout << "Output postscript for step " << current_step << "." << endl;
}

void output_ground_truth_postscript(FL_OBJECT *ob, long data)
{
  if (ground_truth_flag && this_step_has_ground_truth_flag)
    postscriptA->output_postscript(simulationA);
  else
    cout << "Ground truth not available for output at this step." << endl;

  cout << "Output postscript for step " << current_step << "." << endl;
}

/* Output a special trajectory picture up to the current step */
void output_trajectory_picture(FL_OBJECT *ob, long data)
{
  if (ground_truth_flag && this_step_has_ground_truth_flag)
    postscriptA->output_postscript_and_leave_file_open(sceneA, simulationA);
  else
    postscriptA->output_postscript_and_leave_file_open(sceneA);

  for (int i = first_step; i <= current_step; i++)
  {
    set_step(i);

    if (ground_truth_flag && this_step_has_ground_truth_flag)
      postscriptA->just_draw_vehicle(sceneA, simulationA);
    else
      postscriptA->just_draw_vehicle(sceneA);
  }

  postscriptA->close_file();

  cout << "Output postscript for steps " << first_step << " to " 
       << current_step << "." << endl;
}

// Print State
void print_robot_state(FL_OBJECT *ob, long data)
{
  sceneA->print_robot_state();
}

void print_feature_state(FL_OBJECT *ob, long data)
{
  sceneA->print_marked_feature_state();
}

void print_whole_state(FL_OBJECT *ob, long data)
{
  sceneA->print_whole_state();
}

// Print True State
void print_true_state(FL_OBJECT *ob, long data)
{
  simulationA->print_true_vehicle();
}

/************************Initialise stuff in this file************************/

static FD_formanalyse *fd_formanalyse;

void pass_fd_pointer_analyse(FD_formanalyse *fd_fc)
{
  fd_formanalyse = fd_fc;
}

int initialise_simulation()
{
  Hor_Matrix *initial_xv = hor_mats_zero(motion_modelA->STATE_SIZE, 1);

  simulationA = new Simulation(motion_modelA, initial_xv);
  hor_mat_free_list(initial_xv, NULL);

  return 0;
}

int initialise_scene()
{
  Hor_Matrix *initial_xv = hor_mats_zero(motion_modelA->STATE_SIZE, 1);
  Hor_Matrix *initial_Pxx = hor_mats_zero(motion_modelA->STATE_SIZE,
					  motion_modelA->STATE_SIZE); 

#ifdef _THREED_
  sceneA = new Scene_Single(initial_xv, initial_Pxx, motion_modelA,
			   internal_measurement_modelA);
#else
  sceneA = new Scene_Single(initial_xv, initial_Pxx, motion_modelA);
#endif
  hor_mat_free_list(initial_xv, initial_Pxx, NULL);

  return 0;
}


XtAppContext app_con;

/**********Called from 3d.cc when feature is selected with the mouse**********/

int please_redisplay_features_flag = 0;

int true_feature_selected(int i)
{
  // Get the identifier for this feature (always a pointer)
  void *id = (void *) simulationA->find_identifier(i);

  // See if it is a feature we already know about
  if (sceneA->find_feature(id))
  {
    if (!sceneA->toggle_feature(id))
      cerr << "Trouble toggling feature with identifier" << id << endl;

    sceneA->mark_feature_by_lab(sceneA->find_feature(id)->get_label());
  }
  else // New feature
  {
    Feature_Measurement_Model *f_m_m = 
                             simulationA->find_feature_measurement_model(i);

    initialise_feature(sceneA, simulationA, three_d_dispA, f_m_m->feature_type,
		       id, number_of_feature_measurement_models, 
		       feature_measurement_model_array);
  }

  sceneA->print_selected_features();

  // This is a bit of a hack because we can't just call 
  // display_estimated_features from here (gives trouble in 3D tool code)
  // Instead called from main_control_loop()
  please_redisplay_features_flag = 1;

  return 0;
}

int estimated_feature_selected(int i)
{
  if (!sceneA->toggle_feature_lab(i))
    cerr << "Trouble toggling feature " << i << endl;

  sceneA->print_selected_features();

  // This is a bit of a hack because we can't just call 
  // display_estimated_features from here (gives trouble in 3D tool code)
  // Instead called from main_control_loop()
  please_redisplay_features_flag = 1;

  sceneA->mark_feature_by_lab(i);

  return 0;
}

int set_up_analyse( int argc, char **argv )
{
  display = fl_get_display();

  turn_move_twod_motion_modelA = new Turn_Move_TwoD_Motion_Model;
#ifdef _THREED_
  turn_move_threed_motion_modelA = 
         new Turn_Move_ThreeD_Motion_Model(turn_move_twod_motion_modelA);
  motion_modelA = turn_move_threed_motion_modelA;
  roll_pitch_sensor_internal_measurement_modelA = 
    new Roll_Pitch_Sensor_Internal_Measurement_Model(motion_modelA);
  internal_measurement_modelA =
   (Internal_Measurement_Model *) roll_pitch_sensor_internal_measurement_modelA;
#else
  motion_modelA = turn_move_twod_motion_modelA;
#endif

  Escher_Head_Point_Feature_Measurement_Model 
          *escher_head_point_feature_measurement_modelA = 
               new Escher_Head_Point_Feature_Measurement_Model (motion_modelA);
  
  head_point_feature_measurement_modelA = 
                          escher_head_point_feature_measurement_modelA;


  // Initialise horatio display for 3D tool
  hor_colourmap_setup ( display, 2, 6,
		      "Red",       &Red,       "Green",         &Green,
		      "Blue",      &Blue,      "Yellow",        &Yellow,
		      "SteelBlue", &SteelBlue, "LightSeaGreen", &LightSeaGreen,
		      "thistle",   &thistle,   "Cyan",          &Cyan,
			NULL );

  String fallback_resources[] = {
    "*input: True",
    "*text*editType:          append",
    "*text*scrollVertical:    Always",
    "*text*height:            20",
    NULL,
  };
  Widget dummy;
  dummy = XtAppInitialize(&app_con, "horatio", NULL, ZERO,
			  &argc, argv, fallback_resources, NULL, ZERO);
  GC gc = fl_get_gc();
  hor_display_initialise ( display, gc,
			   SteelBlue, Green, Red, LightSeaGreen, thistle );

  scan_command_line ( argc-1, argv+1 );


  if (ground_truth_flag)
    cout << "Number of features: " << no_features << endl;
  cout << "Number of steps: " << no_steps << endl;
  if (ground_truth_flag)
  {
    cout << "True points file: " << true_point_file << endl;
    cout << "True vehicle file: " << true_vehicle_file << endl;
  }
  cout << "Estimate file: " << estimate_file << endl;

  read_in_files();

  fl_set_counter_bounds(fd_formanalyse->counter_set_step, 0, no_steps - 1);
  fl_set_counter_value(fd_formanalyse->counter_set_step, 0);

  // Instantiate classes
  if (ground_truth_flag)
  {
    initialise_simulation();
  }

  three_d_dispA = new Three_D_Display(dummy, display);
  postscriptA = new Postscript;
  initialise_scene();

  set_step(current_step);



  return 0;
}

/******************************Main Control Loop******************************/

int main_analyse_loop()
{
  // Enter main loop
  FL_OBJECT *obj;
  XEvent event;

  for (;;)
  {
    // Check events from Xforms
    while ((obj=fl_check_forms()) != NULL);

    // Check events from 3D tool
    while (XtAppPending(app_con))
    {
      XtAppNextEvent(app_con, &event);
      XtDispatchEvent(&event);
    }
  }

  return 0;
}

/************************************Quit*************************************/

void quit(FL_OBJECT *ob, long data)
{
  // Delete simulation class so hardware is shut down in robot case
  delete simulationA;

  exit(0);
}

/*****************************************************************************/
