/* Copyright 1993 Paul A. Beardsley (pab@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

/*+******** 3D display **********

the important coordinate frame is the one in which the original 3D items 
arrive.  this is referred to as the "base frame".  

the camera always looks towards the origin O of the 3D points.  
there are three variables (measured in the base frame) to specify the 
camera position C -

 (a) the unit vector OC is called the viewpoint.
 (b) the unit vector for the up direction of the camera is called 
     the viewpoint_up.
 (c) the distance OC is called the viewpoint_distance.

the most obvious way to implement rotation is to rotate the 3D items.  this
might lead to cumulative errors in the points though.  instead the original
3D items are kept unchanged and the following approach is taken -

 (a) the starting point of the camera is stored in base_viewpoint_dir and 
     base_viewpoint_up.  these vectors are kept unchanged throughout the 
     processing.  base_viewpoint is in the negative z-direction.  note 
     that this means that, from the camera's point of view, it is looking along 
     the positive z-axis to the 3D data.  then can use the usual equations
     x/f = X/Z to project to the image.  (currently f is fixed equal to 1).

 (b) the variables viewpoint and viewpoint_up give the position
     of the camera for the current viewpoint.  to compute the appearance of 
     the 3D data from the current viewpoint, simply rotate the coordinate frame of
     the 3D data so that viewpoint and viewpoint_up are aligned with
     the directions base_viewpoint and base_viewpoint_up.  once that 
     is done, it is again possible to use x/f = X/Z.

 (c) the viewpoint_distance fits into the above scheme in the obvious way.

*****/

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/Shell.h>
#include <X11/StringDe.h>
#include <X11/cursorfo.h>

#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiTx.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/Canvas.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/ToggleC.h>
#include <X11/Xow/PanelTe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/Shell.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/Label.h>
#include <X11/Xow/Canvas.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/ToggleChoice.h>
#include <X11/Xow/PanelText.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/tool.h"

#define MOVE_COUNT   4     /* the number of mouse positions stored up before
				 rotating around an axis perp to the mouse
				 motion (to avoid over-sensitivity). */
#define ROTATE_ANGLE 0.025 /* rotation step in radians. */
#define ZOOM_FACTOR  0.975 /* scaling factor for zoom. */

typedef struct {double x;
		double y;} BIVEC_DOUBLE;

typedef enum {UNSELECTED, SELECTED, DELETED} Status_Flag;

typedef struct
{
   Hor_Item_3D item;
   Status_Flag status;
   u_long      colour;
} Internal_Item;

typedef struct
{
   int    c, r;
   double depth;
} Proj_Point;

typedef struct
{
   Internal_Item *item;
   union {
      Proj_Point                        p;
      struct { Proj_Point p1, p2; }     l;
      struct { Proj_Point p1, p2, p3; } f;
   } proj;
} Proj_3D;

typedef struct
{
   Widget          parent;             /* parent of popup 3D tool widget */
   Widget          popup_frame;        /* transient panel frame */
   Widget          popup_panel;        /* form widget containing canvases */
   Widget          canvas;             /* canvas on which graph is drawn */
   Widget          proj_toggle;        /* toggle widget for projection type */
   Widget          mode_toggle;        /* toggle widget for item selection */
   Widget          coord_toggle;       /* toggle widget for coordinate system*/
   Widget          pick_popup_frame;   /* popup panel for pick options */
   Widget          pick_points_toggle; /* toggle for picking points */
   Widget          pick_lines_toggle;  /* toggle for picking lines */
   Widget          pick_facets_toggle; /* toggle for picking facets */
   Hor_Assoc_Label string_label; /* label for mouse button string widget */
   Hor_Assoc_Label canvas_label; /* label for defining canvas to graphics
				    library */
   int             size;         /* dimensions of graph canvas */
   Hor_Bool        in_use;       /* whether graph is currently displayed */
   Hor_Bool        initialised;  /* whether hor_init_3D() has been called
				    or not */
   enum {PERSPECTIVE_PROJ,
	 ORTHOGRAPHIC_PROJ} proj_type;
   enum {RIGHT_HANDED_COORDS,
	 LEFT_HANDED_COORDS} coord_system;
   Hor_Bool pick_points, pick_lines, pick_facets;

   /* 3D stuff. */

   Hor_3D_Item_Func      oneselect_func;    /* user callback for an
					       individual selection. */
   Hor_3D_Item_List_Func procselected_func; /* user callback for processing
					       a full set of selected
					       points. */
   Hor_3D_Item_List_Func allclear_func;     /* user callback for clearing
					       (deselecting) all selected
					       points */
   Hor_3D_Item_Func      onedelete_func;    /* user callback for an
					       individual item deletion. */
   Hor_3D_Item_List_Func allrestore_func;   /* user callback for restoring
					       (undeleting) all deleted
					       points */
   Hor_3D_Item_List_Func done_func;         /* user callback for termination
					       of processing. */
   Hor_Assoc_List        list;              /* list of Internal_Item's, with
					       labels associated for each. */
   int                   no_items;          /* no. of items in list */
   enum                {SELECT_MODE,
	                DELETE_MODE} mode;    /* what type of items are
						 currently being selected. */
   Hor_Bool             mouse_leftdown_flag;   /* HOR_TRUE if left button down. */
   Hor_Bool             mouse_middledown_flag; /* HOR_TRUE if middle button down.*/
   int              move_count;            /* count of entries in move array.*/
   BIVEC_DOUBLE     move_array[MOVE_COUNT];/* store of successive mouse
					      positions. */
   u_long           background_colour;     /* this is the same as background.*/
   u_long           selected_colour;       /* colour for selected items. */
   Hor_Matrix      *rot_mat;
   Hor_Matrix      *base_viewpoint_dir;    /* reference camera direction.*/
   Hor_Matrix      *base_viewpoint_up;     /* reference up direction. */
   Hor_Matrix      *viewpoint_dir;         /* current camera direction. */
   Hor_Matrix      *viewpoint_up;          /* current up direction. */
   double           viewpoint_distance;    /* distance of viewpoint from
					      origin of points */
   double           focal_length;
   Hor_Bool         subtract_centroid;     /* whether or not to use centroid as
					      offset vector */
   Hor_Matrix      *offset;                /* is subtracted off 3D positions
					      before projection */
   Widget base_name_widget;
   Widget image_size_widget;
} Def_3D;

static void mouse_leftdown (int col_canvas, int row_canvas, void *data);
static void mouse_leftup (int col_canvas, int row_canvas, void *data);
static void mouse_middledown (int col_canvas, int row_canvas, void *data);
static void mouse_middleup (int col_canvas, int row_canvas, void *data);
static void mouse_move (int col_canvas, int row_canvas, void *data);
static void mouse_rightdown (int col_canvas, int row_canvas, void *data);
static void mouse_rotate (int col_canvas, int row_canvas,
			  Def_3D *def);
static void mouse_spinzoom (int col_canvas, int row_canvas,
			    Def_3D *def);
static void command_process_selection (Widget button, XtPointer client_data,
				       XtPointer call_data);
static void write_to_text_file (Widget button, XtPointer client_data,
				               XtPointer call_data);
static void write_to_image_file (Widget button, XtPointer client_data,
				                XtPointer call_data);
static void clear_selection (Widget button, XtPointer client_data,
			                    XtPointer call_data);
static void restore_deleted (Widget button, XtPointer client_data,
			                    XtPointer call_data);
static void command_done (Widget button, XtPointer client_data,
			                 XtPointer call_data);
static void command_setproj (Widget button, XtPointer client_data,
			                    XtPointer call_data);
static void command_setmode (Widget button, XtPointer client_data,
			                    XtPointer call_data);
static void command_setcoord (Widget button, XtPointer client_data,
			                     XtPointer call_data);
static void command_restart (Widget button, XtPointer client_data,
			                    XtPointer call_data);
static void axis_to_rotmatrix (double angle, Hor_Matrix *axis,
			       Hor_Matrix *rot_matrix);
static void compute_rotmatrix (Def_3D *def, Hor_Matrix *source1,
			       Hor_Matrix *source2,
			       Hor_Matrix *target1,
			       Hor_Matrix *target2, Hor_Matrix *rot_matrix);
static void display_single_item (Def_3D *def, Internal_Item *item);
static void display_items (Def_3D *def);
BIVEC_DOUBLE project_point (Def_3D *def, Hor_Matrix *vector1);
static void project_line (Def_3D *def, Hor_Matrix *vector1,
			  Hor_Matrix *vector2, BIVEC_DOUBLE *bivec1_ptr,
			  BIVEC_DOUBLE *bivec2_ptr);
static void transform_from_baseframe (Def_3D *def,
				      Hor_Matrix *base,
				      Hor_Matrix *view);
static void transform_to_baseframe (Def_3D *def, Hor_Matrix *view,
				    Hor_Matrix *base);

/* static variables */
static Hor_Assoc_Label next_label = HOR_ASSOC_START;
static Hor_Assoc_List  list_3D = NULL;
static Hor_Matrix     *vec1, *vec2, *vec3, *vec4, *vec5, *vec6;
static Hor_Matrix     *a_mat, *b_vec;

/* the strings and defines below must be in corresponding order.
*/

#define NO_PROJ_TYPE_CHOICES 2
static String proj_type_choices[] = { "Perspective", "Orthographic" };

#define NO_MODE_CHOICES 2
static String mode_choices[] = { "Select", "Delete" };

#define NO_COORD_CHOICES 2
static String coord_choices[] = { "Right-handed", "Left-handed" };

#define NO_PICK_CHOICES 2
static String pick_choices[] = { "Yes", "No" };

#define PROJECTION_SCALE 2.5
#define FOCAL_LENGTH     1.0

static void command_setpick (Widget button, XtPointer client_data,
			                    XtPointer call_data)
{
   Hor_Bool *pick = (Hor_Bool *) client_data;

   if ( (int) call_data == 0 ) *pick = HOR_TRUE;
   else                        *pick = HOR_FALSE;
}

static Widget create_pick_option_popup ( Def_3D *def, String name,
					 Widget parent, ... )
{
   Widget  button, last = NULL, popup_frame, popup_panel;
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( name, commandWidgetClass,
                                    parent, args, num_args );
   hor_free ( (void *) args );
   popup_frame = XtVaCreatePopupShell ( "Params", transientShellWidgetClass,
                                        button, NULL );
   def->pick_popup_frame = popup_frame;
   popup_panel = XtVaCreateManagedWidget ( "Params Popup", formWidgetClass,
                                           popup_frame, NULL );
   popup_data->x           = (Position) 30;
   popup_data->y           = (Position)  0;
   popup_data->popup_frame = popup_frame;
   XtAddCallback ( button, XtNcallback, hor_show_popup, popup_data );

   last = XtVaCreateManagedWidget ( "Pick Points:", toggleChoiceWidgetClass,
				    popup_panel,
				    XtNnumberChoices, NO_PICK_CHOICES,
				    XtNchoices,       pick_choices,
				    XtNfromVert,      last,
				    NULL );
   XtAddCallback ( last, XtNcallback, command_setpick, &def->pick_points );
   def->pick_points_toggle = last;

   last = XtVaCreateManagedWidget ( "Pick Lines: ", toggleChoiceWidgetClass,
				    popup_panel,
				    XtNnumberChoices, NO_PICK_CHOICES,
				    XtNchoices,       pick_choices,
				    XtNfromVert,      last,
				    NULL );
   XtAddCallback ( last, XtNcallback, command_setpick, &def->pick_lines );
   def->pick_lines_toggle = last;

   last = XtVaCreateManagedWidget ( "Pick Facets:", toggleChoiceWidgetClass,
				    popup_panel,
				    XtNnumberChoices, NO_PICK_CHOICES,
				    XtNchoices,       pick_choices,
				    XtNfromVert,      last,
				    NULL );
   XtAddCallback ( last, XtNcallback, command_setpick, &def->pick_facets );
   def->pick_facets_toggle = last;

   hor_create_reset_done_buttons ( popup_panel, last, NULL );
   return button;
}


/*******************
*   Hor_Assoc_Label @hor_create_3D ( Widget parent, int size,
*                                   u_long background_colour,
*                                   u_long selected_colour )
*
*   Creates a popup 3D panel for dynamic 3D drawing.
*   The returned label for the panel is then used in subsequent calls to
*   hor_popup_3D() in order to invoke the panel. The parent will typically
*   be the button that will invoke the panel.
********************/
Hor_Assoc_Label hor_create_3D ( Widget parent, int size,
			        u_long background_colour,
			        u_long selected_colour )
{
   Def_3D  *def = hor_malloc_type ( Def_3D );
   Arg      args[4];
   Cardinal no_args = ZERO;
   Widget   rest_form, button_panel, button, button2;

   if ( size <= 0 )
      hor_error ( "illegal 3D popup panel dimensions (hor_create_3D)",
		  HOR_FATAL );

   def->parent      = parent;
   def->popup_frame = XtVaCreatePopupShell ( "3D", transientShellWidgetClass,
					     parent, NULL );
   def->popup_panel = XtVaCreateManagedWidget ( "form", formWidgetClass,
					        def->popup_frame, NULL );

   XtSetArg ( args[no_args], XtNwidth,  size ); no_args++;
   XtSetArg ( args[no_args], XtNheight, size ); no_args++;
   def->canvas = XtCreateManagedWidget ( "canvas", canvasWidgetClass,
					 def->popup_panel, args, no_args );

   /* create mouse button string label widget */
   def->string_label = hor_display_set_string (def->popup_panel,
					       XtNfromVert, def->canvas, NULL);

   rest_form = XtVaCreateManagedWidget ( "Complicated",
					 formWidgetClass, def->popup_panel,
					 XtNfromHoriz, def->canvas,
					 NULL );
   
   /* create button panel */
   button_panel = XtVaCreateManagedWidget ( "Button Panel", formWidgetClass,
					    rest_form, NULL );

   def->proj_toggle = XtVaCreateManagedWidget ( " Projection:",
					        toggleChoiceWidgetClass,
					        button_panel,
					        XtNnumberChoices,
					                  NO_PROJ_TYPE_CHOICES,
					        XtNchoices, proj_type_choices,
					        NULL );
   XtAddCallback ( def->proj_toggle, XtNcallback, command_setproj, def );

   def->mode_toggle = XtVaCreateManagedWidget ( "       Mode:",
					        toggleChoiceWidgetClass,
					        button_panel,
					        XtNnumberChoices,
					                     NO_MODE_CHOICES,
					        XtNchoices,  mode_choices,
					        XtNfromVert, def->proj_toggle,
					        NULL );
   XtAddCallback ( def->mode_toggle, XtNcallback, command_setmode, def );

   def->coord_toggle = XtVaCreateManagedWidget ( "Coordinates:",
					         toggleChoiceWidgetClass,
					         button_panel,
					         XtNnumberChoices,
						              NO_COORD_CHOICES,
					         XtNchoices,  coord_choices,
					         XtNfromVert, def->mode_toggle,
					         NULL );
   XtAddCallback ( def->coord_toggle, XtNcallback, command_setcoord, def );

   button = XtVaCreateManagedWidget ( "Init", commandWidgetClass, button_panel,
				      XtNfromVert, def->coord_toggle,
				      NULL );
   XtAddCallback ( button, XtNcallback, command_restart, def );
       
   button2 = XtVaCreateManagedWidget ("Done", commandWidgetClass, button_panel,
				      XtNfromHoriz, button,
				      XtNfromVert, def->coord_toggle,
				      NULL );
   XtAddCallback ( button2, XtNcallback, command_done, def );
       
   button2 = create_pick_option_popup ( def, "Pick Options", button_panel,
				        XtNfromHoriz, button2,
				        XtNfromVert, def->coord_toggle,
				        NULL );
       
   button = XtVaCreateManagedWidget ( "Clear Selection", commandWidgetClass,
				      button_panel, XtNfromVert, button,
				      NULL );
   XtAddCallback ( button, XtNcallback, clear_selection, def );

   button = XtVaCreateManagedWidget ( "Restore Deleted", commandWidgetClass,
				      button_panel, XtNfromVert, button,
				      NULL );
   XtAddCallback ( button, XtNcallback, restore_deleted, def );

   button = XtVaCreateManagedWidget ( "Process Selection", commandWidgetClass,
				      button_panel,
				      XtNfromVert, button, NULL );
   XtAddCallback ( button, XtNcallback, command_process_selection, def );

   button = XtVaCreateManagedWidget ( "Write to Text File", commandWidgetClass,
				      button_panel,
				      XtNfromVert, button, NULL );
   XtAddCallback ( button, XtNcallback, write_to_text_file,
		   (void *) ((int) next_label) );

   button = XtVaCreateManagedWidget ( "Write to Image File",commandWidgetClass,
				      button_panel,
				      XtNfromVert, button, NULL );
   XtAddCallback ( button, XtNcallback, write_to_image_file,
		   (void *) ((int) next_label) );

   def->base_name_widget = XtVaCreateManagedWidget ( "File Base Name:",
						     panelTextWidgetClass,
						     button_panel,
						     XtNvalue,    "unnamed",
						     XtNfromVert, button,
						     NULL );
   def->image_size_widget = hor_create_text ( button_panel,
					      def->base_name_widget,
					      "Image Size:" );
   hor_set_widget_value ( def->image_size_widget, "256" );

   def->size         = size;
   def->in_use       = HOR_FALSE;
   def->initialised  = HOR_FALSE;
   def->canvas_label = HOR_ASSOC_ERROR;

   def->background_colour = background_colour;
   def->selected_colour   = selected_colour;

   /* do all hor_mat_allocs here to avoid slowness while running.
    */
    
   def->base_viewpoint_dir = hor_mat_alloc (3, 1);
   def->base_viewpoint_up  = hor_mat_alloc (3, 1);
   def->viewpoint_dir      = hor_mat_alloc (3, 1);
   def->viewpoint_up       = hor_mat_alloc (3, 1);
   def->rot_mat            = hor_mat_alloc (3, 3);
   def->offset             = hor_mat_alloc (3, 1);
   list_3D = hor_assoc_insert ( list_3D, next_label, (void *) def );
   return next_label++;
}

/*******************
*   void @hor_register_3D ( Display *display )
*
*   Once all the 3D tools have been created, this must be called to create the
*   string label for position display and registering all the 3D tools with
*   the graphics library.
********************/
void hor_register_3D ( Display *display )
{
   Hor_Assoc_List       list;
   Def_3D              *def;
   XSetWindowAttributes sw;
   Hor_Assoc_Label      old_label = hor_display_get_window();

   sw.backing_store = Always;
   for ( list = list_3D; list != NULL; list = list->next )
   {
      def = (Def_3D *) list->data;

      XtRealizeWidget ( def->popup_frame );
      def->canvas_label = hor_display_set_window(XtWindow(def->canvas),
						 def->canvas, XC_cross_reverse,
						 def->string_label );
      hor_display_set_params ( def->size, def->size );

      hor_display_set_mouse_functions(def->canvas_label,
				  mouse_leftdown,   mouse_leftup,   "Rotate", 
				  mouse_middledown, mouse_middleup, "SpinZoom",
				  mouse_rightdown,  NULL,           "Pick", 
				  mouse_move, NULL, NULL, def);

      /* tell X to update 3D when is is redisplayed */
      XChangeWindowAttributes ( display, XtWindow(def->canvas),
			        CWBackingStore, &sw );
   }

   vec1    = hor_mat_alloc ( 3, 1 );
   vec2    = hor_mat_alloc ( 3, 1 );
   vec3    = hor_mat_alloc ( 3, 1 );
   vec4    = hor_mat_alloc ( 3, 1 );
   vec5    = hor_mat_alloc ( 3, 1 );
   vec6    = hor_mat_alloc ( 3, 1 );
   a_mat   = hor_mat_alloc ( 9, 9 );
   b_vec   = hor_mat_alloc ( 9, 1 );

   hor_display_reset_window ( old_label );
}

static Hor_Bool calc_centroid ( Hor_Assoc_List list, Hor_Matrix *centroid )
{
   Internal_Item *item;
   int total_points = 0, i;

   hor_matq_zero ( centroid );
   for ( i = 0; hor_list_non_null (list); list = hor_assoc_next (list), i++ )
   {
      item = (Internal_Item *) hor_assoc_data (list);
      if ( item->status != DELETED )
	 switch ( item->item.type )
	 {
	    case HOR_POINT_3D:
	    hor_mat_increment ( centroid, item->item.u.point.p );
	    total_points++;
	    break;

	    case HOR_LINE_3D:
	    hor_mat_increment ( centroid, item->item.u.line.p1 );
	    hor_mat_increment ( centroid, item->item.u.line.p2 );
	    total_points += 2;
	    break;

	    case HOR_FACET_3D:
	    hor_mat_increment ( centroid, item->item.u.facet.p1 );
	    hor_mat_increment ( centroid, item->item.u.facet.p2 );
	    hor_mat_increment ( centroid, item->item.u.facet.p3 );
	    total_points += 3;
	    break;

	    case HOR_LABEL_3D:
	    break;

	    default:
	    hor_error ( "illegal 3D item type (calc_centroid)", HOR_FATAL );
	    break;
	 }
   }

   if ( total_points > 0 )
   {
      hor_matq_scale ( centroid, 1.0/((double) total_points) );
      return HOR_TRUE;
   }
   else
      return HOR_FALSE;
}

/*******************
*   Hor_Bool @hor_popup_3D ( Widget button, Hor_Assoc_Label label_3D,
*                           Hor_3D_Item_Func      oneselect_func,
*                           Hor_3D_Item_List_Func procselected_func,
*                           Hor_3D_Item_List_Func allclear_func,
*                           Hor_3D_Item_Func      onedelete_func,
*                           Hor_3D_Item_List_Func allrestore_func,
*                           Hor_3D_Item_List_Func done_func )
*
*   Causes a 3D tool to appear on the screen. The button argument is used to
*   place the graph panel. It will normally be the button that invoked the,
*   graph, but can be NULL, in which case the parent widget passed into
*   hor_create_3D() is used to place the panel. The 3D tool is specified by
*   the label_3D argument. The function arguments are user callback procedures
*   called on the following mouse events:
*
*   oneselect_func()    selection of a single 3D item.
*   procselected_func() selection of all 3D items with the "Process Selection"
*                       button.
*   allclear_func()     clearing (deselection) of all 3D items using the
*                       "Clear Selction" button.
*   onedelete_func()    deletion of a single 3D item.
*   allrestore_func()   restoration (undeletion) of all 3D items using the
*                       "Restore Deleted" button.
*   done_func()         exiting the 3D tool with the "Done" button.
********************/
Hor_Bool hor_popup_3D ( Widget button, Hor_Assoc_Label label_3D,
		        Hor_3D_Item_Func      oneselect_func,
		        Hor_3D_Item_List_Func procselected_func,
		        Hor_3D_Item_List_Func allclear_func,
		        Hor_3D_Item_Func      onedelete_func,
		        Hor_3D_Item_List_Func allrestore_func,
		        Hor_3D_Item_List_Func done_func )
{
   Def_3D  *def;
   Position x, y;
   Hor_Assoc_Label old_label = hor_display_get_window();

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_popup_3D)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_ALREADY_IN_USE;
      return HOR_FALSE;
   }

   if ( button == NULL ) button = def->parent;

   hor_display_reset_window(def->canvas_label);
   hor_display_clear (def->background_colour);
   hor_display_reset_window (old_label);
   XtTranslateCoords( button, (Position)0, (Position)0, &x, &y);
   XtVaSetValues(def->popup_frame,
		 XtNx, x + 30,
		 XtNy, y,
		 0);
   XtPopup(def->popup_frame, XtGrabNone);

   def->oneselect_func    = oneselect_func;
   def->procselected_func = procselected_func;
   def->allclear_func     = allclear_func;
   def->onedelete_func    = onedelete_func;
   def->allrestore_func   = allrestore_func;
   def->done_func         = done_func;
   def->list     = NULL;
   def->no_items = 0;
   def->proj_type    = PERSPECTIVE_PROJ;
   def->mode         = SELECT_MODE;
   def->coord_system = RIGHT_HANDED_COORDS;
   def->pick_points = def->pick_lines = def->pick_facets = HOR_TRUE;
   XtVaSetValues ( def->proj_toggle,        XtNindex, 0, NULL );
   XtVaSetValues ( def->mode_toggle,        XtNindex, 0, NULL );
   XtVaSetValues ( def->coord_toggle,       XtNindex, 0, NULL );
   XtVaSetValues ( def->pick_points_toggle, XtNindex, 0, NULL );
   XtVaSetValues ( def->pick_lines_toggle,  XtNindex, 0, NULL );
   XtVaSetValues ( def->pick_facets_toggle, XtNindex, 0, NULL );
   def->mouse_leftdown_flag = HOR_FALSE;
   def->mouse_middledown_flag = HOR_FALSE;
   def->in_use = HOR_TRUE;
   def->initialised = HOR_FALSE;

   /* canvas coordinates are the same as user coordinates.
    */

   hor_display_canvas_set_params ( def->canvas_label, def->size, def->size );

   return HOR_TRUE;
}

static void hor_3D_item_free ( void *data )
{
   Internal_Item *item = (Internal_Item *) data;

   switch ( item->item.type )
   {
      case HOR_POINT_3D:
      hor_mat_free ( item->item.u.point.p );
      break;

      case HOR_LINE_3D:
      hor_mat_free ( item->item.u.line.p2 );
      hor_mat_free ( item->item.u.line.p1 );
      break;

      case HOR_FACET_3D:
      hor_mat_free ( item->item.u.facet.p3 );
      hor_mat_free ( item->item.u.facet.p2 );
      hor_mat_free ( item->item.u.facet.p1 );
      hor_free_image ( item->item.u.facet.image );
      break;

      case HOR_LABEL_3D:
      hor_mat_free ( item->item.u.label.point.p );
      break;

      default:
      hor_error ( "illegal 3D item type (hor_3D_item_free)", HOR_FATAL );
      break;
   }

   hor_free ( data );
}

/*******************
*   Hor_Bool @hor_popdown_3D ( Hor_Assoc_Label label_3D )
*
*   Causes a 3D tool to disppear, losing all the points displayed in it.
********************/
Hor_Bool hor_popdown_3D ( Hor_Assoc_Label label_3D )
{
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_popdown_3D)",
		  HOR_FATAL, label_3D );

   if ( !def->in_use ) return HOR_FALSE;

   hor_assoc_free ( def->list, hor_3D_item_free );
   XtPopdown ( def->popup_frame );
   XtPopdown ( def->pick_popup_frame );
   def->in_use = HOR_FALSE;
   def->initialised = HOR_FALSE;
   return HOR_TRUE;
}

static double max_coordinate ( Hor_Assoc_List list, Hor_Matrix *offset )
{
   double         maxp = 0.0;
   Internal_Item *item;
   
   for ( ; list != NULL; list = list->next )
   {
      item = (Internal_Item *) hor_assoc_data(list);

      switch ( item->item.type )
      {
	 case HOR_POINT_3D:
	 maxp = hor_dmax (maxp,
			  hor_vec_max_coord(hor_matq_sub (item->item.u.point.p,
							  offset, vec6)));
	 break;

	 case HOR_LINE_3D:
	 maxp = hor_dmax (maxp,
			  hor_vec_max_coord(hor_matq_sub (item->item.u.line.p1,
							  offset, vec6)));
	 maxp = hor_dmax (maxp,
			  hor_vec_max_coord(hor_matq_sub (item->item.u.line.p2,
							  offset, vec6)));
	 break;

	 case HOR_FACET_3D:
	 maxp = hor_dmax (maxp,
		  hor_vec_max_coord(hor_matq_sub (item->item.u.facet.p1,
						  offset, vec6)));
	 maxp = hor_dmax (maxp,
		  hor_vec_max_coord(hor_matq_sub (item->item.u.facet.p2,
						  offset, vec6)));
	 maxp = hor_dmax (maxp,
		  hor_vec_max_coord(hor_matq_sub (item->item.u.facet.p3,
						  offset, vec6)));
	 break;

	 case HOR_LABEL_3D:
	 break;

	 default:
	 hor_error ( "illegal 3D item type (max_coordinate)", HOR_FATAL );
	 break;
      }
   }

   return maxp;
}

/*******************
*   Hor_Bool @hor_init_3D ( Hor_Assoc_Label label_3D,
*                          Hor_Bool subtract_centroid )
*
*   Displays 3D items in a 3D tool, at an initial default viewing position.
********************/
Hor_Bool hor_init_3D ( Hor_Assoc_Label label_3D,
		       Hor_Bool subtract_centroid )
{
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_init_3D)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   /* Use either zero or the centroid as an offset vector to be subtracted
      from 3D points prior to projection */
   def->subtract_centroid = subtract_centroid;
   if ( subtract_centroid )
   {
      if ( !calc_centroid ( def->list, def->offset ) )
      {
	 hor_warning ( "no points to display" );
	 hor_matq_zero ( def->offset );
      }
   }
   else
      hor_matq_zero ( def->offset );

   /* pick a sensible distance away for the initial viewpoint.
    */
   def->viewpoint_distance = PROJECTION_SCALE*
                             max_coordinate ( def->list, def->offset );
   def->focal_length = FOCAL_LENGTH;

   /* the following are reference values which are never updated. The camera is
      along the negative z-axis i.e. it is looking in the positive z-direction
     (the usual type of setup). */

   hor_matq_fill ( def->base_viewpoint_dir, 0.0,  0.0, -1.0 );
   hor_matq_fill ( def->base_viewpoint_up,  0.0, -1.0,  0.0 );

    /* the following specify the current viewpoint position, and so are updated
       during motion. */

   hor_matq_copy ( def->base_viewpoint_dir, def->viewpoint_dir );
   hor_matq_copy ( def->base_viewpoint_up,  def->viewpoint_up );

   /* display the 3D data. */
    
   display_items (def);
   def->initialised = HOR_TRUE;
   return HOR_TRUE;
}

static BIVEC_DOUBLE proj_point ( Def_3D *def, Hor_Matrix *point,
				 double *depth )
{
   hor_matq_sub ( point, def->offset, vec1 );
   if ( def->coord_system == LEFT_HANDED_COORDS ) vec1->m[2][0] *= -1.0;

   transform_from_baseframe ( def, vec1, vec2 );
   *depth = vec2->m[2][0];
   return ( project_point (def, vec2) );
}

static void proj_line ( Def_3D *def, Hor_Matrix *p1, Hor_Matrix *p2,
		        BIVEC_DOUBLE *bivec1, double *depth1, 
		        BIVEC_DOUBLE *bivec2, double *depth2 )
{
   hor_matq_sub ( p1, def->offset, vec1 );
   hor_matq_sub ( p2, def->offset, vec2 );
   if ( def->coord_system == LEFT_HANDED_COORDS )
   {
      vec1->m[2][0] *= -1.0;
      vec2->m[2][0] *= -1.0;
   }

   transform_from_baseframe ( def, vec1, vec3 );
   transform_from_baseframe ( def, vec2, vec4);
   *depth1 = vec3->m[2][0];
   *depth2 = vec4->m[2][0];
   project_line ( def, vec3, vec4, bivec1, bivec2 );
}

static void project_item ( Def_3D *def, Internal_Item *item, double scale,
			   Proj_3D *proj_3D )
{
   BIVEC_DOUBLE bivec1, bivec2, bivec3;
   double       depth1, depth2, depth3;

   switch ( item->item.type )
   {
      case HOR_POINT_3D:
      bivec1 = proj_point ( def, item->item.u.point.p, &depth1 );
      proj_3D->proj.p.c = (int) (scale*bivec1.x + 0.5);
      proj_3D->proj.p.r = (int) (scale*bivec1.y + 0.5);
      proj_3D->proj.p.depth = depth1;
      break;

      case HOR_LINE_3D:
      proj_line ( def, item->item.u.line.p1, item->item.u.line.p2,
		       &bivec1, &depth1, &bivec2, &depth2 );
      proj_3D->proj.l.p1.c = (int) (scale*bivec1.x + 0.5);
      proj_3D->proj.l.p1.r = (int) (scale*bivec1.y + 0.5);
      proj_3D->proj.l.p1.depth = depth1;
      proj_3D->proj.l.p2.c = (int) (scale*bivec2.x + 0.5);
      proj_3D->proj.l.p2.r = (int) (scale*bivec2.y + 0.5);
      proj_3D->proj.l.p2.depth = depth2;
      break;

      case HOR_FACET_3D:
      bivec1 = proj_point ( def, item->item.u.facet.p1, &depth1 );
      bivec2 = proj_point ( def, item->item.u.facet.p2, &depth2 );
      bivec3 = proj_point ( def, item->item.u.facet.p3, &depth3 );
      proj_3D->proj.f.p1.c = (int) (scale*bivec1.x + 0.5);
      proj_3D->proj.f.p1.r = (int) (scale*bivec1.y + 0.5);
      proj_3D->proj.f.p1.depth = depth1;
      proj_3D->proj.f.p2.c = (int) (scale*bivec2.x + 0.5);
      proj_3D->proj.f.p2.r = (int) (scale*bivec2.y + 0.5);
      proj_3D->proj.f.p2.depth = depth2;
      proj_3D->proj.f.p3.c = (int) (scale*bivec3.x + 0.5);
      proj_3D->proj.f.p3.r = (int) (scale*bivec3.y + 0.5);
      proj_3D->proj.f.p3.depth = depth3;
      break;

      case HOR_LABEL_3D:
      bivec1 = proj_point ( def, item->item.u.label.point.p, &depth1 );
      proj_3D->proj.p.c = (int) (scale*bivec1.x + 0.5);
      proj_3D->proj.p.r = (int) (scale*bivec1.y + 0.5);
      proj_3D->proj.p.depth = depth1;
      break;

      default:
      hor_error ( "illegal 3D item type (project_item)", HOR_FATAL );
      break;
   }

   proj_3D->item = item;
}

static void colour_item ( Def_3D *def, Proj_3D *proj_3D,
			  Hor_Bool skeleton_flag )
{
   Internal_Item *item = proj_3D->item;

   switch ( item->status )
   {
      case UNSELECTED:
      hor_display_set_colour (item->colour); break;


      /* Hacked by davison@etl.go.jp 3/3/98: don't want colour when selected */
      case SELECTED:
      /*      hor_display_set_colour (def->selected_colour); break; */
      hor_display_set_colour (item->colour); break;

      case DELETED:
      hor_display_set_colour (def->background_colour); break;

      default: hor_error ( "illegal status %d (colour_item)", HOR_FATAL );
      break;
   }

   switch ( item->item.type )
   {
      case HOR_POINT_3D:
      hor_display_draw_circle ( proj_3D->proj.p.c, proj_3D->proj.p.r, 1);
      break;

      case HOR_LINE_3D:
      hor_display_line ( (float) proj_3D->proj.l.p1.c + 0.5,
			 (float) proj_3D->proj.l.p1.r + 0.5,
			 (float) proj_3D->proj.l.p2.c + 0.5,
			 (float) proj_3D->proj.l.p2.r + 0.5 );
      break;

      case HOR_FACET_3D:
      {
	 Hor_Item_3D_facet f = item->item.u.facet;
	 int x1, y1, x2, y2, x3, y3, a1, b1, c1, a2, b2, c2, a3, b3, c3;
	 int c, r, min_c, min_r, max_c, max_r, im_c, im_r;
	 double denom;

	 min_c = max_c = x1 = proj_3D->proj.f.p1.c;
	 min_r = max_r = y1 = proj_3D->proj.f.p1.r;
	 if ( (x2 = proj_3D->proj.f.p2.c) < min_c ) min_c = x2;
	 if ( x2 > max_c ) max_c = x2;
	 if ( (y2 = proj_3D->proj.f.p2.r) < min_r ) min_r = y2;
	 if ( y2 > max_r ) max_r = y2;
	 if ( (x3 = proj_3D->proj.f.p3.c) < min_c ) min_c = x3;
	 if ( x3 > max_c ) max_c = x3;
	 if ( (y3 = proj_3D->proj.f.p3.r) < min_r ) min_r = y3;
	 if ( y3 > max_r ) max_r = y3;

	 /* check whether facet still goes clockwise, i.e. is being
	    displayed from the right side */
	 if ( skeleton_flag || (denom = (double)(x1*y2 - x2*y1 + x2*y3 - x3*y2 + x3*y1 - x1*y3)) < 0.0  )
	 {
	    /* display fragmentary version of facet */
	    hor_display_line ( (float) x1 + 0.5, (float) y1 + 0.5,
			       (float) x2 + 0.5, (float) y2 + 0.5 );
	    hor_display_line ( (float) x2 + 0.5, (float) y2 + 0.5,
			       (float) x3 + 0.5, (float) y3 + 0.5 );
	    hor_display_line ( (float) x3 + 0.5, (float) y3 + 0.5,
			       (float) x1 + 0.5, (float) y1 + 0.5 );
	    break;
	 }

	 a1 = y3 - y2; b1 = x2 - x3; c1 = x3*y2 - x2*y3;
	 a2 = y1 - y3; b2 = x3 - x1; c2 = x1*y3 - x3*y1;
	 a3 = y2 - y1; b3 = x1 - x2; c3 = x2*y1 - x1*y2;
	 switch ( f.image->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char **arr = f.image->array.uc;

	       for ( r = min_r; r <= max_r; r++ )
		  for ( c = min_c; c <= max_c; c++ )
		     if ( a1*c + b1*r + c1 <= 0 && a2*c + b2*r + c2 <= 0 &&
			  a3*c + b3*r + c3 <= 0 )
		     {
			/* project point onto image */
			im_c = (int) ((double)
				 (f.c1*((y2-y3)*c + (x3-x2)*r + x2*y3 - x3*y2)
			        + f.c2*((y3-y1)*c + (x1-x3)*r + x3*y1 - x1*y3)
			        + f.c3*((y1-y2)*c + (x2-x1)*r + x1*y2 - x2*y1))
			       / denom + 0.5);
			im_r = (int) ((double)
			         (f.r1*((y2-y3)*c + (x3-x2)*r + x2*y3 - x3*y2)
			        + f.r2*((y3-y1)*c + (x1-x3)*r + x3*y1 - x1*y3)
			        + f.r3*((y1-y2)*c + (x2-x1)*r + x1*y2 - x2*y1))
			       / denom + 0.5);
			if ( im_c >= 0 && im_c < f.image->width &&
			     im_r >= 0 && im_r < f.image->height ) {
			   hor_display_set_colour(Hor_Grey[arr[im_r][im_c]]);
			   hor_display_plot ( c, r );
			}
		     }
	    }

	    if ( item->status == SELECTED ) /* display outline as well */
	    {
	       hor_display_set_colour (def->selected_colour);
	       hor_display_line ( (float) x1 + 0.5, (float) y1 + 0.5,
				  (float) x2 + 0.5, (float) y2 + 0.5 );
	       hor_display_line ( (float) x2 + 0.5, (float) y2 + 0.5,
				  (float) x3 + 0.5, (float) y3 + 0.5 );
	       hor_display_line ( (float) x3 + 0.5, (float) y3 + 0.5,
				  (float) x1 + 0.5, (float) y1 + 0.5 );
	    }

	    break;

	    default:
	    hor_error ( "illegal image type (colour_item)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_LABEL_3D:
      hor_display_set_font ( item->item.u.label.font );
      hor_display_text ( proj_3D->proj.p.c, proj_3D->proj.p.r,
			 item->item.u.label.label );
      break;

      default:
      hor_error ( "illegal 3D item type (colour_item)", HOR_FATAL );
      break;
   }
}

/*******************
*   Hor_Bool @hor_3D_item ( Hor_Assoc_Label label_3D,
*                          Hor_Assoc_Label item_label,
*                          Hor_Item_3D item, u_long colour )
*
*   Adds a new 3D item to a 3D tool.
********************/
Hor_Bool hor_3D_item ( Hor_Assoc_Label label_3D,
		       Hor_Assoc_Label item_label,
		       Hor_Item_3D item, u_long colour )
{
   Def_3D         *def;
   Internal_Item  *new, *old;
   Hor_Assoc_Label old_label = HOR_ASSOC_ERROR;
   Proj_3D         proj_3D;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_item)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( colour == def->background_colour )
      hor_warning ( "colour clash with 3D tool background" );

   if ( def->initialised )
   {
      old_label = hor_display_get_window();
      hor_display_reset_window ( def->canvas_label );
   }

   new = hor_malloc_type ( Internal_Item );
   if ( (old = (Internal_Item *) hor_assoc_find ( def->list, item_label ))
        != NULL )
   {
      if ( old->status == DELETED ) new->status = UNSELECTED;
      else
      {
	 new->status = old->status;
	 if ( def->initialised )
	 {
	    old->status = DELETED;
	    project_item ( def, old, 1.0, &proj_3D );
	    colour_item ( def, &proj_3D, HOR_FALSE );
	 }
      }

      hor_assoc_remove ( &(def->list), item_label, hor_3D_item_free );
   }
   else new->status = UNSELECTED;

   new->item = item;
   switch ( item.type )
   {
      int min_c, min_r, max_c, max_r;

      case HOR_POINT_3D:
      new->item.u.point.p = hor_mats_copy ( item.u.point.p );
      break;

      case HOR_LINE_3D:
      new->item.u.line.p1 = hor_mats_copy ( item.u.line.p1 );
      new->item.u.line.p2 = hor_mats_copy ( item.u.line.p2 );
      break;

      case HOR_FACET_3D:
      {
	 Hor_Item_3D_facet f = item.u.facet,
	                     *new_t = &new->item.u.facet;
	 Hor_Sub_Image *sub_image;

	 /* order points to go clockwise */
	 if ( f.c1*f.r2 - f.c2*f.r1 + f.c2*f.r3 - f.c3*f.r2
	    + f.c3*f.r1 - f.c1*f.r3 >= 0 )
	 {
	    new_t->p1 = hor_mats_copy ( f.p1 );
	    new_t->c1 = f.c1;
	    new_t->r1 = f.r1;
	    new_t->p2 = hor_mats_copy ( f.p2 );
	    new_t->c2 = f.c2;
	    new_t->r2 = f.r2;
	 }
	 else
	 {
	    new_t->p1 = hor_mats_copy ( f.p2 );
	    new_t->c1 = f.c2;
	    new_t->r1 = f.r2;
	    new_t->p2 = hor_mats_copy ( f.p1 );
	    new_t->c2 = f.c1;
	    new_t->r2 = f.r1;
	 }

	 new_t->p3 = hor_mats_copy ( f.p3 );
	 new_t->c3 = f.c3;
	 new_t->r3 = f.r3;

	 min_c = max_c = f.c1; min_r = max_r = f.r1;
	 if ( f.c2 < min_c ) min_c = f.c2; if ( f.c2 > max_c ) max_c = f.c2;
	 if ( f.r2 < min_r ) min_r = f.r2; if ( f.r2 > max_r ) max_r = f.r2;
	 if ( f.c3 < min_c ) min_c = f.c3; if ( f.c3 > max_c ) max_c = f.c3;
	 if ( f.r3 < min_r ) min_r = f.r3; if ( f.r3 > max_r ) max_r = f.r3;
	 sub_image = hor_extract_from_image ( f.image, min_c, min_r,
					      max_c-min_c+1, max_r-min_r+1 );
	 new_t->image = hor_copy_image ( &sub_image->image );
	 hor_free_sub_image ( sub_image );
	 new_t->c1 -= min_c; new_t->r1 -= min_r;
	 new_t->c2 -= min_c; new_t->r2 -= min_r;
	 new_t->c3 -= min_c; new_t->r3 -= min_r;
      }
      break;

      case HOR_LABEL_3D:
      new->item.u.label = item.u.label;
      new->item.u.label.point.p = hor_mats_copy ( item.u.label.point.p );
      break;

      default:
      hor_error ( "illegal 3D item type (hor_3D_item)", HOR_FATAL );
      break;
   }

   new->colour = colour;
   if ( def->initialised )
   {
      project_item ( def, new, 1.0, &proj_3D );
      colour_item ( def, &proj_3D, HOR_FALSE );
      hor_display_reset_window ( old_label );
   }

   def->list = hor_assoc_insert ( def->list, item_label, (void *) new );
   if ( old == NULL ) def->no_items++;
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_colour_3D_item ( Hor_Assoc_Label label_3D,
*                                 Hor_Assoc_Label item_label,
*                                 u_long          colour)
*
*   Colours the specified item displayed in the specified 3D tool
*   using the specified colour.
********************/
Hor_Bool hor_colour_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label, u_long colour )
{
   Internal_Item  *item = NULL;
   Hor_Assoc_Label old_label = hor_display_get_window();
   Def_3D         *def;
   Proj_3D         proj_3D;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_colour_item)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( !def->initialised )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_INIT;
      return HOR_FALSE;
   }

   item = (Internal_Item *) hor_assoc_find ( def->list, item_label );
   if ( item == NULL )
   {
      hor_errno = HOR_TOOL_3D_ITEM_NOT_FOUND;
      return HOR_FALSE;
   }

   if ( colour == def->background_colour )
      hor_warning ( "colour clash with 3D tool background" );

   if ( colour == def->selected_colour )
      hor_warning ( "colour clash with 3D tool pick colour" );

   if ( item->status == UNSELECTED )
   {
      item->colour = colour; /* so that new colour will be preserved */
      hor_display_reset_window ( def->canvas_label );
      project_item ( def, item, 1.0, &proj_3D );
      colour_item ( def, &proj_3D, HOR_FALSE );
      hor_display_reset_window ( old_label );
   }

   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_select_3D_item ( Hor_Assoc_Label label_3D,
*                                 Hor_Assoc_Label item_label )
*   Hor_Bool @hor_delete_3D_item ( Hor_Assoc_Label label_3D,
*                                 Hor_Assoc_Label item_label )
*   Hor_Bool @hor_reset_3D_item  ( Hor_Assoc_Label label_3D,
*                                 Hor_Assoc_Label item_label )
*
*   Selects/deletes/resets the specified item/all items displayed in the
*   specified 3D tool as if it/they had been selected/deleted/reset using
*   the mouse.
********************/
Hor_Bool hor_select_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label )
{
   Internal_Item *item = NULL;
   Def_3D        *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_select_item)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( !def->initialised )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_INIT;
      return HOR_FALSE;
   }

   item = (Internal_Item *) hor_assoc_find ( def->list, item_label );
   if ( item == NULL )
   {
      hor_errno = HOR_TOOL_3D_ITEM_NOT_FOUND;
      return HOR_FALSE;
   }

   item->status = SELECTED;
   display_single_item ( def, item );
   return HOR_TRUE;
}

Hor_Bool hor_delete_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label )
{
   Internal_Item *item = NULL;
   Def_3D        *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_delete_item)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( !def->initialised )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_INIT;
      return HOR_FALSE;
   }

   item = (Internal_Item *) hor_assoc_find ( def->list, item_label );
   if ( item == NULL )
   {
      hor_errno = HOR_TOOL_3D_ITEM_NOT_FOUND;
      return HOR_FALSE;
   }

   item->status = DELETED;
   display_single_item ( def, item );
   return HOR_TRUE;
}

Hor_Bool hor_reset_3D_item ( Hor_Assoc_Label label_3D,
			     Hor_Assoc_Label item_label )
{
   Internal_Item *item = NULL;
   Def_3D        *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_restore_item)",
		  HOR_FATAL, label_3D );

   if ( def->canvas_label == HOR_ASSOC_ERROR )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_REGISTERED;
      return HOR_FALSE;
   }

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   if ( !def->initialised )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_INIT;
      return HOR_FALSE;
   }

   item = (Internal_Item *) hor_assoc_find ( def->list, item_label );
   if ( item == NULL )
   {
      hor_errno = HOR_TOOL_3D_ITEM_NOT_FOUND;
      return HOR_FALSE;
   }

   item->status = UNSELECTED;
   display_single_item ( def, item );
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_3D_clear ( Hor_Assoc_Label label_3D )
*
*   Clears the specified 3D tool of all current 3D items, but maintains
*   viewing geometry to that new points may replace the deleted ones and
*   shown from the same viewpoint.
********************/
Hor_Bool hor_3D_clear ( Hor_Assoc_Label label_3D )
{
   Def_3D *def;
   Hor_Assoc_Label old_label = hor_display_get_window();

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_clear)",
		  HOR_FATAL, label_3D );

   hor_assoc_free ( def->list, hor_3D_item_free );
   def->list     = NULL;
   def->no_items = 0;
   hor_display_reset_window(def->canvas_label);
   hor_display_clear (def->background_colour);
   hor_display_reset_window (old_label);
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_3D_in_use ( Hor_Assoc_Label label_3D )
*
*   Returns HOR_FALSE if a registered (by hor_create_3D()) 3D tool
*   is not in use, HOR_TRUE if it is, i.e. if hor_popup_3D() has been called
*   for it and the "Done" button not yet pressed.
********************/
Hor_Bool hor_3D_in_use ( Hor_Assoc_Label label_3D )
{
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_3D_in_use)",
		  HOR_FATAL, label_3D );

   return ( def->in_use );
}

/*******************
*   Hor_Bool @hor_redisplay_3D ( Hor_Assoc_Label label_3D )
*
*   Causes a popup 3D tool to appear on the screen.
*   It should only be called during use of the 3D tool, i.e. between the calls
*   to hor_popup_3D() and pressing the "Done" button.
*   It pops up the 3D tool to the front on the screen.
********************/
Hor_Bool hor_redisplay_3D ( Hor_Assoc_Label label_3D )
{
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ("3D tool with label %d does not exist (hor_redisplay_3D)",
		 HOR_FATAL, label_3D );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   XtPopup(def->popup_frame, XtGrabNone);
   return HOR_TRUE;
}

/*******************
*   Hor_Bool @hor_write_3D_text ( Hor_Assoc_Label label_3D,
*                                const char     *base_name )
*
*   Writes the contents of a 3D tool to text files with the given base name
*   in gtool format. The files are <base_name>.points, <base_name>.lines and
*   <base_name>.facets for the point, line and facet 3D items respectively.
********************/
Hor_Bool hor_write_3D_text ( Hor_Assoc_Label label_3D, const char *base_name )
{
   char           name[300];
   FILE          *fd_point, *fd_line, *fd_facet;
   Def_3D        *def;
   Hor_Assoc_List list_ptr;
   Internal_Item *item;
   BIVEC_DOUBLE   bivec1, bivec2, bivec3;
   int            size;
   double         depth;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_write_3D_text)",
		  HOR_FATAL, label_3D );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   sprintf ( name, "%s.points", base_name ); fd_point = fopen ( name, "w" );
   sprintf ( name, "%s.lines",  base_name ); fd_line  = fopen ( name, "w" );
   sprintf ( name, "%s.facets", base_name ); fd_facet = fopen ( name, "w" );
   if ( fd_point == NULL || fd_line == NULL || fd_facet == NULL )
   {
      hor_errno = HOR_TOOL_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   size = def->size;
   for ( list_ptr = def->list; list_ptr != NULL; list_ptr = list_ptr->next )
   {
      item = (Internal_Item *) hor_assoc_data(list_ptr);
      switch ( item->item.type )
      {
	 case HOR_POINT_3D:
	 bivec1 = proj_point ( def, item->item.u.point.p, &depth );
	 fprintf ( fd_point, "%d %d\n", (int) (bivec1.x+0.5),
		                 size - (int) (bivec1.y+0.5) );
	 break;

	 case HOR_LINE_3D:
	 proj_line ( def, item->item.u.line.p1, item->item.u.line.p2,
		     &bivec1, &depth, &bivec2, &depth );
	 fprintf ( fd_line, "\n%d %d\n%d %d\n\n",
		           (int) (bivec1.x+0.5), size - (int) (bivec1.y+0.5),
		           (int) (bivec2.x+0.5), size - (int) (bivec2.y+0.5) );
	 break;

	 case HOR_FACET_3D:
	 bivec1 = proj_point ( def, item->item.u.facet.p1, &depth );
	 bivec2 = proj_point ( def, item->item.u.facet.p2, &depth );
	 bivec3 = proj_point ( def, item->item.u.facet.p3, &depth );
	 fprintf ( fd_facet, "\n%d %d\n%d %d\n%d %d\n\n",
		          (int) (bivec1.x+0.5), size - (int) (bivec1.y+0.5),
		          (int) (bivec2.x+0.5), size - (int) (bivec2.y+0.5),
		          (int) (bivec3.x+0.5), size - (int) (bivec3.y+0.5) );
	 break;

	 case HOR_LABEL_3D:
	 break;

	 default:
	 hor_error ( "illegal 3D item type (hor_write_3D_text)", HOR_FATAL );
	 break;
      }
   }

   fclose ( fd_facet );
   fclose ( fd_line );
   fclose ( fd_point );
   return HOR_TRUE;
}

static int compare_func ( const void *ptr1, const void *ptr2 )
{
   Proj_3D *proj1 = (Proj_3D *) ptr1, *proj2 = (Proj_3D *) ptr2;
   double depth1 = 0.0, depth2 = 0.0;
   Hor_Item_3D *item1 = &proj1->item->item, *item2 = &proj2->item->item;

   switch ( item1->type )
   {
      case HOR_POINT_3D:
      depth1 = proj1->proj.p.depth;
      break;

      case HOR_LINE_3D:
      depth1 = (proj1->proj.l.p1.depth + proj1->proj.l.p2.depth)/2.0;
      break;

      case HOR_FACET_3D:
      depth1 = (proj1->proj.f.p1.depth + proj1->proj.f.p2.depth +
		proj1->proj.f.p3.depth)/3.0;
      break;

      case HOR_LABEL_3D:
      depth1 = proj1->proj.p.depth;
      break;

      default: hor_error ( "illegal 3D item type (compare_func)", HOR_FATAL );
      break;
   }

   switch ( item2->type )
   {
      case HOR_POINT_3D:
      depth2 = proj2->proj.p.depth;
      break;

      case HOR_LINE_3D:
      depth2 = (proj2->proj.l.p1.depth + proj2->proj.l.p2.depth)/2.0;
      break;

      case HOR_FACET_3D:
      depth2 = (proj2->proj.f.p1.depth + proj2->proj.f.p2.depth +
		proj2->proj.f.p3.depth)/3.0;
      break;

      case HOR_LABEL_3D:
      depth2 = proj2->proj.p.depth;
      break;

      default: hor_error ( "illegal 3D item type (compare_func)", HOR_FATAL );
      break;
   }

   if      ( depth1 < depth2 ) return  1;
   else if ( depth1 > depth2 ) return -1;

   return 0;
}

/*******************
*   Hor_Bool @hor_write_3D_image ( Hor_Assoc_Label label_3D,
*                                  const char     *base_name,
*                                  int             image_size )
*
*   Writes the contents of a 3D tool to the given image file.
*   Currently only facets are written out, on a white background.
********************/
Hor_Bool hor_write_3D_image ( Hor_Assoc_Label label_3D, const char *base_name,
			      int image_size )
{
   Def_3D        *def;
   Hor_Assoc_List list_ptr;
   Internal_Item *item;
   Hor_Image     *image;
   u_char       **imarr;
   Hor_Impixel    pixel;
   Proj_3D       *proj_3D;
   int            item_count = 0, i;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (hor_write_3D_image)",
		  HOR_FATAL, label_3D );

   if ( !def->in_use )
   {
      hor_errno = HOR_TOOL_POPUP_PANEL_NOT_IN_USE;
      return HOR_FALSE;
   }

   proj_3D = hor_malloc_ntype ( Proj_3D, def->no_items );
   image = hor_alloc_image ( image_size, image_size, HOR_U_CHAR, NULL );
   if ( image == NULL ) return HOR_FALSE;
   imarr = image->array.uc;

   pixel.uc = 255; hor_fill_image_with_constant ( image, pixel );

   for ( list_ptr = def->list; list_ptr != NULL; list_ptr = list_ptr->next )
   {
      item = (Internal_Item *) hor_assoc_data(list_ptr);
      if ( item->status != DELETED )
	 project_item ( def, item, (double) image_size / (double) def->size,
		        &proj_3D[item_count++] );
   }

   qsort ( (void *) proj_3D, item_count, sizeof(Proj_3D), compare_func );
   for ( i = 0; i < item_count; i++ )
   {
      Proj_3D *proj = &proj_3D[i];
      item = proj->item;

      switch ( item->item.type )
      {
	 Hor_Item_3D_facet f;
	 int x1, y1, x2, y2, x3, y3, a1, b1, c1, a2, b2, c2, a3, b3, c3, c, r;
	 int min_c, min_r, max_c, max_r, im_c, im_r;
	 double denom;

	 case HOR_POINT_3D: break;
	 case HOR_LINE_3D:  break;

	 case HOR_FACET_3D:
	 f = item->item.u.facet;

	 min_c = max_c = x1 = proj->proj.f.p1.c;
	 min_r = max_r = y1 = proj->proj.f.p1.r;
	 if ( (x2 = proj->proj.f.p2.c) < min_c ) min_c = x2;
	 if ( x2 > max_c ) max_c = x2;
	 if ( (y2 = proj->proj.f.p2.r) < min_r ) min_r = y2;
	 if ( y2 > max_r ) max_r = y2;
	 if ( (x3 = proj->proj.f.p3.c) < min_c ) min_c = x3;
	 if ( x3 > max_c ) max_c = x3;
	 if ( (y3 = proj->proj.f.p3.r) < min_r ) min_r = y3;
	 if ( y3 > max_r ) max_r = y3;

	 /* force ranges of c and r to fit size of image */
	 if ( min_c < 0 ) min_c = 0;
	 else if ( min_c >= image_size ) continue;

	 if ( max_c >= image_size ) max_c = image_size-1;
	 else if ( max_c < 0 ) continue;

	 if ( min_r < 0 ) min_r = 0;
	 else if ( min_r >= image_size ) continue;

	 if ( max_r >= image_size ) max_r = image_size-1;
	 else if ( max_r < 0 ) continue;

	 /* check whether facet still goes clockwise, i.e. is being
	    displayed from the right side */
	 if ( (denom = (double)(x1*y2 - x2*y1 + x2*y3 - x3*y2 + x3*y1 - x1*y3))
	      < 0.0 ) continue;

	 a1 = y3 - y2; b1 = x2 - x3; c1 = x3*y2 - x2*y3;
	 a2 = y1 - y3; b2 = x3 - x1; c2 = x1*y3 - x3*y1;
	 a3 = y2 - y1; b3 = x1 - x2; c3 = x2*y1 - x1*y2;
	 switch ( f.image->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char **arr = f.image->array.uc;

	       for ( r = min_r; r < max_r; r++ )
		  for ( c = min_c; c < max_c; c++ )
		     if ( a1*c + b1*r + c1 <= 0 && a2*c + b2*r + c2 <= 0 &&
			  a3*c + b3*r + c3 <= 0 )
		     {
			/* project point onto image */
			im_c = (int) ((double)
			         (f.c1*((y2-y3)*c + (x3-x2)*r + x2*y3 - x3*y2)
			        + f.c2*((y3-y1)*c + (x1-x3)*r + x3*y1 - x1*y3)
			        + f.c3*((y1-y2)*c + (x2-x1)*r + x1*y2 - x2*y1))
			       / denom + 0.5);
			im_r = (int) ((double)
			         (f.r1*((y2-y3)*c + (x3-x2)*r + x2*y3 - x3*y2)
			        + f.r2*((y3-y1)*c + (x1-x3)*r + x3*y1 - x1*y3)
			        + f.r3*((y1-y2)*c + (x2-x1)*r + x1*y2 - x2*y1))
			       / denom + 0.5);
			if ( im_c >= 0 && im_c < f.image->width &&
			     im_r >= 0 && im_r < f.image->height )
			   imarr[r][c] = arr[im_r][im_c];
		     }
	    }
	    break;

	    default:
	    hor_error ( "illegal image type (hor_write_3D_image)",
		        HOR_FATAL );
	    break;
	 }
	 break;

	 case HOR_LABEL_3D: break;

	 default:
	 hor_error ( "illegal 3D item type (hor_write_3D_image)", HOR_FATAL );
	 break;
      }
   }

   if ( !hor_write_image ( base_name, image ) ) return HOR_FALSE;
   hor_free ( (void *) proj_3D );

   return HOR_TRUE;
}

/*+******** mouse-activated functions **********/

/*****
* static void mouse_leftdown (int col_canvas, int row_canvas, void *data)
*
* left mouse button down - set toggle and store mouse position
*****/
static void mouse_leftdown (int col_canvas, int row_canvas, void *data)
{
    Def_3D *def;

    def = (Def_3D *) data;

    def->mouse_leftdown_flag = HOR_TRUE;
    def->mouse_middledown_flag = HOR_FALSE;

    def->move_count = 1;
    def->move_array[0].x = col_canvas;
    def->move_array[0].y = row_canvas;
}

/*****
* static void mouse_leftup (int col_canvas, int row_canvas, void *data)
*
* left mouse button up - unset toggle
*****/
static void mouse_leftup (int col_canvas, int row_canvas, void *data)
{
    Def_3D *def;

    def = (Def_3D *) data;

    def->mouse_leftdown_flag = HOR_FALSE;
    display_items ( def );
}

/*****
* static void mouse_middledown (int col_canvas, int row_canvas, void *data)
*
* middle mouse button down - set toggle and store mouse position
*****/
static void mouse_middledown (int col_canvas, int row_canvas, void *data)
{
    Def_3D *def;

    def = (Def_3D *) data;

    def->mouse_leftdown_flag = HOR_FALSE;
    def->mouse_middledown_flag = HOR_TRUE;

    def->move_count = 1;
    def->move_array[0].x = col_canvas;
    def->move_array[0].y = row_canvas;
}

/*****
* static void mouse_middleup (int col_canvas, int row_canvas, void *data)
*
* middle mouse button up - unset toggle
*****/
static void mouse_middleup (int col_canvas, int row_canvas, void *data)
{
    Def_3D *def;

    def = (Def_3D *) data;

    def->mouse_middledown_flag = HOR_FALSE;
    display_items ( def );
}

/*****
* static void mouse_move (int col_canvas, int row_canvas, void *data)
*
* mouse motion - if left or middle buttons are down, route to the appropriate procedure.
*****/
static void mouse_move (int col_canvas, int row_canvas, void *data)
{
    Def_3D *def;

    def = (Def_3D *) data;

    if (def->mouse_leftdown_flag)
	mouse_rotate (col_canvas, row_canvas, def);

    else
    if (def->mouse_middledown_flag)
	mouse_spinzoom (col_canvas, row_canvas, def);
}

/*****
* static double distance_line_point ( double x1, double y1,
*                                     double x2, double y2,
*                                     double x, double y )
*
* Finds the distance between a line specified by its endpoints and a point.
*****/
static double distance_line_point ( double x1, double y1, double x2, double y2,
				    double x, double y )
{
   double gap = sqrt ( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );

   if ( gap >= 0.00001 )
   {
      if ( (x2-x1)*x + (y2-y1)*y + x1*(x1-x2) + y1*(y1-y2) < 0.0 )
	 return ( HOR_DISTANCE_2D ( x, y, x1, y1 ) );
      else if ( (x2-x1)*x + (y2-y1)*y + x2*(x1-x2) + y2*(y1-y2) > 0.0 )
	 return ( HOR_DISTANCE_2D ( x, y, x2, y2 ) );
      else
	 return ( fabs ( (x1*y2 - x2*y1 + y*(x2-x1) - x*(y2-y1))/gap ) );
   }

   return ( HOR_DISTANCE_2D ( x, y, x1, y1 ) );
}

/*****
* static void mouse_rightdown (int col_canvas, int row_canvas, void *data)
*****/
static void mouse_rightdown (int col_canvas, int row_canvas, void *data)
{
    double distance = 0.0, distance_min;

    Internal_Item  *item, *closest_item = NULL;
    Hor_Assoc_List  list_ptr;
    Hor_Assoc_Label closest_label = HOR_ASSOC_ERROR;
    BIVEC_DOUBLE    bivec1, bivec2;
    Def_3D *def;
    double x = (double) col_canvas + 0.5;
    double y = (double) row_canvas + 0.5;
    double depth;

    def = (Def_3D *) data;

    compute_rotmatrix (def, def->viewpoint_dir, def->viewpoint_up,
		       def->base_viewpoint_dir, def->base_viewpoint_up,
		       def->rot_mat);

    /* initialise distance_min to something bigger than the minimum which
       will be found. */

    distance_min = def->size * def->size;
    for ( list_ptr = def->list; list_ptr != NULL; list_ptr = list_ptr->next )
    {
	item = (Internal_Item *) hor_assoc_data (list_ptr);

	if (item->status != DELETED)
	{
	   switch ( item->item.type )
	   {
	      case HOR_POINT_3D:
	      if ( !def->pick_points )
	      {
		 distance = distance_min + 1.0;
		 break;
	      }

	      bivec1 = proj_point ( def, item->item.u.point.p, &depth );
	      distance = HOR_DISTANCE_2D(x, y, bivec1.x, bivec1.y);
	      break;

	      case HOR_LINE_3D:
	      if ( !def->pick_lines )
	      {
		 distance = distance_min + 1.0;
		 break;
	      }

	      proj_line ( def, item->item.u.line.p1, item->item.u.line.p2,
			  &bivec1, &depth, &bivec2, &depth );
	      distance = distance_line_point ( bivec1.x, bivec1.y,
					       bivec2.x, bivec2.y, x, y );
	      break;

	      case HOR_FACET_3D:
	      if ( !def->pick_facets )
	      {
		 distance = distance_min + 1.0;
		 break;
	      }

	      proj_line ( def, item->item.u.facet.p1, item->item.u.facet.p2,
			  &bivec1, &depth, &bivec2, &depth );
	      distance = distance_line_point ( bivec1.x, bivec1.y,
					       bivec2.x, bivec2.y, x, y );
	      proj_line ( def, item->item.u.facet.p2, item->item.u.facet.p3,
			  &bivec1, &depth, &bivec2, &depth );
	      distance = hor_dmin ( distance,
				    distance_line_point ( bivec1.x, bivec1.y,
							  bivec2.x, bivec2.y,
							  x, y ) );
	      proj_line ( def, item->item.u.facet.p3, item->item.u.facet.p1,
			  &bivec1, &depth, &bivec2, &depth );
	      distance = hor_dmin ( distance,
				    distance_line_point ( bivec1.x, bivec1.y,
							  bivec2.x, bivec2.y,
							  x, y ) );
	      break;

	      case HOR_LABEL_3D:
	      distance = distance_min + 1.0;
	      break;

	      default:
	      hor_error ( "illegal item type (mouse_rightdown)", HOR_FATAL );
	      break;
	   }

	   if (distance < distance_min)
	   {
	      distance_min = distance;
	      closest_label = list_ptr->label;
	      closest_item  = item;
	   }
	}
    }

    if (closest_item != NULL)
    {
       switch ( def->mode )
       {
	  case SELECT_MODE:
	  if (def->oneselect_func != NULL)
	     def->oneselect_func ( closest_label, closest_item->item );

	  closest_item->status = SELECTED;
	  break;

	  case DELETE_MODE:
	  if (def->onedelete_func != NULL)
	     def->onedelete_func ( closest_label, closest_item->item );

	  closest_item->status = DELETED;
	  break;

	  default:
	  hor_error ( "illegal mouse mode (mouse_rightdown)", HOR_FATAL );
	  break;
       }

       display_single_item ( def, closest_item );
    }
}

/*****
* static void mouse_rotate (int col_canvas, int row_canvas, Def_3D *def)
*
* rotate around an axis which (a) is perpendicular to the mouse motion and (b)
* passes through the origin of the 3D data.
*****/
static void mouse_rotate (int col_canvas, int row_canvas, Def_3D *def)
{

    double mouse_distance,
           axis_x, axis_y;

    int ripple_index;

    /* store the motion.
     */

    for (ripple_index = MOVE_COUNT - 1; ripple_index >= 1; ripple_index--)
    {
	def->move_array[ripple_index] = def->move_array[ripple_index - 1];
    }
    def->move_array[0].x = col_canvas;
    def->move_array[0].y = row_canvas;

    if (def->move_count < MOVE_COUNT)
	def->move_count++;

    else
    {
	mouse_distance = HOR_DISTANCE_2D(def->move_array[0].x,
					 def->move_array[0].y,
					 def->move_array[MOVE_COUNT - 1].x,
					 def->move_array[MOVE_COUNT - 1].y);

	if (mouse_distance > 0)
	{
	    /* desired rotation axis is perp to the mouse motion - hence the
	       swopping of x,y coordinates below. */

	    axis_x = (double) (def->move_array[0].y
			       - def->move_array[MOVE_COUNT - 1].y);
	    axis_y = (double) -(def->move_array[0].x
				- def->move_array[MOVE_COUNT - 1].x);

 	    /* set up the 3D rotation axis passing through the origin of the
	       data set in the coordinate frame seen from the current
	       viewpoint. axis1 and axis2 are two points on the axis. */

	    hor_matq_fill ( vec1,    0.0,    0.0, def->viewpoint_distance );
	    hor_matq_fill ( vec2, axis_x, axis_y, def->viewpoint_distance );

	    /* transform to the base frame.
	       axis is a vector along the axis in the base frame. */
	    transform_to_baseframe (def, vec1, vec3);
	    hor_matq_copy (vec3, vec1);
	    transform_to_baseframe (def, vec2, vec3);
	    hor_matq_copy (vec3, vec2);

	    hor_matq_sub (vec1, vec2, vec3);

	    /* compute the rotation matrix. */
	    if (mouse_distance < 5.0)
		axis_to_rotmatrix (ROTATE_ANGLE, vec3, def->rot_mat);
	    else
		axis_to_rotmatrix ((mouse_distance / 5.0) * ROTATE_ANGLE,
				   vec3, def->rot_mat);

	    /* update the viewpoint and viewpoint_up vectors. */
	    hor_matq_prod2 (def->rot_mat, def->viewpoint_dir, vec1);
	    hor_matq_copy  (vec1, def->viewpoint_dir);
	    hor_matq_prod2 (def->rot_mat, def->viewpoint_up, vec1);
	    hor_matq_copy  (vec1, def->viewpoint_up);

	    /*	make sure numerical errors don't slip in to make viewpoint_dir
		not perp to viewpoint_up. */
	    hor_vecq_cross_prod (def->viewpoint_dir, def->viewpoint_up, vec1);
	    hor_vecq_cross_prod (vec1, def->viewpoint_dir, def->viewpoint_up);

	    display_items (def);
	}
    }
}

/*****
* static void mouse_spinzoom (int col_canvas, int row_canvas, Def_3D *def)
*
* break the mouse motion into radial and tangential components relative to the
* canvas centre. if mainly tangential, cyclotort around the current line of
* sight. else zoom in or out according to the direction of the radial motion.
*****/
static void mouse_spinzoom (int col_canvas, int row_canvas, Def_3D *def)
{
    double mouse_distance,
           new_theta,
           old_theta,
           radial_distance,
           tangent_distance,
           tangent_squared;

    BIVEC_DOUBLE new_radial,
                    old_radial;

    /* decompose the mouse motion into radial and tangential components.
     */

    mouse_distance = HOR_DISTANCE_2D (def->move_array[0].x,
				      def->move_array[0].y,
				      col_canvas, row_canvas);
    radial_distance = HOR_DISTANCE_2D (def->size / 2, def->size / 2,
				       col_canvas,    row_canvas) 
                     -HOR_DISTANCE_2D (def->size / 2, def->size / 2,
				       def->move_array[0].x,
				       def->move_array[0].y);
    tangent_squared = mouse_distance * mouse_distance
                     -radial_distance * radial_distance;
    if (tangent_squared <= 0) /* just in case of numerical error.*/
	tangent_distance = 0;
    else
	tangent_distance = sqrt (tangent_squared);

    /* test for zoom condition.
     */

    if (fabs (radial_distance) > tangent_distance)
    {
#if 1
	if (radial_distance < 0)
	   def->focal_length /= ZOOM_FACTOR;
	else
	   def->focal_length *= ZOOM_FACTOR;
#else
	if (radial_distance < 0)
	   def->viewpoint_distance *= ZOOM_FACTOR;
	else
	   def->viewpoint_distance /= ZOOM_FACTOR;
#endif
    }

    /* else cyclotort.
     */

    else
    {
	old_radial.x = def->move_array[0].x - def->size / 2;
	old_radial.y = def->move_array[0].y - def->size / 2;
	new_radial.x = col_canvas - def->size / 2;
	new_radial.y = row_canvas - def->size / 2;

	if ((old_radial.x != 0 || old_radial.y != 0) &&
	    (new_radial.x != 0 || new_radial.y != 0))
	{
	    old_theta = atan2 (old_radial.y, old_radial.x);
	    new_theta = atan2 (new_radial.y, new_radial.x);

	    /* compute the rotation matrix and update viewpoint_up.
	     */

	    axis_to_rotmatrix(new_theta - old_theta, def->viewpoint_dir,
			      def->rot_mat);
	    hor_matq_prod2 (def->rot_mat, def->viewpoint_up, vec2);
	    hor_matq_copy (vec2, def->viewpoint_up);

	    /*	make sure numerical errors don't slip in to make viewpoint not
		perp to viewpoint_up. */

	    hor_vecq_cross_prod (def->viewpoint_dir, def->viewpoint_up,
			     vec2);
	    hor_vecq_cross_prod (vec2, def->viewpoint_dir,
			     def->viewpoint_up);
	}
    }

    def->move_array[0].x = col_canvas;
    def->move_array[0].y = row_canvas;

    display_items (def);
}

/*+******** button procedures **********/

static void write_to_text_file (Widget button, XtPointer client_data,
				               XtPointer call_data)
{
   Hor_Assoc_Label label_3D = (Hor_Assoc_Label) ((int) client_data);
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (write_to_text_file)",
		  HOR_FATAL, label_3D );
   
   if ( !hor_write_3D_text(label_3D,
			   XowPanelTextGetValueString(def->base_name_widget)))
      hor_perror ( "Writing 3D text file" );
}

static void write_to_image_file (Widget button, XtPointer client_data,
				                XtPointer call_data)
{
   int image_size;
   Hor_Assoc_Label label_3D = (Hor_Assoc_Label) ((int) client_data);
   Def_3D *def;

   def = (Def_3D *) hor_assoc_find ( list_3D, label_3D );
   if ( def == NULL )
      hor_error ( "3D tool with label %d does not exist (write_to_image_file)",
		  HOR_FATAL, label_3D );
   

   if ( !hor_read_int_param ( hor_get_widget_value(def->image_size_widget),
			      "image size", hor_int_pos, HOR_NON_FATAL,
			      &image_size ) )
      return;

   if ( !hor_write_3D_image ((Hor_Assoc_Label) ((int) client_data),
			     XowPanelTextGetValueString(def->base_name_widget),
			     image_size) )
      hor_perror ( "Writing 3D image file" );
}

/*****
*
static void command_process_selection (Widget button, XtPointer client_data,
*                                      XtPointer call_data)
*
* invoke the user callback for processing the set of selected points.
*
*****/
static void command_process_selection (Widget button, XtPointer client_data,
				       XtPointer call_data)
{
   Hor_Assoc_List item_list = NULL, list;
   Internal_Item *item;
   Def_3D        *def;

   def = (Def_3D *) client_data;

   for ( list = def->list; list != NULL; list = list->next )
   {
      item = (Internal_Item *) hor_assoc_data (list);
      if ( item->status == SELECTED )
	 item_list = hor_assoc_insert ( item_list, list->label,
				        (void *) &(item->item) );
   }

   if (item_list == NULL) hor_warning ( "no 3D items selected" );
   else
   {
      if (def->procselected_func != NULL) def->procselected_func (item_list);
      hor_assoc_free_nodes (item_list);
   }
}

/*****
* static void clear_selection (Widget button, XtPointer client_data,
*                              XtPointer call_data)
*
* unselect all selected points.
*
*****/
static void clear_selection (Widget button, XtPointer client_data,
			     XtPointer call_data)
{
   Hor_Assoc_List item_list = NULL, list;
   Def_3D        *def;
   Internal_Item *item;

   def = client_data;
   for ( list = def->list; list != NULL; list = list->next )
   {
      item = (Internal_Item *) list->data;
      if ( item->status == SELECTED )
      {
	 item->status = UNSELECTED;
	 item_list = hor_assoc_insert ( item_list, list->label,
				        (void *) &(item->item) );
      }
   }

   if (item_list == NULL) hor_warning ( "no 3D items selected" );
   else
   {
      if (def->allclear_func != NULL) def->allclear_func (item_list);
      for ( list = item_list; list != NULL; list = list->next )
	 display_single_item ( def, (Internal_Item *) list->data );

      hor_assoc_free_nodes (item_list);
   }
}

/*****
* static void restore_deleted (Widget button, XtPointer client_data,
*                              XtPointer call_data)
*
* restores all deleted points.
*
*****/
static void restore_deleted (Widget button, XtPointer client_data,
			     XtPointer call_data)
{
   Hor_Assoc_List list, item_list = NULL;
   Def_3D        *def;
   Internal_Item *item;

   def = client_data;
   for ( list = def->list; list != NULL; list = list->next )
   {
      item = (Internal_Item *) hor_assoc_data (list);
      if ( item->status == DELETED )
      {
	 item->status = UNSELECTED;
	 item_list = hor_assoc_insert ( item_list, list->label,
				        (void *) &(item->item) );
      }
   }

   if (item_list == NULL) hor_warning ( "no 3D items restored" );
   else
   {
      if (def->allrestore_func != NULL) def->allrestore_func (item_list);
      for ( list = item_list; list != NULL; list = list->next )
	 display_single_item ( def, (Internal_Item *) list->data );

      hor_assoc_free_nodes (item_list);
   }
}

/*****
*
* static void command_done (Widget button, XtPointer client_data,
*                           XtPointer call_data)
*
* free memory and invoke the user callback for terminal processing of the 3d
* list.
*
*****/
static void command_done (Widget button, XtPointer client_data,
			  XtPointer call_data)
{
   Hor_Assoc_List list, item_list = NULL;
   Def_3D        *def;
   Internal_Item *item;

   def = client_data;
   for ( list = def->list; list != NULL; list = list->next )
   {
      item = (Internal_Item *) hor_assoc_data (list);
      if ( item->status == SELECTED )
	 item_list = hor_assoc_insert ( item_list, list->label,
				        (void *) &(item->item) );
   }

   if (def->done_func != NULL) def->done_func (item_list);

   hor_assoc_free_nodes (item_list);

   def->in_use      = HOR_FALSE;
   def->initialised = HOR_FALSE;

   hor_assoc_free ( def->list, hor_3D_item_free );
   XtPopdown (def->pick_popup_frame);
   XtPopdown (def->popup_frame);
}

/*****
*
static void command_setproj (Widget button, XtPointer client_data,
*                                           XtPointer call_data)
*
* set the projection type.
*
*****/
static void command_setproj (Widget button, XtPointer client_data,
			                    XtPointer call_data)
{
    Def_3D *def;

    def = (Def_3D *) client_data;
    def->proj_type = (int) call_data;
    if ( def->initialised ) display_items(def);
}

/*****
*
static void command_setmode (Widget button, XtPointer client_data,
*                                           XtPointer call_data)
*
* set the mode.
*
*****/
static void command_setmode (Widget button, XtPointer client_data,
			                    XtPointer call_data)
{
    Def_3D *def;

    def = (Def_3D *) client_data;
    def->mode = (int) call_data;
}

/*****
*
static void command_setcoord (Widget button, XtPointer client_data,
*                                            XtPointer call_data)
*
* set the coordinate system.
*
*****/
static void command_setcoord (Widget button, XtPointer client_data,
			                     XtPointer call_data)
{
   Def_3D        *def;

   def = (Def_3D *) client_data;
   def->coord_system = (int) call_data;
   
   if ( def->initialised ) display_items(def);
}

/*****
*
static void command_restart (Widget button, XtPointer client_data,
*                            XtPointer call_data)
*
* back to the initial display, undeleting all deletions and unselecting all selections.
*
*****/
static void command_restart (Widget button, XtPointer client_data,
			     XtPointer call_data)
{
   Def_3D        *def;

   def = client_data;
   def->mouse_leftdown_flag = HOR_FALSE;
   def->mouse_middledown_flag = HOR_FALSE;

   if ( def->subtract_centroid )
   {
      if ( !calc_centroid ( def->list, def->offset ) )
      {
	 hor_warning ( "no points to display" );
	 hor_matq_zero ( def->offset );
      }
   }
   else
      hor_matq_zero ( def->offset );

   /* pick a sensible distance away for the initial viewpoint.
    */

   def->viewpoint_distance = PROJECTION_SCALE*
                             max_coordinate ( def->list, def->offset );
   def->focal_length = FOCAL_LENGTH;

   /* return to the initial viewpoint.
    */

   hor_matq_fill ( def->viewpoint_dir, 0.0,  0.0, -1.0 );
   hor_matq_fill ( def->viewpoint_up,  0.0, -1.0,  0.0 );

   /* display the 3d data.
    */

   display_items (def);
   def->initialised = HOR_TRUE;
}

/*+******** utilities **********/

/*****
* static void axis_to_rotmatrix (double angle, Hor_Matrix *axis,
*                                Hor_Matrix *rot_matrix)
*
* convert a rotation around an axis to a rotation matrix.
*
*****/
static void axis_to_rotmatrix (double angle, Hor_Matrix *axis,
			       Hor_Matrix *rot_matrix)
{
    double av0, av1, av2, sa = sin (angle), ca = cos (angle);

    hor_vecq_unit (axis);

    hor_mat_read ( axis, &av0, &av1, &av2 );
    hor_matq_fill(rot_matrix,
	      ca + hor_sqr (av0) * (1.0 - ca),
	      av0 * av1 * (1.0 - ca) - av2 * sa,
	      av0 * av2 * (1.0 - ca) + av1 * sa,
	      av0 * av1 * (1.0 - ca) + av2 * sa,
	      ca + hor_sqr (av1) * (1.0 - ca),
	      av1 * av2 * (1.0 - ca) - av0 * sa,
	      av0 * av2 * (1.0 - ca) - av1 * sa,
	      av1 * av2 * (1.0 - ca) + av0 * sa,
	      ca + hor_sqr (av2) * (1.0 - ca) );
}

/*****
*
* static void compute_rotmatrix (Def_3D *def,
*                                Hor_Matrix *source1, Hor_Matrix *source2,
*                                Hor_Matrix *target1, Hor_Matrix *target2,
*                                Hor_Matrix *rot_matrix)
*
* find the rotation which transforms source1/source2 vectors to
* target1/target2. there must be better approaches than this, but don't know
* them.
*****/
static void compute_rotmatrix (Def_3D *def,
			       Hor_Matrix *source1, Hor_Matrix *source2,
			       Hor_Matrix *target1, Hor_Matrix *target2,
			       Hor_Matrix *rot_matrix)
{

    int row_index;

    Hor_Matrix *nr_x;

    hor_vecq_unit (source1);
    hor_vecq_unit (source2);
    hor_vecq_unit (target1);
    hor_vecq_unit (target2);

    hor_vecq_cross_prod (source1, source2, vec4);
    hor_vecq_unit (vec4);
    hor_vecq_cross_prod (target1, target2, vec5);
    hor_vecq_unit (vec5);
    
    /* set up the linear system  R s_i = t_i, i=1..3, and solve for R.
       naming is consistent with the Numerical Recipes names for variables used
       by the routines ludcmp and lubksb. */

    hor_matq_zero ( a_mat );

    row_index = 0;
    a_mat->m[row_index][0] = source1->m[0][0];
    a_mat->m[row_index][1] = source1->m[1][0];
    a_mat->m[row_index][2] = source1->m[2][0];
    b_vec->m[row_index][0] = target1->m[0][0];
    row_index++;

    a_mat->m[row_index][3] = source1->m[0][0];
    a_mat->m[row_index][4] = source1->m[1][0];
    a_mat->m[row_index][5] = source1->m[2][0];
    b_vec->m[row_index][0] = target1->m[1][0];
    row_index++;

    a_mat->m[row_index][6] = source1->m[0][0];
    a_mat->m[row_index][7] = source1->m[1][0];
    a_mat->m[row_index][8] = source1->m[2][0];
    b_vec->m[row_index][0] = target1->m[2][0];
    row_index++;

    a_mat->m[row_index][0] = source2->m[0][0];
    a_mat->m[row_index][1] = source2->m[1][0];
    a_mat->m[row_index][2] = source2->m[2][0];
    b_vec->m[row_index][0] = target2->m[0][0];
    row_index++;

    a_mat->m[row_index][3] = source2->m[0][0];
    a_mat->m[row_index][4] = source2->m[1][0];
    a_mat->m[row_index][5] = source2->m[2][0];
    b_vec->m[row_index][0] = target2->m[1][0];
    row_index++;

    a_mat->m[row_index][6] = source2->m[0][0];
    a_mat->m[row_index][7] = source2->m[1][0];
    a_mat->m[row_index][8] = source2->m[2][0];
    b_vec->m[row_index][0] = target2->m[2][0];
    row_index++;

    a_mat->m[row_index][0] = vec4->m[0][0];
    a_mat->m[row_index][1] = vec4->m[1][0];
    a_mat->m[row_index][2] = vec4->m[2][0];
    b_vec->m[row_index][0] = vec5->m[0][0];
    row_index++;

    a_mat->m[row_index][3] = vec4->m[0][0];
    a_mat->m[row_index][4] = vec4->m[1][0];
    a_mat->m[row_index][5] = vec4->m[2][0];
    b_vec->m[row_index][0] = vec5->m[1][0];
    row_index++;

    a_mat->m[row_index][6] = vec4->m[0][0];
    a_mat->m[row_index][7] = vec4->m[1][0];
    a_mat->m[row_index][8] = vec4->m[2][0];
    b_vec->m[row_index][0] = vec5->m[2][0];
    row_index++;

    nr_x = hor_mats_solve (a_mat, b_vec);

    rot_matrix->m[0][0] = nr_x->m[0][0];
    rot_matrix->m[0][1] = nr_x->m[1][0];
    rot_matrix->m[0][2] = nr_x->m[2][0];
    rot_matrix->m[1][0] = nr_x->m[3][0];
    rot_matrix->m[1][1] = nr_x->m[4][0];
    rot_matrix->m[1][2] = nr_x->m[5][0];
    rot_matrix->m[2][0] = nr_x->m[6][0];
    rot_matrix->m[2][1] = nr_x->m[7][0];
    rot_matrix->m[2][2] = nr_x->m[8][0];

    hor_mat_free (nr_x);
}

/*****
* static void display_single_item (Def_3D *def, Internal_Item *item)
*
* update display of single item.
*****/
static void display_single_item (Def_3D *def, Internal_Item *item)
{
   Hor_Assoc_Label old_label = hor_display_get_window();
   Proj_3D         proj_3D;

   hor_display_reset_window(def->canvas_label);
   compute_rotmatrix (def,
		      def->viewpoint_dir, def->viewpoint_up,
		      def->base_viewpoint_dir,
		      def->base_viewpoint_up, def->rot_mat);
   project_item ( def, item, 1.0, &proj_3D );
   colour_item ( def, &proj_3D, HOR_FALSE );
   hor_display_reset_window (old_label);
}

/*****
* static void display_items (Def_3D *def)
*
* update display.
*****/
static void display_items (Def_3D *def)
{
   Hor_Assoc_List  list_ptr;
   Hor_Assoc_Label old_label = hor_display_get_window();
   Proj_3D        *proj_3D = hor_malloc_ntype (Proj_3D, def->no_items);
   Internal_Item  *item;
   int             item_count = 0, i;

   hor_display_reset_window(def->canvas_label);
   hor_display_clear (def->background_colour);

   compute_rotmatrix (def,
		      def->viewpoint_dir, def->viewpoint_up,
		      def->base_viewpoint_dir,
		      def->base_viewpoint_up, def->rot_mat);

   for ( list_ptr = def->list; list_ptr != NULL; list_ptr = list_ptr->next )
   {
      item = (Internal_Item *) hor_assoc_data (list_ptr);
      if ( item->status != DELETED )
	 project_item ( def, item, 1.0, &proj_3D[item_count++] );
   }

   qsort ( (void *) proj_3D, item_count, sizeof(Proj_3D), compare_func );
   for ( i = 0; i < item_count; i++ )
      colour_item ( def, &proj_3D[i],
		    def->mouse_leftdown_flag || def->mouse_middledown_flag );

   hor_free ( (void *) proj_3D );
   hor_display_reset_window (old_label);
}

/*****
* BIVEC_DOUBLE project_point (Def_3D *def, Hor_Matrix *vec)
*
* project a 3d point to its 2d display position.
*****/
BIVEC_DOUBLE project_point (Def_3D *def, Hor_Matrix *vec)
{
   BIVEC_DOUBLE bivec;
   double       size_2 = (double) def->size/2.0;

   switch ( def->proj_type )
   {
      case PERSPECTIVE_PROJ:
      /* if the point is behind the camera, return some coordinates off the
	 display. */

      if (vec->m[2][0] <= 0.0)
      {
	 bivec.x = -1.0;
	 bivec.y = -1.0;
      }
      else
      {
	 bivec.x = size_2*(1.0 + def->focal_length*vec->m[0][0]/vec->m[2][0]);
	 bivec.y = size_2*(1.0 + def->focal_length*vec->m[1][0]/vec->m[2][0]);
      }
      break;

      case ORTHOGRAPHIC_PROJ:
      bivec.x = size_2*(1.0 + def->focal_length*vec->m[0][0]/def->viewpoint_distance);
      bivec.y = size_2*(1.0 + def->focal_length*vec->m[1][0]/def->viewpoint_distance);
      break;

      default:
      hor_error ( "illegal projection type (project_point)", HOR_FATAL );
      break;
   }

   return bivec;
}

/*****
* static void project_line (Def_3D *def,
*                           Hor_Matrix *vec1, Hor_Matrix *vec2,
*                           BIVEC_DOUBLE *bivec1_ptr, BIVEC_DOUBLE *bivec2_ptr)
*
* project the endpoints of a 3D line to their 2d display positions.
*
*****/
static void project_line (Def_3D *def,
			  Hor_Matrix *vec1, Hor_Matrix *vec2,
			  BIVEC_DOUBLE *bivec1_ptr, BIVEC_DOUBLE *bivec2_ptr)
{
   double mu, plane_x, plane_y, size_2 = (double) def->size/2.0;

   switch ( def->proj_type )
   {
      case PERSPECTIVE_PROJ:
      /* if both points behind the camera, return some coordinates off the
	 display. */

      if (vec1->m[2][0] <= 0.0 && vec2->m[2][0] <= 0.0)
      {
	 bivec1_ptr->x = -1.0;
	 bivec1_ptr->y = -1.0;

	 bivec2_ptr->x = -1.0;
	 bivec2_ptr->y = -1.0;
      }
      else if (vec1->m[2][0] <= 0.0)
      {
	 mu = (def->focal_length - vec1->m[2][0])
	      /(vec2->m[2][0] - vec1->m[2][0]);

	 /* compute the intersection point of the line with the plane Z=f. */
	 plane_x = vec1->m[0][0] + mu*(vec2->m[0][0] - vec1->m[0][0]);
	 plane_y = vec1->m[1][0] + mu*(vec2->m[1][0] - vec1->m[1][0]);

	 bivec1_ptr->x = size_2*(1.0 + plane_x);
	 bivec1_ptr->y = size_2*(1.0 + plane_y);

	 bivec2_ptr->x = size_2*(1.0 + def->focal_length*vec2->m[0][0]/vec2->m[2][0]);
	 bivec2_ptr->y = size_2*(1.0 + def->focal_length*vec2->m[1][0]/vec2->m[2][0]);
      }
      else if (vec2->m[2][0] <= 0.0)
      {
	 mu = (def->focal_length - vec2->m[2][0])
	      /(vec1->m[2][0] - vec2->m[2][0]);

	 /* compute the intersection point of the line with the plane Z=1. */
	 plane_x = vec2->m[0][0] + mu*(vec1->m[0][0] - vec2->m[0][0]);
	 plane_y = vec2->m[1][0] + mu*(vec1->m[1][0] - vec2->m[1][0]);

	 bivec1_ptr->x = size_2*(1.0 + plane_x);
	 bivec1_ptr->y = size_2*(1.0 + plane_y);

	 bivec2_ptr->x = size_2*(1.0 + def->focal_length*vec1->m[0][0]/vec1->m[2][0]);
	 bivec2_ptr->y = size_2*(1.0 + def->focal_length*vec1->m[1][0]/vec1->m[2][0]);
      }
      else
      {
	 bivec1_ptr->x = size_2*(1.0 + def->focal_length*vec1->m[0][0]/vec1->m[2][0]);
	 bivec1_ptr->y = size_2*(1.0 + def->focal_length*vec1->m[1][0]/vec1->m[2][0]);

	 bivec2_ptr->x = size_2*(1.0 + def->focal_length*vec2->m[0][0]/vec2->m[2][0]);
	 bivec2_ptr->y = size_2*(1.0 + def->focal_length*vec2->m[1][0]/vec2->m[2][0]);
      }
      break;

      case ORTHOGRAPHIC_PROJ:
      bivec1_ptr->x = size_2*(1.0 + def->focal_length*vec1->m[0][0]/def->viewpoint_distance);
      bivec1_ptr->y = size_2*(1.0 + def->focal_length*vec1->m[1][0]/def->viewpoint_distance);

      bivec2_ptr->x = size_2*(1.0 + def->focal_length*vec2->m[0][0]/def->viewpoint_distance);
      bivec2_ptr->y = size_2*(1.0 + def->focal_length*vec2->m[1][0]/def->viewpoint_distance);
      break;

      default:
      hor_error ( "illegal projection type (project_line)", HOR_FATAL );
      break;
   }
}

/*****
* static void transform_from_baseframe (Def_3D *def,
*                                       Hor_Matrix *base,
*                                       Hor_Matrix *view)
*
* transform from the base frame to the viewpoint frame.
*
* !!!!!! set up def->rot_mat before invoking !!!!!!
*
*****/
static void transform_from_baseframe (Def_3D *def,
				      Hor_Matrix *base,
				      Hor_Matrix *view)
{
   hor_matq_prod2 (def->rot_mat, base, view);
   view->m[2][0] += def->viewpoint_distance;
}

/*****
* static void transform_to_baseframe (Def_3D *def, Hor_Matrix *view,
*                                     Hor_Matrix *base)
*
* transform to the base frame from the viewpoint frame.
*****/
static void transform_to_baseframe (Def_3D *def, Hor_Matrix *view,
				    Hor_Matrix *base)
{

    /* note: the invocation parameter "base" is sometimes equal to
       vec1/vec2, so don't attempt to reuse those vectors here.
     */

   compute_rotmatrix (def, def->base_viewpoint_dir,
		           def->base_viewpoint_up,
		           def->viewpoint_dir,
		           def->viewpoint_up, def->rot_mat);

   hor_matq_copy (view, vec4);
   vec4->m[2][0] -= def->viewpoint_distance;
   hor_matq_prod2 (def->rot_mat, vec4, base);
}

/* These functions: Copyright Jason Merron, March 1994 */

/*******************
*   Hor_Item_3D *@hor_alloc_item_3D ( int item_type )
*
*   Allocates the memory associated with a 3D item, except that it doesn't
*   allocate space for a facet image.
********************/
Hor_Item_3D *hor_alloc_item_3D ( int item_type )
{
   Hor_Item_3D *item;

   if ((item = hor_malloc_type (Hor_Item_3D)) == NULL)
   {
      hor_errno = HOR_TOOL_ALLOCATION_FAILED;
      return NULL;
   }

   item->type = item_type;

   switch (item->type)
   {
      case HOR_POINT_3D:
      if ( (item->u.point.p = hor_mat_alloc (3, 1)) == NULL )
      { hor_free ( item ); return NULL; }
      break;

      case HOR_LINE_3D:
      if ( (item->u.line.p1 = hor_mat_alloc (3, 1)) == NULL )
      { hor_free ( item ); return NULL; }

      if ( (item->u.line.p2 = hor_mat_alloc (3, 1)) == NULL )
      { hor_mat_free ( item->u.line.p1 ); hor_free ( item ); return NULL; }

      break;
  
      case HOR_FACET_3D:
      if ( (item->u.facet.p1 = hor_mat_alloc (3, 1)) == NULL )
      { hor_free ( item ); return NULL; }

      if ( (item->u.facet.p2 = hor_mat_alloc (3, 1)) == NULL )
      { hor_mat_free ( item->u.facet.p1 ); hor_free ( item ); return NULL; }

      if ( (item->u.facet.p3 = hor_mat_alloc (3, 1)) == NULL )
      { hor_mat_free_list ( item->u.facet.p2, item->u.facet.p1, NULL );
	hor_free ( item ); return NULL; }

      item->u.facet.image = NULL;
      break;

      default:
      hor_error ( "illegal 3D item type '%d' (hor_alloc_item_3D)", item_type, 
		  HOR_FATAL );
   }

   return item;
}

/* this function adapted from Phil McLauchlan's matric.c source code, for
   filling a 3x1 matrix */
static void mat31_fill (double **M, va_list *aptr )
{
   int r;

   for (r = 0; r < 3; r++)
     M[r][0] = va_arg (*aptr, double);
}

static Hor_Item_3D *fill_item_3D (Hor_Item_3D *item, va_list *valist)
{
   switch (item->type)
   {
      case HOR_POINT_3D:
      mat31_fill (item->u.point.p->m, valist);
      break;

      case HOR_LINE_3D:
      mat31_fill (item->u.line.p1->m, valist);
      mat31_fill (item->u.line.p2->m, valist);
      break;

      case HOR_FACET_3D:
      mat31_fill (item->u.facet.p1->m, valist);
      mat31_fill (item->u.facet.p2->m, valist);
      mat31_fill (item->u.facet.p3->m, valist);
      item->u.facet.c1 = va_arg (*valist, int);
      item->u.facet.r1 = va_arg (*valist, int);
      item->u.facet.c2 = va_arg (*valist, int);
      item->u.facet.r2 = va_arg (*valist, int);
      item->u.facet.c3 = va_arg (*valist, int);
      item->u.facet.r3 = va_arg (*valist, int);
      item->u.facet.image = va_arg (*valist, Hor_Image *);
      break;

      default:
      hor_error("illegal 3D item type '%d' (fill_item_3D)", item->type, HOR_FATAL);
   }

   return item;
}

/*******************
*   Hor_Item_3D *@hor_fill_item_3D ( Hor_Item_3D *item, ... )
*
*   Takes all the coordinates for a 3D item in one go, thereby reducing
*   the risk of coding errors.  The arguments are passed in the same order
*   that they appear in the horatio "improc.h" header file.  For example, 
*   to fill a facet item, the arguments would be:
*
*    (item_ptr, p1_x, p1_y, p1_z, p2_..., p3_..., c1, r1, c2, ..., image_ptr)
********************/
Hor_Item_3D *hor_fill_item_3D ( Hor_Item_3D *item, ... )
{
   va_list  ap;

   va_start (ap, item);
   fill_item_3D (item, &ap);
   va_end (ap);

   return item;
}

/*******************
*   Hor_Item_3D *@hor_make_item_3D ( int item_type, ... )
*
*   Does the same as hor_alloc_item_3D followed by hor_fill_item_3D.
********************/
Hor_Item_3D *hor_make_item_3D ( int item_type, ... )
{
   va_list      ap;
   Hor_Item_3D *item;

   item = hor_alloc_item_3D (item_type);
   if ( item != NULL )
   {
      va_start (ap, item_type);
      fill_item_3D (item, &ap);
      va_end (ap);
   }

   return item;
}

/*******************
*   void hor_free_item_3D ( Hor_Item_3D *item )
*
*   Frees the memory associated with a 3D item, except for the image data.  
*   Cast this function to (void *) when passing it as the action argument 
*   in "hor_list_action".
********************/
void hor_free_item_3D ( Hor_Item_3D *item )
{
   switch (item->type)
   {
      case HOR_POINT_3D:
      hor_mat_free (item->u.point.p);
      break;
    
      case HOR_LINE_3D:
      hor_mat_free (item->u.line.p2);
      hor_mat_free (item->u.line.p1);
      break;
    
      case HOR_FACET_3D:
      if ( item->u.facet.image != NULL ) hor_free_image (item->u.facet.image);
      hor_mat_free (item->u.facet.p3);
      hor_mat_free (item->u.facet.p2);
      hor_mat_free (item->u.facet.p1);
      break;
    
      default:
      hor_error ("illegal 3D item type (hor_free_item_3D)", HOR_FATAL);
   }

   hor_free ((void *) item);
}
