#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <X11/Intrinsic.h>
#include <X11/cursorfont.h>
#include <X11/StringDefs.h>
#include <X11/Shell.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/AsciiText.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/Command.h>
#include <X11/Xow/Canvas.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

static XtAppContext app_con;

/* colour identifiers */
u_long Red, Green, Blue, Yellow, SteelBlue, LightSeaGreen, thistle, Cyan;

static char base_name[100];
static int  canvas_size = 512;

static String fallback_resources[] = {
   "*input: True",
   "*text*editType:          append",
   "*text*scrollVertical:    Always",
   "*text*height:            100",
   NULL,
 };

static void print_horatio_args ( void )
{
   fprintf ( stderr, "horbasic -s<canvas size> <base-name>\n" );
}

static void scan_command_line ( int    argc,
			        char **argv,
			        int   *canvas_size_ptr,
			        char  *base_name )
{
   for ( ; argc > 0; argv++, argc-- )
   {
      if ( (*argv)[0] != '-' ) break;

      switch ( (*argv)[1] )
      {
         case 's':
	 hor_read_int_param ( *argv+2, "canvas size", hor_int_abs_pos,
			      HOR_FATAL, canvas_size_ptr );
	 break;

         case 'h':
	 print_horatio_args();
	 exit(0);
	 break;

         default:
	 hor_error ( "illegal argument -%c", HOR_FATAL, (*argv)[1] );
	 break;
      }
   }

   if ( argc < 1 )
   {
      print_horatio_args();
      exit(0);
   }
   
   strcpy ( base_name, *argv );
}

static Widget ops_toggle_group;
#define OPS_TOGGLE_CHOICES 2
static String ops_toggle_choices[] = {
	"Canny",
	"Corner",
};

static Hor_Image *image = NULL;

void select_operation ( int c1, int r1, int c2, int r2, void *data )
{
   int ops_mask = hor_get_togglegroup ( ops_toggle_group );

   if ( image == NULL )
   {
      hor_error ( "no image to process", HOR_NON_FATAL );
      return;
   }

   if ( ops_mask & 1 )
   {
      Hor_CA_Process_Params params;

      if ( hor_get_canny_params ( &params ) )
      {
	 float *mask = hor_make_gaussian_mask ( params.sigma,
					        params.gauss_size );
	 Hor_Edge_Map *edge_map;

	 edge_map = hor_canny ( image, mask, params.gauss_size,
			        params.low_thres, params.high_thres,
			        params.length_thres, c1, r1, c2, r2 );
	 if ( edge_map != NULL )
	 {
	    Hor_ED_Output_Params disp_params;

	    hor_get_canny_colours ( &disp_params );
	    hor_display_edge_map ( edge_map, HOR_NO_ATTRIB,
				   HOR_NO_ATTRIB, &disp_params );
	    hor_free_edge_map ( edge_map );
	 }

	 hor_free_gaussian_mask ( mask, params.gauss_size );
      }
   }

   if ( ops_mask & 2 )
   {
      Hor_PC_Process_Params params;

      if ( hor_get_plessey_corner_params ( &params ) )
      {
	 Hor_Corner_Map *corner_map;
	 float          *mask = hor_make_gaussian_mask ( params.sigma,
							 params.gauss_size );
	 corner_map = hor_plessey_corners ( image, mask, params.gauss_size,
					    params.strength_thres,
					    params.patch_size,
					    c1, r1, c2, r2 );
	 if ( corner_map != NULL )
	 {
	    Hor_CO_Output_Params disp_params;

	    hor_get_plessey_corner_colours ( &disp_params );
	    hor_display_corner_map ( corner_map, HOR_NO_ATTRIB, &disp_params );
	    hor_free_corner_map ( corner_map );
	 }

	 hor_free_gaussian_mask ( mask, params.gauss_size );
      }
   }
}

/* horbasic quit procedure */
static void quit_proc(void)
{
   hor_free_colourmap();
   XtDestroyApplicationContext(app_con);
   exit(1);
}

static void read_image_proc ( Widget button, XtPointer client_data,
			                     XtPointer call_data )
{
   if ( image != NULL ) hor_free_image ( image );

   image = hor_read_image ( base_name );
   if ( image == NULL )
      hor_error ( "cannot read image with base-name %s", HOR_NON_FATAL,
		  base_name );
   else
      if ( !hor_display_image ( image, 1, 0.0, 256.0 ) )
	 hor_error ( "display failed (read_image_proc)", HOR_FATAL );
      else /* set image region processing operation */
	 hor_region_set_function ( select_operation );
}

/* 3D tool stuff starts here */

static Hor_Assoc_Label threed_label1;

static void oneselect_proc ( Hor_Assoc_Label label, Hor_Item_3D item )
{
   switch ( item.type )
   {
      case HOR_LINE_3D:
      {
	 Hor_Matrix *p1 = item.u.line.p1, *p2 = item.u.line.p2;

	 hor_message ( "Selected line (%lf %lf %lf)-(%lf %lf %lf)", 
		       p1->m[0][0], p1->m[1][0], p1->m[2][0],
		       p2->m[0][0], p2->m[1][0], p2->m[2][0] );
      }
      break;

      case HOR_POINT_3D:
      {
	 Hor_Matrix *p = item.u.point.p;

	 hor_message ( "Selected point (%lf %lf %lf)",
		       p->m[0][0], p->m[1][0], p->m[2][0] );
      }
      break;

      case HOR_FACET_3D:
      hor_message ( "facet selected" );
      break;

      case HOR_LABEL_3D:
      break;
   }
}

static void procselected_proc ( Hor_Assoc_List item_list )
{
   int            line_count = 0, point_count = 0, facet_count = 0;
   Hor_Item_3D   *item;
   Hor_Assoc_List list;

   for ( list = item_list; list != NULL; list = list->next )
   {
      item = (Hor_Item_3D *) list->data;
      switch ( item->type )
      {
	 case HOR_LINE_3D:  line_count++;  break;
	 case HOR_POINT_3D: point_count++; break;
	 case HOR_FACET_3D: facet_count++; break;
	 case HOR_LABEL_3D: break;
      }
   }

   hor_message ( "3D selection callback received %d lines, %d points and %d facets", line_count, point_count, facet_count );
}

static void allclear_proc ( Hor_Assoc_List item_list )
{
   int            line_count = 0, point_count = 0, facet_count = 0;
   Hor_Item_3D   *item;
   Hor_Assoc_List list;

   for ( list = item_list; list != NULL; list = list->next )
   {
      item = (Hor_Item_3D *) list->data;
      switch ( item->type )
      {
	 case HOR_LINE_3D:  line_count++;  break;
         case HOR_POINT_3D: point_count++; break;
         case HOR_FACET_3D: facet_count++; break;
	 case HOR_LABEL_3D: break;
      }
   }

   hor_message ( "3D clear callback received %d lines, %d points and %d facets", line_count, point_count, facet_count );
}

static void onedelete_proc ( Hor_Assoc_Label label, Hor_Item_3D item )
{
   switch ( item.type )
   {
      case HOR_LINE_3D:
      {
	 Hor_Matrix *p1 = item.u.line.p1, *p2 = item.u.line.p2;

	 hor_message ( "Deleted line (%lf %lf %lf)-(%lf %lf %lf)", 
		       p1->m[0][0], p1->m[1][0], p1->m[2][0],
		       p2->m[0][0], p2->m[1][0], p2->m[2][0] );
      }
      break;

      case HOR_POINT_3D:
      {
	 Hor_Matrix *p = item.u.point.p;

	 hor_message ( "Deleted point (%lf %lf %lf)",
		       p->m[0][0], p->m[1][0], p->m[2][0] );
      }
      break;

      case HOR_FACET_3D:
      hor_message ( "facet deleted" );
      break;

      case HOR_LABEL_3D:
      break;
   }
}

static void allrestore_proc ( Hor_Assoc_List item_list )
{
   int            line_count = 0, point_count = 0, facet_count = 0;
   Hor_Item_3D   *item;
   Hor_Assoc_List list;

   for ( list = item_list; list != NULL; list = list->next )
   {
      item = (Hor_Item_3D *) list->data;
      switch ( item->type )
      {
	 case HOR_LINE_3D:  line_count++;  break;
         case HOR_POINT_3D: point_count++; break;
         case HOR_FACET_3D: facet_count++; break;
	 case HOR_LABEL_3D: break;
      }
   }

   hor_message ( "3D restore callback received %d lines, %d points and %d facets", line_count, point_count, facet_count );
}

static void threed_proc ( Widget button, XtPointer client_data,
			  XtPointer call_data )
{
   Hor_Item_3D     item;
   Hor_Assoc_Label label = HOR_ASSOC_START;

   if ( hor_3D_in_use ( threed_label1 ) )
   {
      hor_redisplay_3D ( threed_label1 );
      return;
   }

   hor_popup_3D ( button, threed_label1, oneselect_proc, procselected_proc,
		  allclear_proc, onedelete_proc, allrestore_proc, NULL );

   item.u.point.p = hor_mat_alloc (3, 1);
   item.type = HOR_POINT_3D;

   hor_matq_fill ( item.u.point.p, 1.0, -1.0, 1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.point.p, 1.0, 0, 0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.point.p, 1.0, 1.0, 1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_mat_free ( item.u.point.p );
   item.type = HOR_LINE_3D;

   item.u.line.p1 = hor_mat_alloc (3, 1);
   item.u.line.p2 = hor_mat_alloc (3, 1);

   hor_matq_fill ( item.u.line.p1, 1.0, 1.0, -1.0);
   hor_matq_fill ( item.u.line.p2, -1.0, 1.0, -1.0);
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, -1.0, -1.0, -1.0 );
   hor_matq_fill ( item.u.line.p2, -1.0, 1.0, -1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, -1.0, -1.0, -1.0 );
   hor_matq_fill ( item.u.line.p2, 1.0, -1.0, -1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, 1.0, 1.0, -1.0 );
   hor_matq_fill ( item.u.line.p2, 1.0, -1.0, -1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, -1.0, -1.0, -1.0 );
   hor_matq_fill ( item.u.line.p2, -1.0, -1.0, 1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, -1.0, 1.0, 1.0 );
   hor_matq_fill ( item.u.line.p2, -1.0, -1.0, 1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_matq_fill ( item.u.line.p1, -1.0, 1.0, 1.0 );
   hor_matq_fill ( item.u.line.p2, -1.0, 1.0, -1.0 );
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_mat_free ( item.u.line.p2 );
   hor_mat_free ( item.u.line.p1 );

   item.type = HOR_FACET_3D;

   item.u.facet.p1 = hor_mat_alloc (3, 1);
   item.u.facet.p2 = hor_mat_alloc (3, 1);
   item.u.facet.p3 = hor_mat_alloc (3, 1);
   item.u.facet.image = hor_alloc_image ( 256, 256, HOR_U_CHAR, NULL );

   {
      int i, j;

      for ( i = 0; i < 256; i++ )
	 for ( j = 0; j < 256; j++ )
	    item.u.facet.image->array.uc[i][j] = j;
   }

   hor_matq_fill ( item.u.facet.p1, 0.0, 0.0, 0.0 );
   hor_matq_fill ( item.u.facet.p2, 1.0, 0.0, 0.0 );
   hor_matq_fill ( item.u.facet.p3, 0.0, 1.0, 0.0 );
   item.u.facet.c1 =   0; item.u.facet.r1 =   0;
   item.u.facet.c2 = 255; item.u.facet.r2 =   0;
   item.u.facet.c3 =   0; item.u.facet.r3 = 255;
   hor_3D_item ( threed_label1, label++, item, LightSeaGreen );

   hor_free_image ( item.u.facet.image );
   hor_mat_free ( item.u.facet.p3 );
   hor_mat_free ( item.u.facet.p2 );
   hor_mat_free ( item.u.facet.p1 );

   hor_init_3D ( threed_label1, HOR_TRUE );
}

/* 3D tool stuff ends here */

/* graph stuff starts here */

static Hor_Assoc_Label graph_label1, graph_label2;
static Hor_Assoc_Label trace_label1, trace_label2;

static void graph_proc ( Widget button, XtPointer client_data,
			 XtPointer call_data )
{
   if ( hor_graph_in_use ( graph_label1 ) )
      hor_redisplay_graph ( graph_label1 );
   else
   {
      hor_popup_graph ( button, graph_label1, 100.0F, -5.0F, 5.0F,
		        &trace_label1, NULL );
      hor_popup_graph ( button, graph_label2, 10.0F, 0.0F, 20.0F,
		        &trace_label2, NULL );
   }
}

static void graph_point_proc ( Widget button, XtPointer client_data,
			                      XtPointer call_data )
{
   static float t = 0.0;

   if ( !hor_graph_in_use ( graph_label1 ) ) return;

   hor_graph_point(graph_label1, trace_label1, t, (float) (3.0*sin(t)), Red );
   hor_graph_point(graph_label2, trace_label2, t, (float) (15.0*cos(t)*cos(t)),
		   Yellow);
   t += 1.0F;
}

static void graph_quit_proc ( Widget button, XtPointer client_data,
			                     XtPointer call_data )
{
   hor_popdown_graph ( graph_label1 );
}

/* graph stuff ends here */

static Widget ops_panel ( Widget parent, ... )
{
   int     num_args;
   Arg    *args;
   va_list ap;
   Hor_Popup_Data *popup_data = hor_malloc_type(Hor_Popup_Data);
   Widget button, frame, panel, last;

   /* count number of variable arguments */
   va_start ( ap, parent );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   button = XtCreateManagedWidget ( "Parameters", commandWidgetClass,
				    parent, args, num_args );
   hor_free ( (void *) args );
   frame = XtVaCreatePopupShell ( "Parameters", transientShellWidgetClass,
				   button, NULL );
   panel = XtVaCreateManagedWidget ( "Parameters", formWidgetClass,
				     frame, NULL );

   popup_data->popup_frame = frame;
   popup_data->x = 35;
   popup_data->y = 110;
   XtAddCallback (button, XtNcallback, hor_show_popup, (XtPointer) popup_data);

   last = XtVaCreateManagedWidget ( "Parameters", labelWidgetClass,
				    panel, NULL );
   /* create parameter pop-up buttons */
   last = hor_create_canny_popup          ( "Canny", panel,
					    XtNfromVert, last, NULL );
   last = hor_create_plessey_corner_popup ( "Corner", panel,
					    XtNfromVert, last, NULL );

   hor_create_done_button ( panel, last );
   return button;
}

static void fill_button_panel ( Widget panel )
{
   Widget button, button2;

   button = XtVaCreateManagedWidget ( "Quit", commandWidgetClass, panel, NULL);
   XtAddCallback ( button, XtNcallback, (XtCallbackProc) quit_proc, NULL );

   /* create button to read and display image */
   button = XtVaCreateManagedWidget ( "Read Image", commandWidgetClass, panel,
				      XtNfromVert, button, NULL );
   XtAddCallback ( button, XtNcallback, read_image_proc, NULL );

   /* create operation popup toggle list widget */
   button2 = hor_create_togglegroup_widget ( "Operations", panel,
					     ops_toggle_choices,
					     OPS_TOGGLE_CHOICES,
					     15, 110, NULL, NULL, NULL,
					     XtNfromVert, button, NULL );
   ops_toggle_group = button2;

   /* create parameter popups */
   button = ops_panel ( panel, XtNfromVert,  button,
		               XtNfromHoriz, button2, NULL );

   /* more 3D tool stuff starts here */
   button = XtVaCreateManagedWidget ( "3D Tool", commandWidgetClass, panel,
				      XtNfromVert, button2,
				      NULL );
   XtAddCallback ( button, XtNcallback, threed_proc, NULL );
   threed_label1 = hor_create_3D ( button, 300, Hor_Grey[0], Red );
   /* 3D tool stuff ends here */

   /* more graph stuff starts here */
   button = XtVaCreateManagedWidget ( "Graph Tool", commandWidgetClass, panel,
				      XtNfromVert, button,
				      NULL );
   XtAddCallback ( button, XtNcallback, graph_proc, NULL );
   graph_label1 = hor_create_graph ( button, HOR_ASSOC_ERROR, HOR_ASSOC_ERROR,
				     300, 200, Hor_Grey[0] );
   graph_label2 = hor_create_graph ( button, HOR_ASSOC_ERROR, graph_label1,
				     300, 200, Hor_Grey[0] );

   button = XtVaCreateManagedWidget ( "Graph Point", commandWidgetClass,panel,
				      XtNfromVert, button,
				      NULL );
   XtAddCallback ( button, XtNcallback, graph_point_proc, NULL );

   button = XtVaCreateManagedWidget ( "Graph Quit", commandWidgetClass, panel,
				      XtNfromVert, button,
				      NULL );
   XtAddCallback ( button, XtNcallback, graph_quit_proc, NULL );

   /* graph stuff ends here */
}

static void fill_complex_panel ( Widget panel )
{
   Widget button_panel;

   /* create button panel */
   button_panel = XtVaCreateManagedWidget ( "Button Panel", formWidgetClass,
					    panel, NULL );
   fill_button_panel ( button_panel );
}

static Widget          canvas;
static Hor_Assoc_Label canvas_label;
static Hor_Assoc_Label string_label;

static void create_canvases ( Widget panel, int canvas_size )
{
   canvas = XtVaCreateManagedWidget ( "canvas", canvasWidgetClass, panel,
				      XtNheight, canvas_size,
				      XtNwidth,  canvas_size, NULL );

   /* create button function window below canvases */
   string_label = hor_display_set_string ( panel, XtNfromVert, canvas, NULL );
}

static void canvas_register ( Display *display )
{
   Window                canvas_window = XtWindow ( canvas );
   XSetWindowAttributes  sw;

   canvas_label = hor_display_set_window ( canvas_window, canvas,
					   XC_cross_reverse, string_label );

   /* set other window attributes */
   sw.backing_store = Always ;
   XChangeWindowAttributes ( display, canvas_window, CWBackingStore, &sw );

   hor_display_reset_window ( canvas_label );

   /* set mouse callback functions */
   hor_display_set_mouse_functions ( canvas_label,
				hor_region_start, hor_region_finish, "Region",
                                hor_region_cancel, NULL,             "Cancel",
				hor_region_select, NULL,             "Select",
                                hor_region_moving, NULL, NULL, NULL );
}

static XGCValues gc; /* made global static so that fields will be
			initialised to zero */

void main ( int argc, char **argv )
{
   Widget   frame, box_parent, text_form, rest_form, canvas_form;
   Display *display;
/*malloc_debug(2);*/

   scan_command_line ( argc-1, argv+1, &canvas_size, base_name );

   /* initialise X application */
   frame = XtAppInitialize(&app_con, "horatio", NULL, ZERO,
			   &argc, argv, fallback_resources, NULL, ZERO);
   box_parent = XtVaCreateManagedWidget ( "Panel", formWidgetClass, frame,
					  NULL );
   display = XtDisplay ( box_parent );

   /* set function to call on Horatio fatal error */
   hor_set_fatal_error_function ( quit_proc );

   /* define panel containing everything except the text window */
   rest_form = XtVaCreateManagedWidget ( "rest box", formWidgetClass,
					 box_parent, NULL );

   /* set up colour map */
   hor_colourmap_setup ( display, 2, 6,
		     "Red",       &Red,       "Green",         &Green,
		     "Blue",      &Blue,      "Yellow",        &Yellow,
		     "SteelBlue", &SteelBlue, "LightSeaGreen", &LightSeaGreen,
		     "thistle",   &thistle,   "Cyan",          &Cyan,
		     NULL );

   /* create and fill graphics canvas */
   canvas_form = XtVaCreateManagedWidget ( "canvas box", formWidgetClass,
					   rest_form, NULL );
   create_canvases ( canvas_form, canvas_size );

   /* create and fill panel with operations, parameters and commands */
   fill_complex_panel ( XtVaCreateManagedWidget ( "Complicated",
						  formWidgetClass, rest_form,
						  XtNfromHoriz, canvas_form,
						  NULL ) );

   /* create text window below graphics panel */
   text_form = XtVaCreateManagedWidget("text box", formWidgetClass, box_parent,
				       XtNfromVert, rest_form, NULL);

   hor_set_text_window ( XtVaCreateManagedWidget (
				"text", asciiTextWidgetClass, text_form,
				XtNwidth, canvas_size + 200,
				NULL ) );

   /* realise top level widget */
   XtRealizeWidget(frame);

   hor_display_initialise ( display, XCreateGC(display, XtWindow(canvas),
					       0, &gc),
			    SteelBlue, Green, Red, LightSeaGreen, thistle );
   canvas_register     ( display );
   hor_register_3D     ( display );
   hor_register_graphs ( display );

   /* register I/O error handler function */
   XSetIOErrorHandler ( (int (*) (Display *)) quit_proc );

   /* set colours for vision processes */
   hor_set_canny_colours ( Red, Green, Blue, Yellow );
   hor_set_plessey_corner_colours ( Cyan );

   /* Pass control to notifier */
   XtAppMainLoop(app_con);
}
