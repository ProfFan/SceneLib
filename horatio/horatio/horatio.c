#include <stdio.h>

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#include <X11/Intrinsic.h>
#include <X11/cursorfont.h>
#include <X11/StringDefs.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiText.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

#include "select.h"
#include "canvas.h"
#include "buttons.h"

static XtAppContext app_con;

/* colour identifiers */
u_long Red, Green, Blue, Yellow, Cyan, SteelBlue, LightSeaGreen, thistle;

static char    base_name[80] = "agv";
static int     canvas_size   = 512;
static int     no_canvases   = 1;

static String fallback_resources[] = {
   "*input: True",
   "*text*editType:          append",
   "*text*scrollVertical:    Always",
   "*text*height:            100",
   NULL,
 };

static void print_horatio_args ( void )
{
   fprintf ( stderr,
	     "horatio -b<base name> -s<canvas size> -n<no. of canvases>\n" );
}

static void scan_command_line ( int    argc,
			        char **argv,
			        char  *base_name,
			        int   *canvas_size_ptr,
			        int   *no_canvases_ptr )
{
   for ( ; argc > 0; argv++, argc-- )
   {
      if ( (*argv)[0] != '-' ) break;

      switch ( (*argv)[1] )
      {
         case 'b':
	 sscanf ( *argv+2, "%s", base_name );
	 break;

         case 's':
	 hor_read_int_param ( *argv+2, "canvas size", hor_int_abs_pos,
			      HOR_FATAL, canvas_size_ptr );
	 break;

         case 'n':
	 hor_read_int_param ( *argv+2, "number of canvases", hor_int_abs_pos,
			      HOR_FATAL, no_canvases_ptr );
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
}

static int smallest_geq_square ( int number )
{
   int i;

   if ( number < 0 )
      hor_error ( "input number must be >=0 (smallest_geq_square)", HOR_FATAL);

   /* calculate squares from 0 upwards until number is exceeded */
   for ( i = 0; i*i < number; i++ ) ;

   return ( i );
}

static void quit_proc(void)
{
   hor_free_colourmap();
   XtDestroyApplicationContext(app_con);
   exit(1);
}

void main ( int argc, char **argv )
{
   Widget   frame, box_parent;
   Widget   text_form, rest_form;
   Widget   canvas_form;
   Display *display;
   int      canvases_to_a_side;
/*malloc_debug(2);*/

   scan_command_line ( argc-1, argv+1, base_name, &canvas_size, &no_canvases );

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
   hor_colourmap_setup(display, 2, 6,
		       "Red",           &Red,          "Green",     &Green,
		       "Blue",          &Blue,         "Yellow",    &Yellow,
		       "Cyan",          &Cyan,         "SteelBlue", &SteelBlue,
		       "LightSeaGreen", &LightSeaGreen,"thistle",   &thistle,
		       NULL );

   /* create and fill graphics canvas */
   canvas_form = XtVaCreateManagedWidget ( "canvas box", formWidgetClass,
					   rest_form, NULL );
   canvases_to_a_side = smallest_geq_square ( no_canvases );
   fill_canvas_panel ( canvas_form, canvas_size,
		       no_canvases, canvases_to_a_side );

   /* create and fill panel with operations, parameters and commands */
   fill_complicated_panel ( app_con,
			    XtVaCreateManagedWidget ( "Complicated",
						 formWidgetClass, rest_form,
					         XtNfromHoriz, canvas_form,
						 NULL ), base_name,
			    (XtCallbackProc) quit_proc );

   /* create text window below graphics panel */
   text_form = XtVaCreateManagedWidget("text box", formWidgetClass, box_parent,
				       XtNfromVert, rest_form, NULL);

   hor_set_text_window ( XtVaCreateManagedWidget (
			       "text", asciiTextWidgetClass, text_form,
			       XtNwidth, canvases_to_a_side*canvas_size + 200,
			       NULL) );

   /* realise top level widget */
   XtRealizeWidget(frame);

   register_canvases ( display,
		       SteelBlue, Green, Red, LightSeaGreen, thistle );

   /* register I/O error handler function */
   XSetIOErrorHandler ( (int (*) (Display *)) quit_proc );

   /* set colours for vision processes */
   hor_set_canny_colours ( Red, Green, Blue, Yellow );
   hor_set_plessey_corner_colours ( Cyan );
   hor_set_smith_corner_colours ( Green );
   hor_set_wang_corner_colours ( Red );
   hor_set_bog_corner_match_colours ( LightSeaGreen, Blue, SteelBlue );
   hor_set_bd_corner_match_colours ( LightSeaGreen, Blue, SteelBlue );
   hor_set_line_fit_colours ( Cyan );
   hor_set_line_segment_colours ( SteelBlue );
   hor_set_bog_line_match_colours ( Blue, thistle, LightSeaGreen );
   hor_set_image_flow_colours ( Blue, Cyan );
   hor_set_image_segment_colours ( Green );
   hor_set_correlation_colours ( Green );

   hor_init_process_stuff();

   /* Pass control to notifier */
   XtAppMainLoop(app_con);
}
