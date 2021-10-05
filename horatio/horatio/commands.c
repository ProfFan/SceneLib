/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Command.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

#include "buttons.h"
#include "select.h"
#include "canvas.h"

/*******************
*   static void @init_proc ( Widget button,
*                           XtPointer client_data, XtPointer call_data )
*
*   Clears process list, starts new list with grab "process" as only member,
*   executes the process once to read first image, clears the selected image
*   region list, sets the selected output image format, sets the mouse
*   event functions to the image region selection functions (from get_region.c)
*   and sets the region selection function to select_improc_in_region()
*   which creates the new selected processes.
********************/
static void init_proc ( Widget button,
		        XtPointer client_data, XtPointer call_data )
{
   if ( hor_get_gb_process_type_label() == HOR_ASSOC_ERROR )
      hor_error ( "grab process type undefined (init_proc)", HOR_FATAL );

   hor_clear_processes();
   hor_gb_add_process ( get_base_name() );
   hor_execute_processes();

   /* discard any pending image ROIs */
   hor_region_clear();

   hor_set_image_format ( get_image_format() );

   /* set image region processing operation */
   hor_region_set_function ( select_improc_in_region );
}

/*******************
*   static void @step_proc ( Widget button, XtPointer client_data,
*                           XtPointer call_data )
*
*   Deletes the last selected image region, if any, clears all other selected
*   regions and executes one step of each process.
********************/
static void step_proc ( Widget button,
		        XtPointer client_data, XtPointer call_data )
{
   hor_region_delete_selected();
   hor_region_clear();
   hor_execute_processes();
}

static Hor_Assoc_Label memory_panel = HOR_ASSOC_ERROR;

static void movie_proc ( Widget button,
			 XtPointer client_data, XtPointer call_data )
{
   Hor_List movie;

   if ( hor_memory_panel_in_use ( memory_panel ) )
   {
      hor_redisplay_memory_panel ( memory_panel );
      return;
   }

   movie = hor_display_make_movie ( get_base_name(), 0.0, 256.0 );

   if ( movie == NULL ) return;

   hor_popup_memory_panel ( button, memory_panel, movie,
			    (void (*)(void *)) hor_display_show_movie_image,
			    NULL, NULL, NULL, hor_display_destroy_movie, 0.0 );
}

static void read_result_image_proc ( Widget    button,
				     XtPointer client_data,
				     XtPointer call_data )
{
   if ( hor_display_read_from_file ( get_base_name() ) )
      hor_message ( "result image read and displayed" );
   else hor_perror ( "(read_result_image_proc)" );
}

static void done_proc ( Widget button,
		        XtPointer client_data, XtPointer call_data )
{
   hor_popdown_memory_panel ( memory_panel );
}

static void window_proc ( Widget button,
			  XtPointer client_data, XtPointer call_data )
{
   change_canvas();
}

/*******************
*   Widget @fill_command_panel ( XtAppContext app_con, Widget parent,
*                               XtCallbackProc quit_proc, ... )
*
*   Returns a panel filled with horatio command buttons.
********************/
Widget fill_command_panel ( XtAppContext app_con, Widget parent,
			    XtCallbackProc quit_proc, ... )
{
   Widget  init_button, button, panel;
   int     num_args;
   Arg    *args;
   va_list ap;

   /* count number of variable arguments */
   va_start ( ap, quit_proc );
   num_args = hor_convert_X_args ( &ap, &args );
   va_end(ap);

   panel = XtCreateManagedWidget ( "Command Panel", formWidgetClass, parent,
				   args, num_args );
   hor_free ( (void *) args );

   /* create canvas init button */
   init_button = XtVaCreateManagedWidget ( "Init", commandWidgetClass,
					   panel, NULL );
   XtAddCallback ( init_button, XtNcallback, init_proc, NULL );

   /* create button to perform image processing operations on next frame */
   button = XtVaCreateManagedWidget ( "Step", commandWidgetClass,
				      panel, XtNfromHoriz, init_button, NULL );
   XtAddCallback ( button, XtNcallback, step_proc, NULL );

   /* create button to show movie (all images in sequence in turn) */
   button = XtVaCreateManagedWidget ( "Movie", commandWidgetClass,
				      panel, XtNfromHoriz, button, NULL );
   memory_panel = hor_create_memory_panel ( app_con, button );
   XtAddCallback ( button, XtNcallback, movie_proc, NULL );

   button = XtVaCreateManagedWidget ( "Done", commandWidgetClass,
				      panel, XtNfromHoriz, button, NULL );
   XtAddCallback ( button, XtNcallback, done_proc, NULL );

   /* create button to read and display previously stored result image */
   button = XtVaCreateManagedWidget ( "Read", commandWidgetClass,
				      panel, XtNfromHoriz, button, NULL );
   XtAddCallback ( button, XtNcallback, read_result_image_proc, NULL );

   /* create button to change graphics window */
   button = XtVaCreateManagedWidget ( "Window", commandWidgetClass,
				      panel, XtNfromVert, init_button, NULL );
   XtAddCallback ( button, XtNcallback, (XtCallbackProc) window_proc, NULL );

   /* create quit button */
   button = XtVaCreateManagedWidget ( "Quit", commandWidgetClass, panel,
				      XtNfromHoriz, button,
                                      XtNfromVert, init_button,
                                      NULL );
   XtAddCallback ( button, XtNcallback, quit_proc, NULL );

   return panel;
}
