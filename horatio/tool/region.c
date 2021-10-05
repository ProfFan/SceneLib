/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/tool.h"

static Hor_Bool start_fixed = HOR_FALSE;
static Hor_Bool end_fixed = HOR_FALSE;
static int start_c, region_width;
static int start_r, region_height;

static void (*region_function)(int,int,int,int,void*);

static Hor_List selected_region_list = NULL;

typedef struct canvas_region
{
   int c0, r0;
   int width, height;
} Canvas_Region;

static void add_to_selected_region_list ( int c0,    int r0,
					  int width, int height )
{
   Canvas_Region *region;

   region = hor_malloc_type ( Canvas_Region );
   region->c0     = c0;
   region->r0     = r0;
   region->width  = width;
   region->height = height;
   selected_region_list = hor_insert ( selected_region_list, region );
}

static void delete_selected_region ( void *ptr, void *data )
{
   Canvas_Region *region = (Canvas_Region *) ptr;

   hor_display_draw_canvas_rectangle  ( region->c0,    region->r0,
				        region->width, region->height );
}

/*******************
*   void @hor_region_delete_last_selected(void)
*   void @hor_region_delete_selected(void)
*
*   hor_region_delete_last_selected() deletes the last image region selected.
*   hor_region_delete_selected()      deletes the whole list of selected
*                                     regions.
********************/
void hor_region_delete_last_selected(void)
{
   Hor_List       temp_node;
   Canvas_Region *region;

   if ( selected_region_list == NULL )
      hor_error ("selected region list NULL (hor_region_delete_last_selected)",
		 HOR_FATAL );

   region = (Canvas_Region *) hor_node_contents(selected_region_list);
   delete_selected_region ( region, NULL );
   hor_free ( (void *) region );
   temp_node = selected_region_list->next;
   hor_free ( (void *) selected_region_list );
   selected_region_list = temp_node;
}

void hor_region_delete_selected(void)
{
   hor_display_set_function ( HOR_DISPLAY_XOR );
   hor_display_set_colour ( hor_display_get_xor_colour() );

   hor_list_action ( selected_region_list, delete_selected_region, NULL );
   hor_free_list   ( selected_region_list, hor_free_func );
   selected_region_list = NULL;
}

/*******************
*   void @hor_region_set_function ( void (*func)(int c1, int r1,
*                                               int c2, int r2, void *data) )
*
*   Sets the function to be called when an image region is selected.
********************/
void hor_region_set_function ( void (*func)(int c1, int r1, int c2, int r2,
					    void *data) )
{
   region_function = func;
}

static void update_rectangle ( int c, int r )
{
   hor_display_set_function ( HOR_DISPLAY_XOR );
   hor_display_set_colour ( hor_display_get_xor_colour() );

   /* delete rectangle at old position */
   hor_display_draw_canvas_rectangle ( start_c, start_r,
				       region_width, region_height );

   /* draw new rectangle */
   region_width  = c - start_c;
   region_height = r - start_r;
   hor_display_draw_canvas_rectangle ( start_c, start_r,
				       region_width, region_height );
}

static void delete_region(void)
{
   hor_display_set_function ( HOR_DISPLAY_XOR );
   hor_display_set_colour ( hor_display_get_xor_colour() );

   hor_display_draw_canvas_rectangle  ( start_c, start_r,
				        region_width, region_height );
}

/*******************
*   void hor_region_start  ( int c, int r, void *data ) (left button down)
*   void hor_region_moving ( int c, int r, void *data ) (button motion)
*   void hor_region_finish ( int c, int r, void *data ) (left button up)
*   void hor_region_cancel ( int c, int r, void *data ) (middle button down)
*   void hor_region_select ( int c, int r, void *data ) (right button down)
*
*   Functions called during image region selection. hor_fill_command_panel()
*   sets the mouse event handler to call these functions on mouse events.
*   c and r are in canvas coordinates and specify the mouse position.
*   hor_region_select() adds a new region to the selected region list.
********************/
void hor_region_start ( int c, int r, void *data )
{
   if ( !hor_display_within_image ( c, r ) ) return;

   if ( start_fixed && !end_fixed ) /* must have exited clip region, released
				       button and reentered. Treat as
				       if button just released */
   {
      end_fixed = HOR_TRUE;
      return;
   }

   if ( end_fixed ) /* discard old region */
      delete_region();

   start_c = c;
   start_r = r;
   region_width = region_height = 0;
   start_fixed = HOR_TRUE;
   end_fixed = HOR_FALSE;
}

void hor_region_moving ( int c, int r, void *data )
{
   if ( !hor_display_within_image ( c, r ) || !start_fixed || end_fixed )
      return;

   update_rectangle ( c, r );
}

void hor_region_finish ( int c, int r, void *data )
{
   if ( !hor_display_within_image ( c, r ) ) return;
   if ( !start_fixed ) return;

   update_rectangle ( c, r );
   end_fixed = HOR_TRUE;
}

void hor_region_cancel ( int c, int r, void *data )
{
   if ( !start_fixed ) return;

   delete_region();
   start_fixed = HOR_FALSE;
   end_fixed = HOR_FALSE;
}

void hor_region_select ( int c, int r, void *data )
{
   int image_c1, image_r1, image_c2, image_r2;

   if ( !end_fixed ) return;

   hor_display_set_function ( HOR_DISPLAY_XOR );
   hor_display_set_colour ( hor_display_get_xor_colour() );

   /* delete rectangle at old position */
   hor_display_draw_canvas_rectangle ( start_c, start_r,
				   region_width, region_height );

   hor_display_region_convert (start_c, start_r,
			       start_c + region_width, start_r + region_height,
			       &image_c1, &image_r1, &image_c2, &image_r2);
   add_to_selected_region_list (start_c, start_r, region_width, region_height);
   if ( region_function == NULL )
      hor_warning ( "no region selection action set" );
   else
      region_function ( image_c1, image_r1, image_c2, image_r2, data );

   start_fixed = HOR_FALSE;
   end_fixed = HOR_FALSE;
}

/*******************
*   void hor_region_clear(void)
*
*   Aborts region selection and clears selected region list.
********************/
void hor_region_clear(void)
{
   hor_free_list ( selected_region_list, hor_free_func );
   selected_region_list = NULL;
   start_fixed = end_fixed = HOR_FALSE;
}
