/* STATE.C - Copyright Jason Merron, March 1994
 *
 * Functions for playing sequences of captured images in parallel (hereafter
 * referred to as state movies or multi-canvas movies).
 *
 * A state movie consists of a set of state frames (stored in a Hor_List).  
 * Each state frame is a list of association labels, where each label 
 * refers to a particular image captured using "hor_display_store_state".
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

/*******************
*   void @hor_recall_states ( Hor_List states )
*
*   Displays a state frame (a list of captured images, each stored using 
*   "hor_display_store_state").
*   The argument is a Hor_List of Hor_Assoc_Label's, each of which refers 
*   to a particular image in a particular canvas (so canvas selection is
*   automatic).
*
*   What's it for?  This function can be called directly from
*   "hor_popup_memory_panel" to display a frame from a multi-canvas movie.
********************/
void hor_recall_states (Hor_List states)
{
   while (hor_list_non_null (states))
   {
      hor_display_recall_state ((Hor_Assoc_Label) hor_node_contents (states));
      states = hor_next_node (states);
   }
}

/*******************
*   void @hor_destroy_states ( Hor_List states )
*
*   Destroys a state frame.
*   Use this function for freeing a single frame from a multi-canvas movie.
********************/
void hor_destroy_states ( Hor_List states )
{
   hor_free_list (states, (void *) hor_display_destroy_state);
}

/*******************
*   void @hor_destroy_states_movie ( Hor_List movie )
*
*   Destroys a state movie (a list of state frames).  
*   Use this function for freeing a multi-canvas movie.
********************/
void hor_destroy_states_movie ( Hor_List movie )
{
   hor_free_list (movie, (void *) hor_destroy_states);
}

