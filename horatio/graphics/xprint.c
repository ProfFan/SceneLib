/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* xprint.c: supports multiple X text windows. */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/cursorfo.h>
#include <X11/StringDe.h>

#include <X11/Xaw/Cardinal.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiTx.h>
#else
#include <X11/Intrinsic.h>
#include <X11/cursorfont.h>
#include <X11/StringDefs.h>

#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/AsciiText.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

typedef struct
{
   Widget          text_window;
   Display        *display;
   XawTextPosition start;
} X_Text_Window;

/* list of registered text windows */
static Hor_Assoc_List x_text_window_list = NULL;

/* label of next new text window */
static Hor_Assoc_Label next_text_window_label = HOR_ASSOC_START;

/* window that text is currently printed in */
static X_Text_Window  *current_text_window;
static Hor_Assoc_Label current_text_window_label = HOR_ASSOC_ERROR;

/*******************
*   static void @print_text ( const char *s )
*
*   Prints text in current text window.
********************/
static void print_text ( const char *s )
{

   if ( current_text_window_label == HOR_ASSOC_ERROR )
      hor_error ( "text window not set (print_text)", HOR_FATAL );
   else
   {
      int          length = strlen(s);
      XawTextBlock block;

      if ( length == 0 ) return;

      block.firstPos = 0;
      block.length   = length;
      block.ptr      = (char *) s;
      block.format   = FMT8BIT;

      XawTextReplace ( current_text_window->text_window,
		       current_text_window->start,
		       current_text_window->start + length, &block );
      current_text_window->start += length;
      XawTextSetInsertionPoint ( current_text_window->text_window,
				 current_text_window->start-1 );
      XFlush ( current_text_window->display );
   }
}

/*******************
*   Hor_Assoc_Label @hor_set_text_window ( Widget wid )
*   void @hor_reset_text_window ( Hor_Assoc_Label text_window_label )
*   void @hor_delete_text_window ( Hor_Assoc_Label text_window_label )
*
*   hor_set_text_window() registers a new text window, sets the current text
*   window to it and return label which can be used to reset the current text
*   window to it at a later stage using hor_reset_text_window().
*
*   hor_reset_text_window() sets the current text window to the registered
*   window with the specified label.
*
*   hor_delete_text_window() removes a text window from the registered window
*   list.
********************/
Hor_Assoc_Label hor_set_text_window ( Widget wid )
{
   X_Text_Window *new_text_window;

   new_text_window = hor_malloc_type ( X_Text_Window );
   new_text_window->text_window = wid;
   new_text_window->display     = XtDisplay ( wid );
   new_text_window->start       = 0;
   XawTextDisplayCaret ( wid, False );
   x_text_window_list = hor_assoc_insert ( x_text_window_list,
					   next_text_window_label,
					   new_text_window );
   current_text_window       = new_text_window;
   current_text_window_label = next_text_window_label;
   hor_set_print_func ( print_text );
   return ( next_text_window_label++ );
}

void hor_reset_text_window ( Hor_Assoc_Label text_window_label )
{
   void *hor_assoc_data;

   /* find window in registered window list */
   if ( (hor_assoc_data = hor_assoc_find ( x_text_window_list,
					   text_window_label ))
        == NULL )
      hor_error ( "invalid text window label (hor_reset_text_window)", HOR_FATAL );
   else
   {
      current_text_window       = (X_Text_Window *) hor_assoc_data;
      current_text_window_label = text_window_label;
   }
}

void hor_delete_text_window ( Hor_Assoc_Label text_window_label )
{
   if ( hor_assoc_remove ( &x_text_window_list, text_window_label,
			   hor_free_func ) == HOR_ASSOC_ERROR )
      hor_error ( "invalid text window label (hor_delete_text_window)", HOR_FATAL);

   if ( text_window_label == current_text_window_label )
   {
      hor_warning ("deleted current text window (hor_display_delete_window)");
      current_text_window_label = HOR_ASSOC_ERROR;
   }
}

