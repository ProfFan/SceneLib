/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* colourmap.c: functions for setting up the colourmap */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#include <X11/StringDe.h>
#else
#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

#define IMAGE_DISPLAY_FORMAT HOR_U_SHORT

u_long *Hor_Grey;           /* array of grey level associations between
			       grey levels implicit in image format and: */

Hor_X_to_Image_ColourMap x_to_image_colourmap;
u_long                  *image_to_x_colourmap;
Hor_Bool                 x_to_image_init = HOR_FALSE;
Colormap                 cmap;

static Display *cmap_display = NULL;
static u_long   all_colours[300];
static int      no_colours = 0;

/* stuff for dealing with swapped bytes */
static const int endian = 0x1;
static int cpu_byte_order;
u_long *image_grey_values;


/*******************
*   void @hor_free_colourmap(void)
*
*   Frees Horatio colourmap.
********************/
void hor_free_colourmap(void)
{
   if ( cmap_display == NULL ) return;

   if ( no_colours > 0 )
      XFreeColors ( cmap_display, cmap, all_colours, no_colours, 0 );

   no_colours = 0;
}

static Hor_Bool alloc_colour ( Display                 *display,
			       Colormap                 cmap,
			       long                    *top_grey_level_ptr,
			       Hor_X_to_Image_ColourMap x_to_image_colourmap,
			       u_long                  *image_to_x_colourmap,
			       const char              *colour_name,
			       u_long                  *colour_ptr )
{
   XColor screen_col, exact_col;

   if (!XAllocNamedColor (display, cmap, colour_name, &screen_col, &exact_col))
   {
      hor_free_colourmap();
      return HOR_FALSE;
   }

   *colour_ptr = all_colours[no_colours++] = screen_col.pixel;
   switch ( hor_get_image_display_format() )
   {
      case HOR_U_CHAR:
      x_to_image_colourmap.uc[*colour_ptr] = *top_grey_level_ptr;
      break;

      case HOR_U_SHORT:
      x_to_image_colourmap.us[*colour_ptr] = *top_grey_level_ptr;
      break;

      default:
      hor_error ( "illegal format (alloc_colours)", HOR_FATAL );
      break;
   }

   image_to_x_colourmap[*top_grey_level_ptr] = *colour_ptr;
   (*top_grey_level_ptr)--;
   return HOR_TRUE;
}

typedef struct
{
   const char *name;
   u_long colour;
} Colour_Name;

static int          no_named_colours;
static Colour_Name *colour_name;

static int count_colours ( va_list *aptr )
{
   int count = 0;

   for(;;)
      if ( va_arg ( *aptr, const char * ) == NULL ) return count;
      else count++;
}

static Hor_Bool alloc_colours ( Display *dis, Colormap cmap, long *top_gl,
			        Hor_X_to_Image_ColourMap x_to_image_colourmap,
			        u_long                  *image_to_x_colourmap,
			        va_list *aptr )
{
   const char *colour_string;
   u_long     *colour_ptr;
   int         count = 0;

   if ( no_named_colours > 0 )
      colour_name = hor_malloc_ntype ( Colour_Name, no_named_colours );

   for(;;)
   {
      colour_string = va_arg ( *aptr, const char * );
      if ( colour_string == NULL ) return HOR_TRUE;

      colour_ptr = va_arg ( *aptr, u_long * );
      if ( !alloc_colour ( dis, cmap, top_gl, x_to_image_colourmap,
			   image_to_x_colourmap, colour_string, colour_ptr ) )
	 return HOR_FALSE;

#ifdef HOR_MSDOS
      colour_name[count].name = _strdup ( colour_string );
#else
      colour_name[count].name = strdup ( colour_string );
#endif
      colour_name[count].colour = *colour_ptr;
      count++;
   }
}

static u_long *alloc_grey_levels ( Display *display,
				   Colormap cmap,
				   int      try_grey_levels,
				   int     *actual_grey_levels )
{
   u_int   grey_level;
   u_long  xlevel, grey_step = 16777216/(try_grey_levels-1);
   XColor  xcolor;
   u_long *grey_screen;

   /* allocate array of grey level pixel values */
   grey_screen = hor_malloc_ntype ( u_long, try_grey_levels );

   /* set the grey levels, starting with black */
   for ( grey_level = 0, xlevel = 0; grey_level < try_grey_levels-1;
	 grey_level++, xlevel += grey_step )
   {
      xcolor.red   = xlevel;
      xcolor.green = xlevel;
      xcolor.blue  = xlevel;
      xcolor.flags = DoRed | DoGreen | DoBlue ;
      if ( !XAllocColor ( display, cmap, &xcolor ) )  /* run out of room */
      {
	 *actual_grey_levels = grey_level;
	 XFreeColors ( display, cmap, grey_screen, *actual_grey_levels, 0 );
	 hor_free ( (void *) grey_screen );
	 no_colours -= *actual_grey_levels;
	 return NULL;
      }

      grey_screen[grey_level] = all_colours[no_colours++] = xcolor.pixel;
   }

   /* now set the white pixel value */
   xcolor.red   = 65535 ;
   xcolor.green = 65535 ;
   xcolor.blue  = 65535 ;
   xcolor.flags = DoRed | DoGreen | DoBlue ;
   if ( !XAllocColor ( display, cmap, &xcolor ) )  /* run out of room */
   {
      *actual_grey_levels = try_grey_levels-1;
      XFreeColors ( display, cmap, grey_screen, *actual_grey_levels, 0 );
      hor_free ( (void *) grey_screen );
      no_colours -= *actual_grey_levels;
      return NULL;
   }

   grey_screen[try_grey_levels-1] = xcolor.pixel;
   *actual_grey_levels = try_grey_levels;
   return grey_screen;
}

static void associate_grey_levels (Display *display,
				   u_long  *grey_screen,
				   int      grey_levels,
				   int      internal_grey_bits,
				   long     top_grey_level,
				   Hor_X_to_Image_ColourMap x_to_image_colourmap,
				   u_long                  *image_to_x_colourmap,
				   u_long                 **grey_ptr )
/* internal_grey_bits: number of bits per pixel for internal image format
   top_grey_level:     highest available grey level in internal format */
{
   int    internal_grey_levels = hor_power_of_two ( internal_grey_bits );
   int    grey_bits = hor_int_log_to_base_two ( grey_levels );
   int    level, factor;
   double x_image_factor;

   if ( internal_grey_bits < 1 || grey_levels < 2 )
      hor_error ( "cannot associate grey levels", HOR_FATAL );

   *grey_ptr = hor_malloc_ntype ( u_long, internal_grey_levels );
   if ( grey_bits <= internal_grey_bits ) /* this should normally be HOR_TRUE: the
					     internal pixel format (MIT, IFF
					     etc.) will normally have more bits
					     per pixel than the grey levels
					     on the display */
   {
      factor = hor_power_of_two ( internal_grey_bits - grey_bits );
      for ( level = 0; level < internal_grey_levels; level++ )
	 (*grey_ptr)[level] = grey_screen[level/factor];
   }
   else /* this should never happen, but just in case: */
   {
      factor = hor_power_of_two ( grey_bits - internal_grey_bits );
      for ( level = 0; level < internal_grey_levels; level++ )
	 (*grey_ptr)[level] = grey_screen[level*factor];
   }

   x_image_factor = (double)grey_levels/(double)(top_grey_level+1);
   switch ( hor_get_image_display_format() )
   {
      case HOR_U_CHAR:
      for ( level = 0; level < grey_levels; level++ )
         x_to_image_colourmap.uc[grey_screen[level]] =
	                         (u_char) ((double)level/x_image_factor + 0.5);
      break;

      case HOR_U_SHORT:
      for ( level = 0; level < grey_levels; level++ )
         x_to_image_colourmap.us[grey_screen[level]] =
	                        (u_short) ((double)level/x_image_factor + 0.5);
      break;

      default:
      hor_error ( "illegal image display format (associate_grey_levels)",
		  HOR_FATAL);
      break;
   }

   for ( level = 0; level <= top_grey_level; level++ )
      image_to_x_colourmap[level] =
	              grey_screen[(int)((double)(level + 0.5)*x_image_factor)];


   cpu_byte_order = (*((char *)&endian) == 0);
   image_grey_values = hor_malloc_ntype ( u_long, internal_grey_levels );

   if (cpu_byte_order == ImageByteOrder(display)) { /* no need to swap */
       for (level=0; level<internal_grey_levels; level++)
	   image_grey_values[level] = (*grey_ptr)[level];
   } else if (DefaultDepth ( display, DefaultScreen(display) ) > 16) {
       hor_error ("unsupported byte swap required for this depth screen",
		  HOR_FATAL);
   } else {
       for (level=0; level<internal_grey_levels; level++)
	   image_grey_values[level] = ((((*grey_ptr)[level])&0xFF)<<8) | (((*grey_ptr)[level])>>8);
   }

   return;
}

static void alloc_x_to_image ( int                       depth,
			       long                     *top_grey_level_ptr,
			       Hor_X_to_Image_ColourMap *x_to_image_colourmap,
			       u_long                  **image_to_x_colourmap )
{
   int x_colours;

   if ( depth > 31 )
      hor_error ( "depth of X display too big for inverse colour map (alloc_x_image_conversion_colourmaps)", HOR_FATAL );

   x_colours = hor_power_of_two ( depth );

   switch ( hor_get_image_display_format() )
   {
      case HOR_U_CHAR:
      x_to_image_colourmap->uc = hor_malloc_ntype ( u_char, x_colours );
      *image_to_x_colourmap = hor_malloc_ntype ( u_long, 256 );
      *top_grey_level_ptr = 255;
      break;

#ifndef HOR_MSDOS
      case HOR_U_SHORT:
      x_to_image_colourmap->us = hor_malloc_ntype ( u_int, x_colours );
      *image_to_x_colourmap = hor_malloc_ntype ( u_long, 16777216 );
      *top_grey_level_ptr = 16777215;
      break;
#endif

      default:
      hor_error ( "illegal image display format (alloc_x_image_conversion_colourmaps)", HOR_FATAL);

      break;
   }
}

/** following functions exported **/

/*******************
*   void @hor_colourmap_setup ( Display *display,
*                              u_int min_grey_bits, u_int max_grey_bits, ... )
*
*   Sets up X-windows colourmap with grey levels and colours. Arguments:
*
*   display       Connection to the X server.
*
*   min_grey_bits
*   max_grey_bits The minimum/maximum number of bits for displayed grey levels.
*
*                 The global array Hor_Grey[] is set by hor_colourmap_setup() to a grey
*                 level array of 0...255 which indexes into the displayed grey
*                 level colours, usually in many-to-one fashion.
*
*   ...           A list of colour definitions terminated by NULL.
*                 Each colour definition consists of two arguments:
*                 1: a string name for the colour which must appear in the
*                    colour database file /usr/lib/X11/rgb.txt (e.g. "Red").
*                 2: a variable of type (u_int *). This is set by
*                    hor_colourmap_setup() to an identifier for the colour.
********************/
void hor_colourmap_setup ( Display *display,
		           u_int min_grey_bits, u_int max_grey_bits, ... )
{
   int      max_grey_levels = hor_power_of_two ( max_grey_bits );
   int      min_grey_levels = hor_power_of_two ( min_grey_bits );
   int      depth;
   int      top_grey_level, grey_levels;

   u_long              *grey_screen;

   va_list  ap;

   cmap_display = display;
   if ( max_grey_bits >= hor_get_image_display_depth() )
      hor_error ( "max_grey_bits >= internal depth (hor_colourmap_setup)",
		  HOR_FATAL );

   cmap = XDefaultColormap ( display, DefaultScreen(display) );

   /* allocate inverse of colourmap */
   depth = DefaultDepth ( display, DefaultScreen(display) );
   alloc_x_to_image ( depth, &top_grey_level,
		      &x_to_image_colourmap, &image_to_x_colourmap );

   va_start ( ap, max_grey_bits );
   no_named_colours = count_colours ( &ap );
   va_end(ap);
   
   va_start ( ap, max_grey_bits );
   if ( !alloc_colours ( display, cmap, &top_grey_level,
			 x_to_image_colourmap, image_to_x_colourmap, &ap ) )
      hor_error ( "could not allocate colour cells for colour-map", HOR_FATAL );

   va_end(ap);

   /* now try to generate max_grey_levels grey levels from what's left of
      colour resources */
   if ( (grey_screen = alloc_grey_levels ( display, cmap, max_grey_levels,
					   &grey_levels ) ) == NULL )
      if ( grey_levels < min_grey_levels )
	 hor_error("could not allocate enough grey level cells for colour map",
		   HOR_FATAL);
      else /* free grey level cells and try again with smaller number */
      {
	 grey_levels = hor_highest_power_of_two_leq ( grey_levels );
	 if ( (grey_screen = alloc_grey_levels ( display, cmap, grey_levels,
						 &grey_levels ) ) == NULL )
	    hor_error ( "could not allocate grey level colour cells", HOR_FATAL );
      }

   associate_grey_levels ( display, grey_screen, grey_levels, hor_get_image_display_depth(),
			   top_grey_level, x_to_image_colourmap,
			   image_to_x_colourmap, &Hor_Grey );
   x_to_image_init = HOR_TRUE;
   hor_free ( (void *) grey_screen );
}

/*******************
*   u_long @hor_colour_alloc ( u_short red, u_short green, u_short blue )
*
*   Allocates a new colour with given RGB values (0 to 65535 on Sparc X
*   windows systems) and returns an identifier for it to be used in graphics
*   commands (e.g. hor_display_set_colour()).
********************/
u_long hor_colour_alloc ( u_short red, u_short green, u_short blue )
{
   XColor xcolor;

   if ( cmap_display == NULL )
      hor_error ( "colourmap not initialised (hor_colour_alloc)", HOR_FATAL );

   xcolor.red   = red;
   xcolor.green = green;
   xcolor.blue  = blue;
   if ( !XAllocColor ( cmap_display, cmap, &xcolor ) )
      hor_error ( "ran out of colourmap resources (hor_colour_alloc)", HOR_FATAL );

   all_colours[no_colours++] = xcolor.pixel;
   return xcolor.pixel;
}

/*******************
*   Hor_Bool @hor_get_colour_name_value ( const char *name, u_long *colour )
*
*   Finds the value for the colour with the given name, as passed to
*   hor_colourmap_setup(). Returns HOR_TRUE on success, HOR_FALSE with an error message
*   on failure.
********************/
Hor_Bool hor_get_colour_name_value ( const char *name, u_long *colour )
{
   int count;

   for ( count = 0; count < no_named_colours; count++ )
      if ( strcmp ( colour_name[count].name, name ) == 0 )
      {
	 *colour = colour_name[count].colour;
	 return HOR_TRUE;
      }

   hor_errno = HOR_GRAPHICS_COLOUR_NOT_ALLOCATED;
   return HOR_FALSE;
}

/*******************
*   Hor_Image_Type @hor_get_image_display_format(void)
*   u_int          @hor_get_image_display_depth(void)
*
*   hor_get_image_display_format() returns the type of the internal grey-level
*   colourmap, currently HOR_U_CHAR (unsigned char).
*
*   hor_get_image_display_depth() returns the bit depth of the internal grey-level
*   colourmap type, i.e. 8 for HOR_U_CHAR.
********************/
Hor_Image_Type hor_get_image_display_format ( void )
{
   return IMAGE_DISPLAY_FORMAT;
}

u_int hor_get_image_display_depth ( void )
{
   switch ( hor_get_image_display_format() )
   {
      case HOR_U_CHAR:
      return ( 8*sizeof(u_char) );
      break;

      case HOR_U_SHORT:
      return 24;
      break;

      default:
      hor_error ( "illegal image display format (hor_get_image_display_depth)",
		  HOR_FATAL);
      return 0;
   }
}
