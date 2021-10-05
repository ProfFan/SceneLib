/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <stdarg.h>
#ifdef HOR_MSDOS
#include <X11/Intrnsc.h>
#else
#include <X11/Intrinsic.h>
#endif

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"

#include "ximage.h"

static const int endian = 0x1;
static int cpu_byte_order, swap_bytes;
extern u_long *image_grey_values;

static u_long   below_threshold_colour, above_threshold_colour;
static Hor_Bool initialized = HOR_FALSE;

void hor_set_threshold_colours ( u_long loc_below_threshold_colour,
				 u_long loc_above_threshold_colour )
{
   below_threshold_colour = loc_below_threshold_colour;
   above_threshold_colour = loc_above_threshold_colour;
   initialized = HOR_TRUE;
}

static char *byte_array_pixel_factor_1 ( Hor_Imdata     imarray,
					 Hor_Image_Type type,
					 int        width,
					 int        height )
{
   char   *xdata;
   int     size = width*height;
   u_char *ptr;

   xdata = hor_malloc ( size );
   ptr = (u_char *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    for ( col = 0; col < width; col++, ptr++ )
	       if ( hor_get_bit ( arptr, col ) ) *ptr = image_grey_values[255];
	       else                              *ptr = image_grey_values[0];
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char *arptr, *aend;

	 for ( aend = imarray.uc[0] + width*height, arptr = imarray.uc[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[*arptr];
      }
      break;

      case HOR_CHAR:
      {
	 char *arptr, *aend;

	 for ( aend = imarray.c[0] + width*height, arptr = imarray.c[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[128 + (int)*arptr];
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *arptr, *aend;

	 for ( aend = imarray.us[0] + width*height, arptr = imarray.us[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[*arptr/256];
      }
      break;

      case HOR_SHORT:
      {
	 short *arptr, *aend;

	 for ( aend = imarray.s[0] + width*height, arptr = imarray.s[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[128 + *arptr/256];
      }
      break;

      default:
      hor_error ( "invalid image type (byte_array_pixel_factor_1)", HOR_FATAL);
      break;
   }

   return xdata;
}

/* byte_array_pixel_factor_2(): copies each internal format pixel into area of
                                display memory 2*2 canvas pixels */

static char *byte_array_pixel_factor_2 ( Hor_Imdata     imarray,
					 Hor_Image_Type type,
					 int            width,
					 int            height )
{
   char   *xdata;
   int     size = width*height*2*2;
   int     row;
   u_char *ptr1, *ptr2;

   xdata = hor_malloc ( size );
   ptr2 = (u_char *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    ptr1 = ptr2;
            ptr2 = ptr1 + width*2;
	    for ( col = 0; col < width; col++, ptr1 += 2, ptr2 += 2 )
	       if ( hor_get_bit ( arptr, col ) )
		  ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[255];
	       else
		  ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[0];
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[*arptr];
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[128 + (int)*arptr];
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[*arptr/256];
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[128 + *arptr/256];
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (byte_array_pixel_factor_2)", HOR_FATAL );
      break;
   }

   return xdata;
}

/* byte_array_any_pixel_factor(): copies each internal format pixel into area
                                  of display memory
				  pixel_size_factor*pixel_size_factor canvas
				  pixels */
static char *byte_array_any_pixel_factor ( Hor_Imdata     imarray,
					   Hor_Image_Type type,
					   int width, int height,
					   int pixel_size_factor_c,
					   int pixel_size_factor_r )
{
   char    *xdata;
   int      size = width*height*pixel_size_factor_c*pixel_size_factor_r;
   int      row, count1, count2;
   u_char **ptrarray, *ptr, colour;

   xdata = hor_malloc ( size );
   ptrarray = hor_malloc_ntype ( u_char *, pixel_size_factor_r );
   ptrarray[pixel_size_factor_r-1] = (u_char *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( col = 0; col < width; col++ )
	    {
	       if ( hor_get_bit ( arptr, col ) ) colour = image_grey_values[255];
	       else                          colour = image_grey_values[0];

	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[*arptr];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[128 + (int)*arptr];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[*arptr/256];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[128 + *arptr/256];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (byte_array_any_factor)", HOR_FATAL );
      break;
   }

   hor_free ( (void *) ptrarray );
   return xdata;
}

/* byte_array(): converts image data from internal format to X display format*/
static char *byte_array ( Hor_Imdata     imarray,
			  Hor_Image_Type type,
			  int width,           int height,
			  int hor_subsample_c, int hor_subsample_r,
			  int pixel_size_factor )
{
   char   *xdata;
   u_char *ptr;
   int     cwidth, cheight, c, r;

   if ( pixel_size_factor > 0 )
   {
      int pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
      int pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

      /* special cases for efficiency */
      if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
      return ( byte_array_pixel_factor_1 ( imarray, type, width, height ) );
      else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
	 /* this is also quite common, e.g. displaying 256*256 images
	    on a 512*512 canvas, so make this run fast too */
	 return ( byte_array_pixel_factor_2 ( imarray, type, width, height ) );
      else /* the general case */
	 return ( byte_array_any_pixel_factor ( imarray, type, width, height,
					        pixel_size_factor_c,
					        pixel_size_factor_r ) );
   }

   pixel_size_factor *= -1;
   cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
   cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
   xdata = hor_malloc ( cwidth*cheight );
   ptr = (u_char *) xdata;

   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	    {
	       if ( hor_get_bit ( arptr, c*pixel_size_factor/hor_subsample_c ))
		  *ptr = image_grey_values[255];
	       else *ptr = image_grey_values[0];
	    }
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[arptr[c*pixel_size_factor/hor_subsample_c]];
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[128 + (int) arptr[c*pixel_size_factor/hor_subsample_c]];
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[arptr[c*pixel_size_factor/hor_subsample_c]/256];
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[128 + arptr[c*pixel_size_factor/hor_subsample_c]/256];
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (byte_array_any_factor)", HOR_FATAL );
      break;
   }

   return xdata;
}

/* byte_array_float(): converts image data from internal float format
                       to X display format*/
static char *byte_array_float ( Hor_Imdata imarray,
			        int    width,           int    height,
			        int    hor_subsample_c, int hor_subsample_r,
			        int    pixel_size_factor,
			        float  black_val, float  white_val )
{
   float   low_float_val, float_factor;
   int     pixel_size_factor_c, pixel_size_factor_r;
   char   *xdata;
   int     size;

   if ( black_val == white_val )
      hor_error ( "illegal grey-level range (byte_array_float)", HOR_FATAL );

   low_float_val = black_val;
   float_factor  = 256.0/(white_val - black_val);

   if ( pixel_size_factor < 0 )
   {
      int     cwidth, cheight, c, r, temp;
      float **array = imarray.f, *arptr;
      u_char *ptr;

      pixel_size_factor *= -1;
      cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
      cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
      xdata = hor_malloc ( cwidth*cheight );
      ptr = (u_char *) xdata;

      for ( r = 0; r < cheight; r++ )
      {
	 arptr = array[r*pixel_size_factor/hor_subsample_r];
	 for ( c = 0; c < cwidth; c++, ptr++ )
	 {
	    temp = (int) ((arptr[c*pixel_size_factor/hor_subsample_c] -
			   low_float_val)*float_factor);
	    *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	           : ( temp < 0 ? below_threshold_colour :
		                  above_threshold_colour );
	 }
      }

      return xdata;
   }

   pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
   pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

   /* special cases for efficiency */
   if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
   {
      float  *arptr, *aend;
      u_char *ptr;
      int     temp;

      size = width*height;
      xdata = hor_malloc ( size );
      ptr = (u_char *) xdata;

      for ( aend = imarray.f[0] + width*height, arptr = imarray.f[0];
	    arptr != aend; ptr++, arptr++ )
      {
	 temp = (int) ((*arptr - low_float_val)*float_factor);
	 *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	        : ( temp < 0 ? below_threshold_colour :
		               above_threshold_colour );
      }
   }
   else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
      /* this is also quite common, e.g. displaying 256*256 images
	 on a 512*512 canvas, so make this run fast too */
   {
      int     row;
      u_char *ptr1, *ptr2;
      float **array = imarray.f, *arptr, *aend;
      int     temp;

      size = width*height*2*2;
      xdata = hor_malloc ( size );
      ptr2 = (u_char *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptr1 = ptr2;
	 ptr2 = ptr1 + width*2;
	 for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	 {
	    temp = (int) ((*arptr - low_float_val)*float_factor);
	    ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1]
	            = (temp >= 0 && temp < 256) ? image_grey_values[temp]
		      : ( temp < 0 ? below_threshold_colour :
			             above_threshold_colour );
	 }
      }
   }
   else /* the general case */
   {
      int      row, count1, count2;
      u_char **ptrarray, *ptr, colour;
      float  **array = imarray.f, *arptr, *aend;
      int      temp;

      size = width*height*pixel_size_factor_c*pixel_size_factor_r;
      xdata = hor_malloc ( size );
      ptrarray = hor_malloc_ntype ( u_char *, pixel_size_factor_r );
      ptrarray[pixel_size_factor_r-1] = (u_char *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	 for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	    ptrarray[count1] = ptrarray[count1-1] + width*pixel_size_factor_c;

	 for ( ; arptr != aend; arptr++ )
	 {
	    temp = (int) ((*arptr - low_float_val)*float_factor);
	    colour = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	             : ( temp < 0 ? below_threshold_colour :
			            above_threshold_colour );
	    for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	    {
	       ptr = ptrarray[count1];
	       for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		  ptr[count2] = colour;

	       ptrarray[count1] += pixel_size_factor_c;
	    }
	 }
      }

      hor_free ( (void *) ptrarray );
   }

   return xdata;
}

/* byte_array_int(): converts image data from internal int format
                     to X display format*/
static char *byte_array_int ( Hor_Imdata imarray,
			      int width,           int height,
			      int hor_subsample_c, int hor_subsample_r,
			      int pixel_size_factor,
			      int black_val, int white_val )
{
   int   low_int_val, range;
   int   pixel_size_factor_c, pixel_size_factor_r;
   char *xdata;
   int   size;

   if ( black_val == white_val )
      hor_error ( "illegal grey-level range (byte_array_int)", HOR_FATAL );

   low_int_val = black_val;
   range = white_val - black_val;

   if ( pixel_size_factor < 0 )
   {
      int     cwidth, cheight, c, r, temp;
      int   **array = imarray.i, *arptr;
      u_char *ptr;

      pixel_size_factor *= -1;
      cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
      cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
      xdata = hor_malloc ( cwidth*cheight );
      ptr = (u_char *) xdata;

      for ( r = 0; r < cheight; r++ )
      {
	 arptr = array[r*pixel_size_factor/hor_subsample_r];
	 for ( c = 0; c < cwidth; c++, ptr++ )
	 {
	    temp = (int) ((arptr[c*pixel_size_factor/hor_subsample_c] -
			   low_int_val)*255)/range;
	    *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	           : ( temp < 0 ? below_threshold_colour :
		                  above_threshold_colour );
	 }
      }

      return xdata;
   }

   pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
   pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

   /* special cases for efficiency */
   if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
   {
      int    *arptr, *aend;
      u_char *ptr;
      int     temp;

      size = width*height;
      xdata = hor_malloc ( size );
      ptr = (u_char *) xdata;

      for ( aend = imarray.i[0] + width*height, arptr = imarray.i[0];
	    arptr != aend; ptr++, arptr++ )
      {
	 temp = ((*arptr - low_int_val)*255)/range;
	 *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	        : ( temp < 0 ? below_threshold_colour :
		               above_threshold_colour );
      }
   }
   else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
      /* this is also quite common, e.g. displaying 256*256 images
	 on a 512*512 canvas, so make this run fast too */
   {
      int     row;
      u_char *ptr1, *ptr2;
      int   **array = imarray.i, *arptr, *aend;
      int     temp;

      size = width*height*2*2;
      xdata = hor_malloc ( size );
      ptr2 = (u_char *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptr1 = ptr2;
	 ptr2 = ptr1 + width*2;
	 for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	 {
	    temp = ((*arptr - low_int_val)*255)/range;
	    ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1]
	            = (temp >= 0 && temp < 256) ? image_grey_values[temp]
		      : ( temp < 0 ? below_threshold_colour :
			             above_threshold_colour );
	 }
      }
   }
   else /* the general case */
   {
      int      row, count1, count2;
      u_char **ptrarray, *ptr, colour;
      int    **array = imarray.i, *arptr, *aend;
      int      temp;

      size = width*height*pixel_size_factor_c*pixel_size_factor_r;
      xdata = hor_malloc ( size );
      ptrarray = hor_malloc_ntype ( u_char *, pixel_size_factor_r );
      ptrarray[pixel_size_factor_r-1] = (u_char *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	 for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	    ptrarray[count1] = ptrarray[count1-1] + width*pixel_size_factor_c;

	 for ( ; arptr != aend; arptr++ )
	 {
	    temp = ((*arptr - low_int_val)*255)/range;
	    colour = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	             : ( temp < 0 ? below_threshold_colour :
			            above_threshold_colour );
	    for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	    {
	       ptr = ptrarray[count1];
	       for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		  ptr[count2] = colour;

	       ptrarray[count1] += pixel_size_factor_c;
	    }
	 }
      }

      hor_free ( (void *) ptrarray );
   }

   return xdata;
}


static short *short_array_pixel_factor_1 ( Hor_Imdata     imarray,
					 Hor_Image_Type type,
					 int        width,
					 int        height )
{
   short   *xdata;
   int      size = width*height;
   u_short *ptr;

   xdata = hor_malloc ( 2*size );
   ptr = (u_short *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    for ( col = 0; col < width; col++, ptr++ )
	       if ( hor_get_bit ( arptr, col ) ) *ptr = image_grey_values[255];
	       else                              *ptr = image_grey_values[0];
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char *arptr, *aend;

	 for ( aend = imarray.uc[0] + width*height, arptr = imarray.uc[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[*arptr];
      }
      break;

      case HOR_CHAR:
      {
	 char *arptr, *aend;

	 for ( aend = imarray.c[0] + width*height, arptr = imarray.c[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[128 + (int)*arptr];
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *arptr, *aend;

	 for ( aend = imarray.us[0] + width*height, arptr = imarray.us[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[*arptr/256];
      }
      break;

      case HOR_SHORT:
      {
	 short *arptr, *aend;

	 for ( aend = imarray.s[0] + width*height, arptr = imarray.s[0];
	       arptr != aend; ptr++, arptr++ )
	    *ptr = image_grey_values[128 + *arptr/256];
      }
      break;

      default:
      hor_error ( "invalid image type (short_array_pixel_factor_1)", HOR_FATAL);
      break;
   }

   return xdata;
}

/* short_array_pixel_factor_2(): copies each internal format pixel into area of
                                 display memory 2*2 canvas pixels */

static short *short_array_pixel_factor_2 ( Hor_Imdata     imarray,
					 Hor_Image_Type type,
					 int            width,
					 int            height )
{
   short   *xdata;
   int      size = width*height*2*2;
   int      row;
   u_short *ptr1, *ptr2;

   xdata = hor_malloc ( sizeof(short) * size );
   ptr2 = (u_short *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    ptr1 = ptr2;
            ptr2 = ptr1 + width*2;
	    for ( col = 0; col < width; col++, ptr1 += 2, ptr2 += 2 )
	       if ( hor_get_bit ( arptr, col ) )
		  ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[255];
	       else
		  ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[0];
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[*arptr];
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[128 + (int)*arptr];
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[*arptr/256];
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptr1 = ptr2;
	    ptr2 = ptr1 + width*2;
	    for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	       ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1] = image_grey_values[128 + *arptr/256];
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (short_array_pixel_factor_2)", HOR_FATAL );
      break;
   }

   return xdata;
}

/* short_array_any_pixel_factor(): copies each internal format pixel into area
                                   of display memory
		 		   pixel_size_factor*pixel_size_factor canvas
				   pixels */
static short *short_array_any_pixel_factor ( Hor_Imdata     imarray,
					   Hor_Image_Type type,
					   int width, int height,
					   int pixel_size_factor_c,
					   int pixel_size_factor_r )
{
   short    *xdata;
   int       size = width*height*pixel_size_factor_c*pixel_size_factor_r;
   int       row, count1, count2;
   u_short **ptrarray, *ptr, colour;

   xdata = hor_malloc ( 2 * size );
   ptrarray = hor_malloc_ntype ( u_short *, pixel_size_factor_r );
   ptrarray[pixel_size_factor_r-1] = (u_short *) xdata;
   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;
	 int       col, row;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( col = 0; col < width; col++ )
	    {
	       if ( hor_get_bit ( arptr, col ) ) colour = image_grey_values[255];
	       else                          colour = image_grey_values[0];

	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[*arptr];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[128 + (int)*arptr];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[*arptr/256];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr, *aend;

	 for ( row = 0; row < height; row++ )
	 {
	    arptr = array[row];
	    aend = arptr + width;
	    ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	    for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	       ptrarray[count1] = ptrarray[count1-1]+width*pixel_size_factor_c;

	    for ( ; arptr != aend; arptr++ )
	    {
	       colour = image_grey_values[128 + *arptr/256];
	       for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	       {
		  ptr = ptrarray[count1];
		  for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		     ptr[count2] = colour;

		  ptrarray[count1] += pixel_size_factor_c;
	       }
	    }
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (short_array_any_factor)", HOR_FATAL );
      break;
   }

   hor_free ( (void *) ptrarray );
   return xdata;
}

/* short_array(): converts image data from internal format to X display format*/
static short *short_array ( Hor_Imdata     imarray,
			   Hor_Image_Type type,
			   int width,           int height,
			   int hor_subsample_c, int hor_subsample_r,
			   int pixel_size_factor )
{
   short   *xdata;
   u_short *ptr;
   int      cwidth, cheight, c, r;


   if ( pixel_size_factor > 0 )
   {
      int pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
      int pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

      /* special cases for efficiency */
      if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
      return ( short_array_pixel_factor_1 ( imarray, type, width, height ) );
      else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
	 /* this is also quite common, e.g. displaying 256*256 images
	    on a 512*512 canvas, so make this run fast too */
	 return ( short_array_pixel_factor_2 ( imarray, type, width, height ) );
      else /* the general case */
	 return ( short_array_any_pixel_factor ( imarray, type, width, height,
					        pixel_size_factor_c,
					        pixel_size_factor_r ) );
   }

   pixel_size_factor *= -1;
   cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
   cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
   xdata = hor_malloc ( 2 * cwidth*cheight );
   ptr = (u_short *) xdata;

   switch ( type )
   {
      case HOR_BIT:
      {
	 hor_bit **array = imarray.b, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	    {
	       if ( hor_get_bit ( arptr, c*pixel_size_factor/hor_subsample_c ))
		  *ptr = image_grey_values[255];
	       else *ptr = image_grey_values[0];
	    }
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **array = imarray.uc, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[arptr[c*pixel_size_factor/hor_subsample_c]];
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **array = imarray.c, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[128 + (int) arptr[c*pixel_size_factor/hor_subsample_c]];
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **array = imarray.us, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[arptr[c*pixel_size_factor/hor_subsample_c]/256];
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **array = imarray.s, *arptr;

	 for ( r = 0; r < cheight; r++ )
	 {
	    arptr = array[r*pixel_size_factor/hor_subsample_r];
	    for ( c = 0; c < cwidth; c++, ptr++ )
	       *ptr = image_grey_values[128 + arptr[c*pixel_size_factor/hor_subsample_c]/256];
	 }
      }
      break;

      default:
      hor_error ( "invalid image type (short_array_any_factor)", HOR_FATAL );
      break;
   }

   return xdata;
}

/* short_array_float(): converts image data from internal float format
                       to X display format*/
static short *short_array_float ( Hor_Imdata imarray,
				 int    width,           int    height,
				 int    hor_subsample_c, int hor_subsample_r,
				 int    pixel_size_factor,
				 float  black_val, float  white_val )
{
   float   low_float_val, float_factor;
   int     pixel_size_factor_c, pixel_size_factor_r;
   short  *xdata;
   int     size;

   if ( black_val == white_val )
      hor_error ( "illegal grey-level range (byte_array_float)", HOR_FATAL );

   low_float_val = black_val;
   float_factor  = 256.0/(white_val - black_val);

   if ( pixel_size_factor < 0 )
   {
      int      cwidth, cheight, c, r, temp;
      float  **array = imarray.f, *arptr;
      u_short *ptr;

      pixel_size_factor *= -1;
      cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
      cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
      xdata = hor_malloc ( 2 * cwidth*cheight );
      ptr = (u_short *) xdata;

      for ( r = 0; r < cheight; r++ )
      {
	 arptr = array[r*pixel_size_factor/hor_subsample_r];
	 for ( c = 0; c < cwidth; c++, ptr++ )
	 {
	    temp = (int) ((arptr[c*pixel_size_factor/hor_subsample_c] -
			   low_float_val)*float_factor);
	    *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	           : ( temp < 0 ? below_threshold_colour :
		                  above_threshold_colour );
	 }
      }

      return xdata;
   }

   pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
   pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

   /* special cases for efficiency */
   if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
   {
      float   *arptr, *aend;
      u_short *ptr;
      int      temp;

      size = width*height;
      xdata = hor_malloc ( 2 * size );
      ptr = (u_short *) xdata;

      for ( aend = imarray.f[0] + width*height, arptr = imarray.f[0];
	    arptr != aend; ptr++, arptr++ )
      {
	 temp = (int) ((*arptr - low_float_val)*float_factor);
	 *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	        : ( temp < 0 ? below_threshold_colour :
		               above_threshold_colour );
      }
   }
   else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
      /* this is also quite common, e.g. displaying 256*256 images
	 on a 512*512 canvas, so make this run fast too */
   {
      int       row;
      u_short  *ptr1, *ptr2;
      float   **array = imarray.f, *arptr, *aend;
      int       temp;

      size = width*height*2*2;
      xdata = hor_malloc ( 2 * size );
      ptr2 = (u_short *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptr1 = ptr2;
	 ptr2 = ptr1 + width*2;
	 for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	 {
	    temp = (int) ((*arptr - low_float_val)*float_factor);
	    ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1]
	            = (temp >= 0 && temp < 256) ? image_grey_values[temp]
		      : ( temp < 0 ? below_threshold_colour :
			             above_threshold_colour );
	 }
      }
   }
   else /* the general case */
   {
      int       row, count1, count2;
      u_short **ptrarray, *ptr, colour;
      float   **array = imarray.f, *arptr, *aend;
      int       temp;

      size = width*height*pixel_size_factor_c*pixel_size_factor_r;
      xdata = hor_malloc ( 2 * size );
      ptrarray = hor_malloc_ntype ( u_short *, pixel_size_factor_r );
      ptrarray[pixel_size_factor_r-1] = (u_short *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	 for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	    ptrarray[count1] = ptrarray[count1-1] + width*pixel_size_factor_c;

	 for ( ; arptr != aend; arptr++ )
	 {
	    temp = (int) ((*arptr - low_float_val)*float_factor);
	    colour = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	             : ( temp < 0 ? below_threshold_colour :
			            above_threshold_colour );
	    for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	    {
	       ptr = ptrarray[count1];
	       for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		  ptr[count2] = colour;

	       ptrarray[count1] += pixel_size_factor_c;
	    }
	 }
      }

      hor_free ( (void *) ptrarray );
   }

   return xdata;
}

/* short_array_int(): converts image data from internal int format
                      to X display format*/
static short *short_array_int ( Hor_Imdata imarray,
			       int width,           int height,
			       int hor_subsample_c, int hor_subsample_r,
			       int pixel_size_factor,
			       int black_val, int white_val )
{
   int    low_int_val, range;
   int    pixel_size_factor_c, pixel_size_factor_r;
   short *xdata;
   int    size;

   if ( black_val == white_val )
      hor_error ( "illegal grey-level range (byte_array_int)", HOR_FATAL );

   low_int_val = black_val;
   range = white_val - black_val;

   if ( pixel_size_factor < 0 )
   {
      int       cwidth, cheight, c, r, temp;
      int     **array = imarray.i, *arptr;
      u_short  *ptr;

      pixel_size_factor *= -1;
      cwidth = (width-1)*hor_subsample_c/pixel_size_factor + 1;
      cheight = (height-1)*hor_subsample_r/pixel_size_factor + 1;
      xdata = hor_malloc ( 2 * cwidth*cheight );
      ptr = (u_short *) xdata;

      for ( r = 0; r < cheight; r++ )
      {
	 arptr = array[r*pixel_size_factor/hor_subsample_r];
	 for ( c = 0; c < cwidth; c++, ptr++ )
	 {
	    temp = (int) ((arptr[c*pixel_size_factor/hor_subsample_c] -
			   low_int_val)*255)/range;
	    *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	           : ( temp < 0 ? below_threshold_colour :
		                  above_threshold_colour );
	 }
      }

      return xdata;
   }

   pixel_size_factor_c = hor_subsample_c*pixel_size_factor;
   pixel_size_factor_r = hor_subsample_r*pixel_size_factor;

   /* special cases for efficiency */
   if ( pixel_size_factor_c == 1 && pixel_size_factor_r == 1 )
      /* usually HOR_TRUE, so make this run fast */
   {
      int     *arptr, *aend;
      u_short *ptr;
      int      temp;

      size = width*height;
      xdata = hor_malloc ( 2 * size );
      ptr = (u_short *) xdata;

      for ( aend = imarray.i[0] + width*height, arptr = imarray.i[0];
	    arptr != aend; ptr++, arptr++ )
      {
	 temp = ((*arptr - low_int_val)*255)/range;
	 *ptr = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	        : ( temp < 0 ? below_threshold_colour :
		               above_threshold_colour );
      }
   }
   else if ( pixel_size_factor_c == 2 && pixel_size_factor_r == 2 )
      /* this is also quite common, e.g. displaying 256*256 images
	 on a 512*512 canvas, so make this run fast too */
   {
      int       row;
      u_short  *ptr1, *ptr2;
      int     **array = imarray.i, *arptr, *aend;
      int       temp;

      size = width*height*2*2;
      xdata = hor_malloc ( 2 * size );
      ptr2 = (u_short *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptr1 = ptr2;
	 ptr2 = ptr1 + width*2;
	 for ( ; arptr != aend; arptr++, ptr1 += 2, ptr2 += 2 )
	 {
	    temp = ((*arptr - low_int_val)*255)/range;
	    ptr1[0] = ptr1[1] = ptr2[0] = ptr2[1]
	            = (temp >= 0 && temp < 256) ? image_grey_values[temp]
		      : ( temp < 0 ? below_threshold_colour :
			             above_threshold_colour );
	 }
      }
   }
   else /* the general case */
   {
      int       row, count1, count2;
      u_short **ptrarray, *ptr, colour;
      int     **array = imarray.i, *arptr, *aend;
      int       temp;

      size = width*height*pixel_size_factor_c*pixel_size_factor_r;
      xdata = hor_malloc ( 2 * size );
      ptrarray = hor_malloc_ntype ( u_short *, pixel_size_factor_r );
      ptrarray[pixel_size_factor_r-1] = (u_short *) xdata;

      for ( row = 0; row < height; row++ )
      {
	 arptr = array[row];
	 aend = arptr + width;
	 ptrarray[0] = ptrarray[pixel_size_factor_r-1];
	 for ( count1 = 1; count1 < pixel_size_factor_r; count1++ )
	    ptrarray[count1] = ptrarray[count1-1] + width*pixel_size_factor_c;

	 for ( ; arptr != aend; arptr++ )
	 {
	    temp = ((*arptr - low_int_val)*255)/range;
	    colour = (temp >= 0 && temp < 256) ? image_grey_values[temp]
	             : ( temp < 0 ? below_threshold_colour :
			            above_threshold_colour );
	    for ( count1 = 0; count1 < pixel_size_factor_r; count1++ )
	    {
	       ptr = ptrarray[count1];
	       for ( count2 = 0; count2 < pixel_size_factor_c; count2++)
		  ptr[count2] = colour;

	       ptrarray[count1] += pixel_size_factor_c;
	    }
	 }
      }

      hor_free ( (void *) ptrarray );
   }

   return xdata;
}


/*******************
*   XImage *@hor_convert_image_to_X_format ( Hor_Image *imptr,
*                                           Display   *display,
*                                           int        scrnum,
*                                           int        depth,
*                                           int        hor_subsample_c,
*                                           int        hor_subsample_r,
*                                           int        pixel_size_factor,
*                                           va_list *  aptr )
*
*   Converts internal format image (imptr) to X format so it can be displayed.
*   If the image is of type HOR_INT or HOR_FLOAT, it is converted using two
*   double-precision floating-point arguments in the variable argument list
*   aptr as lower/upper thresholds.
********************/
XImage *hor_convert_image_to_X_format ( Hor_Image *imptr,
				        Display   *display,
				        int        scrnum,
				        int        depth,
				        int        hor_subsample_c,
				        int        hor_subsample_r,
				        int        pixel_size_factor,
				        va_list   *aptr )
{
   char *xdata = NULL;

   if ( !initialized )
      hor_error ( "not initialized (hor_convert_image_to_X_format)",
		  HOR_FATAL );

   cpu_byte_order = (*((char *)&endian) == 0);
   swap_bytes = (cpu_byte_order!=ImageByteOrder(display));

   /* convert image grey level data to XImage format using colour map */
   switch ( depth )
   {
      case 8: /* byte image data */
      switch ( imptr->type )
      {
         case HOR_INT:
	 xdata = byte_array_int ( imptr->array, imptr->width, imptr->height,
				  hor_subsample_c, hor_subsample_r,
				  pixel_size_factor,
				  (int) (va_arg ( *aptr, double ) + 0.5),
				  (int) (va_arg ( *aptr, double ) + 0.5) );
	 break;

         case HOR_FLOAT:
	 xdata = byte_array_float (imptr->array, imptr->width, imptr->height,
				   hor_subsample_c, hor_subsample_r,
				   pixel_size_factor,
				   va_arg ( *aptr, double ),
				   va_arg ( *aptr, double ) );
	 break;

         default:
	 xdata = byte_array ( imptr->array, imptr->type,
			      imptr->width, imptr->height,
			      hor_subsample_c, hor_subsample_r,
			      pixel_size_factor );
	 break;
      }
      break;

      case 16: /* short ximage data */
      switch ( imptr->type )
      {
         case HOR_INT:
	 xdata = (char *)short_array_int ( imptr->array, imptr->width, imptr->height,
					  hor_subsample_c, hor_subsample_r,
					  pixel_size_factor,
					  (int) (va_arg ( *aptr, double ) + 0.5),
					  (int) (va_arg ( *aptr, double ) + 0.5));
	 break;

         case HOR_FLOAT:
	 xdata = (char *)short_array_float (imptr->array, imptr->width, imptr->height,
					    hor_subsample_c, hor_subsample_r,
					    pixel_size_factor,
					    va_arg ( *aptr, double ),
					    va_arg ( *aptr, double ) );
	 break;

         default:
	 xdata = (char *)short_array ( imptr->array, imptr->type,
				      imptr->width, imptr->height,
				      hor_subsample_c, hor_subsample_r,
				      pixel_size_factor );
	 break;
      }
      break;

      /* stick other depths here */

      default:
      hor_errno = HOR_GRAPHICS_INVALID_DISPLAY_DEPTH;
      break;
   }

   /* create XImage structure */
   if ( pixel_size_factor > 0 )
      return ( XCreateImage ( display, DefaultVisual (display, scrnum),
			      depth, ZPixmap, 0, xdata,
			      imptr->width*hor_subsample_c*pixel_size_factor,
			      imptr->height*hor_subsample_r*pixel_size_factor,
			      depth, 0 ) );
   else
      return ( XCreateImage ( display, DefaultVisual (display, scrnum),
			      depth, ZPixmap, 0, xdata,
			      (imptr->width-1)*hor_subsample_c
			      /(-pixel_size_factor) + 1,
			      (imptr->height-1)*hor_subsample_r
			      /(-pixel_size_factor) + 1,
			      depth, 0 ) );
}

/*******************
*   void @hor_display_X_format_image ( XImage  *ximage,
*                                     Display *display,
*                                     Window   canvas_window,
*                                     GC       graphics_context,
*                                     int      canvas_top_left_c,
*                                     int      canvas_top_left_r )
*
*   Displays X format image on canvas.
********************/
void hor_display_X_format_image ( XImage  *ximage,
				  Display *display,
				  Window   canvas_window,
				  GC       graphics_context,
				  int      canvas_top_left_c,
				  int      canvas_top_left_r )
{
   hor_display_set_function ( HOR_DISPLAY_COPY );

   /* display image on canvas */
   XPutImage ( display, canvas_window, graphics_context,
	       ximage, 0, 0, canvas_top_left_c, canvas_top_left_r,
	       ximage->width, ximage->height );

   hor_display_flush();
}
