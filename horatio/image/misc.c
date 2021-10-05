/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <math.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/image.h"

static Hor_Bool compatible_sub_images ( Hor_Sub_Image *sub_image1,
				        Hor_Sub_Image *sub_image2 )
{
   Hor_Image *image1 = &(sub_image1->image);
   Hor_Image *image2 = &(sub_image2->image);

   if ( image1->type != image2->type ) return HOR_FALSE;

   /* check whether images overlap */
   if ( sub_image1->c0 > sub_image2->c0 + image2->width ||
        sub_image2->c0 > sub_image1->c0 + image1->width ||
        sub_image1->r0 > sub_image2->r0 + image2->height ||
        sub_image2->r0 > sub_image1->r0 + image1->height )
      return HOR_FALSE;

   return HOR_TRUE;
}

/* add_image_data(): assumes images are same size and type */
static void add_image_data ( Hor_Image *image1, Hor_Image *image2,
			     Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;
   int temp;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 u_char *resp, *resend;

	 result->type = HOR_U_CHAR;
	 hor_alloc_image_data ( width, height, HOR_U_CHAR, &result->array );
	 resp = result->array.uc[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p + (int)*im2p) <= UCHAR_MAX ? temp : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *im1p, *im2p;
	 char *resp, *resend;

	 result->type = HOR_CHAR;
	 hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
	 resp = result->array.c[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p + (int)*im2p) < CHAR_MIN  ? CHAR_MIN
	            : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
	 hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
	 resp = result->array.us[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p + (int)*im2p) <= USHRT_MAX ? temp :USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
	 hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
	 resp = result->array.s[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p + (int)*im2p) < SHRT_MIN ? SHRT_MIN
	            : (temp <= SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
	 hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
	 resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = *im1p + *im2p;
      }
      break;

      default:
      hor_error ( "illegal image type (add_image_data)", HOR_FATAL );
      break;
   }
}

/* subtract_image_data(): assumes images are same size and type */
static void subtract_image_data ( Hor_Image *image1,
				  Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;
   int temp;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
	 hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
	 resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       *resp++ = *im1p++ - *im2p++ )
	    ;
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 u_char  *resp, *resend;

	 result->type = HOR_U_CHAR;
	 hor_alloc_image_data ( width, height, HOR_U_CHAR, &result->array );
	 resp = result->array.uc[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < 0 ? 0 :
	            (temp <= UCHAR_MAX ? temp : UCHAR_MAX);
      }
      break;

      case HOR_CHAR:
      {
	 char *im1p, *im2p;
	 char *resp, *resend;

	 result->type = HOR_CHAR;
	 hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
	 resp = result->array.c[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < CHAR_MIN ? CHAR_MIN
	            : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
	 hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
	 resp = result->array.us[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < 0 ? 0 :
	            (temp <= USHRT_MAX ? temp : USHRT_MAX);
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
	 hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
	 resp = result->array.s[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < SHRT_MIN ? SHRT_MIN
	            : (temp <= SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      default:
      hor_error ( "illegal image type (subtract_image_data)", HOR_FATAL );
      break;
   }
}

/* subtract_signed_image_data(): assumes images are same size and type */
static void subtract_signed_image_data ( Hor_Image *image1,
					 Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;
   int temp;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 char   *resp, *resend;

	 result->type = HOR_CHAR;
	 hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
	 resp = result->array.c[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < CHAR_MIN ? CHAR_MIN :
	            (temp <= CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 short   *resp, *resend;

	 result->type = HOR_SHORT;
	 hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
	 resp = result->array.s[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p - (int)*im2p) < SHRT_MIN ? SHRT_MIN :
	            (temp <= SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      default:
      hor_error ( "illegal image type (subtract_signed_image_data)", HOR_FATAL );
      break;
   }
}

/* multiply_image_data(): assumes images are same size and type */
static void multiply_image_data ( Hor_Image *image1,
				  Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;
   int temp;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 u_char *resp, *resend;

	 result->type = HOR_U_CHAR;
	 hor_alloc_image_data ( width, height, HOR_U_CHAR, &result->array );
	 resp = result->array.uc[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p * (int)*im2p) <= UCHAR_MAX ? temp : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *im1p, *im2p;
	 char *resp, *resend;

	 result->type = HOR_CHAR;
	 hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
	 resp = result->array.c[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p * (int)*im2p) < CHAR_MIN ? CHAR_MIN
	            : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
	 hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
	 resp = result->array.us[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p * (int)*im2p) <= USHRT_MAX ? temp :USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
	 hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
	 resp = result->array.s[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (temp = (int)*im1p * (int)*im2p) < SHRT_MIN ? SHRT_MIN
	            : (temp <= SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
	 hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
	 resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = *im1p * *im2p;
      }
      break;

      default:
      hor_error ( "illegal image type (multiply_image_data)", HOR_FATAL );
      break;
   }
}

/* multiply_double_image_data(): assumes images are same size and type */
static void multiply_double_image_data ( Hor_Image *image1,
					 Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char  *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
	 hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
	 resp = result->array.us[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (u_short)*im1p * (u_short)*im2p;
      }
      break;

      case HOR_CHAR:
      {
	 char  *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
	 hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
	 resp = result->array.s[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (short)*im1p * (short)*im2p;
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_int   *resp, *resend;

	 result->type = HOR_U_INT;
	 hor_alloc_image_data ( width, height, HOR_U_INT, &result->array );
	 resp = result->array.ui[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (u_int)*im1p * (u_int)*im2p;
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 int   *resp, *resend;

	 result->type = HOR_INT;
	 hor_alloc_image_data ( width, height, HOR_INT, &result->array );
	 resp = result->array.i[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (int)*im1p * (int)*im2p;
      }
      break;

      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
	 hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
	 resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = *im1p * *im2p;
      }
      break;

      default:
      hor_error ( "illegal image type (multiply_double_image_data)", HOR_FATAL );
      break;
   }
}

/* divide_image_data(): assumes images are same size and type */
static void divide_image_data ( Hor_Image *image1,
			        Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
         hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
         resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       *resp++ = *im1p++/(*im2p++) )
	    ;
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 u_char *resp, *resend;

	 result->type = HOR_U_CHAR;
         hor_alloc_image_data ( width, height, HOR_U_CHAR, &result->array );
         resp = result->array.uc[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (*im2p != 0) ? *im1p/(*im2p) : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *im1p, *im2p;
	 char *resp, *resend;

	 result->type = HOR_CHAR;
         hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
         resp = result->array.c[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (*im2p != 0) ? *im1p/(*im2p) : CHAR_MAX; /* not very
							   satisfactory */
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
         hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
         resp = result->array.us[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (*im2p != 0) ? *im1p/(*im2p) : USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
         hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
         resp = result->array.s[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (*im2p != 0) ? *im1p/(*im2p) : SHRT_MAX; /* not very
							     satisfactory */
      }
      break;

      default:
      hor_error ( "illegal image type (divide_image_data)", HOR_FATAL );
      break;
   }
}

/* average_image_data(): assumes images are same size and type */
static void average_image_data ( Hor_Image *image1,
				 Hor_Image *image2, Hor_Image *result )
{
   int width  = image1->width;
   int height = image1->height;

   result->width  = width;
   result->height = height;
   switch ( image1->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *im1p, *im2p;
	 u_char *resp, *resend;

	 result->type = HOR_U_CHAR;
         hor_alloc_image_data ( width, height, HOR_U_CHAR, &result->array );
         resp = result->array.uc[0];
	 for ( im1p = image1->array.uc[0], im2p = image2->array.uc[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (u_char) (((int)*im1p + (int)*im2p)/2);
      }
      break;

      case HOR_CHAR:
      {
	 char *im1p, *im2p;
	 char *resp, *resend;

	 result->type = HOR_CHAR;
         hor_alloc_image_data ( width, height, HOR_CHAR, &result->array );
         resp = result->array.c[0];
	 for ( im1p = image1->array.c[0], im2p = image2->array.c[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (char) (((int)*im1p + (int)*im2p)/2);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *im1p, *im2p;
	 u_short *resp, *resend;

	 result->type = HOR_U_SHORT;
         hor_alloc_image_data ( width, height, HOR_U_SHORT, &result->array );
         resp = result->array.us[0];
	 for ( im1p = image1->array.us[0], im2p = image2->array.us[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (u_short) (((int)*im1p + (int)*im2p)/2);
      }
      break;

      case HOR_SHORT:
      {
	 short *im1p, *im2p;
	 short *resp, *resend;

	 result->type = HOR_SHORT;
         hor_alloc_image_data ( width, height, HOR_SHORT, &result->array );
         resp = result->array.s[0];
	 for ( im1p = image1->array.s[0], im2p = image2->array.s[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = (short) (((int)*im1p + (int)*im2p)/2);
      }
      break;

      case HOR_FLOAT:
      {
	 float *im1p, *im2p;
	 float *resp, *resend;

	 result->type = HOR_FLOAT;
         hor_alloc_image_data ( width, height, HOR_FLOAT, &result->array );
         resp = result->array.f[0];
	 for ( im1p = image1->array.f[0], im2p = image2->array.f[0],
	       resend = resp + width*height; resp != resend;
	       im1p++, im2p++, resp++ )
	    *resp = 0.5F*(*im1p + *im2p);
      }
      break;

      default:
      hor_error ( "illegal image type (average_image_data)", HOR_FATAL );
      break;
   }
}

/** these functions exported **/

/*******************
*   Hor_Sub_Image *@hor_extract_from_image ( Hor_Image *image, int c0, int r0,
*                                           int width, int height )
*
*   Copies part of image.
********************/
Hor_Sub_Image *hor_extract_from_image ( Hor_Image *image,
				        int c0, int r0, int width, int height )
{
   int            imwidth  = image->width;
   int            imheight = image->height;
   Hor_Sub_Image *result;
   int            subr, imr;

   if ( c0 < 0 || r0 < 0 || width < 0 || height < 0 ||
        c0 + width > imwidth || r0 + height > imheight )
   {
      hor_errno = HOR_IMAGE_INVALID_IMAGE_REGION;
      return NULL;
   }

   result = hor_alloc_sub_image ( c0, r0, width, height, image->type, NULL );

   switch ( image->type )
   {
      case HOR_FLOAT:
      {
	 float **subarr = result->image.array.f;
	 float **imarr  = image->array.f;
	 float  *subp, *subend, *imp;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **subarr = result->image.array.uc;
	 u_char **imarr  = image->array.uc;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
#ifdef HOR_TRANSPUTER
	    _memcpy ( subarr[subr], imarr[imr] + c0, width*sizeof(HOR_U_CHAR) );
#else
	 {
	    u_char *subp, *subend, *imp;

	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
	 }
#endif
      }
      break;

      case HOR_CHAR:
      {
	 char **subarr = result->image.array.c;
	 char **imarr  = image->array.c;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
#ifdef HOR_TRANSPUTER
	    _memcpy ( subarr[subr], imarr[imr] + c0, width*sizeof(HOR_CHAR) );
#else
	 {
	    char *subp, *subend, *imp;

	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
	 }
#endif
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **subarr = result->image.array.us;
	 u_short **imarr  = image->array.us;
	 u_short *subp, *subend, *imp;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
      }
      break;

      case HOR_SHORT:
      {
	 short **subarr = result->image.array.s;
	 short **imarr  = image->array.s;
	 short *subp, *subend, *imp;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
      }
      break;

      case HOR_U_INT:
      {
	 u_int **subarr = result->image.array.ui;
	 u_int **imarr  = image->array.ui;
	 u_int *subp, *subend, *imp;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
      }
      break;

      case HOR_INT:
      {
	 int **subarr = result->image.array.i;
	 int **imarr  = image->array.i;
	 int *subp, *subend, *imp;

	 for ( subr = 0, imr = r0; subr < height; subr++, imr++ )
	    for ( subp = subarr[subr], subend = subarr[subr] + width,
		  imp = imarr[imr] + c0; subp != subend; *subp++ = *imp++ )
	       ;
      }
      break;

      default:
      hor_error ( "illegal image type (hor_extract_from_image)", HOR_FATAL );
      break;
   }

   return result;
}

/*******************
*   Hor_Sub_Image *@hor_extract_from_sub_image ( Hor_Sub_Image *sub_image,
*                                               int ext_c0,    int ext_r0,
*                                               int ext_width, int ext_height )
*
*   Extracts part of sub-image lying within given dimensions.
********************/
Hor_Sub_Image *hor_extract_from_sub_image ( Hor_Sub_Image *sub_image,
					    int ext_c0,    int ext_r0,
					    int ext_width, int ext_height )
{
   Hor_Image     *image = &(sub_image->image);
   int            sim_c0, sim_r0, sim_width, sim_height;
   int            res_c0, res_r0, res_width, res_height;
   Hor_Sub_Image *result;

   sim_c0 = sub_image->c0; sim_width  = image->width; 
   sim_r0 = sub_image->r0; sim_height = image->height; 
   
   res_c0 = hor_imax ( sub_image->c0, ext_c0 );
   res_r0 = hor_imax ( sub_image->r0, ext_r0 );
   res_width  = hor_imin ( sim_c0 + sim_width,  ext_c0 + ext_width )  - res_c0;
   res_height = hor_imin ( sim_r0 + sim_height, ext_r0 + ext_height ) - res_r0;
   if ( res_width < 0 || res_height < 0 ) return NULL;

   result = hor_extract_from_image ( image, res_c0 - sim_c0, res_r0 - sim_r0,
				 res_width, res_height );
   result->c0 = res_c0; result->r0 = res_r0;
   return result;
}

/*******************
*   void @hor_insert_sub_image_in_image ( Hor_Sub_Image *source_image,
*                                        Hor_Image     *dest_image )
*   void @hor_insert_image_in_image ( Hor_Image *source_image,
*                                    Hor_Image *dest_image,
*                                    int c_offset, int r_offset )
*
*   hor_insert_sub_image_in_image() writes the contents of source_image into
*   dest_image, where dest_image encloses source_image, taking the sub-image
*   offsets into account. hor_insert_image_in_image() uses c_offset and
*   r_offset as offsets of source_image relative to dest_image.
********************/
void hor_insert_image_in_image ( Hor_Image *source_image,
				 Hor_Image *dest_image,
				 int c_offset, int r_offset )
{
   int r;
   int s_width  = source_image->width;
   int s_height = source_image->height;
   int d_width  = dest_image->width;
   int d_height = dest_image->height;

   /* check that offset source image lies within destination image */
   if ( c_offset < 0 || r_offset < 0 ||
        c_offset + s_width  > d_width || r_offset + s_height > d_height )
      hor_error ( "image 2 not inside image 1 (hor_insert_image_in_image)",
		  HOR_FATAL);

   if ( source_image->type != dest_image->type )
      hor_error ( "inconsistent types (hor_insert_image_in_image)", HOR_FATAL );

   switch ( source_image->type )
   {
      case HOR_FLOAT:
      {
	 float **arr_s = source_image->array.f;
	 float **arr_d =   dest_image->array.f;

#ifdef HOR_TRANSPUTER
	 hor_block_move_2D ( s_width*sizeof(float), d_width*sizeof(float),
			     s_height, arr_s[0], arr_d[r_offset] + c_offset,
			     s_width*sizeof(float) );
#else
	 float *sp, *spend, *dp;

	 for ( r = 0; r < s_height; r++ )
	    for ( sp = arr_s[r], spend = sp + s_width,
		  dp = arr_d[r+r_offset] + c_offset; sp != spend;
		  *dp++ = *sp++ )
	       ;
#endif
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **arr_s = source_image->array.uc;
	 u_char **arr_d =   dest_image->array.uc;

#ifdef HOR_TRANSPUTER
	 hor_block_move_2D ( s_width*sizeof(u_char), d_width*sizeof(u_char),
			     s_height, arr_s[0], arr_d[r_offset] + c_offset,
			     s_width*sizeof(u_char) );
#else
	 for ( r = 0; r < s_height; r++ )
	    memcpy ( arr_d[r+r_offset] + c_offset, arr_s[r], s_width );
#endif
      }
      break;

      case HOR_INT:
      {
	 int **arr_s = source_image->array.i;
	 int **arr_d =   dest_image->array.i;

#ifdef HOR_TRANSPUTER
	 hor_block_move_2D ( s_width*sizeof(int), d_width*sizeof(int),
			     s_height, arr_s[0], arr_d[r_offset] + c_offset,
			     s_width*sizeof(int) );
#else
	 int *sp, *spend, *dp;

	 for ( r = 0; r < s_height; r++ )
	    for ( sp = arr_s[r], spend = sp + s_width,
		  dp = arr_d[r+r_offset] + c_offset; sp != spend;
		  *dp++ = *sp++ )
	       ;
#endif
      }
      break;

      case HOR_POINTER:
      {
	 void ***arr_s = source_image->array.p;
	 void ***arr_d =   dest_image->array.p;

#ifdef HOR_TRANSPUTER
	 hor_block_move_2D ( s_width*sizeof(void *), d_width*sizeof(void *),
			     s_height, arr_s[0], arr_d[r_offset] + c_offset,
			     s_width*sizeof(void *) );
#else
	 void **sp, **spend, **dp;

	 for ( r = 0; r < s_height; r++ )
	    for ( sp = arr_s[r], spend = sp + s_width,
		  dp = arr_d[r+r_offset] + c_offset; sp != spend;
		  *dp++ = *sp++ )
	       ;
#endif
      }
      break;

      case HOR_BIT:
      {
	 hor_bit **arr_s = source_image->array.b;
	 hor_bit **arr_d = dest_image->array.b;

	 for ( r = 0; r < s_height; r++ )
	    hor_insert_bit_array ( arr_d[r+r_offset], arr_s[r], s_width,
				   c_offset);
      }
      break;

      default:
      hor_error ( "illegal image type (hor_insert_image_in_image)", HOR_FATAL );
      break;
   }
}

/*******************
*   void @hor_add_constant_to_image        ( Hor_Image *image,
*                                           Hor_Impixel pixel_value )
*   void @hor_subtract_constant_from_image ( Hor_Image *image,
*                                           Hor_Impixel pixel_value )
*   void @hor_multiply_image_by_constant   ( Hor_Image *image,
*                                           Hor_Impixel pixel_value )
*   void @hor_divide_image_by_constant     ( Hor_Image *image,
*                                           Hor_Impixel pixel_value )
*   void @hor_add_constant_to_sub_image        ( Hor_Sub_Image *image,
*                                               Hor_Impixel pixel_value )
*   void @hor_subtract_constant_from_sub_image ( Hor_Sub_Image *image,
*                                               Hor_Impixel pixel_value )
*   void @hor_multiply_sub_image_by_constant   ( Hor_Sub_Image *image,
*                                               Hor_Impixel pixel_value )
*   void @hor_divide_sub_image_by_constant     ( Hor_Sub_Image *image,
*                                               Hor_Impixel pixel_value )
*
*   Perform simple arithmetical operations on a (sub-)image, i.e. adding to,
*   subtracting from, multiplying or divising by a constant at each pixel.
********************/
void hor_add_constant_to_image ( Hor_Image *image, Hor_Impixel pixel_value )
{
   int size = image->width*image->height;
   int temp;

   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *imp, *imend;
	 int     val = (int) pixel_value.uc;

	 for ( imp = image->array.uc[0], imend = image->array.uc[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp + val) <= UCHAR_MAX ? temp : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *imp, *imend;
	 int   val = (int) pixel_value.c;

	 for ( imp = image->array.c[0], imend = image->array.c[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp + val) < CHAR_MIN ? CHAR_MIN
	           : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp, *imend;
	 int      val = (int) pixel_value.us;

	 for ( imp = image->array.us[0], imend = image->array.us[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp + val) <= USHRT_MAX ? temp : USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *imp, *imend;
	 int    val = (int) pixel_value.s;

	 for ( imp = image->array.s[0], imend = image->array.s[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp + val) < SHRT_MIN ? SHRT_MIN
	           : (temp < SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      case HOR_FLOAT:
      {
	 float *imp, *imend;
	 float  val = pixel_value.f;

	 for ( imp = image->array.f[0], imend = image->array.f[0] + size;
	       imp != imend; imp++ )
	    *imp += val;
      }
      break;

      default:
      hor_error ( "illegal image type (hor_add_constant_to_image)", HOR_FATAL );
      break;
   }
}

void hor_subtract_constant_from_image ( Hor_Image *image,
				        Hor_Impixel pixel_value )
{
   int size = image->width*image->height;
   int temp;

   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *imp, *imend;
	 int     val = (int) pixel_value.uc;

	 for ( imp = image->array.uc[0], imend = image->array.uc[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp - val) >= 0 ? temp : 0;
      }
      break;

      case HOR_CHAR:
      {
	 char *imp, *imend;
	 int   val = (int) pixel_value.c;

	 for ( imp = image->array.c[0], imend = image->array.c[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp - val) < CHAR_MIN  ? CHAR_MIN
	           : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp, *imend;
	 int      val = (int) pixel_value.us;

	 for ( imp = image->array.us[0], imend = image->array.us[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp - val) >= 0 ? temp : 0;
      }
      break;

      case HOR_SHORT:
      {
	 short *imp, *imend;
	 int    val = (int) pixel_value.s;

	 for ( imp = image->array.s[0], imend = image->array.s[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp - val) < SHRT_MIN ? SHRT_MIN
	           : (temp < SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      case HOR_FLOAT:
      {
	 float *imp, *imend;
	 float  val = pixel_value.f;

	 for ( imp = image->array.f[0], imend = image->array.f[0] + size;
	       imp != imend; imp++ )
	    *imp -= val;
      }
      break;

      default:
      hor_error ( "illegal image type (hor_subtract_constant_from_image)",
		  HOR_FATAL );
      break;
   }
}

void hor_multiply_image_by_constant ( Hor_Image *image,
				      Hor_Impixel pixel_value )
{
   int size = image->width*image->height;
   int temp;

   switch ( image->type )
   {
      case HOR_FLOAT:
      {
	 float *imp, *imend;
	 float  val = pixel_value.f;

	 for ( imp = image->array.f[0], imend = image->array.f[0] + size;
	       imp != imend; *imp++ *= val )
	    ;
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char *imp, *imend;
	 int     val = (int) pixel_value.uc;

	 for ( imp = image->array.uc[0], imend = image->array.uc[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp * val) <= UCHAR_MAX ? temp : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *imp, *imend;
	 int   val = (int) pixel_value.c;

	 for ( imp = image->array.c[0], imend = image->array.c[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp * val) < CHAR_MIN ? CHAR_MIN
	           : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp, *imend;
	 int      val = (int) pixel_value.us;

	 for ( imp = image->array.us[0], imend = image->array.us[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp * val) <= USHRT_MAX ? temp : USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *imp, *imend;
	 int    val = (int) pixel_value.s;

	 for ( imp = image->array.s[0], imend = image->array.s[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp * val) < SHRT_MIN ? SHRT_MIN
	           : (temp < SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      default:
      hor_error ( "illegal image type (hor_multiply_image_by_constant)",
		  HOR_FATAL );
      break;
   }
}

void hor_divide_image_by_constant ( Hor_Image *image, Hor_Impixel pixel_value )
{
   int size = image->width*image->height;
   int temp;

   switch ( image->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *imp, *imend;
	 int     val = (int) pixel_value.uc;

	 if ( val == 0 )
	    hor_error ( "division by zero (hor_divide_image_by_constant)",
		        HOR_FATAL );

	 for ( imp = image->array.uc[0], imend = image->array.uc[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp / val) <= UCHAR_MAX ? temp : UCHAR_MAX;
      }
      break;

      case HOR_CHAR:
      {
	 char *imp, *imend;
	 int   val = (int) pixel_value.c;

	 if ( val == 0 )
	    hor_error ( "division by zero (hor_divide_image_by_constant)",
		        HOR_FATAL );

	 for ( imp = image->array.c[0], imend = image->array.c[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp / val) < CHAR_MIN ? CHAR_MIN
	           : (temp < CHAR_MAX ? temp : CHAR_MAX);
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp, *imend;
	 int      val = (int) pixel_value.us;

	 if ( val == 0 )
	    hor_error ( "division by zero (hor_divide_image_by_constant)",
		        HOR_FATAL );

	 for ( imp = image->array.us[0], imend = image->array.us[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp / val) <= USHRT_MAX ? temp : USHRT_MAX;
      }
      break;

      case HOR_SHORT:
      {
	 short *imp, *imend;
	 int    val = (int) pixel_value.s;

	 if ( val == 0 )
	    hor_error ( "division by zero (hor_divide_image_by_constant)",
		        HOR_FATAL );

	 for ( imp = image->array.s[0], imend = image->array.s[0] + size;
	       imp != imend; imp++ )
	    *imp = (temp = (int)*imp / val) < SHRT_MIN ? SHRT_MIN
	           : (temp < SHRT_MAX ? temp : SHRT_MAX);
      }
      break;

      case HOR_FLOAT:
      {
	 float *imp, *imend;
	 float  val = pixel_value.f;

	 if ( val == 0.0 )
	    hor_error ( "division by zero (hor_divide_image_by_constant)",
		        HOR_FATAL );

	 for ( imp = image->array.f[0], imend = image->array.f[0] + size;
	       imp != imend; imp++ )
	    *imp /= val;
      }
      break;

      default:
      hor_error ( "illegal image type (hor_divide_image_by_constant)", HOR_FATAL );
      break;
   }
}

/*******************
*   Hor_Image *@hor_add_images            (Hor_Image *image1,Hor_Image *image2)
*   Hor_Image *@hor_subtract_images       (Hor_Image *image1,Hor_Image *image2)
*   Hor_Image *@hor_subtract_signed_images(Hor_Image *image1,Hor_Image *image2)
*   Hor_Image *@hor_multiply_images       (Hor_Image *image1,Hor_Image *image2)
*   Hor_Image *@hor_multiply_double_images(Hor_Image *image1,Hor_Image *image2)
*   Hor_Image *@hor_divide_images         (Hor_Image *image1,Hor_Image *image2)
*   Hor_Sub_Image *@hor_add_sub_images             ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*   Hor_Sub_Image *@hor_subtract_sub_images        ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*   Hor_Sub_Image *@hor_subtract_signed_sub_images ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*   Hor_Sub_Image *@hor_multiply_sub_images        ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*   Hor_Sub_Image *@hor_multiply_double_sub_images ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*   Hor_Sub_Image *@hor_divide_sub_images          ( Hor_Sub_Image *sub_image1,
*                                                   Hor_Sub_Image *sub_image2 )
*
*   Perform simple arithmetic operation on each pixel of the input
*   (sub-)images and write result to a new (sub-)image, which is returned.
*   hor_subtract_signed_images() and hor_subtract_signed_sub_images() produce a
*   result in a signed type (e.g. char, short) from the result of subtracting
*   two unsigned type images (e.g. unsigned char, unsigned short).
*   hor_multiply_double_images() and hor_multiply_double_sub_images() double
*   the precision of the result, producing e.g. a short image from multiplying
*   two char images.
********************/
Hor_Image *hor_add_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_add_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   add_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_add_sub_images ( Hor_Sub_Image *sub_image1,
				    Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_add_sub_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      add_image_data ( &(sub_image1->image), &(sub_image2->image),
		       &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
				       sub_image2->c0, sub_image2->r0,
				       sub_image2->image.width,
				       sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
				       sub_image1->c0, sub_image1->r0,
				       sub_image1->image.width,
				       sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      add_image_data ( &(temp1->image), &(temp2->image), &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

Hor_Image *hor_subtract_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_subtract_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   subtract_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_subtract_sub_images ( Hor_Sub_Image *sub_image1,
					 Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_subtract_sub_images)",
		  HOR_FATAL );

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      subtract_image_data ( &(sub_image1->image), &(sub_image2->image),
			    &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      subtract_image_data ( &(temp1->image), &(temp2->image),
			    &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

Hor_Image *hor_subtract_signed_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_subtract_signed_images)",
		  HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   subtract_signed_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_subtract_signed_sub_images ( Hor_Sub_Image *sub_image1,
					        Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_subtract_signed_sub_images)",
		  HOR_FATAL);

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      subtract_signed_image_data ( &(sub_image1->image), &(sub_image2->image),
				   &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      subtract_signed_image_data ( &(temp1->image), &(temp2->image),
				   &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

Hor_Image *hor_multiply_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_multiply_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   multiply_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_multiply_sub_images ( Hor_Sub_Image *sub_image1,
					 Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_multiply_sub_images)",
		  HOR_FATAL );

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      multiply_image_data ( &(sub_image1->image), &(sub_image2->image),
			    &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      multiply_image_data ( &(temp1->image), &(temp2->image),
			    &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

Hor_Image *hor_multiply_double_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_multiply_double_images)", HOR_FATAL);

   result = hor_malloc_type ( Hor_Image );
   multiply_double_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_multiply_double_sub_images ( Hor_Sub_Image *sub_image1,
					        Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_multiply_double_sub_images)",
		  HOR_FATAL );

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      multiply_double_image_data ( &(sub_image1->image), &(sub_image2->image),
				   &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      multiply_double_image_data ( &(temp1->image), &(temp2->image),
				   &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

Hor_Image *hor_divide_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_divide_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   divide_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_divide_sub_images ( Hor_Sub_Image *sub_image1,
				       Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_divide_sub_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      divide_image_data ( &(sub_image1->image), &(sub_image2->image),
			  &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      divide_image_data ( &(temp1->image), &(temp2->image), &(result->image) );
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}

/*******************
*   void @hor_fill_image_with_constant     ( Hor_Image  *image,
*                                           Hor_Impixel pixel_value)
*   void @hor_fill_sub_image_with_constant ( Hor_Sub_Image *image,
*                                           Hor_Impixel    pixel_value)
*
*   Fills (sub-)image with a constant value.
********************/
void hor_fill_image_with_constant ( Hor_Image *image, Hor_Impixel pixel_value )
{
   int size = image->width*image->height;

   switch ( image->type )
   {
      case HOR_BIT:
      {
	 hor_bit **imarray = image->array.b, *imp, *imend;
	 int       row, height = image->height, row_size=(image->width+31)/32;

	 if ( pixel_value.b == 0)
	    for ( row = 0; row < height; row++ )
	       for ( imp = imarray[row], imend = imarray[row] + row_size;
		     imp != imend; *imp++ = 0 )
		  ;
	 else /* pixel_value.b == 1 or bigger */
	    for ( row = 0; row < height; row++ )
	       for ( imp = imarray[row], imend = imarray[row] + row_size;
		     imp != imend; *imp++ = HOR_MAX_BIT_VAL )
		  ;

	    break;
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char val = pixel_value.uc;
#ifdef HOR_TRANSPUTER
	 memset ( image->array.uc[0], val, size );
#else
	 u_char *imp, *imend;

	 for ( imp = image->array.uc[0], imend = image->array.uc[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
#endif
      }
      break;

      case HOR_CHAR:
      {
	 char val = pixel_value.c;
#ifdef HOR_TRANSPUTER
	 memset ( image->array.c[0], val, size );
#else
	 char *imp, *imend;

	 for ( imp = image->array.c[0], imend = image->array.c[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
#endif
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short *imp, *imend, val = pixel_value.us;

	 for ( imp = image->array.us[0], imend = image->array.us[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      case HOR_SHORT:
      {
	 short *imp, *imend, val = pixel_value.s;

	 for ( imp = image->array.s[0], imend = image->array.s[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      case HOR_U_INT:
      {
	 u_int *imp, *imend, val = pixel_value.ui;

	 for ( imp = image->array.ui[0], imend = image->array.ui[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      case HOR_INT:
      {
	 int *imp, *imend, val = pixel_value.i;

	 for ( imp = image->array.i[0], imend = image->array.i[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      case HOR_FLOAT:
      {
	 float *imp, *imend, val = pixel_value.f;

	 for ( imp = image->array.f[0], imend = image->array.f[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      case HOR_POINTER:
      {
	 void **imp, **imend, *val = pixel_value.p;

	 for ( imp = image->array.p[0], imend = image->array.p[0] + size;
	       imp != imend; imp++ )
	    *imp = val;
      }
      break;

      default:
      hor_error ( "illegal image type (hor_fill_image_with_constant)", HOR_FATAL );
      break;
   }
}

/*******************
*   Hor_Image *@hor_average_images ( Hor_Image *image1, Hor_Image *image2 )
*   Hor_Sub_Image *@hor_average_sub_images ( Hor_Sub_Image *sub_image1,
*                                           Hor_Sub_Image *sub_image2 )
*
*   Averages two (sub-)images, returning result in new image.
********************/
Hor_Image *hor_average_images ( Hor_Image *image1, Hor_Image *image2 )
{
   Hor_Image *result;

   if ( !hor_same_type_dims_images ( image1, image2, NULL ) )
      hor_error ( "images not compatible (hor_average_images)", HOR_FATAL );

   result = hor_malloc_type ( Hor_Image );
   average_image_data ( image1, image2, result );
   return result;
}

Hor_Sub_Image *hor_average_sub_images ( Hor_Sub_Image *sub_image1,
				        Hor_Sub_Image *sub_image2 )
{
   Hor_Sub_Image *result;

   if ( !compatible_sub_images ( sub_image1, sub_image2 ) )
      hor_error ( "sub-images not compatible (hor_average_sub_images)", HOR_FATAL);

   result = hor_malloc_type ( Hor_Sub_Image );
   if ( hor_same_type_dims_sub_images ( sub_image1, sub_image2, NULL ) )
   {
      result->c0 = sub_image1->c0; result->r0 = sub_image1->r0;
      average_image_data ( &(sub_image1->image), &(sub_image2->image),
			   &(result->image) );
   }
   else
   {
      Hor_Sub_Image *temp1, *temp2;

      temp1 = hor_extract_from_sub_image ( sub_image1,
					   sub_image2->c0, sub_image2->r0,
					   sub_image2->image.width,
					   sub_image2->image.height );
      temp2 = hor_extract_from_sub_image ( sub_image2,
					   sub_image1->c0, sub_image1->r0,
					   sub_image1->image.width,
					   sub_image1->image.height );
      result->c0 = temp1->c0; result->r0 = temp1->r0;
      average_image_data ( &(temp1->image), &(temp2->image), &(result->image));
      hor_free_sub_images ( temp2, temp1, NULL );
   }

   return result;
}
