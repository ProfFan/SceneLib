/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   Hor_Image     *@hor_copy_image     ( Hor_Image     *image )
*   Hor_Sub_Image *@hor_copy_sub_image ( Hor_Sub_Image *image )
*
*   Return copies of input (sub-)image.
********************/

/*******************
*   void @hor_convert_image_data ( Hor_Image *image, Hor_Image *result )
*
*   Copies image data from image to result, converting between types if
*   necessary. Assumes result image is already allocated.
********************/
void hor_convert_image_data ( Hor_Image *image, Hor_Image *result )
{
   int width  = image->width;
   int height = image->height;

   switch ( image->type )
   {
      case HOR_BIT:
      {
	 hor_bit **imarr = image->array.b, *imp;

	 switch ( result->type )
	 {
	    case HOR_BIT:
	    {
	       hor_bit **resarr = result->array.b;
	       int       row;

	       for ( row = 0; row < height; row++ )
		  hor_copy_bit_array ( imarr[row], resarr[row], width );
	    }
	    break;

	    case HOR_U_CHAR:
	    {
	       u_char **resarr = result->array.uc, *resp;
	       int      row, col;

	       for ( row = 0; row < height; row++ )
	       {
		  imp = imarr[row];
		  for ( col = 0, resp = resarr[row]; col < width;
		        col++, resp++ )
		     if ( hor_get_bit ( imp, col ) ) *resp = UCHAR_MAX;
		     else                            *resp = 0;
	       }
	    }
            break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC **resarr = result->array.cuc, *resp;
	       int      row, col;

	       for ( row = 0; row < height; row++ )
	       {
		  imp = imarr[row];
		  for ( col = 0, resp = resarr[row]; col < width;
		        col++, resp++ )
		     if ( hor_get_bit ( imp, col ) )
		     { resp->r = UCHAR_MAX;
		       resp->g = UCHAR_MAX;
		       resp->b = UCHAR_MAX; }
		     else
		     { resp->r = 0;
		       resp->g = 0;
		       resp->b = 0; }
	       }
	    }
            break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_U_CHAR:
      {
	 u_char **imarr = image->array.uc, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.uc[0], imarr[0],
		      width*height*sizeof(u_char) );
#else
	    memcpy ( result->array.uc[0], imarr[0],
		     width*height*sizeof(u_char) );
#endif
	    break;

	    case HOR_CHAR:
	    {
	       char *resp = result->array.c[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= CHAR_MAX ? *imp : CHAR_MAX;
	    }
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_INT:
	    {
	       int *resp = result->array.i[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
 	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp;
		 resp->g = *imp;
		 resp->b = *imp; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp;
		 resp->g = *imp;
		 resp->b = *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_CHAR:
      {
	 char **imarr = image->array.c, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp < 0) ? 0 : *imp;
	    }
	    break;

	    case HOR_CHAR:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.c[0], imarr[0],
		      width*height*sizeof(char) );
#else
	    memcpy ( result->array.c[0], imarr[0],
		     width*height*sizeof(char) );
#endif
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp < 0) ? 0 : (u_short) *imp;
	    }
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (short) *imp;
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp < 0) ? 0 : (u_int) *imp;
	    }
	    break;

	    case HOR_INT:
	    {
	       int *resp = result->array.i[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (int) *imp;
	    }
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = (*imp < 0) ? 0 : *imp;
		 resp->g = (*imp < 0) ? 0 : *imp;
		 resp->b = (*imp < 0) ? 0 : *imp; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = (*imp < 0) ? 0 : (u_int) *imp;
		 resp->g = (*imp < 0) ? 0 : (u_int) *imp;
		 resp->b = (*imp < 0) ? 0 : (u_int) *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_U_SHORT:
      {
	 u_short **imarr = image->array.us, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
	    }
	    break;

	    case HOR_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= CHAR_MAX ? (char) *imp : CHAR_MAX;
	    }
	    break;

	    case HOR_U_SHORT:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.us[0], imarr[0],
		      width*height*sizeof(u_short) );
#else
	    memcpy ( result->array.us[0], imarr[0],
		     width*height*sizeof(u_short) );
#endif
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp > SHRT_MAX ? SHRT_MAX : *imp;
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_INT:
	    {
	       int *resp = result->array.i[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
		 resp->g = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
		 resp->b = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp;
		 resp->g = *imp;
		 resp->b = *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_SHORT:
      {
	 short **imarr = image->array.s, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= 0 && *imp <= UCHAR_MAX) ? (u_char) *imp :
		          (*imp < 0 ? 0 : UCHAR_MAX);
	    }
	    break;

	    case HOR_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= CHAR_MIN && *imp <= CHAR_MAX) ? (char) *imp :
		          (*imp < CHAR_MIN ? CHAR_MIN : CHAR_MAX);
	    }

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0 ? 0 : *imp;
	    }
	    break;

	    case HOR_SHORT:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.s[0], imarr[0],
		      width*height*sizeof(short) );
#else
	    memcpy ( result->array.s[0], imarr[0],
		     width*height*sizeof(short) );
#endif
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0 ? 0 : *imp;
	    }
	    break;

	    case HOR_INT:
	    {
	       int *resp = result->array.i[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;
	       u_char ucres;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { ucres = (*imp >= 0 && *imp <= UCHAR_MAX) ? (u_char) *imp :
		         (*imp < 0 ? 0 : UCHAR_MAX);
		 resp->r = ucres;
		 resp->g = ucres;
		 resp->b = ucres; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		 { resp->r = *imp < 0 ? 0 : *imp;
		   resp->g = *imp < 0 ? 0 : *imp;
		   resp->b = *imp < 0 ? 0 : *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_U_INT:
      {
	 u_int **imarr = image->array.ui, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
	    }
	    break;

	    case HOR_CHAR:
	    {
	       char *resp = result->array.c[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= CHAR_MAX ? (u_char) *imp : CHAR_MAX;
	    }
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= USHRT_MAX ? (u_char) *imp : USHRT_MAX;
	    }
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp <= SHRT_MAX ? (u_char) *imp : SHRT_MAX;
	    }
	    break;

	    case HOR_U_INT:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.ui[0], imarr[0],
		      width*height*sizeof(u_int) );
#else
	    memcpy ( result->array.ui[0], imarr[0],
		     width*height*sizeof(u_int) );
#endif
	    break;

	    case HOR_INT:
	    {
	       int *resp = result->array.i[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp > INT_MAX ? INT_MAX : *imp;
	    }
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
		 resp->g = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX;
		 resp->b = *imp <= UCHAR_MAX ? (u_char) *imp : UCHAR_MAX; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	      Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp;
		 resp->g = *imp;
		 resp->b = *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_INT:
      {
	 int **imarr = image->array.i, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= 0 && *imp <= UCHAR_MAX) ? (u_char) *imp :
		          (*imp < 0 ? 0 : UCHAR_MAX);
	    }
	    break;

	    case HOR_CHAR:
	    {
	       char *resp = result->array.c[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= -0x80 && *imp < 0x80) ? (u_char) *imp :
		          (*imp < -0x80 ? -0x80 : 0x7f);
	    }
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= 0 && *imp < 0x10000) ? (u_short) *imp :
		          (*imp < 0 ? 0 : 0xffff);
	    }
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (*imp >= SHRT_MIN && *imp <= SHRT_MAX) ? (u_short) *imp :
		          (*imp < SHRT_MIN ? SHRT_MIN : SHRT_MAX);
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0 ? 0 : *imp;
	    }
	    break;

	    case HOR_INT:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.i[0], imarr[0],
		      width*height*sizeof(int) );
#else
	    memcpy ( result->array.i[0], imarr[0],
		     width*height*sizeof(int) );
#endif
	    break;

	    case HOR_FLOAT:
	    {
	       float *resp = result->array.f[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (float) *imp;
	    }
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;
	       u_char ucres;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { ucres = (*imp >= 0 && *imp <= UCHAR_MAX) ? (u_char) *imp :
		          (*imp < 0 ? 0 : UCHAR_MAX);
		 resp->r = ucres;
		 resp->g = ucres;
		 resp->b = ucres; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { resp->r = *imp < 0 ? 0 : *imp;
		 resp->g = *imp < 0 ? 0 : *imp;
		 resp->b = *imp < 0 ? 0 : *imp; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_FLOAT:
      {
	 float **imarr = image->array.f, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0.0F ? 0 : (*imp >= ((float)UCHAR_MAX+0.5) ? UCHAR_MAX
					    : (u_char) (*imp + 0.5F) );
	    }
	    break;

	    case HOR_CHAR:
	    {
	       char *resp = result->array.c[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < ((float) CHAR_MIN) ? CHAR_MIN : (*imp >= ((float) CHAR_MAX+0.5) ? CHAR_MAX
					          : (char) (*imp + 0.5F) );
	    }
	    break;

	    case HOR_U_SHORT:
	    {
	       u_short *resp = result->array.us[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0.0F ? 0 : (*imp >= ((float)USHRT_MAX+0.5F) ? USHRT_MAX
					    : (u_short) (*imp + 0.5F) );
	    }
	    break;

	    case HOR_SHORT:
	    {
	       short *resp = result->array.s[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < ((float) SHRT_MIN) ? SHRT_MIN : (*imp >= ((float)SHRT_MAX+0.5F) ? 32767
						      : (short) (*imp + 0.5F) );
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;
	       float  lim = (float) UINT_MAX;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < 0.0F ? 0 : (*imp >= lim ? UINT_MAX
					    : (u_int) (*imp + 0.5F) );
	    }
	    break;
	
	    case HOR_INT:
	    {
	       int  *resp = result->array.i[0], *resend;
	       float lim1 = (float) INT_MIN, lim2 = (float) INT_MAX;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp < lim1 ? INT_MIN : (*imp >= lim2 ? INT_MAX
						       : (int) (*imp + 0.5F) );
	    }
	    break;

	    case HOR_FLOAT:
#ifdef HOR_TRANSPUTER
	    _memcpy ( result->array.f[0], imarr[0],
		      width*height*sizeof(float) );
#else
	    memcpy ( result->array.f[0], imarr[0],
		     width*height*sizeof(float) );
#endif
	    break;

#ifdef HOR_PROVIDE_RGB
	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;
	       u_char ucres;
	       
	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { ucres  = *imp < 0.0F ? 0 : (*imp >= ((float)UCHAR_MAX+0.5) ? UCHAR_MAX
					    : (u_char) (*imp + 0.5F) );
		 resp->r = ucres;
		 resp->g = ucres;
		 resp->b = ucres; }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;
	       float  lim = (float) UINT_MAX;
	       u_int  uires;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { uires = *imp < 0.0F ? 0 : (*imp >= lim ? UINT_MAX
					    : (u_int) (*imp + 0.5F) );
		 resp->r = uires;
		 resp->g = uires;
		 resp->b = uires; }
	    }
	    break;
#endif /* HOR_PROVIDE_RGB */

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
	    break;
	 }
      }
      break;

#ifdef HOR_PROVIDE_RGB
      case HOR_RGB_UC:
      {
	 Hor_RGB_UC **imarr = image->array.cuc, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (imp->r + imp->g + imp->b)/3;
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = (imp->r + imp->g + imp->b)/3;
	    }
	    break;

	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       {
		  resp->r = imp->r;
		  resp->g = imp->g;
		  resp->b = imp->b;
	       }
	    }
	    break;

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)",
		        HOR_FATAL );
	    break;
	 }
      }
      break;

      case HOR_RGB_UI:
      {
	 Hor_RGB_UI **imarr = image->array.cui, *imp;

	 switch ( result->type )
	 {
	    case HOR_U_CHAR:
	    {
	       u_char *resp = result->array.uc[0], *resend;
	       u_int ave;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       { 
		  ave = 
		    (int) ((float) imp->r + (float) imp->g + (float) imp->b)/3;
		  *resp = (ave > UCHAR_MAX) ? UCHAR_MAX : ave;
	       }
	    }
	    break;

	    case HOR_U_INT:
	    {
	       u_int *resp = result->array.ui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		 *resp =
		   (int) ((float) imp->r + (float) imp->g + (float) imp->b)/3;

	    }
	    break;

	    case HOR_RGB_UC:
	    {
	       Hor_RGB_UC *resp = result->array.cuc[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
	       {
		  resp->r = (imp->r > UCHAR_MAX) ? UCHAR_MAX : imp->r;
		  resp->g = (imp->g > UCHAR_MAX) ? UCHAR_MAX : imp->g;
		  resp->b = (imp->b > UCHAR_MAX) ? UCHAR_MAX : imp->b;
	       }
	    }
	    break;

	    case HOR_RGB_UI:
	    {
	       Hor_RGB_UI *resp = result->array.cui[0], *resend;

	       for ( imp = imarr[0], resend = resp + width*height;
		     resp != resend; imp++, resp++ )
		  *resp = *imp;
	    }
	    break;

	    default:
	    hor_error ( "illegal image type (hor_convert_image_data)",
		        HOR_FATAL );
	    break;
	 }
      }
      break;
#endif /* HOR_PROVIDE_RGB */

      default:
      hor_error ( "illegal image type (hor_convert_image_data)", HOR_FATAL );
      break;
   }
}

/*******************
*   Hor_Image *@hor_convert_image ( Hor_Image     *imptr,
*                                  Hor_Image_Type type )
*   Hor_Sub_Image *@hor_convert_sub_image ( Hor_Sub_Image *sub_imptr,
*                                          Hor_Image_Type type )
*
*   Create and return a copy of an image (or sub-image), converting all the
*   image data to the given type (if possible).
********************/
Hor_Image *hor_convert_image ( Hor_Image *imptr, Hor_Image_Type type )
{
   Hor_Image *result;

   result = hor_alloc_image ( imptr->width, imptr->height, type, NULL );
   if ( result == NULL ) return NULL;

   hor_convert_image_data ( imptr, result );
   return result;
}

Hor_Sub_Image *hor_convert_sub_image ( Hor_Sub_Image *sub_imptr,
				       Hor_Image_Type type )
{
   Hor_Sub_Image *result;

   result = hor_alloc_sub_image ( sub_imptr->c0,          sub_imptr->r0,
			      sub_imptr->image.width, sub_imptr->image.height,
			      type, NULL );
   if ( result == NULL ) return NULL;

   hor_convert_image_data ( &(sub_imptr->image), &(result->image) );
   return result;
}
