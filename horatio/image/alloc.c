/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  and Ian Reid (ian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   Hor_Bool @hor_alloc_image_data ( int width, int height,
*                                   Hor_Image_Type type,
*                                   Hor_Imdata    *image_data )
*   void @hor_free_image_data ( Hor_Imdata image_data, int size,
*                              Hor_Image_Type type,
*                              void (*pixel_free_func)(void *) )
*
*   hor_alloc_image_data() allocates space for image data, returning HOR_TRUE
*   on success, HOR_FALSE on failure. hor_free_image_data() frees space
*   associated with an image.
********************/
Hor_Bool hor_alloc_image_data ( int width, int height,
			        Hor_Image_Type type, Hor_Imdata *image_data )
{
   switch ( type )
   {
      int row, row_size;

      case HOR_BIT:
      /* create array of pointers to start of each image row */
      if ( (image_data->b = hor_malloc_ntype ( hor_bit *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      row_size = hor_bit_words(width);
      if ( (image_data->b[0] = hor_alloc_bit_array ( row_size*height*HOR_BITS_IN_WORD ))
	   == NULL)
      { hor_free ( (void *) image_data->b ); image_data->b = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->b[row] = image_data->b[row-1] + row_size;

      break;

      case HOR_U_CHAR:
      /* create array of pointers to start of each image row */
      if ( (image_data->uc = hor_malloc_ntype ( u_char *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->uc[0] = hor_malloc_ntype ( u_char, width*height ))
	   == NULL)
      { hor_free ( (void *) image_data->uc ); image_data->uc = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->uc[row] = image_data->uc[row-1] + width;

      break;

      case HOR_CHAR:
      /* create array of pointers to start of each image row */
      if ( (image_data->c = hor_malloc_ntype ( char *, height)) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ((image_data->c[0] = hor_malloc_ntype ( char, width*height )) == NULL)
      { hor_free ( (void *) image_data->c ); image_data->c = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->c[row] = image_data->c[row-1] + width;

      break;

      case HOR_U_SHORT:
      /* create array of pointers to start of each image row */
      if ( (image_data->us = hor_malloc_ntype ( u_short *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->us[0] = hor_malloc_ntype ( u_short, width*height ))
	   ==NULL)
      { hor_free ( (void *) image_data->us ); image_data->us = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->us[row] = image_data->us[row-1] + width;

      break;

      case HOR_SHORT:
      /* create array of pointers to start of each image row */
      if ( (image_data->s = hor_malloc_ntype ( short *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->s[0] = hor_malloc_ntype ( short, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->s ); image_data->s = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->s[row] = image_data->s[row-1] + width;

      break;

      case HOR_U_INT:
      /* create array of pointers to start of each image row */
      if ( (image_data->ui = hor_malloc_ntype ( u_int *, height)) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->ui[0] = hor_malloc_ntype ( u_int, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->ui ); image_data->ui = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->ui[row] = image_data->ui[row-1] + width;

      break;

      case HOR_INT:
      /* create array of pointers to start of each image row */
      if ( (image_data->i = hor_malloc_ntype ( int *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->i[0] = hor_malloc_ntype ( int, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->i ); image_data->i = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->i[row] = image_data->i[row-1] + width;

      break;

      case HOR_FLOAT:
      /* create array of pointers to start of each image row */
      if ( (image_data->f = hor_malloc_ntype ( float *, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->f[0] = hor_malloc_ntype ( float, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->f ); image_data->f = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->f[row] = image_data->f[row-1] + width;

      break;

#ifdef HOR_PROVIDE_RGB
      case HOR_RGB_UC:
      /* create array of pointers to start of each image row */
      if ( (image_data->cuc = hor_malloc_ntype ( Hor_RGB_UC *, height ))
	  == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->cuc[0] = hor_malloc_ntype ( Hor_RGB_UC, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->cuc ); image_data->cuc = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->cuc[row] = image_data->cuc[row-1] + width;

      break;

      case HOR_RGB_UI:
      /* create array of pointers to start of each image row */
      if ( (image_data->cui = hor_malloc_ntype ( Hor_RGB_UI *, height ))
	  == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->cui[0] = hor_malloc_ntype ( Hor_RGB_UI, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->cui ); image_data->cui = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->cui[row] = image_data->cui[row-1] + width;

      break;
#endif /* HOR_PROVIDE_RGB */
      case HOR_POINTER:
      /* create array of pointers to start of each image row */
      if ( (image_data->p = hor_malloc_ntype ( void **, height )) == NULL )
	 return HOR_FALSE;

      /* allocate space for image */
      if ( (image_data->p[0] = hor_malloc_ntype ( void *, width*height ))
	   == NULL )
      { hor_free ( (void *) image_data->p ); image_data->p = NULL;
	return HOR_FALSE; }

      /* set pointers to start of each image row */
      for ( row = 1; row < height; row++ )
	 image_data->p[row] = image_data->p[row-1] + width;

      break;

      default:
      hor_error ( "invalid image type (hor_alloc_image_data)", HOR_FATAL );
      break;
   }

   return HOR_TRUE;
}

void hor_free_image_data ( Hor_Imdata     image_data,
			   int            size,
			   Hor_Image_Type type,
			   void         (*pixel_free_func)(void *) )
{
   switch ( type )
   {
      case HOR_BIT:
      hor_free ( (void *) image_data.b[0] );
      hor_free ( (void *) image_data.b );
      break;

      case HOR_U_CHAR:
      hor_free ( (void *) image_data.uc[0] );
      hor_free ( (void *) image_data.uc );
      break;

      case HOR_CHAR:
      hor_free ( (void *) image_data.c[0] );
      hor_free ( (void *) image_data.c );
      break;

      case HOR_U_SHORT:
      hor_free ( (void *) image_data.us[0] );
      hor_free ( (void *) image_data.us );
      break;

      case HOR_SHORT:
      hor_free ( (void *) image_data.s[0] );
      hor_free ( (void *) image_data.s );
      break;

      case HOR_U_INT:
      hor_free ( (void *) image_data.ui[0] );
      hor_free ( (void *) image_data.ui );
      break;

      case HOR_INT:
      hor_free ( (void *) image_data.i[0] );
      hor_free ( (void *) image_data.i );
      break;

      case HOR_FLOAT:
      hor_free ( (void *) image_data.f[0] );
      hor_free ( (void *) image_data.f );
      break;

#ifdef HOR_PROVIDE_RGB
      case HOR_RGB_UC:
      hor_free ( (void *) image_data.cuc[0] );
      hor_free ( (void *) image_data.cuc );
      break;

      case HOR_RGB_UI:
      hor_free ( (void *) image_data.cui[0] );
      hor_free ( (void *) image_data.cui );
      break;
#endif /* HOR_PROVIDE_RGB */

      case HOR_POINTER:
      {
	 void **imp, **imend;

	 if ( pixel_free_func != NULL )
	    for ( imp = (void **) image_data.p[0],
		  imend = (void **) image_data.p[0] + size;
		  imp != imend; imp++ )
	       if ( *imp != NULL )
		  pixel_free_func ( *imp );

	 hor_free ( (void *) image_data.p[0] );
	 hor_free ( (void *) image_data.p );
      }
      break;

      default:
      hor_error ( "illegal image type (hor_free_image_data)", HOR_FATAL );
      break;
   }
}

/*******************
*   Hor_Image *@hor_alloc_image ( int width, int height,
*                                Hor_Image_Type type,
*                                void (*pixel_free_func)(void *) )
*   void @hor_free_image  ( Hor_Image *imptr )
*   void @hor_free_images ( Hor_Image *imptr1, ... )
*   Hor_Image *@hor_alloc_image_header ( int width, int height, 
*                                       Hor_Image_Type type,
*                                       void **data)
*   void @hor_free_image_header ( Hor_Image *imptr )
*   Hor_Sub_Image *@hor_alloc_sub_image ( int c0,    int r0,
*                                        int width, int height,
*                                        Hor_Image_Type type,
*                                        void (*pixel_free_func)(void *) )
*   void @hor_free_sub_image  ( Hor_Sub_Image *imptr )
*   void @hor_free_sub_images ( Hor_Sub_Image *imptr1, ... )
*
*   hor_alloc_image() allocates space for and returns an image, or NULL on
*   failure. The dimensions of the image are width & height. hor_free_image()
*   frees space associated with an image, and hor_free_images frees space for a
*   NULL-terminated list of images. hor_alloc_image_header() takes a block of
*   memory already allocated for an image and sets up the header and row
*   pointer info. hor_free_image_header() frees the header and row pointers
*   without freeing the image data. hor_alloc_sub_image(), hor_free_sub_image()
*   and hor_free_sub_images() do the same for sub-images.
********************/
Hor_Image *hor_alloc_image ( int width, int height, Hor_Image_Type type,
			     void (*pixel_free_func)(void *) )
{
   Hor_Image *imptr;

   if ( width < 0 || height < 0 )
      return NULL;

   imptr = hor_malloc_type(Hor_Image);
   if ( imptr == NULL )
   {
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      return NULL;
   }

   imptr->width  = width;
   imptr->height = height;
   imptr->type   = type;
   if ( !hor_alloc_image_data ( width, height, type, &imptr->array ) )
   {
      hor_free ( (void *) imptr );
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      return NULL;
   }

   imptr->pixel_free_func = pixel_free_func;
   return imptr;
}


Hor_Image *hor_alloc_image_header (int width, int height, 
				   Hor_Image_Type type,
				   void **data)
{
   Hor_Image *imptr;
   int row, row_size;

   if ( width < 0 || height < 0 )
      return NULL;

   imptr = hor_malloc_type(Hor_Image);
   if ( imptr == NULL )
   {
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      return NULL;
   }

   imptr->width  = width;
   imptr->height = height;
   imptr->type   = type;

   switch (type) {
      case HOR_BIT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.b = hor_malloc_ntype ( hor_bit *, height ))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  row_size = hor_bit_words(width);
	  imptr->array.b[0] = (hor_bit *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.b[row] = imptr->array.b[row-1] + row_size;

	  break;

      case HOR_U_CHAR:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.uc = hor_malloc_ntype ( u_char *, height ))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.uc[0] = (u_char *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.uc[row] = imptr->array.uc[row-1] + width;

	  break;

      case HOR_CHAR:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.c = hor_malloc_ntype ( char *, height)) == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.c[0] = (char *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.c[row] = imptr->array.c[row-1] + width;

	  break;

      case HOR_U_SHORT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.us = hor_malloc_ntype ( u_short *, height ))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.us[0] = (u_short *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.us[row] = imptr->array.us[row-1] + width;

	  break;

      case HOR_SHORT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.s = hor_malloc_ntype ( short *, height )) == NULL)
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.s[0] = (short *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.s[row] = imptr->array.s[row-1] + width;

	  break;

      case HOR_U_INT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.ui = hor_malloc_ntype ( u_int *, height))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.ui[0] = (u_int *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.ui[row] = imptr->array.ui[row-1] + width;

	  break;

      case HOR_INT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.i = hor_malloc_ntype ( int *, height )) == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.i[0] = (int *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.i[row] = imptr->array.i[row-1] + width;

	  break;

      case HOR_FLOAT:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.f = hor_malloc_ntype ( float *, height )) == NULL)
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.f[0] = (float *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.f[row] = imptr->array.f[row-1] + width;

	  break;

      case HOR_RGB_UC:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.cuc = hor_malloc_ntype ( Hor_RGB_UC *, height))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.cuc[0] = (Hor_RGB_UC *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.cuc[row] = imptr->array.cuc[row-1] + width;

	  break;

      case HOR_RGB_UI:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.cui = hor_malloc_ntype ( Hor_RGB_UI *, height))
	       == NULL )
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.cui[0] = (Hor_RGB_UI *)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.cui[row] = imptr->array.cui[row-1] + width;

	  break;

      case HOR_POINTER:
	  /* create array of pointers to start of each image row */
	  if ( (imptr->array.p = hor_malloc_ntype ( void **, height )) == NULL)
	     return NULL;

	  /* set pointers to start of each image row */
	  imptr->array.p[0] = (void **)data;
	  for ( row = 1; row < height; row++ )
	     imptr->array.p[row] = imptr->array.p[row-1] + width;

	  break;

      default:
          hor_error ( "invalid image type (hor_alloc_image_data)", HOR_FATAL );
          break;
   }

   return imptr;
}


Hor_Sub_Image *hor_alloc_sub_image ( int c0, int r0, int width, int height,
				     Hor_Image_Type type,
				     void (*pixel_free_func)(void *) )
{
   Hor_Sub_Image *imptr;

   if ( width < 0 || height < 0 ) return NULL;

   imptr = hor_malloc_type ( Hor_Sub_Image );
   if ( imptr == NULL )
   {
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      return NULL;
   }

   imptr->c0                    = c0;
   imptr->r0                    = r0;
   imptr->image.width           = width;
   imptr->image.height          = height;
   imptr->image.type            = type;
   if ( !hor_alloc_image_data ( width, height, type, &imptr->image.array ) )
   {
      hor_free ( (void *) imptr );
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      return NULL;
   }

   imptr->image.pixel_free_func = pixel_free_func;
   return imptr;
}

void hor_free_image ( Hor_Image *imptr )
{
   if ( imptr == NULL ) return;

   hor_free_image_data ( imptr->array, imptr->width*imptr->height,
			 imptr->type,  imptr->pixel_free_func );
   hor_free ( (void *) imptr );
}

void hor_free_images ( Hor_Image *imptr1, ... )
{
   Hor_Image  *imptr;
   va_list     ap;

   if ( imptr1 == NULL )
      return;

   hor_free_image ( imptr1 );
   va_start ( ap, imptr1 );
   while(1)
   {
      imptr = va_arg ( ap, Hor_Image * );
      if ( imptr == NULL ) break;

      hor_free_image ( imptr );
   }

   va_end(ap);
}

void hor_free_image_header ( Hor_Image *imptr )
{
   if ( imptr == NULL ) return;

   switch ( imptr->type )
   {
      case HOR_BIT:
         hor_free ( (void *) imptr->array.b );
	 break;

      case HOR_U_CHAR:
	 hor_free ( (void *) imptr->array.uc );
	 break;

      case HOR_CHAR:
	 hor_free ( (void *) imptr->array.c );
	 break;

      case HOR_U_SHORT:
	 hor_free ( (void *) imptr->array.us );
	 break;

      case HOR_SHORT:
	 hor_free ( (void *) imptr->array.s );
	 break;

      case HOR_U_INT:
	 hor_free ( (void *) imptr->array.ui );
	 break;

      case HOR_INT:
	 hor_free ( (void *) imptr->array.i );
	 break;

      case HOR_FLOAT:
	 hor_free ( (void *) imptr->array.f );
	 break;

      case HOR_RGB_UC:
	 hor_free ( (void *) imptr->array.cuc );
	 break;

      case HOR_RGB_UI:
	 hor_free ( (void *) imptr->array.cui );
	 break;

      case HOR_POINTER:
	 hor_free ( (void **) imptr->array.p );
	 break;

      default:
	 hor_error ( "illegal image type (hor_free_image_header)", HOR_FATAL );
	 break;
   }
   hor_free ( (void *) imptr );
}

void hor_free_sub_image ( Hor_Sub_Image *imptr )
{
   if ( imptr == NULL ) return;

   hor_free_image_data ( imptr->image.array,
			 imptr->image.width*imptr->image.height,
			 imptr->image.type,  imptr->image.pixel_free_func );
   hor_free ( (void *) imptr );
}

void hor_free_sub_images ( Hor_Sub_Image *imptr1, ... )
{
   Hor_Sub_Image *imptr;
   va_list    ap;

   if ( imptr1 == NULL )
      return;

   hor_free_sub_image ( imptr1 );
   va_start ( ap, imptr1 );
   while(1)
   {
      imptr = va_arg ( ap, Hor_Sub_Image * );
      if ( imptr == NULL ) break;

      hor_free_sub_image ( imptr );
   }

   va_end(ap);
}
