/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#ifdef HOR_TRANSPUTER

#include <stdlib.h>
#include <stddef.h>
#include <stdiored.h>
#include <process.h>
#include <misc.h>

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   void       @HorChanInImageHeader    (Channel *channel,Hor_Image     *image)
*   void       @HorChanInSubImageHeader (Channel *channel,Hor_Sub_Image *image)
*   void       @HorChanInImageData      (Channel *channel,Hor_Image     *image)
*   Hor_Image     *@HorChanInImage      (Channel *channel)
*   Hor_Sub_Image *@HorChanInSubImage   (Channel *channel)
*   void       @HorChanInAllocatedImage (Channel *channel,Hor_Image     *image)
*   void       @HorChanInAllocatedSubImage ( Channel *channel,
*                                           Hor_Sub_Image *image )
*   void       @HorChanOutImageHeader   (Channel *channel,Hor_Image     *image)
*   void       @HorChanOutSubImageHeader(Channel *channel,Hor_Sub_Image *image)
*   void       @HorChanOutImageData     (Channel *channel,Hor_Image     *image)
*   void       @HorChanOutImage         (Channel *channel,Hor_Image     *image)
*   void       @HorChanOutSubImage      (Channel *channel,Hor_Sub_Image *image)
*
*   Image channel I/O functions for transputers.
********************/
void HorChanInImageHeader ( Channel *channel, Hor_Image *image )
{
   ChanIn ( channel, image, sizeof(Hor_Image) );
}

void HorChanInSubImageHeader ( Channel *channel, Hor_Sub_Image *image )
{
   ChanIn ( channel, image, sizeof(Hor_Sub_Image) );
}

void HorChanInImageData ( Channel *channel, Hor_Image *image )
{
   int width = image->width, height = image->height;

   switch ( image->type )
   {
      case HOR_FLOAT:
      ChanIn ( channel, image->array.f[0], width*height*sizeof(float) );
      break;

      case HOR_BIT:
      ChanIn ( channel, image->array.b[0],
	       hor_bit_words(width)*height*sizeof(hor_bit) );
      break;

      case HOR_U_CHAR:
      ChanIn ( channel, image->array.uc[0], width*height*sizeof(u_char) );
      break;

      case HOR_CHAR:
      ChanIn ( channel, image->array.c[0], width*height*sizeof(char) );
      break;

      case HOR_U_SHORT:
      ChanIn ( channel, image->array.us[0], width*height*sizeof(u_short) );
      break;

      case HOR_SHORT:
      ChanIn ( channel, image->array.s[0], width*height*sizeof(short) );
      break;

      case HOR_U_INT:
      ChanIn ( channel, image->array.ui[0], width*height*sizeof(u_int) );
      break;

      case HOR_INT:
      ChanIn ( channel, image->array.i[0], width*height*sizeof(int) );
      break;

      default:
      hor_error ( "illegal image type (HorChanInImageData)", HOR_FATAL );
      break;
   }
}

Hor_Image *HorChanInImage ( Channel *channel )
{
   Hor_Image *image = hor_malloc_type ( Hor_Image );

   if ( image == NULL ) return NULL;

   HorChanInImageHeader ( channel, image );
   if ( !hor_alloc_image_data ( image->width, image->height, image->type,
			    &image->array ) )
   {
      hor_free ( (void *) image );
      return NULL;
   }

   HorChanInImageData ( channel, image );
   return image;
}

Hor_Sub_Image *HorChanInSubImage ( Channel *channel )
{
   Hor_Sub_Image *sub_image = hor_malloc_type ( Hor_Sub_Image );

   if ( sub_image == NULL ) return NULL;

   HorChanInSubImageHeader ( channel, sub_image );
   if ( !hor_alloc_image_data ( sub_image->image.width,
			        sub_image->image.height,
			        sub_image->image.type,
			        &sub_image->image.array ) )
   {
      hor_free ( (void *) sub_image );
      return NULL;
   }

   HorChanInImageData ( channel, &sub_image->image );
   return sub_image;
}

void HorChanInAllocatedImage ( Channel *channel, Hor_Image *image )
{
   Hor_Image new_image;

   HorChanInImageHeader ( channel, &new_image );
   if ( !hor_same_type_dims_images ( image, &new_image, NULL ) )
      hor_error ( "inconsistent images (HorChanInAllocatedImage)", HOR_FATAL );

   HorChanInImageData ( channel, image );
}

void HorChanInAllocatedSubImage ( Channel *channel, Hor_Sub_Image *image )
{
   Hor_Sub_Image new_image;

   HorChanInSubImageHeader ( channel, &new_image );
   if ( !hor_same_type_dims_sub_images ( image, &new_image, NULL ) )
      hor_error ( "inconsistent images (HorChanInAllocatedSubImage)",
		  HOR_FATAL );

   HorChanInImageData ( channel, &image->image );
}

void HorChanOutImageHeader ( Channel *channel, Hor_Image *image )
{
   ChanOut ( channel, image, sizeof(Hor_Image) );
}

void HorChanOutSubImageHeader ( Channel *channel, Hor_Sub_Image *image )
{
   ChanOut ( channel, image, sizeof(Hor_Sub_Image) );
}

void HorChanOutImageData ( Channel *channel, Hor_Image *image )
{
   int width = image->width, height = image->height;

   switch ( image->type )
   {
      case HOR_FLOAT:
      ChanOut ( channel, image->array.f[0], width*height*sizeof(float) );
      break;

      case HOR_BIT:
      ChanOut ( channel, image->array.b[0],
	        hor_bit_words(width)*height*sizeof(hor_bit) );
      break;

      case HOR_U_CHAR:
      ChanOut ( channel, image->array.uc[0], width*height*sizeof(u_char) );
      break;

      case HOR_CHAR:
      ChanOut ( channel, image->array.c[0], width*height*sizeof(char) );
      break;

      case HOR_U_SHORT:
      ChanOut ( channel, image->array.us[0], width*height*sizeof(u_short) );
      break;

      case HOR_SHORT:
      ChanOut ( channel, image->array.s[0], width*height*sizeof(short) );
      break;

      case HOR_U_INT:
      ChanOut ( channel, image->array.ui[0], width*height*sizeof(u_int) );
      break;

      case HOR_INT:
      ChanOut ( channel, image->array.i[0], width*height*sizeof(int) );
      break;

      default:
      hor_error ( "illegal image type (HorChanOutImage)", HOR_FATAL );
      break;
   }
}

void HorChanOutImage ( Channel *channel, Hor_Image *image )
{
   HorChanOutImageHeader ( channel, image );
   HorChanOutImageData   ( channel, image );
}

void HorChanOutSubImage ( Channel *channel, Hor_Sub_Image *image )
{
   HorChanOutSubImageHeader ( channel, image );
   HorChanOutImageData      ( channel, &image->image );
}

#endif
