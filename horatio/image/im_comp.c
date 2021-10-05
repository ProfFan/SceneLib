/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <math.h>
#include <stdarg.h>
#include <stddef.h>

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   Hor_Bool @hor_same_type_images      ( Hor_Image *imptr1, ... )
*   Hor_Bool @hor_same_dims_images      ( Hor_Image *imptr1, ... )
*   Hor_Bool @hor_same_type_dims_images ( Hor_Image *imptr1, ... )
*   Hor_Bool @hor_same_type_sub_images      ( Hor_Sub_Image *sub_imptr1, ... )
*   Hor_Bool @hor_same_dims_sub_images      ( Hor_Sub_Image *sub_imptr1, ... )
*   Hor_Bool @hor_same_type_dims_sub_images ( Hor_Sub_Image *sub_imptr1, ... )
*
*   Return HOR_TRUE if all (sub-)images in the NULL-terminated list are the same
*   (type, dimensions, type and dimensions), and HOR_FALSE otherwise.
********************/
Hor_Bool hor_same_type_images ( Hor_Image *imptr1, ... )
{
   va_list        ap;
   Hor_Image_Type type;
   Hor_Image     *imptr;

   if ( imptr1 == NULL ) return HOR_FALSE;

   type = imptr1->type;
   va_start ( ap, imptr1 );
   while(1)
   {
      imptr = va_arg ( ap, Hor_Image * );
      if ( imptr == NULL ) break;

      if ( imptr->type != type )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}

Hor_Bool hor_same_type_sub_images ( Hor_Sub_Image *sub_imptr1, ... )
{
   va_list        ap;
   Hor_Image_Type type;
   Hor_Sub_Image *sub_imptr;

   if ( sub_imptr1 == NULL ) return HOR_FALSE;

   type = sub_imptr1->image.type;
   va_start ( ap, sub_imptr1 );
   while(1)
   {
      sub_imptr = va_arg ( ap, Hor_Sub_Image * );
      if ( sub_imptr == NULL ) break;

      if ( sub_imptr->image.type != type )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}

Hor_Bool hor_same_dims_images ( Hor_Image *imptr1, ... )
{
   va_list    ap;
   int        width, height;
   Hor_Image *imptr;

   if ( imptr1 == NULL ) return HOR_FALSE;
   width  = imptr1->width;
   height = imptr1->height;
   va_start ( ap, imptr1 );
   while(1)
   {
      imptr = va_arg ( ap, Hor_Image * );
      if ( imptr == NULL ) break;

      if ( imptr->width != width || imptr->height != height )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}

Hor_Bool hor_same_dims_sub_images ( Hor_Sub_Image *sub_imptr1, ... )
{
   va_list        ap;
   int            c0, r0, width, height;
   Hor_Sub_Image *sub_imptr;

   if ( sub_imptr1 == NULL ) return HOR_FALSE;

   c0     = sub_imptr1->c0;
   r0     = sub_imptr1->r0;
   width  = sub_imptr1->image.width;
   height = sub_imptr1->image.height;
   va_start ( ap, sub_imptr1 );
   while(1)
   {
      sub_imptr = va_arg ( ap, Hor_Sub_Image * );
      if ( sub_imptr == NULL ) break;

      if ( sub_imptr->c0 != c0 || sub_imptr->image.width !=  width ||
	   sub_imptr->r0 != r0 || sub_imptr->image.height != height )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}

Hor_Bool hor_same_type_dims_images ( Hor_Image *imptr1, ... )
{
   va_list        ap;
   Hor_Image_Type type;
   int            width, height;
   Hor_Image     *imptr;

   if ( imptr1 == NULL ) return HOR_FALSE;

   type   = imptr1->type;
   width  = imptr1->width;
   height = imptr1->height;
   va_start ( ap, imptr1 );
   while(1)
   {
      imptr = va_arg ( ap, Hor_Image * );
      if ( imptr == NULL ) break;

      if ( imptr->type != type ||
	   imptr->width != width || imptr->height != height )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}

Hor_Bool hor_same_type_dims_sub_images ( Hor_Sub_Image *sub_imptr1, ... )
{
   va_list        ap;
   Hor_Image_Type type;
   int            c0, r0, width, height;
   Hor_Sub_Image *sub_imptr;

   type   = sub_imptr1->image.type;
   c0     = sub_imptr1->c0;
   r0     = sub_imptr1->r0;
   width  = sub_imptr1->image.width;
   height = sub_imptr1->image.height;
   va_start ( ap, sub_imptr1 );
   while(1)
   {
      sub_imptr = va_arg ( ap, Hor_Sub_Image * );
      if ( sub_imptr == NULL ) break;

      if ( sub_imptr->image.type != type ||
	   sub_imptr->c0 != c0 || sub_imptr->image.width !=  width ||
	   sub_imptr->r0 != r0 || sub_imptr->image.height != height )
      { va_end(ap); return HOR_FALSE; }
   }

   va_end(ap);
   return HOR_TRUE;
}
