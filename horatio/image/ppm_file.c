/* Copyright 1994 Charles S Wiles (csw@robots.oxford.ac.uk) and
                  Andrew Wildenberg (andrew@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */

#ifdef HOR_PROVIDE_RGB

#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#ifdef HOR_TRANSPUTER
#include <iocntrl.h>
#else
#include <fcntl.h>
#ifdef HOR_MSDOS
#include <sys/stat.h>
#include <io.h>
#else
#include <unistd.h>
#endif
#endif

#include "horatio/global.h"
#include "horatio/image.h"

/*******************
*   Hor_Image *@hor_read_ppm_image_stream  ( int fd )
*   Hor_Bool   @hor_write_ppm_image_stream ( int fd, Hor_Image *imptr, ... )
*   Hor_Image *@hor_read_ppm_image  ( const char *file_name )
*   Hor_Bool   @hor_write_ppm_image ( const char *file_name,
*                                    Hor_Image *imptr, ... )
*
*   hor_read_ppm_image_stream() reads and returns an PPM image read from the
*   given file stream. hor_read_ppm_image() opens the given file and calls
*   hor_read_ppm_image_stream().
*
*   PPM format image file I/O functions.
*
*   hor_write_ppm_image_stream() outputs the given image to a file stream.
*   Extra arguments are required to convert images of type HOR_FLOAT to
*   an unsigned char file. hor_write_ppm_image() opens the given file and calls
*   hor_write_ppm_image_stream().
********************/
Hor_Image *hor_read_ppm_image_stream ( int fd )
{
   Hor_Image *result;
   char       buffer[100];
   int        width, height, maxval;
   FILE      *fp;

   fp = fdopen ( dup(fd), "r" );
   if ( fp == NULL )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   fscanf ( fp, "%s", buffer );

   if ( strcmp ( buffer, "P6" ) )
   {
      hor_errno = HOR_IMAGE_BAD_MAGIC_NUMBER;
      fclose ( fp );
      return NULL;
   }

   if ( fscanf ( fp, "%d %d\n%d\n", &width, &height, &maxval ) != 3 )
   {
      hor_errno = HOR_IMAGE_READ_FAILED;
      fclose ( fp );
      return NULL;
   }

   if ( maxval != 255 )
   {
      hor_errno = HOR_IMAGE_ILLEGAL_BITS_PER_PIXEL;
      fclose ( fp );
      return NULL;
   }

   result = hor_alloc_image ( width, height, HOR_RGB_UC, NULL );
   if ( result == NULL )
   {
      hor_errno = HOR_IMAGE_ALLOCATION_FAILED;
      fclose ( fp );
      return NULL;
   }

   fread ( result->array.cuc[0], sizeof(Hor_RGB_UC), width*height, fp );
   fclose ( fp );
   return result;
}

Hor_Bool hor_write_ppm_image_stream ( int fd, Hor_Image *imptr, ... )
{
   FILE *fp;

   fp = fdopen ( dup(fd), "w" );
   if ( fp == NULL )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return HOR_FALSE;
   }

   if ( imptr->type != HOR_RGB_UC )
   {
      hor_errno = HOR_IMAGE_WRONG_TYPE_IMAGE;
      fclose ( fp );
      return HOR_FALSE;
   }

   fprintf ( fp, "P6\n%d %d\n255\n", imptr->width, imptr->height );
   fwrite ( imptr->array.cuc[0], sizeof(Hor_RGB_UC),
	   imptr->width*imptr->height, fp );
   fclose ( fp );
   return HOR_TRUE;
}

Hor_Image *hor_read_ppm_image ( const char *file_name )
{
   Hor_Image *result;
   int        fd;

   /* open PPM file */
#ifdef HOR_MSDOS
   if ( ( fd = _open ( (char *) file_name, _O_RDONLY ) ) == -1 )
#else
   if ( ( fd = open ( (char *) file_name, O_RDONLY ) ) == -1 )
#endif
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   result = hor_read_ppm_image_stream ( fd );
#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_write_ppm_image ( const char *file_name, Hor_Image *imptr, ... )
{
   int      fd = -1;
   Hor_Bool result;

   if ( imptr->type != HOR_RGB_UC )
   {
      hor_errno = HOR_IMAGE_WRONG_TYPE_IMAGE;
      return HOR_FALSE;
   }

#ifdef HOR_TRANSPUTER
   fd = open ( (char *) file_name, O_BINARY | O_WRONLY );
#else
#ifdef HOR_MSDOS
   fd = _open ( (char *) file_name, _O_BINARY | _O_CREAT |_O_WRONLY,
   		_S_IREAD | _S_IWRITE );
#else
   switch ( hor_get_image_compress() )
   {
      case HOR_NO_COMPRESS:
      fd = open ( (char *) file_name, O_CREAT | O_WRONLY, 0644 );
      break;

      case HOR_UNIX_COMPRESS:
      fd = hor_compress ( file_name );
      break;

      case HOR_GNU_COMPRESS:
      fd = hor_gzip ( file_name );
      break;

      default:
      hor_error ( "illegal compression mode (hor_write_ppm_image)", HOR_FATAL);
   }
#endif
#endif
   if ( fd == -1 )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   result = hor_write_ppm_image_stream ( fd, imptr );

#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

#endif /* HOR_PROVIDE_RGB */
