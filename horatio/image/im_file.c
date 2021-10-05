/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#ifdef HOR_TRANSPUTER
#include <iocntrl.h>
#else
#include <fcntl.h>
#ifdef HOR_MSDOS
#include <io.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#endif
#endif

#include "horatio/global.h"
#include "horatio/image.h"


#ifdef HOR_PROVIDE_RGB
#include "ppm_file.h"
#include "gif_file.h"
#include "jpeg_file.h"
#endif /* HOR_PROVIDE_RGB */

static Hor_Image_Format image_format = HOR_MIT_FORMAT;
#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
static Hor_Compress_Type compress_type = HOR_NO_COMPRESS;
#endif
#endif

/*******************
*   Hor_Image *@hor_read_image_stream ( int fd )
*   Hor_Compress_Type @hor_read_image_compress (int fd )
*   Hor_Image_Format @hor_read_image_format ( int fd,
*					  Hor_Compress_Type read_compress_type)
*   Hor_Image *@hor_read_image        ( const char *base_name )
*   Hor_Image *@hor_read_named_image  ( const char *file_name,
*                                      Hor_Image_Format format )
*
*   void       @hor_set_image_format ( Hor_Image_Format new_image_format )
*   void       @hor_set_image_compress ( Hor_Compress_Type compress_type )
*   Hor_Compress_Type @hor_get_image_compress(void)
*   Hor_Bool   @hor_write_image_stream ( int fd, Hor_Image *imptr, ... )
*   Hor_Bool   @hor_write_image ( const char *base_name, Hor_Image *imptr, ... )
*   Hor_Bool   @hor_write_named_image ( const char *file_name,
*                                      Hor_Image *imptr, ... )
*
*   Image file I/O functions.
*
*   hor_read_image_stream() reads an image from a stream. The format is assumed
*                           be that set by hor_set_image_format(). NULL is
*                           returned on error.
*
*   hor_read_image_compress () reads the compression type of a file.
*                              The result returned will be HOR_NO_COMPRESS,
*                              HOR_UNIX_COMPRESS or HOR_GNU_COMPRESS.
*
*   hor_read_image_format () reads the image format of a file. HOR_TRUE is
*                            returned if the file format is recognised,
*                            otherwise HOR_FALSE.  Recognised formats are
*                            are HOR_MIT_FORMAT, HOR_IFF_FORMAT,
*                            HOR_PGM_FORMAT, HOR_PPM_FORMAT, HOR_GIF_FORMAT
*                            and HOR_JPEG_FORMAT.
*
*   hor_read_image() reads and returns an image with given name.
*                    The following file name suffices are tried in this order:
*
*                    no suffix .mit .iff .pgm .i .ppm .gif .jpg
*
*                    The first one that is found is read (multiple occurrences
*                    are not detected). Both .mit and .i indicate MIT format.
*                    An extra .Z suffix indicates UNIX compression, and .gz or
*                    .z indicate GNU compression (both types of compression
*                    are handled). NULL is returned on error.
*
*   hor_read_named_image() is the same as hor_read_image(), but is left in for
*                          compatibility with older versions.
*                          The image_format argument is ignored.
*
*   hor_set_image_format() sets the image format for subsequent calls to
*                          hor_write_image_stream(), hor_write_image() and
*                          hor_write_named_image(). Currently accepted image
*                          formats are HOR_MIT_FORMAT, HOR_IFF_FORMAT,
*                          HOR_PGM_FORMAT, HOR_PPM_FORMAT, HOR_GIF_FORMAT,
*                          HOR_PS_FORMAT,
*                          and HOR_JPEG_FORMAT. HOR_MIT_FORMAT is the default.
*
*   hor_set_image_compress() sets the compression mode for writing images
*                            to files.
*   hor_get_image_compress() returns the current compression type.
*
*   hor_write_image_stream() outputs the image to the stream in the current
*                            image format set by hor_set_image_format().
*                            Returns HOR_TRUE on success, HOR_FALSE on failure.
*
*   hor_write_image() outputs the given image in the current image format set
*                     by hor_set_image_format()) to a file formed by
*                     appending .mit, .iff, .pgm, .ppm, .gif, .ps, or .jpg to
*                     base_name as appropriate.
*                     Returns HOR_TRUE on success, HOR_FALSE on failure.
*
*   hor_write_named_image() does the same as hor_write_image() but to a file
*                           without .mit, .iff, .pgm, .ppm, .gif, .ps or .jpg
*                           extension.
*
*   HOR_FLOAT type images are written to unsigned char files. In that case the
*   variable argument lists in hor_write_image_stream() and hor_write_image()
*   contain the lower and upper limits on pixel value (corresponding to 0 and
*   255 in the image that is written out).
********************/
Hor_Image *hor_read_image_stream ( int fd )
{
   switch ( image_format )
   {
      case HOR_MIT_FORMAT:
      return ( hor_read_mit_image_stream ( fd ) );
      break;

      case HOR_IFF_FORMAT:
      return ( hor_read_iff_image_stream ( fd ) );
      break;

#ifndef HOR_TRANSPUTER
      case HOR_PGM_FORMAT:
      return ( hor_read_pgm_image_stream ( fd ) );
      break;
#endif

#ifdef HOR_PROVIDE_RGB
      case HOR_PPM_FORMAT:
      return ( hor_read_ppm_image_stream ( fd ) );
      break;

#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
      case HOR_GIF_FORMAT:
      return ( hor_read_gif_image_stream ( fd ) );
      break;

      case HOR_JPEG_FORMAT:
      return ( hor_read_jpeg_image_stream ( fd ) );
      break;
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */

      default:
      hor_error ( "illegal image format (hor_read_image_stream)", HOR_FATAL );
      return NULL;
      break;
   }
}

#ifndef HOR_TRANSPUTER
Hor_Compress_Type hor_read_image_compress (int fd)
{
  FILE *fp;
  u_char  magicno[8];    /* first 8 bytes of file */
  Hor_Compress_Type read_compress_type;

  fp = fdopen ( dup(fd), "r" );
  if ( fp == NULL )
  {
    hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
    return -1;
  }

  if (fread(magicno,8,1,fp) != 1)
  {
    fclose(fp);
    return -1;  
  }
  fclose(fp);
  
  if (magicno[0]==0x1f && magicno[1]==0x9d) /* COMPRESSED */
    read_compress_type = HOR_UNIX_COMPRESS;
  else if (magicno[0]==0x1f && magicno[1]==0x8b &&
      magicno[2]==0x08 && magicno[3]==0x08) /* GZIPPED */
    read_compress_type = HOR_GNU_COMPRESS;
  else
    read_compress_type = HOR_NO_COMPRESS;

  return read_compress_type;
}

Hor_Image_Format hor_read_image_format ( int fd,
					 Hor_Compress_Type read_compress_type )
{
  FILE *fp;
  u_char  magicno[8];    /* first 8 bytes of file */
  int pid;
  Hor_Image_Format read_image_format = HOR_MIT_FORMAT;

  switch (read_compress_type)
  {
  case HOR_NO_COMPRESS:
    break;

  case HOR_UNIX_COMPRESS:
    if ( (fd = hor_uncompress ( fd, &pid )) == -1 )
      return HOR_FALSE;
    break;

  case HOR_GNU_COMPRESS:
    if ( (fd = hor_gunzip ( fd, &pid )) == -1 )
      return HOR_FALSE;
    break;
    
  default:
    hor_error ("bad compress type (hor_read_image_format)", HOR_FATAL);
  }

  fp = fdopen ( fd, "r" );
  if ( fp == NULL )
  {
    hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
    return HOR_FALSE;
  }

  if (fread(magicno,8,1,fp) != 1)
  {
    fclose(fp);
    return HOR_FALSE;  
  }
  fclose(fp);

  if ((magicno[0]==0x01 && (magicno[1]&0x7f)==0x00 &&
       magicno[2]==0x01 && (magicno[3]&0x7f)==0x00) ||
      (magicno[0]==0x01 && (magicno[1]&0x7f)==0x00 &&
       magicno[2]==0x08 && (magicno[3]&0x7f)==0x00) ||
      (magicno[0]==0x01 && (magicno[1]&0x7f)==0x00 &&
       magicno[2]==0x20 && (magicno[3]&0x7f)==0x00) ||
      (magicno[0]==0x06 && (magicno[1]&0x7f)==0x00 &&
       magicno[2]==0x20 && (magicno[3]&0x7f)==0x00))
    read_image_format = HOR_MIT_FORMAT;
  else if ((magicno[0]==0x01 && (magicno[1]&0x7f)==0x00 &&
	    magicno[2]==0x00 && (magicno[3]&0x7f)==0x00))
    read_image_format = HOR_IFF_FORMAT;
  else if (magicno[0] == 'P' && magicno[1]>='1' && 
	   magicno[1] =='5')
    read_image_format = HOR_PGM_FORMAT;
#ifdef HOR_PROVIDE_RGB
  else if (magicno[0] == 'P' && magicno[1]>='1' && 
	   magicno[1] =='6')
    read_image_format = HOR_PPM_FORMAT;
#ifndef HOR_MSDOS
  else if (strncmp((char *) magicno,"GIF87a",6)==0 ||
	   strncmp((char *) magicno,"GIF89a",6)==0)
    read_image_format = HOR_GIF_FORMAT;
  else if (magicno[0]==0xff && magicno[1]==0xd8 && 
	   magicno[2]==0xff)
    read_image_format = HOR_JPEG_FORMAT;
#endif /* HOR_MSDOS */
#endif

#if 0 /* these formats don't exist yet */
  else if (strncmp((char *) magicno,"VIEW",4)==0 ||
	   strncmp((char *) magicno,"WEIV",4)==0)
    read_image_format = HOR_PM_FORMAT;
  else if (strncmp((char *) magicno,"#define",7)==0)
    read_image_format = HOR_XBM_FORMAT;
  else if (magicno[0]==0x59 && (magicno[1]&0x7f)==0x26 &&
	   magicno[2]==0x6a && (magicno[3]&0x7f)==0x15)
    read_image_format = HOR_RAS_FORMAT;
  else if (magicno[0] == 'B' && magicno[1] == 'M')
    read_image_format = HOR_BMP_FORMAT;
  else if (magicno[0]==0x52 && magicno[1]==0xcc)
    read_image_format = HOR_UTAHRLE_FORMAT;
  else if ((magicno[0]==0x01 && magicno[1]==0xda) ||
	   (magicno[0]==0xda && magicno[1]==0x01))
    read_image_format = HOR_IRIS_FORMAT;
  else if (magicno[0]==0x0a && magicno[1] <= 5)
    read_image_format = HOR_PCX_FORMAT;
  else if (!strncmp((char *) magicno,"/* XPM */",8))
    read_image_format = HOR_XPM_FORMAT;
  else if ((magicno[0]=='M' && magicno[1]=='M') ||
	   (magicno[0]=='I' && magicno[1]=='I'))
    read_image_format = HOR_TIFF_FORMAT;
#endif

  else
    hor_error ("unknown image format (hor_read_image_format)",
	       HOR_FATAL);

  return read_image_format;
}

static int open_file_rdonly (const char *full_name)
{
  int fd;

#ifdef HOR_MSDOS
  fd =_open ( full_name,_O_RDONLY );
#else
  fd = open ( (char *) full_name, O_RDONLY );
#endif
  return fd;
}

static void close_file (int fd)
{
#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
}

Hor_Image *hor_read_image ( const char *base_name )
{
  Hor_Image *result = NULL;
  char full_name[300];
  int  fd, pid;
  Hor_Image_Format read_image_format;
  Hor_Compress_Type read_compress_type;

   sprintf ( full_name, "%s", base_name );
   fd = open_file_rdonly (full_name);
   if (fd == -1)
   {
     sprintf ( full_name, "%s.mit", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.iff", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.pgm", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.i", base_name );
     fd = open_file_rdonly (full_name);
   }
#ifdef HOR_PROVIDE_RGB
   if (fd == -1)
   {
     sprintf ( full_name, "%s.ppm", base_name );
     fd = open_file_rdonly (full_name);
   }
#ifndef HOR_MSDOS
   if (fd == -1)
   {
     sprintf ( full_name, "%s.gif", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.jpg", base_name );
     fd = open_file_rdonly (full_name);
   }
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */


#ifndef HOR_MSDOS
   if (fd == -1)
   {
     sprintf ( full_name, "%s.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.mit.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.iff.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.pgm.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.i.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
#ifdef HOR_PROVIDE_RGB
   if (fd == -1)
   {
     sprintf ( full_name, "%s.ppm.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.gif.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.jpg.Z", base_name );
     fd = open_file_rdonly (full_name);
   }
#endif /* HOR_PROVIDE_RGB */

   if (fd == -1)
   {
     sprintf ( full_name, "%s.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.mit.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.iff.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.pgm.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.i.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
#ifdef HOR_PROVIDE_RGB
   if (fd == -1)
   {
     sprintf ( full_name, "%s.ppm.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.gif.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.jpg.gz", base_name );
     fd = open_file_rdonly (full_name);
   }
#endif /* HOR_PROVIDE_RGB */

   if (fd == -1)
   {
     sprintf ( full_name, "%s.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.mit.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.iff.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.pgm.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.i.z", base_name );
     fd = open_file_rdonly (full_name);
   }
#ifdef HOR_PROVIDE_RGB
   if (fd == -1)
   {
     sprintf ( full_name, "%s.ppm.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.gif.z", base_name );
     fd = open_file_rdonly (full_name);
   }
   if (fd == -1)
   {
     sprintf ( full_name, "%s.jpg.z", base_name );
     fd = open_file_rdonly (full_name);
   }
#endif /* HOR_PROVIDE_RGB */

#endif /* MSDOS */
   if (fd == -1)
   {
     hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
     return NULL;
   }

   read_compress_type = hor_read_image_compress(fd);
   close_file (fd);

   fd = open_file_rdonly (full_name);
   read_image_format = hor_read_image_format ( fd, read_compress_type );
   close_file (fd);

   fd = open_file_rdonly (full_name);

   switch (read_compress_type)
   {
   case HOR_NO_COMPRESS:
     break;

   case HOR_UNIX_COMPRESS:
     if ( (fd = hor_uncompress ( fd, &pid )) == -1 )
       return NULL;
     break;

   case HOR_GNU_COMPRESS:
     if ( (fd = hor_gunzip ( fd, &pid )) == -1 )
       return NULL;
     break;
    
   default:
     hor_error ("bad compress type (hor_read_image_format)", HOR_FATAL);
   }

   switch (read_image_format)
   {
   case HOR_MIT_FORMAT:
     result = hor_read_mit_image_stream ( fd );
     break;
   case HOR_IFF_FORMAT:
     result = hor_read_iff_image_stream ( fd );
     break;
   case HOR_PGM_FORMAT:
     result = hor_read_pgm_image_stream ( fd );
     break;
#ifdef HOR_PROVIDE_RGB
   case HOR_PPM_FORMAT:
     result = hor_read_ppm_image_stream ( fd );
     break;
#ifndef HOR_MSDOS
   case HOR_GIF_FORMAT:
     result = hor_read_gif_image_stream ( fd );
     break;
   case HOR_JPEG_FORMAT:
     result = hor_read_jpeg_image_stream ( fd );
     break;
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */
   default:
     break;
   }
   close_file (fd);

   return result;
}

Hor_Image *hor_read_named_image ( const char *file_name,
				  Hor_Image_Format format )
{
   return hor_read_image (file_name);
}

#else /* HOR_TRANSPUTER */

static Hor_Image *read_mit_file ( int fd )
{
   Hor_Image *result;

   result = hor_read_mit_image_stream ( fd );
   close ( fd );
   return result;
}

static Hor_Image *read_iff_file ( int fd )
{
   Hor_Image *result;

   result = hor_read_iff_image_stream ( fd );
   close ( fd );
   return result;
}

Hor_Image *hor_read_image ( const char *base_name )
{
   char full_name[300];
   int  fd;

   sprintf ( full_name, "%s.mit", base_name );
   if ( (fd = open ( full_name, O_RDONLY )) != -1 ) return (read_mit_file(fd));

   sprintf ( full_name, "%s.iff", base_name );
   if ( (fd = open ( full_name, O_RDONLY )) != -1 ) return (read_iff_file(fd));

   sprintf ( full_name, "%s.i", base_name );
   if ( (fd = open ( full_name, O_RDONLY )) != -1 ) return (read_mit_file(fd));

   hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
   return NULL;
}

Hor_Image *hor_read_named_image ( const char *file_name,
				  Hor_Image_Format format )
{
   int fd;

   switch ( format )
   {
      case HOR_MIT_FORMAT:
      if ( (fd = open ( (char *) file_name, O_RDONLY )) != -1 )
	 return (read_mit_file(fd));

       break;

      case HOR_IFF_FORMAT:
      if ( (fd = open ( (char *) file_name, O_RDONLY )) != -1 )
	 return (read_iff_file(fd));

      break;

      default:
      hor_error ( "illegal image format %d (hor_read_named_image)", HOR_FATAL,
		  format );
      
      break;
   }

   hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
   return NULL;
}
#endif /* HOR_TRANSPUTER */

void hor_set_image_format ( Hor_Image_Format new_image_format )
{
   image_format = new_image_format;
}

#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
void hor_set_image_compress ( Hor_Compress_Type new_compress_type )
{
   compress_type = new_compress_type;
}

Hor_Compress_Type hor_get_image_compress(void)
{
   return compress_type;
}
#endif /* HOR_MSDOS */
#endif /* HOR_TRANSPUTER */

Hor_Bool hor_write_image_stream ( int fd, Hor_Image *imptr, ... )
{
   float black_val = 0.0F, white_val = 256.0F;

   if ( imptr->type == HOR_FLOAT )
   {
      va_list ap;

      va_start ( ap, imptr );
      black_val = (float) va_arg ( ap, double );
      white_val = (float) va_arg ( ap, double );
      va_end(ap);
   }

   switch ( image_format )
   {
      case HOR_MIT_FORMAT:
      return ( hor_write_mit_image_stream ( fd, imptr, black_val, white_val ));
      break;

      case HOR_IFF_FORMAT:
      return ( hor_write_iff_image_stream ( fd, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
      case HOR_PGM_FORMAT:
      return ( hor_write_pgm_image_stream ( fd, imptr, black_val, white_val ));
      break;

      case HOR_PS_FORMAT:
      return ( hor_write_ps_image_stream ( fd, imptr, black_val, white_val ));
      break;
#endif

#ifdef HOR_PROVIDE_RGB
      case HOR_PPM_FORMAT:
      return ( hor_write_ppm_image_stream ( fd, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
      case HOR_GIF_FORMAT:
      return ( hor_write_gif_image_stream ( fd, imptr, black_val, white_val ));
      break;

      case HOR_JPEG_FORMAT:
      return ( hor_write_jpeg_image_stream ( fd, imptr, black_val, white_val ));
      break;
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */

      default:
      hor_error ( "illegal image format (hor_write_image_stream)", HOR_FATAL );
      break;
   }

   return HOR_FALSE;
}

Hor_Bool hor_write_image ( const char *base_name, Hor_Image *imptr, ... )
{
   float black_val = 0.0F, white_val = 0.0F;

   if ( imptr->type == HOR_FLOAT )
   {
      va_list ap;

      va_start ( ap, imptr );
      black_val = (float) va_arg ( ap, double );
      white_val = (float) va_arg ( ap, double );
      va_end(ap);
   }

   switch ( image_format )
   {
      char file_name[100];

      case HOR_MIT_FORMAT:
      sprintf ( file_name, "%s.mit", base_name );
      return (hor_write_mit_image ( file_name, imptr, black_val, white_val ));
      break;

      case HOR_IFF_FORMAT:
      sprintf ( file_name, "%s.iff", base_name );
      return (hor_write_iff_image ( file_name, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
      case HOR_PGM_FORMAT:
      sprintf ( file_name, "%s.pgm", base_name );
      return (hor_write_pgm_image ( file_name, imptr, black_val, white_val ));
      break;

      case HOR_PS_FORMAT:
      sprintf ( file_name, "%s.ps", base_name );
      return (hor_write_ps_image ( file_name, imptr, black_val, white_val ));
      break;
	  
#endif

#ifdef HOR_PROVIDE_RGB
      case HOR_PPM_FORMAT:
      sprintf ( file_name, "%s.ppm", base_name );
      return (hor_write_ppm_image ( file_name, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
      case HOR_GIF_FORMAT:
      sprintf ( file_name, "%s.gif", base_name );
      return (hor_write_gif_image ( file_name, imptr, black_val, white_val ));
      break;

      case HOR_JPEG_FORMAT:
      sprintf ( file_name, "%s.jpg", base_name );
      return (hor_write_jpeg_image ( file_name, imptr, black_val, white_val ));
      break;
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */

      default:
      hor_error ( "illegal image format (hor_write_image)", HOR_FATAL );
      break;
   }

   return HOR_FALSE;
}

Hor_Bool hor_write_named_image ( const char *file_name, Hor_Image *imptr, ... )
{
   float black_val = 0.0F, white_val = 0.0F;

   if ( imptr->type == HOR_FLOAT )
   {
      va_list ap;

      va_start ( ap, imptr );
      black_val = (float) va_arg ( ap, double );
      white_val = (float) va_arg ( ap, double );
      va_end(ap);
   }

   switch ( image_format )
   {
      case HOR_MIT_FORMAT:
      return (hor_write_mit_image ( file_name, imptr, black_val, white_val ));
      break;

      case HOR_IFF_FORMAT:
      return (hor_write_iff_image ( file_name, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
      case HOR_PGM_FORMAT:
      return (hor_write_pgm_image ( file_name, imptr, black_val, white_val ));
      break;
#endif

#ifdef HOR_PROVIDE_RGB
      case HOR_PPM_FORMAT:
      return (hor_write_ppm_image ( file_name, imptr, black_val, white_val ));
      break;

#ifndef HOR_TRANSPUTER
#ifndef HOR_MSDOS
      case HOR_GIF_FORMAT:
      return (hor_write_gif_image ( file_name, imptr, black_val, white_val ));
      break;

      case HOR_JPEG_FORMAT:
      return (hor_write_jpeg_image ( file_name, imptr, black_val, white_val ));
      break;
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
#endif /* HOR_PROVIDE_RGB */

      default:
      hor_error ( "illegal image format (hor_write_named_image)", HOR_FATAL );
      break;
   }

   return HOR_FALSE;
}
