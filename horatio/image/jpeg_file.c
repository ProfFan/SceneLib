/* Copyright 1993 Charles S Wiles (csw@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ppm_file.h */

#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER
#ifdef HOR_PROVIDE_RGB

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/wait.h>
#include <signal.h>

#include "horatio/global.h"
#include "horatio/image.h"

#define DEFAULT_JPEG_QUALITY      100

static int quality = DEFAULT_JPEG_QUALITY;

static void IOTerminateJpegChild(int sig,int code,struct sigcontext *scp,char *addr)
{
  wait(NULL);
  return;
}

static int hor_djpeg (int fdin)
{
  int fd[2],pid;
  
  if(pipe(fd))   return -1;
  signal(SIGCHLD,/*SIG_IGN*/ IOTerminateJpegChild);
  if((pid=fork())<0) return -1;
    
  if(!pid) /* child */
  {
    if(dup2(fdin,0) == -1) 
      hor_error ("dup2(fdin,0) == -1 (hor_djpeg)", HOR_FATAL);
    (void) close(fdin);
      
    if(dup2(fd[1],1) == -1)
      hor_error ("dup2(fd[1],1) == -1 (hor_djpeg)", HOR_FATAL);

    (void) close(fd[0]);
    (void) execlp("djpeg","djpeg","-P",0);
    _exit(127);
    /* return; */
  }
    
  /*parent*/
  (void) close(fd[1]);
  (void) close(fdin);
  return fd[0];
}

static int hor_cjpeg (int fdout)
{
  int  fd[2],pid;
  char Q[10];
  
  if(pipe(fd))       return -1;
  signal(SIGCHLD,/*SIG_IGN*/ IOTerminateJpegChild);
  if((pid=fork())<0) return -1;
  
  if(!pid) /* child */
  {
    if(dup2(fdout,1) == -1)
      hor_error ("dup2(fdout,1) == -1 (hor_cjpeg)", HOR_FATAL);
    (void) close(fdout);
    
    if(dup2(fd[0],0) == -1)
      hor_error ("dup2(fd[0],0) == -1 (hor_cjpeg)", HOR_FATAL);

    (void) close(fd[1]);
    sprintf(Q,"%d", quality);
    (void) execlp("cjpeg","cjpeg","-Q",Q,0);

    _exit(127);
  }
    
  /*parent*/
  (void) close(fd[0]);
  (void) close(fdout);
  return fd[1];  
}

/*******************
*   Hor_Image *@hor_read_jpeg_image_stream  ( int fd )
*   Hor_Bool   @hor_write_jpeg_image_stream ( int fd, Hor_Image *imptr, ... )
*   Hor_Image *@hor_read_jpeg_image  ( const char *file_name )
*   Hor_Bool   @hor_write_jpeg_image ( const char *file_name,
*                                    Hor_Image *imptr, ... )
*
*   hor_read_jpeg_image_stream() reads and returns an JPEG image read from the
*   given file stream. hor_read_jpeg_image() opens the given file and calls
*   hor_read_jpeg_image_stream().
*
*   JPEG format image file I/O functions.
*
*   hor_write_jpeg_image_stream() outputs the given image to a file stream.
*   Extra arguments are required to convert images of type HOR_FLOAT to
*   an unsigned char file. hor_write_jpeg_image() opens the given file and calls
*   hor_write_jpeg_image_stream().
********************/
Hor_Image *hor_read_jpeg_image_stream ( int fd )
{
  if ((fd = hor_djpeg ( fd )) == -1)
    hor_error ("fd == -1 (hor_read_jpeg_image_stream)", HOR_NON_FATAL);
  return hor_read_pgm_image_stream ( fd );
}

Hor_Bool hor_write_jpeg_image_stream ( int fd, Hor_Image *imptr, ... )
{
  if ((fd = hor_cjpeg ( fd )) == -1)
    hor_error ("fd == -1 (hor_write_jpeg_image_stream)", HOR_NON_FATAL);
  return (imptr->type == HOR_U_CHAR) ? hor_write_pgm_image_stream ( fd, imptr )
                                     : hor_write_ppm_image_stream ( fd, imptr );
}

Hor_Image *hor_read_jpeg_image ( const char *file_name )
{
   Hor_Image *result;
   int        fd;

   /* open JPEG file */
#ifdef HOR_MSDOS
   if ( ( fd = _open ( (char *) file_name, _O_RDONLY ) ) == -1 )
#else
   if ( ( fd = open ( (char *) file_name, O_RDONLY ) ) == -1 )
#endif
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   result = hor_read_jpeg_image_stream ( fd );
#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_write_jpeg_image ( const char *file_name, Hor_Image *imptr, ... )
{
   int      fd = -1;
   Hor_Bool result;

   if ( imptr->type != HOR_RGB_UC && imptr->type != HOR_U_CHAR)
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
      hor_error ( "illegal compression mode (hor_write_jpeg_image)", HOR_FATAL);
   }
#endif
#endif
   if ( fd == -1 )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   result = hor_write_jpeg_image_stream ( fd, imptr );

#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_set_jpeg_quality ( int loc_quality )
{
  if (loc_quality <= 0 && loc_quality > 100)
  {
    hor_error ("jpeg quality must be between 1 and 100", HOR_NON_FATAL);
    return (HOR_FALSE);
  }

  quality = loc_quality;
  return (HOR_TRUE);
}

#endif /* HOR_PROVIDE_RGB */
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
