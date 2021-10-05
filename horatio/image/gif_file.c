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

#define DEFAULT_GIF_NCOLOURS  256

static int ncolours = DEFAULT_GIF_NCOLOURS;

static void IOTerminateGifChild(int sig,int code,struct sigcontext *scp,char *addr)
{
  wait(NULL);
  return;
}

static int hor_giftoppm (int fdin)
{
  int fd[2],pid;
  
  if(pipe(fd))   return -1;
  signal(SIGCHLD,/*SIG_IGN*/ IOTerminateGifChild);
  if((pid=fork())<0) return -1;
    
  if(!pid) /* child */
  {
    if(dup2(fdin,0) == -1) 
      hor_error ("dup2(fdin,0) == -1 (giftoppm)", HOR_FATAL);
    (void) close(fdin);
      
    if(dup2(fd[1],1) == -1)
      hor_error ("dup2(fd[1],1) == -1 (giftoppm)", HOR_FATAL);

    (void) close(fd[0]);
    (void) execlp("giftoppm","giftoppm",0);
    _exit(127);
    /* return; */
  }
    
  /*parent*/
  (void) close(fd[1]);
  (void) close(fdin);
  return fd[0];
}

static int hor_ppmquant (int fdout)
{
  int  fd[2],pid;
  char nc[10];
  
  if(pipe(fd))       return -1;
  signal(SIGCHLD,/*SIG_IGN*/ IOTerminateGifChild);
  if((pid=fork())<0) return -1;
  
  if(!pid) /* child */
  {
    if(dup2(fdout,1) == -1)
      hor_error ("dup2(fdout,1) == -1 (hor_ppmquant)", HOR_FATAL);
    (void) close(fdout);
    
    if(dup2(fd[0],0) == -1)
      hor_error ("dup2(fd[0],0) == -1 (hor_ppmquant)", HOR_FATAL);

    (void) close(fd[1]);
    sprintf (nc, "%d", ncolours);
    (void) execlp("ppmquant","ppmquant", nc, 0);
    _exit(127);
  }
    
  /*parent*/
  (void) close(fd[0]);
  (void) close(fdout);
  return fd[1];  
}

static int hor_ppmtogif (int fdout)
{
  int  fd[2],pid;
  
  if(pipe(fd))       return -1;
  signal(SIGCHLD,/*SIG_IGN*/ IOTerminateGifChild);
  if((pid=fork())<0) return -1;
  
  if(!pid) /* child */
  {
    if(dup2(fdout,1) == -1)
      hor_error ("dup2(fdout,1) == -1 (hor_ppmtogif)", HOR_FATAL);
    (void) close(fdout);
    
    if(dup2(fd[0],0) == -1)
      hor_error ("dup2(fd[0],0) == -1 (hor_ppmtogif)", HOR_FATAL);

    (void) close(fd[1]);
    (void) execlp("ppmtogif","ppmtogif",0);
    _exit(127);
  }
    
  /*parent*/
  (void) close(fd[0]);
  (void) close(fdout);
  return fd[1];  
}

/*******************
*   Hor_Image *@hor_read_gif_image_stream  ( int fd )
*   Hor_Bool   @hor_write_gif_image_stream ( int fd, Hor_Image *imptr, ... )
*   Hor_Image *@hor_read_gif_image  ( const char *file_name )
*   Hor_Bool   @hor_write_gif_image ( const char *file_name,
*                                    Hor_Image *imptr, ... )
*
*   hor_read_gif_image_stream() reads and returns an GIF image read from the
*   given file stream. hor_read_gif_image() opens the given file and calls
*   hor_read_gif_image_stream().
*
*   GIF format image file I/O functions.
*
*   hor_write_gif_image_stream() outputs the given image to a file stream.
*   Extra arguments are required to convert images of type HOR_FLOAT to
*   an unsigned char file. hor_write_gif_image() opens the given file and calls
*   hor_write_gif_image_stream().
********************/
Hor_Image *hor_read_gif_image_stream ( int fd )
{
  if ((fd = hor_giftoppm ( fd )) == -1)
    hor_error ("fd == -1 (hor_read_gif_image_stream)", HOR_NON_FATAL);
  return hor_read_ppm_image_stream ( fd );
}

Hor_Bool hor_write_gif_image_stream ( int fd, Hor_Image *imptr, ... )
{
  if ((fd = hor_ppmtogif ( fd )) == -1)
    hor_error ("fd == -1 (hor_write_gif_image_stream)", HOR_NON_FATAL);
  if ((fd = hor_ppmquant ( fd )) == -1)
    hor_error ("fd == -1 (hor_write_gif_image_stream)", HOR_NON_FATAL);
  return hor_write_ppm_image_stream ( fd, imptr );
}

Hor_Image *hor_read_gif_image ( const char *file_name )
{
   Hor_Image *result;
   int        fd;

   /* open GIF file */
#ifdef HOR_MSDOS
   if ( ( fd = _open ( (char *) file_name, _O_RDONLY ) ) == -1 )
#else
   if ( ( fd = open ( (char *) file_name, O_RDONLY ) ) == -1 )
#endif
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_READ;
      return NULL;
   }

   result = hor_read_gif_image_stream ( fd );
#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_write_gif_image ( const char *file_name, Hor_Image *imptr, ... )
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
      hor_error ( "illegal compression mode (hor_write_gif_image)", HOR_FATAL);
   }
#endif
#endif
   if ( fd == -1 )
   {
      hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_WRITE;
      return HOR_FALSE;
   }

   result = hor_write_gif_image_stream ( fd, imptr );

#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}

Hor_Bool hor_set_gif_ncolours (int loc_ncolours)
{
  if (loc_ncolours < 2 || loc_ncolours > 256)
  {
    hor_error ("gif files must have between 2 and 256 colours", HOR_NON_FATAL);
    return (HOR_FALSE);
  }

  ncolours = loc_ncolours;
  return (HOR_TRUE);
}

#endif /* HOR_PROVIDE_RGB */
#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
