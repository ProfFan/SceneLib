/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
		  Robotics Research Group, Oxford University. */
#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include "horatio/global.h"

static void TerminateChild(int sig,int code,struct sigcontext *scp,char *addr)
{
   wait(NULL);
   return;
}

/*******************
*   int @hor_compress ( const char *file_name )
*   int @hor_gzip     ( const char *file_name )
*   int @hor_uncompress ( int fdin, int *pid_ptr )
*   int @hor_gunzip     ( int fdin, int *pid_ptr )
*
*   hor_compress() adds a ".Z" extension to the file name and opens the file
*                  for writing. Writing to the returned file descriptor causes
*                  Unix compressed output to be written to the file.
*   hor_gzip() ditto for Gnu compression, with a ".gz" file name extension.
*   Code for hor_compress() and hor_gzip() written by Nick Cerneaz.
*
*   hor_uncompress() forks off a process to uncompress the file on fdin,
*                    and returns a stream pointer to the output of that
*                    process.
*   hor_gunzip() ditto for Gnu decompression.
*
*   Code for hor_uncompress() & hor_gunzip() adapted from chuck source code
*   (Simon Turner).
********************/
int hor_compress ( const char *file_name )
{
   char full_name[300];
   int  fd[2],pid,fdout;

   sprintf ( full_name, "%s.Z", file_name );

   fdout = creat(full_name,0644);
   if(fdout == -1)    return -1;
   if(pipe(fd))       return -1;
   if((pid=fork())<0) return -1;
  
   if(!pid) {/* we're in the child */
      if(dup2(fdout,1) == -1)
      hor_error ( "dup2(fdout,1) in child failed (hor_compress)", HOR_FATAL );

      (void) close(fdout);
    
      if(dup2(fd[0],0) == -1)
	 hor_error ( "dup2(fdout,0) in child failed (hor_compress)",
		     HOR_FATAL );

      (void) close(fd[1]);
      (void) execlp("compress","compress",0);
      _exit(127);
   }
    
   /* and back in the parent*/
   (void) close(fd[0]);
   (void) close(fdout);
   return ( fd[1] );
}

int hor_gzip ( const char *file_name )
{
   char full_name[300];
   int  fd[2],pid,fdout;

   sprintf ( full_name, "%s.gz", file_name );

   fdout = creat(full_name,0644);
   if(fdout == -1)    return -1;
   if(pipe(fd))       return -1;
   if((pid=fork())<0) return -1;
  
   if(!pid) {/* we're in the child */
      if(dup2(fdout,1) == -1)
	 hor_error ( "dup2(fdout,1) in child failed (hor_gzip)", HOR_FATAL );

      (void) close(fdout);

      if(dup2(fd[0],0) == -1)
	 hor_error ( "dup2(fdout,0) in child failed (hor_gzip)", HOR_FATAL );

      (void) close(fd[1]);
      (void) execlp("gzip","gzip",0);
      _exit(127);
   }
    
   /* and back in the parent*/
   (void) close(fd[0]);
   (void) close(fdout);
   return ( fd[1] );
}

int hor_uncompress ( int fdin, int *pid_ptr )
{
   int fd[2], pid;

   if (pipe(fd)) return -1;

#ifndef Linux
   signal ( SIGCHLD, TerminateChild );
#endif
   if ( (pid = fork()) < 0 ) return -1;

   if ( !pid )     /* child */
   {
      if (dup2(fdin, 0) == -1)
	 hor_error ( "dup2(fdin, 0) in hor_uncompress() child failed",
		     HOR_FATAL );

      (void) close(fdin);
      if (dup2(fd[1], 1) == -1)
	 hor_error ( "dup2(fd[1], 1) in hor_uncompress() child failed",
		     HOR_FATAL);

      (void) close(fd[0]);
      (void) execlp("uncompress", "uncompress", 0);
      _exit(127);
   }

   /* parent */

   (void) close(fd[1]);
   (void) close(fdin);
   *pid_ptr = pid;
   return ( fd[0] );
}

int hor_gunzip ( int fdin, int *pid_ptr )
{
   int fd[2], pid;

   if (pipe(fd))
      return -1;

#ifndef Linux
   signal ( SIGCHLD, TerminateChild );
#endif
   if ( (pid = fork()) < 0 )
      return -1;

   if ( !pid )     /* child */
   {
      if (dup2(fdin, 0) == -1)
	 hor_error ( "dup2(fdin, 0) in hor_gunzip() child failed", HOR_FATAL );

      (void) close(fdin);
      if (dup2(fd[1], 1) == -1)
	 hor_error ( "dup2(fd[1], 1) in hor_gunzip() child failed", HOR_FATAL);

      (void) close(fd[0]);
      (void) execlp("gunzip", "gunzip", 0);
      _exit(127);
   }

   /* parent */

   (void) close(fd[1]);
   (void) close(fdin);
   *pid_ptr = pid;
   return ( fd[0] );
}

#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */

