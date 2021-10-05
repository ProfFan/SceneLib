/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <process.h>
#include <channel.h>
#include <stdarg.h>

#include "horatio/global.h"

/*******************
*   void @HorProcRunPar ( Process *p1, ... )
*   void @HorProcRunList ( Process **plist )
*
*   Versions of ProcRun() for multiple parallel processes.
*   For transputers only.
*
*   HorProcRunPar() starts up the given NULL-terminated list of processes.
*   HorProcRunList() takes a NULL-terminated array instead of a list.
********************/
void HorProcRunPar ( Process *p1, ... )
{
   va_list  ap;
   Process *p;

   va_start ( ap, p1 );
   for ( p = p1; p != NULL; p = va_arg ( ap, Process * ) )
      ProcRun(p);

   va_end ( ap );
}

void HorProcRunList ( Process **plist )
{
   for ( ; *plist != NULL; plist++ ) ProcRun(*plist);
}

/*******************
*   Channel **@HorChanList      ( Channel **old_list, int no_channels )
*   void      @HorChanListReset ( Channel **chan_dest, Channel **chan_source,
*                                int       no_channels )
*
*   HorChanList() takes an array of channels and copies it into a
*                 null-terminated array. Useful to convert arrays passed
*                 in to main() into a form usable for ProcAltList().
*   HorChanListReset() copies an array of channels and NULL-terminates
*                      the copy. Both functions are for transputers only.
********************/
Channel **HorChanList ( Channel **old_list, int no_channels )
{
   Channel **new_list;
   int       i;

   if ( no_channels <= 0 )
      return NULL;

   new_list = hor_malloc_ntype ( Channel *, no_channels+1 );
   for ( i = 0; i < no_channels; i++ )
      new_list[i] = old_list[i];

   new_list[no_channels] = NULL;
   return new_list;
}

void HorChanListReset ( Channel **chan_dest, Channel **chan_source,
		        int no_channels )
{
   int pipe_no;

   for ( pipe_no = 0; pipe_no < no_channels; pipe_no++ )
      chan_dest[pipe_no] = chan_source[pipe_no];

   chan_dest[no_channels] = NULL;
}

/*******************
*   void @hor_block_move_2D ( int sstride, int dstride, int rows,
*                            void *saddr, void *daddr, int width )
*
*   Transputer assembler routine to copy 2D block of memory.
********************/
void hor_block_move_2D ( int sstride, int dstride, int rows,
			 void *saddr, void *daddr, int width )
{
    __asm
    {
	ldl sstride;        /* source stride in bytes stored in Creg         */
	ldl dstride;        /* destination stride in bytes stored in Breg    */
	ldl rows;           /* number of rows to be copied stored in Areg    */
	move2dinit;         /* memorises these first 3 parameters            */
	
        ldl saddr;          /* source address stored in Creg                 */
	ldl daddr;          /* destination address stored in Breg            */
	ldl width;          /* number of columns to be copied stored in Areg */
	move2dall;    /* use the above 3 parameters and perform a block move */
                      /* from the source address to the destination address  */
    }
}

/*******************
*   void @HorChanInChar ( Channel *chan, char *number )
*   void @HorChanInInt  ( Channel *chan, int  *number )
*
*   Horatio versions of Inmos C functions ChanInChar() and ChanInInt(), which
*   take pointers as arguments rather than returning the read value directly.
*   These should be used when you want to debug the code and then switch to
*   the macro versions by compiling with the -DHOR_CHANNEL_MACRO flag, since
*   there are no macro versions of ChanInChar() and ChanInInt(), while there
*   are macro versions of the Hor... versions. For transputers only.
********************/
void HorChanInChar ( Channel *chan, char *number )
{
   ChanIn ( chan, number, sizeof(char) );
}

void HorChanInInt ( Channel *chan, int *number )
{
   ChanIn ( chan, number, sizeof(int) );
}
