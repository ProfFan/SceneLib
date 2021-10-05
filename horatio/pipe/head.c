/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <process.h>
#include <channel.h>

#include "horatio/global.h"
#include "horatio/pipe.h"

/*******************
*   void @hor_pipe_return_stop_signal ( Channel **pipe_in,
*                                      Channel **pipe_out, int no_pipes )
*
*   Waits at head of pipelines for a HOR_PIPE_STOP char to arrive on each
*   pipeline and returns it down the pipelines to signal end of data flow.
*   For transputers only.
********************/
void hor_pipe_return_stop_signal ( Channel **pipe_in,
				   Channel **pipe_out, int no_pipes )
{
   int pipe_no;

   for ( pipe_no = 0; pipe_no < no_pipes; pipe_no++ )
   {
      /* Read in HOR_PIPE_STOP and relay it back */
      ChanInChar  ( pipe_in[pipe_no] );
      ChanOutChar ( pipe_out[pipe_no], HOR_PIPE_STOP);
   }
}
