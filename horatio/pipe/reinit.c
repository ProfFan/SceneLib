/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <process.h>
#include <channel.h>

#include "horatio/global.h"
#include "horatio/pipe.h"

/* pipeline hor_message:

int   tag:    identifies destination process and hor_message type.
int   length: length of hor_message in bytes.
char *buffer: hor_message.
*/

/*******************
*   void @hor_pipe_message ( Channel *pipe_out, int tag,
*                           char    *buffer,   int length )
*
*   sends message to previous process in pipeline (towards head).
********************/
void hor_pipe_message ( Channel *pipe_out, int tag,
		        char    *buffer,   int length )
{
   ChanOutInt ( pipe_out, tag );
   ChanOutInt ( pipe_out, length );
   if ( length > 0 ) ChanOut ( pipe_out, buffer, length );
}

/*******************
*   int @hor_pipe_wait_for_value ( Channel *prev_in, Channel *prev_out,
*                                 Channel *next_in, Channel *next_out,
*                                 int      wait_tag )
*
*   passes messages up and down a pipeline, waiting for get ready signal
*   HOR_PIPE_RESTART or specified argument wait_tag to be read from the down
*   side on channel next_in. The value received is returned.
********************/
int hor_pipe_wait_for_value ( Channel *prev_in, Channel *prev_out,
			      Channel *next_in, Channel *next_out,
			      int      wait_tag )
{
   char buffer[HOR_MAX_PIPE_COMM_LENGTH];
   int  tag, length;

   /* wait for pipeline communication */
   for(;;)
      if ( ProcAlt ( prev_in, next_in, NULL ) == 0 )
      /* input on prev_in: must be an ack from further up the pipeline */
         ChanOutInt ( next_out, ChanInInt ( prev_in ) );
      else /* input on next_in */
      {
	 tag = ChanInInt ( next_in );
	 if ( tag == HOR_PIPE_RESTART )
	 {
	    ChanOutInt ( prev_out, HOR_PIPE_RESTART );
	    return HOR_PIPE_RESTART;
	 }

	 length = ChanInInt ( next_in );
	 if ( tag == wait_tag ) return wait_tag;
	 if ( length > 0 ) ChanIn ( next_in, buffer, length );

	 hor_pipe_message ( prev_out, tag, buffer, length );
      }
}

/*******************
*   int @hor_pipe_head_wait_for_value ( Channel **next_in, int no_pipelines,
*                                      int       wait_tag )
*
*   Runs on head processor on pipeline, waits for HOR_PIPE_RESTART or specified
*   argument wait_tag to be read from the down side on channel next_in.
*   The value received is returned.
********************/
int hor_pipe_head_wait_for_value ( Channel **next_in,
				   int       no_pipelines,
				   int       wait_tag )
{
   int pipe_no, tag, new_tag, length, new_length;

   /* wait for pipeline communication */
   tag = ChanInInt ( next_in[0] );
   for ( pipe_no = 1; pipe_no < no_pipelines; pipe_no++ )
      if ( (new_tag = ChanInInt ( next_in[pipe_no])) != tag )
         hor_error ( "tags not the same %d %d (hor_pipe_head_wait_for_value)",
		     HOR_FATAL, new_tag, tag );

   if ( tag == HOR_PIPE_RESTART ) return HOR_PIPE_RESTART;

   length = ChanInInt ( next_in[0] );
   for ( pipe_no = 1; pipe_no < no_pipelines; pipe_no++ )
      if ( (new_length = ChanInInt ( next_in[pipe_no] )) != length )
         hor_error("lengths not the same %d %d (hor_pipe_head_wait_for_value)",
		   HOR_FATAL, new_length, length );

   if ( tag == wait_tag ) return wait_tag;
   hor_error ( "illegal tag %d received (hor_pipe_head_wait_for_value)",
	       HOR_FATAL, tag );
}
