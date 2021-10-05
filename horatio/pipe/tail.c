/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University.
		  Code for dealing with unsynchronised pipelines:
		  Stuart Fairley (stuart@robots.oxford.ac.uk) 28/5/94 */
#include <stdlib.h>
#include <process.h>
#include <channel.h>
#include <misc.h>

#include "horatio/global.h"
#include "horatio/pipe.h"

/*******************
*   static void @pipe_tail_input (
*                  Process *p,           (process pointer)
*                  Channel *pipe_in,     (pipeline input channel)
*                  Channel *merge_in,    (input channel from MERGE)
*                  Channel *merge_out,   (output channel to MERGE)
*                  void   **buffer,      (array of input buffers)
*                  int      no_buffers,  (number of buffers in the buffer
*                                         array)
*                  Hor_IO_Func input_func ) (function to call to perform
*                                            data input into pipe_buffer)
*
*   reads input data and signals merge process. For transputers only.
********************/
static void pipe_tail_input ( Process *p,        Channel *pipe_in,
			      Channel *merge_in, Channel *merge_out,
			      void   **buffer,   int      no_buffers,
			      Hor_IO_Func  input_func )
{
   int  buf_no = 0, freq = 1, count = 0;
   char string[HOR_MAX_PIPE_COMM_LENGTH], tag;
   int  length;

   p = p;
   for(;;)
      switch ( ChanInChar ( pipe_in ) )
      {
         case HOR_PIPE_STOP:
	 /* signal merge process that data flow has stopped*/
	 ChanOutChar ( merge_out, HOR_PIPE_STOP );

	 /* wait for signal to restart */
	 while ( (tag = ChanInChar ( merge_in )) != HOR_PIPE_RESTART )
	    switch ( tag )
	    {
	       case HOR_PIPE_FREQ:
	       freq = ChanInInt ( merge_in );
	       count = 0;
	       break;

	       default:
	       hor_error ( "illegal tag %d (pipe_tail_input)", HOR_FATAL, tag);
	       break;
	    }
	 break;

         case HOR_PIPE_PRINT: /* print request from pipeline process */
	 length = ChanInInt ( pipe_in );
	 ChanIn ( pipe_in, string, length );
	 hor_print ( string );
	 break;

         case HOR_PIPE_DATA: /* data being sent down pipeline */

	 /* receive input data */
	 input_func ( pipe_in, buffer[buf_no] );

	 if ( ++count == freq ) /* send every freq'th buffer onto merge */
	 {
	    /* send merge process a pointer to the input data */
	    ChanOutChar ( merge_out, HOR_PIPE_DATA );
	    ChanOut     ( merge_out, &buffer[buf_no], sizeof(void *) );

	    /* switch input buffer */
	    if ( ++buf_no == no_buffers ) buf_no = 0;

	    count = 0;
	 }
	 break;

         default:
	 hor_error ( "illegal hor_message received (pipe_tail_input)", HOR_FATAL );
	 break;
      }
}

/*******************
*   static void @pipe_tail_merge_exec (
*                  Process  *p,            (process pointer)
*                  Channel  *reinit_in,    (input channel from RE-INIT)
*                  Channel  *reinit_out,   (output channel to RE-INIT)
*                  Channel **input_in,     (input channels from INPUT
*                                           processes)
*                  Channel **input_out,    (output channels to INPUT processes)
*                  Channel **pipe_out,     (pipeline output channels)
*                  int       no_pipelines, (number of pipelines in the
*                                           pipe-group)
*                  void    **buffer,       (array of buffers to merge into)
*                  int       no_buffers,   (number of merge buffers)
*                  void    **merge_params, (parameters to merge input data from
*                                           each pipeline)
*                  Hor_Bool  ordered,      (HOR_TRUE if inputs are required to
*                                           taken in the order they appear in
*                                           input_in, HOR_FALSE otherwise)
*
*                  Hor_Merge_Func merge_func, (function to merge input buffers)
*                  Hor_Tail_Exec_Func exec_func ) (process execution function)
*
*   receives input data from pipelines, merges it into a single representation,
*   executes data processing. For transputers only.
********************/
static void pipe_tail_merge_exec ( Process  *p,
				   Channel  *reinit_in, Channel  *reinit_out,
				   Channel **input_in,  Channel **input_out,
				   Channel **pipe_out,  int       no_pipelines,
				   void    **buffer,    int       no_buffers,
				   void    **merge_params, Hor_Bool   ordered,
				   Hor_Merge_Func     merge_func,
				   Hor_Tail_Exec_Func exec_func )
{
   int      buf_no = 0, pipe_no = -1, no_sent = 0;
   Channel *null_channel = ChanAlloc(), **input_in_list;
   void    *input_buf;
   Hor_Bool last_pipe, reinitialized = HOR_TRUE, first_buf_merge = HOR_TRUE;
   char     tag;


   int no_stops_sent = 0;
   Hor_Bool stopping = HOR_FALSE;
   Channel **stop_input_in_list;

   p = p;

   /* create NULL-terminated input channel list */
   input_in_list      = hor_malloc_ntype ( Channel *, no_pipelines+1 );
   stop_input_in_list = hor_malloc_ntype ( Channel *, no_pipelines+1 );
   HorChanListReset ( input_in_list, input_in, no_pipelines );

   for(;;)
   {
      /* get signal from an input process */
      if ( !ordered ) /* take inputs in the order they come */
	 pipe_no = ProcAltList ( input_in_list );
      else /* force ordering on input processes */
	 if ( ++pipe_no == no_pipelines ) pipe_no = 0;

      switch ( ChanInChar ( input_in_list[pipe_no] ) )
      {
         case HOR_PIPE_DATA:

	 /* get pointer to new input data */
	 ChanIn ( input_in_list[pipe_no], &input_buf, sizeof(void *) );

	 /* SF: data has been fully read in by input process and
	    received here - but if we have got a HOR_PIPE_STOP from another
	    pipeline, we want to ignore this data, so no merge, no increment
	    of no_sent, no setting the channel to null_channel, no exec, no
	    reset of counts etc. Could there be any problems if one merge is
	    done and not the next? */
	 if ( stopping ) continue;

	 /* determine whether this is the last pipeline to send data */
	 if ( no_sent < no_pipelines-1 ) last_pipe = HOR_FALSE;
	 else                            last_pipe = HOR_TRUE;

	 /* call procedure to merge new data into single representation */
	 if ( merge_params == NULL )
	    merge_func ( input_buf, buffer[buf_no], pipe_no, last_pipe,
			 NULL, first_buf_merge );
	 else
	    merge_func ( input_buf, buffer[buf_no], pipe_no, last_pipe,
			 merge_params[pipe_no], first_buf_merge );

	 /* set input channel so it will be ignored by next ProcAltList() */
	 input_in_list[pipe_no] = null_channel;

	 /* check if all pipelines have sent their data */
	 if ( ++no_sent < no_pipelines ) continue;

	 /* execute data processing/output */
	 exec_func ( buffer[buf_no], reinit_in, reinit_out, reinitialized,
		     pipe_out, no_pipelines);

	 /* reset input channels and counter */
	 HorChanListReset ( input_in_list, input_in, no_pipelines );
	 no_sent = 0;

	 /* switch input buffer */
	 if ( ++buf_no == no_buffers )
	 {
	    buf_no = 0;
	    first_buf_merge = HOR_FALSE;
	 }

	 reinitialized = HOR_FALSE;
	 break;

         case HOR_PIPE_STOP:

	 /* SF: if this is the first HOR_PIPE_STOP, we must re-enable all
	    inputs except that on which the HOR_PIPE_STOP has come in on */
	 if ( !stopping )
	     HorChanListReset ( stop_input_in_list, input_in, no_pipelines );

	 /* set input channel so it will be ignored by next ProcAltList():
	    modified by SF 28/5/94 */
	 stop_input_in_list[pipe_no] = null_channel;

	 /* allow input only on those channels which have not already sent an
	    HOR_PIPE_STOP */
	 HorChanListReset ( input_in_list, stop_input_in_list, no_pipelines );

	 stopping = HOR_TRUE;

	 /* check if all pipelines have sent stop signals */
	 if ( ++no_stops_sent < no_pipelines ) continue;

	 /* signal reinitialization process that all pipelines have stopped */
	 ChanOutChar ( reinit_out, 0 );

	 /* wait for signal to restart */
	 while ( (tag = ChanInChar ( reinit_in )) != HOR_PIPE_RESTART )
	    switch ( tag )
	    {
	       case HOR_PIPE_FREQ:
	       {
		  int freq = ChanInInt ( reinit_in );

		  for ( pipe_no = 0; pipe_no < no_pipelines; pipe_no++ )
		  {
		     ChanOutChar ( input_out[pipe_no], HOR_PIPE_FREQ );
		     ChanOutInt  ( input_out[pipe_no], freq );
		  }
	       }
	       break;

	       default:
	       hor_error ( "illegal tag %d (pipe_tail_merge_exec)",
			   HOR_FATAL, tag );
	       break;
	    }

	 /* signal input processes to restart */
	 for ( pipe_no = 0; pipe_no < no_pipelines; pipe_no++ )
            ChanOutChar ( input_out[pipe_no], HOR_PIPE_RESTART );

	 /* reset input channels and counter */
	 HorChanListReset ( input_in_list, input_in, no_pipelines );
	 no_sent = 0;

	 /* Addition by SF */
	 no_stops_sent = 0;
	 stopping = HOR_FALSE;

	 reinitialized = first_buf_merge = HOR_TRUE;
	 buf_no = 0;
	 if ( ordered ) pipe_no = -1;
	 break;

         default:
	 hor_error ( "illegal tag (pipe_merge_exec)", HOR_FATAL );
	 break;
      }
   }
}

/*******************
*   void @hor_pipe_tail_control (
*           Channel **pipe_in,      (pipeline input channels)
*           Channel **pipe_out,     (pipeline output channels)
*           int       no_pipelines, (the number of pipelines in the pipe-group)
*
*           void    **buffer,     (array of input buffers for MERGE/EXEC to
*                                  write into (MERGE) and read (EXEC))
*           int       no_buffers, (number of buffers in the buffer array)
*
*           void   ***pipe_buffer,     (array of arrays of buffers, one array
*                                       for each pipeline)
*           int       no_pipe_buffers, (number of input buffers per pipelines)
*           void    **merge_params,    (parameters to merge input data from
*                                       each pipeline, one pointer per
*                                       pipeline, to be passed to merge_func()
*                                       when it is called)
*
*           Hor_Bool  ordered_inputs, (HOR_TRUE if inputs are required to taken
*                                      in the order they appear in pipe_in,
*                                      HOR_FALSE otherwise)
*
*           Hor_IO_Func          input_func,       (function to call to perform
*                                                   data input into pipe_buffer
*                                                   (INPUT process))
*           int                  input_stack,      (stack space for INPUT
*                                                   process)
*           Hor_Merge_Func       merge_func,       (function to merge input
*                                                   buffers (MERGE))
*           Hor_Tail_Exec_Func   execute_func,     (process execution function
*                                                   (EXEC))
*           int                  merge_exec_stack, (stack space for MERGE/EXEC)
*           Hor_Tail_Reinit_Func reinit_func )     (re-initialization function
*                                                   (RE-INIT process))
*
*   Starts up INPUT and MERGE_EXEC processes on pipe tail processor,
*   calls RE-INIT function which should not exit. For transputers only.
********************/
void hor_pipe_tail_control ( Channel **pipe_in,
			     Channel **pipe_out,    int    no_pipelines,
			     void    **buffer,      int    no_buffers,
			     void   ***pipe_buffer, int    no_pipe_buffers,
			     void    **merge_params, Hor_Bool ordered_inputs,
			     Hor_IO_Func          input_func,
			     int                  input_stack,
			     Hor_Merge_Func       merge_func,
			     Hor_Tail_Exec_Func   execute_func,
			     int merge_exec_stack,
			     Hor_Tail_Reinit_Func reinit_func )
{
   Process  *p;
   Channel **input_merge1,     **input_merge2;
   Channel   merge_exec_reinit1, merge_exec_reinit2;
   int       pipe_no;

   /* create channels for internal communication */
   input_merge1 = hor_malloc_ntype ( Channel *, no_pipelines+1 );
   input_merge2 = hor_malloc_ntype ( Channel *, no_pipelines+1 );
   for ( pipe_no = 0; pipe_no < no_pipelines; pipe_no++ )
   {
      input_merge1[pipe_no] = ChanAlloc();
      input_merge2[pipe_no] = ChanAlloc();
   }

   /* NULL-terminate channels between merge and input processes */
   input_merge1[no_pipelines] = input_merge2[no_pipelines] = NULL;

   ChanInit ( &merge_exec_reinit1 );
   ChanInit ( &merge_exec_reinit2 );

   /* start input processes */
   for ( pipe_no = 0; pipe_no < no_pipelines; pipe_no++ )
   {
      p = ProcAlloc ( pipe_tail_input, input_stack, 6,
		      pipe_in[pipe_no],
		      input_merge1[pipe_no], input_merge2[pipe_no],
                      pipe_buffer[pipe_no], no_pipe_buffers, input_func );
      if ( p == NULL )
	 hor_error("can't alloc. space for input proc.(hor_pipe_tail_control)",
		   HOR_FATAL );

      ProcRun ( p );
   }

   /* start processes that merge input data and execute data processing
      and that which relays stop/restart signals */
   p = ProcAlloc ( pipe_tail_merge_exec, merge_exec_stack, 12,
		   &merge_exec_reinit1, &merge_exec_reinit2,
		   input_merge2, input_merge1, pipe_out, no_pipelines,
		   buffer, no_buffers, merge_params, ordered_inputs,
		   merge_func, execute_func );
   if ( p == NULL )
      hor_error("cannot allocate space for processes (hor_pipe_tail_control)",
		HOR_FATAL );

   ProcRun ( p );

   /* call reinitialization function, which should not terminate */
   reinit_func ( &merge_exec_reinit2, &merge_exec_reinit1,
		 pipe_in, pipe_out, no_pipelines,
		 buffer,            no_buffers,
		 pipe_buffer,       no_pipe_buffers, merge_params );

   hor_error ( "reinitialization function terminated (hor_pipe_tail_control)",
	       HOR_FATAL );
}

/*******************
*   void @hor_pipe_stop (
*           Channel **pipe_out,     (pipeline output channels)
*           int       no_pipelines, (number of pipelines in the pipe-group)
*           Channel  *merge_in )    (input channel from MERGE_EXEC)
*
*   Stops pipelines prior to reinitialization. For transputers only.
********************/
void hor_pipe_stop ( Channel **pipe_out, int no_pipelines, Channel *merge_in )
{
   int pipe_no;

   /* send stop signals towards start of pipelines */
   for ( pipe_no = 0; pipe_no < no_pipelines; pipe_no++ )
      ChanOutChar ( pipe_out[pipe_no], HOR_PIPE_STOP );

   /* wait for stop ack merge process */
   ChanInChar ( merge_in );
}
