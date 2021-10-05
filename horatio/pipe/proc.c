/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
              and Stuart Fairley (stuart@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <process.h>
#include <channel.h>
#include <string.h>
#include <semaphor.h>
#include <stdarg.h>

#include "horatio/global.h"
#include "horatio/pipe.h"

/*******************
*   static void @pipe_input ( Process *p, (process pointer)
*
*                            Channel *prev_in, (input channel from previous
*                                               processor in the pipeline)
*
*                            Channel *exec_in,  (input channel from EXEC)
*                            Channel *exec_out, (output channel to EXEC)
*
*                            void   **buffer,     (array of input buffers)
*                            int      no_buffers, (number of input buffers)
*
*                            Hor_IO_Func  input_func ) (function to input data
*                                                       from the pipeline)
*
*   Receives input data from pipeline process, writes it into
*   a buffer, and signals execution process when data is ready for processing.
*   When execution process has read signal and started to process data, the
*   buffer is swapped with another to avoid being overwritten while being
*   processed. Stop signals are also passed on to the execution process.
*   Constitutes INPUT process. For transputers only.
********************/   
static void pipe_input ( Process *p,       Channel *prev_in,
			 Channel *exec_in, Channel *exec_out,
			 void   **buffer,  int      no_buffers,
			 Hor_IO_Func  input_func )
{
   int  buf_no = 0, freq = 1, count = 0;
   char tag;

   p = p;
   for(;;)
      switch ( ChanInChar ( prev_in ) )
      {
         case HOR_PIPE_STOP: /* pipeline stop signal */

	 /* send stop signal on to execution process */
	 ChanOutChar ( exec_out, HOR_PIPE_STOP );

	 /* wait for continue signal */
	 while ( (tag = ChanInChar ( exec_in )) != HOR_PIPE_RESTART )
	    switch ( tag )
	    {
	       case HOR_PIPE_FREQ:
	       freq = ChanInInt ( exec_in );
	       count = 0;
	       break;

	       default:
	       hor_error ( "illegal tag %d (pipe_input)", HOR_FATAL, tag );
	       break;
	    }
	 break;

         case HOR_PIPE_DATA: /* receive input data */

	 /* call function to receive data into input buffer */
	 input_func ( prev_in, buffer[buf_no] );

	 if ( ++count == freq ) /* send every freq'th buffer */
	 {
	    /* send data signal and pointer to buffer to execution process */
	    ChanOutChar ( exec_out, HOR_PIPE_DATA );
	    ChanOut     ( exec_out, &buffer[buf_no], sizeof(void *) );

	    /* switch buffers to prevent overwriting */
	    if ( ++buf_no == no_buffers ) buf_no = 0;

	    count = 0;
	 }
	 break;

         case HOR_PIPE_PRINT:
	 {
	    int  length;
	    char hor_message[HOR_MAX_PIPE_COMM_LENGTH];

	    length = ChanInInt ( prev_in );
	    ChanIn ( prev_in, hor_message, length );
	    hor_print ( hor_message );
	 }
	 break;

         default:
	 hor_error ( "illegal input tag (pipe_input)", HOR_FATAL );
	 break;
      }
}

/*******************
*   static void @pipe_output ( Process *p, (process pointer)
*
*                             Channel *next_out, (output channel to next
*                                                 process in pipeline)
*                             Channel *exec_in, (input channel from EXEC)
*                             Channel *exec_out (output channel to EXEC)
*
*                             Hor_IO_Func output_func, (function to perform
*                                                       result output)
*
*                             Hor_Bool print_flag ) (will output print buffer
*                                                    if flag is HOR_TRUE)
*
*   outputs results to next process in the pipeline along channel next_out,
*   synchronising with execution process to swap processing/output buffers.
*   For transputers only.
********************/
static void pipe_output ( Process *p, Channel *next_out,
			  Channel *exec_in, Channel *exec_out,
			  Hor_IO_Func output_func, Hor_Bool print_flag )
{
   void *output_buffer;

   p = p;
   for(;;)
      switch ( ChanInChar ( exec_in ) )
      {
         case HOR_PIPE_DATA:
	 /* receive pointer to output data */
	 ChanIn ( exec_in, &output_buffer, sizeof(void *) );

	 if ( print_flag ) /* send any buffered print requests */
	    hor_pipe_print_buffer ( next_out );

	 /* send data signal down pipeline */
	 ChanOutChar ( next_out, HOR_PIPE_DATA );

	 /* output data to next process in pipeline */
	 output_func ( next_out, output_buffer );
	 break;

	 case HOR_PIPE_STOP:
	 /* send stop signal down pipeline */
	 ChanOutChar ( next_out, HOR_PIPE_STOP );

	 /* return ack to EXEC process */
	 ChanOutChar ( exec_out, 0 );

	 /* wait for continue signal */
	 ChanInChar ( exec_in );
	 break;
      }
}

/*******************
*   static void @pipe_exec (
*                  Process *p, (process pointer)
*
*                  Channel *input_in,   (input channel from INPUT process)
*                  Channel *input_out,  (output channel to INPUT process)
*                  Channel *output_in,  (input channel from OUTPUT process)
*                  Channel *output_out, (output channel to OUTPUT process)
*
*                  Channel *reinit_up_in,  (input/output channels from/to
*                  Channel *reinit_up_out,  RE-INIT/UP process)
*
*                  Channel *next_in,  (input/output channel from next/previous
*                  Channel *prev_out,  process in the pipeline)
*
*                  Channel *extra_input_in, (arrays of input/output channels
*                  Channel *extra_input_out, from/to EXTRA INPUT processes)
*                  int      extra_ins,       (number of EXTRA INPUT processes)
*
*                  Channel *extra_output_in, (arrays of input/output channels
*                  Channel *extra_output_out, from/to EXTRA OUTPUT processes)
*                  int      extra_outs.       (number of EXTRA OUTPUT
*                                              processes)
*
*                  Hor_Exec_Func exec_func ) (function to perform process
*                                             execution)
*
*   Calls pipeline process execution function, constituting EXEC process.
*   For transputers only.
********************/
static void pipe_exec ( Process  *p,
		        Channel  *input_in,         Channel *input_out,
		        Channel  *output_in,        Channel *output_out,
		        Channel  *reinit_up_in,     Channel *reinit_up_out,
		        Channel  *next_in,          Channel *prev_out,
		        void    **output_buffer,    int      no_buffers,
		        Channel  *extra_input_in,
		        Channel  *extra_input_out,  int      extra_ins,
		        Channel  *extra_output_in,
		        Channel  *extra_output_out, int      extra_outs,
		        Hor_Exec_Func exec_func )
{
   int   buf_no = 0, i;
   void *input_buf, *extra_input_buf[HOR_NO_LINKS];
   Hor_Bool  reinitialized = HOR_TRUE, valid;

   p = p;
   for(;;)
      switch ( ChanInChar ( input_in ) )
      {
         case HOR_PIPE_DATA: /* data being sent down pipeline */

	 /* receive buffers from input/extra input reception process */
	 ChanIn ( input_in, &input_buf, sizeof(void *) );
	 for ( i = 0; i < extra_ins; i++ )
	 {
	    ChanInChar ( &extra_input_in[i] ); /* HOR_PIPE_DATA */
	    ChanIn ( &extra_input_in[i], &extra_input_buf[i], sizeof(void *) );
	 }

	 /* process input data into output data, returning flag determining
	    whether to output result */
	 valid = exec_func ( input_buf, output_buffer[buf_no],
			     extra_input_buf, extra_ins, reinitialized,
			     reinit_up_in, reinit_up_out, next_in, prev_out);

	 if ( valid )
	 {
	    /* send output buffer to output and extra output processes */
	    ChanOutChar ( output_out, HOR_PIPE_DATA );
	    ChanOut     ( output_out, &output_buffer[buf_no], sizeof(void *) );
	    for ( i = 0; i < extra_outs; i++ )
	    {
	       ChanOutChar ( &extra_output_out[i], HOR_PIPE_DATA );
	       ChanOut     ( &extra_output_out[i], &output_buffer[buf_no],
			     sizeof(void *) );
	    }
	 }

	 /* switch buffers */
	 if ( ++buf_no == no_buffers ) buf_no = 0;

	 reinitialized = HOR_FALSE;
	 break;

         case HOR_PIPE_STOP: /* stop signal being sent down pipeline */

	 /* receive stop signals from extra input processes */
	 for ( i = 0; i < extra_ins; i++ )
	    ChanInChar ( &extra_input_in[i] );

	 /* send stop signal to output process and wait for ack. */
	 ChanOutChar ( output_out, HOR_PIPE_STOP );
	 ChanInChar  ( output_in );

	 /* send stop signals to extra output processes and wait for
	    acknowledgement */
	 for ( i = 0; i < extra_outs; i++ )
	 {
	    ChanOutChar ( &extra_output_out[i], HOR_PIPE_STOP );
	    ChanInChar  ( &extra_output_in[i] );
	 }

	 /* signal reinitialization process that pipeline has stopped */
	 ChanOutChar ( reinit_up_out, HOR_PIPE_STOP );

	 /* receive continue signal, indicating that pipeline has been
	    reinitialized */
	 ChanInChar ( reinit_up_in );

	 /* signal input processes to continue */
	 ChanOutChar ( input_out, HOR_PIPE_RESTART );
	 for ( i = 0; i < extra_ins; i++ )
	    ChanOutChar ( &extra_input_out[i], HOR_PIPE_RESTART );

	 /* signal output processes to continue */
	 ChanOutChar ( output_out, HOR_PIPE_RESTART );
	 for ( i = 0; i < extra_outs; i++ )
	    ChanOutChar ( &extra_output_out[i], HOR_PIPE_RESTART );

	 /* set flag so that execution process will know that pipeline has
	    just been reinitialized */
	 reinitialized = HOR_TRUE;
	 break;

         default:
	 hor_error ( "illegal input tag (pipe_exec)", HOR_FATAL );
	 break;
      }
}

/*******************
*   void @hor_pipe_control (
*           Channel *prev_in,  (channel input from previous pipeline process)
*           Channel *prev_out, (channel output to previous pipeline process)
*           Channel *next_in,  (channel input from next pipeline process)
*           Channel *next_out, (channel output to next pipeline process)
*
*           int      no_io_buffers, (number of I/O buffers)
*
*           Hor_Reinit_Func reinit_func, (function to reinitialize processor
*                                         (RE-INIT))
*           Hor_Up_Func     up_func,     (function to read input from next
*                                         pipeline process during normal
*                                         operation (UP))
*
*           Hor_Exec_Func exec_func,   (function to execute process (EXEC))
*           int           exec_stack,  (stack space for EXEC process)
*
*           Hor_IO_Func input_func,  (function to input data on prev_in
*                                     (INPUT))
*           int         input_stack, (stack space for INPUT process)
*           void      **in_buffer,   (array of input buffers)
*
*           Hor_IO_Func output_func, (function to output results on next_out
*                                     (OUTPUT))
*           int         output_stack, (stack space for OUTPUT process)
*           void      **out_buffer,   (array of output buffers)
*
*           ... ) (NULL-terminated list of extra inputs, followed by
*                  a NULL-terminated list of extra outputs.
*                  Each extra input is specified by five arguments:
*                    1,2: input/output channels from/to extra input processor.
*                    3:   function to input data on input channel
*                         (EXTRA INPUT).
*                    4:   stack space for EXTRA INPUT process.
*                    5:   input buffer to read into.
*                  Each extra output is specified bu four arguments.
*                    1,2: input/output channels from/to extra output processor.
*                    3:   function to output data on output channel
*                         (EXTRA OUTPUT).
*                    4:   stack space for EXTRA OUTPUT process.)
*
*   Controls pipeline. Runs processes to do reinitialization, input,
*   processing and output on a pipeline processor. no_io_buffers will normally
*   be two, except when input and output processes are to share memory, when
*   it should be set to three. For transputers only.
********************/
void hor_pipe_control (Channel *prev_in, Channel *prev_out,
		       Channel *next_in, Channel *next_out,
		       int      no_io_buffers,
		       Hor_Reinit_Func reinit_func, Hor_Up_Func up_func,
		       Hor_Exec_Func   exec_func,  int   exec_stack,
		       Hor_IO_Func     input_func, int  input_stack,
		       void  **in_buffer,
		       Hor_IO_Func    output_func, int output_stack,
		       void **out_buffer, ... )
{
   Channel  reinit_up_exec1, reinit_up_exec2;
   Channel  input_exec1,     input_exec2;
   Channel  output_exec1,    output_exec2;
   Process *p1, *p2, *p3;

   int         extra_inputs = 0,               extra_outputs = 0, i;
   Channel    *extra_input_in[HOR_NO_LINKS],  *extra_output_in[HOR_NO_LINKS];
   Channel    *extra_input_out[HOR_NO_LINKS], *extra_output_out[HOR_NO_LINKS];
   Hor_IO_Func extra_input_func[HOR_NO_LINKS], extra_output_func[HOR_NO_LINKS];
   int      extra_input_stack[HOR_NO_LINKS], extra_output_stack[HOR_NO_LINKS];
   void   **extra_input_buffer[HOR_NO_LINKS];
   Channel  extra_input_exec1[HOR_NO_LINKS], extra_output_exec1[HOR_NO_LINKS];
   Channel  extra_input_exec2[HOR_NO_LINKS], extra_output_exec2[HOR_NO_LINKS];
   char     tag;
   va_list  ap;

   /* read extra I/O info */
   va_start ( ap, out_buffer );

   /* read extra input functions, stack sizes and data buffers */
   for(;;)
   {
      extra_input_in[extra_inputs]     = va_arg ( ap, Channel * );
      if ( extra_input_in[extra_inputs] == NULL ) break;

      extra_input_out[extra_inputs]    = va_arg ( ap, Channel * );
      extra_input_func[extra_inputs]   = va_arg ( ap, Hor_IO_Func );
      extra_input_stack[extra_inputs]  = va_arg ( ap, int );
      extra_input_buffer[extra_inputs] = va_arg ( ap, void ** );
      ChanInit ( &extra_input_exec1[extra_inputs] );
      ChanInit ( &extra_input_exec2[extra_inputs] );
      extra_inputs++;
   }

   /* read extra output functions, stack sizes and data buffers */
   for(;;)
   {
      extra_output_in[extra_outputs]    = va_arg ( ap, Channel * );
      if ( extra_output_in[extra_outputs] == NULL ) break;

      extra_output_out[extra_outputs]   = va_arg ( ap, Channel * );
      extra_output_func[extra_outputs]  = va_arg ( ap, Hor_IO_Func );
      extra_output_stack[extra_outputs] = va_arg ( ap, int );
      ChanInit ( &extra_output_exec1[extra_outputs] );
      ChanInit ( &extra_output_exec2[extra_outputs] );
      extra_outputs++;
   }

   va_end(ap);

   /* set print function to send hor_message down pipeline */
   hor_pipe_set_print_func();

   /* initialize channels to be used for internal communication */
   ChanInit ( &reinit_up_exec1 ); ChanInit ( &reinit_up_exec2 );
   ChanInit ( &input_exec1 );     ChanInit ( &input_exec2 );
   ChanInit ( &output_exec1 );    ChanInit ( &output_exec2 );

   /* create execution process that does all the data processing */
   p1 = ProcAlloc ( pipe_exec, exec_stack, 17,
		    &input_exec1,  &input_exec2, &output_exec1, &output_exec2,
		    &reinit_up_exec1, &reinit_up_exec2, next_in, prev_out,
		    out_buffer, no_io_buffers,
		    extra_input_exec1,  extra_input_exec2,  extra_inputs,
		    extra_output_exec1, extra_output_exec2, extra_outputs,
		    exec_func );

   /* create process that receives data from further up the pipeline */
   p2 = ProcAlloc ( pipe_input, input_stack, 6,
		    prev_in, &input_exec2, &input_exec1,
		    in_buffer, no_io_buffers, input_func );

   /* create process that outputs data down the pipeline */
   p3 = ProcAlloc ( pipe_output, output_stack, 5,
		    next_out, &output_exec2, &output_exec1,
		    output_func, HOR_TRUE );

   if ( (p1 == NULL) || (p2 == NULL) || (p3 == NULL) )
      hor_error ( "process allocation failed (hor_pipe_control)", HOR_FATAL );

   /* start processes running in parallel with re-initialization process */
   HorProcRunPar ( p1, p2, p3, NULL );

   /* start extra input processes */
   for ( i = 0; i < extra_inputs; i++ )
   {
      p1 = ProcAlloc ( pipe_input, extra_input_stack[i], 6,
		       extra_input_in[i],
		       &extra_input_exec2[i], &extra_input_exec1[i],
		       extra_input_buffer[i], no_io_buffers,
		       extra_input_func[i] );
      if ( p1 == NULL )
	 hor_error ( "extra input process allocate failed (hor_pipe_control)",
		     HOR_FATAL );
		       
      ProcRun ( p1 );
   }

   /* start extra output processes */
   for ( i = 0; i < extra_outputs; i++ )
   {
      p1 = ProcAlloc ( pipe_output, extra_output_stack[i], 5,
		       extra_output_out[i],
		       &extra_output_exec2[i], &extra_output_exec1[i],
		       extra_output_func[i], HOR_FALSE );
      if ( p1 == NULL )
	 hor_error ( "extra output process allocate failed (hor_pipe_control)",
		     HOR_FATAL);
		       
      ProcRun ( p1 );
   }

   for(;;)
      if ( ProcAlt ( next_in, &reinit_up_exec2, NULL ) == 0 )
	 /* read stop signal or pipeline data from down pipeline */
	 if ( (tag = ChanInChar ( next_in )) == HOR_PIPE_STOP )
	 {
	    ChanOutChar ( prev_out, HOR_PIPE_STOP );

	    /* wait for stop signal from execution process */
	    ChanInChar ( &reinit_up_exec2 );

	    /* reinitialize pipeline */
	    reinit_func ( prev_in, prev_out, next_in, next_out,
			  in_buffer, out_buffer, no_io_buffers,
			  extra_output_in, extra_output_out, extra_outputs );

	    /* signal execution process to receive data down pipeline */
	    ChanOutChar ( &reinit_up_exec1, HOR_PIPE_RESTART );
	 }
	 else
	    up_func ( &reinit_up_exec2, &reinit_up_exec1, next_in, prev_out,
		      tag );
      else
      {
	 /* input from EXEC process: synchronises use of channel prev_out */
	 ChanInChar ( &reinit_up_exec2 );
	 ChanInChar ( &reinit_up_exec2 );
      }
}
