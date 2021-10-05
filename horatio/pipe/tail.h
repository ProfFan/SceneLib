/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/tail.h */

/*******************
*   typedef void (*@Hor_Merge_Func)(void *pipe_input_buf, void *input_buf,
*                                  int pipe_no, Hor_Bool last_pipe,
*                                  void *merge_params,
*                                  Hor_Bool first_buf_merge);
*   typedef void (*@Hor_Tail_Exec_Func)(void    *input_buf,
*                                      Channel *reinit_in, Channel *reinit_out,
*                                      Hor_Bool reinitialized,
*                                      Channel **pipe_out, int no_pipelines);
*   typedef void (*@Hor_Tail_Reinit_Func)(Channel  *merge_exec_in,
*                                   Channel  *merge_exec_out,
*                                   Channel **pipe_in,
*                                   Channel **pipe_out,    int no_pipelines,
*                                   void    **buffer,      int no_buffers,
*                                   void   ***pipe_buffer, int no_pipe_buffers,
*                                   void    **merge_params );
*
*   Pipeline tail processor control function definitions.
********************/
typedef void (*Hor_Merge_Func)(void *pipe_input_buf, void *input_buf,
			       int pipe_no, Hor_Bool last_pipe,
			       void *merge_params, Hor_Bool reinitialized);
typedef void (*Hor_Tail_Exec_Func)(void     *input_buf,
				   Channel  *reinit_in, Channel *reinit_out,
				   Hor_Bool  reinitialized,
				   Channel **pipe_out, int no_pipelines);
typedef void (*Hor_Tail_Reinit_Func)(Channel  *merge_exec_in,
				    Channel  *merge_exec_out,
				    Channel **pipe_in,
				    Channel **pipe_out,    int no_pipelines,
				    void    **buffer,      int no_buffers,
				    void   ***pipe_buffer, int no_pipe_buffers,
				    void    **merge_params );

void hor_pipe_tail_control ( Channel **pipe_in,
		       Channel **pipe_out,    int no_pipelines,
		       void    **buffer,      int no_buffers,
		       void   ***pipe_buffer, int no_pipe_buffers,
		       void    **merge_params, Hor_Bool ordered_inputs,
		       Hor_IO_Func          input_func,   int input_stack,
		       Hor_Merge_Func       merge_func,
		       Hor_Tail_Exec_Func   execute_func, int merge_exec_stack,
		       Hor_Tail_Reinit_Func reinit_func );

void hor_pipe_stop ( Channel **pipe_out, int no_pipelines, Channel *merge_in );
