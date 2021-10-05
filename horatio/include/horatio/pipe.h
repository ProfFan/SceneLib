/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_PIPE_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/head.h */

#define hor_pipe_data_signal(c) ChanOutChar(c,HOR_PIPE_DATA)

void hor_pipe_return_stop_signal ( Channel **pipe_in,
				   Channel **pipe_out, int no_pipes );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/proc.h */

/*******************
*   @HOR_PIPE_STOP, @HOR_PIPE_RESTART, @HOR_PIPE_DATA, @HOR_PIPE_PRINT, @HOR_PIPE_FREQ
*
*   Pipeline communication prefix tags. For transputers only.
********************/
#define HOR_PIPE_STOP     0
#define HOR_PIPE_RESTART  1
#define HOR_PIPE_DATA     2
#define HOR_PIPE_PRINT    3
#define HOR_PIPE_FREQ     4

/*******************
*  @HOR_MAX_PIPE_COMM_LENGTH: maximum length of pipe hor_message.
********************/
#define HOR_MAX_PIPE_COMM_LENGTH 1024

/*******************
*  typedef void (*@Hor_Reinit_Func)(Channel *prev_in,   Channel *prev_out,
*                              Channel *next_in,   Channel *next_out,
*                              void   **in_buffer, void   **out_buffer,
*                              int      no_io_buffers,
*                              Channel *extra_next_in[HOR_NO_LINKS],
*                              Channel *extra_next_out[HOR_NO_LINKS],
*                              int extra_outs);
*   typedef void (*@Hor_Up_Func)(Channel *exec_in, Channel *exec_out,
*                           Channel *next_in, Channel *prev_out, char tag);
*   typedef Hor_Bool (*@Hor_Exec_Func)(void *input_buf, void *output_buf,
*                             void **extra_input_buf, int extra_ins,
*                             Hor_Bool reinitialised,
*                             Channel *reinit_up_in, Channel *reinit_up_out,
*                             Channel *next_in,      Channel *prev_out);
*   typedef void (*@Hor_IO_Func)(Channel *pipe_in, void *input_buf);
*
*   Function types for pipeline processor control functions.
*   For transputers only.
********************/
typedef void (*Hor_Reinit_Func)(Channel *prev_in,   Channel *prev_out,
			    Channel *next_in,   Channel *next_out,
			    void   **in_buffer, void   **out_buffer,
			    int      no_io_buffers,
			    Channel *extra_next_in[HOR_NO_LINKS],
			    Channel *extra_next_out[HOR_NO_LINKS], int extra_outs);
typedef void (*Hor_Up_Func)(Channel *exec_in, Channel *exec_out,
			Channel *next_in, Channel *prev_out, char tag);
typedef Hor_Bool (*Hor_Exec_Func)(void *input_buf, void *output_buf,
			  void **extra_input_buf, int extra_ins,
			  Hor_Bool reinitialised,
			  Channel *reinit_up_in, Channel *reinit_up_out,
			  Channel *next_in,      Channel *prev_out);
typedef void (*Hor_IO_Func)(Channel *pipe_in, void *input_buf);

void hor_pipe_control (Channel *prev_in, Channel *prev_out,
		Channel *next_in, Channel *next_out,
		int      no_io_buffers,
		Hor_Reinit_Func reinit_func, Hor_Up_Func up_func,
		Hor_Exec_Func   exec_func, int      exec_stack,
		Hor_IO_Func    input_func, int  input_stack, void  **in_buffer,
		Hor_IO_Func   output_func, int output_stack, void **out_buffer,
		   ... );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/print.h */

void hor_pipe_print_buffer ( Channel *out );
void hor_pipe_set_print_func(void);
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/reinit.h */

void hor_pipe_message ( Channel *pipe_out, int tag,
		        char    *buffer,   int length );

int hor_pipe_wait_for_value ( Channel *prev_in, Channel *prev_out,
			      Channel *next_in, Channel *next_out,
			      int      wait_tag );
int hor_pipe_head_wait_for_value ( Channel **next_in,
			       int       no_pipelines,
			       int       wait_tag );
