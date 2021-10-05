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
