/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from pipe/head.h */

#define hor_pipe_data_signal(c) ChanOutChar(c,HOR_PIPE_DATA)

void hor_pipe_return_stop_signal ( Channel **pipe_in,
				   Channel **pipe_out, int no_pipes );
