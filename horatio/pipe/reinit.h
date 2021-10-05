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
