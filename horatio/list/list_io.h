/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* include from list/list_io.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h
Hor_List HorChanInList ( Channel *in, int no_nodes,
			 void * (*input_func)(Channel *),
			 void (*free_func)(void *) );
void HorChanOutList ( Channel *out, Hor_List list,
		      void (*output_func)(Channel *,void *) );
#endif
#endif

#ifndef HOR_REDUCED_LIB
Hor_List hor_read_list ( int fd, int no_nodes, void * (*input_func)(int),
			 void (*free_func)(void *) );
void hor_write_list ( int fd, Hor_List list,
		      void (*output_func)(int,void *) );
#endif /* HOR_REDUCED_LIB */
