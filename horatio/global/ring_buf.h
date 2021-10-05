/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/ring_buf.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

/*******************
*   typedef void (*@hor_ring_buf_free_func)(void *data);
*   typedef void (*@hor_ring_buf_output_func)(Channel *channel,void *data,
*                                            void *params);
*
*   Function types for freeing and output ring-buffer functions.
*   The arguments to the output function are a channel, a pointer to the data
*   to be output and a pointer to parameters that may modify the data is
*   outputted. For transputers only.
********************/
typedef void (*hor_ring_buf_free_func)(void *data);
typedef void (*hor_ring_buf_output_func)(Channel *channel,void *data,
					 void *params);

void hor_ring_buf_init ( int result_memory_size,
			 hor_ring_buf_free_func   free_func );
void hor_ring_buf_clear(void);
void hor_ring_buf_store ( void *result );
void hor_ring_buf_output ( Channel *out, int memory_size,
			   hor_ring_buf_output_func output_func, void *params);

#endif
#endif
