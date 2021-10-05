/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/number.h */

#ifdef HOR_TRANSPUTER
#ifdef _channel_h

short HorChanInShort  ( Channel *c );
float HorChanInFloat  ( Channel *c );
void  HorChanOutShort ( Channel *c, short x );
void  HorChanOutFloat ( Channel *c, float x );

#endif
#endif

#ifndef HOR_REDUCED_LIB

int hor_pipe_read ( int fd, char *buf, int n );

void hor_reverse_byte_order2 ( char *ptr, int no_bytes );
void hor_reverse_byte_order4 ( char *ptr, int no_bytes );
void hor_reverse_byte_order8 ( char *ptr, int no_bytes );

char  hor_read_char   ( int fd );
int   hor_read_int    ( int fd );
float hor_read_float  ( int fd );
void  hor_write_char  ( int fd, char  number );
void  hor_write_int   ( int fd, int   number );
void  hor_write_float ( int fd, float number );

#endif /* HOR_REDUCED_LIB */
