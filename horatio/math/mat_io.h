/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/mat_io.h */


#ifdef HOR_TRANSPUTER
#ifdef _channel_h

Hor_Matrix *HorChanInMatrix          ( Channel *c );
void        HorChanInAllocatedMatrix ( Channel *c, Hor_Matrix *M );
void        HorChanOutMatrix         ( Channel *c, Hor_Matrix *M );

#endif
#endif

#ifndef HOR_REDUCED_LIB

Hor_Matrix *hor_read_matrix           ( int fd );
void        hor_read_allocated_matrix ( int fd, Hor_Matrix *M );
void        hor_write_matrix          ( int fd, Hor_Matrix *M );

#endif
