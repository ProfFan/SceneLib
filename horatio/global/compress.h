/* included from global/types.h */

/* copied from /usr/include/sys/types.h */

#ifndef HOR_MSDOS
#ifndef HOR_TRANSPUTER

/*******************
*   typedef enum { HOR_NO_COMPRESS, HOR_UNIX_COMPRESS, HOR_GNU_COMPRESS }
*      @Hor_Compress_Type;
*
*   Definition of Horatio compression types for reading/writing files.
*   HOR_UNIX_COMPRESS specifies UNIX compression (compress).
*   HOR_GNU_COMPRESS  specifies Gnu compression (gzip).
********************/
typedef enum { HOR_NO_COMPRESS, HOR_UNIX_COMPRESS, HOR_GNU_COMPRESS }
   Hor_Compress_Type;

int hor_compress ( const char *file_name );
int hor_gzip     ( const char *file_name );
int hor_uncompress ( int fdin, int *pid_ptr );
int hor_gunzip     ( int fdin, int *pid_ptr );

#endif /* HOR_TRANSPUTER */
#endif /* HOR_MSDOS */
