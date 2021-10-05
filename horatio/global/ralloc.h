/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from global/ralloc.h */

int hor_test_malloc(void);
int hor_test_free(void);

#define hor_malloc(n) malloc(n)
#define hor_free(d)  free(d)

#if 0
/* alternative allocation routines for testing whether every malloc is
   getting freed */
static void *zxqynp;
static int   yzpwj1, yzpwj2, yzpwj3;

#define hor_malloc(nbytes) (hor_print("malloc of %7d at %8x (%4d)", nbytes, (int) (zxqynp = malloc(nbytes)), hor_test_malloc()), hor_print(": codes %8x %8x, line %4d of %s\n", *(((int *) zxqynp)-2), *(((int *) zxqynp )-1), __LINE__, __FILE__ ), zxqynp)

#define hor_free(address) (yzpwj1 = *(((int *)(address))-2), yzpwj2 = *(((int *)(address))-1), hor_print("free  (result %1d)  at %8x (%4d): codes %8x %8x, line %4d of %s\n", yzpwj3 = free(address)), (int)address, hor_test_free(), yzpwj1, yzpwj2, __LINE__, __FILE__ ),yzpwj3)
#endif

#define hor_malloc_type(type) ((type *) hor_malloc ( sizeof(type) ))
#define hor_malloc_ntype(type,n) ((type *) hor_malloc ( (n)*sizeof(type) ))

void hor_free_func ( void * ); /* function version of hor_free */
void hor_free_multiple ( void *ptr, ... );
