/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/image.h */

typedef enum { HOR_BIT, HOR_U_CHAR, HOR_CHAR, HOR_U_SHORT, HOR_SHORT,
	       HOR_U_INT, HOR_INT, HOR_FLOAT, HOR_RGB_UC, HOR_RGB_UI,
	       HOR_POINTER }
   Hor_Image_Type;

typedef struct
{
   u_char r, g, b;
} Hor_RGB_UC;

typedef struct
{
   u_int r, g, b;
} Hor_RGB_UI;

typedef union
{
   hor_bit    b;   /* for bitmap image */
   u_char     uc;  /* for unsigned character image */
   char       c;   /* for (signed) character image */
   u_short    us;  /* for unsigned short image */
   short      s;   /* for (signed) short image */
   u_int      ui;  /* for unsigned int image */
   int        i;   /* for (signed) int image */
   float      f;   /* for floating point image */
   Hor_RGB_UC cuc; /* for colour unsigned character image */
   Hor_RGB_UI cui; /* for colour unsigned integer image */
   void      *p;   /* for pointer image */
} Hor_Impixel;

typedef union
{
   hor_bit    **b;
   u_char     **uc;
   char       **c;
   u_short    **us;
   short      **s;
   u_int      **ui;
   int        **i;
   float      **f;
   Hor_RGB_UC **cuc;
   Hor_RGB_UI **cui;
   void      ***p;
} Hor_Imdata;

typedef struct
{
   int            width, height;            /* dimensions of image */
   Hor_Image_Type type;                     /* pixel type */
   Hor_Imdata     array;                    /* pointer to image data */
   void         (*pixel_free_func)(void *); /* function to free data associated
					       with a pixel (pointer image
					       only) */
} Hor_Image;

typedef struct
{
   int       c0, r0; /* offsets from origin (top-left corner) of main image */
   Hor_Image image;
} Hor_Sub_Image;

/*******************
*   typedef struct
*   {
*      int c0,    r0;     (offsets of image window)
*      int width, height; (dimensions of image window)
*   } @Hor_Image_Window;
*
*   Rectangular image window definition.
********************/
typedef struct
{
   int c0, r0, width, height;
} Hor_Image_Window;
