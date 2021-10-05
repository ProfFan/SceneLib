/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from improc/corner.h */

#ifdef _HORATIO_IMAGE_
typedef struct _Hor_Corner
{
   int   status; /* for use by corner matcher. Standard values are
		    HOR_UNMATCHED, HOR_PROVISIONAL and HOR_MATCHED. */
   float strength; /* corner strength */
   float cf, rf;   /* corner position to sub-pixel precision relative to the
		      top-left hand corner of the image */
   Hor_Sub_Image *patch; /* image patch around corner */

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
   void (*display_func)(struct _Hor_Corner *corner, void *params);
} Hor_Corner;

typedef struct
{
   int  c0,    r0;     /* the top-left position relative to parent image */
   int  width, height; /* the width, height of this rectangle */
   float max_strength; /* maximum corner strength */
   Hor_Corner ***corners;
   Hor_List      corner_list;
   int           ncorners;

   /* user-defined attributes */
   int    type; /* e.g. HOR_NO_ATTRIB */
   void  *attrib;
   void (*attrib_free_func)(void *attrib);
} Hor_Corner_Map;

Hor_Corner_Map *hor_alloc_corner_map (int c0, int r0, int width, int height,
				      int type, void *attrib,
				      void (*attrib_free_func)(void *attrib));
void hor_free_corner_map ( Hor_Corner_Map *corner_map );

Hor_Corner *hor_add_corner ( Hor_Corner_Map *map, float strength,
			     float cf, float rf,
			     Hor_Image *image, int    c0, int     r0,
			                       int width, int height,
			     int type, void *attrib,
			     void (*attrib_free_func)(void *attrib),
			     void (*display_func)(Hor_Corner *corner,
						  void *params) );
#endif /* _HORATIO_IMAGE_ */
