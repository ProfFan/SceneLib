/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from graphics/colour.h */

/*******************
*   typedef union
*   {
*      u_char  *uc;
*      u_short *us;
*   } @Hor_X_to_Image_ColourMap;
*
*   Definition of colour-map structure to map X colour identifiers into
*   internal image format, whether u_char (8 bits) or u_short (16 bits).
********************/
typedef union
{
   u_char  *uc;
   u_int *us;
} Hor_X_to_Image_ColourMap;

#ifdef _XtIntrinsic_h
void hor_colourmap_setup ( Display *display,
			   u_int min_grey_bits, u_int max_grey_bits, ... );
#endif

u_long hor_colour_alloc ( u_short red, u_short green, u_short blue );
void hor_free_colourmap(void);

Hor_Bool hor_get_colour_name_value ( const char *name, u_long *colour );

#ifdef _HORATIO_IMAGE_
Hor_Image_Type hor_get_image_display_format(void);
#endif /* _HORATIO_IMAGE_ */
u_int          hor_get_image_display_depth(void);

/*******************
*   extern u_long *@Hor_Grey;
*
*   Grey-level array (0..255) indexing into X grey-level colourmap.
*   Hor_Grey[0] is black, Hor_Grey[255] is white.
********************/
extern u_long *Hor_Grey;

/*******************
*   #define @HOR_NEUTRAL_COLOUR 0xffffffff
*
*   Can be used to turn off drawing commands (must be detected
*   explicitly to work)
********************/
#define HOR_NEUTRAL_COLOUR 0xffffffff
