/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/misc.h */

/*******************
*   Hor_Bool @hor_type_image     ( Hor_Image     *image, Hor_Image_Type type)
*   Hor_Bool @hor_type_sub_image ( Hor_Sub_Image *image, Hor_Image_Type type)
*
*   Return HOR_TRUE if (sub-)image is of given type, HOR_FALSE otherwise.
*   Both implemented as macros.
********************/
#define hor_type_image(imptr,imtype) ((imptr)->type == (imtype))
#define hor_type_sub_image(subimptr,imtype) ((subimptr)->image.type==(imtype))

Hor_Sub_Image *hor_extract_from_image     ( Hor_Image *image,
					    int c0,    int r0,
					    int width, int height );
Hor_Sub_Image *hor_extract_from_sub_image ( Hor_Sub_Image *sub_image,
					    int ext_c0,    int ext_r0,
					    int ext_width, int ext_height );
void hor_insert_image_in_image (Hor_Image *source_image, Hor_Image *dest_image,
				int c_offset, int r_offset );
#define hor_insert_sub_image_in_image(sim,im) hor_insert_image_in_image(&(sim)->image,im,(sim)->c0,(sim)->r0)

void hor_add_constant_to_image        ( Hor_Image *, Hor_Impixel );
void hor_subtract_constant_from_image ( Hor_Image *, Hor_Impixel );
void hor_multiply_image_by_constant   ( Hor_Image *, Hor_Impixel );
void hor_divide_image_by_constant     ( Hor_Image *, Hor_Impixel );

Hor_Image     *hor_add_images ( Hor_Image *, Hor_Image * );
Hor_Sub_Image *hor_add_sub_images            (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_subtract_images           (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_subtract_sub_images       (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_subtract_signed_images    (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_subtract_signed_sub_images(Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_multiply_images           (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_multiply_sub_images       (Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_multiply_double_images    (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_multiply_double_sub_images(Hor_Sub_Image *,Hor_Sub_Image *);
Hor_Image     *hor_divide_images             (Hor_Image *,    Hor_Image *    );
Hor_Sub_Image *hor_divide_sub_images         (Hor_Sub_Image *,Hor_Sub_Image *);

#define hor_add_constant_to_sub_image(sim,pix) (hor_add_constant_to_image(&((sim)->image),pix))
#define hor_subtract_constant_from_sub_image(sim,pix) (hor_subtract_constant_from_image(&((sim)->image),pix))
#define hor_multiply_sub_image_by_constant(sim,pix) (hor_multiply_image_by_constant(&((sim)->image),pix))
#define hor_divide_sub_image_by_constant(sim,pix) (hor_divide_image_by_constant(&((sim)->image),pix))

void           hor_fill_image_with_constant (Hor_Image *,     Hor_Impixel    );
Hor_Image     *hor_average_images           (Hor_Image *,     Hor_Image *    );
Hor_Sub_Image *hor_average_sub_images       (Hor_Sub_Image *, Hor_Sub_Image *);

#define hor_fill_sub_image_with_constant(sim,pix) (hor_fill_image_with_constant(&((sim)->image),pix))
