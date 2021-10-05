/* Copyright 1993 Paul A. Beardsley (pab@robots.oxford.ac.uk) and
                  Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from tool/threed.h */

#ifdef _XtIntrinsic_h
#ifdef _HORATIO_MATH_

typedef struct {Hor_Matrix *p;}       Hor_Item_3D_point; /* p is a 3x1 matrix */
typedef struct {Hor_Matrix *p1, *p2;} Hor_Item_3D_line;  /* p1 & p2 are 3x1 matrices */
typedef struct
{
   Hor_Matrix *p1,    *p2,    *p3;
   int         c1, r1, c2, r2, c3, r3;
   Hor_Image  *image;
} Hor_Item_3D_facet;

typedef struct
{
   Hor_Item_3D_point point;
   Hor_Assoc_Label   font;
   const char       *label;
} Hor_Item_3D_label;

typedef struct
{
   enum {HOR_LINE_3D, HOR_POINT_3D, HOR_FACET_3D, HOR_LABEL_3D} type;
   union {Hor_Item_3D_point point;
	  Hor_Item_3D_line  line;
          Hor_Item_3D_facet facet;
          Hor_Item_3D_label label;} u;
} Hor_Item_3D;

typedef void (*Hor_3D_Item_Func) (Hor_Assoc_Label, Hor_Item_3D);

#ifdef _HORATIO_LIST_

typedef void (*Hor_3D_Item_List_Func) (Hor_Assoc_List assoc_list);

Hor_Assoc_Label hor_create_3D ( Widget parent, int size,
			        u_long background_colour,
			        u_long selected_colour );
void hor_register_3D ( Display *display );
Hor_Bool hor_popup_3D ( Widget button, Hor_Assoc_Label label_3D,
		        Hor_3D_Item_Func      oneselect_func,
		        Hor_3D_Item_List_Func procselected_func,
		        Hor_3D_Item_List_Func allclear_func,
		        Hor_3D_Item_Func      onedelete_func,
		        Hor_3D_Item_List_Func allrestore_func,
		        Hor_3D_Item_List_Func finish_func );
Hor_Bool hor_popdown_3D ( Hor_Assoc_Label label_3D );
Hor_Bool hor_3D_item ( Hor_Assoc_Label label_3D,
		       Hor_Assoc_Label item_label,
		       Hor_Item_3D item_ptr, u_long colour );
Hor_Bool hor_init_3D ( Hor_Assoc_Label label_3D,
		       Hor_Bool subtract_centroid );
Hor_Bool hor_colour_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label, u_long colour );
Hor_Bool hor_select_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label );
Hor_Bool hor_delete_3D_item ( Hor_Assoc_Label label_3D,
			      Hor_Assoc_Label item_label );
Hor_Bool hor_reset_3D_item ( Hor_Assoc_Label label_3D,
			     Hor_Assoc_Label item_label );
Hor_Bool hor_3D_clear ( Hor_Assoc_Label label_3D );
Hor_Bool hor_3D_in_use    ( Hor_Assoc_Label label_3D );
Hor_Bool hor_redisplay_3D ( Hor_Assoc_Label label_3D );
Hor_Bool hor_write_3D_text ( Hor_Assoc_Label label_3D, const char *base_name );
Hor_Bool hor_write_3D_image ( Hor_Assoc_Label label_3D, const char *base_name,
			      int image_size );

Hor_Item_3D *hor_alloc_item_3D ( int item_type );
Hor_Item_3D *hor_fill_item_3D ( Hor_Item_3D *item, ... );
Hor_Item_3D *hor_make_item_3D ( int item_type, ... );
void         hor_free_item_3D ( Hor_Item_3D *item );

#endif /* _HORATIO_LIST_ */
#endif /* _HORATIO_MATH_ */
#endif /* _XtIntrinsic_h */
