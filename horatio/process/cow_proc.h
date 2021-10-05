/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/cow_proc.h */

/*******************
*   typedef struct
*   {
*      float edge_thresh;       (edge strength threshold)
*      float corner_thresh;     (corner strength threshold)
*      int   patch_size;        (size of image patch stored around corner)
*   } @Hor_COW_Process_Params;
*
*   typedef struct
*   {
*      COW_Process_Params params;
*      int    cv1, rv1;       (velocity of top-left corner of subwindow)
*      int    cv2, rv2;       (velocity of bottom-right corner of subwindow)
*   } @Hor_COW_Fixed;
*
*   typedef struct
*   {
*      int c1,  r1;  (top-left corner of subwindow)
*      int c2,  r2;  (bottom-right corner of subwindow)
*   } @Hor_COW_Writeable;
*
*   typedef struct
*   {
*      Hor_COW_Fixed     fixed;
*      Hor_COW_Writeable writeable;
*   } @Hor_COW_Process_Data;
*
*   Wang/Brady corner process parameter structure definitions.
*   The Hor_COW_Fixed part does not change over time, whereas the
*   Hor_COW_Writeable part is updated at every image in the sequence.
********************/
typedef struct
{
   float edge_thresh;       /* edge strength threshold */
   float corner_thresh;     /* corner strength threshold */
   int   patch_size;        /* size of image patch stored around corner */
} Hor_COW_Process_Params;

typedef struct
{
   Hor_COW_Process_Params params;
   int    cv1, rv1;       /* velocity of top-left corner of subwindow */
   int    cv2, rv2;       /* velocity of bottom-right corner of subwindow */
} Hor_COW_Fixed;

typedef struct
{
   int c1,  r1;  /* top-left corner of subwindow */
   int c2,  r2;  /* bottom-right corner of subwindow */
} Hor_COW_Writeable;

typedef struct
{
   Hor_COW_Fixed     fixed;
   Hor_COW_Writeable writeable;
} Hor_COW_Process_Data;

#ifdef _HORATIO_LIST_
void            hor_set_cow_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_cow_process_type_label(void);
#endif /* _HORATIO_LIST_ */

Hor_COW_Process_Data *hor_cow_make_process_data ( Hor_COW_Process_Params,
					  int cv1, int rv1, int cv2, int rv2,
					  int  c1, int  r1, int  c2, int  r2 );
#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_cow_add_process ( Hor_COW_Process_Params,
				      Hor_CO_Output_Params,
				      int cv1, int rv1, int cv2, int rv2,
				      int  c1, int  r1, int  c2, int  r2 );
#endif /* _HORATIO_IMPROC_ */

void *hor_cow_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif /* _HORATIO_LIST_ */

void hor_cow_update_process_data ( void *process_data   );

