/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/pc_proc.h */

/*******************
*   typedef struct
*   {
*      float sigma;            (standard dev. of Gaussian convolution mask)
*      int   gauss_size;       (half-size of Gaussian convolution mask)
*      float strength_thres;   (corner strength threshold)
*      int   patch_size;       (size of image patch stored around corner)
*   } @Hor_PC_Process_Params;
*
*   typedef struct
*   {
*      Hor_PC_Process_Params params;
*      float *gauss_mask;     (Gaussian convolution mask)
*      int    cv1, rv1;       (velocity of top-left corner of subwindow)
*      int    cv2, rv2;       (velocity of bottom-right corner of subwindow)
*   } @Hor_PC_Fixed;
*
*   typedef struct
*   {
*      int c1,  r1;  (top-left corner of subwindow)
*      int c2,  r2;  (bottom-right corner of subwindow)
*   } @Hor_PC_Writeable;
*
*   typedef struct
*   {
*      Hor_PC_Fixed     fixed;
*      Hor_PC_Writeable writeable;
*   } @Hor_PC_Process_Data;
*
*   Plessey corner process parameter structure definitions. The Hor_PC_Fixed
*   part does not change over time, whereas the Hor_PC_Writeable part is
*   updated at every image in the sequence.
********************/
typedef struct
{
   float sigma;           /* standard dev. of Gaussian convolution mask */
   int   gauss_size;      /* half-size of Gaussian convolution mask */
   float strength_thres;  /* corner strength threshold */
   int   patch_size;      /* size of image patch stored around corner */
} Hor_PC_Process_Params;

typedef struct
{
   Hor_PC_Process_Params params;
   float *gauss_mask;     /* Gaussian convolution mask */
   int    cv1, rv1;       /* velocity of top-left corner of subwindow */
   int    cv2, rv2;       /* velocity of bottom-right corner of subwindow */
} Hor_PC_Fixed;

typedef struct
{
   int c1,  r1;  /* top-left corner of subwindow */
   int c2,  r2;  /* bottom-right corner of subwindow */
} Hor_PC_Writeable;

typedef struct
{
   Hor_PC_Fixed     fixed;
   Hor_PC_Writeable writeable;
} Hor_PC_Process_Data;

#ifdef _HORATIO_LIST_
void            hor_set_pc_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_pc_process_type_label(void);
#endif /* _HORATIO_LIST_ */

Hor_PC_Process_Data *hor_pc_make_process_data ( Hor_PC_Process_Params,
                                        int cv1, int rv1, int cv2, int rv2,
                                        int  c1, int  r1, int  c2, int  r2 );

#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_pc_add_process ( Hor_PC_Process_Params,
				     Hor_CO_Output_Params,
				     int cv1, int rv1, int cv2, int rv2,
				     int  c1, int  r1, int  c2, int  r2 );
#endif /* _HORATIO_IMPROC_ */

void *hor_pc_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif /* _HORATIO_LIST_ */

void  hor_pc_update_process_data ( void *process_data   );
void  hor_pc_free_process_data   ( void *process_data   );
