/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk) and
                  HOR_CHARles S. Wiles (csw@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/sc_proc.h */

/*******************
*   typedef struct
*   {
*      int   diff_thres;       (brightness difference threshold)
*      int   geom_thres;       (geometric threshold)
*      int   patch_size;       (size of image patch stored around corner)
*   } @Hor_SC_Process_Params;
*
*   typedef struct
*   {
*      Hor_SC_Process_Params params;
*      int    cv1, rv1;       (velocity of top-left corner of subwindow)
*      int    cv2, rv2;       (velocity of bottom-right corner of subwindow)
*   } @Hor_SC_Fixed;
*
*   typedef struct
*   {
*      Hor_SC_Fixed     fixed;
*      Hor_PC_Writeable writeable;
*   } @Hor_SC_Process_Data;
*
*   Smith Corner process parameter structure definitions
*   The Hor_SC_Fixed part does not change over time, whereas the
*   Hor_PC_Writeable part is updated at every image in the sequence.
********************/
typedef struct
{
   int   diff_thres;       /* brightness difference threshold */
   int   geom_thres;       /* geometric threshold */
   int   patch_size;       /* size of image patch stored around corner */
} Hor_SC_Process_Params;

typedef struct
{
   Hor_SC_Process_Params params;
   int    cv1, rv1;       /* velocity of top-left corner of subwindow */
   int    cv2, rv2;       /* velocity of bottom-right corner of subwindow */
} Hor_SC_Fixed;

typedef struct
{
   Hor_SC_Fixed     fixed;
   Hor_PC_Writeable writeable;
} Hor_SC_Process_Data;

#ifdef _HORATIO_LIST_
void            hor_set_sc_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_sc_process_type_label(void);
#endif /* _HORATIO_LIST_ */

Hor_SC_Process_Data *hor_sc_make_process_data ( Hor_SC_Process_Params,
                                        int cv1, int rv1, int cv2, int rv2,
                                        int  c1, int  r1, int  c2, int  r2 );
#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_sc_add_process ( Hor_SC_Process_Params,
				     Hor_CO_Output_Params,
				     int cv1, int rv1, int cv2, int rv2,
				     int  c1, int  r1, int  c2, int  r2 );
#endif /* _HORATIO_IMPROC_ */

void *hor_sc_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif /* _HORATIO_LIST_ */

void hor_sc_update_process_data ( void *process_data   );
