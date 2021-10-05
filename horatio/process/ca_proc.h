/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/ca_proc.h */

/*******************
*   typedef struct
*   {
*      float sigma;        (standard dev. of Gaussian convolution mask)
*      int   gauss_size;   (half-size of Gaussian convolution mask)
*      float low_thres;
*      float high_thres;   (edge strength thresholds used in hysteresis stage)
*      int   length_thres; (string length threshold)
*   } @Hor_CA_Process_Params;
*
*   typedef struct
*   {
*      Hor_CA_Process_Params params;
*      float *gauss_mask;  (Gaussian convolution mask)
*      int    cv1, rv1;    (velocity of top-left corner of subwindow)
*      int    cv2, rv2;    (velocity of bottom-right corner of subwindow)
*   } @Hor_CA_Fixed;
*
*   typedef struct
*   {
*      int c1,  r1;  (top-left corner of subwindow)
*      int c2,  r2;  (bottom-right corner of subwindow)
*   } @Hor_CA_Writeable;
*
*   typedef struct
*   {
*      Hor_CA_Fixed     fixed;
*      Hor_CA_Writeable writeable;
*   } @Hor_CA_Process_Data;
*
*   Canny process parameter structure definitions
*   The Hor_CA_Fixed part does not change over time, whereas the Hor_CA_Writeable
*   part is updated at every image in the sequence.
********************/
typedef struct
{
   float sigma;        /* standard dev. of Gaussian convolution mask */
   int   gauss_size;   /* half-size of Gaussian convolution mask */
   float low_thres;
   float high_thres;   /* edge strength thresholds used in hysteresis stage */
   int   length_thres; /* string length threshold */
} Hor_CA_Process_Params;

typedef struct
{
   Hor_CA_Process_Params params;
   float *gauss_mask;        /* Gaussian convolution mask */
   int    cv1, rv1;          /* velocity of top-left corner of subwindow */
   int    cv2, rv2;          /* velocity of bottom-right corner of subwindow */
} Hor_CA_Fixed;

typedef struct
{
   int c1,  r1;  /* top-left corner of subwindow */
   int c2,  r2;  /* bottom-right corner of subwindow */
} Hor_CA_Writeable;

typedef struct
{
   Hor_CA_Fixed     fixed;
   Hor_CA_Writeable writeable;
} Hor_CA_Process_Data;

#ifdef _HORATIO_LIST_
void            hor_set_ca_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_ca_process_type_label(void);
#endif /* _HORATIO_LIST_ */

Hor_CA_Process_Data  *hor_ca_make_process_data ( Hor_CA_Process_Params,
					 int cv1, int rv1, int cv2, int rv2,
					 int  c1, int  r1, int  c2, int  r2);
#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_ca_add_process ( Hor_CA_Process_Params,
				     Hor_ED_Output_Params,
				     int cv1, int rv1, int cv2, int rv2,
				     int  c1, int  r1, int  c2, int  r2 );
#endif /* _HORATIO_IMPROC_ */
void *hor_ca_execute ( Hor_List input_list, void *old_result,
		                            void *process_data );
#endif /* _HORATIO_LIST_ */

void  hor_ca_update_process_data ( void *process_data );
void  hor_ca_free_process_data   ( void *process_data );
