/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#define _HORATIO_PROCESS_
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/initproc.h */

void hor_init_process_stuff(void);
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/process.h */

/*******************
*   typedef void   (*@Hor_Output_Result_Func) (void *process_result,
*                                         void *output_data);
*   typedef void   (*@Hor_Free_Result_Func)   (void *process_result);
*   typedef void   (*@Hor_Free_Data_Func)     (void *data);
*   typedef void * (*@Hor_Execute_Func)       (Hor_List, void *process_result,
*                                                       void *process_data);
*   typedef void   (*@Hor_Update_Data_Func)   (void *process_data);
*
*   Virtual process pipeline control function type definitions.
********************/
typedef void   (*Hor_Output_Result_Func) (void *process_result,
					  void *output_data);
typedef void   (*Hor_Free_Result_Func)   (void *process_result);
typedef void   (*Hor_Free_Data_Func)     (void *data);

#ifdef _HORATIO_LIST_
typedef void * (*Hor_Execute_Func)       (Hor_List, void *process_result,
					            void *process_data);
#endif

typedef void   (*Hor_Update_Data_Func)   (void *process_data);

#ifdef _HORATIO_LIST_

Hor_Assoc_Label hor_add_result_type (
   Hor_Output_Result_Func output_result_func,   /* function called to output a
						   result of the new type,
						   usually either to the
						   display it or write it to a
						   file */
   Hor_Free_Result_Func   free_result_func,     /* function called to free a
						   result of the new type */
   Hor_Free_Data_Func     free_data_func );     /* function called to free the
						   data associated with the
						   outputting of a result of
						   the new type */

Hor_Assoc_Label hor_add_process_type (
   Hor_List             input_types,       /* list of result type labels of
					      input processes, to be checked
					      against result type labels of
					      processes actually input to
					      processes of the new type */
   Hor_Assoc_Label      result_type_label, /* label of the type of result that
					      executing a process of the new
					      type will return */
   Hor_Execute_Func     execute_func,      /* function called to execute a
					      process of the new type */
   Hor_Update_Data_Func update_data_func,  /* function called to update data
					      associated with a process of the
					      new type at each step of the
					      sequence */
   Hor_Free_Data_Func   free_data_func );  /* function called to free the data
					      associated with a process of the
					      new type */

Hor_Assoc_Label hor_add_process (
   Hor_Assoc_Label process_type_label, /* label of the process's type */
   Hor_List        input_processes,    /* list of input process labels */
   void       *process_data,       /* pointer to structure containing
				      parameters used by the process */
   void       *output_data );   /* pointer to structure containing
				   parameters used by the process for
				   output purposes */

#endif /* _HORATIO_LIST_ */

void hor_execute_processes(void);
void hor_clear_processes(void);

#ifdef _HORATIO_LIST_

void *hor_get_process_data ( Hor_Assoc_Label proc_label );

typedef struct
{
   Hor_Assoc_Label process_label, result_type_label;
   void           *process_result;
} Hor_Process_Input;

#endif /* _HORATIO_LIST_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/gb_proc.h */

/*******************
*   typedef struct
*   {
*      char root_name[200];
*      int  image_count;
*   } @Hor_GB_Process_Data;
*
*   Grab process parameter structure definition.
********************/
typedef struct
{
   char root_name[200];
   int  image_count;
} Hor_GB_Process_Data;

#ifdef _HORATIO_LIST_

void hor_set_gb_result_type_label  ( Hor_Assoc_Label );
void hor_set_gb_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_gb_result_type_label(void);
Hor_Assoc_Label hor_get_gb_process_type_label(void);
Hor_Assoc_Label hor_get_gb_process_label(void);

#endif /* _HORATIO_LIST_ */

Hor_GB_Process_Data *hor_gb_make_process_data ( const char *root_name );
void                 hor_gb_add_process       ( const char *root_name );

#ifdef _HORATIO_LIST_
void *hor_gb_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif

void  hor_gb_output              ( void *process_result, void *output_data );
void  hor_gb_update_process_data ( void *process_data   );
void  hor_gb_free_process_data   ( void *process_data   );
void  hor_gb_free_result         ( void *process_result );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/ed_proc.h */

#ifdef _HORATIO_LIST_
void            hor_set_ed_result_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_ed_result_type_label(void);
#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_ED_Output_Params *hor_ed_make_output_data ( Hor_ED_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_ed_output      ( void *process_result, void *output_data );
void hor_ed_free_result ( void *process_result );
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
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/li_proc.h */

#ifdef _HORATIO_LIST_

void            hor_set_li_result_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_li_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_LI_Output_Params *hor_li_make_output_data ( Hor_LI_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void  hor_li_output      ( void *process_result, void *output_data );
void  hor_li_free_result ( void *process_result );
/* Copyright 1994 David Djian (ddjian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/lf_proc.h */

typedef struct
{
   int   no_points;      /* number of points used to fit a line */
   float sigma;          /* standard deviation of edge position error */
   float fit_thres;      /* line fitting threshold */
} Hor_LF_Process_Params;

#ifdef _HORATIO_LIST_

void            hor_set_lf_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_lf_process_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_lf_add_process ( Hor_LF_Process_Params,
				     Hor_LI_Output_Params, Hor_Assoc_Label );

#endif /* _HORATIO_IMPROC_ */
void *hor_lf_execute ( Hor_List input_list, void *old_result,
		                            void *process_data );
#endif /* _HORATIO_LIST_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/ls_proc.h */

#ifdef _HORATIO_LIST_

void            hor_set_ls_process_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_ls_process_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_LIST_
#ifdef _HORATIO_IMPROC_
Hor_Assoc_Label hor_ls_add_process ( Hor_LI_Output_Params, Hor_Assoc_Label );

#endif /* _HORATIO_IMPROC_ */
void *hor_ls_execute ( Hor_List input_list, void *old_result,
		                            void *process_data );
#endif /* _HORATIO_LIST_ */
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/lm_proc.h */

#ifdef _HORATIO_LIST_

void hor_set_lm_result_type_label  ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_lm_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_LM_Output_Params *hor_lm_make_output_data ( Hor_LM_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_lm_output      ( void *process_result, void *output_data );
void hor_lm_free_result ( void *process_result );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/blm_proc.h */

/*******************
*   typedef struct
*   {
*      int   dist_thresh;   (distance threshold between two matched lines)
*      float cos_thresh;    (orientation cosine threshold between two lines)
*      float size_thresh;   (threshold on line length ratio)
*      int   iterations;    (number of winner-take-all matching iterations)
*   } @Hor_BLM_Process_Params;
*
*   Bog-standard line matching process parameter structure definitions.
********************/
typedef struct
{
   int   dist_thresh;   /* distance threshold between two matched lines */
   float cos_thresh;    /* orientation cosine threshold between two lines */
   float size_thresh;   /* threshold on line length ratio */
   int   iterations;    /* number of winner-take-all matching iterations */
} Hor_BLM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_blm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_blm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_BLM_Process_Params *hor_blm_make_process_data ( Hor_BLM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_blm_add_process ( Hor_BLM_Process_Params, Hor_LM_Output_Params,
			   Hor_Assoc_Label );

void *hor_blm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/co_proc.h */

#ifdef _HORATIO_LIST_
void            hor_set_co_result_type_label ( Hor_Assoc_Label );
Hor_Assoc_Label hor_get_co_result_type_label(void);
#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_CO_Output_Params *hor_co_make_output_data ( Hor_CO_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_co_output      ( void *process_result, void *output_data );
void hor_co_free_result ( void *process_result );
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

/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/cm_proc.h */

#ifdef _HORATIO_LIST_

void hor_set_cm_result_type_label  ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_cm_result_type_label(void);

#endif /* _HORATIO_LIST_ */

#ifdef _HORATIO_IMPROC_
Hor_CM_Output_Params *hor_cm_make_output_data ( Hor_CM_Output_Params );
#endif /* _HORATIO_IMPROC_ */

void hor_cm_output      ( void *process_result, void *output_data );
void hor_cm_free_result ( void *process_result );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/bcm_proc.h */

/*******************
*   typedef struct
*   {
*      int   range;              (maximum corner displacement between frames)
*      float imdiff_ratio_thres; (threshold on image differences)
*      int   iterations;         (number of passes through corner maps)
*   } @Hor_BCM_Process_Params;
*
*   Bog-standard corner matching process parameter structure definitions.
********************/
typedef struct
{
   int   range;              /* maximum corner displacement between frames */
   float imdiff_ratio_thres; /* threshold on image differences */
   int   iterations;         /* (number of passes through corner maps */
} Hor_BCM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_bcm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_bcm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_BCM_Process_Params *hor_bcm_make_process_data ( Hor_BCM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_bcm_add_process ( Hor_BCM_Process_Params, Hor_CM_Output_Params,
			   Hor_Assoc_Label );

void *hor_bcm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/dcm_proc.h */

/*******************
*   typedef struct
*   {
*      int   match_window; (maximum pixel distance between predicted
*                           corner position and actual position)
*      float corr_thres;   (threshold on image patch correlation)
*   } @Hor_DCM_Process_Params;
*
*   Brain-dead corner matching process parameter structure definitions.
********************/
typedef struct
{
   int   match_window; /* maximum pixel distance between predicted
			  corner position and actual position */
   float corr_thres;   /* threshold on image patch correlation */
} Hor_DCM_Process_Params;

#ifdef _HORATIO_LIST_

void hor_set_dcm_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_dcm_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_DCM_Process_Params *hor_dcm_make_process_data ( Hor_DCM_Process_Params );

#ifdef _HORATIO_LIST_
void hor_dcm_add_process ( Hor_DCM_Process_Params, Hor_CM_Output_Params,
			   Hor_Assoc_Label );

void *hor_dcm_execute ( Hor_List input_list, void *old_result,
		        void *process_data );
#endif
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/fl_proc.h */

/*******************
*   typedef struct
*   {
*      float sigma;              (standard dev. of Gaussian convolution mask)
*      int   gauss_size;         (half-size of Gaussian convolution mask)
*      float rc_deriv_threshold; (root-sum-square image derivatives thresholded
*                                 at this value)
*      float t_deriv_threshold;  (image time derivatives thresholded
*                                 at this value)
*   } @Hor_FL_Process_Params;
*
*   typedef struct
*   {
*      Hor_FL_Process_Params params;
*      float *gauss_mask;        (Gaussian convolution mask)
*      int    c1,  r1;           (top-left corner of subwindow)
*      int    c2,  r2;           (bottom-right corner of subwindow)
*   } @Hor_FL_Process_Data;
*
*   Image flow process parameter structure definitions.
********************/
typedef struct
{
   float sigma;              /* standard dev. of Gaussian convolution mask */
   int   gauss_size;         /* half-size of Gaussian convolution mask */
   float rc_deriv_threshold; /* root-sum-square image derivatives thresholded
                                at this value */
   float t_deriv_threshold;  /* image time derivatives thresholded
                                at this value */
} Hor_FL_Process_Params;

typedef struct
{
   Hor_FL_Process_Params params;
   float *gauss_mask;
   int    c1, r1, c2, r2;
} Hor_FL_Process_Data;

/*******************
*   typedef struct
*   {
*      u_long start_colour; (colour of dot plotted at start of vector)
*      u_long line_colour;  (colour of normal flow vectors)
*      float  scale;        (scale used to display image flow vectors)
*      int    increment;    (pixel distance between display sample)
*   } @Hor_FL_Output_Params;
*
*   Image flow process output parameter structure definition.
********************/
typedef struct
{
   u_long start_colour; /* colour of dot plotted at start of vector */
   u_long line_colour;  /* of normal flow vectors */
   float  scale;        /* scale used to display image flow vectors */
   int    increment;    /* pixel distance between display samples */
} Hor_FL_Output_Params;

#ifdef _HORATIO_LIST_

void hor_set_fl_result_type_label  ( Hor_Assoc_Label );
void hor_set_fl_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_fl_result_type_label(void);
Hor_Assoc_Label hor_get_fl_process_type_label(void);

#endif /* _HORATIO_LIST_ */

Hor_FL_Process_Data  *hor_fl_make_process_data ( Hor_FL_Process_Params,
						 int c1, int r1,
						 int c2, int r2 );
Hor_FL_Output_Params *hor_fl_make_output_data  ( Hor_FL_Output_Params );
void hor_fl_add_process ( Hor_FL_Process_Params, Hor_FL_Output_Params,
			  int c1, int r1, int c2, int r2 );

#ifdef _HORATIO_LIST_
void *hor_fl_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif

void  hor_fl_output            ( void *process_result, void *output_data );
void  hor_fl_free_process_data ( void *process_data   );
void  hor_fl_free_result       ( void *process_result );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/is_proc.h */

/*******************
*   typedef struct
*   {
*      int    patch_size;      (size of image patch in which image variance
*                               is calculated)
*      int    patch_density;   (overlap density: number of small patch lengths
*                               that make up full patch length)
*      float  low_threshold;   (initial value of s.d. to mean ratio threshold)
*      float  threshold_step;  (increment to threshold)
*      float  high_threshold;  (maximum value of threshold)
*      float  sd_scale;        (standard deviation in each image patch is
*                               magnified by this scaling factor for display
*                               purposes)
*      u_long thres_colour;    (colour for below-threshold variances)
*   } @Hor_IS_Process_Params;
*
*   typedef struct
*   {
*      Hor_IS_Process_Params params;
*      int    c1,  r1;         (top-left corner of subwindow)
*      int    c2,  r2;         (bottom-right corner of subwindow)
*   } @Hor_IS_Process_Data;
*
*   Image segmentation process parameter structure definitions.
********************/
typedef struct
{
   int    patch_size;
   int    patch_density;
   float  low_threshold;
   float  threshold_step;
   float  high_threshold;
   float  sd_scale;
   u_long thres_colour;
} Hor_IS_Process_Params;

typedef struct
{
   Hor_IS_Process_Params params;
   int c1, r1, c2, r2;
} Hor_IS_Process_Data;

/*******************
*   typedef struct
*   {
*      float sd_scale;  (standard deviation in each image patch is magnified by
*                        this scaling factor for display purposes)
*   } @Hor_IS_Output_Params;
*
*   Image segmentation process output parameter structure definition.
********************/
typedef struct
{
   float sd_scale;
} Hor_IS_Output_Params;

#ifdef _HORATIO_LIST_

void hor_set_is_result_type_label ( Hor_Assoc_Label );
void hor_set_is_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_is_result_type_label  ( void );
Hor_Assoc_Label hor_get_is_process_type_label ( void );

#endif /* _HORATIO_LIST_ */

Hor_IS_Process_Data *hor_is_make_process_data (Hor_IS_Process_Params,
					       int c1, int r1, int c2, int r2);
Hor_IS_Output_Params *hor_is_make_output_data  ( float sd_scale );
void hor_is_add_process ( Hor_IS_Process_Params,
			  int c1, int r1, int c2, int r2 );

#ifdef _HORATIO_LIST_
void *hor_is_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif

void  hor_is_output      ( void *process_result, void *output_data );
void  hor_is_free_result ( void *process_result );
/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from process/cl_proc.h */

/*******************
*   typedef struct
*   {
*      int    patch_size;      (size of image patch in which correlation
*                               is calculated)
*      int    patch_density;   (overlap density: number of small patch lengths
*                               that make up full patch length)
*      int    max_motion;      (maximum frame-to-frame motion allowed)
*      float  display_scale;   (motion vector in each image patch is
*                               magnified by this scaling factor for display
*                               purposes)
*      u_long display_colour;  (colour of motion vector)
*   } @Hor_CL_Process_Params;
*
*   typedef struct
*   {
*      Hor_CL_Process_Params params;
*      int    c1,  r1;         (top-left corner of subwindow)
*      int    c2,  r2;         (bottom-right corner of subwindow)
*   } @Hor_CL_Process_Data;
*
*   Correlation process parameter structure definitions.
********************/
typedef struct
{
   int    patch_size;
   int    patch_density;
   int    max_motion;
   float  display_scale;
   u_long display_colour;
} Hor_CL_Process_Params;

typedef struct
{
   Hor_CL_Process_Params params;
   int c1, r1, c2, r2;
} Hor_CL_Process_Data;

/*******************
*   typedef struct
*   {
*      float display_scale; (standard deviation in each image patch is
*                            magnified by this scaling factor for display
*                            purposes)
*   } @Hor_CL_Output_Params;
*
*   Correlation process output parameter structure definition.
********************/
typedef struct
{
   float display_scale;
} Hor_CL_Output_Params;

#ifdef _HORATIO_LIST_

void hor_set_cl_result_type_label ( Hor_Assoc_Label );
void hor_set_cl_process_type_label ( Hor_Assoc_Label );

Hor_Assoc_Label hor_get_cl_result_type_label  ( void );
Hor_Assoc_Label hor_get_cl_process_type_label ( void );

#endif /* _HORATIO_LIST_ */

Hor_CL_Process_Data *hor_cl_make_process_data (Hor_CL_Process_Params,
					       int c1, int r1, int c2, int r2);
Hor_CL_Output_Params *hor_cl_make_output_data  ( float display_scale );
void hor_cl_add_process ( Hor_CL_Process_Params,
			  int c1, int r1, int c2, int r2 );

#ifdef _HORATIO_LIST_
void *hor_cl_execute ( Hor_List input_list, void *old_result,
		       void *process_data );
#endif

void  hor_cl_output      ( void *process_result, void *output_data );
void  hor_cl_free_result ( void *process_result );
