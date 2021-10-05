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
