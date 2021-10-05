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
