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
