/* Copyright 1993 Philip F. McLauchlan (pm@uk.ac.oxford.robots)
                  Robotics Research Group, Oxford University. */

#ifdef TRANSPUTER
#define HOR_TRANSPUTER
#endif

#ifdef REDUCED_LIBRARY
#define HOR_REDUCED_LIBRARY
#endif

#ifdef GEC_HRC
#define HOR_GEC_HRC
#endif

/* global/error.h */
#if 0
#define ALLOCATION_FAILED     HOR_ALLOCATION_FAILED
#define NULL_POINTER_ARGUMENT HOR_NULL_POINTER_ARGUMENT
#define OPEN_FAILED_FOR_READ  HOR_OPEN_FAILED_FOR_READ
#define OPEN_FAILED_FOR_WRITE HOR_OPEN_FAILED_FOR_WRITE
#define READ_FAILED           HOR_READ_FAILED
#define WRITE_FAILED          HOR_WRITE_FAILED
#define MATRIX_SINGULAR           HOR_MATRIX_SINGULAR
#define MATRIX_ILLEGAL_DIMENSIONS HOR_MATRIX_ILLEGAL_DIMENSIONS
#define MATRIX_INCOMPATIBLE       HOR_MATRIX_INCOMPATIBLE
#define MATRIX_NOT_POS_DEFINITE   HOR_MATRIX_NOT_POS_DEFINITE
#define MATRIX_EXTRA_ZEROES       HOR_MATRIX_EXTRA_ZEROES
#define MATRIX_NOT_SQUARE         HOR_MATRIX_NOT_SQUARE
#define ILLEGAL_VALUE             HOR_ILLEGAL_VALUE
#define NO_CONVERGENCE            HOR_NO_CONVERGENCE
#define NUMREC_BAD_PERMUTATION    HOR_NUMREC_BAD_PERMUTATION
#define ASSOCIATION_NOT_FOUND HOR_ASSOCIATION_NOT_FOUND
#define WRONG_TYPE_IMAGE             HOR_WRONG_TYPE_IMAGE
#define NON_EXISTENT_IMAGE           HOR_NON_EXISTENT_IMAGE
#define INCOMPATIBLE_IMAGE           HOR_INCOMPATIBLE_IMAGE
#define CANNOT_READ_IMAGE_HEADER     HOR_CANNOT_READ_IMAGE_HEADER
#define ILLEGAL_IMAGE_TYPE_IN_HEADER HOR_ILLEGAL_IMAGE_TYPE_IN_HEADER
#define ILLEGAL_PIXEL_SCALE          HOR_ILLEGAL_PIXEL_SCALE
#define ILLEGAL_BITS_PER_PIXEL       HOR_ILLEGAL_BITS_PER_PIXEL
#define INVALID_IMAGE_REGION         HOR_INVALID_IMAGE_REGION
#define IMAGE_TOO_SMALL              HOR_IMAGE_TOO_SMALL
#define DISPLAY_WINDOW_NOT_SET      HOR_DISPLAY_WINDOW_NOT_SET
#define CANVAS_NOT_INITIALIZED      HOR_CANVAS_NOT_INITIALIZED
#define ILLEGAL_SUBSAMPLING_RATIO   HOR_ILLEGAL_SUBSAMPLING_RATIO
#define NON_EXISTENT_FONT_NAME      HOR_NON_EXISTENT_FONT_NAME
#define ILLEGAL_INTERNAL_DIMENSIONS HOR_ILLEGAL_INTERNAL_DIMENSIONS
#define X_DISPLAY_FUNCTION_FAILED   HOR_X_DISPLAY_FUNCTION_FAILED
#define INVALID_DISPLAY_DEPTH       HOR_INVALID_DISPLAY_DEPTH
#define COLOUR_NOT_ALLOCATED        HOR_COLOUR_NOT_ALLOCATED
#define REGION_TOO_SMALL        HOR_REGION_TOO_SMALL
#define ILLEGAL_IMAGE_TYPE      HOR_ILLEGAL_IMAGE_TYPE
#define ILLEGAL_CORNER_POSITION HOR_ILLEGAL_CORNER_POSITION
#define CORNRECTS_INCOMPATIBLE  HOR_CORNRECTS_INCOMPATIBLE
#define ILLEGAL_PARAMETERS      HOR_ILLEGAL_PARAMETERS
#define IMAGES_INCOMPATIBLE     HOR_IMAGES_INCOMPATIBLE
#define UNDEFINED_RESULT_TYPE         HOR_UNDEFINED_RESULT_TYPE
#define UNDEFINED_PROCESS             HOR_UNDEFINED_PROCESS
#define UNDEFINED_PROCESS_TYPE        HOR_UNDEFINED_PROCESS_TYPE
#define INCOMPATIBLE_RESULT_TYPES     HOR_INCOMPATIBLE_RESULT_TYPES
#define DIFFERENT_LENGTH_RESULT_LISTS HOR_DIFFERENT_LENGTH_RESULT_LISTS
#define DUPLICATED_RESULT_TYPES       HOR_DUPLICATED_RESULT_TYPES
#define DUPLICATED_PROCESS_TYPES      HOR_DUPLICATED_PROCESS_TYPES
#define ILLEGAL_PROCESS_DATA          HOR_ILLEGAL_PROCESS_DATA
#define ILLEGAL_OUTPUT_DATA           HOR_ILLEGAL_OUTPUT_DATA
#define POPUP_PANEL_NOT_IN_USE     HOR_POPUP_PANEL_NOT_IN_USE
#define POPUP_PANEL_ALREADY_IN_USE HOR_POPUP_PANEL_ALREADY_IN_USE
#define POPUP_PANEL_NOT_REGISTERED HOR_POPUP_PANEL_NOT_REGISTERED
#define POPUP_PANEL_NOT_INIT       HOR_POPUP_PANEL_NOT_INIT
#define GRAPH_NEGATIVE_TIME_RANGE  HOR_GRAPH_NEGATIVE_TIME_RANGE
#define GRAPH_NEGATIVE_F_RANGE     HOR_GRAPH_NEGATIVE_F_RANGE
#define GRAPH_ALREADY_SET_UP       HOR_GRAPH_ALREADY_SET_UP
#define GRAPH_TIME_REVERSAL        HOR_GRAPH_TIME_REVERSAL
#define GRAPH_NO_POINTS            HOR_GRAPH_NO_POINTS
#endif

/* global/types.h */
#define bool  Hor_Bool
#define false HOR_FALSE
#define true  HOR_TRUE

/* global/ralloc.h */
#define new_malloc   hor_test_malloc
#define new_free     hor_test_free
#define ralloc       hor_malloc
#define rfree        hor_free
#define ralloc_type  hor_malloc_type
#define ralloc_ntype hor_malloc_ntype
#define ffree        hor_free_func

/* global/text_IO.h */
#define Error_Type               Hor_Error_Type
#define Fatal                    HOR_FATAL
#define Non_Fatal                HOR_NON_FATAL
#define set_print_func           hor_set_print_func
#define message                  hor_message
#define warning                  hor_warning
#define error                    hor_error
#define set_fatal_error_function hor_set_fatal_error_function
#define wait_for_keyboard        hor_wait_for_keyboard

/* list/bit_array.h */
#define bit               hor_bit
#define Bit_Array         Hor_Bit_Array
#define set_bit           hor_set_bit
#define unset_bit         hor_unset_bit
#define get_bit           hor_get_bit
#define alloc_bit_array   hor_alloc_bit_array
#define free_bit_array    hor_free_bit_array
#define fill_bit_array    hor_fill_bit_array
#define copy_bit_array    hor_copy_bit_array
#define insert_bit_array  hor_insert_bit_array
#define common_bits       hor_common_bits
#define bit_array_and     hor_bit_array_and
#define bit_array_or      hor_bit_array_or
#define bit_array_eor     hor_bit_array_eor
#define bit_array_and_not hor_bit_array_and_not
#define bit_words         hor_bit_words

/* global/inmos_extensions.h */
#define NO_LINKS      4
#define ProcRunPar    HorProcRunPar
#define ProcRunList   HorProcRunList
#ifdef HOR_TRANSPUTER
#define ChanList      HorChanList
#define ChanListReset HorChanListReset
#define block_move_2D hor_block_move_2D
#endif

/* include from global/ring_buffer.h */
#ifdef HOR_TRANSPUTER
#define ring_buf_free_func   hor_ring_buf_free_func
#define ring_buf_output_func hor_ring_buf_output_func
#define ring_buf_init        hor_ring_buf_init
#define ring_buf_clear       hor_ring_buf_clear
#define ring_buf_store       hor_ring_buf_store
#define ring_buf_output      hor_ring_buf_output
#endif

/* global/number_io.h */
#ifdef HOR_TRANSPUTER
#define ChanInShort         HorChanInShort
#define ChanInFloat         HorChanInFloat
#define ChanOutShort        HorChanOutShort
#define ChanOutFloat        HorChanOutFloat
#endif
#define pipe_read           hor_pipe_read
#define reverse_byte_order2 hor_reverse_byte_order2
#define reverse_byte_order4 hor_reverse_byte_order4
#define reverse_byte_order8 hor_reverse_byte_order8
#define read_char           hor_read_char
#define read_int            hor_read_int
#define read_float          hor_read_float
#define write_char          hor_write_char
#define write_int           hor_write_int
#define write_float         hor_write_float

/* math/math.h */
#define sqr          hor_sqr
#ifdef HOR_TRANSPUTER
#define sqrf         hor_sqrf
#define cbrt         hor_cbrt
#else
#define sqrf         hor_sqrf
#endif
#define DISTANCE_2D  HOR_DISTANCE_2D
#define DISTANCE_3D  HOR_DISTANCE_3D
#ifdef HOR_TRANSPUTER
#define DISTANCE_2Df HOR_DISTANCE_2Df
#define DISTANCE_3Df HOR_DISTANCE_3Df
#endif
#define SIGN         HOR_SIGN
#define SIGNF        HOR_SIGNF
#define SIGND        HOR_SIGND

/* math/power_of_two.h */
#define power_of_two             hor_power_of_two
#define highest_power_of_two_leq hor_highest_power_of_two_leq
#define int_log_to_base_two      hor_int_log_to_base_two

/* math/compare.h */
#define min2 hor_min2
#define max2 hor_max2
#define min3 hor_min3
#define max3 hor_max3
#define imin hor_imin
#define imax hor_imax
#define fmin hor_fmin
#define fmax hor_fmax
#define dmin hor_dmin
#define dmax hor_dmax

/* math/matrix.h */
#define Matrix           Hor_Matrix
#define mat_alloc        hor_mat_alloc
#define mat_free         hor_mat_free
#define mat_free_list    hor_mat_free_list
#define mat_print        hor_mat_print
#define mats_fill        hor_mats_fill
#define mats_copy        hor_mats_copy
#define mats_add2        hor_mats_add2
#define mats_sub         hor_mats_sub
#define mats_prod2       hor_mats_prod2
#define mats_prod3       hor_mats_prod3
#define mats_scale       hor_mats_scale
#define mats_transpose   hor_mats_transpose
#define mats_lud         hor_mats_lud
#define mats_lud_inv     hor_mats_lud_inv
#define mats_prod_lu     hor_mats_prod_lu
#define mats_prod_ul     hor_mats_prod_ul
#define mats_inv         hor_mats_inv
#define mats_solve       hor_mats_solve
#define mats_identity    hor_mats_identity
#define mats_zero        hor_mats_zero
#define mats_diagonal    hor_mats_diagonal
#define matq_fill        hor_matq_fill
#define matq_copy        hor_matq_copy
#define matq_add2        hor_matq_add2
#define matq_sub         hor_matq_sub
#define matq_prod2       hor_matq_prod2
#define matq_prod3       hor_matq_prod3
#define matq_scale       hor_matq_scale
#define matq_transpose   hor_matq_transpose
#define matq_lud         hor_matq_lud
#define matq_lud_inv     hor_matq_lud_inv
#define matq_prod_lu     hor_matq_prod_lu
#define matq_prod_ul     hor_matq_prod_ul
#define matq_solve_lower hor_matq_solve_lower
#define matq_solve_upper hor_matq_solve_upper
#define matq_inv         hor_matq_inv
#define matq_ludcmp      hor_matq_ludcmp
#define matq_lubksb      hor_matq_lubksb
#define matq_tred2       hor_matq_tred2
#define matq_tqli        hor_matq_tqli
#define matq_eigsrt      hor_matq_eigsrt
#define matq_svdcmp      hor_matq_svdcmp
#define matq_gaussj      hor_matq_gaussj
#define matq_mrqmin      hor_matq_mrqmin
#define matq_identity    hor_matq_identity
#define matq_zero        hor_matq_zero
#define matq_diagonal    hor_matq_diagonal
#define mat_read         hor_mat_read
#define mat_is_square    hor_mat_is_square
#define mat_test_size    hor_mat_test_size
#define mat_same_size2   hor_mat_same_size2
#define mat_same_size3   hor_mat_same_size3
#define mat_fits_in      hor_mat_fits_in
#define mat_big_enough   hor_mat_big_enough
#define mat_increment    hor_mat_increment
#define mat_decrement    hor_mat_decrement

/* math/vector.h */
#define vec_scalar_prod hor_vec_scalar_prod
#define vec_max_coord   hor_vec_max_coord
#define vecs_cross_prod hor_vecs_cross_prod
#define vecs_unit       hor_vecs_unit
#define vecq_cross_prod hor_vecq_cross_prod
#define vecq_unit       hor_vecq_unit

/* math/gauss.h */
#define drandom hor_drandom
#define gasdev  hor_gauss_rand

/* math/chi_squared.h */
#define normal_prob  hor_normal_prob
#define normal_thres hor_normal_thres
#define chi_2_prob   hor_chi_2_prob

/* math/kalman_filter.h */
#define kalman_init         hor_kalman_init
#define kalman_predict      hor_kalman_predict
#define kalman_update       hor_kalman_update
#define kalman_step         hor_kalman_step
#define kalman_state_vector hor_kalman_state_vector
#define kalman_covariance   hor_kalman_covariance
#define kalman_print_state  hor_kalman_print_state

/* math/variable_state.h */
#define VS_State                 Hor_VS_State
#define VS_Mode                  Hor_VS_Mode
#define VS_FIXED                 HOR_VS_FIXED
#define VS_DYNAMIC               HOR_VS_DYNAMIC
#define VS_DYNAMIC_OPTIMAL       HOR_VS_DYNAMIC_OPTIMAL
#define vs_init                  hor_vs_init
#define vs_add_state             hor_vs_add_state
#define vs_add_state_init        hor_vs_add_state_init
#define vs_discard_state         hor_vs_discard_state
#define vs_observation           hor_vs_observation
#define vs_state_update          hor_vs_state_update
#define vs_get_local_residual    hor_vs_get_local_residual
#define vs_get_local_dof         hor_vs_get_local_dof
#define vs_get_time_step         hor_vs_get_time_step
#define vs_get_global_state      hor_vs_get_global_state
#define vs_get_global_covariance hor_vs_get_global_covariance
#define vs_get_no_local_states   hor_vs_get_no_local_states
#define vs_get_local_states      hor_vs_get_local_states
#define vs_get_residual          hor_vs_get_residual
#define vs_get_dof               hor_vs_get_dof
#define vs_innovation            hor_vs_innovation
#define vs_print_state           hor_vs_print_state

/* math/matrix_io.h */
#ifdef HOR_TRANSPUTER
#define ChanInMatrix          HorChanInMatrix
#define ChanInAllocatedMatrix HorChanInAllocatedMatrix
#define ChanOutMatrix         HorChanOutMatrix
#endif
#define read_matrix           hor_read_matrix
#define read_allocated_matrix hor_read_allocated_matrix
#define write_matrix          hor_write_matrix

/* math/poly_root.h */
#define solve_quadratic hor_solve_quadratic
#define solve_cubic     hor_solve_cubic

/* pipe/pipe_head.h */
#define pipe_data_signal        hor_pipe_data_signal
#define pipe_return_stop_signal hor_pipe_return_stop_signal

/* pipe/pipe_proc.h */
#define PIPE_STOP            0
#define PIPE_RESTART         1
#define PIPE_DATA            2
#define PIPE_PRINT           3
#define PIPE_FREQ            4
#define MAX_PIPE_COMM_LENGTH 1024
#define pipe_wait_for_value  hor_pipe_wait_for_value
#define Reinit_Func          Hor_Reinit_Func
#define Up_Func              Hor_Up_Func
#define Exec_Func            Hor_Exec_Func
#define IO_Func              Hor_IO_Func
#define pipe_control         hor_pipe_control

/* pipe/pipe_print.h */
#define pipe_print_buffer   hor_pipe_print_buffer
#define pipe_set_print_func hor_pipe_set_print_func

/* pipe/pipe_tail.h */
#define Merge_Func        Hor_Merge_Func
#define Tail_Exec_Func    Hor_Tail_Exec_Func
#define Tail_Reinit_Func  Hor_Tail_Reinit_Func
#define pipe_tail_control hor_pipe_tail_control
#define pipe_stop         hor_pipe_stop

/* pipe/pipe_reinit.h */
#define pipe_message             hor_pipe_message
#define pipe_wait_for_value      hor_pipe_wait_for_value
#define pipe_head_wait_for_value hor_pipe_head_wait_for_value

/* list/single_list.h */
#define Node            Hor_Node
#define List            Hor_List
#define node_contents   hor_node_contents
#define next_node       hor_next_node
#define list_null       hor_list_null
#define list_non_null   hor_list_non_null
#define insert          hor_insert
#define append          hor_append
#define concat          hor_concat
#define free_list       hor_free_list
#define free_nodes      hor_free_nodes
#define reverse         hor_reverse
#define reverse_no_free hor_reverse_no_free
#define list_action     hor_list_action
#define delete_first    hor_delete_first
#define delete_next     hor_delete_next
#define list_size       hor_list_size
#define list_to_array   hor_list_to_array
#define array_to_list   hor_array_to_list

/* list/double_list.h */
#define DNode                    Hor_DNode
#define DList                    Hor_DList
#define prev_node                hor_dprev_node
#define dmake_straight           hor_dmake_straight
#define dmake_circular           hor_dmake_circular
#define dinsert_before           hor_dinsert_before
#define dinsert_after            hor_dinsert_after
#define dfree_list               hor_dfree_list
#define dfree_list_before        hor_dfree_list_before
#define dfree_list_after         hor_dfree_list_after
#define dfree_nodes              hor_dfree_nodes
#define dlist_action             hor_dlist_action
#define dmake_straight_from_list hor_dmake_straight_from_list
#define dmake_circular_from_list hor_dmake_circular_from_list

/* list/association_list.h */
#define Assoc_Label           Hor_Assoc_Label
#define ASSOC_END             -2
#define ASSOC_ERROR           -1
#define ASSOC_START           0
#define assoc_label_contents  hor_assoc_label_contents
#define Assoc_Node            Hor_Assoc_Node
#define Assoc_List            Hor_Assoc_List
#define assoc_label           hor_assoc_label
#define assoc_data            hor_assoc_data
#define assoc_next            hor_assoc_next
#define assoc_null            hor_assoc_null
#define assoc_non_null        hor_assoc_non_null
#define assoc_null_data       hor_assoc_null_data
#define assoc_add             hor_assoc_add
#define assoc_concat          hor_assoc_concat
#define assoc_list_size       hor_assoc_list_size
#define assoc_find            hor_assoc_find
#define assoc_remove          hor_assoc_remove
#define assoc_free            hor_assoc_free
#define assoc_free_nodes      hor_assoc_free_nodes
#define make_assoc_label_list hor_make_assoc_label_list
#define copy_assoc_label_list hor_copy_assoc_label_list
#define free_assoc_label_list hor_free_assoc_label_list

/* include from list/list_io.h */
#ifdef HOR_TRANSPUTER
#define ChanInList  HorChanInList
#define ChanOutList HorChanOutList
#endif
#define read_list   hor_read_list
#define write_list  hor_write_list

/* image/image.h */
#define Image_Type   Hor_Image_Type
#define Bit          HOR_BIT
#define U_Char       HOR_U_CHAR
#define Char         HOR_CHAR
#define U_Short      HOR_U_SHORT
#define Short        HOR_SHORT
#define U_Int        HOR_U_INT
#define Int          HOR_INT
#define Float        HOR_FLOAT
#define Pointer      HOR_POINTER
#define Impixel      Hor_Impixel
#define Imdata       Hor_Imdata
#define Image        Hor_Image
#define Sub_Image    Hor_Sub_Image
#define Image_Window Hor_Image_Window

/* image/image_alloc.h */
#define alloc_image_data hor_alloc_image_data
#define alloc_image      hor_alloc_image
#define alloc_sub_image  hor_alloc_sub_image
#define free_image_data  hor_free_image_data
#define free_image       hor_free_image
#define free_images      hor_free_images
#define free_sub_image   hor_free_sub_image
#define free_sub_images  hor_free_sub_images

/* image/image_misc.h */
#define type_image                       hor_type_image
#define type_sub_image                   hor_type_sub_image
#define extract_from_image               hor_extract_from_image
#define extract_from_sub_image           hor_extract_from_sub_image
#define insert_image_in_image            hor_insert_image_in_image
#define insert_sub_image_in_image        hor_insert_sub_image_in_image
#define add_constant_to_image            hor_add_constant_to_image
#define subtract_constant_from_image     hor_subtract_constant_from_image
#define multiply_image_by_constant       hor_multiply_image_by_constant
#define divide_image_by_constant         hor_divide_image_by_constant
#define add_images                       hor_add_images
#define add_sub_images                   hor_add_sub_images
#define subtract_images                  hor_subtract_images
#define subtract_sub_images              hor_subtract_sub_images
#define subtract_signed_images           hor_subtract_signed_images
#define subtract_signed_sub_images       hor_subtract_signed_sub_images
#define multiply_images                  hor_multiply_images
#define multiply_sub_images              hor_multiply_sub_images
#define multiply_double_images           hor_multiply_double_images
#define multiply_double_sub_images       hor_multiply_double_sub_images
#define divide_images                    hor_divide_images
#define divide_sub_images                hor_divide_sub_images
#define add_constant_to_sub_image        hor_add_constant_to_sub_image
#define subtract_constant_from_sub_image hor_subtract_constant_from_sub_image
#define multiply_sub_image_by_constant   hor_multiply_sub_image_by_constant
#define divide_sub_image_by_constant     hor_divide_sub_image_by_constant
#define fill_image_with_constant         hor_fill_image_with_constant
#define average_images                   hor_average_images
#define average_sub_images               hor_average_sub_images
#define fill_sub_image_with_constant     hor_fill_sub_image_with_constant

/* image/image_compare.h */
#define same_type_images          hor_same_type_images
#define same_type_sub_images      hor_same_type_sub_images
#define same_dims_images          hor_same_dims_images
#define same_dims_sub_images      hor_same_dims_sub_images
#define same_type_dims_images     hor_same_type_dims_images
#define same_type_dims_sub_images hor_same_type_dims_sub_images

/* image/convert_image.h */
#define convert_image_data hor_convert_image_data
#define convert_image      hor_convert_image
#define convert_sub_image  hor_convert_sub_image
#define copy_image         hor_copy_image
#define copy_sub_image     hor_copy_sub_image
#define subsample          hor_subsample

/* image/image_io.h */
#ifdef HOR_TRANSPUTER
#define ChanInImageHeader       HorChanInImageHeader
#define ChanInSubImageHeader    HorChanInSubImageHeader
#define ChanInImageData         HorChanInImageData
#define ChanInImage             HorChanInImage
#define ChanInSubImage          HorChanInSubImage
#define ChanInAllocatedImage    HorChanInAllocatedImage
#define ChanInAllocatedSubImage HorChanInAllocatedSubImage
#define ChanOutImageHeader      HorChanOutImageHeader
#define ChanOutSubImageHeader   HorChanOutSubImageHeader
#define ChanOutImageData        HorChanOutImageData
#define ChanOutImage            HorChanOutImage
#define ChanOutSubImage         HorChanOutSubImage
#define ChanInSubImageData      HorChanInSubImageData
#define ChanOutSubImageData     HorChanOutSubImageData
#endif

/* image/image_file.h */
#define Image_Format       Hor_Image_Format
#define MIT_format         HOR_MIT_FORMAT
#define IFF_format         HOR_IFF_FORMAT
#define set_image_format   hor_set_image_format
#define read_image_stream  hor_read_image_stream
#define read_image         hor_read_image
#define write_image_stream hor_write_image_stream
#define write_image        hor_write_image

/* image/iff_image_file.h */
#define read_iff_image_stream  hor_read_iff_image_stream
#define read_iff_image         hor_read_iff_image
#define write_iff_image_stream hor_write_iff_image_stream
#define write_iff_image        hor_write_iff_image

/* image/mit_image_file.h */
#define read_mit_image_stream  hor_read_mit_image_stream
#define read_mit_image         hor_read_mit_image
#define write_mit_image_stream hor_write_mit_image_stream
#define write_mit_image        hor_write_mit_image

/* image/read_image_sequence.h */
#define Count_String     Hor_Count_String
#define count_string     hor_count_string
#define read_next_image  hor_read_next_image
#define write_next_image hor_write_next_image

/* image/image_stream_io.h */
#define read_image_data_from_stream     hor_read_image_data_from_stream
#define read_image_from_stream          hor_read_image_from_stream
#define read_sub_image_from_stream      hor_read_sub_image_from_stream
#define write_image_data_to_stream      hor_write_image_data_to_stream
#define write_image_to_stream           hor_write_image_to_stream
#define write_sub_image_to_stream       hor_write_sub_image_to_stream
#define read_sub_image_data_from_stream hor_read_sub_image_data_from_stream
#define write_sub_image_data_to_stream  hor_write_sub_image_data_to_stream

/* graphics/print.h */
#define set_text_window    hor_set_text_window
#define reset_text_window  hor_reset_text_window
#define delete_text_window hor_delete_text_window

/* graphics/colourmap.h */
#define X_to_Image_ColourMap     Hor_X_to_Image_ColourMap
#define colourmap_setup          hor_colourmap_setup
#define colour_alloc             hor_colour_alloc
#define free_colourmap           hor_free_colourmap
#define get_colour_name_value    hor_get_colour_name_value
#define get_image_display_format hor_get_image_display_format
#define get_image_display_depth  hor_get_image_display_depth
#define Grey                     Hor_Grey
#define NEUTRAL_COLOUR           0xffffffff

/* graphics/display.h */
#define button_func                      hor_button_func
#define window_func                      hor_window_func
#define display_set_string               hor_display_set_string
#define display_delete_string            hor_display_delete_string
#define Display_Function                 Hor_Display_Function
#define display_copy                     HOR_DISPLAY_COPY
#define display_xor                      HOR_DISPLAY_XOR
#define display_initialise               hor_display_initialise
#define display_set_window               hor_display_set_window
#define display_reset_window             hor_display_reset_window
#define display_get_window               hor_display_get_window
#define display_delete_window            hor_display_delete_window
#define display_set_mouse_functions      hor_display_set_mouse_functions
#define display_remove_mouse_functions   hor_display_remove_mouse_functions
#define display_set_params               hor_display_set_params
#define display_canvas_set_params        hor_display_canvas_set_params
#define display_get_params               hor_display_get_params
#define display_get_dims                 hor_display_get_dims
#define display_set_function             hor_display_set_function
#define display_set_colour               hor_display_set_colour
#define display_set_line_width           hor_display_set_line_width
#define display_load_font                hor_display_load_font
#define display_set_font                 hor_display_set_font
#define display_get_fontstruct           hor_display_get_fontstruct
#define display_text                     hor_display_text
#define display_canvas_text              hor_display_canvas_text
#define display_clear                    hor_display_clear
#define display_flush                    hor_display_flush
#define display_point_convert            hor_display_point_convert
#define display_region_convert           hor_display_region_convert
#define display_within_image             hor_display_within_image
#define display_draw_canvas_rectangle    hor_display_draw_canvas_rectangle
#define display_fill_canvas_rectangle    hor_display_fill_canvas_rectangle
#define display_highlight_region         hor_display_highlight_region
#define display_draw_rectangle           hor_display_draw_rectangle
#define display_fill_rectangle           hor_display_fill_rectangle
#define display_fill_diamond             hor_display_fill_diamond
#define display_draw_circle              hor_display_draw_circle
#define display_draw_circle_actual_size  hor_display_draw_circle_actual_size
#define display_fill_circle              hor_display_fill_circle
#define display_fill_circle_actual_size  hor_display_fill_circle_actual_size
#define display_draw_ellipse_actual_size hor_display_draw_ellipse_actual_size
#define display_plot                     hor_display_plot
#define display_line                     hor_display_line
#define display_get_pixel                hor_display_get_pixel
#define display_image                    hor_display_image
#define display_sub_image                hor_display_sub_image
#define display_subsampled_image         hor_display_subsampled_image
#define get_xor_colour                   hor_display_get_xor_colour
#define display_store_state              hor_display_store_state
#define display_recall_state             hor_display_recall_state
#define display_destroy_state            hor_display_destroy_state
#define display_get_image                hor_display_get_image
#define display_write_to_file            hor_display_write_to_file
#define display_read_from_file           hor_display_read_from_file
#define display_make_movie               hor_display_make_movie
#define display_make_movie_image         hor_display_make_movie_image
#define display_show_movie_image         hor_display_show_movie_image
#define display_destroy_movie            hor_display_destroy_movie

/* improc/adjust_region.h */
#define adjust_region_for_border hor_adjust_region_for_border

/* improc/convolution.h */
#define convolve_image hor_convolve_image

/* improc/gaussian_mask.h */
#define make_gaussian_mask hor_make_gaussian_mask
#define free_gaussian_mask hor_free_gaussian_mask

/* improc/canny.h */
#define Edgel     Hor_Edgel
#define Estring   Hor_Estring
#define EdgeRect  Hor_Edge_Map
#define canny     hor_canny
#define ISOLATED  0
#define IN_STRING 1

/* improc/canny_alloc.h */
#define alloc_edgerect hor_alloc_edge_map
#define free_edgerect  hor_free_edge_map
#define free_string    hor_free_string

/* improc/canny_file.h */
#define write_edgerect hor_write_edge_map
#define read_edgerect  hor_read_edge_map

/* improc/corner.h */
#define Corner     Hor_Corner
#define CornerRect Hor_Corner_Map
#define corners    hor_plessey_corners

/* improc/smith_corner.h */
#define smith_corners hor_smith_corners

/* improc/corner_match.h */
#define CMatch        Hor_CMatch
#define Corner_Match  Hor_Corner_Match
#define match_corners hor_pab_match_corners

/* improc/corner_alloc.h */
#define alloc_cornerrect hor_alloc_corner_map
#define free_cornerrect  hor_free_corner_map
#define copy_cornerrect  hor_copy_corner_map

/* improc/corner_file.h */
#define write_cornerrect hor_write_corner_map
#define read_cornerrect  hor_read_corner_map

/* improc/image_flow.h */
#define Image_Flow      Hor_Image_Flow
#define image_flow      hor_image_flow
#define free_image_flow hor_free_image_flow

/* improc/image_segment.h */
#define Image_Segment Hor_Image_Segment
#define image_segment hor_image_segment

/* improc/correlation.h */
#define Correlation Hor_Correlation
#define correlation hor_correlation

/* improc/canny_display.h */
#define display_edgerect hor_display_edge_map
#define output_edgerect  hor_output_edge_map

/* improc/corner_display.h */
#define display_cornerrect     hor_display_corner_map
#define display_corner_matches hor_display_corner_matches

/* improc/image_flow_display.h */
#define display_image_flow hor_display_image_flow

/* process/init_process.h */
#define init_process_stuff hor_init_process_stuff

/* process/process.h */
#define Output_Result_Func Hor_Output_Result_Func
#define Free_Result_Func   Hor_Free_Result_Func
#define Free_Data_Func     Hor_Free_Data_Func
#define Execute_Func       Hor_Execute_Func
#define Update_Data_Func   Hor_Update_Data_Func
#define add_result_type    hor_add_result_type
#define add_process_type   hor_add_process_type
#define add_process        hor_add_process
#define execute_processes  hor_execute_processes
#define clear_processes    hor_clear_processes
#define Process_Input      Hor_Process_Input
                  
/* process/grab_proc.h */
#define GB_Process_Data           Hor_GB_Process_Data
#define set_gb_result_type_label  hor_set_gb_result_type_label
#define set_gb_process_type_label hor_set_gb_process_type_label
#define get_gb_result_type_label  hor_get_gb_result_type_label
#define get_gb_process_type_label hor_get_gb_process_type_label
#define get_gb_process_label      hor_get_gb_process_label
#define gb_make_process_data      hor_gb_make_process_data
#define gb_add_process            hor_gb_add_process
#define gb_execute                hor_gb_execute
#define gb_output                 hor_gb_output
#define gb_update_process_data    hor_gb_update_process_data
#define gb_free_process_data      hor_gb_free_process_data
#define gb_free_result            hor_gb_free_result

/* process/canny_proc.h */
#define CA_Process_Params         Hor_CA_Process_Params
#define CA_Fixed                  Hor_CA_Fixed
#define CA_Writeable              Hor_CA_Writeable
#define CA_Process_Data           Hor_CA_Process_Data
#define CA_Output_Params          Hor_CA_Output_Params
#define set_ca_result_type_label  hor_set_ca_result_type_label
#define set_ca_process_type_label hor_set_ca_process_type_label
#define get_ca_result_type_label  hor_get_ca_result_type_label
#define get_ca_process_type_label hor_get_ca_process_type_label
#define ca_make_process_data      hor_ca_make_process_data
#define ca_make_output_data       hor_ca_make_output_data
#define ca_add_process            hor_ca_add_process
#define ca_execute                hor_ca_execute
#define ca_output                 hor_ca_output
#define ca_update_process_data    hor_ca_update_process_data
#define ca_free_process_data      hor_ca_free_process_data
#define ca_free_result            hor_ca_free_result

/* process/corner_proc.h */                  
#define CO_Output_Params          Hor_CO_Output_Params
#define set_co_result_type_label  hor_set_co_result_type_label
#define get_co_result_type_label  hor_get_co_result_type_label
#define co_make_output_data       hor_co_make_output_data
#define co_output                 hor_co_output
#define co_free_result            hor_co_free_result

/* process/plessey_corner_proc.h */
#define CO_Process_Params         Hor_PC_Process_Params
#define CO_Fixed                  Hor_PC_Fixed
#define CO_Writeable              Hor_PC_Writeable
#define CO_Process_Data           Hor_PC_Process_Data
#define set_co_process_type_label hor_set_pc_process_type_label
#define get_co_process_type_label hor_get_pc_process_type_label
#define co_make_process_data      hor_pc_make_process_data
#define co_add_process            hor_pc_add_process
#define co_execute                hor_pc_execute
#define co_update_process_data    hor_pc_update_process_data
#define co_free_process_data      hor_pc_free_process_data

/* process/smith_corner_proc.h */
#define SC_Process_Params         Hor_SC_Process_Params
#define SC_Fixed                  Hor_SC_Fixed
#define SC_Process_Data           Hor_SC_Process_Data
#define set_sc_process_type_label hor_set_sc_process_type_label
#define get_sc_process_type_label hor_get_sc_process_type_label
#define sc_make_process_data      hor_sc_make_process_data
#define sc_add_process            hor_sc_add_process
#define sc_execute                hor_sc_execute
#define sc_update_process_data    hor_sc_update_process_data

/* process/corner_match_proc.h */
#define CM_Process_Params Hor_BM_Process_Params
#define CM_Output_Params Hor_CM_Output_Params
#define set_cm_result_type_label hor_set_cm_result_type_label
#define set_cm_process_type_label hor_set_bm_process_type_label
#define get_cm_result_type_label hor_get_cm_result_type_label
#define get_cm_process_type_label hor_get_bm_process_type_label
#define cm_make_process_data hor_bm_make_process_data
#define cm_make_output_data hor_cm_make_output_data
#define cm_add_process hor_bm_add_process
#define cm_execute hor_bm_execute
#define cm_output     hor_cm_output
#define cm_free_result hor_cm_free_result

/* process/image_flow_proc.h */
#define FL_Process_Params         Hor_FL_Process_Params
#define FL_Process_Data           Hor_FL_Process_Data
#define FL_Output_Params          Hor_FL_Output_Params
#define set_fl_result_type_label  hor_set_fl_result_type_label
#define set_fl_process_type_label hor_set_fl_process_type_label
#define get_fl_result_type_label  hor_get_fl_result_type_label
#define get_fl_process_type_label hor_get_fl_process_type_label
#define fl_make_process_data      hor_fl_make_process_data
#define fl_make_output_data       hor_fl_make_output_data
#define fl_add_process            hor_fl_add_process
#define fl_execute                hor_fl_execute
#define fl_output                 hor_fl_output
#define fl_free_process_data      hor_fl_free_process_data
#define fl_free_result            hor_fl_free_result

/* process/image_segment_proc.h */
#define IS_Process_Params         Hor_IS_Process_Params
#define IS_Process_Data           Hor_IS_Process_Data
#define IS_Output_Params          Hor_IS_Output_Params
#define set_is_result_type_label  hor_set_is_result_type_label
#define set_is_process_type_label hor_set_is_process_type_label
#define get_is_result_type_label  hor_get_is_result_type_label
#define get_is_process_type_label hor_get_is_process_type_label
#define is_make_process_data      hor_is_make_process_data
#define is_make_output_data       hor_is_make_output_data
#define is_add_process            hor_is_add_process
#define is_execute                hor_is_execute
#define is_output                 hor_is_output
#define is_free_result            hor_is_free_result

/* process/image_segment_proc.h */
#define CL_Process_Params         Hor_CL_Process_Params
#define CL_Process_Data           Hor_CL_Process_Data
#define CL_Output_Params          Hor_CL_Output_Params
#define set_cl_result_type_label  hor_set_cl_result_type_label
#define set_cl_process_type_label hor_set_cl_process_type_label
#define get_cl_result_type_label  hor_get_cl_result_type_label
#define get_cl_process_type_label hor_get_cl_process_type_label
#define cl_make_process_data      hor_cl_make_process_data
#define cl_make_output_data       hor_cl_make_output_data
#define cl_add_process            hor_cl_add_process
#define cl_execute                hor_cl_execute
#define cl_output                 hor_cl_output
#define cl_free_result            hor_cl_free_result

/* tool/velocity_panel.h */
#define fill_velocity_panel        hor_fill_velocity_panel
#define set_velocity_defaults      hor_set_velocity_defaults
#define get_upper_left_c_velocity  hor_get_upper_left_c_velocity
#define get_upper_left_r_velocity  hor_get_upper_left_r_velocity
#define get_lower_right_c_velocity hor_get_lower_right_c_velocity
#define get_lower_right_r_velocity hor_get_lower_right_r_velocity

/* tool/process_code.h */
#define IMAGE_CODE         0x0
#define CANNY_CODE         0x1
#define CORNER_CODE        0x2
#define SMITH_CORNER_CODE  0x4
#define IMAGE_FLOW_CODE    0x8
#define IMAGE_SEGMENT_CODE 0x10
#define CORRELATION_CODE   0x20
#define CANNY_MASK         (0xffffffff ^            CANNY_CODE)
#define CORNER_MASK        (0xffffffff ^           CORNER_CODE)
#define SMITH_CORNER_MASK  (0xffffffff ^     SMITH_CORNER_CODE)
#define IMAGE_FLOW_MASK    (0xffffffff ^       IMAGE_FLOW_CODE)
#define IMAGE_SEGMENT_MASK (0xffffffff ^    IMAGE_SEGMENT_CODE)
#define CORRELATION_MASK   (0xffffffff ^      CORRELATION_CODE)
#define get_process_code   hor_get_process_code
#define set_process_code   hor_set_process_code

/* tool/canny_tool.h */
#define create_canny_popup hor_create_canny_popup
#define popup_canny_params hor_popup_canny_params
#define set_canny_colours  hor_set_canny_colours
#define get_canny_params   hor_get_canny_params
#define get_canny_colours  hor_get_canny_colours

/* tool/corner_tool.h */
#define create_corner_popup hor_create_plessey_corner_popup
#define popup_corner_params hor_popup_plessey_corner_params
#define set_corner_colours  hor_set_plessey_corner_colours
#define get_corner_params   hor_get_plessey_corner_params
#define get_corner_colours  hor_get_plessey_corner_colours

/* tool/smith_corner_tool.h */
#define create_smith_corner_popup hor_create_smith_corner_popup
#define popup_smith_corner_params hor_popup_smith_corner_params
#define set_smith_corner_colours  hor_set_smith_corner_colours
#define get_smith_corner_params   hor_get_smith_corner_params
#define get_smith_corner_colours  hor_get_smith_corner_colours

/* tool/corner_match_tool.h */
#define create_corner_match_popup hor_create_pab_corner_match_popup
#define popup_corner_match_params hor_popup_pab_corner_match_params
#define set_corner_match_colours hor_set_pab_corner_match_colours
#define get_corner_match_params hor_get_pab_corner_match_params

/* tool/image_flow_tool.h */
#define create_image_flow_popup hor_create_image_flow_popup
#define popup_image_flow_params hor_popup_image_flow_params
#define set_image_flow_colours  hor_set_image_flow_colours
#define get_image_flow_params   hor_get_image_flow_params

/* tool/image_segment_tool.h */
#define create_image_segment_popup hor_create_image_segment_popup
#define popup_image_segment_params hor_popup_image_segment_params
#define set_image_segment_colours  hor_set_image_segment_colours
#define get_image_segment_params   hor_get_image_segment_params

/* tool/correlation_tool.h */
#define create_correlation_popup hor_create_correlation_popup
#define popup_correlation_params hor_popup_correlation_params
#define set_correlation_colours  hor_set_correlation_colours
#define get_correlation_params   hor_get_correlation_params

/* tool/popup.h */
#define set_widget_value          hor_set_widget_value
#define get_widget_value          hor_get_widget_value
#define create_text               hor_create_text
#define hide_popup                hor_hide_popup
#define create_reset_done_buttons hor_create_reset_done_buttons

/* tool/memory_popup.h */
#define create_memory_panel    hor_create_memory_panel
#define popup_memory_panel     hor_popup_memory_panel
#define memory_panel_in_use    hor_memory_panel_in_use
#define redisplay_memory_panel hor_redisplay_memory_panel

/* tool/graph_popup.h */
#define create_graph     hor_create_graph
#define register_graphs  hor_register_graphs
#define popup_graph      hor_popup_graph
#define popdown_graph    hor_popdown_graph
#define graph_point      hor_graph_point
#define graph_reset_time hor_graph_reset_time
#define graph_in_use     hor_graph_in_use
#define redisplay_graph  hor_redisplay_graph
#define write_graph      hor_write_graph

/* tool/threed_popup.h */
#define Item_3d_point         Hor_Item_3D_point
#define Item_3d_line          Hor_Item_3D_line
#define Item_3d               Hor_Item_3D
#define LINE_3D               HOR_LINE_3D
#define POINT_3D              HOR_POINT_3D
#define Threed_OneSelect_Func Hor_3D_OneSelect_Func
#define Threed_AllSelect_Func Hor_3D_AllSelect_Func
#define Threed_Finish_Func    Hor_3D_Finish_Func
#define create_threed         hor_create_3D
#define register_threed       hor_register_3D
#define popup_threed          hor_popup_3D
#define threed_item           hor_3D_item
#define init_threed           hor_init_3D
#define colour_threed_item    hor_colour_3D_item
#define threed_in_use         hor_3D_in_use
#define redisplay_threed      hor_redisplay_3D
#define write_threed          hor_write_3D

/* tool/get_region.h */
#define region_delete_last_selected hor_region_delete_last_selected
#define region_delete_selected      hor_region_delete_selected
#define region_set_function         hor_region_set_function
#define region_start                hor_region_start
#define region_moving               hor_region_moving
#define region_finish               hor_region_finish
#define region_cancel               hor_region_cancel
#define region_select               hor_region_select
#define region_clear                hor_region_clear

/* tool/read_param.h */
#define read_float_param hor_read_float_param
#define read_int_param   hor_read_int_param

/* tool/test_param.h */
#define int_null         hor_int_null
#define int_pos          hor_int_pos
#define int_abs_pos      hor_int_abs_pos
#define int_abs_pos_even hor_int_abs_pos_even
#define float_null       hor_float_null
#define float_pos        hor_float_pos
#define float_abs_pos    hor_float_abs_pos
#define float_01_inc     hor_float_01_inc
