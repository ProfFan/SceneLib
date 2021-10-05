#define HOR_FM_BOOLEAN int
#define HOR_FM_DEBUG 1
#define HOR_FM_MATCH_MASK1_INDEX 3   /* the origin of the matching mask in the first corner patch is (3,3) */
#define HOR_FM_MATCH_MASK_SIZE 7     /* size of mask used for matching */
#define HOR_FM_MATCH_PATCH_SIZE 13   /* total size of corner patch */
#define HOR_FM_MATCH_SEARCH_OFFSET 2 /* specifies range of shifts for the mask in the second corner patch, when looking for a match */
#define HOR_FM_NO_MATCH -1
#define HOR_FM_TRIVEC_HOMG_VALUE 256

/* macros */

#define HOR_FM_MATX_INDEX_2DTO1D(row_index, col_index, col_count) (((row_index) * (col_count)) + (col_index))
#define HOR_FM_ROUND(x) ((int) ((x) + 0.5))
#define HOR_FM_SQUARE(x) ((x) * (x))

/* parameters */

#define HOR_FM_MATCH_EPIPOLAR_THRESHOLD 3.0
#define HOR_FM_FMATX_EPIPOLE_THRESHOLD 10.0
#define HOR_FM_FMATX_OUTLIER_THRESHOLD 2.0
#define HOR_FM_FMATX_WEIGHTING_MIN 0.2
#define HOR_FM_FMATX_ELIMINATE_RATIO_MAX 0.4

typedef struct {
	double x;
	double y;
    } HOR_FM_BIVEC_DOUBLE;

typedef struct {
	int x;
	int y;
    } HOR_FM_BIVEC_INT;

typedef struct {
	double x;
	double y;
	double z;
    } HOR_FM_TRIVEC_DOUBLE;

typedef struct HOR_FM_CORNER_STR {
	HOR_FM_BIVEC_DOUBLE bivec;
	HOR_FM_BIVEC_INT i_bivec;
	HOR_FM_TRIVEC_DOUBLE trivec;
	int match_back_index;
	int match_fore_index;
	Hor_Image *patch_ptr;
    } HOR_FM_CORNER_STR;

typedef struct HOR_FM_CORNER_TABLE_STR {
	int count;
	HOR_FM_CORNER_STR array [1];
    } HOR_FM_CORNER_TABLE_STR;

typedef struct {
	HOR_FM_BOOLEAN found_flag;
	int match_index;
	float affinity;
    } HOR_FM_MATCH_STR;

typedef struct {
	int count;
	HOR_FM_MATCH_STR array [1];
    } HOR_FM_MATCH_TABLE_STR;

typedef struct HOR_FM_FMATRIX_PARAMS_STR {
	float epipole_distance_threshold;
	float epipolar_outlier_threshold;
	float epipolar_weighting_min;
	float eliminate_ratio_max;
    } HOR_FM_FMATRIX_PARAMS_STR;

typedef struct HOR_FM_MATCH_PARAMS_STR {
        int distance_threshold;
	int affinity_threshold;
	float epipolar_distance_threshold;
    } HOR_FM_MATCH_PARAMS_STR;

extern void hor_fm_match_corners (HOR_FM_MATCH_PARAMS_STR *match_params_ptr, HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, 
				  HOR_FM_MATCH_TABLE_STR *match_table_ptr);
extern void hor_fm_match_corners_fmatrix (HOR_FM_MATCH_PARAMS_STR *match_params_ptr, double f_matrix [3][3], HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, 
				      HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, HOR_FM_MATCH_TABLE_STR *match_table_ptr);

extern void hor_fm_compute_epipoles (double f_matrix [3][3], HOR_FM_TRIVEC_DOUBLE *epipole1_trivec_ptr, HOR_FM_TRIVEC_DOUBLE *epipole2_trivec_ptr);
extern HOR_FM_BOOLEAN hor_fm_compute_fmatrix_linear (HOR_FM_FMATRIX_PARAMS_STR *fmatrix_params_ptr, HOR_FM_CORNER_TABLE_STR *corner_table1_ptr, HOR_FM_CORNER_TABLE_STR *corner_table2_ptr, 
						     double f_matrix [3][3]);

extern HOR_FM_TRIVEC_DOUBLE hor_fm_bivec_to_trivec (HOR_FM_BIVEC_DOUBLE *bivec_ptr);
extern float hor_fm_compute_perpdistance_squared (HOR_FM_TRIVEC_DOUBLE *line_trivec_ptr, HOR_FM_TRIVEC_DOUBLE *point_trivec_ptr);
extern float hor_fm_distance_squared_between_points (HOR_FM_TRIVEC_DOUBLE *trivec1_ptr, HOR_FM_TRIVEC_DOUBLE *trivec2_ptr);
extern void hor_fm_matx_multiply (int row_count1, int col_count1, double *matrix1_ptr, int row_count2, int col_count2, double *matrix2_ptr, double *matrix_product_ptr);
extern void hor_fm_matx_premultiply_by_transpose (int row_count, int col_count, double *source_ptr, double *target_ptr);
extern void hor_fm_matx_transpose (int row_count, int col_count, double *matrix_ptr, double *matrix_transpose_ptr);
extern HOR_FM_BIVEC_DOUBLE hor_fm_trivec_to_bivec (HOR_FM_TRIVEC_DOUBLE *trivec_ptr);
