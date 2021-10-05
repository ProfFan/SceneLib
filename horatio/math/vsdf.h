/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/vsdf.h */

#ifdef _HORATIO_LIST_
void hor_vsdf_init ( Hor_Assoc_Label label,
		     Hor_Matrix *xf0, int xf_type, Hor_Matrix *Pf0inv,
		     Hor_Matrix *xd0, int xd_type,
		     void *user_state, int max_y_size, int max_z_size,
		     int max_n, Hor_Bool use_optimal_dynamic_vsdf );
Hor_Bool hor_vsdf_initialised(void);
void hor_vsdf_switch ( Hor_Assoc_Label label );
void hor_vsdf_free ( Hor_Assoc_Label label );
#endif /* _HORATIO_LIST_ */

void *hor_vsdf_get_user_state(void);
void hor_vsdf_set_user_state ( void *new_user_state );

int hor_vsdf_add_state ( Hor_Matrix *y0, int y_type,
			 Hor_Matrix *T0inv, int *index_ptr );

/*******************
*   #define @HOR_VSDF_UNINIT_OFFSET 16383 (offset for start index of
*                                          uninitialized local states)
*   #define @HOR_VSDF_ERROR            -1 (index and return value signifying
*                                          an error)
*   #define @HOR_VSDF_NOT_PRESENT      -2 (index and return value signifying
*                                          lack of presence in VSDF)
*   #define @HOR_VSDF_OUTLIER          -3 (index and return value signifying
*                                          an outlier detected in the VSDF)
********************/
#define HOR_VSDF_UNINIT_OFFSET 16383
#define HOR_VSDF_ERROR            -1
#define HOR_VSDF_NOT_PRESENT      -2
#define HOR_VSDF_OUTLIER          -3

int hor_vsdf_addu_state ( int *index_ptr );
int hor_vsdf_uninit_frames ( int index );
int hor_vsdf_init_state ( int index, int iterations, double conf_level,
			  Hor_Matrix *(*y0_func)(Hor_Matrix  *xf, int  xf_type,
						 Hor_Matrix **xd, int *xd_type,
						 Hor_Matrix **z,  int  *z_type,
						 void       **user_data, int k,
						 void        *user_state,
						 int  *y_type) );
void hor_vsdf_remove_state ( int index, Hor_Bool keep_data );
Hor_Bool hor_vsdf_test_state ( int index, double conf_level );

void hor_vsdf_obs_h ( int index, Hor_Matrix *z, int z_type, Hor_Matrix *Rinv,
		      void (*h_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type, int z_type,
				      Hor_Matrix *h,  Hor_Matrix *D,
				      Hor_Matrix *Dp, Hor_Matrix *E,
				      void *user_data, void *user_state),
		      void *user_data );
void hor_vsdf_obs_F ( int index, Hor_Matrix *z, int z_type,
		      Hor_Matrix *N, int F_size,
		      void (*F_func) (Hor_Matrix *xf, int xf_type,
				      Hor_Matrix *xd, int xd_type,
				      Hor_Matrix *y,  int  y_type,
				      Hor_Matrix *z,  int  z_type,
				      Hor_Matrix *F,  Hor_Matrix *D,
				      Hor_Matrix *Dp, Hor_Matrix *E,
				      Hor_Matrix *Fz,
				      void *user_data, void *user_state),
		      void *user_data );
void hor_vsdf_obs_h_num ( int index, Hor_Matrix *z, int z_type,
			  Hor_Matrix *Rinv,
			  void (*hn_func) (Hor_Matrix *xf, int xf_type,
					   Hor_Matrix *xd, int xd_type,
					   Hor_Matrix *y,  int  y_type,
					   int  z_type, Hor_Matrix *h,
					   void *user_data, void *user_state),
			  void *user_data );

void hor_vsdf_store(void);
Hor_Bool hor_vsdf_batch ( Hor_Bool (*init_func)(Hor_Matrix  *xf, int *xf_type,
						Hor_Matrix **xd,
						int         *xd_type, int k,
						Hor_Matrix **y,  int *y_type,
						                 int n,
						Hor_Matrix **yu, int *yu_type,
						                 int nu,
						Hor_Matrix ***zu,
						int         **zu_type,
						void       ***data_u,
						void *user_data,
						void *user_state),
			  int iterations1, int iterations2,
			  double conf_level, void *user_data );

Hor_Bool hor_vsdf_change_frame(Hor_Bool (*change_func)(Hor_Matrix  *xf,
						       int *xf_type,
						       Hor_Matrix *Af,
						       Hor_Matrix  *xd,
						       int *xd_type,
						       int n,
						       Hor_Matrix **y,
						       int *y_type,
						       Hor_Matrix **vf,
						       Hor_Matrix **C));

typedef enum { HOR_MOTION_UPDATE, HOR_DROID_UPDATE, HOR_FULL_UPDATE }
   Hor_VSDF_Update_Type;
Hor_Bool *hor_vsdf_update ( int dynamic_iterations, double conf_level,
			    Hor_VSDF_Update_Type update_type,
			    void (*update_func)(Hor_Matrix  *xf, int  xf_type,
						Hor_Matrix  *xd, int *xd_type,
						Hor_Matrix **y,  int *y_type,
						Hor_Matrix **C,
						Hor_Matrix **z,  int *z_type,
						void       **zdata,
						Hor_Bool    *z_accept,
						int          n,
						void *user_data,
						void *user_state),
			    void (*reset_func)(Hor_Matrix  *xf, int  xf_type,
					       Hor_Matrix  *xd, int *xd_type,
					       Hor_Matrix **y,  int  *y_type,
					       Hor_Matrix **C,
					       Hor_Matrix **z,  int *z_type,
					       void       **zdata,
					       Hor_Bool    *z_accept,
					       int          n,
					       void *user_data,
					       void *user_state),
			    void *user_data );

void hor_vsdf_scale ( double factor );
void hor_vsdf_reset(void);

typedef struct
{
   Hor_Matrix *xd;
   int         xd_type;
   Hor_Matrix *Ad;
} Hor_VSDF_XD_Def;

int         hor_vsdf_get_k(void);
Hor_Matrix *hor_vsdf_get_xf(void);
int         hor_vsdf_get_xf_type(void);
Hor_Matrix *hor_vsdf_get_xd(void);
int         hor_vsdf_get_xd_type(void);
#ifdef _HORATIO_LIST_
Hor_List    hor_vsdf_get_xdlist(void); /* returns a list of pointers
					  to Hor_VSDF_XD_Def's */
#endif
Hor_Matrix *hor_vsdf_get_Pf(void);
Hor_Matrix *hor_vsdf_get_Af(void);
Hor_Matrix *hor_vsdf_get_Pd(void);
Hor_Matrix *hor_vsdf_get_Ad(void);
double      hor_vsdf_get_J(void);
int         hor_vsdf_get_DOF(void);
int         hor_vsdf_get_n(void);
int         hor_vsdf_get_max_ysize(void);
int         hor_vsdf_get_max_zsize(void);
Hor_Matrix *hor_vsdf_get_y      ( int index );
int         hor_vsdf_get_y_type ( int index );
Hor_Matrix *hor_vsdf_get_T      ( int index );
Hor_Matrix *hor_vsdf_get_C      ( int index );
int         hor_vsdf_get_obs_k  ( int index );
void       *hor_vsdf_get_data   ( int index );
double      hor_vsdf_get_Ji     ( int index );
int         hor_vsdf_get_DOFi   ( int index );
Hor_Matrix *hor_vsdf_get_z      ( int index );
int         hor_vsdf_get_z_type ( int index );

void hor_vsdf_print_state(void);
