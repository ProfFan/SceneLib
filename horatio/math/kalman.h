/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from math/kalman.h */

void hor_kalman_init ( Hor_Matrix *x0, /* initial value of state vector     */
		       Hor_Matrix *P0, /* prior covariance or x0            */
		       int     usize,  /* size of control vector            */
		       int     max_zsize ); /* maximum size of observation
					       vector*/
void hor_kalman_predict ( Hor_Matrix *F, /* state transition matrix          */
			  Hor_Matrix *G, /* input (control) transition matrix*/
			  Hor_Matrix *u, /* control vector                   */
			  Hor_Matrix *Q ); /* prediction error covariance    */
Hor_Bool hor_kalman_update ( Hor_Matrix *z, /* observation vector           */
			     Hor_Matrix *H, /* observation matrix           */
			     Hor_Matrix *R, /* observation covariance       */
			     double  gate_thres ); /* threshold on validation
						      gate */
Hor_Bool hor_kalman_step ( Hor_Matrix *F, /* state transition matrix */
			   Hor_Matrix *G, /* input (control) transition
					     matrix */
			   Hor_Matrix *u, /* control vector */
			   Hor_Matrix *Q, /* prediction error covariance */
			   Hor_Matrix *z, /* observation vector */
			   Hor_Matrix *H, /* observation matrix */
			   Hor_Matrix *R, /* observation covariance */
			   double  gate_thres ); /* threshold on validation
						    gate */

Hor_Bool hor_ext_kalman_update ( Hor_Matrix *z,
				 void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
						Hor_Matrix *grad_hT),
				 Hor_Matrix *R, double gate_thres );
Hor_Bool hor_ext_kalman_step ( Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
			       Hor_Matrix *Q, Hor_Matrix *z,
			       void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					      Hor_Matrix *grad_hT),
			       Hor_Matrix *R, double  gate_thres );

Hor_Matrix *hor_kalman_state_vector(void);
Hor_Matrix *hor_kalman_covariance(void);
void        hor_kalman_print_state(void);

/* multiple filter functions */

/*******************
*   typedef enum { @HOR_EKF_BASIC, @HOR_EKF_ITERATED, @HOR_EKF_RECURSIVE }
*      @Hor_EKF_Mode;
*
*   Modes for running Extended Kalman Filter:
*
*   HOR_EKF_BASIC:     Basic linear EKF.
*   HOR_EKF_ITERATED:  Iterated extended Kalman filter.
*   HOR_EKF_RECURSIVE: Recursively iterated extended Kalman filter.
********************/
typedef enum { HOR_EKF_BASIC, HOR_EKF_ITERATED, HOR_EKF_RECURSIVE }
   Hor_EKF_Mode;

#ifdef _HORATIO_LIST_

void hor_kf_init ( Hor_Assoc_Label label, Hor_Matrix *x0, Hor_Matrix *P0,
		   int usize, int max_zsize );
void hor_kf_predict ( Hor_Assoc_Label label, Hor_Matrix *F,
		      Hor_Matrix *G, Hor_Matrix *u, Hor_Matrix *Q );
Hor_Bool hor_kf_update ( Hor_Assoc_Label label, Hor_Matrix *z, Hor_Matrix *H,
			 Hor_Matrix *R, double gate_thres );
Hor_Bool hor_kf_step ( Hor_Assoc_Label label,
		       Hor_Matrix *F, Hor_Matrix *G, Hor_Matrix *u,
		       Hor_Matrix *Q, Hor_Matrix *z, Hor_Matrix *H,
		       Hor_Matrix *R, double  gate_thres );

void hor_ekf_set_mode ( Hor_Assoc_Label label, Hor_EKF_Mode mode );
void hor_ekf_set_iterations ( Hor_Assoc_Label label, int iterations );
void hor_ekf_set_threshold ( Hor_Assoc_Label label, double threshold );
void hor_ekf_predict ( Hor_Assoc_Label label,
		       void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				      Hor_Matrix *f, Hor_Matrix *grad_fT,
				      void *data),
		       Hor_Matrix *u, Hor_Matrix *Q, void *predict_data );
Hor_Bool hor_ekf_update ( Hor_Assoc_Label label, Hor_Matrix *z,
			  void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
					 Hor_Matrix *grad_hT, void *data),
			  Hor_Matrix *R, double gate_thres, void *update_data);
Hor_Bool hor_ekf_step ( Hor_Assoc_Label label,
		        void (*f_func)(Hor_Matrix *x, Hor_Matrix *u,
				       Hor_Matrix *f, Hor_Matrix *grad_fT,
				       void *data),
		        Hor_Matrix *u, Hor_Matrix *Q, void *predict_data,
		        Hor_Matrix *z,
		        void (*h_func)(Hor_Matrix *x, Hor_Matrix *h,
				       Hor_Matrix *grad_hT, void *data),
		        Hor_Matrix *R, double  gate_thres, void *update_data );

Hor_Matrix *hor_kf_state_vector ( Hor_Assoc_Label label );
Hor_Matrix *hor_kf_covariance   ( Hor_Assoc_Label label );
void        hor_kf_print_state  ( Hor_Assoc_Label label );

#endif /* _HORATIO_LIST_ */
