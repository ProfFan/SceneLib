/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University.

		  Based on an algorithm by Han Wang and Ian Reid. */

#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/math.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#define MAX_CANDIDATES 20

#define SQR(X)   ((X)*(X))

typedef struct {
    int             n;
    Hor_Trajectory *match;
    Hor_Corner     *corner;
} Cand_List;

static Cand_List       candidates[MAX_CANDIDATES];
static float Q[4][4] = {{0.0F, 0.0F, 0.0F, 0.0F},
			{0.0F, 0.0F, 0.0F, 0.0F},
			{0.0F, 0.0F, 1.0F, 0.0F},
			{0.0F, 0.0F, 0.0F, 1.0F}};
static float R[2][2] = {{0.5F, 0.0F},
			{0.0F, 0.5F}};

static int get_candidates(float      X[4], 
			  float      P[4][4],
			  float      R[2][2],
			  int        obs_err,
			  Hor_List   corners,
			  Cand_List *candidates,
			  Hor_Trajectory **curr_match);
static Cand_List *correlation_match(Hor_Image *patch, int n,
				    Cand_List *candidates,
				    float *corr_coeff,
				    float  corr_thresh);
static void predict_state(float X_Predict[4], float P_Predict[4][4],
			  float X_Old[4],     float P_Old[4][4], float Q[4][4],
			  float u_c, float u_r,
			  int t);
static void update(float X_New[4], float P_New[4][4],
		   float X_Predict[4], float P_Predict[4][4], float R[2][2],
		   float x0, float y0,
		   int frame_interval);


/*******************
*   void @hor_bd_cm_traj_state_free ( void *state )
*
*   Brain-dead corner matcher trajectory state free function.
********************/
void hor_bd_cm_traj_state_free ( void *state )
{
   hor_free_sub_image ( ((Hor_BD_CM_Traj_State *) state)->patch );
   hor_free ( state );
}

static Hor_Bool bd_cm_test_traj ( Hor_Trajectory *traj, void *data )
{
   if ( traj->type != HOR_CM_BRAIN_DEAD || traj->status != HOR_UNMATCHED )
      return HOR_TRUE;

   return HOR_FALSE;
}

/*******************
*   void @hor_bd_match_corners ( Hor_Trajectory_Map *traj_map,
*                               Hor_Corner_Map     *cmap_new,
*                               int   match_window,
*                               float corr_thresh)
********************/
void hor_bd_match_corners ( Hor_Trajectory_Map *traj_map,
			    Hor_Corner_Map     *cmap_new,
			    int   match_window,
			    float corr_thresh)
{
    int             i, j;
    Hor_List        ml, cl;
    Hor_Traj_Point *point;
    Hor_Trajectory *traj;
    Hor_Corner     *corner;
    int             n_cand;
    Cand_List      *cand;
    float           X_Predict[4], P_Predict[4][4];
    float           corr_coeff;

    int             frame_interval = 1;
    float           u_c = 0.0F,
                    u_r = 0.0F;

    Hor_Trajectory      **curr_match;
    Hor_BD_CM_Traj_State *state;

    curr_match = hor_malloc_ntype(Hor_Trajectory *,cmap_new->ncorners);
    for (i=0; i<cmap_new->ncorners; i++) curr_match[i] = NULL;

    /* reset status of every trajectory to HOR_UNMATCHED */
    for (ml=traj_map->traj_list; ml!=NULL; ml=ml->next)
       ((Hor_Trajectory *) ml->contents)->status = HOR_UNMATCHED;

    for (ml=traj_map->traj_list; ml!=NULL; ml=ml->next) {

        traj = (Hor_Trajectory *) ml->contents;
	if ( traj->type != HOR_CM_BRAIN_DEAD ) continue;

	/* only match trajectories with more than one corner at this stage */
	if ( traj->length == 1 ) continue;

	state = (Hor_BD_CM_Traj_State *) traj->state;
	
	predict_state(X_Predict, P_Predict, state->state, state->covar, Q,
		      u_c, u_r, frame_interval);

	n_cand = get_candidates(X_Predict, P_Predict, R, match_window, 
				cmap_new->corner_list, candidates, curr_match);

	cand = NULL;
	if (n_cand)
	    cand = correlation_match(&state->patch->image, n_cand, candidates, 
				     &corr_coeff, corr_thresh);

        /* Have we found a match, if so update the trajectory */
	if (cand != NULL) {

	   if (cand->match != NULL) cand->match->status = HOR_UNMATCHED;

	   point = hor_malloc_type(Hor_Traj_Point);
	   point->cf = cand->corner->cf;
	   point->rf = cand->corner->rf;
	   hor_add_trajectory_element ( traj, (void *) point );
	   hor_convert_image_data ( &cand->corner->patch->image,
				    &state->patch->image );
	   traj->status = cand->corner->status = HOR_PROVISIONAL;

	   state->n = cand->n;
	   for (j=HOR_BD_CM_MAX_HISTORY-1; j>=0; j--) {
	      state->x[j] = state->x[j-1];
	      state->y[j] = state->y[j-1];
	   }
	   state->x[0] = cand->corner->cf;
	   state->y[0] = cand->corner->rf;
	   state->age++;
	   state->corr = corr_coeff;
	   state->innovation  = SQR(X_Predict[0] - state->x[0]) +
	                        SQR(X_Predict[1] - state->y[0]);

	   /* update Kalman filter equations for this match */
	   update(state->state, state->covar, X_Predict, P_Predict, R, 
		  state->x[0], state->y[0], 1);

	   state->speed = sqrt(SQR(state->state[2]) + SQR(state->state[3]));

	   curr_match[cand->n] = traj;
	}
    }

    for (ml=traj_map->traj_list; ml!=NULL; ml=ml->next) {

        traj = (Hor_Trajectory *) ml->contents;
	if ( traj->type != HOR_CM_BRAIN_DEAD ) continue;

	/* only match new trajectories here */
	if ( traj->length > 1 ) continue;

	state = (Hor_BD_CM_Traj_State *) traj->state;

	X_Predict[0] = state->x[0];
	X_Predict[1] = state->y[0];
	P_Predict[0][0] = P_Predict[1][1] = 0.0F;

	n_cand = get_candidates(X_Predict, P_Predict, R, match_window,
				cmap_new->corner_list, candidates, curr_match);

	cand = NULL;
	if (n_cand) 
	    cand = correlation_match(&state->patch->image, n_cand, candidates,
				     &corr_coeff, corr_thresh);
	
	if (cand != NULL) {

	   if (cand->match != NULL) cand->match->status = HOR_UNMATCHED;

	   point = hor_malloc_type(Hor_Traj_Point);
	   point->cf = cand->corner->cf;
	   point->rf = cand->corner->rf;
	   hor_add_trajectory_element ( traj, (void *) point );
	   hor_convert_image_data ( &cand->corner->patch->image,
				    &state->patch->image );
	   traj->status = cand->corner->status = HOR_PROVISIONAL;

	   state->n = cand->n;
	   state->x[1] = X_Predict[0];
	   state->y[1] = X_Predict[1];
	   state->x[0] = cand->corner->cf;
	   state->y[0] = cand->corner->rf;
	   state->corr = corr_coeff;
	   state->innovation = 0.0F;
	   state->age = 1;

	   /* initialize Kalman filter equations */
	   state->state[0] = state->x[0];
	   state->state[1] = state->y[0];
	   state->state[2] = (state->x[0] - state->x[1])/frame_interval - u_c;
	   state->state[3] = (state->y[0] - state->y[1])/frame_interval - u_r;

	   state->covar[0][0] = 5.0F; state->covar[0][1] = 0.0F;  
	   state->covar[0][2] = 0.0F; state->covar[0][3] = 0.0F;  

	   state->covar[1][0] = 0.0F; state->covar[1][1] = 5.0F;  
	   state->covar[1][2] = 0.0F; state->covar[1][3] = 0.0F;  

	   state->covar[2][0] = 0.0F; state->covar[2][1] = 0.0F;  
	   state->covar[2][2] = 5.0F; state->covar[2][3] = 0.0F;  

	   state->covar[3][0] = 0.0F; state->covar[3][1] = 0.0F;  
	   state->covar[3][2] = 0.0F; state->covar[3][3] = 5.0F;  

	   state->speed = sqrt(SQR(state->state[2]) + SQR(state->state[3]));

	   curr_match[cand->n] = traj;
	}
    }

    /* discard old corner trajectories */
    hor_delete_old_trajectories ( traj_map, bd_cm_test_traj, NULL );

    /* update all match status's and initialise new corner trajectories */
    for (cl=cmap_new->corner_list, i=0; cl!=NULL; cl=cl->next, i++) {

       corner = cl->contents;
       if (curr_match[i] != NULL)
	  corner->status = curr_match[i]->status = HOR_MATCHED;
       else { /* unmatched corner: create a new trajectory for it */

	  point = hor_malloc_type(Hor_Traj_Point);
	  point->cf = corner->cf;
	  point->rf = corner->rf;
	  state = hor_malloc_type(Hor_BD_CM_Traj_State);
	  state->x[0] = corner->cf;
	  state->y[0] = corner->rf;
	  state->patch = hor_copy_sub_image ( corner->patch );
	  hor_add_new_trajectory ( traj_map, HOR_CM_BRAIN_DEAD,
				   (void *) point, 100000,
				   (void *) state, hor_free_func,
				   hor_bd_cm_traj_state_free,
				   hor_cm_traj_display );
       }
    }
}

/*************
* static int @get_candidates(float       X[4], 
*                            float        P[4][4],
*                            float        R[2][2],
*                            int          obs_err,
*                            Hor_List     corners,
*                            Cand_List   *candidates,
*                            Hor_Trajectory **curr_match)
*
* Find all potential matches for the corner by listing all corners
* within a certain distance of the position given X.
**************/
static int get_candidates(float        X[4], 
			  float        P[4][4],
			  float        R[2][2],
			  int          obs_err,
			  Hor_List     corners,
			  Cand_List   *candidates,
			  Hor_Trajectory **curr_match)
{
    int xmin, xmax, ymin, ymax;
    int p00, p11;
    int i, r, c, n=0;
    int dist, mindist1=SQR(512), mindist2=SQR(512);
    Hor_List    cl;
    Hor_Corner *corner;

    p00 = (int)(2.0 * sqrt(P[0][0] + R[0][0]) + 0.5);
    p11 = (int)(2.0 * sqrt(P[1][1] + R[1][1]) + 0.5);

    if (P[0][0] < obs_err) {
        xmin = (int)(X[0] - obs_err);
	xmax = (int)(X[0] + obs_err);
    } else {
        xmin = (int)(X[0] - p00);
        xmax = (int)(X[0] + p00);
    }

    if (P[1][1] < obs_err) {
        ymin = (int)(X[1] - obs_err);
        ymax = (int)(X[1] + obs_err);
    } else {
        ymin = (int)(X[1] - p11);
        ymax = (int)(X[1] + p11);
    }

    for (cl=corners, i=0; cl!=NULL; cl=cl->next, i++) {
	corner = cl->contents;
	if (corner->status == HOR_MATCHED) continue;

	c = (int) corner->cf;
	r = (int) corner->rf;

	if (c < xmin || c > xmax || r < ymin || r > ymax) continue;

	dist = SQR(c - X[0]) + SQR(r - X[1]);
	if (dist < mindist1) {
	    if (n<2) n++;
	    candidates[1] = candidates[0];
	    candidates[0].n = i;
	    candidates[0].match = curr_match[i];
	    candidates[0].corner = corner;
	    mindist2 = mindist1;
	    mindist1 = dist;
	} else if (dist < mindist2) {
	    if (n<2) n++;
	    candidates[1].n = i;
	    candidates[1].match = curr_match[i];
	    candidates[1].corner = corner;
	    mindist2 = dist;
        }
    }
    return n;
}


/*******************
*   static Cand_List *@correlation_match(Hor_Image *patch,
*                                        int        n,
*                                        Cand_List *candidates,
*                                        float *corr_coeff,
*                                        float  corr_thresh)
*
*   Find the best candidate match for a corner based on a correlation 
*   measure.
********************/
static Cand_List *correlation_match(Hor_Image *patch,
				    int        n,
				    Cand_List *candidates,
				    float *corr_coeff,
				    float  corr_thresh)
{
   float corr, max_corr=0.0F;
   int i, best=-1;
   float s_lf=0.0F, s_llf=0.0F, s_rf=0.0F, s_rrf=0.0F, s_lrf=0.0F, fm, fz, fn;
   int size = patch->width*patch->height;
   float corr_window_xy = (float) size;
   Hor_Image *new_patch;

   switch ( patch->type )
   {
      case HOR_U_CHAR:
      {
	 u_char *tmp_l, *pend;
	 int s_l=0, s_ll=0;

	 for (tmp_l = patch->array.uc[0], pend = tmp_l+size; tmp_l != pend;
	      tmp_l++) {
	    s_l  += (int)*tmp_l;
	    s_ll += (int)*tmp_l * (int)*tmp_l;
	 }

	 s_lf  = (float) s_l;
	 s_llf = (float) s_ll;
      }
      break;

      case HOR_INT:
      {
	 int *tmp_l, *pend;
	 int s_l=0, s_ll=0;

	 s_l = s_ll = 0;
	 for (tmp_l = patch->array.i[0], pend = tmp_l+size; tmp_l != pend;
	      tmp_l++) {
	    s_l  += *tmp_l;
	    s_ll += *tmp_l * *tmp_l;
	 }

	 s_lf  = (float) s_l;
	 s_llf = (float) s_ll;
      }
      break;

      case HOR_FLOAT:
      {
	 float *tmp_l, *pend;

	 s_lf = s_llf = 0.0F;
	 for (tmp_l = patch->array.f[0], pend = tmp_l+size; tmp_l != pend;
	      tmp_l++) {
	    s_lf  += *tmp_l;
	    s_llf += *tmp_l * *tmp_l;
	 }
      }
      break;

      default:
      hor_error ( "illegal corner image patch type (correlation_match)",
		  HOR_FATAL );
      break;
   }

   fm = s_llf - s_lf*s_lf/corr_window_xy;

   for (i=0; i<n; i++) {
      new_patch = &candidates[i].corner->patch->image;
      if (new_patch->type != patch->type ||
	  new_patch->width != patch->width ||
	  new_patch->height != patch->height)
	 hor_error("inconsistent corner image patch types (correlation_match)",
		   HOR_FATAL);

      switch ( patch->type )
      {
	 case HOR_U_CHAR:
	 {
	    u_char *tmp_l, *tmp_r, *pend;
	    int s_r=0, s_rr=0, s_lr=0;

	    for (tmp_l = patch->array.uc[0], pend = tmp_l+size,
		 tmp_r = new_patch->array.uc[0]; tmp_l != pend;
		 tmp_l++, tmp_r++) {
	       s_r  += (int)*tmp_r;
	       s_rr += (int)*tmp_r * (int)*tmp_r;
	       s_lr += (int)*tmp_l * (int)*tmp_r;
	    }

	    s_rf  = (float) s_r;
	    s_rrf = (float) s_rr;
	    s_lrf = (float) s_lr;
	 }
	 break;

	 case HOR_INT:
	 {
	    int *tmp_l, *tmp_r, *pend;
	    int s_r=0, s_rr=0, s_lr=0;

	    for (tmp_l = patch->array.i[0], pend = tmp_l+size,
		 tmp_r = new_patch->array.i[0]; tmp_l != pend;
		 tmp_l++, tmp_r++) {
	       s_r  += *tmp_r;
	       s_rr += *tmp_r * *tmp_r;
	       s_lr += *tmp_l * *tmp_r;
	    }

	    s_rf  = (float) s_r;
	    s_rrf = (float) s_rr;
	    s_lrf = (float) s_lr;
	 }
	 break;

	 case HOR_FLOAT:
	 {
	    float *tmp_l, *tmp_r, *pend;

	    s_rf = s_rrf = s_lrf = 0.0F;
	    for (tmp_l = patch->array.f[0], pend = tmp_l+size,
		 tmp_r = new_patch->array.f[0]; tmp_l != pend;
		 tmp_l++, tmp_r++) {
	       s_rf  += *tmp_r;
	       s_rrf += *tmp_r * *tmp_r;
	       s_lrf += *tmp_l * *tmp_r;
	    }
	 }
	 break;

	 default:
	 hor_error ( "illegal corner image patch type (correlation_match)",
		     HOR_FATAL );
	 break;
      }

      fz = s_lrf - s_lf*s_rf/corr_window_xy;
      fn = s_rrf - s_rf*s_rf/corr_window_xy;
      corr = (fz==0.0F && fm==0.0F && fn==0.0F) ? 1.0F : (fz*fz)/(fm*fn);
      if (corr > max_corr) {
	 if (candidates[i].match != NULL) {
	    if (corr > ((Hor_BD_CM_Traj_State *)
			candidates[i].match->state)->corr) {
	       *corr_coeff = corr;
	       max_corr = i;
	       best = i;
	    } else
	       continue;
	 } else if (corr > corr_thresh) {
	    *corr_coeff = corr;
	    max_corr = corr;
	    best = i;
	 }
      }
   }
   return (best==-1) ? NULL : &(candidates[best]);
}



/*******************
*   void @predict_state(float X_Predict[4], float P_Predict[4][4],
*                      float X_Old[4],     float P_Old[4][4], float Q[4][4],
*                      float u_c, float u_r,
*                      int t)
*
*   Predict new state of tracked corner using Kalman filter prediction
*   equations.
********************/
void predict_state(float X_Predict[4], float P_Predict[4][4],
		   float X_Old[4],     float P_Old[4][4], float Q[4][4],
		   float u_c, float u_r,
		   int t)
{
#define p00 P_Old[0][0]
#define p01 P_Old[0][1]
#define p02 P_Old[0][2]
#define p03 P_Old[0][3]
#define p10 P_Old[1][0]
#define p11 P_Old[1][1]
#define p12 P_Old[1][2]
#define p13 P_Old[1][3]
#define p20 P_Old[2][0]
#define p21 P_Old[2][1]
#define p22 P_Old[2][2]
#define p23 P_Old[2][3]
#define p30 P_Old[3][0]
#define p31 P_Old[3][1]
#define p32 P_Old[3][2]
#define p33 P_Old[3][3]

    X_Predict[0] = X_Old[0] + t*X_Old[2] + t*u_c;
    X_Predict[1] = X_Old[1] + t*X_Old[3] + t*u_r;
    X_Predict[2] = X_Old[2];
    X_Predict[3] = X_Old[3];

    P_Predict[0][0] = p00 + t*(p20+p02+p22*t);
    P_Predict[0][1] = p01 + t*(p21+p03+p23*t);
    P_Predict[0][2] = p02 + t*p22;
    P_Predict[0][3] = p03 + t*p23;

    P_Predict[1][0] = p10 + t*(p30+p12+p32*t);
    P_Predict[1][1] = p11 + t*(p31+p13+p33*t);
    P_Predict[1][2] = p12 + t*p32;
    P_Predict[1][3] = p13 + t*p33;

    P_Predict[2][0] = p20 + t*p22;
    P_Predict[2][1] = p21 + t*p23;
    P_Predict[2][2] = p22 + Q[2][2];
    P_Predict[2][3] = p23;

    P_Predict[3][0] = p30 + t*p32;
    P_Predict[3][1] = p31 + t*p33;
    P_Predict[3][2] = p32;
    P_Predict[3][3] = p33 + Q[3][3];

#undef p00
#undef p01
#undef p02
#undef p03
#undef p10
#undef p11
#undef p12
#undef p13
#undef p20
#undef p21
#undef p22
#undef p23
#undef p30
#undef p31
#undef p32
#undef p33
}

/*******************
*   void @update(float X_New[4], float P_New[4][4],
*               float X_Predict[4], float P_Predict[4][4], float R[2][2],
*               float x0, float y0,
*               int frame_interval)
*
*   Update state of tracked corner using Kalman filter update equations.
********************/
static void update(float X_New[4], float P_New[4][4],
		   float X_Predict[4], float P_Predict[4][4], float R[2][2],
		   float x0, float y0,
		   int frame_interval)
{
#define p00 P_Predict[0][0]
#define p01 P_Predict[0][1]
#define p02 P_Predict[0][2]
#define p03 P_Predict[0][3]
#define p10 P_Predict[1][0]
#define p11 P_Predict[1][1]
#define p12 P_Predict[1][2]
#define p13 P_Predict[1][3]
#define p20 P_Predict[2][0]
#define p21 P_Predict[2][1]
#define p22 P_Predict[2][2]
#define p23 P_Predict[2][3]
#define p30 P_Predict[3][0]
#define p31 P_Predict[3][1]
#define p32 P_Predict[3][2]
#define p33 P_Predict[3][3]
#define r00 R[0][0]
#define r11 R[1][1]

    float k00, k01;
    float k10, k11;
    float k20, k21;
    float k30, k31;
    float denominator, a, b;
    /*float s00, s11, t1, t2;*/

    /* filter gain W = P_Predict . Transpose[H] . Inverse[S],
       where S = R + H . P_predict . Transpose[H]             */
    denominator = -p01*p10 + (p00+r00)*(p11+r11);
    k00 = (-p01*p10+p00*(p11+r11))/denominator;
    k01 = (-p01*p00+p01*(p00+r00))/denominator; /* Han's old: (p01*r00)/denominator */
    k10 = (-p10*p11+p10*(p11+r11))/denominator; /* Han's old: (p10*r11)/denominator */
    k11 = (-p01*p10+p11*(p00+r00))/denominator;
    k20 = (-p10*p21+p20*(p11+r11))/denominator;
    k21 = (-p01*p20+p21*(p00+r00))/denominator;
    k30 = (-p10*p31+p30*(p11+r11))/denominator;
    k31 = (-p01*p30+p31*(p00+r00))/denominator;

    /* estimator X = X_Predict + W . Innovation */
    a = x0 - X_Predict[0];
    b = y0 - X_Predict[1];
    X_New[0] = X_Predict[0] + k00*a + k01*b;
    X_New[1] = X_Predict[1] + k10*a + k11*b;
    X_New[2] = X_Predict[2] + k20*a + k21*b;
    X_New[3] = X_Predict[3] + k30*a + k31*b;

    /* covariance matrix P = P_Predict - W . S . Transpose[W] */
    /*
    s00 = p00+r00;
    s11 = p11+r11;
    t1  = k01*p10 + k00*s00;
    t2  = k00*p01 + k01*s11;
    P_New[0][0] = p00 - k00*t1 + k01*t2;
    P_New[0][1] = p01 - k10*t1 + k11*t2;
    P_New[0][2] = p02 - k20*t1 + k21*t2;
    P_New[0][3] = p03 - k30*t1 + k31*t2;

    t1  = k11*p10 + k10*s00;
    t2  = k10*p01 + k11*s11;
    P_New[1][0] = p10 - k00*t1 + k01*t2;
    P_New[1][1] = p11 - k10*t1 + k11*t2;
    P_New[1][2] = p12 - k20*t1 + k21*t2;
    P_New[1][3] = p13 - k30*t1 + k31*t2;

    t1  = k21*p10 + k20*s00;
    t2  = k20*p01 + k21*s11;
    P_New[2][0] = p20 - k00*t1 + k01*t2;
    P_New[2][1] = p21 - k10*t1 + k11*t2;
    P_New[2][2] = p22 - k20*t1 + k21*t2;
    P_New[2][3] = p23 - k30*t1 + k31*t2;

    t1  = k31*p10 + k30*s00;
    t2  = k30*p01 + k31*s11;
    P_New[3][0] = p30 - k00*t1 + k01*t2;
    P_New[3][1] = p31 - k10*t1 + k11*t2;
    P_New[3][2] = p32 - k20*t1 + k21*t2;
    P_New[3][3] = p33 - k30*t1 + k31*t2;
    */

    /* Han's old stuff (in error?) */
    P_New[0][0] = p00 - k00*p00 - k01*p10;
    P_New[0][1] = p01 - k00*p01 - k01*p11;
    P_New[0][2] = p02 - k00*p02 - k01*p12;
    P_New[0][3] = p03 - k00*p03 - k01*p13;

    P_New[1][0] = p10 - k10*p00 - k11*p10;
    P_New[1][1] = p11 - k10*p01 - k11*p11;
    P_New[1][2] = p12 - k10*p02 - k11*p12;
    P_New[1][3] = p13 - k10*p03 - k11*p13;

    P_New[2][0] = p20 - k20*p00 - k21*p10;
    P_New[2][1] = p21 - k20*p01 - k21*p11;
    P_New[2][2] = p22 - k20*p02 - k21*p12;
    P_New[2][3] = p23 - k20*p03 - k21*p13;

    P_New[3][0] = p30 - k30*p00 - k31*p10;
    P_New[3][1] = p31 - k30*p01 - k31*p11;
    P_New[3][2] = p32 - k30*p02 - k31*p12;
    P_New[3][3] = p33 - k30*p03 - k31*p13;

    return;

#undef p00
#undef p01
#undef p02
#undef p03
#undef p10
#undef p11
#undef p12
#undef p13
#undef p20
#undef p21
#undef p22
#undef p23
#undef p30
#undef p31
#undef p32
#undef p33
}

