/* ------------------------------------------------------------------   */
/*                                                                      */
/* Module which contains the procedures to fit line segments to a list  */
/* of strings of edges using an orthogonal regression                   */
/*  This is a new recursive version                                     */
/*                                                                      */
/* David Djian. November 1994                                           */
/*                                                                      */
/* new_or.c                                                             */
/*                                                                      */
/* ------------------------------------------------------------------   */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/math.h"
#include "horatio/image.h"
#include "horatio/graphics.h"
#include "horatio/improc.h"
#include "horatio/process.h"
#include "horatio/tool.h"

#define LABEL 75894
#define FIT_2D 2


typedef struct
{
   /* unit direction vector of the line */
   double v_direct[2];
   /* coordinates of the centroid (belongs to the line) */
   double centroid[2];
   /* angle of the eigenvector (in radians) */
   float angle; 
   /* coordinates of the first and last point in the fit;  */
   /* the orthogonal projection of these points on the line */
   /* determine the end points of the segment */
   float first_x, first_y, last_x, last_y; 

} OR_fit_param;


/* structure containing the current parameters used by the regression */
typedef struct
{
   /* parameters from the user */
   int no_init_points; /* number of points necessary to init a fit */
   float sigma; /* standard deviation of edge points */
   float thres; /* threshold for fit */

   Hor_Line_Segment_Map *map; /* line map which is being built */
   Hor_Bool global_fit_init; /* initialisation flag */

   /* parameters of the previous good fit */
   Hor_Bool previous_fit_ok; /* flag tells if previous fit worked */
   OR_fit_param previous_fit; /* results of the previous fit */

} OR_current_data;


/* Returns the parameter t associated with point (x, y) projected on the
*  2-D line defined by a direction vector v_direct and a point p
*/
static double get_t_line ( double *v_direct, double *p, double x, double y )
{
   double t;

   t = ( v_direct[0] * ( x - p[0] ) + v_direct[1] * ( y - p[1] ) ) 
     / ( v_direct[0] * v_direct[0] + v_direct[1] * v_direct[1] );

   return t;
}

/* Sorts m vectors of dimension n according to the corresponding
*  value of an m-vector of doubles
*/
static void sort_t ( double *d, double **v, int n, int m )
{
   int k,j,i;
   double p;

   for (i=0;i<n-1;i++) {
      p=d[k=i];
      for (j=i+1;j<n;j++)
	 if (d[j] >= p) p=d[k=j];
      if (k != i) {
	 d[k]=d[i];
	 d[i]=p;
	 for (j=0;j<m;j++) {
	    p=v[j][i];
	    v[j][i]=v[j][k];
	    v[j][k]=p;
	 }
      }
   }
}

static void init_seg_fit ( OR_current_data *p, float x1, float y1, 
			                       float x2, float y2 )
{
    p->previous_fit.first_x   = x1;
    p->previous_fit.first_y   = y1;
    p->previous_fit.last_x   = x2;
    p->previous_fit.last_y   = y2;
}

static void init_edge_fit ( OR_current_data *p, float x, float y )
{
    p->previous_fit.first_x   = x;
    p->previous_fit.first_y   = y;
}

/* Determines the end points of the fitted line segment in the general
*  case when they are not ordered 
*/
static void update_seg_fit ( OR_current_data *p, 
			     float x1, float y1,
			     float x2, float y2,
			     double *v_direct, double *centroid )
{
    int i;
    double t[4];
    Hor_Matrix *M = hor_mat_alloc ( 2, 4 );

    for ( i=0; i<2; i++ )
    {
      p->previous_fit.v_direct[i] = v_direct[i];
      p->previous_fit.centroid[i] = centroid[i];
    }

    t[0] = get_t_line ( p->previous_fit.v_direct, 
		        p->previous_fit.centroid,
		        p->previous_fit.first_x, p->previous_fit.first_y );
    t[1] = get_t_line ( p->previous_fit.v_direct, 
		        p->previous_fit.centroid,
		        p->previous_fit.last_x, p->previous_fit.last_y );
    t[2] = get_t_line ( p->previous_fit.v_direct, 
		        p->previous_fit.centroid,
		        x1, y1 );
    t[3] = get_t_line ( p->previous_fit.v_direct, 
		        p->previous_fit.centroid,
		        x2, y2 );
    M->m[0][0] = p->previous_fit.first_x; 
    M->m[1][0] = p->previous_fit.first_y; 
    M->m[0][1] = p->previous_fit.last_x; 
    M->m[1][1] = p->previous_fit.last_y; 
    M->m[0][2] = x1; M->m[1][2] = y1; 
    M->m[0][3] = x2; M->m[1][3] = y2; 
    sort_t ( t, M->m, 4, 2 );
    p->previous_fit.first_x   = M->m[0][0];
    p->previous_fit.first_y   = M->m[1][0];
    p->previous_fit.last_x   = M->m[0][3];
    p->previous_fit.last_y   = M->m[1][3];
}


static void update_edge_fit ( OR_current_data *p, 
			      float x, float y,
			      double *v_direct, double *centroid )
{
    int i;

    for ( i=0; i<2; i++ )
    {
      p->previous_fit.v_direct[i] = v_direct[i];
      p->previous_fit.centroid[i] = centroid[i];
    }
    p->previous_fit.last_x   = x;
    p->previous_fit.last_y   = y;
}


static void compute_projection ( OR_fit_param *fp, 
				 double px, double py, double *x, double *y )
{
   double t; /* parametric representation of the line */

   t = get_t_line ( fp->v_direct, fp->centroid, px, py );
   *x = fp->v_direct[0] * t + fp->centroid[0];
   *y = fp->v_direct[1] * t + fp->centroid[1];
}

/* Procedure which computes the end points of the line segment and allocates */
/* the space for the horatio structure which contains the line segment */
static void compute_end_points ( OR_current_data *p )
{
   double x1, y1, x2, y2;

   compute_projection ( &(p->previous_fit), 
		       p->previous_fit.first_x, 
		       p->previous_fit.first_y, &x1, &y1 );
   compute_projection ( &(p->previous_fit), 
		       p->previous_fit.last_x, 
		       p->previous_fit.last_y, &x2, &y2 );

   p->previous_fit.angle = 180.0/M_PI*atan2(y2 - y1, x2 - x1);
   if ((p->previous_fit.angle > 90.0) || (p->previous_fit.angle < -90.0))
      p->previous_fit.angle = 180.0/M_PI*atan2(y1 - y2, x1 - x2);
   hor_add_line_segment ( p->map, x1, y1, x2, y2, p->previous_fit.angle,
			          1, HOR_FALSE, HOR_FALSE,
			          HOR_NO_ATTRIB, NULL, NULL,
			          hor_display_line_segment );
}

/* orthogonal regression applied to a string/list of 2-D points */
static void orthog_regression ( Hor_DList list, OR_current_data *params )
{
   int res;
   double v_direct[2], centroid[2];
   Hor_Edgel *edge = (Hor_Edgel *) list->contents;
      
   if ( params->global_fit_init == HOR_FALSE )
   {
      hor_oreg_line_init ( LABEL, FIT_2D );
      params->global_fit_init = HOR_TRUE; /* global fit initialised */
      init_edge_fit ( params, edge->cf, edge->rf );
      params->previous_fit_ok = HOR_FALSE; /* NO previous fit yet */
   }

   /* add point to global fit */
   hor_oreg_line_data_v ( LABEL, edge->cf, edge->rf );

   /* check that there are enough points for the global fit */
   /* otherwise do nothing; points will be added next iteration */
   res = hor_oreg_line_solve ( LABEL, params->sigma, params->thres, 
			       params->no_init_points, 
			       v_direct, centroid );

   switch (res)
   {
   case 0:
      {
	 params->previous_fit_ok = HOR_TRUE;
	 update_edge_fit ( params, edge->cf, edge->rf, 
			  v_direct, centroid );
      }
      break;
    case 1:
      /* discontinuity/corner detected here */
      {
	 if ( params->previous_fit_ok == HOR_TRUE )
	 {
	    compute_end_points ( params );
	    params->previous_fit_ok = HOR_FALSE;
	    /* global fit NOT initialised */
	    params->global_fit_init = HOR_FALSE; 
	 }
      }
      break;
   }
}


static Hor_DList or_fit_dlist ( Hor_DList list, 
			       Hor_DList stop_node, 
			       OR_current_data * params )
{
   Hor_DList result;

   if ( list == NULL || list == stop_node )
   {
      /* for the last segment */
      if ( params->previous_fit_ok == HOR_TRUE )
	 compute_end_points ( params );
      return list;
   }

   orthog_regression ( list, params );
   result = or_fit_dlist ( list->next, stop_node, params );
   return result;
}


/* line fitting to a string of edges */
void string_line_fit ( Hor_Estring *string, void *params )
{
   Hor_DList list;
   OR_current_data *fit_params = (OR_current_data *) params;
   fit_params->global_fit_init = HOR_FALSE; /* global fit NOT initialised */
   
   /* call orthogonal regression for each edge of the string */
   list = string->leftmost;

   if ( list != NULL ) 
      or_fit_dlist ( list->next, list, fit_params );
}

/*******************
*   Hor_Line_Segment_Map *@hor_find_lines (
*      Hor_Edge_Map *edge_map, (edge map to fit line segments to)
*      int   no_init_points,   (minimum number of points used to fit a line)
*      float sigma,            (estimate of edge position error std. deviation)
*      float thres             (chi^2 confidence level, e.g. 0.01) )
*
*   Fits line segments to an edge map using orthogonal regression.
********************/
Hor_Line_Segment_Map *hor_find_lines ( Hor_Edge_Map *edge_map, 
				       int no_init_points,
				       float sigma, float thres )
{
   OR_current_data params;

   params.no_init_points = no_init_points;
   params.sigma = sigma;
   params.thres = thres;
   params.map = hor_alloc_line_segment_map ( HOR_NO_ATTRIB, NULL, NULL ); 
   params.map->c0 = edge_map->c0;
   params.map->r0 = edge_map->r0;
   params.map->width  = edge_map->width;
   params.map->height = edge_map->height;

   /* for each string in the list of strings */
   hor_list_action ( edge_map->string_list,
		     (void (*)(void*,void*)) string_line_fit, &params );

   return params.map;
}


/* orthogonal regression line fitting to the endpoints of segments */
static void orthog_regression2 ( Hor_Line_Segment *segment, 
				OR_current_data *params )
{
   double v_direct[2], centroid[2];
   int res;
      
   if ( params->global_fit_init == HOR_FALSE )
   {
      /*init_stat_sum ( &(params->global_fit) );*/
      /* params->no_points = 0; */
      hor_oreg_line_init ( LABEL, FIT_2D );
      params->global_fit_init = HOR_TRUE; /* global fit initialised */
      /* Init with 0 because  */
      init_seg_fit ( params, segment->x1, segment->y1,
		             segment->x2, segment->y2 );
      params->previous_fit_ok = HOR_FALSE; /* NO previous fit yet */
   }

   /* add both segment end points to global fit */
   hor_oreg_line_data_v ( LABEL, segment->x1, segment->y1 );
   hor_oreg_line_data_v ( LABEL, segment->x2, segment->y2 );

   /* check that there are enough points for the global fit */
   /* otherwise do nothing; points will be added next iteration */
   res = hor_oreg_line_solve ( LABEL, params->sigma, params->thres, 
			       params->no_init_points, 
			       v_direct, centroid );

   switch (res)
   {
      case 0:
      {
	 params->previous_fit_ok = HOR_TRUE;
	 update_seg_fit ( params, segment->x1, segment->y1,
			          segment->x2, segment->y2,
			          v_direct, centroid );
      }
      break;

      case 1:
      /* discontinuity/corner detected here */
      {
	 if ( params->previous_fit_ok == HOR_TRUE )
	 {
	    compute_end_points ( params );
	    /* global fit NOT initialised */
	    params->global_fit_init = HOR_FALSE; 
	 }
      }
      break;
   }
}

/*******************
*   Hor_Line_Segment_Map *@hor_find_lines_rec ( Hor_Line_Segment_Map *line_map,
*      int   no_init_points,   (minimum number of points used to fit a line)
*      float sigma,            (estimate of edge position error std. deviation)
*      float thres             (chi^2 confidence level, e.g. 0.01) )
*
*   Fits line segments to an edge map using orthogonal regression (recursive).
********************/
Hor_Line_Segment_Map *hor_find_lines_rec ( Hor_Line_Segment_Map *line_map,
					   int no_init_points,
					   float sigma, float thres )
{
   OR_current_data params;

   params.no_init_points = no_init_points;
   params.sigma = sigma;
   params.thres = thres;
   params.map = hor_alloc_line_segment_map ( HOR_NO_ATTRIB, NULL, NULL ); 
   params.global_fit_init = HOR_FALSE; /* global fit NOT initialised */

   /* call orthogonal regression for each segment in the list */
   hor_list_action ( line_map->line_list,
		     (void (*)(void*,void*)) orthog_regression2, &params );

   /* for the last segment */
   if ( params.previous_fit_ok == HOR_TRUE )
	 compute_end_points ( &params );

   return params.map;
}
