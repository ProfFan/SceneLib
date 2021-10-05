/* Canny operator originally written by David Murray 1990
   HORATIO additions Phil McLauchlan 11-1992

   Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

/*******************
*   static void @image_derivative ( Hor_Sub_Image *sub_image,
*                                  Hor_Sub_Image *deriv_c,
*                                  Hor_Sub_Image *deriv_r,
*                                  Hor_Sub_Image *deriv_sq )
*
*   Takes input sub-image (sub_image) and calculates x-derivative,
*   y-derivative and sum-of-squares of x & y derivatives, writing the results
*   into deriv_c, deriv_r and deriv_sq.
********************/
static void image_derivative ( Hor_Sub_Image *sub_image,
			       Hor_Sub_Image *deriv_c,
			       Hor_Sub_Image *deriv_r,
			       Hor_Sub_Image *deriv_sq )
{
   Hor_Image  *image     = &(sub_image->image);
   Hor_Image  *dc_image  = &(deriv_c->image);
   Hor_Image  *dr_image  = &(deriv_r->image);
   Hor_Image  *dsq_image = &(deriv_sq->image);
   float **imarr, **dcarr, **drarr, **dsqarr;
   float  *pimp, *nimp, *imcol, *imend, *dcp, *drp, *dsqp, *dsqend;
   int     width, height, big_height, big_width;
   int     r, c;

   if ( image->type    != HOR_FLOAT || dc_image->type  != HOR_FLOAT ||
        dr_image->type != HOR_FLOAT || dsq_image->type != HOR_FLOAT )
      hor_error ( "non-float image (image_derivative)", HOR_FATAL );

   big_width  = image->width;
   big_height = image->height;
   width  = big_width  - 2;
   height = big_height - 2;
   if ( dc_image->width  != width || dc_image->height  != height ||
        dr_image->width  != width || dr_image->height  != height ||
        dsq_image->width != width || dsq_image->height != height )
      hor_error ("derivative images wrong size (image_derivative)", HOR_FATAL);

   dcarr = dc_image->array.f;
   drarr = dr_image->array.f;
   dsqarr = dsq_image->array.f;

   imarr = hor_malloc_ntype ( float *, big_height );
   for ( r = 0; r < big_height; r++ )
      imarr[r] = image->array.f[r] + 1;

   imarr++; /* now imarr points to same positions on image as dcarr, drarr
	       and dsqarr */

   /* do c-derivative */
   for ( r = 0; r < height; r++ )
      for ( imend = imarr[r] + width + 1, pimp = imarr[r]-1, nimp = imarr[r]+1,
	    dcp =  dcarr[r]; nimp != imend; pimp++, nimp++, dcp++ )
	 *dcp = *nimp - *pimp;

   /* do r-derivative */
   imcol = hor_malloc_ntype ( float, big_height );
   imcol++;
   for ( c = 0; c < width; c++ )
   {
      /* set up column in image */
      for ( r = -1; r <= height; r++ )
	 imcol[r] = imarr[r][c];

      for ( r = 0, pimp = imcol-1, nimp = imcol+1; r < height;
	    r++, pimp++, nimp++ )
	 drarr[r][c] = *nimp - *pimp;
   }

   /* calculate RMS derivative */
   for ( r = 0; r < height; r++ )
      for ( dsqend = dsqarr[r] + width, dsqp = dsqarr[r], dcp = dcarr[r],
	    drp = drarr[r]; dsqp != dsqend; dsqp++, dcp++, drp++ )
	 *dsqp = sqrt(*dcp**dcp + *drp**drp);

   hor_free ( (void *) (imcol - 1) );
   hor_free ( (void *) (imarr - 1) );
}

#define RADIAN       57.29577951
#define ATAN_NUMBER 100

static short arctan[ATAN_NUMBER];
static short lcdright[37][3];
static short lcdleft[37][3];
static short rcoffs[8][2];

static Hor_Bool hor_canny_luts_initialised = HOR_FALSE;

/*******************
*   static void @init_canny_luts(void)
*
*   Set up look up tables for Canny operator.
********************/
static void init_canny_luts(void)
{
   int i, j, n;
   /* this is clumsy, but it avoids putting the initialization elsewhere. */
   const char *s1 = "\
435453453546546564564657657657675675760760706706\
071071071017017102102120120213213213231231324324342342\
435435435";
   const char *s2 = "\
071017017102102120120213213213231231324324342342435435435453453\
546546564564657657657675675760760706706071071071";

   for ( i = 0; i < ATAN_NUMBER; i++ )
      arctan[i] = (short) (atan ((double)i/(double)ATAN_NUMBER)*RADIAN + 0.5);

   for(i=0,n=0;i<37;i++) for(j=0;j<3;j++) lcdright[i][j] = (int)(s1[n++]) - 48;
   for(i=0,n=0;i<37;i++) for(j=0;j<3;j++) lcdleft[i][j] = (int)(s2[n++]) - 48;

   rcoffs[0][0] = 0; rcoffs[0][1] = 1;
   rcoffs[1][0] =-1; rcoffs[1][1] = 1;
   rcoffs[2][0] =-1; rcoffs[2][1] = 0;
   rcoffs[3][0] =-1; rcoffs[3][1] =-1;
   rcoffs[4][0] = 0; rcoffs[4][1] =-1;
   rcoffs[5][0] = 1; rcoffs[5][1] =-1;
   rcoffs[6][0] = 1; rcoffs[6][1] = 0;
   rcoffs[7][0] = 1; rcoffs[7][1] = 1;
   hor_canny_luts_initialised = HOR_TRUE;
}

static short orienthf ( float c, float r )
{
   int tangent;

   tangent = (int) fabs ( c*(double)ATAN_NUMBER/r );  /* r is always > c */

   if ( r > 0 )
      return ( ( c > 0 ) ? arctan[tangent] : -arctan[tangent]);
   else
      return ( ( c > 0 ) ? 180-arctan[tangent] : arctan[tangent]-180);
}

static short orientvf ( float c, float r )
{
   int tangent;

   tangent = (int) fabs ( r*(double)ATAN_NUMBER/c );  /* c is always > r */

   if ( r > 0.0 )
      return ( (c > 0.0) ? 90-arctan[tangent] : arctan[tangent]-90);
   else
      return ( (c > 0.0) ? 90+arctan[tangent] : -arctan[tangent]-90);
}
 
static short orientf ( float c, float r )
{ 
   if ( fabs(c) > fabs(r) )
      return ( orientvf ( c, r ) ); 
   else 
      return ( orienthf ( c, r ) ); 
}

/*******************
*   static Hor_Edge_Map *@non_max_supression ( Hor_Sub_Image *deriv_c,
*                                             Hor_Sub_Image *deriv_r,
*                                             Hor_Sub_Image *deriv_sq,
*                                             float          low_thres )
*
*   Performs non-maximum suppression stage of Canny edge detector. Finds points
*   where the directional image derivative in the direction of the
*   image gradient is a maximum relative to its neighbours in that direction.
*   Creates an edge for each such point.
********************/
static Hor_Edge_Map *non_max_supression ( Hor_Sub_Image *deriv_c,
				          Hor_Sub_Image *deriv_r,
					  Hor_Sub_Image *deriv_sq,
					  float          low_thres )
{
   Hor_Image  *dc_image  = &(deriv_c->image);
   Hor_Image  *dr_image  = &(deriv_r->image);
   Hor_Image  *dsq_image = &(deriv_sq->image);
   float **dcarr, **drarr, **dsqarr;
   int     width, height;
   int     c, r, c0, r0;

   Hor_Edge_Map   *edge_map;
   Hor_Edgel    ***edges;

   float      *rowc, *rowr;
   float      *grad, *gradbelow, *gradabove;
   Hor_Edgel **rowep;
    
   float  a, b, c0f, r0f;
   float  st;
   float  del;
   float  dr, dc;
   int    or;
   float  gradratio, fabsgradratio;

   if ( dc_image->type  != HOR_FLOAT || dr_image->type != HOR_FLOAT ||
        dsq_image->type != HOR_FLOAT )
      hor_error ( "non-float image (non_max_supression)", HOR_FATAL );

   width  = dc_image->width;
   height = dc_image->height;
  
   if ( dr_image->width  != width || dr_image->height  != height ||
        dsq_image->width != width || dsq_image->height != height )
      hor_error("derivative images wrong size (non_max_supression)",HOR_FATAL);

   dcarr = dc_image->array.f;
   drarr = dr_image->array.f;
   dsqarr = dsq_image->array.f;

   c0 = deriv_c->c0; c0f = (float) c0 + 1.0F;
   r0 = deriv_c->r0; r0f = (float) r0 + 1.0F;
   edge_map = hor_alloc_edge_map ( c0+1, r0+1, width-2, height-2,
				   HOR_NO_ATTRIB, NULL, NULL );
   edges = edge_map->edges;

   /* make edge list in reverse direction, so insert() will produce list in
      the "correct" direction, i.e. from top-left to botton-right */
   for ( r = height-3; r >= 0; r-- )
   {
      rowc = dcarr[r+1]+1;
      rowr = drarr[r+1]+1;
      grad = dsqarr[r+1]+1;
      gradabove = dsqarr[r]+1;
      gradbelow = dsqarr[r+2]+1;
      rowep=edges[r];
    
      for ( c = width-3; c >= 0; c-- )
      {
	 if ( (st = grad[c]) < low_thres ) continue;

	 if (rowr[c] == 0.0) gradratio = 1000.0F;
	 else                gradratio = rowc[c]/rowr[c];

	 fabsgradratio = fabs(gradratio); 
	 if ( fabsgradratio < 1.5F && fabsgradratio > 0.67F )
	 {
	    if ( ( st <= grad[c-1]    || st < grad[c+1]) &&
		 ( st <= gradabove[c] || st < gradbelow[c] ) )
	       continue; 

	    if ( gradratio > 0 ) /* right diagonal */
	    {
	       if ( st <= gradabove[c-1] || st < gradbelow[c+1] ) continue;

	       a   = gradabove[c-1];
	       b   = gradbelow[c+1];
	       del = (a - b)/((a + b - st*2.0F)*2.0F);
	       if ( fabs(del) > 0.5F ) continue;

	       dr = del + 0.5F; 
	       dc = del + 0.5F;
	    }
	    else
	    {
	       if ( st <= gradbelow[c-1] || st < gradabove[c+1] ) continue;

	       a   = gradbelow[c-1];
	       b   = gradabove[c+1];
	       del = (a - b)/((a + b - st*2.0F)*2.0F);
	       if ( fabs(del) > 0.5F ) continue;

	       dr = 0.5F - del;    
	       dc = del + 0.5F;
	    }
	 }
	 else if ( fabs(rowc[c]) > fabs(rowr[c]) )
	 {
	    if ( st <= grad[c-1] || st < grad[c+1] ) continue;

	    a   = grad[c-1];
	    b   = grad[c+1];
	    del = (a - b)/((a + b - st*2.0F)*2.0F);
	    if ( fabs(del) > 0.5F ) continue;

	    dr = 0.5F; 
	    dc = del + 0.5F;
	 }
	 else
	 {  
	    if ( st <= gradabove[c] || st < gradbelow[c]) continue;

	    a   = gradabove[c];
	    b   = gradbelow[c];
	    del = (a - b)/((a + b - st*2.0F)*2.0F);
	    if ( fabs(del) > 0.5F ) continue;

	    dr = del + 0.5F;
	    dc = 0.5F;   /* pixel centre */
	 }

	 or = orientf ( rowc[c], rowr[c] ); 
	 hor_add_edge ( edge_map, st, c, r,
		        c0f + (float) c + dc, r0f + (float) r + dr,
		        ((float) or)/RADIAN, HOR_ORIENT_ATTRIB,
		        (void *) ((int) ((float)or/10.0F + 18.5F)), NULL,
		        hor_display_edge );
      }
   }

   return edge_map;
}

#define LIGHT 0
#define CENTRE 1
#define DARK 2
#define ROW 0
#define COL 1

static Hor_Edgel *left_neighbour ( Hor_Edgel *edgel, Hor_Edgel ***edges )
{
   Hor_Edgel *eptr;
   int        row, col, nrow, ncol, position, neigh;

   row = edgel->r;
   col = edgel->c;
   for ( position = 0; position < 3; position++ )
   {
      neigh = lcdleft[(int) edgel->attrib][position];
      nrow  = row + rcoffs[neigh][ROW];
      ncol  = col + rcoffs[neigh][COL];
      eptr  = edges[nrow][ncol];
      if ( eptr != NULL ) return eptr;
   }

   return NULL;
}

static Hor_Edgel *right_neighbour ( Hor_Edgel *edgel, Hor_Edgel ***edges )
{
   Hor_Edgel *eptr;
   int    row, col, nrow, ncol, position, neigh;

   row = edgel->r;
   col = edgel->c;
   for ( position = 0; position < 3; position++ )
   {
      neigh = lcdright[(int) edgel->attrib][position];
      nrow  = row + rcoffs[neigh][ROW];
      ncol  = col + rcoffs[neigh][COL];
      eptr  = edges[nrow][ncol];
      if ( eptr != NULL ) return eptr;
   }

   return NULL;
}

static Hor_DList grow_left ( Hor_DList    start_node, Hor_Edgel   *start_edgel,
			     Hor_Edgel   *new_edgel,  Hor_Edgel ***edges,
			     Hor_Estring *string )
{
   if ( new_edgel == NULL ) return NULL;

   if ( new_edgel == start_edgel ) /* found closed contour */
   {
      /* make list circular */
      start_node->next = string->leftmost;
      string->leftmost->prev = start_node;
      string->leftmost = start_node;
      return start_node;
   }

   if ( new_edgel->status == HOR_IN_STRING ) return NULL;

   string->leftmost = hor_dinsert_before ( string->leftmost, new_edgel );
   string->length++;
   new_edgel->status = HOR_IN_STRING;
   new_edgel->string = string;
   return ( grow_left ( start_node, start_edgel,
		        left_neighbour ( new_edgel, edges ), edges, string ) );
}

static Hor_DList grow_right ( Hor_DList    start_node, Hor_Edgel *start_edgel,
			      Hor_Edgel   *new_edgel,  Hor_Edgel ***edges,
			      Hor_Estring *string )
{
   if ( new_edgel == NULL ) return NULL;

   if ( new_edgel == start_edgel ) /* found closed contour */
   {
      /* make list circular */
      start_node->prev = string->rightmost;
      string->rightmost->next = start_node;
      string->rightmost = start_node;
      return start_node;
   }

   if ( new_edgel->status == HOR_IN_STRING ) return NULL;

   string->rightmost = hor_dinsert_after ( string->rightmost, new_edgel );
   string->length++;
   new_edgel->status = HOR_IN_STRING;
   new_edgel->string = string;
   return ( grow_right ( start_node, start_edgel,
			 right_neighbour ( new_edgel, edges ), edges, string));
}

/*******************
*   static void @makestrings ( Hor_Edge_Map *edge_map )
*
*   Takes a edge map and forms the edges into connected strings.
*   Closed contours are detected.
********************/
static void makestrings ( Hor_Edge_Map *edge_map )
{
   Hor_Edgel  ***edges = edge_map->edges; /* edges[i][j] is a pointer
					     to edgel or NULL */
   Hor_List      firstisolated;
   Hor_Edgel    *eptr;
   Hor_Estring  *string;

   firstisolated = edge_map->edge_list; /* this is the first we know
					   to be isolated */
   while ( hor_list_non_null(firstisolated) )
   {
      eptr = hor_node_contents(firstisolated);
      if ( eptr->status != HOR_IN_STRING )
      {
	 /* we have a new string! */
	 string = hor_add_estring ( edge_map, eptr,
				    HOR_NO_ATTRIB, NULL, NULL, NULL );
	 if ( grow_left  ( string->leftmost, eptr,
			   left_neighbour ( eptr, edges ),
			   edges, string ) == NULL )
	    grow_right ( string->leftmost,
			 (Hor_Edgel *) hor_node_contents(string->leftmost),
			 right_neighbour ( eptr, edges ), edges, string );
      }

      firstisolated = firstisolated->next;
   }
}

/*******************
*   static void @hysteresis ( Hor_Edge_Map *edge_map, (edge map)
*                            float high_thres, (upper hysteresis threshold)
*                            int string_length_threshold )
*
*   Applies hysteresis stage of Canny edge detector. Hor_Edgels below the low
*   threshold have already been rejected and edge strings formed, so the
*   strings now have to be checked to see if they have at least one
*   above-upper-threshold edge in them. Strings having smaller than
*   string_length_thres edges are removed.
********************/
static void hysteresis ( Hor_Edge_Map *edge_map,
			 float high_thres, int string_length_threshold )
{
   Hor_List    *slist;
   Hor_Estring *sptr;
   Hor_DList    elist, start;
   Hor_Edgel   *eptr;
   int          keep_string;

   slist = &(edge_map->string_list);
   while ( *slist != NULL ) /* traverse string list */
   {
      sptr = (Hor_Estring *) hor_node_contents(*slist);
      keep_string = 0;
      if ( sptr->length >= string_length_threshold )
      {
	 start = sptr->leftmost;
	 eptr = (Hor_Edgel *) hor_node_contents(start);
	 if ( eptr->strength > high_thres ) /* found an above-threshold edge */
	    keep_string = 1;
	 else
	    for ( elist = start->next; elist != NULL && elist != start;
		  elist = elist->next ) /* traverse edge list in string */
	    {
	       eptr = (Hor_Edgel *) elist->contents;
	       if ( eptr->strength > high_thres ) /* found an above-threshold
						     edge */
	       {
		  keep_string = 1;
		  break;
	       }
	    }
      }

      if ( !keep_string )
	 *slist = hor_delete_estring ( edge_map, slist );
      else
	 slist = &((*slist)->next);
   }
}

/*******************
*   Hor_Edge_Map *@hor_canny ( Hor_Image *imptr, (input image)
*                     float *gauss_mask,       (1D gaussian convolution mask)
*                     int    gauss_size,       (half-size of mask)
*                     float  low_thres,        (lower hysteresis threshold)
*                     float  high_thres,       (upper hysteresis threshold)
*                     int    string_length_thres, (lower threshold on string
*                                                  length)
*                     int    c1, int r1,       (region in which Canny operator
*                     int    c2, int r2 )       is to be run)
*
*   Applies Canny edge detector on input image in region with top-left and
*   bottom-right corners (c1,r1) and (c2,r2) respectively.
********************/
Hor_Edge_Map *hor_canny ( Hor_Image *imptr,
			  float *gauss_mask,
			  int    gauss_size,
			  float  low_thres,
			  float  high_thres,
			  int    string_length_thres,
			  int    c1, int r1,
			  int    c2, int r2 )
{
   Hor_Sub_Image *gauss;    /* Gaussian convolved image */
   Hor_Sub_Image *deriv_c;  /* column derivative of Gaussian convolved image */
   Hor_Sub_Image *deriv_r;  /* row derivative of Gaussian convolved image */
   Hor_Sub_Image *deriv_sq; /* sum of squares of column & row derivatives */
   int            width, height;
   Hor_Edge_Map  *edge_map;

   /* set up look-up tables if this is the first call to hor_canny() */
   if ( !hor_canny_luts_initialised ) init_canny_luts();

   /* adjust rectangular region to take account of convolution mask size */
   hor_adjust_region_for_border ( imptr->width, imptr->height,
				  gauss_size+2, gauss_size+2,
				  gauss_size+2, gauss_size+2,
				  &c1, &r1, &c2, &r2 );
   width  = c2 - c1;
   height = r2 - r1;

   /* can't do hor_canny in anything less than one by one! */
   if ( width < 1 || height < 1 )
   {
      hor_errno = HOR_IMPROC_REGION_TOO_SMALL;
      return NULL;
   }

   /* allocate space for smoothed image and derivative arrays */
   gauss    = hor_alloc_sub_image (c1-2, r1-2, width+4, height+4, HOR_FLOAT, NULL);
   deriv_c  = hor_alloc_sub_image (c1-1, r1-1, width+2, height+2, HOR_FLOAT, NULL);
   deriv_r  = hor_alloc_sub_image (c1-1, r1-1, width+2, height+2, HOR_FLOAT, NULL);
   deriv_sq = hor_alloc_sub_image (c1-1, r1-1, width+2, height+2, HOR_FLOAT, NULL);

   /* convolve image with separable 2-D Gaussian */
   hor_convolve_image ( imptr, gauss_mask, gauss_size,
		        gauss_mask, gauss_size, gauss );

   /* take derivative of image */
   image_derivative ( gauss, deriv_c, deriv_r, deriv_sq );
   hor_free_sub_image ( gauss );

   /* perform non-maximum suppression */
   edge_map = non_max_supression ( deriv_c, deriv_r, deriv_sq,
				   low_thres*low_thres );

   hor_free_sub_images ( deriv_sq, deriv_r, deriv_c, NULL );

   /* construct strings of connected edges */
   makestrings ( edge_map );

   /* perform hysteresis */
   hysteresis ( edge_map, high_thres, string_length_thres );

   return edge_map;
}
