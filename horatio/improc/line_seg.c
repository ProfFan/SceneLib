#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "horatio/global.h"
#include "horatio/list.h"
#include "horatio/image.h"
#include "horatio/improc.h"

#ifndef PI
#define PI 3.1415927
#endif
#define MIN_CONF 0
#define SQR(X) ((X)*(X))
#define SGN(X) (((X)<0.0)?-1:1)

typedef struct Edgel_S      EdgeL;
typedef struct Link_S       Link;
typedef struct Edge_point_S Edge_point;
typedef struct Line_S       Line;
typedef struct Plist_S      Plist;
typedef struct Conf_S       Conf;
typedef struct Precal_S     Precal;

struct Edgel_S      { float row, col, str, ori; };
struct Link_S       { Line *l;	/* a line which ends at this point */
		      short dir; /* 0 = starts, 1 = ends */
		      Link *next; };
struct Edge_point_S { EdgeL edge_point;
		      short x, y;
		      Hor_Estring *str;
		      Link *links; };
struct Line_S       { double sum_x, sum_y, cen_x, cen_y;
		      double mxx, myy, mxy;
		      int point_count;
		      Plist *s, *e;
		      Conf *best;
		      Line *next, *prev; };
struct Plist_S      { Edge_point *p;
		      Plist *next, *prev; };
struct Conf_S       { Line *l;
		      short dir; /* 0 for link via s, 1 for via e */
		      double conf;
		      Precal *moments;
		      Conf *next; };
struct Precal_S     { double sum_x, sum_y, cen_x, cen_y, mxx, myy, mxy; 
		      int point_count; };




static double LAMBDA_LIM = 0.5;


static Line *applin ( Hor_List edge_list, int max_edgel_count, int flag, Edge_point **pt_arr, int *n_used_pt, float c0f, float r0f );
static void  Link_Up(Edge_point *points, int count, Line **lines);
static void  Calc_Confidence(Line *l1, Conf *c1, Line *l2, Conf *c2, Edge_point *p);
static void  Do_End_Conf(Line *l, Plist *p, int d);
static void  Calc_Line_Conf(Line *line);
static void  Cleanup_Best(Line *l, short d);
static int   Comp_Conf(Conf **c1, Conf **c2);
static void  Sort_Conf(Line *l);
static void  Merge_Line(Line *l, Line **lines);
static void  Find_Lines(Edge_point *points, int edgel_count, Line **lines);
static void  free_point_array(Edge_point *point_array, int edgel_count);
static void  free_link_list(Link *link);
static void  free_lines(Line *lines);

/*******************
*   Hor_Line_Segment_Map *@hor_fit_line_segments ( Hor_Edge_Map *e, double lambda_lim )
*
*   Fits line segments to an edge map using Will Dickson's algorithm.
********************/
Hor_Line_Segment_Map *hor_fit_line_segments(Hor_Edge_Map *e, double lambda_lim)
{
    Hor_Line_Segment_Map *map;
    Line *lines, *cur_line;
    double nx, ny, os;
    double lambda, w1x, w1y, w2x, w2y, tf1, tf2;
    Edge_point *pt_arr;  /* to free mem allocated in applin */
    int        n_used_pt;

    LAMBDA_LIM = lambda_lim;

    map = hor_alloc_line_segment_map ( HOR_NO_ATTRIB, NULL, NULL );
    map->c0     = e->c0;
    map->r0     = e->r0;
    map->width  = e->width;
    map->height = e->height;

    /* transmit all edges, even the one non linked to a chain */
    lines = applin(e->edge_list, e->nedges, 1, &pt_arr, &n_used_pt,
		   (float) e->c0, (float) e->r0);

    for (cur_line = lines; cur_line != NULL; cur_line = cur_line->next) {
	lambda = (cur_line->mxx + cur_line->myy -
		  sqrt((double)((cur_line->mxx - cur_line->myy)
		       *(cur_line->mxx - cur_line->myy)
		       + 4*cur_line->mxy*cur_line->mxy))) / 2.0;

	w1x=cur_line->mxy; w1y=lambda-cur_line->mxx;
	w2x=lambda-cur_line->myy; w2y=cur_line->mxy;

	if((tf1 = w1x*w1x+w1y*w1y) > (tf2 = w2x*w2x+w2y*w2y)) {
	    tf1 = sqrt(tf1);
	    nx = w1x/tf1;
	    ny = w1y/tf1;
	} else {
	    tf2 = sqrt(tf2);
	    nx = w2x/tf2;
	    ny = w2y/tf2;
	}

	os = cur_line->cen_x * nx + cur_line->cen_y * ny;

	tf1 = cur_line->s->p->edge_point.col * ny
	  - cur_line->s->p->edge_point.row * nx;
	tf2 = cur_line->e->p->edge_point.col * ny
	  - cur_line->e->p->edge_point.row * nx;

	if (SQR((os * nx + tf1 * ny)-(os * nx + tf2 * ny)) + 
	    SQR((os * ny - tf1 * nx)-(os * ny - tf2 * nx)) < SQR(15.0)) continue;

	{  
	    double x1, y1, x2, y2, angle, orient, d;
	    Plist *ep;

	    x1 = os * nx + tf1 * ny + e->c0,
	    y1 = os * ny - tf1 * nx + e->r0,
	    x2 = os * nx + tf2 * ny + e->c0,
	    y2 = os * ny - tf2 * nx + e->r0;
	    angle = 180.0/PI*atan2(y2 - y1, x2 - x1);
	    orient = 0.0;
	    for (ep = cur_line->s; ep!=NULL; ep=ep->next)
	       orient -= ep->p->edge_point.ori;
	    orient /= cur_line->point_count;
	    d = fabs(orient - angle);
	    hor_add_line_segment ( map, x1, y1, x2, y2, angle,
				   (d < 80.0 || d > 280.0) ? 1 : -1,
				   HOR_FALSE, HOR_FALSE,
				   HOR_NO_ATTRIB, NULL, NULL,
				   hor_display_line_segment );
	}
    }
    free_point_array( pt_arr, n_used_pt);
    free_lines(lines);
    
    return map;
}


static Line *applin ( Hor_List edge_list, int max_edgel_count, int flag, Edge_point **pt_arr, int *n_used_pt, float c0f, float r0f )
{
  Line *lines = NULL;
  int edgel_count = 0;
  Edge_point *point_array, *point;
  Hor_Edgel *e;
  Hor_List s;

  point_array = (Edge_point*) malloc(max_edgel_count*sizeof(Edge_point));
  if (point_array == NULL) {
      fprintf(stderr,"T'malloc fell over on point_array (%d elements)\n",max_edgel_count);
      exit(1);
  }

  point=point_array; 
  for (s=edge_list; s!=NULL; s=s->next) {
      e = s->contents;
      if (e->status == HOR_IN_STRING || flag == 0) {
	  point->edge_point.row = e->rf - r0f;
	  point->edge_point.col = e->cf - c0f;
	  point->edge_point.str = e->strength;
	  point->edge_point.ori = e->angle*57.29577951;
	  point->x = e->c;
	  point->y = e->r;
	  point->links = NULL;
	  point++; edgel_count++;
      }
  }

/*
  The data's all in now; just a question of processing it.
*/

  Find_Lines(point_array, (int)edgel_count, &lines);

  /* Error, should be freed later => use pt_arr and n_used_pt */
/*  free_point_array(point_array, edgel_count); */
  *pt_arr = point_array;
  *n_used_pt = edgel_count;

  return lines;
}


static void Link_Up(Edge_point *points, int count, Line **lines)
{
  register Edge_point *cur_pt, *match_pt;
  Line *cur_line;
  double x1, x2, y1, y2;
  short i;
  register short diff_col, cur_col, j, cur_row;
  Link *cur_ln;
	      
  *lines = NULL;

  for(cur_pt=points, i=0; i<count; i++, cur_pt++)
    {
      cur_row = cur_pt->y;
      cur_col = cur_pt->x;
      for(j=i+1, match_pt=(cur_pt+1);
	  j<count && match_pt->y<cur_row+2;
	  j++, match_pt++)
	{
	  diff_col = cur_col - match_pt->x;
	  if ((diff_col < 2) && (diff_col > -2))
	    {
	      if (NULL == (cur_line = (Line*)malloc(sizeof(Line))))
		{ fprintf(stderr, "couldn't allocate space for line (1)\n");
		  exit(7); }
	      cur_line->sum_x = (x1 = cur_pt->edge_point.col)
		+ (x2 = match_pt->edge_point.col);
	      cur_line->sum_y = (y1 = cur_pt->edge_point.row)
		+ (y2 = match_pt->edge_point.row);
	      cur_line->cen_x = cur_line->sum_x / 2.0;
	      cur_line->cen_y = cur_line->sum_y / 2.0;
	      cur_line->mxx = (x1-cur_line->cen_x)*(x1-cur_line->cen_x)
		+ (x2-cur_line->cen_x)*(x2-cur_line->cen_x);
	      cur_line->myy = (y1-cur_line->cen_y)*(y1-cur_line->cen_y)
		+ (y2-cur_line->cen_y)*(y2-cur_line->cen_y);
	      cur_line->mxy = (x1-cur_line->cen_x)*(y1-cur_line->cen_y)
		+ (x2-cur_line->cen_x)*(y2-cur_line->cen_y);

	      cur_line->point_count = 2;

	      if (*lines != NULL) (*lines)->prev = cur_line;
	      cur_line->prev = NULL;
	      cur_line->next = *lines;
	      *lines = cur_line;

	      if (NULL == (cur_line->s = (Plist*)malloc(sizeof(Plist))))
		{ fprintf(stderr, "couldn't allocate space for plist (1)\n");
		  exit(7); }
	      
	      if (NULL == (cur_line->e = (Plist*)malloc(sizeof(Plist))))
		{ fprintf(stderr, "couldn't allocate space for plist (1)\n");
		  exit(7); }

	      cur_line->s->prev = NULL;
	      cur_line->s->next = cur_line->e;
	      cur_line->s->p = cur_pt;

	      cur_line->e->next = NULL;
	      cur_line->e->prev = cur_line->s;
	      cur_line->e->p = match_pt;

	      cur_line->best = NULL;

	      if (NULL == (cur_ln = (Link*)malloc(sizeof(Link))))
		{ fprintf(stderr, "couldn't allocate space for link (1)\n");
		  exit(7); }
	      
	      cur_ln->l = cur_line;
	      cur_ln->dir = 0;
	      cur_ln->next = cur_pt->links;
	      cur_pt->links = cur_ln;
	      
	      if (NULL == (cur_ln = (Link*)malloc(sizeof(Link))))
		{ fprintf(stderr, "couldn't allocate space for link (2)\n");
		  exit(7); }
	      
	      cur_ln->l = cur_line;
	      cur_ln->dir = 1;
	      cur_ln->next = match_pt->links;
	      match_pt->links = cur_ln;
	    }
	}
    }
}

/*
  confidence for a whole line:
    foreach link from s->p where we have no conf
      generate a conf structure for each of the two lines
      compute the appropriate confidence
      put it in both line lists in the appropriate place.
   ditto for the links from e->p

   Note the the best pointers for all the lines are set to NULL at
   creation time, so we can safely assume that non-null values in best
   correspond to genuine data.
*/


static void Calc_Confidence(Line *l1, Conf *c1, Line *l2, Conf *c2, Edge_point *p)
{
  Precal *pre;
  double fb;
  Edge_point *p1, *p2;
  double dx1, dy1, dx2, dy2, lambda;

  if(c1->dir)
    p1 = l1->s->p;
  else
    p1 = l1->e->p;
  if(c2->dir)
    p2 = l2->s->p;
  else
    p2 = l2->e->p;

  fb = (p1->edge_point.col - p->edge_point.col)
    * (p2->edge_point.col - p->edge_point.col)
      + (p1->edge_point.row - p->edge_point.row)
	* (p2->edge_point.row - p->edge_point.row);

  if(fb > 0)
    c1->conf = c2->conf = -999.99;
  else
    {
      if ((pre = c1->moments) == NULL)
	{
	  if (NULL == (pre = (Precal*)malloc(sizeof(Precal))))
	    { fprintf(stderr, "Can't get space for precal.\n");
	      exit(7); }
	  c1->moments = c2->moments = pre;
	}

      pre->sum_x = l1->sum_x + l2->sum_x - p->edge_point.col;
      pre->sum_y = l1->sum_y + l2->sum_y - p->edge_point.row;
      pre->point_count = l1->point_count + l2->point_count - 1;
      pre->cen_x = pre->sum_x / pre->point_count;
      pre->cen_y = pre->sum_y / pre->point_count;
      dx1 = pre->cen_x - l1->cen_x;
      dy1 = pre->cen_y - l1->cen_y;
      dx2 = pre->cen_x - l2->cen_x;
      dy2 = pre->cen_y - l2->cen_y;

      pre->mxx = l1->mxx + dx1 * l1->point_count * dx1
	+ l2->mxx + dx2 * l2->point_count * dx2
	  - (p->edge_point.col - pre->cen_x)
	    * (p->edge_point.col - pre->cen_x);

      pre->myy = l1->myy + dy1 * l1->point_count * dy1
	+ l2->myy + dy2 * l2->point_count * dy2
	  - (p->edge_point.row - pre->cen_y)
	    * (p->edge_point.row - pre->cen_y);

      pre->mxy = l1->mxy + l1->point_count * dx1 * dy1
	+ l2->mxy + l2->point_count * dx2 * dy2
	  - (p->edge_point.col - pre->cen_x)
	    * (p->edge_point.row - pre->cen_y);

      lambda = (pre->mxx + pre->myy -
		sqrt((pre->mxx - pre->myy)*(pre->mxx - pre->myy)
		     + 4*pre->mxy*pre->mxy)) / 2.0;

      c1->conf = c2->conf = LAMBDA_LIM - lambda / pre->point_count;
    }
}


static void Do_End_Conf(Line *l, Plist *p, int d)
{
  Conf *cur_conf, *other_conf, **prev;
  Line *poss[7];		/* max size for 8-connected excluding self */
  int n_poss, i;
  short done;
  Link *cur_link;

  for(n_poss = 0, cur_link = p->p->links;
      cur_link != NULL; cur_link = cur_link->next) {
    if (cur_link->l != l)
      {
	for(done = 0, cur_conf = l->best;
	    (cur_conf != NULL) && !done;
	    cur_conf = cur_conf->next)
	  if (cur_link->l == cur_conf->l) done = 1;
	if (!done)
	  poss[n_poss++] = cur_link->l;
      }
    if (n_poss>6)
       printf("Oh no, bigger 'n 6 (%d)!\n",n_poss);
  }

  for(i=0; i<n_poss; i++)
    {
      if ((NULL == (cur_conf = (Conf*)malloc(sizeof(Conf)))) ||
	  (NULL == (other_conf = (Conf*)malloc(sizeof(Conf)))))
	{ fprintf(stderr, "No memory for new Conf links\n");
	  exit(7); }
      cur_conf->l = poss[i];
      other_conf->l = l;
      cur_conf->dir = d;
      if(poss[i]->s->p == p->p)
	other_conf->dir = 0;
      else if (poss[i]->e->p == p->p)
	other_conf->dir = 1;
      else
	{ fprintf(stderr, "Incorrect links detected in Do_Line_Conf\n");
	  exit(1); }

      cur_conf->moments = other_conf->moments = NULL;

      Calc_Confidence(l, cur_conf, poss[i], other_conf, p->p);

      prev = &(l->best);
      while((*prev != NULL) && ((*prev)->conf > cur_conf->conf))
	prev = &((*prev)->next);
      cur_conf->next = *prev;
      *prev = cur_conf;

      prev = &(poss[i]->best);
      while((*prev != NULL) && ((*prev)->conf > other_conf->conf))
	prev = &((*prev)->next);
      other_conf->next = *prev;
      *prev = other_conf;
    }
}


static void Calc_Line_Conf(Line *line)
{
  Do_End_Conf(line, line->s, 0);
  Do_End_Conf(line, line->e, 1);
}


static void Cleanup_Best(Line *l, short d) /* remove any links from/to this line */
                                           /* via the end given by d */
{
  Conf *b, *ob, **pb, **opb;
  Edge_point *p;
  Line *ol;

  p = (d?l->e->p:l->s->p);

  for(b = l->best, pb = &l->best; b != NULL;/* done in loop */)
    {
      if (b->dir == d)
	{
	  ol = b->l;
	  opb = &ol->best;
	  if ((ob = *opb) == NULL)
	    { fprintf(stderr, "No return ptr !!\n");
	      exit(1); }
	  while ((ob->l != l) || (p != (ob->dir?ol->e->p:ol->s->p)))
	    { opb = &ob->next;
	      if((ob = ob->next) == NULL)
		{ fprintf(stderr, "No return ptr !!\n");
		  exit(1); }
	    }

	  *pb = b->next;
	  *opb = ob->next;
	  free((char*)ob);

	  if(b->moments != NULL)
	    {
	      free((char*)b->moments);
	      b->moments = NULL;
	    }

	  free((char*)b);
	  b = *pb;
	}
      else
	{
	  pb = &b->next;
	  b = *pb;
	}
    }
}


static int Comp_Conf(Conf **c1, Conf **c2)
{
  return (((**c1).conf >= (**c2).conf) ? -1 : 1);
}

static void Sort_Conf(Line *l)
{
  int count;
  Conf **conf_ary, *c, **cp;

  if ( (c = l->best) != NULL )
  {
      count = 0;
      while(c != NULL)
	{ c = c->next; count++; }

      if ( (conf_ary = hor_malloc_ntype ( Conf *, count )) == NULL )
	 hor_error ( "No room for sorting array (Sort_Conf)", HOR_FATAL );

      for(cp = conf_ary, c = l->best; c != NULL; c = c->next, cp++)
	*cp = c;

      qsort((char*)conf_ary, count, sizeof(Conf*),
	    (int (*)(const void *, const void *)) Comp_Conf);

      c = l->best = *conf_ary;
      cp = conf_ary+1;
      count--;

      while (count>0)
	{
	  c->next = *cp;
	  cp++;
	  c = c->next;
	  count--;
	}
      c->next = NULL;

      hor_free ( (void *) conf_ary );
    }
}


static void Merge_Line(Line *l, Line **lines) /* merge line, adj ptrs, recalc conf */
{
  Edge_point *p;
  Link *ln, **pln;
  short d, od;
  Line *ol;
  Conf **pb, **opb, *b, *ob;
  Plist *plist, *ptemp;

  l->sum_x = l->best->moments->sum_x;
  l->sum_y = l->best->moments->sum_y;
  l->cen_x = l->best->moments->cen_x;
  l->cen_y = l->best->moments->cen_y;
  l->mxx = l->best->moments->mxx;
  l->myy = l->best->moments->myy;
  l->mxy = l->best->moments->mxy;
  l->point_count = l->best->moments->point_count;

  /* remove pointers from node */
  if (l->best->dir)
    p = l->e->p;
  else
    p = l->s->p;

  for(ln = p->links, pln = &p->links; ln != NULL;/* done in loop */)
    if((ln->l == l) || (ln->l == l->best->l))
      {
	*pln = ln->next;
	free((char*)ln);
	ln = *pln;
      }
    else
      {
	pln = &ln->next;
	ln = ln->next;
      }

  /* Merge neighbour list for the two lines, killing off any links via */
  /* the endpoint which is disappearing and changing the direction of */
  /* the links of the second line if necessary. */

  d = l->best->dir;
  ol = l->best->l;
  od = l->best->l->best->dir;

  Cleanup_Best(l, d);
  Cleanup_Best(ol, od);
	      
  /* merge neighbour list */
  pb = &l->best;
  while (*pb != NULL)
    pb = &(*pb)->next;
  *pb = ol->best;
  while (*pb != NULL)
    {
      (*pb)->dir = d;
      pb = &(*pb)->next;
    }

  /* merge point list */
  if (d)			/* then add points after l->e */
    {
      if (od)			/* we need to reverse ol's points */
	{
	  l->e->next = ol->e->prev; /* do not duplicate ept */
	  ol->e->prev->next = l->e;
	  plist = ol->e->prev;
	  free((char*)ol->e);
	  while (plist != NULL)
	    {
	      ptemp = plist->next;
	      plist->next = plist->prev;
	      plist->prev = ptemp;
	      plist = plist->next;
	    }
	  l->e = ol->s;
	}
      else
	{
	  l->e->next = ol->s->next;
	  ol->s->next->prev = l->e;
	  free((char*)ol->s);
	  l->e = ol->e;
	}
    }
  else				/* add points before l->s */
    {
      if (od)			/* no need to reverse ol's points */
	{
	  l->s->prev = ol->e->prev;
	  ol->e->prev->next = l->s;
	  free((char*)ol->e);
	  l->s = ol->s;
	}
      else			/* reverse them */
	{
	  l->s->prev = ol->s->next;
	  ol->s->next->prev = l->s;
	  plist = ol->s->next;
	  free((char*)ol->s);
	  while (plist != NULL)
	    {
	      ptemp = plist->next;
	      plist->next = plist->prev;
	      plist->prev = ptemp;
	      plist = plist->prev;
	    }
	  l->s = ol->e;
	}
    }

  /* We still have to change all the refs to to ol so that they now */
  /* point to l.  Then at last we will be able to remove ol from the */
  /* list. */

  if (d)
    p = l->e->p;
  else
    p = l->s->p;

  for(ln = p->links; ln != NULL; ln = ln->next)
    if (ln->l == ol) ln->l = l;

  for(b = ol->best;		/* still not disturbed */
      b != NULL;
      b = b->next)
    for(ob = b->l->best; ob != NULL; ob = ob->next)
      if (ob->l == ol) ob->l = l;

  /* finally we can rm ol from the list */
  if (ol->next != NULL)
    ol->next->prev = ol->prev;
  if (ol->prev != NULL)
    ol->prev->next = ol->next;
  else
    *lines = ol->next;

  free((char*)ol);

  /* The last thing to do is to update the confidences for our new */
  /* line's links, as these will no doubt have changed. */

  for(b = l->best; b != NULL; b = b->next)
    {
      p = (b->dir?l->e->p:l->s->p);
      ol = b->l;
      ob = b->l->best;
      opb = &b->l->best;
      while ((ob->l != l) || (p != (ob->dir?ol->e->p:ol->s->p)))
	{ opb = &ob->next; ob = *opb; }
      if (b->dir)
	Calc_Confidence(l, b, b->l, ob, l->e->p);
      else
	Calc_Confidence(l, b, b->l, ob, l->s->p);
      pb = &b->l->best;
      while ((*pb != NULL) && ((*pb)->conf > ob->conf))
	pb = &(*pb)->next;
      if (opb != pb)
	{
	  *opb = ob->next;
	  ob->next = *pb;
	  *pb = ob;
	}
    }
  Sort_Conf(l);
}

static void Find_Lines(Edge_point *points, int edgel_count, Line **lines)
{
  int adj;
  Line *l;
  
  Link_Up(points, edgel_count, lines); /* create the putative (=supposed)
					  lines (initialise linking process
					  by linking 2 adjacent edges */

  for (l = *lines; l != NULL; l = l->next)
    Calc_Line_Conf(l);

  do
    {
      adj = 0;
      for (l = *lines; l != NULL; l = l->next)
	while ((l->best != NULL)
	       && (l->best->l->best->l == l)
	       && (l->best->conf >= MIN_CONF))
	  {
	    adj++;
	    Merge_Line(l, lines); /* merge line, adj ptrs, recalc conf */
	  }
    } while(adj);

}


/*
  To find lines:
    The strategy is split-and-merge.  To start with make a line for
    every link.  Then work around finding local maxima of confidence for
    each possible merging, and merge at these local maxima.  

  So: each line sets up a pointer to the best line for it to merge with;
  a merge is carried out when two lines agree with each other (a pointer
  one way is matched by another pointer the other way).  This change
  then needs to be propagated as far as necessary (and no further) to
  recompute the merge confidences.

  The architecture required is:
    The linked list of edgels (which is already here);
    A list of lines which start and finish at every point (or is this
      two lists?)
    The linked list of currently_active lines, which should include a
      list of potential neighbours at each end.

  We do not really need the linked list of edgels, as that can be
  completely replaced by the node-to-line pointers and the currently
  active lines, so long as these are maintained correctly (which they
  WILL (or else!)).

  There is some memory usage benefit to be had in dynamically allocating
  the array of nodes, which should be fairly sparse towards the end of
  the grouping operation, while the pointers from lines to their member
  edge-points will increase as the algorithm progresses.  On the other
  hand there is potential loss in speed and complexity...
*/

static void free_point_array(Edge_point *point_array, int edgel_count)
{
    Edge_point *p;
    int i;

    for (i=0, p = point_array; i<edgel_count; i++, p++) {
	free_link_list(p->links);
    }
    free(point_array);
}

static void free_link_list(Link *link)
{
    Link *l=link, *lf;

    while (l!=NULL) {
	lf = l; 
	l = l->next;
	free(lf);
    }
}


static void  free_Conf( Conf *conf)
{
   if ( !conf ) return;
   free_Conf( conf->next);
   /* the "moments" structure is shared between two lines ; to avoid
      freing it twice, let's use point_count as a flag */
   if (conf->moments) { /* I don't understand why this test is necessary */
      if ( conf->moments->point_count >=0 )  /* count should always be >0 */
	conf->moments->point_count = -1 ;
      else 
	free( conf->moments);
   }
   free( conf);
}

static void free_forward_Plist_node( Plist *plist)
{
   if ( !plist) return ;
   free_forward_Plist_node( plist->next);
   free( plist);
}

static void free_lines(Line *lines)
{
    Line *l=lines, *lf;

    while (l!=NULL) {
	lf = l;
	l = l->next;
        free_Conf( lf->best);
	free_forward_Plist_node( lf->s);
	free(lf);
    }
}
