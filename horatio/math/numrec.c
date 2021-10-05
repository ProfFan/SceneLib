/* Modified from Numerical Recipes:

   Copyright Numerical Recipes Software 1987, 1988

   HORATIO mods: Copyright 1993 Philip F. McLauchlan
                                (pm@robots.oxford.ac.uk)
                                Robotics Research Group, Oxford University. */
#include <math.h>
#include <stdlib.h>
#ifdef HOR_TRANSPUTER
#include <mathf.h>
#else
#define fabsf fabs
#endif

#ifdef HOR_REDUCED_LIB
#include <stdiored.h>
#else
#include <stdio.h>
#endif

#include "horatio/global.h"
#include "horatio/math.h"

#define TINY 1.0e-20;
#define TINYF 1.0e-6F;
#define ITMAX 150
#define EPS 3.0e-7

void hor_ludcmp ( double **a, int n, int *indx, double *d, double *vv )
{
	int i,hor_imax=0,j,k;
	double big,dum,sum,temp;

	*d=1.0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0)
		   hor_error("singular matrix (hor_ludcmp)", HOR_FATAL);

		vv[i]=1.0/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a[i][j];
			for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			sum=a[i][j];
			for (k=0;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				hor_imax=i;
			}
		}
		if (j != hor_imax) {
			for (k=0;k<n;k++) {
				dum=a[hor_imax][k];
				a[hor_imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[hor_imax]=vv[j];
		}
		indx[j]=hor_imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n-1) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<n;i++) a[i][j] *= dum;
		}
	}
}

void hor_lubksb ( double **a, int n, int *indx, double **b )
{
	int i,ii=-1,ip,j;
	double sum;

	for (i=0;i<n;i++) {
		ip=indx[i];
		sum=b[ip][0];
		b[ip][0]=b[i][0];
		if (ii>=0)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j][0];
		else if (sum) ii=i;
		b[i][0]=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=b[i][0];
		for (j=i+1;j<n;j++) sum -= a[i][j]*b[j][0];
		b[i][0]=sum/a[i][i];
	}
}

void hor_ludcmpf ( float **a, int n, int *indx, float *d, float *vv )
{
	int i,hor_imax=0,j,k;
	float big,dum,sum,temp;

	*d=1.0F;
	for (i=0;i<n;i++) {
		big=0.0F;
		for (j=0;j<n;j++)
			if ((temp=fabsf(a[i][j])) > big) big=temp;
		if (big == 0.0F) {
		   hor_errno = HOR_MATH_MATRIX_SINGULAR;
		   return;
	        }

		vv[i]=1.0F/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a[i][j];
			for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0F;
		for (i=j;i<n;i++) {
			sum=a[i][j];
			for (k=0;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabsf(sum)) >= big) {
				big=dum;
				hor_imax=i;
			}
		}
		if (j != hor_imax) {
			for (k=0;k<n;k++) {
				dum=a[hor_imax][k];
				a[hor_imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[hor_imax]=vv[j];
		}
		indx[j]=hor_imax;
		if (a[j][j] == 0.0F) a[j][j]=TINYF;
		if (j != n-1) {
			dum=1.0F/(a[j][j]);
			for (i=j+1;i<n;i++) a[i][j] *= dum;
		}
	}
}

void hor_lubksbf ( float **a, int n, int *indx, float *b )
{
	int i,ii=-1,ip,j;
	float sum;

	for (i=0;i<n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii>=0)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=b[i];
		for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}

double hor_gammln ( double xx )
{
   double x,tmp,ser;
   static double cof[6]={76.18009173,-86.50532033,24.01409822,
		-1.231739516,0.120858003e-2,-0.536382e-5};
   int j;

   x=xx-1.0;
   tmp=x+5.5;
   tmp -= (x+0.5)*log(tmp);
   ser=1.0;
   for (j=0;j<=5;j++)
   {
      x += 1.0;
      ser += cof[j]/x;
   }

   return -tmp+log(2.50662827465*ser);
}

void hor_gser ( double *gamser, double a, double x, double *gln )
{
   int n;
   double sum,del,ap;

   *gln=hor_gammln(a);
   if (x <= 0.0)
   {
      if (x < 0.0)
	 hor_error ( "x less than 0 (hor_gser)", HOR_FATAL );

      *gamser=0.0;
      return;
   }
   else
   {
      ap=a;
      del=sum=1.0/a;
      for (n=1;n<=ITMAX;n++)
      {
	 ap += 1.0;
	 del *= x/ap;
	 sum += del;
	 if (fabs(del) < fabs(sum)*EPS)
	 {
	    *gamser=sum*exp(-x+a*log(x)-(*gln));
	    return;
	 }
      }

      hor_error ( "a too large, ITMAX too small (hor_gser)", HOR_FATAL );
      return;
   }
}

void hor_gcf ( double *gammcf, double a, double x, double *gln )
{
   int n;
   double gold=0.0,g,fac=1.0,b1=1.0;
   double b0=0.0,anf,ana,an,a1,a0=1.0;

   *gln=hor_gammln(a);
   a1=x;
   for (n=1;n<=ITMAX;n++)
   {
      an=(double) n;
      ana=an-a;
      a0=(a1+a0*ana)*fac;
      b0=(b1+b0*ana)*fac;
      anf=an*fac;
      a1=x*a0+anf*a1;
      b1=x*b0+anf*b1;
      if (a1!=0.0)
      {
	 fac=1.0/a1;
	 g=b1*fac;
	 if (fabs((g-gold)/g) < EPS)
	 {
	    *gammcf=exp(-x+a*log(x)-(*gln))*g;
	    return;
	 }

	 gold=g;
      }
   }

   hor_error ( "a too large, ITMAX too small (hor_gcf)", HOR_FATAL );
}

double hor_gammp ( double a, double x )
{
   double gamser,gammcf,gln;

   if (x >= 0.0 && a > 0.0)
      if (x < (a+1.0))
      {
	 hor_gser(&gamser,a,x,&gln);
	 return gamser;
      }
      else
      {
	 hor_gcf(&gammcf,a,x,&gln);
	 return 1.0-gammcf;
      }

   hor_error ( "invalid arguments (hor_gammp) a=%f x=%f", HOR_FATAL, a, x );
   return -1.0;
}

double hor_gammq ( double a, double x )
{
   double gamser,gammcf,gln;

   if (x >= 0.0 && a > 0.0)
      if (x < (a+1.0))
      {
	 hor_gser(&gamser,a,x,&gln);
	 return 1.0-gamser;
      }
      else
      {
	 hor_gcf(&gammcf,a,x,&gln);
	 return gammcf;
      }

   hor_error ( "invalid arguments (hor_gammq) a=%f x=%f", HOR_FATAL, a, x );
   return -1.0;
}

double hor_erfc ( double x )
{
   return x < 0.0 ? 1.0+hor_gammp(0.5,x*x) : hor_gammq(0.5,x*x);
}

void hor_tred2 ( double **a, int n, double d[], double e[] )
{
   int l,k,j,i;
   double scale,hh,h,g,f;

   for (i=n-1;i>0;i--) {
      l=i-1;
      h=scale=0.0;
      if (l > 0) {
	 for (k=0;k<=l;k++)
	    scale += fabs(a[i][k]);
	 if (scale == 0.0)
	    e[i]=a[i][l];
	 else {
	    for (k=0;k<=l;k++) {
	       a[i][k] /= scale;
	       h += a[i][k]*a[i][k];
	    }
	    f=a[i][l];
	    g = f>0 ? -sqrt(h) : sqrt(h);
	    e[i]=scale*g;
	    h -= f*g;
	    a[i][l]=f-g;
	    f=0.0;
	    for (j=0;j<=l;j++) {
	       /* Next statement can be omitted if eigenvectors not wanted */
	       a[j][i]=a[i][j]/h;
	       g=0.0;
	       for (k=0;k<=j;k++)
		  g += a[j][k]*a[i][k];
	       for (k=j+1;k<=l;k++)
		  g += a[k][j]*a[i][k];
	       e[j]=g/h;
	       f += e[j]*a[i][j];
	    }
	    hh=f/(h+h);
	    for (j=0;j<=l;j++) {
	       f=a[i][j];
	       e[j]=g=e[j]-hh*f;
	       for (k=0;k<=j;k++)
		  a[j][k] -= (f*e[k]+g*a[i][k]);
	    }
	 }
      } else
	 e[i]=a[i][l];
      d[i]=h;
   }
   /* Next statement can be omitted if eigenvectors not wanted */
   d[0]=0.0;
   e[0]=0.0;
   /* Contents of this loop can be omitted if eigenvectors not
      wanted except for statement d[i]=a[i][i]; */
   for (i=0;i<n;i++) {
      l=i-1;
      if (d[i]) {
	    for (j=0;j<=l;j++) {
	       g=0.0;
	       for (k=0;k<=l;k++)
		  g += a[i][k]*a[k][j];
	       for (k=0;k<=l;k++)
		  a[k][j] -= g*a[k][i];
	    }
	 }
      d[i]=a[i][i];
      a[i][i]=1.0;
      for (j=0;j<=l;j++) a[j][i]=a[i][j]=0.0;
   }
}

#define SIGN_NR(a,b) ((b)<0 ? -fabs(a) : fabs(a))

void hor_tqli ( double d[], double e[], int n, double **z )
{
   int m,l,iter,i,k;
   double s,r,p,g,f,dd,c,b;

   for (i=1;i<n;i++) e[i-1]=e[i];
   e[n-1]=0.0;
   for (l=0;l<n;l++) {
      iter=0;
      do {
	 for (m=l;m<n-1;m++) {
	       dd=fabs(d[m])+fabs(d[m+1]);
	       if (fabs(e[m])+dd == dd) break;
	    }
	 if (m != l) {
	    if (iter++ == 30)
	       hor_error("too many iterations (hor_tqli)", HOR_FATAL);

	    g=(d[l+1]-d[l])/(2.0*e[l]);
	    r=sqrt((g*g)+1.0);
	    g=d[m]-d[l]+e[l]/(g+SIGN_NR(r,g));
	    s=c=1.0;
	    p=0.0;
	    for (i=m-1;i>=l;i--) {
	       f=s*e[i];
	       b=c*e[i];
	       if (fabs(f) >= fabs(g)) {
		  c=g/f;
		  r=sqrt((c*c)+1.0);
		  e[i+1]=f*r;
		  c *= (s=1.0/r);
	       } else {
		  s=f/g;
		  r=sqrt((s*s)+1.0);
		  e[i+1]=g*r;
		  s *= (c=1.0/r);
	       }
	       g=d[i+1]-p;
	       r=(d[i]-g)*s+2.0*c*b;
	       p=s*r;
	       d[i+1]=g+p;
	       g=c*r-b;
	       /* Next loop can be omitted if eigenvectors not wanted */
	       for (k=0;k<n;k++) {
		  f=z[k][i+1];
		  z[k][i+1]=s*z[k][i]+c*f;
		  z[k][i]=c*z[k][i]-s*f;
	       }
	    }
	    d[l]=d[l]-p;
	    e[l]=g;
	    e[m]=0.0;
	 }
      } while (m != l);
   }
}

void hor_eigsrt ( double *d, double **v, int n )
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
	 for (j=0;j<n;j++) {
	    p=v[j][i];
	    v[j][i]=v[j][k];
	    v[j][k]=p;
	 }
      }
   }
}

static double at,bt,ct;
#define PYTHAG(a,b) ((at=fabs(a)) > (bt=fabs(b)) ? \
(ct=bt/at,at*sqrt(1.0+ct*ct)) : (bt ? (ct=at/bt,bt*sqrt(1.0+ct*ct)): 0.0))

static double maxarg1,maxarg2;
#define MAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ?\
	(maxarg1) : (maxarg2))

Hor_Bool hor_svdcmp ( double **a, int m, int n, double *w, double **v )
{
   int flag,i,its,j,jj,k,l=0,nm=0;
   double c,f,h,s,x,y,z;
   double anorm=0.0,g=0.0,scale=0.0;
   double *rv1;

   rv1 = hor_malloc_ntype ( double, n );
   if (rv1 == NULL)
   { hor_errno = HOR_MATH_ALLOCATION_FAILED; return HOR_FALSE; }

   for (i=0;i<n;i++) {
      l=i+1;
      rv1[i]=scale*g;
      g=s=scale=0.0;
      if (i < m) {
	 for (k=i;k<m;k++) scale += fabs(a[k][i]);
	 if (scale) {
	    for (k=i;k<m;k++) {
	       a[k][i] /= scale;
	       s += a[k][i]*a[k][i];
	    }
	    f=a[i][i];
	    g = -SIGN_NR(sqrt(s),f);
	    h=f*g-s;
	    a[i][i]=f-g;
	    if (i != n-1) {
	       for (j=l;j<n;j++) {
		  for (s=0.0,k=i;k<m;k++) s += a[k][i]*a[k][j];
		  f=s/h;
		  for (k=i;k<m;k++) a[k][j] += f*a[k][i];
	       }
	    }
	    for (k=i;k<m;k++) a[k][i] *= scale;
	 }
      }
      w[i]=scale*g;
      g=s=scale=0.0;
      if (i < m && i != n-1) {
	 for (k=l;k<n;k++) scale += fabs(a[i][k]);
	 if (scale) {
	    for (k=l;k<n;k++) {
	       a[i][k] /= scale;
	       s += a[i][k]*a[i][k];
	    }
	    f=a[i][l];
	    g = -SIGN_NR(sqrt(s),f);
	    h=f*g-s;
	    a[i][l]=f-g;
	    for (k=l;k<n;k++) rv1[k]=a[i][k]/h;
	    if (i != m-1) {
	       for (j=l;j<m;j++) {
		  for (s=0.0,k=l;k<n;k++) s += a[j][k]*a[i][k];
		  for (k=l;k<n;k++) a[j][k] += s*rv1[k];
	       }
	    }
	    for (k=l;k<n;k++) a[i][k] *= scale;
	 }
      }
      anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
   }
   for (i=n-1;i>=0;i--) {
      if (i < n-1) {
	 if (g) {
	    for (j=l;j<n;j++)
	       v[j][i]=(a[i][j]/a[i][l])/g;
	    for (j=l;j<n;j++) {
	       for (s=0.0,k=l;k<n;k++) s += a[i][k]*v[k][j];
	       for (k=l;k<n;k++) v[k][j] += s*v[k][i];
	    }
	 }
	 for (j=l;j<n;j++) v[i][j]=v[j][i]=0.0;
      }
      v[i][i]=1.0;
      g=rv1[i];
      l=i;
   }
   for (i=n-1;i>=0;i--) {
      l=i+1;
      g=w[i];
      if (i < n-1)
	 for (j=l;j<n;j++) a[i][j]=0.0;
      if (g) {
	 g=1.0/g;
	 if (i != n-1) {
	    for (j=l;j<n;j++) {
	       for (s=0.0,k=l;k<m;k++) s += a[k][i]*a[k][j];
	       f=(s/a[i][i])*g;
	       for (k=i;k<m;k++) a[k][j] += f*a[k][i];
	    }
	 }
	 for (j=i;j<m;j++) a[j][i] *= g;
      } else {
	 for (j=i;j<m;j++) a[j][i]=0.0;
      }
      ++a[i][i];
   }
   for (k=n-1;k>=0;k--) {
      for (its=1;its<=30;its++) {
	 flag=1;
	 for (l=k;l>=0;l--) {
	    nm=l-1;
	    if (fabs(rv1[l])+anorm == anorm) {
	       flag=0;
	       break;
	    }
	    if (fabs(w[nm])+anorm == anorm) break;
	 }
	 if (flag) {
	    c=0.0;
	    s=1.0;
	    for (i=l;i<=k;i++) {
	       f=s*rv1[i];
	       if (fabs(f)+anorm != anorm) {
		  g=w[i];
		  h=PYTHAG(f,g);
		  w[i]=h;
		  h=1.0/h;
		  c=g*h;
		  s=(-f*h);
		  for (j=0;j<m;j++) {
		     y=a[j][nm];
		     z=a[j][i];
		     a[j][nm]=y*c+z*s;
		     a[j][i]=z*c-y*s;
		  }
	       }
	    }
	 }
	 z=w[k];
	 if (l == k) {
	    if (z < 0.0) {
	       w[k] = -z;
	       for (j=0;j<n;j++) v[j][k]=(-v[j][k]);
	    }
	    break;
	 }
	 if (its == 30) { hor_errno = HOR_MATH_NO_CONVERGENCE; return HOR_FALSE; }
	 x=w[l];
	 nm=k-1;
	 y=w[nm];
	 g=rv1[nm];
	 h=rv1[k];
	 f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
	 g=PYTHAG(f,1.0);
	 f=((x-z)*(x+z)+h*((y/(f+SIGN_NR(g,f)))-h))/x;
	 c=s=1.0;
	 for (j=l;j<=nm;j++) {
	    i=j+1;
	    g=rv1[i];
	    y=w[i];
	    h=s*g;
	    g=c*g;
	    z=PYTHAG(f,h);
	    rv1[j]=z;
	    c=f/z;
	    s=h/z;
	    f=x*c+g*s;
	    g=g*c-x*s;
	    h=y*s;
	    y=y*c;
	    for (jj=0;jj<n;jj++) {
	       x=v[jj][j];
	       z=v[jj][i];
	       v[jj][j]=x*c+z*s;
	       v[jj][i]=z*c-x*s;
	    }
	    z=PYTHAG(f,h);
	    w[j]=z;
	    if (z) {
	       z=1.0/z;
	       c=f*z;
	       s=h*z;
	    }
	    f=(c*g)+(s*y);
	    x=(c*y)-(s*g);
	    for (jj=0;jj<m;jj++) {
	       y=a[jj][j];
	       z=a[jj][i];
	       a[jj][j]=y*c+z*s;
	       a[jj][i]=z*c-y*s;
	    }
	 }
	 rv1[l]=0.0;
	 rv1[k]=f;
	 w[k]=x;
      }
   }
   hor_free ( (void *) rv1 );
   return HOR_TRUE;
}

#undef SIGN_NR
#undef MAX
#undef PYTHAG

#define SWAP(a,b) {float temp=(a);(a)=(b);(b)=temp;}

Hor_Bool hor_gaussj ( double **a, int n, double **b, int m )
{
   int   *indxc,*indxr,*ipiv;
   int    i,icol=0,irow=0,j,k,l,ll;
   double big,dum,pivinv;

   indxc = hor_malloc_ntype ( int, n );
   indxr = hor_malloc_ntype ( int, n );
   ipiv  = hor_malloc_ntype ( int, n );
   for (j=0;j<n;j++) ipiv[j]=0;
   for (i=0;i<n;i++) {
      big=0.0;
      for (j=0;j<n;j++)
	 if (ipiv[j] != 1)
	    for (k=0;k<n;k++) {
	       if (ipiv[k] == 0) {
		  if (fabs(a[j][k]) >= big) {
		     big=fabs(a[j][k]);
		     irow=j;
		     icol=k;
		  }
	       } else if (ipiv[k] > 1) {
		    hor_free_multiple ( (void *) ipiv, (void *) indxr,
				        (void *) indxc, NULL );
		    hor_errno = HOR_MATH_MATRIX_SINGULAR;
		    return HOR_FALSE;
		 }
	    }
      ++(ipiv[icol]);
      if (irow != icol) {
	 for (l=0;l<n;l++) SWAP(a[irow][l],a[icol][l])
	    for (l=0;l<m;l++) SWAP(b[irow][l],b[icol][l])
      }
      indxr[i]=irow;
      indxc[i]=icol;
      if (a[icol][icol] == 0.0) { hor_errno = HOR_MATH_MATRIX_SINGULAR;
				  return HOR_FALSE; }
      pivinv=1.0/a[icol][icol];
      a[icol][icol]=1.0;
      for (l=0;l<n;l++) a[icol][l] *= pivinv;
      for (l=0;l<m;l++) b[icol][l] *= pivinv;
      for (ll=0;ll<n;ll++)
	 if (ll != icol) {
	    dum=a[ll][icol];
	    a[ll][icol]=0.0;
	    for (l=0;l<n;l++) a[ll][l] -= a[icol][l]*dum;
	    for (l=0;l<m;l++) b[ll][l] -= b[icol][l]*dum;
	 }
   }
   for (l=n-1;l>=0;l--) {
      if (indxr[l] != indxc[l])
	 for (k=0;k<n;k++)
	    SWAP(a[k][indxr[l]],a[k][indxc[l]]);
   }

   hor_free_multiple ( (void *) ipiv, (void *) indxr, (void *) indxc, NULL );
   return HOR_TRUE;
}

#undef SWAP

void hor_mrqcof ( double x[], double y[], double sig[], int ndata,
		  double a[], int ma, int lista[], int mfit,
		  double **alpha, double beta[], double *chisq,
		  void (*funcs)(double, double *, double *, double *, int) )
{
   int k,j,i;
   double ymod,wt,sig2i,dy,*dyda;

   dyda = hor_malloc_ntype ( double, ma );
   for (j=0;j<mfit;j++) {
      for (k=0;k<=j;k++) alpha[j][k]=0.0;
      beta[j]=0.0;
   }
   *chisq=0.0;
   for (i=0;i<ndata;i++) {
      (*funcs)(x[i],a,&ymod,dyda,ma);
      sig2i=1.0/(sig[i]*sig[i]);
      dy=y[i]-ymod;
      for (j=0;j<mfit;j++) {
	 wt=dyda[lista[j]]*sig2i;
	 for (k=0;k<=j;k++)
	    alpha[j][k] += wt*dyda[lista[k]];
	 beta[j] += dy*wt;
      }
      (*chisq) += dy*dy*sig2i;
   }
   for (j=1;j<mfit;j++)
      for (k=0;k<=j-1;k++) alpha[k][j]=alpha[j][k];
   hor_free ( (void *) dyda );
}

void hor_covsrt ( double **covar, int ma, int lista[], int mfit )
{
   int i,j;
   float swap;

   for (j=0;j<ma-1;j++)
      for (i=j+1;i<ma;i++) covar[i][j]=0.0;
   for (i=0;i<mfit-1;i++)
      for (j=i+1;j<mfit;j++) {
	 if (lista[j] > lista[i])
	    covar[lista[j]][lista[i]]=covar[i][j];
	 else
	    covar[lista[i]][lista[j]]=covar[i][j];
      }
   swap=covar[0][0];
   for (j=0;j<ma;j++) {
      covar[0][j]=covar[j][j];
      covar[j][j]=0.0;
   }
   covar[lista[0]][lista[0]]=swap;
   for (j=1;j<mfit;j++) covar[lista[j]][lista[j]]=covar[0][j];
   for (j=1;j<ma;j++)
      for (i=0;i<=j-1;i++) covar[i][j]=covar[j][i];
}

Hor_Bool hor_mrqmin ( double x[], double y[], double sig[], int ndata,
		      double a[], int ma, int lista[], int mfit,
		      double **covar, double **alpha, double *chisq,
		      void (*funcs)(double, double *, double *, double *, int),
		      double *alamda )
{
   int k,kk,j,ihit;
   static double *da,*atry,**oneda,*beta,ochisq;
   static Hor_Matrix *M;

   if (*alamda < 0.0) {
      M = hor_mat_alloc ( mfit, 1 );
      oneda = M->m;
      atry  = hor_malloc_ntype(double,ma);
      da    = hor_malloc_ntype(double,ma);
      beta  = hor_malloc_ntype(double,ma);
      kk=mfit;
      for (j=0;j<ma;j++) {
	 ihit=0;
	 for (k=0;k<mfit;k++)
	    if (lista[k] == j) ihit++;
	 if (ihit == 0)
	    lista[kk++]=j;
	 else if (ihit > 1) { hor_errno = HOR_MATH_NUMREC_BAD_PERMUTATION;
			      hor_free_multiple ( (void *) beta, (void *) da,
						  (void *) atry, NULL );
			      hor_mat_free ( M );
			      return HOR_FALSE; }
      }
      if (kk != ma) { hor_errno = HOR_MATH_NUMREC_BAD_PERMUTATION;
		      hor_free_multiple ( (void *) beta, (void *) da,
					  (void *) atry, NULL );
		      hor_mat_free ( M );
		      return HOR_FALSE; }
      *alamda=0.001;
      hor_mrqcof(x,y,sig,ndata,a,ma,lista,mfit,alpha,beta,chisq,funcs);
      ochisq=(*chisq);
   }
   for (j=0;j<mfit;j++) {
      for (k=0;k<mfit;k++) covar[j][k]=alpha[j][k];
      covar[j][j]=alpha[j][j]*(1.0+(*alamda));
      oneda[j][0]=beta[j];
   }
   if ( !hor_gaussj(covar,mfit,oneda,1) ) {
      hor_free_multiple ( (void *) beta, (void *) da, (void *) atry, NULL );
      hor_mat_free ( M );
      return HOR_FALSE;
   }

   for (j=0;j<mfit;j++)
      da[j]=oneda[j][0];
   if (*alamda == 0.0) {
      hor_covsrt(covar,ma,lista,mfit);
      hor_free_multiple ( (void *) beta, (void *) da, (void *) atry, NULL );
      hor_mat_free ( M );
      return HOR_TRUE;
   }
   for (j=0;j<ma;j++) atry[j]=a[j];
   for (j=0;j<mfit;j++)
      atry[lista[j]] = a[lista[j]]+da[j];
   hor_mrqcof(x,y,sig,ndata,atry,ma,lista,mfit,covar,da,chisq,funcs);
   if (*chisq < ochisq) {
      *alamda *= 0.1;
      ochisq=(*chisq);
      for (j=0;j<mfit;j++) {
	 for (k=0;k<mfit;k++) alpha[j][k]=covar[j][k];
	 beta[j]=da[j];
	 a[lista[j]]=atry[lista[j]];
      }
   } else {
      *alamda *= 10.0;
      *chisq=ochisq;
   }
   return HOR_TRUE;
}

void hor_svbksb ( double **u, double *w, double **v,
		  int m, int n, double **b, double **x )
{
   int    jj, j, i;
   double s, *tmp;
  
   tmp = hor_malloc_ntype (double, n);
   for (j=0; j<n; j++) {
      s = 0.0;
      if (w[j]) {
	 for (i=0; i<m; i++) 
	    s += u[i][j]*b[i][0];
	 s /= w[j];
      }
      tmp[j] = s;
   }
   for (j=0; j<n; j++) {
      s = 0.0;
      for (jj=0; jj<n; jj++) 
	 s += v[j][jj]*tmp[jj];
      x[j][0] = s;
   }
   hor_free ((void *) tmp);
}
