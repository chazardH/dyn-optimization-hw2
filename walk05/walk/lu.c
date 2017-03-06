/*
http://rainman.astro.uiuc.edu/codelib/codes/inflow/src/nrutil.c
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define NR_END 1
#define FREE_ARG char*

void nrerror(error_text)
     char error_text[];
/* Numerical Recipes standard error handler */
{
  void exit();

  fprintf(stderr,"Numerical Recipes run-time error...\n");
  fprintf(stderr,"%s\n",error_text);
  fprintf(stderr,"...now exiting to system...\n");
  exit(1);
}

float *vector(nl,nh)
     long nh,nl;
     /* allocate a float vector with subscript range v[nl..nh] */
{
  float *v;

  v=(float *)malloc((unsigned int) ((nh-nl+1+NR_END)*sizeof(float)));
  if (!v) nrerror("allocation failure in vector()");
  return v-nl+NR_END;
}

float **matrix(nrl,nrh,ncl,nch)
     long nch,ncl,nrh,nrl;
     /* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  float **m;

  /* allocate pointers to rows */
  m=(float **) malloc((unsigned int)((nrow+NR_END)*sizeof(float*)));
  if (!m) nrerror("allocation failure 1 in matrix()");
  m += NR_END;
  m -= nrl;

  /* allocate rows and set pointers to them */
  m[nrl]=(float *) malloc((unsigned int)((nrow*ncol+NR_END)*sizeof(float)));
  if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  /* return pointer to array of pointers to rows */
  return m;
}

void free_vector(v,nl,nh)
     float *v;
     long nh,nl;
     /* free a float vector allocated with vector() */
{
  free((FREE_ARG) (v+nl-NR_END));
}

/*
http://rainman.astro.uiuc.edu/codelib/codes/inflow/src/ludcmp.c
*/

#define TINY 1.0e-20;

void ludcmp(a,n,indx,d)
     float **a,*d;
     int *indx,n;
{
  int i,imax,j,k;
  float big,dum,sum,temp;
  float *vv;

  vv=vector(1,n);
  *d=1.0;
  for (i=1;i<=n;i++) {
    big=0.0;
    for (j=1;j<=n;j++)
      if ((temp=fabs(a[i][j])) > big) big=temp;
    if (big == 0.0) nrerror("Singular matrix in routine ludcmp");
    vv[i]=1.0/big;
  }
  for (j=1;j<=n;j++) {
    for (i=1;i<j;i++) {
      sum=a[i][j];
      for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
    }
    big=0.0;
    for (i=j;i<=n;i++) {
      sum=a[i][j];
      for (k=1;k<j;k++)
	sum -= a[i][k]*a[k][j];
      a[i][j]=sum;
      if ( (dum=vv[i]*fabs(sum)) >= big) {
	big=dum;
	imax=i;
      }
    }
    if (j != imax) {
      for (k=1;k<=n;k++) {
	dum=a[imax][k];
	a[imax][k]=a[j][k];
	a[j][k]=dum;
      }
      *d = -(*d);
      vv[imax]=vv[j];
    }
    indx[j]=imax;
    if (a[j][j] == 0.0) a[j][j]=TINY;
    if (j != n) {
      dum=1.0/(a[j][j]);
      for (i=j+1;i<=n;i++) a[i][j] *= dum;
    }
  }
  free_vector(vv,1,n);
}
#undef TINY

/*
http://rainman.astro.uiuc.edu/codelib/codes/inflow/src/lubksb.c
*/

void lubksb(a,n,indx,b)
     float **a,b[];
int *indx,n;
{
  int i,ii=0,ip,j;
  float sum;

  for (i=1;i<=n;i++) {
    ip=indx[i];
    sum=b[ip];
    b[ip]=b[i];
    if (ii)
      for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
    else if (sum) ii=i;
    b[i]=sum;
  }
  for (i=n;i>=1;i--) {
    sum=b[i];
    for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
    b[i]=sum/a[i][i];
  }
}
