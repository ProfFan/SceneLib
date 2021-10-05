/*
 * Horatio routines for postcript graphics 
 * Ian Reid, April 1995
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "horatio/global.h"
#include "horatio/image.h"


/*********************
*   FILE *@hor_ps_open(char *filename)
*
*   Open a stream for postscript output
**********************/
FILE *hor_ps_open(char *filename)
{
    FILE *fp;

    if ((fp=fopen(filename, "w")) == NULL) {
	hor_error("Couldn't open postscript output file", HOR_NON_FATAL);
	return NULL;
    }
    return fp;
}


/*********************
*   Hor_Bool @hor_ps_close(FILE *fp)
*
*   Close a postscript stream after ensuring that the ps graphics state
*   is restored.rite_image
**********************/
Hor_Bool hor_ps_close(FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }
    fprintf(fp, "grestore\n");
    fprintf(fp, "%%%%END POSTSCRIPT OVERLAY\n");
    fclose(fp);
    return HOR_TRUE;
}


/*********************
*   Hor_Bool @hor_ps_write_image(Hor_Image *im, FILE *fp)
*
*   Write an image in postscript including a bounding box to an open
*   stream fp.
**********************/
Hor_Bool hor_ps_write_image(Hor_Image *im, FILE *fp)
{
    int i, j;

    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    if (im->type != HOR_U_CHAR) {
	hor_error("Illegal image type (not uchar) in hor_ps_write_image", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    fprintf(fp, "%%!PS-Adobe-2.0 EPSF-1.2\n");
    fprintf(fp, "%%%%BoundingBox: %d %d %d %d\n", 0, 0, im->width, im->height);
    fprintf(fp, "gsave\n");

    fprintf(fp, "/imline %d string def\n", 2*im->width);
    fprintf(fp, "/drawimage {\n");
    fprintf(fp, "    %d %d 8\n", im->width, im->height);
    fprintf(fp, "    [%d 0 0 %d 0 %d]\n", im->width, -im->height, im->height);
    fprintf(fp, "    { currentfile imline readhexstring pop } image\n");
    fprintf(fp, "} def\n");
    fprintf(fp, "%d %d scale\n", im->width, im->height);
    fprintf(fp, "drawimage\n");
    for (i=0; i<im->height; i++) {
	for (j=0; j<im->width; j++)
	    fprintf(fp, "%02X", im->array.uc[i][j]);
	fprintf(fp, "\n");
    }
    fprintf(fp, "grestore\n");

    return HOR_TRUE;
}


/*********************
*   Hor_Bool @hor_write_ps_image(const char *file_name, Hor_Image *im, ...)
*
**********************/
Hor_Bool hor_write_ps_image(const char *file_name, Hor_Image *imptr, ...)
{
    int      fd = -1;
    Hor_Bool result;

   if ( imptr->type != HOR_U_CHAR ) {
       hor_errno = HOR_IMAGE_WRONG_TYPE_IMAGE;
       return HOR_FALSE;
   }

#ifdef HOR_TRANSPUTER
   fd = open ( (char *) file_name, O_BINARY | O_WRONLY );
#else
#ifdef HOR_MSDOS
   fd = _open ( (char *) file_name, _O_BINARY | _O_CREAT |_O_WRONLY,
   		_S_IREAD | _S_IWRITE );
#else
   switch ( hor_get_image_compress() )
   {
      case HOR_NO_COMPRESS:
      fd = open ( (char *) file_name, O_CREAT | O_WRONLY, 0644 );
      break;

      case HOR_UNIX_COMPRESS:
      fd = hor_compress ( file_name );
      break;

      case HOR_GNU_COMPRESS:
      fd = hor_gzip ( file_name );
      break;

      default:
      hor_error ( "illegal compression mode (hor_write_pgm_image)", HOR_FATAL);
   }
#endif
#endif
   if ( fd == -1 ) {
       hor_errno = HOR_IMAGE_OPEN_FAILED_FOR_WRITE;
       return HOR_FALSE;
   }

   result = hor_write_ps_image_stream ( fd, imptr );

#ifdef HOR_MSDOS
   _close ( fd );
#else
   close ( fd );
#endif
   return result;
}


/*********************
*   Hor_Bool @hor_write_ps_image_stream(int fd, Hor_Image *im, ...)
*
*   Write an image in postscript including a bounding box to an open
*   file descriptor fd.
**********************/
Hor_Bool hor_write_ps_image_stream(int fd, Hor_Image *im, ...)
{
    int i, j;
    FILE *fp = fdopen(dup(fd), "w");

    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    if (im->type != HOR_U_CHAR) {
	hor_error("Illegal image type (not uchar) in hor_ps_write_image", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    fprintf(fp, "%%!PS-Adobe-2.0 EPSF-1.2\n");
    fprintf(fp, "%%%%BoundingBox: %d %d %d %d\n", 0, 0, im->width, im->height);
    fprintf(fp, "gsave\n");

    fprintf(fp, "/imline %d string def\n", 2*im->width);
    fprintf(fp, "/drawimage {\n");
    fprintf(fp, "    %d %d 8\n", im->width, im->height);
    fprintf(fp, "    [%d 0 0 %d 0 %d]\n", im->width, -im->height, im->height);
    fprintf(fp, "    { currentfile imline readhexstring pop } image\n");
    fprintf(fp, "} def\n");
    fprintf(fp, "%d %d scale\n", im->width, im->height);
    fprintf(fp, "drawimage\n");
    for (i=0; i<im->height; i++) {
	for (j=0; j<im->width; j++)
	    fprintf(fp, "%02X", im->array.uc[i][j]);
	fprintf(fp, "\n");
    }
    fprintf(fp, "grestore\n");

    fclose(fp);

    return HOR_TRUE;
}


/*********************
*   Hor_Bool @hor_ps_init(int width, int height, FILE *fp)
*
*   Initialize default scaling and grey value and linewidth.  The width
*   and height should match the image previously written in order to get
*   overlays.  This routine will typically be called after hor_ps_open 
*   and hor_ps_write_image
**********************/
Hor_Bool hor_ps_init(int width, int height, FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    fprintf(fp, "%%!PS-Adobe-2.0 EPSF-1.2\n");
    fprintf(fp, "%%%%BoundingBox: %d %d %d %d\n", 0, 0, width, height);
    fprintf(fp, "%%%%START POSTSCRIPT OVERLAY\n");
    fprintf(fp, "gsave\n");
    fprintf(fp, "0 %d translate\n", height);
    fprintf(fp, "1 -1 scale\n");
    fprintf(fp, "1 setlinewidth 1 setgray\n");

    return HOR_TRUE;
}


/*********************
*   Hor_Bool @hor_ps_setlinewidth(double linewidth, FILE *fp)
*
*   Set the postscript linewidth in output stream fp
**********************/
Hor_Bool hor_ps_setlinewidth(double linewidth, FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }
    fprintf(fp, "%f setlinewidth\n", linewidth);
    return HOR_TRUE;
}

/*********************
*   Hor_Bool @hor_ps_setgray(double greyval, FILE *fp)
*
*   Set the postscript grey level (0.0 -- 1.0) in output stream fp
**********************/
Hor_Bool hor_ps_setgray(double greyval, FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }
    if (greyval < 0.0 || greyval >1.0) {
	hor_warning("Postscript grey value (%f) not in range [0.0,1.0]", greyval);
	return HOR_FALSE;
    }
    fprintf(fp, "%f setgray\n", greyval);
    return HOR_TRUE;
}


/*********************
*   Hor_Bool @hor_ps_line(double x1, double y1, double x2, double y2, FILE *fp)
*
*   Draw a line of the current width in the current grey from x1,y1 to x2,y2
*   in image coordinates, to output stream fp
**********************/
Hor_Bool hor_ps_line(double x1, double y1, double x2, double y2, FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }
    fprintf(fp, "newpath %f %f moveto %f %f lineto stroke\n", x1, y1, x2, y2);
    return HOR_TRUE;
}


/*********************
*    Hor_Bool hor_ps_circle (double c, double r, double radius, FILE *fp)
*
*    Draw a circle
**********************/
Hor_Bool hor_ps_circle (double c, double r, double radius, FILE *fp)
{
    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }
    fprintf(fp,"newpath %f %f moveto %f %f %f 0 360 arc stroke\n",
		 c + radius, r, c, r, radius );

    return HOR_TRUE;
}    


/*********************
*    Hor_Bool hor_ps_ellipse (double c, double r,
*		    	      double axis1, double axis2, double angle, 
*			      FILE *fp )
*
*    Draw an ellipse with a bunch of lines.  Almost a direct copy of
*    the routine hor_draw_ellipse_actual_size() in xdisplay.c
**********************/
Hor_Bool hor_ps_ellipse (double c, double r,
			 double axis1, double axis2, double angle, 
			 FILE *fp)
{
#define ELLIPSE_SECTIONS 200
#ifndef PI
#define PI 3.1415927
#endif
    double posc, posr, ca, sa, theta, theta_unit, mct, mst;
    int   section;

    if (fp == NULL) {
	hor_error("Stream has NULL pointer", HOR_NON_FATAL);
	return HOR_FALSE;
    }

    ca = cos(angle);
    sa = sin(angle);
    theta = 0.0;
    mct = axis1*cos(theta);
    mst = axis2*sin(theta);
    posc =   mct*ca + mst*sa;
    posr = - mct*sa + mst*ca;
    theta_unit = 2.0*PI / (double) ELLIPSE_SECTIONS;

    fprintf(fp, "newpath %f %f moveto\n", c+posc, r+posr);

    for ( theta = theta_unit, section = 1; section <= ELLIPSE_SECTIONS;
         theta += theta_unit, section++ ) {
	mct = axis1*cos(theta);
	mst = axis2*sin(theta);
	posc =   mct*ca + mst*sa;
	posr = - mct*sa + mst*ca;
	fprintf(fp, "\t%f %f lineto\n", c+posc, r+posr);
    }
    fprintf(fp, "closepath stroke\n");

    return HOR_TRUE;
}




