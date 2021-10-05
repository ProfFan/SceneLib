/* Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk) and
                  Ian Reid (ian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */

/* Functions to create POSTSCRIPT files with simple graphics primitives */

#include <stdio.h>
#include <stdlib.h>

#include "horatio/global.h"
#include "horatio/graphics.h"

/*******************
*   FILE *@ps_fopen ( char *base_name )
*   void  @ps_fclose ( FILE *fp )
*
*   ps_fopen() opens a text file base_name.ps and writes a POSTSCRIPT file
*              start line onto it.
*   ps_fclose() writes a POSTSCRIPT end line onto the given file and closes it.
********************/
FILE *ps_fopen ( char *base_name )
{
   char  file_name[300];
   FILE *fp;

   sprintf ( file_name, "%s.ps", base_name );
   if ( (fp = fopen ( file_name, "w" )) == NULL )
   {
      hor_errno = HOR_OPEN_FAILED_FOR_WRITE;
      return NULL;
   }

   fprintf ( fp, "%%START\n" );
   return fp;
}

void ps_fclose ( FILE *fp )
{
   fprintf ( fp, "%%END\n" );
   fclose ( fp );
}

/*******************
*   void @ps_set_white       ( FILE *fp )
*   void @ps_set_black       ( FILE *fp )
*   void @ps_set_line_width  ( FILE *fp, int line_width )
*   void @ps_set_solid_line  ( FILE *fp )
*   void @ps_set_dotted_line ( FILE *fp, int dot_length, int gap_length )
*
*   Write POSTSCRIPT graphics setup commands onto the given file.
*
*   ps_set_white(): draw in white.
*   ps_set_black(): draw in black.
*   ps_set_line_width(): draw lines with width the given multiple of the
*                        standard width lw.
*   ps_set_solid_line(): draw solid lines.
*   ps_set_dotted_line(): draw dotted lines with given dot and gap size.
********************/
void ps_set_white ( FILE *fp )
{
   fprintf ( fp, "1 setgray\n" );
}

void ps_set_black ( FILE *fp )
{
   fprintf ( fp, "0 setgray\n" );
}

void ps_set_line_width ( FILE *fp, int line_width )
{
   fprintf ( fp, "%d lw setlinewidth\n", line_width );
}

void ps_set_solid_line ( FILE *fp )
{
   fprintf ( fp, "[] 0 setdash\n" );
}

void ps_set_dotted_line ( FILE *fp, int dot_length, int gap_length )
{
   fprintf ( fp, "[%d %d] 0 setdash\n", dot_length, gap_length );
}

/*******************
*   void @ps_draw_line   ( FILE *fp, float x1, float y1, float x2, float y2 )
*   void @ps_draw_circle ( FILE *fp, float x, float y, float radius )
*
*   Write POSTSCRIPT graphics commands onto the given file.
*
*   ps_draw_line() draws a line between between the two given points.
*   ps_draw_circle() draws a circle width given radius centred on given point.
********************/
void ps_draw_line ( FILE *fp, float x1, float y1, float x2, float y2 )
{
   fprintf ( fp, "newpath %f %f moveto %f %f lineto stroke\n",
	         x1, y1, x2, y2 );
}

void ps_draw_circle ( FILE *fp, float x, float y, float radius )
{
   fprintf ( fp, "newpath %f %f moveto %f %f %f 0 360 arc stroke\n",
	         x + radius, y, x, y, radius );
}
