/*  SceneApp: applications for sequential localisation and map-building
    Copyright (C) 2000 Andrew Davison
    ajd@robots.ox.ac.uk
    Scene home page: http://www.robots.ox.ac.uk/~ajd/Scene/

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/****************************Display type defines*****************************/

/* NTSC */
#define MAXROWS 480
#define MAXCOLS 640
#define DEFAULT_FORMAT METEOR_FMT_NTSC
#define MAXFPS 30

#define BLACK 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define WHITE 255
#define MAXPIXELSIZE 4

/* Do we want to use unwarping? 
   This is for unusual lenses only (like at ETL)
   Set this to zero for normal lenses! */
const int UNWARP_FLAG = 1;

/******************************System parameters******************************/

enum{LEFT, RIGHT};                    /* Identifiers for the two cameras */
enum{CLOSED, OPEN};


/**********************Functions to be called from outside********************/

int g_display_images();
int copy_images();
int g_grab_images();
int g_write_images();
int g_read_images();
int init_grabbing(FD_formrob *fd_fr, Hor_Image **image, int *uLp, int *vLp, 
		  int *uRp, int *vRp, int BS);
int close_grabbing();
void flush_xevents(void);
int drawlineL(int x1, int y1, int x2, int y2, unsigned long colour);
int drawboxL(int x1, int y1, int width, int height, unsigned long colour);
int drawlineR(int x1, int y1, int x2, int y2, unsigned long colour);
int drawboxR(int x1, int y1, int width, int height, unsigned long colour);

/*****************************************************************************/
