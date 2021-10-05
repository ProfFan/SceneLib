/* Copyright 1993 Ian Reid (ian@robots.oxford.ac.uk)
                  Robotics Research Group, Oxford University. */
/* included from image/ps_image.h */

#if (!defined(Linux) && defined(FILE) || \
      defined(Linux) && defined(_STDIO_H))

#ifndef PS_IMAGE
#define PS_IMAGE

FILE *hor_ps_open(char *filename);
Hor_Bool hor_ps_close(FILE *fp);

Hor_Bool hor_ps_write_image(Hor_Image *im, FILE *fp);
Hor_Bool hor_write_ps_image_stream(int fd, Hor_Image *im, ...);
Hor_Bool hor_write_ps_image(const char *file_name, Hor_Image *im, ...);
Hor_Bool hor_ps_init(int width, int height, FILE *fp);

Hor_Bool hor_ps_setlinewidth(double linewidth, FILE *fp);
Hor_Bool hor_ps_setgray(double greyval, FILE *fp);

Hor_Bool hor_ps_line(double x1, double y1, double x2, double y2, FILE *fp);
Hor_Bool hor_ps_circle (double c, double r, double radius, FILE *fp);
Hor_Bool hor_ps_ellipse (double c, double r,
			 double axis1, double axis2, double angle, 
			 FILE *fp);

#endif
#endif
