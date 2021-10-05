# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
NAME = improc
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/$(NAME).h

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

#---compilation options---
CC = gcc
CFLAGS = -I$(INCDIR) -g -O -Wall
ARFLAGS = r

#---files---
LIB = ../lib-$(ARCH)/libhoratio.a
OFILES = adjust.o convolve.o gaussian.o traject.o tr_disp.o edge.o edg_file.o \
	 canny.o line.o li_disp.o line_fit.o line_seg.o li_match.o bog_lm.o \
	 corner.o co_disp.o cor_file.o plessey.o smith.o wang.o feature.o \
	 co_match.o bog_cm.o bd_cm.o flow.o fl_disp.o segment.o correl.o
HFILES = def.h \
	 adjust.h convolve.h gaussian.h traject.h tr_disp.h edge.h edg_file.h \
	 canny.h line.h li_disp.h line_fit.h line_seg.h li_match.h bog_lm.h \
	 corner.h co_disp.h cor_file.h plessey.h smith.h wang.h feature.h \
	 co_match.h bog_cm.h bd_cm.h flow.h fl_disp.h segment.h correl.h

#---make rules---
.c.o:
	$(CC) $(CFLAGS) -c $<

$(LIB): $(OFILES)
	ar r $(LIB) $(OFILES)
	ranlib $@

$(OFILES): $(INCLUDE) $(HFILEDIR)/image.h $(HFILEDIR)/list.h \
		      $(HFILEDIR)/math.h  $(HFILEDIR)/global.h \
		      $(HFILEDIR)/graphics.h

fmprocs.o fmfmatx.o fmutils.o: fmatx.h

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)

#---clean directory rule---
clean:
	rm -f *.o *~
