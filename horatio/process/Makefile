# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
NAME = process
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
OFILES =       initproc.o process.o gb_proc.o ed_proc.o ca_proc.o \
	       li_proc.o lf_proc.o ls_proc.o lm_proc.o blm_proc.o \
	       co_proc.o pc_proc.o sc_proc.o cow_proc.o \
	       cm_proc.o bcm_proc.o dcm_proc.o fl_proc.o is_proc.o cl_proc.o
HFILES = def.h initproc.h process.h gb_proc.h ed_proc.h ca_proc.h \
	       li_proc.h lf_proc.h ls_proc.h lm_proc.h blm_proc.h \
	       co_proc.h pc_proc.h sc_proc.h cow_proc.h \
	       cm_proc.h bcm_proc.h dcm_proc.h fl_proc.h is_proc.h cl_proc.h

#---make rules---
.c.o:
	$(CC) $(CFLAGS) -c $<

$(LIB): $(OFILES)
	ar $(ARFLAGS) $(LIB) $(OFILES)
	ranlib $@

$(OFILES): $(INCLUDE) $(HFILEDIR)/improc.h $(HFILEDIR)/graphics.h \
		      $(HFILEDIR)/image.h  $(HFILEDIR)/list.h \
		      $(HFILEDIR)/math.h   $(HFILEDIR)/global.h

#---library include file rules---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)

#---clean directory rule---
clean:
	rm -f *.o *~
