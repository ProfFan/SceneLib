# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
NAME = graphics
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/$(NAME).h
HFILES = def.h print.h colour.h display.h state.h

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

ifeq ($(ARCH),sun4u)
	ULTRA_FLAGS = -I/usr/openwin/include
endif

#---compilation options---
CC = gcc
CFLAGS = -I$(INCDIR) $(ULTRA_FLAGS) -g -O -Wall
ARFLAGS = r

#---files---
LIB = ../lib-$(ARCH)/libhoratio.a
OFILES = xprint.o colour.o xdisplay.o ximage.o state.o

#---make rules---
.c.o:
	$(CC) $(CFLAGS) -c $<

$(LIB): $(OFILES)
	ar r $(LIB) $(OFILES)
	ranlib $@

$(OFILES): $(INCLUDE) $(HFILEDIR)/image.h $(HFILEDIR)/list.h \
		      $(HFILEDIR)/math.h  $(HFILEDIR)/global.h

$(LIB)(ximage.o xdisplay.o): ximage.h

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)

#---clean directory rule---
clean:
	rm -f *.o *~
