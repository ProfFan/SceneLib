# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/image.h
HFILES = def.h image.h alloc.h misc.h im_comp.h convert.h subsamp.h \
	       image_io.h im_file.h iff_file.h mit_file.h pgm_file.h \
	       ppm_file.h jpeg_file.h gif_file.h sequence.h stream.h \
	       ps_image.h

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

#---compilation options---
CC = gcc
CFLAGS = -I$(INCDIR) -g -Wall -D$(OS) -DHOR_PROVIDE_RGB
ARFLAGS = r

#---files--
LIB = ../lib-$(ARCH)/libhoratio.a
OFILES = alloc.o misc.o im_comp.o convert.o subsamp.o im_file.o \
	 iff_file.o mit_file.o pgm_file.o ppm_file.o jpeg_file.o gif_file.o \
	 sequence.o stream.o ps_image.o

#---make rules---
.c.o:
	$(CC) $(CFLAGS) -c $<

$(LIB): $(OFILES)
	ar $(ARFLAGS) $(LIB) $(OFILES)
	ranlib $@

clean:
	rm -f $(OFILES) *~

#---transputer stuff---

#---compilation options---
TCOFLAGS = -t800 -j $(INCDIR) -DHOR_TRANSPUTER
.SUFFIXES: .tio .trd

#---files---
TRANS_LIB_STDIO = ../lib-T800/image_io.lib
TRANS_LIB_STDRD = ../lib-T800/image_rd.lib
TCOFILES_IO = alloc.tio misc.tio im_comp.tio convert.tio subsamp.tio \
	      image_io.tio im_file.tio iff_file.tio mit_file.tio \
	      sequence.tio stream.tio
TCOFILES_RD = alloc.trd misc.trd im_comp.trd convert.trd subsamp.trd \
	      image_io.trd

#---make rules---
T800: $(TRANS_LIB_STDIO) $(TRANS_LIB_STDRD)

$(TRANS_LIB_STDIO): $(TCOFILES_IO)
	ilibr $(TCOFILES_IO) -o $(TRANS_LIB_STDIO)
	rm -f $(TCOFILES_IO)

$(TRANS_LIB_STDRD): $(TCOFILES_RD)
	ilibr $(TCOFILES_RD) -o $(TRANS_LIB_STDRD)
	rm -f $(TCOFILES_RD)

.c.tio:
	icc $* $(TCOFLAGS) -o $*.tio

.c.trd:
	icc $* $(TCOFLAGS) -DHOR_REDUCED_LIB -o $*.trd

$(OFILES) \
$(TCOFILES_IO) $(TCOFILES_RD): $(INCLUDE)         $(HFILEDIR)/global.h\
			       $(HFILEDIR)/math.h $(HFILEDIR)/list.h

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)
