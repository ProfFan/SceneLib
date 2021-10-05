# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/math.h
HFILES = def.h math.h power2.h num_comp.h matrix.h vector.h gauss.h orth_reg.h\
	       chi2.h kalman.h vsdf.h mat_io.h poly.h numrec.h pseudo.h eigen.h

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
OFILES = math.o numrec.o power2.o num_comp.o matrix.o vector.o gauss.o \
	 orth_reg.o chi2.o kalman.o vsdf.o mat_io.o poly.o pseudo.o eigen.o

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
TRANS_LIB_STDIO = ../lib-T800/math_io.lib
TRANS_LIB_STDRD = ../lib-T800/math_rd.lib
TCOFILES_IO = math.tio numrec.tio power2.tio num_comp.tio matrix.tio \
	      vector.tio gauss.tio chi2.tio kalman.tio vsdf.tio mat_io.tio \
	      poly.tio pseudo.tio eigen.tio orth_reg.tio
TCOFILES_RD = math.trd numrec.trd power2.trd num_comp.trd matrix.trd \
	      vector.trd gauss.trd chi2.trd kalman.trd vsdf.trd mat_io.trd \
	      poly.trd pseudo.trd eigen.trd orth_erg.trd

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

$(OFILES) $(TCOFILES_IO) $(TCOFILES_RD): $(INCLUDE) $(HFILEDIR)/global.h

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)
