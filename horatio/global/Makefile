# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/global.h
HFILES = def.h error.h types.h ralloc.h talloc.h text_io.h bit_arr.h \
	       inmos.h ring_buf.h number.h compress.h

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
OFILES = error.o ralloc.o talloc.o text_io.o bit_arr.o number.o compress.o

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
TRANS_LIB_STDIO = ../lib-T800/glob_io.lib
TRANS_LIB_STDRD = ../lib-T800/glob_rd.lib
TCOFILES_IO = error.tio ralloc.tio talloc.tio bit_arr.tio \
	      inmos.tio ring_buf.tio number.tio text_io.tio
TCOFILES_RD = error.trd ralloc.trd talloc.trd bit_arr.trd \
	      inmos.trd ring_buf.trd number.trd text_io.trd

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

$(OFILES) $(TCOFILES_IO) $(TCOFILES_RD): $(INCLUDE)

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)
