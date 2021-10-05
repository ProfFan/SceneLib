# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
NAME = tool
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/$(NAME).h

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

ifeq ($(ARCH),sun4u)
	ULTRA_FLAGS = -I/usr/openwin/include
endif

#---compilation options---
CC = gcc
CFLAGS = -I$(INCDIR) $(ULTRA_FLAGS) -g -O -Wall -DXOW_OLDNAMES
ARFLAGS = r

#---files---
LIB = ../lib-$(ARCH)/libhoratio.a
OFILES = ca_tool.o lf_tool.o ls_tool.o blm_tool.o pc_tool.o sc_tool.o \
	 cow_tool.o bcm_tool.o dcm_tool.o fl_tool.o is_tool.o cl_tool.o \
	 popup.o memory.o graph.o threed.o toggle.o region.o velocity.o \
	 read_prm.o test_prm.o
HFILES = def.h \
	 ca_tool.h lf_tool.h ls_tool.h blm_tool.h pc_tool.h sc_tool.h \
	 cow_tool.h bcm_tool.h dcm_tool.h fl_tool.h is_tool.h cl_tool.h \
	 popup.h memory.h graph.h threed.h toggle.h region.h velocity.h \
	 read_prm.h test_prm.h
INCLUDE = $(HFILEDIR)/$(NAME).h

#---make rules---
.c.o:
	$(CC) $(CFLAGS) -c $<

$(LIB): $(OFILES)
	ar $(ARFLAGS) $(LIB) $(OFILES)
	ranlib $@

$(OFILES): $(INCLUDE) $(HFILEDIR)/global.h

region.o graph.o memory.o threed.o toggle.o: $(HFILEDIR)/list.h

region.o graph.o threed.o: $(HFILEDIR)/image.h $(HFILEDIR)/graphics.h

region.o threed.o: $(HFILEDIR)/math.h

#---library include file rules---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)

#---clean directory rule---
clean:
	rm -f *.o *~
