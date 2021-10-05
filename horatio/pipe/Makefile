# Copyright 1993 Philip F. McLauchlan (pm@robots.oxford.ac.uk)
#                Robotics Research Group, Oxford University.
SHELL = /bin/sh
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio
INCLUDE = $(HFILEDIR)/pipe.h
HFILES = def.h head.h proc.h print.h tail.h reinit.h

#---transputer stuff---

#---compilation options---
TCOFLAGS = -t800 -j $(INCDIR) -DHOR_TRANSPUTER
.SUFFIXES: .tco

#---files---
TRANS_LIB = ../lib-T800/pipe.lib
TCOFILES = head.tco proc.tco print.tco tail.tco reinit.tco

#---make rules---
T800: $(TRANS_LIB)

$(TRANS_LIB): $(TCOFILES)
	ilibr $(TCOFILES) -o $(TRANS_LIB)
	rm -f $(TCOFILES)

.c.tco:
	icc $* $(TCOFLAGS)

$(TCOFILES): $(INCLUDE) $(HFILEDIR)/global.h

#---library include file rule---
$(INCLUDE): $(HFILES) Makefile
	cat $(HFILES) > $(INCLUDE)
