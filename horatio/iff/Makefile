# Makefile for IFF library
# Simon Turner (simon@robots.oxford.ac.uk) 10-10-88
# changed for transputer compilation Phil McLauchlan 13-2-92

SHELL = /bin/sh
HEADERS = iff.h

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

# Sun stuff
CC = gcc
SOURCES = frd_iff.c fwr_iff.c make_iff.c read_hdr.c writ_hdr.c make_hdr.c
OBJECTS = frd_iff.o fwr_iff.o make_iff.o read_hdr.o writ_hdr.o make_hdr.o

LIB = ../lib-$(ARCH)/libiff.a
$(LIB): $(OBJECTS)
	ar r $(LIB) $(OBJECTS)
	ranlib $(LIB)

$(OBJECTS): $(HEADERS)

.c.o:
	$(CC) -c -O $<

# transputer stuff
.SUFFIXES: .tco
TLIB = ../lib-T800/iff.lib
TSOURCES = read_hdr.c   writ_hdr.c   make_iff.c
TOBJECTS = read_hdr.tco writ_hdr.tco make_iff.tco

T800: $(TLIB)

$(TLIB): $(TOBJECTS)
	ilibr $(TOBJECTS) -o $(TLIB)
	rm -f $(TOBJECTS)

$(TOBJECTS): $(HEADERS)

.c.tco:
	icc $* -t800 -DTRANSPUTER

clean:
	rm -f *.tco *.o *~
