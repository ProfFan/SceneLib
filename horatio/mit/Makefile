# Makefile for mit library. Note that the read_header.c and write_header.c
# files actually contain routines calles read_mit_header() and
# write_mit_header() - the names of the source codes are chosen to avoid
# ar complaining (ar only lets you have filenames up to 16 chars).
# Simon Turner (simon@robots.oxford.ac.uk) 10-10-88
# modified for transputer compilation P.F.M 13-2.92

SHELL = /bin/sh
HEADERS = mit.h

# UNIX stuff

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

CC = gcc
SOURCES = frd_mit.c fwr_mit.c read_hdr.c writ_hdr.c
OBJECTS = frd_mit.o fwr_mit.o read_hdr.o writ_hdr.o

LIB = ../lib-$(ARCH)/libmit.a
$(LIB): $(OBJECTS)
	ar r $(LIB) $(OBJECTS)
	ranlib $(LIB)

$(OBJECTS): $(HEADERS)

.c.o:
	$(CC) -c -O $<

# transputer stuff
.SUFFIXES: .tco
TLIB = ../lib-T800/mit.lib
TSOURCES = read_hdr.c   writ_hdr.c
TOBJECTS = read_hdr.tco writ_hdr.tco

T800: $(TLIB)

$(TLIB): $(TOBJECTS)
	ilibr $(TOBJECTS) -o $(TLIB)
	rm -f $(TOBJECTS)

$(TOBJECTS): $(HEADERS)

.c.tco:
	icc $* -t800 -DTRANSPUTER

clean:
	rm -f $(OBJECTS) *~
	