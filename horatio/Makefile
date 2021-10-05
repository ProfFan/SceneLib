# Copyright (C) 1992 RTCG Consortium

SHELL = /bin/sh

# installation directories for UNIX
#LIBDIR = /usr/local/lib
#LIBDIR = /usr/local/bin/LIB  # for Oxford RRG
#INCDIR = /usr/local/include
#BINDIR = /usr/local/bin
#MANDIR = /usr/local/man
# DOCDIR = /usr/local/doc

CURRDIR = $(shell pwd)
ROOTDIR = $(CURRDIR)/../rootfs
LIBDIR = $(ROOTDIR)/lib
INCDIR = $(ROOTDIR)/include
BINDIR = $(ROOTDIR)/bin
MANDIR = $(ROOTDIR)/man
DOCDIR = $(ROOTDIR)/doc

TARFILE = horatio.tar.gz
ZIPFILE = horatio.zip

# installation directories for transputer
TLIBDIR =

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

#---end of configuration options---

#---Sun stuff---
SRCDIRS = Xow mit iff global list math image graphics improc process tool horatio utils doc

all:
	for f in $(SRCDIRS); do (cd $$f; $(MAKE);) done

elf:
	/bin/rm lib-i586
	ln -s lib-i486-linuxelf lib-i586
	for f in $(SRCDIRS); do (cd $$f; $(MAKE);) done

a.out:
	/bin/rm lib-i586
	ln -s lib-i486-linuxaout lib-i586
	for f in $(SRCDIRS); do (cd $$f; $(MAKE);) done

tar:
	/bin/rm $(TARFILE) $(ZIPFILE); \
	tar zcvf $(TARFILE) . --exclude $(TARFILE) --exclude $(ZIPFILE)

zip:
	/bin/rm -f $(TARFILE) $(ZIPFILE); \
	zip $(ZIPFILE) -v -r ./* -x $(TARFILE) $(ZIPFILE)

clean:
	for f in $(SRCDIRS); do (cd $$f; $(MAKE) clean;) done

realclean:
	for f in lib-*; do (cd $$f; /bin/rm -f *.a); done; \
	for f in $(SRCDIRS); do (cd $$f; $(MAKE) clean;) done; \
	(cd horatio; make realclean;); \
	(cd utils; make realclean;)

install: install-libs install-inc install-bin install-man install-doc

install-libs:
	(cd lib-$(ARCH); cp -p libhoratio.a $(LIBDIR)/)

install-inc:
	mkdir -p $(INCDIR)/horatio
	cp -p include/horatio/* $(INCDIR)/horatio

install-bin:
	cp -p horatio/horatio $(BINDIR)

install-man:
	mkdir -p $(MANDIR)/man1
	mkdir -p $(MANDIR)/man3
	cp -p doc/horatio.1 $(MANDIR)/man1/horatio.1
	cp -p doc/horatio.3 $(MANDIR)/man3/horatio.3

install-doc:
	mkdir -p $(DOCDIR)
	cp -p doc/Report.ps $(DOCDIR)/horatio.ps

IMAGEDIRS = mit iff

image-libs:
	for f in $(IMAGEDIRS); do (cd $$f; $(MAKE);) done

install-image-libs:
	for f in $(IMAGEDIRS); do (cd $$f; cp -p lib$$f.a $(LIBDIR);) done

#---transputer stuff---
TSRCDIRS = global pipe list math image mit iff

T800:
	for f in $(TSRCDIRS); do (cd $$f; $(MAKE) T800;) done

install-T800:
	cp -p lib-T800/*.lib $(TLIBDIR)
