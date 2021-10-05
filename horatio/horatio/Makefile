SHELL = /bin/sh

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

HORLIBS = -lhoratio
IMAGELIBS = -liff -lmit
HORLIB = ../lib-$(ARCH)
LIBDIR = -L/lib -L/usr/X11R6/lib -L$(HORLIB)
LINK_FLAGS = $(LIBDIR)
INCDIR = ../include
HFILEDIR = $(INCDIR)/horatio

ifeq ($(ARCH),sun4c)
XLIBS    = -lXow -lXaw -lXt -lXmu -lXext -lX11
else
XLIBS    = -lXow -lXaw -lXt -lXmu -lXext -lX11 -lSM -lICE
endif


# extra stuff
EXTRA_OFILES = select.o canvas.o buttons.o ops_prms.o commands.o
EXTRA_HFILES = select.h canvas.h buttons.h ops_prms.h commands.h

OFILES = horatio.o $(EXTRA_OFILES)
CC = gcc
CFLAGS = -I$(INCDIR) -g -Wall -D$(OS) -DXOW_OLDNAMES -Bdynamic

horatio: $(OFILES) Makefile $(HORLIB)/* #../lib-i586/libhoratio.a ../lib-i586/libmit.a
	$(CC) $(LINK_FLAGS) -o horatio $(OFILES) \
				     $(HORLIBS) $(IMAGELIBS) $(XLIBS) -lm

horatio.o $(EXTRA_OFILES): $(EXTRA_HFILES)

$(OFILES): $(HFILEDIR)/tool.h     $(HFILEDIR)/process.h       \
	   $(HFILEDIR)/improc.h   $(HFILEDIR)/graphics.h      \
	   $(HFILEDIR)/image.h    $(HFILEDIR)/list.h          \
	   $(HFILEDIR)/math.h     $(HFILEDIR)/global.h

clean:
	rm -f $(OFILES) *~

realclean: 
	rm -f $(OFILES) *~ horatio
