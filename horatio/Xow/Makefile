#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

PROGS = Canvas.c PanelSlider.c PanelText.c Slider.c ToggleChoice.c
HFILES = ${PROGS:%.c=%.h} ${PROGS:%.c=%P.h} XowAll.h oldnames.h
OBJS=${PROGS:%.c=%.o}
#CC=gcc -Wall
LATEX=/usr/local/bin/latex
DVI2PS=/usr/local/bin/dvi2ps
PSPRINTER=lpr -Plw -h
INCLUDE_DIR=../include
LIB = ../lib-$(ARCH)/libXow.a
INC = include

.KEEP_STATE:

CFLAGS=-O -I.

all: $(INC) $(LIB)

$(LIB):	$(OBJS)
	ar r $(LIB) $(OBJS)
	ranlib $@

doc:   Xow.tex
	$(LATEX) Xow

print:	doc
	$(DVI2PS) Xow.dvi | $(PSPRINTER)	

realclean:
	@rm -f  *.o *.a *"^" *~  *.log *.dvi *.aux *.toc core
	@sccs clean

clean:
	@rm -f  *.o *.a *"^" *~  *.log *.dvi *.aux *.toc core

dist: $(PROGS)
	make clean
	make CC=/bin/true
	cd .. ; tar cFf /tmp/xow.tar Xow

get: $(PROGS)

.c.o:
	$(CC) -I$(INCLUDE_DIR) -DXOW_OLDNAMES -c $<

include: $(INCLUDE_DIR)/X11/Xow $(HFILES)
	mkdir -p $(INCLUDE_DIR)/X11/Xow
	cp *.h $(INCLUDE_DIR)/X11/Xow
