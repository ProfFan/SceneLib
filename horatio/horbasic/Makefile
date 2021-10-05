SHELL = /bin/sh

#---UNIX stuff---

#---Architecture dependent stuff---
ARCH = $(shell uname -m)
OS = $(shell uname -s)

INCDIR = /usr/lav/horatio/include
HORLIBS = -lhoratio
IMAGELIBS = -liff -lmit
XLIBS = -lXow -lXaw -lXmu -lXt -lXext -lX11
CC = gcc
CFLAGS = -I$(INCDIR) -g -Wall

horbasic: horbasic.o
	gcc -g -o horbasic horbasic.o -L/usr/lav/horatio/lib-sun4 $(HORLIBS) $(IMAGELIBS) $(XLIBS) -lm
