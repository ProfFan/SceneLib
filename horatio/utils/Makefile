SHELL = /bin/sh
HOR_EXECS = halve_size double_size halve_width ramp circle unlace \
	    pixel_convert invert extract convert_image rotate squares outline
DOC_EXECS = extract_func_headers function_index
all: $(HOR_EXECS) $(DOC_EXECS)

ARCH = $(shell uname -m)
OS = $(shell uname -s)

LINK_FLAGS = -g -L../lib-$(ARCH)
INCDIR = ../include
HORLIBS = -lhoratio
IMAGELIBS = -liff -lmit
CC = gcc
CFLAGS = -I$(INCDIR) -D$(OS) -g -O -Wall

UTILS = double_size halve_size halve_width ramp circle squares unlace pixel_convert \
	invert extract convert_image rotate mult_width mult_height outline \
	extract_func_headers function_index

halve_size: halve_size.o
	$(CC) $(LINK_FLAGS) -o halve_size halve_size.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

double_size: double_size.o
	$(CC) $(LINK_FLAGS) -o double_size double_size.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

halve_width: halve_width.o
	$(CC) $(LINK_FLAGS) -o halve_width halve_width.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

ramp: ramp.o
	$(CC) $(LINK_FLAGS) -o ramp ramp.o $(HORLIBS) $(IMAGELIBS) -lm

circle: circle.o
	$(CC) $(LINK_FLAGS) -o circle circle.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

squares: squares.o
	$(CC) $(LINK_FLAGS) -o squares squares.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

unlace: unlace.o
	$(CC) $(LINK_FLAGS) -o unlace unlace.o $(HORLIBS) $(IMAGELIBS) -lm

pixel_convert: pixel_convert.o
	$(CC) $(LINK_FLAGS) -o pixel_convert pixel_convert.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

invert: invert.o
	$(CC) $(LINK_FLAGS) -o invert invert.o $(HORLIBS) $(IMAGELIBS) -lm

extract: extract.o
	$(CC) $(LINK_FLAGS) -o extract extract.o $(HORLIBS) $(IMAGELIBS) -lm

convert_image: convert_image.o
	$(CC) $(LINK_FLAGS) -o convert_image convert_image.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

rotate: rotate.o
	$(CC) $(LINK_FLAGS) -o rotate rotate.o $(HORLIBS) $(IMAGELIBS) -lm

mult_width: mult_width.o
	$(CC) $(LINK_FLAGS) -o mult_width mult_width.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

mult_height: mult_height.o
	$(CC) $(LINK_FLAGS) -o mult_height mult_height.o \
	      $(HORLIBS) $(IMAGELIBS) -lm

outline: outline.o
	$(CC) $(LINK_FLAGS) -o outline outline.o $(HORLIBS) $(IMAGELIBS) -lm

# utility to extract function headers from source code
extract_func_headers: extract_func_headers.c
	$(CC) -g -O -o extract_func_headers extract_func_headers.c

# utility to create index of function name locations in source files
function_index: function_index.c
	$(CC) -g -O -o function_index function_index.c

#---clean directory rule---
clean:
	rm -f *.o *~

realclean:
	rm -f *.o *~ $(UTILS)
