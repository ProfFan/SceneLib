#
# A 1D testbed simulator for the filter code
#

SCENEBASEDIR = ../../SceneLib

include $(SCENEBASEDIR)/make/Makefile.app.options

# Target executables
EXECUTABLES = scenesim #scenerob

CFLAGS = -I$(HORINCDIR) -I$(SCENEINCDIR) -g -Wall -D$(OS) -D$(ARCH) -DXOW_OLDNAMES -fpermissive
SIMCFLAGS = $(CFLAGS) -U_ROBOT_

LINKFLAGS = $(SCENELIB) $(HORLIBS) $(SCENEXFORMSBITSLIB) $(XFORMSLIBS) $(XLIBS) $(IMAGELIBS) $(MATHSLIBS)
SIMLINKFLAGS = $(LINKFLAGS)


# Simulation / Robot independent source files: some C++, XForms ones C
SIMROBCOMMONCCSOURCEFILES = models_oned.cc

# Simulation-specific source files
SIMONLYCSOURCEFILES = formoned.c
SIMONLYCCSOURCEFILES = control.cc main.cc

# Makefile.master contains general rules and commands
include $(SCENEBASEDIR)/make/Makefile.app.master



OBJFILES = formoned.o control.o main.o models_oned.o
