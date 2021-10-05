#
# Compiles to two executables: scenesim (simulation) and scenerob (robot).
#
# control.cc compiles to controlrob.o and controlsim.o depending on 
# whether _ROBOT_ is defined in CFLAGS.
# Same with main.cc which compiles to mainrob.o and mainsim.o
#

SCENEBASEDIR = ../../SceneLib

# Makefile.options contains location of libraries, etc.
include $(SCENEBASEDIR)/make/Makefile.app.options

# Target executables
EXECUTABLES = scenesim scenerob

#---Uncomment -D_INSPECT_ flag for Scene Inspector and Kalman Form
CFLAGS = -I$(XFORMSINCDIR) -I$(HORINCDIR) -I$(SCENEINCDIR) -g -Wall -D$(OS) -D$(ARCH) -DXOW_OLDNAMES -fpermissive #-D_INSPECT_
ROBCFLAGS = $(CFLAGS) -D_ROBOT_
SIMCFLAGS = $(CFLAGS) -U_ROBOT_

LINKFLAGS = $(SCENELIB) $(SCENEHEADLIB) $(SCENEXFORMSBITSLIB) $(HORLIBS) $(XFORMSLIBS) $(XLIBS) $(IMAGELIBS) $(MATHSLIBS)
ROBLINKFLAGS = $(SCENEIMPROCLIB) $(LINKFLAGS)
SIMLINKFLAGS = $(LINKFLAGS)


# Simulation / Robot independent source files: some C++, XForms ones C
SIMROBCOMMONCSOURCEFILES = formcom.c
SIMROBCOMMONCCSOURCEFILES = models_impulse.cc models_camera.cc

# Robot-specific source files
ROBONLYCSOURCEFILES = formrob.c
ROBONLYCCSOURCEFILES = robot.cc

# Simulation-specific source files
SIMONLYCSOURCEFILES = formsim.c
SIMONLYCCSOURCEFILES =

# Source files which compile to both robot and simulation versions
ROBSIMCCSOURCEFILES = main.cc control.cc

# Makefile.master contains general rules and commands
include $(SCENEBASEDIR)/make/Makefile.app.master
