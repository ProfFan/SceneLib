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
EXECUTABLES = scenesim sceneanalyse #scenerob

#---Robot Libraries---
# Change MODULEBASEDIR if necessary
MODULEBASEDIR = /data/cas1/Nomad/Module
MODULEINCDIR = $(MODULEBASEDIR)/include
MODULELIBS = -L$(MODULEBASEDIR)/lib -lmodule

#---View Simulator Libraries---
# 
SIMINCDIR = /data/cas1/Nomad/Simulator/client
SIMLIBS = /data/cas1/Nomad/Simulator/client/VisionSimulator.o

#---CORBA Libraries
CORBALIBS = -lOB -lOBTest -lOBX11 -lCosNaming -lCosProperty -lCosEvent
PTLIBS = -lJTC -lpthread

#---Uncomment -D_INSPECT_ flag for Scene Inspector and Kalman Form
#---Uncomment -D_THREED_ flag for 3D mode
CFLAGS = -I$(XFORMSINCDIR) -I$(HORINCDIR) -I$(SCENEINCDIR) -g -Wall -D$(OS) -D$(ARCH) -DXOW_OLDNAMES -D_INSPECT_ -fpermissive #-D_THREED_
ROBCFLAGS = $(CFLAGS) -I$(MODULEINCDIR) -I$(SIMINCDIR) -D_ROBOT_
SIMCFLAGS = $(CFLAGS) -U_ROBOT_

LINKFLAGS = $(SCENELIB) $(SCENEHEADLIB) $(SCENEXFORMSBITSLIB) $(HORLIBS) $(XFORMSLIBS) $(XLIBS) $(IMAGELIBS) $(MATHSLIBS)
ROBLINKFLAGS = $(MODULELIBS) $(CORBALIBS) $(PTLIBS) $(SCENEIMPROCLIB) $(LINKFLAGS)
SIMLINKFLAGS = $(LINKFLAGS)


# Simulation / Robot / Analyse independent source files: some C++,XForms ones C
ALLCOMMONCSOURCEFILES = 
ALLCOMMONCCSOURCEFILES = models_nomad.cc models_rollpitch.cc

# Simulation / Robot independent source files: some C++, XForms ones C
SIMROBCOMMONCSOURCEFILES = formcom.c
SIMROBCOMMONCCSOURCEFILES = 

# Robot-specific source files
ROBONLYCSOURCEFILES = formrob.c formrobmode.c
ROBONLYCCSOURCEFILES = grab.cc robot.cc vsimulator.cc

# Simulation-specific source files
SIMONLYCSOURCEFILES = formsim.c
SIMONLYCCSOURCEFILES =

# Analyse-specific source files
ANALYSECSOURCEFILES = formanalyse.c
ANALYSECCSOURCEFILES = analyse.cc analysemain.cc

# Source files which compile to both robot and simulation versions
ROBSIMCCSOURCEFILES = main.cc control.cc

# Makefile.master contains general rules and commands
include $(SCENEBASEDIR)/make/Makefile.app.master
