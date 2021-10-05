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
EXECUTABLES = scenesim # scenerob

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
#CORBALIBS = -L/usr/local/lib -lOB -lOBTest -lOBX11 -lCosNaming -lCosProperty
#CORBALIBS = -L/data/cas1/Nomad/Simulator/lib -lOB -lOBTest -lOBX11 -lCosNaming -lCosProperty
CORBALIBS = -lOB -lOBTest -lOBX11 -lCosNaming -lCosProperty -lCosEvent
PTLIBS = -lJTC -lpthread

#---SHM Communications Libraries---
SHMINCDIR = /data/cas1/Nomad/Slave/include
SHMLIBS = -L/data/cas1/Nomad/Slave/lib -ljijo2agent

#---Single Directory--- 
#(Where to look for files which are shared with the single robot application)
SINGLEDIR = ../ETLRobot
SINGLECOMMONOBJFILES = models_nomad.o
SINGLEROBONLYOBJFILES = robot.o mainrob.o formrob.o formrobmode.o grab.o vsimulator.o
SINGLESIMONLYOBJFILES = mainsim.o formsim.o
SINGLECOMMONLINKFLAG = $(patsubst %,$(SINGLEDIR)/%,$(SINGLECOMMONOBJFILES))
SINGLEROBLINKFLAG = $(patsubst %,$(SINGLEDIR)/%,$(SINGLEROBONLYOBJFILES)) $(SINGLECOMMONLINKFLAG)
SINGLESIMLINKFLAG = $(patsubst %,$(SINGLEDIR)/%,$(SINGLESIMONLYOBJFILES)) $(SINGLECOMMONLINKFLAG)

# ETLDouble depends on object files from ../ETLRobot so make that first
first:
	cd $(SINGLEDIR)	&& make
	make $(EXECUTABLES)

#---Uncomment -D_INSPECT_ flag for Scene Inspector and Kalman Form
CFLAGS = -I$(HORINCDIR) -I$(SCENEINCDIR) -I$(SINGLEDIR) -g -Wall -D$(OS) -D$(ARCH) -DXOW_OLDNAMES -D_INSPECT_ -fpermissive
ROBCFLAGS = $(CFLAGS) -I$(MODULEINCDIR) -I$(SHMINCDIR) -I$(SIMINCDIR) -D_ROBOT_
SIMCFLAGS = $(CFLAGS) -U_ROBOT_

LINKFLAGS = $(SCENELIB) $(SCENEHEADLIB) $(SCENEXFORMSBITSLIB) $(HORLIBS) $(XFORMSLIBS) $(XLIBS) $(IMAGELIBS) $(MATHSLIBS)
ROBLINKFLAGS = $(SINGLEROBLINKFLAG) $(MODULELIBS) $(CORBALIBS) $(PTLIBS) $(SCENEIMPROCLIB) $(SHMLIBS) $(LINKFLAGS) -static
SIMLINKFLAGS = $(SINGLESIMLINKFLAG) $(LINKFLAGS)


# Simulation / Robot independent source files: some C++, XForms ones C
SIMROBCOMMONCSOURCEFILES = formcom.c
SIMROBCOMMONCCSOURCEFILES = models_interrobot.cc

# Robot-specific source files
ROBONLYCSOURCEFILES = 
ROBONLYCCSOURCEFILES = double_robot.cc

# Simulation-specific source files
SIMONLYCSOURCEFILES =
SIMONLYCCSOURCEFILES =

# Source files which compile to both robot and simulation versions
ROBSIMCCSOURCEFILES = control.cc

# Makefile.master contains general rules and commands
include $(SCENEBASEDIR)/make/Makefile.app.master
