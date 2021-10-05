#
# Library Makefile
# Library names specified in make/Makefile.lib.options
# Sources for each library must be contained in directory of the same name
# First does make depend, which makes depend.mk files and copies headers
# to include/
# Makes libraries in lib/
#
# Also makes LaTeX documentation in Docs/ directory
#

include make/Makefile.lib.options

all: .all_depend_done
	$(foreach library,$(LIBRARIES),cd $(library); make; cd ..;)
	cd Docs; make

# For distribution: makes postscript Docs but cleans out everything else
distrib: clean
	cd Docs; make distrib; cd ..
	echo Run make web for SceneLib and SceneApp to publish docs
	echo Have you compiled with static XForms libraries?
	echo Have you compiled both versions of SceneApp/ETLRobot?
	echo Have you done the cvs tag -c ver-*-**  ?

web:
	cp Docs/models.ps /homes/38/ajd/WWW/Scene
	cp -a Scene/autodoc /homes/38/ajd/WWW/Scene
	cp README /homes/38/ajd/WWW/Scene/README.SceneLib

clean:
# For each library, go through list of directories and clean them out
	$(foreach library,$(LIBRARIES),cd $(library); make clean; cd ..;)
	cd Docs; make clean; cd ..
	rm -f include/*.h lib/*.a *~
	rm -f make/*~
	rm -f .*done

# Make depend before making object files
depend: .all_depend_done

.all_depend_done:
	$(foreach library,$(LIBRARIES),cd $(library); make depend; cd ..;)
	touch .all_depend_done