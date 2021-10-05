#
# Master Makefile for applications
#

APPDIRECTORIES = OneD ETLRobot ETLDouble Camera

all: 
	$(foreach dir,$(APPDIRECTORIES),cd $(dir) && make $@ && cd ..;)

clean: 
	$(foreach dir,$(APPDIRECTORIES),cd $(dir) && make $@ && cd ..;)
	rm -f *~

distrib: 
	make all
	$(foreach dir,$(APPDIRECTORIES),cd $(dir) && make $@ && cd ..;)
	rm -f *~

web:
	cp sceneapp.html /homes/38/ajd/WWW/Scene
	cp README /homes/38/ajd/WWW/Scene/README.SceneApp
