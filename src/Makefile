-include Mk/local.Mk
-include /etc/robot/overwrite.mk
SUBDIRS+=common
SUBDIRS+=camera
#SUBDIRS+=server
SUBDIRS+=distance/fast
SUBDIRS+=distance
SUBDIRS+=main

OBJS=$(wildcard ../obj/*.o)

COPY_ETC_CMD=../etc/copy.sh
COPY_ETC_SRC=../etc
COPY_ETC_DST=../bin

ifeq ($(RUNONPC),true)
LXXLIBS+=-lv4l1
endif

#default:
#	echo "type: 'make cameraminoru'    for work with Minoru stereo webcam" 
#	echo "      'make cameraweb'       for work with other webcam"
#	echo "      'make cameradummy'     for wokr with 'dummy' camera (it uses bmp pictures instead of camera)"
#	echo ""
#	echo "then  'make all'             for compiling the code"

camserver: all obj
	$(CXX) $(CXXDEFINE) -o ../bin/$@  $(OBJS)  $(CXXFLAGS) $(LDFLAGS) $(LXXLIBS)

cameraminoru:
	rm -rf camera
	ln -s camera.minoru camera

cameraweb: 
	rm -rf camera
	ln -s camera.web camera

cameradummy: 
	rm -rf camera
	ln -s camera.dummy 

all:
	echo $(OPSYS)
	mkdir -p ../bin
	@for i in $(SUBDIRS) ;\
	do \
	echo "making" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i all; \
	done

obj: all
	echo "Copy objs"
	mkdir -p ../obj
	@for i in $(SUBDIRS) ;\
	do \
	echo "coping all in $(CURRENT_DIR)/$$i..."; \
	cp $$i/*.o ../obj; \
	done


etc: ../bin/.etc

	#cp -r ../etc/$$i ../bin; \
	#
../bin/.etc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

cleanetc:
	rm -f ../bin/.etc	

	#cp -r ../etc/$$i ../bin; 
forceetc:
	echo "Copy dirs from etc"
	$(COPY_ETC_CMD) $(COPY_ETC_SRC) $(COPY_ETC_DST)
	touch ../bin/.etc

clean:
	echo $(OPSYS)
	@for i in $(SUBDIRS) ;\
	do \
	echo "cleaning" all "in $(CURRENT_DIR)/$$i..."; \
	$(MAKE) -C $$i clean; \
	done
	echo "cleaning all objs"
	rm -f ../obj/*.o
	echo "cleaning binaries"
	rm -f ../bin/*
