OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))
OBJSC=$(patsubst %.c,%.o,$(wildcard *.c))
OBJS+=$(OBJSC)

-include /etc/robot/overwrite.mk

-include ../../Mk/local.Mk
CXXINCLUDE+=-I./ -I..  -I../../common -I../../map -I../../server -I/opt/ 
CXXFLAGS+=-Wall

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CC)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)

