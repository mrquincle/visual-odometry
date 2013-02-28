OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

-include ../Mk/local.Mk
-include /etc/robot/overwrite.mk

CXXINCLUDE+=-I./ -I../camera 

all: $(OBJS)

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS)
