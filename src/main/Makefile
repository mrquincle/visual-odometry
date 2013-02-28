OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

-include ../Mk/local.Mk
-include /etc/robot/overwrite.mk

CXXINCLUDE+=-I./ -I../common -I../camera -I../server -I../distance

all: $(OBJS) 

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CC)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS) *.moc $(UI_HEAD) $(UI_CPP)
