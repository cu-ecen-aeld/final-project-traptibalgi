# Starter code from https://github.com/siewertsmooc/RTES-ECEE-5623/blob/main/simple-capture/Makefile
# Modified by Trapti Balgi

INCLUDE_DIRS = -I/usr/include/opencv4
LIB_DIRS = 
CXX = $(CROSS_COMPILE)g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= `pkg-config --cflags --libs opencv4`
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lpthread -L/usr/include

HFILES= 
CPPFILES= trafficlight.cpp

SRCS= ${HFILES} ${CPPFILES}
OBJS= ${CPPFILES:.cpp=.o}

all:	trafficlight

clean:
	-rm -f *.o *.d *.ppm *.pgm *.jpg
	-rm -f trafficlight

distclean:
	-rm -f *.o *.d

trafficlight: ${OBJS}
	#$(CXX) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)
	g++ -O0 -g -I/usr/include/opencv4 -o trafficlight trafficlight.cpp -pthread `pkg-config --cflags --libs opencv4`
depend:

.cpp.o:
	$(CXX) $(CFLAGS) -c $<



