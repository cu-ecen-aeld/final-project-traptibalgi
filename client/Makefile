# Starter code from https://github.com/siewertsmooc/RTES-ECEE-5623/blob/main/simple-capture/Makefile
# Modified by Trapti Balgi

INCLUDE_DIRS = 
LIB_DIRS = 
CC= $(CROSS_COMPILE)gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt

HFILES= 
CFILES= client.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	client

clean:
	-rm -f *.o *.d *.ppm *.pgm
	-rm -f client

distclean:
	-rm -f *.o *.d

capture: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<