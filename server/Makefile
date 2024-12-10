INCLUDE_DIRS = 
LIB_DIRS = 
CC= $(CROSS_COMPILE)gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt -ljpeg

HFILES= 
CFILES= capture.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	capture

clean:
	-rm -f *.o
	-rm -f capture

distclean:
	-rm -f *.o *.d

capture: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<