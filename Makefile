CC=gcc
DEPS=magVector.h filter.h sunVector.h util.h wmm.h controller.h
OBJS=filter.o sunVector.o magVector.o wmm.o WMM_COF.o util.o controller.o
CFLAGS=-I. -lm -g


%.o: %.c $(DEPS)
	$(CC) -Wall -fPIC -c -o $@ $< $(CFLAGS)


filter: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS)

libfilter: $(OBJS)
	$(CC) -shared -o $@.so $^


.PHONY: clean

clean:
	rm -f *.o *.so