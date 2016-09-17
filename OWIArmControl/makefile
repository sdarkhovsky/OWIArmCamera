PROG ?= OWIArmControl
SRCS=main.cpp owi_interface.cpp
OBJS=$(subst .cpp,.o,$(SRCS))
CPPFLAGS=-I/usr/include -O0 -g3 -Wall -c -fmessage-length=0
LDFLAGS=-L/usr/lib/x86_64-linux-gnu -O0 -g3 -Wall -fmessage-length=0

all: $(PROG)

.cpp.o: 
	g++ $(CPPFLAGS) -o $@ $<

# -lusb-1.0 had to be after -o otherwise undefined references to the libusb functions
$(PROG): $(OBJS)
	g++ $(LDFLAGS) -o $@ $(OBJS) -lusb-1.0

clean:
	$(RM) $(OBJS) $(PROG)


