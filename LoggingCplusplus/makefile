PROG ?= example
SRCS=example.cpp
OBJS=$(subst .cpp,.o,$(SRCS))
CPPFLAGS=-DLOGGING_LEVEL_1 -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11
LDFLAGS=-O0 -g3 -Wall -fmessage-length=0 -std=c++11

all: $(PROG)

.cpp.o: 
	g++ $(CPPFLAGS) -o $@ $<

$(PROG): $(OBJS)
	g++ $(LDFLAGS) -o $@ $(OBJS) 

clean:
	$(RM) $(OBJS) $(PROG)


