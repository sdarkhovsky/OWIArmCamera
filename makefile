PROG ?= OWIArmCamera
SRCS=OWIArmCamera.cpp joint_commands.cpp client.cpp
OBJS=$(subst .cpp,.o,$(SRCS))
CPPFLAGS=-c -O0 -g3 -Wall -fmessage-length=0 -std=c++11
LDFLAGS=-O0 -g3 -Wall -fmessage-length=0 -std=c++11

all: $(PROG)

.cpp.o: 
	g++ $(CPPFLAGS) -o $@ $<

$(PROG): $(OBJS)
	g++ $(LDFLAGS) -o $@ $(OBJS)

clean:
	$(RM) $(OBJS) $(PROG)


