PROG ?= OWIFeatCalcPrediction
SRCS=OWIFeatCalcPrediction.cpp ais_history.cpp ais_prediction.cpp ais_event.cpp ais.cpp ../LoggingCplusplus/log.cpp ../joint_commands.cpp
OBJS=$(subst .cpp,.o,$(SRCS))
CPPFLAGS=-DLOGGING_LEVEL_1 -I/usr/local/include/opencv2 -I../LoggingCplusplus -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11
LDFLAGS=-L/usr/local/lib -O0 -g3 -Wall -fmessage-length=0 -std=c++11

all: $(PROG)

.cpp.o: 
	g++ $(CPPFLAGS) -o $@ $<

$(PROG): $(OBJS)
	g++ $(LDFLAGS) -o $@ $(OBJS) -lopencv_core -lopencv_objdetect -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_imgcodecs -lopencv_videoio 

clean:
	$(RM) $(OBJS) $(PROG)


